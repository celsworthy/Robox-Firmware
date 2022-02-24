#include <board.h>
#include <pio/pio.h>
#include "component_hsmci.h"
#include "sd_mmc_protocol.h"
//#include "dmac.h"

void sd_debug(unsigned char n);


/**
 * \brief Reset the HSMCI interface
 */
static void hsmci_reset(void) {
  unsigned long mr = HSMCI->HSMCI_MR;
  unsigned long dtor = HSMCI->HSMCI_DTOR;
  unsigned long sdcr = HSMCI->HSMCI_SDCR;
  unsigned long cstor = HSMCI->HSMCI_CSTOR;
  unsigned long cfg = HSMCI->HSMCI_CFG;
  HSMCI->HSMCI_CR = HSMCI_CR_SWRST;
  HSMCI->HSMCI_MR = mr;
  HSMCI->HSMCI_DTOR = dtor;
  HSMCI->HSMCI_SDCR = sdcr;
  HSMCI->HSMCI_CSTOR = cstor;
  HSMCI->HSMCI_CFG = cfg;
  HSMCI->HSMCI_DMA = 0;
  // Enable the HSMCI
  HSMCI->HSMCI_CR = HSMCI_CR_PWSEN | HSMCI_CR_MCIEN;
}

/**
 * \brief Set speed of the HSMCI clock.
 *
 * \param speed    HSMCI clock speed in Hz.
 * \param mck      MCK clock speed in Hz.
 */
static void hsmci_set_speed(unsigned long speed, unsigned long mck) {
  unsigned long clkdiv;
  unsigned long rest;

  // Speed = MCK clock / (2 * (CLKDIV + 1))
  if (speed > 0) {
    clkdiv = mck / (2 * speed);
    rest = mck % (2 * speed);
    if (rest > 0) {
      // Ensure that the card speed not be higher than expected.
      clkdiv++;
    }
    if (clkdiv > 0) {
      clkdiv -= 1;
    }
  } else {
    clkdiv = 0;
  }
  HSMCI->HSMCI_MR &= ~HSMCI_MR_CLKDIV_Msk;
  HSMCI->HSMCI_MR |= HSMCI_MR_CLKDIV(clkdiv);
}

/** \brief Wait the end of busy signal on data line
 *
 * \return true if success, otherwise false
 */
static unsigned char hsmci_wait_busy(void) {
  unsigned long busy_wait = 1000000;
  unsigned long sr;

  do {
    sr = HSMCI->HSMCI_SR;
    if (busy_wait-- == 0) {
      hsmci_reset();
      return 0;
    }
  } while (!((sr & HSMCI_SR_NOTBUSY) && ((sr & HSMCI_SR_DTIP) == 0)));
  return 1;
}


/** \brief Send a command
 *
 * \param cmdr       CMDR resgister bit to use for this command
 * \param cmd        Command definition
 * \param arg        Argument of the command
 *
 * \return true if success, otherwise false
 */
static unsigned char hsmci_send_cmd_execute(unsigned long cmdr, sdmmc_cmd_def_t cmd, unsigned long arg) {
  unsigned long sr;

  cmdr |= HSMCI_CMDR_CMDNB(cmd) | HSMCI_CMDR_SPCMD_STD;
  if (cmd & SDMMC_RESP_PRESENT) {
    cmdr |= HSMCI_CMDR_MAXLAT;
    if (cmd & SDMMC_RESP_136) {
      cmdr |= HSMCI_CMDR_RSPTYP_136_BIT;
    } else if (cmd & SDMMC_RESP_BUSY) {
      cmdr |= HSMCI_CMDR_RSPTYP_R1B;
    } else {
      cmdr |= HSMCI_CMDR_RSPTYP_48_BIT;
    }
  }
  //if (cmd & SDMMC_CMD_OPENDRAIN) {
  //  cmdr |= HSMCI_CMDR_OPDCMD_OPENDRAIN;
  //}

  // Write argument
  HSMCI->HSMCI_ARGR = arg;
  // Write and start command
  HSMCI->HSMCI_CMDR = cmdr;

  // Wait end of command
  do {
    sr = HSMCI->HSMCI_SR;
    if (cmd & SDMMC_RESP_CRC) {
      if (sr & (HSMCI_SR_CSTOE | HSMCI_SR_RTOE | HSMCI_SR_RENDE | HSMCI_SR_RCRCE | HSMCI_SR_RDIRE | HSMCI_SR_RINDE)) {
        hsmci_reset();
        return 0;
      }
    } else {
      if (sr & (HSMCI_SR_CSTOE | HSMCI_SR_RTOE | HSMCI_SR_RENDE | HSMCI_SR_RDIRE | HSMCI_SR_RINDE)) {
        hsmci_reset();
        return 0;
      }
    }
  } while (!(sr & HSMCI_SR_CMDRDY));

  if (cmd & SDMMC_RESP_BUSY) {
    if (!hsmci_wait_busy()) {
      return 0;
    }
  }
  return 1;
}


//-------------------------------------------------------------------
//--------------------- PUBLIC FUNCTIONS ----------------------------

void hsmci_init(void) {
  const Pin p = PINS_HSMCI;

  PIO_Configure(&p, 1);

  HSMCI->HSMCI_CR = HSMCI_CR_SWRST;

  // pmc_enable_periph_clk(ID_HSMCI);
  AT91C_BASE_PMC->PMC_PCER = 1 << ID_HSMCI; // enable peripheral clock for hsmci

  //pmc_enable_periph_clk(ID_DMAC);
  //AT91C_BASE_PMC->PMC_PCER = 1 << ID_DMAC; // enable clock for DMA controller

  // Set the Data Timeout Register to 2 Mega Cycles
  HSMCI->HSMCI_DTOR = HSMCI_DTOR_DTOMUL_1048576 | HSMCI_DTOR_DTOCYC(2);
  // Set Completion Signal Timeout to 2 Mega Cycles
  HSMCI->HSMCI_CSTOR = HSMCI_CSTOR_CSTOMUL_1048576 | HSMCI_CSTOR_CSTOCYC(2);
  // Set Configuration Register
  HSMCI->HSMCI_CFG = HSMCI_CFG_FIFOMODE | HSMCI_CFG_FERRCTRL;
  // Set power saving to maximum value
  HSMCI->HSMCI_MR = HSMCI_MR_PWSDIV_Msk;

  // Enable the HSMCI and the Power Saving
  HSMCI->HSMCI_CR = HSMCI_CR_MCIEN | HSMCI_CR_PWSEN;
}

void hsmci_select_device(unsigned long clock, unsigned char bus_width, unsigned char high_speed) {
  if (high_speed) {
    HSMCI->HSMCI_CFG |= HSMCI_CFG_HSMODE;
  } else {
    HSMCI->HSMCI_CFG &= ~HSMCI_CFG_HSMODE;
  } /* endif */

  hsmci_set_speed(clock, BOARD_MCK);

  if (bus_width == 4) {
    HSMCI->HSMCI_SDCR = HSMCI_SDCR_SDCSEL_SLOTA | HSMCI_SDCR_SDCBUS_4;
  } else {
    HSMCI->HSMCI_SDCR = HSMCI_SDCR_SDCSEL_SLOTA | HSMCI_SDCR_SDCBUS_1;
  } /* endif */
}


void hsmci_send_clock(void) {
  // Configure command
  HSMCI->HSMCI_MR &= ~(HSMCI_MR_WRPROOF | HSMCI_MR_RDPROOF | HSMCI_MR_FBYTE);
  // Write argument
  HSMCI->HSMCI_ARGR = 0;
  // Write and start initialization command
  //HSMCI->HSMCI_CMDR = HSMCI_CMDR_RSPTYP_NORESP | HSMCI_CMDR_SPCMD_INIT | HSMCI_CMDR_OPDCMD_OPENDRAIN;
  HSMCI->HSMCI_CMDR = HSMCI_CMDR_RSPTYP_NORESP | HSMCI_CMDR_SPCMD_INIT;
  // Wait end of initialization command
  while (!(HSMCI->HSMCI_SR & HSMCI_SR_CMDRDY));
}


unsigned char hsmci_send_cmd(sdmmc_cmd_def_t cmd, unsigned long arg) {
  // Configure command
  HSMCI->HSMCI_MR &= ~(HSMCI_MR_WRPROOF | HSMCI_MR_RDPROOF | HSMCI_MR_FBYTE);
  // Disable DMA for HSMCI
  HSMCI->HSMCI_DMA = 0;
  HSMCI->HSMCI_BLKR = 0;
  return hsmci_send_cmd_execute(0, cmd, arg);
}


unsigned long hsmci_get_response(void) {
  return HSMCI->HSMCI_RSPR[0];
}


void hsmci_get_response_128(unsigned char* response) {
  unsigned long response_32;
  unsigned char i;

  for (i = 0; i < 4; i++) {
    response_32 = HSMCI->HSMCI_RSPR[0];
    *response = (response_32 >> 24) & 0xFF;
    response++;
    *response = (response_32 >> 16) & 0xFF;
    response++;
    *response = (response_32 >>  8) & 0xFF;
    response++;
    *response = (response_32 >>  0) & 0xFF;
    response++;
  }
}


unsigned char hsmci_adtc_start(sdmmc_cmd_def_t cmd, unsigned long arg, unsigned short block_size, unsigned short nb_block, unsigned char access_block) {
  unsigned long cmdr;

  if (access_block) {
    // Enable DMA for HSMCI
    HSMCI->HSMCI_DMA = HSMCI_DMA_DMAEN;
  } else {
    // Disable DMA for HSMCI
    HSMCI->HSMCI_DMA = 0;
  }

  // Enabling Read/Write Proof allows to stop the HSMCI Clock during
  // read/write  access if the internal FIFO is full.
  // This will guarantee data integrity, not bandwidth.
  HSMCI->HSMCI_MR |= HSMCI_MR_WRPROOF | HSMCI_MR_RDPROOF;
  // Force byte transfer if needed
  if (block_size & 0x3) {
    HSMCI->HSMCI_MR |= HSMCI_MR_FBYTE;
  } else {
    HSMCI->HSMCI_MR &= ~HSMCI_MR_FBYTE;
  }

  if (cmd & SDMMC_CMD_WRITE) {
    cmdr = HSMCI_CMDR_TRCMD_START_DATA | HSMCI_CMDR_TRDIR_WRITE;
  } else {
    cmdr = HSMCI_CMDR_TRCMD_START_DATA | HSMCI_CMDR_TRDIR_READ;
  }

  if (cmd & SDMMC_CMD_SDIO_BYTE) {
      cmdr |= HSMCI_CMDR_TRTYP_BYTE;
      HSMCI->HSMCI_BLKR = (block_size << HSMCI_BLKR_BCNT_Pos);
  } else {
    HSMCI->HSMCI_BLKR = (block_size << HSMCI_BLKR_BLKLEN_Pos) | (nb_block << HSMCI_BLKR_BCNT_Pos);
    if (cmd & SDMMC_CMD_SDIO_BLOCK) {
      cmdr |= HSMCI_CMDR_TRTYP_BLOCK;
    } else if (cmd & SDMMC_CMD_STREAM) {
      cmdr |= HSMCI_CMDR_TRTYP_STREAM;
    } else if (cmd & SDMMC_CMD_SINGLE_BLOCK) {
      cmdr |= HSMCI_CMDR_TRTYP_SINGLE;
    } else if (cmd & SDMMC_CMD_MULTI_BLOCK) {
      cmdr |= HSMCI_CMDR_TRTYP_MULTIPLE;
    //} else {
    //  Assert(false); // Incorrect flags
    }
  }

  return hsmci_send_cmd_execute(cmdr, cmd, arg);
}


unsigned char hsmci_adtc_stop(sdmmc_cmd_def_t cmd, unsigned long arg) {
  return hsmci_send_cmd_execute(HSMCI_CMDR_TRCMD_STOP_DATA, cmd, arg);
}

unsigned char hsmci_read_word(unsigned long* value) {
  unsigned long sr;

  // Wait data available
  do {
    sr = HSMCI->HSMCI_SR;
    if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
      hsmci_reset();

      if (sr & HSMCI_SR_UNRE) {
        sd_debug(60);
      }
      if (sr & HSMCI_SR_OVRE) {
        sd_debug(61);
      }
      if (sr & HSMCI_SR_DTOE) {
        sd_debug(62);
      }
      if (sr & HSMCI_SR_DCRCE) {
        sd_debug(63);
      }

      return 0;
    }
  } while (!(sr & HSMCI_SR_RXRDY));

  // Read data
  *value = HSMCI->HSMCI_RDR;
  return 1;
}

unsigned char hsmci_write_word(unsigned long value) {
  unsigned long sr;

  // Wait data available
  do {
    sr = HSMCI->HSMCI_SR;
    if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
      hsmci_reset();
      return 0;
    }
  } while (!(sr & HSMCI_SR_TXRDY));

  // Write data
  HSMCI->HSMCI_TDR = value;
  return 1;
}
