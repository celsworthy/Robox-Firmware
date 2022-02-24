#include "sd_mmc.h"
#include "sd_mmc_protocol.h"
#include "hsmci.h"
#include <string.h>


//! This SD MMC stack supports only the high voltage
#define SD_MMC_VOLTAGE_SUPPORT (OCR_VDD_27_28 | OCR_VDD_28_29 | OCR_VDD_29_30 | OCR_VDD_30_31 | OCR_VDD_31_32 | OCR_VDD_32_33)

//! SD/MMC card information structure
typedef struct {
  unsigned long clock;             // Card access clock
  unsigned long capacity;          // Card capacity in KBytes
  unsigned short rca;              // Relative card address
  unsigned char  bus_width;        // Number of DATA lin on bus (MCI only)
  unsigned char csd[CSD_REG_BSIZE];// CSD register
  unsigned char is_sdhc;           // Indicates it's an SDHC card, so use block addresses instead of byte addresses
  unsigned long block;
  unsigned char reading;
  unsigned char writing;
} t_sd_mmc_state;


//! SD/MMC transfer rate unit codes (10K) list
const unsigned long sd_mmc_trans_units[7] = {10, 100, 1000, 10000, 0, 0, 0};

//! SD transfer multiplier factor codes (1/10) list
const unsigned long sd_trans_multipliers[16] = {0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80};


/**
 * \brief Ask to all cards to send their operations conditions (MCI only).
 * - ACMD41 sends operation condition command.
 * - ACMD41 reads OCR
 *
 * \param v2   Shall be 1 if it is a SD card V2
 *
 * \return true if success, otherwise false
 */
static unsigned char sd_mci_op_cond(unsigned char v2) {
  extern t_sd_mmc_state sd_mmc_state;
  unsigned long         arg, retry, resp;

  /*
   * Timeout 1s = 400KHz / ((6+6+6+6)*8) cylces = 2100 retry
   * 6 = cmd byte size
   * 6 = response byte size
   * 6 = cmd byte size
   * 6 = response byte size
   */
  retry = 2100;
  do {
    // CMD55 - Indicate to the card that the next command is an
    // application specific command rather than a standard command.
    if (!hsmci_send_cmd(SDMMC_CMD55_APP_CMD, 0)) {
      return 0;
    }

    // (ACMD41) Sends host OCR register
    arg = SD_MMC_VOLTAGE_SUPPORT;
    if (v2) {
      arg |= SD_ACMD41_HCS;
    }
    // Check response
    if (!hsmci_send_cmd(SD_MCI_ACMD41_SD_SEND_OP_COND, arg)) {
      return 0;
    }
    resp = hsmci_get_response();
    if (resp & OCR_POWER_UP_BUSY) {
      // Card is ready
      sd_mmc_state.is_sdhc = ((resp & OCR_CCS) != 0);
      break;
    }
    if (retry-- == 0) {
      return 0;
    }
  } while (1);
  return 1;
}


/**
 * \brief CMD8 for SD card - Send Interface Condition Command.
 *
 * \note
 * Send SD Memory Card interface condition, which includes host supply
 * voltage information and asks the card whether card supports voltage.
 * Should be performed at initialization time to detect the card type.
 *
 * \param v2 Pointer to v2 flag to update
 *
 * \return true if success, otherwise false
 *         with a update of \ref sd_mmc_err.
 */
static unsigned char sd_cmd8(unsigned char * v2) {
  unsigned long resp;

  *v2 = 0;
  // Test for SD version 2
  if (!hsmci_send_cmd(SD_CMD8_SEND_IF_COND, SD_CMD8_PATTERN | SD_CMD8_HIGH_VOLTAGE)) {
    return 1; // It is not a V2
  }
  // Check R7 response
  resp = hsmci_get_response();
  if (resp == 0xFFFFFFFF) {
    // No compliance R7 value
    return 1; // It is not a V2
  }
  if ((resp & (SD_CMD8_MASK_PATTERN | SD_CMD8_MASK_VOLTAGE)) != (SD_CMD8_PATTERN | SD_CMD8_HIGH_VOLTAGE)) {
    return 0;
  }
  *v2 = 1;
  return 1;
}

/**
 * \brief CMD9: Addressed card sends its card-specific
 * data (CSD) on the CMD line mci.
 *
 * \return true if success, otherwise false
 */
static unsigned char sd_mmc_cmd9_mci(void) {
  extern t_sd_mmc_state sd_mmc_state;

  if (!hsmci_send_cmd(SDMMC_MCI_CMD9_SEND_CSD, (unsigned long)sd_mmc_state.rca << 16)) {
    return 0;
  }
  hsmci_get_response_128(sd_mmc_state.csd);
  return 1;
}

/**
 * \brief Decodes SD CSD register
 */
static void sd_decode_csd(void) {
  extern t_sd_mmc_state sd_mmc_state;
  unsigned long         unit;
  unsigned long         mul;
  unsigned long         tran_speed;

  // Get SD memory maximum transfer speed in Hz.
  tran_speed = CSD_TRAN_SPEED(sd_mmc_state.csd);
  unit = sd_mmc_trans_units[tran_speed & 0x7];
  mul = sd_trans_multipliers[(tran_speed >> 3) & 0xF];
  sd_mmc_state.clock = unit * mul * 1000;

  /*
   * Get card capacity.
   * ----------------------------------------------------
   * For normal SD/MMC card:
   * memory capacity = BLOCKNR * BLOCK_LEN
   * Where
   * BLOCKNR = (C_SIZE+1) * MULT
   * MULT = 2 ^ (C_SIZE_MULT+2)       (C_SIZE_MULT < 8)
   * BLOCK_LEN = 2 ^ READ_BL_LEN      (READ_BL_LEN < 12)
   * ----------------------------------------------------
   * For high capacity SD card:
   * memory capacity = (C_SIZE+1) * 512K byte
   */
  if (CSD_STRUCTURE_VERSION(sd_mmc_state.csd) >= SD_CSD_VER_2_0) {
    sd_mmc_state.capacity = (SD_CSD_2_0_C_SIZE(sd_mmc_state.csd) + 1) * 512;
  } else {
    unsigned long blocknr = ((SD_CSD_1_0_C_SIZE(sd_mmc_state.csd) + 1) * (1 << (SD_CSD_1_0_C_SIZE_MULT(sd_mmc_state.csd) + 2)));
    sd_mmc_state.capacity = blocknr * (1 << SD_CSD_1_0_READ_BL_LEN(sd_mmc_state.csd)) / 1024;
  }
}

/**
 * \brief CMD13 - Addressed card sends its status register.
 * This function waits the clear of the busy flag
 *
 * \return true if success, otherwise false
 */
static unsigned char sd_mmc_cmd13(void) {
  extern t_sd_mmc_state sd_mmc_state;
  unsigned long         nec_timeout;

  /* Wait for data ready status.
   * Nec timing: 0 to unlimited
   * However a timeout is used.
   * 200 000 * 8 cycles
   */
  nec_timeout = 200000;
  do {
    if (!hsmci_send_cmd(SDMMC_MCI_CMD13_SEND_STATUS,
        (unsigned long)sd_mmc_state.rca << 16)) {
      return 0;
    }
    // Check busy flag
    if (hsmci_get_response() & CARD_STATUS_READY_FOR_DATA) {
      break;
    }
    if (nec_timeout-- == 0) {
      return 0;
    }
  } while (1);

  return 1;
}


/**
 * \brief ACMD6 - Define the data bus width to 4 bits bus
 *
 * \return true if success, otherwise false
 */
static unsigned char sd_acmd6(void) {
  extern t_sd_mmc_state sd_mmc_state;

  // CMD55 - Indicate to the card that the next command is an
  // application specific command rather than a standard command.
  if (!hsmci_send_cmd(SDMMC_CMD55_APP_CMD, (unsigned long)sd_mmc_state.rca << 16)) {
    return 0;
  }
  // 10b = 4 bits bus
  if (!hsmci_send_cmd(SD_ACMD6_SET_BUS_WIDTH, 0x2)) {
    return 0;
  }
  sd_mmc_state.bus_width = 4;
  return 1;
}


/**
 * \brief Initialize the SD card in MCI mode.
 *
 * \note
 * This function runs the initialization procedure and the identification
 * process, then it sets the SD/MMC card in transfer state.
 * At last, it will automaticly enable maximum bus width and transfer speed.
 *
 * \return true if success, otherwise false
 */
static unsigned char sd_mmc_mci_card_init(void) {
  extern t_sd_mmc_state sd_mmc_state;
  unsigned char         v2 = 0;

  // In first, try to install SD/SDIO card
  sd_mmc_state.is_sdhc = 0;
  sd_mmc_state.rca = 0;

  // Card need of 74 cycles clock minimum to start
  hsmci_send_clock();

  // CMD0 - Reset all cards to idle state.
  if (!hsmci_send_cmd(SDMMC_MCI_CMD0_GO_IDLE_STATE, 0)) {
    return 0;
  }
  if (!sd_cmd8(&v2)) {
    return 0;
  }

  // Try to get the SD card's operating condition
  if (!sd_mci_op_cond(v2)) {
    // It is not a SD card
    return 0;
  }

  // SD MEMORY, Put the Card in Identify Mode
  // Note: The CID is not used in this stack
  if (!hsmci_send_cmd(SDMMC_CMD2_ALL_SEND_CID, 0)) {
    return 0;
  }
  // Ask the card to publish a new relative address (RCA).
  if (!hsmci_send_cmd(SD_CMD3_SEND_RELATIVE_ADDR, 0)) {
    return 0;
  }
  sd_mmc_state.rca = (hsmci_get_response() >> 16) & 0xFFFF;

  // SD MEMORY, Get the Card-Specific Data
  if (!sd_mmc_cmd9_mci()) {
    return 0;
  }

  sd_decode_csd();

  // Select the and put it into Transfer Mode
  if (!hsmci_send_cmd(SDMMC_CMD7_SELECT_CARD_CMD, (unsigned long)sd_mmc_state.rca << 16)) {
    return 0;
  }
  // SD MEMORY, Read the SCR to get card version
  //if (!sd_acmd51()) {
  //  return 0;
  //}
  // TRY to enable 4-bit mode
  if (!sd_acmd6()) {
    return 0;
  }
  // Switch to selected bus mode
  hsmci_select_device(sd_mmc_state.clock, sd_mmc_state.bus_width, 0); // never use high speed

  // TRY to enable High-Speed Mode.  Commented out for Robox board A01 as EMI7208 ESD protection seems to cause errors at high speed (50MHz)
  //if (sd_mmc_card.version > CARD_VER_SD_1_0) {
  //  if (!sd_cm6_set_high_speed()) {
  //    return 0;
  //  }
  //}

  // SD MEMORY, Set default block size
  if (!hsmci_send_cmd(SDMMC_CMD16_SET_BLOCKLEN, SD_MMC_BLOCK_SIZE)) {
    return 0;
  }
  return 1;
}


//-------------------------------------------------------------------
//--------------------- PUBLIC FUNCTIONS ----------------------------

unsigned char sd_mmc_init(void) {
  extern t_sd_mmc_state sd_mmc_state;

  sd_mmc_state.reading = 0;
  sd_mmc_state.writing = 0;

  hsmci_init();

  // Set 1-bit bus width and low clock for initialization
  sd_mmc_state.clock = SDMMC_CLOCK_INIT;
  sd_mmc_state.bus_width = 1;
  hsmci_select_device(sd_mmc_state.clock, sd_mmc_state.bus_width, 0); // never use high speed

  return(sd_mmc_mci_card_init());
}


// returns capacity in KB
unsigned long sd_mmc_get_capacity(void) {
  extern t_sd_mmc_state sd_mmc_state;
  return (sd_mmc_state.capacity);
}


void sd_mmc_finish(void) {
  extern t_sd_mmc_state sd_mmc_state;

  if (sd_mmc_state.writing || sd_mmc_state.reading) {
    // WORKAROUND for no compliance card (Atmel Internal ref. !MMC7 !SD19):
    // The errors on this command must be ignored
    // and one retry can be necessary in SPI mode for no compliance card.
    if (!hsmci_adtc_stop(SDMMC_CMD12_STOP_TRANSMISSION, 0)) {
      hsmci_adtc_stop(SDMMC_CMD12_STOP_TRANSMISSION, 0);
    } /* endif */

    sd_mmc_state.writing = 0;
    sd_mmc_state.reading = 0;
  } /* endif */
}


unsigned char sd_mmc_read_block(unsigned long block, unsigned char *d) {
  extern t_sd_mmc_state sd_mmc_state;
  unsigned long         v;
  unsigned short        i;

  // if currently writing, or reading at the wrong block, then abort...
  if (sd_mmc_state.writing || (sd_mmc_state.reading && (block != sd_mmc_state.block))) {
    sd_mmc_finish();
  } /* endif */

  // if we need to initiate a read...
  if (!sd_mmc_state.reading) {
    // Wait for data ready status
    if (!sd_mmc_cmd13()) {
      return(0);
    } /* endif */

    if (!hsmci_adtc_start(SDMMC_CMD18_READ_MULTIPLE_BLOCK, sd_mmc_state.is_sdhc ? block : (block * SD_MMC_BLOCK_SIZE), SD_MMC_BLOCK_SIZE, 0, 0)) { // nb_blocks=0 -> inifinite
      return(0);
    } /* endif */

    if (hsmci_get_response() & CARD_STATUS_ERR_RD_WR) {
      return(0);
    } /* endif */

    sd_mmc_state.reading = 1;
    sd_mmc_state.block = block;
  } /* endif */

  if (d != (unsigned char *)NULL) {
    for (i = 0; i < SD_MMC_BLOCK_SIZE; i += 4) {
      if (!hsmci_read_word(&v)) {
        return(0);
      } /* endif */

      memcpy(&d[i], (unsigned char *)&v, 4); // tolerates arbitrary byte alignment
    } /* endfor */

    sd_mmc_state.block ++;
  } /* endif */

  return(1);
}


unsigned char sd_mmc_write_block(unsigned long block, unsigned char *d) {
  extern t_sd_mmc_state sd_mmc_state;
  unsigned short i;
  unsigned long  v;

  // if currently reading, or writing at the wrong block, then abort...
  if (sd_mmc_state.reading || (sd_mmc_state.writing && (block != sd_mmc_state.block))) {
    sd_mmc_finish();
  } /* endif */

  // if we need to initiate a write...
  if (!sd_mmc_state.writing) {
    if (!hsmci_adtc_start(SDMMC_CMD25_WRITE_MULTIPLE_BLOCK, sd_mmc_state.is_sdhc ? block : (block * SD_MMC_BLOCK_SIZE), SD_MMC_BLOCK_SIZE, 0, 0)) { // nb_blocks=0 -> inifinite
      return(0);
    } /* endif */

    if (hsmci_get_response() & CARD_STATUS_ERR_RD_WR) {
      return(0);
    } /* endif */

    sd_mmc_state.writing = 1;
    sd_mmc_state.block = block;
  } /* endif */

  if (d != (unsigned char *)NULL) {
    for (i = 0; i < SD_MMC_BLOCK_SIZE; i += 4) {
      memcpy((unsigned char *)&v, &d[i], 4); // tolerates arbitrary byte alignment

      if (!hsmci_write_word(v)) {
        return(0);
      } /* endif */
    } /* endfor */

    sd_mmc_state.block ++;
  } /* endif */

  return(1);
}


t_sd_mmc_state sd_mmc_state;
