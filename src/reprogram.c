#include "board.h"
#include "cmsis/core_cm3.h"
#include "file_system.h"
#include "hsmci.h"
#include "sd_mmc.h"
#include "component_hsmci.h"
#include "command.h"

#define MIN_FIRMWARE_SIZE 0x00001000 // if the file contains less than this, it isn't a credible firmware file


// Gets 32 bits of firmware from SD card - assumes hsmci and SD card have been
// prepared accordingly.  This routine must reside in RAM as it's used during
// reprogramming.
unsigned long __attribute__((section(".ramfunc"))) get_long_from_sd_card(void) {
  unsigned char d[8];
  unsigned long r, *p;
  unsigned char c, i;

  r = 0L;
  p = (unsigned long *)d; // allows us to shove longs into an array of chars

  for (i = 0; i < 2; i ++) {
    while (!(HSMCI->HSMCI_SR & HSMCI_SR_RXRDY)) {
    } /* endwhile */

    *(p ++) = HSMCI->HSMCI_RDR;
  } /* endfor */

  for (i = 0; i < 8; i ++) {
    c = d[i];

    if ((c >= 'A') && (c <= 'F')) {
      c -= ('A' - 10);
    } else if ((c >= 'a') && (c <= 'f')) {
      c -= ('a' - 10);
    } else {
      c -= '0';
    } /* endif */

    r |= ((unsigned long)c) << ((i ^ 1) << 2);
  } /* endfor */

  return(r);
}


// Checks whether the SD card file with the specified id contains valid firmware.
unsigned char is_firmware_file_ok(unsigned char *id) {
  unsigned long n, checksum;
  unsigned char r;

  if ((n = start_read_firmware(id)) == 0) {
    report_error(ERROR_BAD_FIRMWARE_FILE);
    return(0);
  } /* endif */

  r = 1;
  checksum = 0L;

  if ((n & ((AT91C_IFLASH_PAGE_SIZE << 1) - 1)) || (n < (MIN_FIRMWARE_SIZE << 1)) || (n > (AT91C_IFLASH_SIZE << 1))) { // check a multiple of the flash page length, not suspiciously small and not too big
    r = 0;
  } else {
    n = n >> 3; // number of long words of firmware (8 chars of file give 1 long word of data)

    while (n) {
      checksum += get_long_from_sd_card();
      n --;
    } /* endwhile */
  } /* endif */

  sd_mmc_finish();

  if (checksum != 0L) {
    r = 0;
  } /* endif */

  if (!r) {
    report_error(ERROR_BAD_FIRMWARE_FILE);
  } /* endif */

  return(r);
}


// Reprograms the flash then resets the processor and peripherals.
// This routine must reside in RAM a] because it potentially modifies the whole flash,
// and obviously mustn't modify itself, and b] because the EEFC instruction to write
// can't be executed from flash.
void __attribute__((section(".ramfunc"))) reprogram(unsigned char *id) {
  unsigned long p, i, n;
  unsigned long *a;

  __disable_irq(); // very important to inhibit all interrupts before we start reprogramming!
  n = start_read_firmware(id) / (2 * AT91C_IFLASH_PAGE_SIZE); // number of pages to program

  // unlock the flash
  for (p = 0; p < (AT91C_IFLASH_SIZE / AT91C_IFLASH_PAGE_SIZE); p ++) {
    AT91C_BASE_EFC->EFC_FCR = (0x5aL << 24) | (p << 8) | AT91C_EFC_FCMD_CLB; // unlock page

    while (!(AT91C_BASE_EFC->EFC_FSR & AT91C_EFC_FRDY)) {
    } /* endwhile */
  } /* endwhile */

  a = (unsigned long *)AT91C_IFLASH;

  // program the flash... anything we call between now and reset must reside in RAM
  for (p = 0; p < n; p ++) {
    for (i = 0; i < AT91C_IFLASH_PAGE_SIZE; i += 4) {
      *(a ++) = get_long_from_sd_card();
    } /* endfor */

    AT91C_BASE_EFC->EFC_FCR = (0x5aL << 24) | (p << 8) | AT91C_EFC_FCMD_EWP; // erase & write page

    while (!(AT91C_BASE_EFC->EFC_FSR & AT91C_EFC_FRDY)) {
    } /* endwhile */
  } /* endfor */

  // lock the flash
  for (p = 0; p < (AT91C_IFLASH_SIZE / AT91C_IFLASH_PAGE_SIZE); p ++) {
    AT91C_BASE_EFC->EFC_FCR = (0x5aL << 24) | (p << 8) | AT91C_EFC_FCMD_SLB; // lock page

    while (!(AT91C_BASE_EFC->EFC_FSR & AT91C_EFC_FRDY)) {
    } /* endwhile */
  } /* endwhile */

  // reset
  AT91C_BASE_RSTC->RSTC_RCR = AT91C_RSTC_KEY | AT91C_RSTC_PERRST | AT91C_RSTC_PROCRST; // reset peripherals and processor

  // just a precaution in case reset isn't immediate...
  while (1) {
  } /* endwhile */
}


void flash_checksum_test(void) {
  extern unsigned long _sprog, _eprog;
  unsigned long checksum, *a;

  checksum = 0L;
  a = &_sprog;

  while (a < &_eprog) {
    checksum += *(a ++);
  } /* endwhile */

  if (checksum != 0L) {
    report_error(ERROR_FLASH_CHECKSUM);
  } /* endif */
}
