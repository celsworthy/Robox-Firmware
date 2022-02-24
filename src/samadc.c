#include <board.h>
#include <pio/pio.h>
#include <irq/irq.h>
#include <adc/adc12.h>
#include <stdio.h>
#include "samadc.h"

#define BOARD_ADC_FREQ 1000000

/// Pio pins to configure.
static const Pin pinsADC[] = {PINS_ADC};


// Enables channel ready for next conversion.  Returns channel conversion.
static unsigned int get_conversion(unsigned char chan) {
  AT91C_BASE_ADC12B->ADC12B_CHER = 1 << chan;
  return(AT91C_BASE_ADC12B->ADC12B_CDR[chan]);
}


void init_adc(void) {
  PIO_Configure(pinsADC, PIO_LISTSIZE(pinsADC));

  ADC12_Initialize(AT91C_BASE_ADC12B,
                   AT91C_ID_ADC12B,
                   AT91C_ADC_TRGEN_DIS,
                   0,
                   AT91C_ADC_SLEEP_NORMAL_MODE,
                   AT91C_ADC_LOWRES_12_BIT,
                   BOARD_MCK,
                   BOARD_ADC_FREQ,
                   10,               // start-up time (us)
                   10000);           // sample & hold time (ns)

  // Make a first conversion on all channels
  AT91C_BASE_ADC12B->ADC12B_CHER = 0xff; // enable all channels
  AT91C_BASE_ADC12B->ADC12B_CR = AT91C_ADC_START;

  while (((AT91C_BASE_ADC12B->ADC12B_SR) & 0xff) != ((AT91C_BASE_ADC12B->ADC12B_CHSR) & 0xff)) {
  } /* endwhile */
}


unsigned int adc_read_nozzle0(void) {
  return(get_conversion(3));
}


unsigned int adc_read_nozzle1(void) {
  return(get_conversion(5));
}


unsigned int adc_read_bed(void) {
  return(get_conversion(1));
}


unsigned int adc_read_ambient(void) {
  return(get_conversion(6));
}


void adc_start_conversion(void) {
  AT91C_BASE_ADC12B->ADC12B_CR = AT91C_ADC_START;
}


// ADC12B has a bug: converting inputs >= full-scale or <= 0 causes erratic deviation of
// around 30 lsbs.  We see this when converting unused thermistor inputs since they have
// pull-down resistors.  What the errata doesn't say is that the effect carries over to
// the next channel converted.  So, we need to avoid converting any unused channels.
// Nozzle1 thermistor is a particular problem as it may or may not be present depending
// on the head type.
// We work around this by only doing conversions on demand.  Here, when conversions are
// complete we inhibit all channels.  Subsequent calls to (eg) adc_read_nozzle1() enable
// the corresponding channels ready for next time around.
unsigned char are_adc_conversions_ready(void) {
  if (((AT91C_BASE_ADC12B->ADC12B_SR) & 0xff) != ((AT91C_BASE_ADC12B->ADC12B_CHSR) & 0xff)) {
    return(0);
  } /* endif */

  AT91C_BASE_ADC12B->ADC12B_CHDR = 0xff; // inhibit all channels
  return(1);
}
