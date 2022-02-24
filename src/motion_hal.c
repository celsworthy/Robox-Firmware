#include <pio/pio.h>
#include "util.h"
#include "motion_hal.h"
#include "motion.h"


// This routine must be called very early at start-up, so that is_rev1_hw() returns
// a valid result.  It's not functionally related to motion, but placed here because
// it uses pins connected to motor drivers.
// On rev1 hardware, PA18 has an external 10K pull-down and PA19 is "SPARE3".
// On rev2 hardware, PA18, PA19 are USART interface to motor drivers and are connected
// by a 1K resistor.
// So, we drive PA19 high then read PA18.  PA18 low -> rev1.  PA18 high -> rev2.
// PA18 is read multiple times so as to be immune to occasional glitches.
void detect_hardware_rev(void) {
  extern t_motion_hal_state motion_hal_state;
  unsigned char             i;
  static unsigned char      rev_counter; // static so it can be observed

  rev_counter = 0;
  AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_PIOA;
  AT91C_BASE_PIOA->PIO_PER = (1 << 19) | (1 << 18); // ensure pins are enabled as pio
  AT91C_BASE_PIOA->PIO_PPUDR = (1 << 19) | (1 << 18); // pull-ups off
  AT91C_BASE_PIOA->PIO_ODR = (1 << 18); // PA18 is input
  AT91C_BASE_PIOA->PIO_SODR = (1 << 19);
  AT91C_BASE_PIOA->PIO_OER = (1 << 19); // PA19 is output
  delay(100e-6); // be careful this doesn't use a resource which hasn't yet been configured
  
  for (i = 0; i < 255; i ++) {
    rev_counter += ((AT91C_BASE_PIOA->PIO_PDSR & (1 << 18)) != 0);
  } /* endfor */
  
  motion_hal_state.hardware_is_rev1 = (rev_counter < 128);
  AT91C_BASE_PIOA->PIO_ODR = (1 << 19); // revert PA19 to input
}


__attribute__((always_inline)) unsigned char is_rev1_hw(void) {
  extern t_motion_hal_state motion_hal_state;
  return(motion_hal_state.hardware_is_rev1);
}


// for status report
void get_hardware_rev(unsigned char *d) {
  *d = is_rev1_hw() ? '1' : '2';
}


void enable_gantry_rotation(unsigned char v) {
  extern t_motion_hal_state motion_hal_state;
  motion_hal_state.gantry_rotation = v;
}


void get_extruder_presence(unsigned char *d) {
  extern t_motion_hal_state motion_hal_state;

  d[0] = '0' + motion_hal_state.extruder_present[0];
  d[1] = '0' + motion_hal_state.extruder_present[1];
}


t_motion_hal_state motion_hal_state;
