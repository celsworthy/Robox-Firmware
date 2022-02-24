#include <pio/pio.h>
#include "parameters.h"
#include "util.h"
#include "eeprom.h" // for head access
#include "motion_hal.h"

#define MOTOR_CURRENT_UNITS                        7.94e-3 // Amps

// serial interface to AD5206, which sets axis currents except B
const Pin MOSI =                                   {1 << 14, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin SCK =                                    {1 << 15, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin NCS =                                    {1 << 16, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_PULLUP};

// motor inhibit pins
static const Pin X_INHIBIT =                       {1 << 31, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT};
static const Pin Y_INHIBIT =                       {1 << 2,  AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT};
static const Pin ZA_INHIBIT =                      {1 << 13, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT};
static const Pin ZB_INHIBIT =                      {1 << 20, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT};
static const Pin E_INHIBIT =                       {1 << 30, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT};
static const Pin D_INHIBIT =                       {1 << 5,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT};

// motor step pins
static const Pin X_STEP =                          {1 << 28, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
static const Pin Y_STEP =                          {1 << 23, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
static const Pin ZA_STEP =                         {1 << 27, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
static const Pin ZB_STEP =                         {1 << 7,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
static const Pin E_STEP =                          {1 << 21, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
static const Pin D_STEP =                          {1 << 14, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
static const Pin B_STEP =                          {1 << 29, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};

// motor direction pins
static const Pin X_DIRECTION =                     {1 << 8,  AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
static const Pin Y_DIRECTION =                     {1 << 31, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
static const Pin ZA_DIRECTION =                    {1 << 27, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
static const Pin ZB_DIRECTION =                    {1 << 8,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
static const Pin E_DIRECTION =                     {1 << 26, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
static const Pin D_DIRECTION =                     {1 << 21, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
static const Pin B_DIRECTION =                     {1 << 25, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};

// motor mode pins
const Pin MS1 =                                    {1 << 9,  AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_1, PIO_PULLUP};
const Pin MS2 =                                    {1 << 10, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_1, PIO_PULLUP};

static const Pin AXIS_SWITCHES[NUM_AXIS] =        {{1 << 16, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_PULLUP},
                                                   {1 << 24, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP},
                                                   {1 << 12, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP},
                                                   {1 << 17, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_PULLUP},
                                                   {1 << 21, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP},
                                                   {1 << 15, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_PULLUP}};

static const Pin INDEX_WHEELS[NUM_EXTRUDERS] =    {{1 << 14, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_PULLUP},
                                                   {1 << 23, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_PULLUP}};


// Private...

// Simply sends 3 bit channel select and 8 bit value to the AD5206
void send_ad5206(unsigned short d) {
  unsigned short m;

  PIO_Clear(&NCS);

  for (m = 0x0400; m > 0; m = m >> 1) {
    if (d & m) {
      PIO_Set(&MOSI);
    } else {
      PIO_Clear(&MOSI);
    } /* endif */

    PIO_Set(&SCK);
    PIO_Clear(&SCK);
  } /* endfor */

  PIO_Set(&NCS);
}


// Used to detect the presence of an external pull-up resistor on an extruder enable port,
// thereby determining whether the extruder is present.
// The port is driven low, then it's made an input and after a short delay, the port is read.
// If there is an external pull-up, it will read as 1.  If not, stray capacitance will be
// sufficient to ensure it still reads as 0.
// It takes about 1us to reach a 1 level, pulled up by the external 100K resistor.
// The max port leakage is 30nA (data sheet pg. 1109), ie. about 1/1000 of the pull-up current.
// However, the stray capacitance is going to be about 10x lower when there is no extruder.
// Consequently, the port leakage could cause the input to reach a 1 after around 100us when
// no extruder is present.
// So, 20us seems a good time to wait.
static unsigned char has_external_pullup(const Pin *p) {
  unsigned char r;

  PIO_Clear(p); // drive it low
  delay(20e-6);
  p->pio->PIO_ODR = p->mask; // temporarily make it an input
  delay(20e-6);
  r = (p->pio->PIO_PDSR & p->mask) != 0; // if still low, there can't be an external pull-up; NB: PIO_Get() will not do the right thing here, as it thinks the pin is an output
  PIO_Set(p); // drive it high
  p->pio->PIO_OER = p->mask; // revert to being an output
  return(r);
}


// Public...

void initialise_motors_r1(void) {
  extern t_motion_hal_state motion_hal_state;

  PIO_Configure(&MOSI, 1);
  PIO_Configure(&SCK, 1);
  PIO_Configure(&NCS, 1);
  PIO_Configure(&MS1, 1);
  PIO_Configure(&MS2, 1);
  set_b_ms1(1);
  set_b_ms3(1);

  PIO_Configure(&X_INHIBIT, 1);
  PIO_Configure(&Y_INHIBIT, 1);
  PIO_Configure(&ZA_INHIBIT, 1);
  PIO_Configure(&ZB_INHIBIT, 1);
  PIO_Configure(&E_INHIBIT, 1);
  PIO_Configure(&D_INHIBIT, 1);
  set_b_inhibit(1);

  PIO_Configure(&X_DIRECTION, 1);
  PIO_Configure(&Y_DIRECTION, 1);
  PIO_Configure(&ZA_DIRECTION, 1);
  PIO_Configure(&ZB_DIRECTION, 1);
  PIO_Configure(&E_DIRECTION, 1);
  PIO_Configure(&D_DIRECTION, 1);
  PIO_Configure(&B_DIRECTION, 1);

  PIO_Configure(&X_STEP, 1);
  PIO_Configure(&Y_STEP, 1);
  PIO_Configure(&ZA_STEP, 1);
  PIO_Configure(&ZB_STEP, 1);
  PIO_Configure(&E_STEP, 1);
  PIO_Configure(&D_STEP, 1);
  PIO_Configure(&B_STEP, 1);

  PIO_Configure(AXIS_SWITCHES, PIO_LISTSIZE(AXIS_SWITCHES));
  PIO_Configure(INDEX_WHEELS, PIO_LISTSIZE(INDEX_WHEELS));

  motion_hal_state.gantry_rotation = 0;
  motion_hal_state.extruder_present[0] = has_external_pullup(&E_INHIBIT);
  motion_hal_state.extruder_present[1] = has_external_pullup(&D_INHIBIT);
}


// NB: reduced current is an option for E/D axes only, when moving until slip.
void make_motor_active_r1(unsigned char axis, unsigned char reduced_current) {
  extern parameter_struct pa;

  switch (axis) {
  case X_AXIS:
    send_ad5206((3 << 8) | (unsigned short)pa.motion.motor_current[X_AXIS]);
    PIO_Clear(&X_INHIBIT);
    break;
  case Y_AXIS:
    send_ad5206((1 << 8) | (unsigned short)pa.motion.motor_current[Y_AXIS]);
    PIO_Clear(&Y_INHIBIT);
    break;
  case Z_AXIS:
    send_ad5206((4 << 8) | (unsigned short)pa.motion.motor_current[Z_AXIS]); // ZA
    send_ad5206((0 << 8) | (unsigned short)pa.motion.motor_current[Z_AXIS]); // ZB
    PIO_Clear(&ZA_INHIBIT);
    PIO_Clear(&ZB_INHIBIT);
    break;
  case E_AXIS:
    if (reduced_current) {
      send_ad5206((5 << 8) | (unsigned short)pa.motion.reduced_current[E_AXIS]);
    } else {
      send_ad5206((5 << 8) | (unsigned short)pa.motion.motor_current[E_AXIS]);
    } /* endif */

    PIO_Clear(&E_INHIBIT);
    break;
  case D_AXIS:
    if (reduced_current) {
      send_ad5206((2 << 8) | (unsigned short)pa.motion.reduced_current[D_AXIS]);
    } else {
      send_ad5206((2 << 8) | (unsigned short)pa.motion.motor_current[D_AXIS]);
    } /* endif */

    PIO_Clear(&D_INHIBIT);
    break;
  case B_AXIS:
    set_head_power(1);
    set_b_high_current(pa.motion.motor_current[B_AXIS] > 64); // B is just switchable between high and low values
    set_b_inhibit(0);
    break;
  } /* endswitch */
}


void make_motor_inactive_r1(unsigned char axis) {
  extern parameter_struct pa;

  switch (axis) {
  case X_AXIS:
    send_ad5206((3 << 8) | (unsigned short)pa.motion.hold_current[X_AXIS]);

    if (pa.motion.hold_current[X_AXIS] == 0) {
      PIO_Set(&X_INHIBIT);
    } /* endif */

    break;
  case Y_AXIS:
    send_ad5206((1 << 8) | (unsigned short)pa.motion.hold_current[Y_AXIS]);

    if (pa.motion.hold_current[Y_AXIS] == 0) {
      PIO_Set(&Y_INHIBIT);
    } /* endif */

    break;
  case Z_AXIS:
    send_ad5206((4 << 8) | (unsigned short)pa.motion.hold_current[Z_AXIS]); // ZA
    send_ad5206((0 << 8) | (unsigned short)pa.motion.hold_current[Z_AXIS]); // ZB

    if (pa.motion.hold_current[Z_AXIS] == 0) {
      PIO_Set(&ZA_INHIBIT);
      PIO_Set(&ZB_INHIBIT);
    } /* endif */

    break;
  case E_AXIS:
    send_ad5206((5 << 8) | (unsigned short)pa.motion.hold_current[E_AXIS]);

    if (pa.motion.hold_current[E_AXIS] == 0) {
      PIO_Set(&E_INHIBIT);
    } /* endif */

    break;
  case D_AXIS:
    send_ad5206((2 << 8) | (unsigned short)pa.motion.hold_current[D_AXIS]);

    if (pa.motion.hold_current[D_AXIS] == 0) {
      PIO_Set(&D_INHIBIT);
    } /* endif */

    break;
  case B_AXIS:
    set_b_high_current(pa.motion.hold_current[B_AXIS] > 64); // B is just switchable between high and low values
    set_b_inhibit(pa.motion.hold_current[B_AXIS] == 0);
    break;
  } /* endswitch */
}


void turn_off_motors_r1(void) {
  PIO_Set(&X_INHIBIT);
  PIO_Set(&Y_INHIBIT);
  PIO_Set(&ZA_INHIBIT);
  PIO_Set(&ZB_INHIBIT);
  PIO_Set(&E_INHIBIT);
  PIO_Set(&D_INHIBIT);
  set_b_inhibit(1);
}


__attribute__((always_inline)) void motor_setdir_r1(unsigned char axis, unsigned char dir) {
  extern t_motion_hal_state motion_hal_state;

  switch (axis) {
  case X_AXIS:
    dir ? PIO_Set(&X_DIRECTION) : PIO_Clear(&X_DIRECTION);
    break;
  case Y_AXIS:
    dir ? PIO_Set(&Y_DIRECTION) : PIO_Clear(&Y_DIRECTION);
    break;
  case Z_AXIS:
    (dir != motion_hal_state.gantry_rotation) ? PIO_Set(&ZA_DIRECTION) : PIO_Clear(&ZA_DIRECTION);
    dir ? PIO_Set(&ZB_DIRECTION) : PIO_Clear(&ZB_DIRECTION);
    break;
  case E_AXIS:
    dir ? PIO_Set(&E_DIRECTION) : PIO_Clear(&E_DIRECTION);
    break;
  case D_AXIS:
    dir ? PIO_Set(&D_DIRECTION) : PIO_Clear(&D_DIRECTION);
    break;
  case B_AXIS:
    dir ? PIO_Set(&B_DIRECTION) : PIO_Clear(&B_DIRECTION);
    break;
  } /* endswitch */
}


__attribute__((always_inline)) void motor_step_r1(unsigned char axis) {
  switch (axis) {
  case X_AXIS:
    PIO_Set(&X_STEP);
    break;
  case Y_AXIS:
    PIO_Set(&Y_STEP);
    break;
  case Z_AXIS:
    PIO_Set(&ZA_STEP);
    PIO_Set(&ZB_STEP);
    break;
  case E_AXIS:
    PIO_Set(&E_STEP);
    break;
  case D_AXIS:
    PIO_Set(&D_STEP);
    break;
  case B_AXIS:
    PIO_Set(&B_STEP);
    break;
  } /* endswitch */
}


__attribute__((always_inline)) void motor_unstep_r1(void) {
  PIO_Clear(&X_STEP);
  PIO_Clear(&Y_STEP);
  PIO_Clear(&ZA_STEP);
  PIO_Clear(&ZB_STEP);
  PIO_Clear(&E_STEP);
  PIO_Clear(&D_STEP);
  PIO_Clear(&B_STEP);
}


unsigned char amps_to_dac_r1(float amps) {
  return((unsigned char)max(0.0, min(255.0, 0.5 + (amps / MOTOR_CURRENT_UNITS))));
}


float dac_to_amps_r1(unsigned char v) {
  return((float)v * MOTOR_CURRENT_UNITS);
}


inline unsigned char get_axis_switch_r1(unsigned char axis) {
  return(PIO_Get(&AXIS_SWITCHES[axis]));
}


// This routine should always be used to read the index wheel inputs.  It applies
// hysteresis by turning the on-chip pull-up on or off (there is also an off-chip
// pull-up resistor).
inline unsigned char get_index_wheel_r1(unsigned char axis) {
  if (PIO_Get(&INDEX_WHEELS[axis])) {
    INDEX_WHEELS[axis].pio->PIO_PPUER = INDEX_WHEELS[axis].mask; // turn on pull-up
    return(1);
  } /* endif */

  INDEX_WHEELS[axis].pio->PIO_PPUDR = INDEX_WHEELS[axis].mask; // turn off pull-up
  return(0);
}
