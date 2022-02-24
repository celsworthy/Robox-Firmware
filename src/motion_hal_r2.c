#include <pio/pio.h>
#include "parameters.h"
#include "util.h"
#include "command.h" // for error codes
#include "eeprom.h" // for head access
#include "motion_hal.h"


static const Pin TMC_COMMON_CLK =                  {1 << 27, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT};

static const Pin STEP[6] =                        {{1 << 23, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 1,  AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 13, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}, // ZA
                                                   {1 << 7,  AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 23, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 25, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}};
static const Pin ZB_STEP =                         {1 << 11, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT};

static const Pin DIRECTION[6] =                   {{1 << 31, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 21, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 26, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}, // ZA
                                                   {1 << 14, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 24, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 0,  AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}};
static const Pin ZB_DIRECTION =                    {1 << 19, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT};

static const Pin AXIS_SWITCHES[NUM_AXIS] =        {{1 << 2,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP},
                                                   {1 << 28, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP},
                                                   {1 << 29, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP},
                                                   {1 << 1,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP},
                                                   {1 << 5,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP},
                                                   {1 << 23, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP}};

static const Pin INDEX_WHEELS[NUM_EXTRUDERS] =    {{1 << 10, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_PULLUP},
                                                   {1 << 26, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_PULLUP}};

// currently the extruder SDA pins just provide a means of detecting the presence of the rev2 extruder
static const Pin EXTRUDER_SDA[NUM_EXTRUDERS] =    {{1 << 31, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP},
                                                   {1 << 20, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_PULLUP}};

// currently the extruder SCL pins just provide a ground for the extruder switch
static const Pin EXTRUDER_SCL[NUM_EXTRUDERS] =    {{1 << 12, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 2,  AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}};

static const unsigned char ERROR_CODES[] =         {ERROR_X_DRIVER, ERROR_Y_DRIVER, ERROR_ZA_DRIVER, ERROR_E_DRIVER, ERROR_D_DRIVER};
static const unsigned char TMC2209S[] =            {0, 1, 2, 4, 5}; // X, Y, ZA, E, D; "addresses" of TMC2209s
static const unsigned char ZB_TMC2209 =            3; // "address" of ZB TMC2209

static const Pin SPARE3 =                          {1 << 9, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}; // debug scope trigger


// Updates crc according to the byte d.
// Trinamic have used an unnatural combination of msb-first CRC with lsb-first data.
static void do_crc8(unsigned char d, unsigned char *crc) {
  const unsigned char CRCPOLY = 0x07; // x^8 + x^2 + x^1 + 1 -> 0000 0111 (x^8 implicit)
  unsigned char m;

  for (m = 1; m != 0; m = m << 1) {
    *crc = *crc ^ ((d & m) ? 0x80 : 0);
    *crc = (*crc << 1) ^ ((*crc & 0x80) ? CRCPOLY : 0);
  } /* endfor */
}


// Up to 8 TMC2209s can be addressed; chip[1:0] is the 2 bit address on the bus; chip[2] selects which
// bus (USART) to use.
static void write_tmc2209(unsigned char chip, unsigned char a, unsigned long d) {
  AT91S_USART   *usart;
  unsigned char tx[8], i;

  usart = (chip & 4) ? AT91C_BASE_US1 : AT91C_BASE_US0;

  tx[0] = 0x55;
  tx[1] = chip & 3;
  tx[2] = a | 0x80; // set write flag
  tx[3] = d >> 24;
  tx[4] = d >> 16;
  tx[5] = d >> 8;
  tx[6] = d;
  tx[7] = 0;

  for (i = 0; i < 7; i ++) {
    do_crc8(tx[i], &tx[7]);
  } /* endfor */

  usart->US_CR = AT91C_US_TXEN;
  usart->US_TPR = (unsigned long)tx;
  usart->US_TCR = 8;
  usart->US_PTCR = AT91C_PDC_TXTEN;

  while (usart->US_TCR != 0) {
  } /* endwhile */

  usart->US_PTCR = AT91C_PDC_TXTDIS;
  delay(100e-6); // TMC2209 seems to like a gap between datagrams
}


// Up to 8 TMC2209s can be addressed; chip[1:0] is the 2 bit address on the bus; chip[2] selects which
// bus (USART) to use.
static unsigned char read_tmc2209(unsigned char chip, unsigned char a, unsigned long *d) {
  static unsigned char usart_tout_counters[8] = {0}, usart_crc_counters[8] = {0}; // for debug
  AT91S_USART   *usart;
  unsigned long timer;
  unsigned char tx[4], rx[12], i, rcrc, retries;

  PIO_Clear(&SPARE3);
  usart = (chip & 4) ? AT91C_BASE_US1 : AT91C_BASE_US0;
  *d = 0;

  tx[0] = 0x55;
  tx[1] = chip & 3;
  tx[2] = a & 0x7f; // clear write flag
  tx[3] = 0;

  for (i = 0; i < 3; i ++) {
    do_crc8(tx[i], &tx[3]);
  } /* endfor */

  for (retries = 0; retries < 2; retries ++) {
    usart->US_CR = AT91C_US_TXEN | AT91C_US_RXEN;
    usart->US_TPR = (unsigned long)tx;
    usart->US_TCR = 4;
    usart->US_RPR = (unsigned long)rx;
    usart->US_RCR = 12;
    usart->US_PTCR = AT91C_PDC_TXTEN | AT91C_PDC_RXTEN;
    timer = get_time();

    while ((usart->US_RCR != 0) && !has_period_elapsed(timer, 2e-3)) { // avoid hang if TMC2209 doesn't respond
    } /* endwhile */

    usart->US_PTCR = AT91C_PDC_TXTDIS | AT91C_PDC_RXTDIS;
    usart->US_CR = AT91C_US_RXDIS;
    delay(100e-6); // TMC2209 seems to like a gap between datagrams
    rcrc = 0;

    for (i = 4; i < 11; i ++) {
      do_crc8(rx[i], &rcrc);
    } /* endfor */

    if ((usart->US_RCR == 0) && (rcrc == rx[11])) {
      *d = rx[7];
      *d = *d << 8;
      *d |= rx[8];
      *d = *d << 8;
      *d |= rx[9];
      *d = *d << 8;
      *d |= rx[10];
      return(1);
    } /* endif */

    PIO_Set(&SPARE3);

    if (usart->US_RCR != 0) {
      usart_tout_counters[chip] += (usart_tout_counters[chip] < 255);
    } else {
      usart_crc_counters[chip] += (usart_crc_counters[chip] < 255);
    } /* endif */
  } /* endfor */

  return(0);
}


static unsigned char set_motor_driver(unsigned char chip, unsigned char run_current, unsigned char hold_current) {
  static unsigned long last_currents[8];
  unsigned char        ok = 1;
  unsigned long        ifcnt0, ifcnt1, currents;

  currents = run_current;
  currents = (currents << 8) | hold_current;

  if (currents == 0) {
    //write_tmc2209(chip, 0x03, 0); // min turnaround delay for read access (this is probably default anyway)
    ok = read_tmc2209(chip, 0x02, &ifcnt0) && ok; // count of write datagrams
    write_tmc2209(chip, 0x6c, 0x140100c0); // bits[3:0] = 0 selects driver disable
    write_tmc2209(chip, 0x00, 0x000000c8); // ignore pdn_uart pin; ignore ms1/ms2 pins; external sense resistors; ignore vref pin; invert direction
    write_tmc2209(chip, 0x11, 0x0000000a);
    //write_tmc2209(chip, 0x13, 0); // always stealthchop
    write_tmc2209(chip, 0x13, 117); // stealthchop below 2 rev/sec, otherwise speadcycle
    write_tmc2209(chip, 0x14, 0); // never use coolstep
    write_tmc2209(chip, 0x22, 0); // use step input (probably default anyway)
    write_tmc2209(chip, 0x70, 0xc8040fc8); // pwmconf
    write_tmc2209(chip, 0x01, 0x00000007); // clear global status flags
    ok = read_tmc2209(chip, 0x02, &ifcnt1) && ok; // count of write datagrams
    ok = ok && (((ifcnt1 - ifcnt0) & 0xff) == 8);
  } else if (currents != last_currents[chip]) {
    ok = read_tmc2209(chip, 0x02, &ifcnt0) && ok; // count of write datagrams
    write_tmc2209(chip, 0x10, 0x00060000 | currents);
    write_tmc2209(chip, 0x6c, 0x140100c3); // microplyer x16; driver enable
    ok = read_tmc2209(chip, 0x02, &ifcnt1) && ok; // count of write datagrams
    ok = ok && (((ifcnt1 - ifcnt0) & 0xff) == 2);
  } /* endif */

  last_currents[chip] = currents;
  return(ok);
}


void initialise_motors_r2(void) {
  extern t_motion_hal_state motion_hal_state;
  unsigned char             i;

  PIO_Configure(&SPARE3, 1);

  // configure USARTs to communicate with TMC2209 motor drivers
  AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_US1) | (1 << AT91C_ID_US0); // enable peripheral clocks to USART1 and USART0

  AT91C_BASE_US0->US_CR = AT91C_US_TXDIS | AT91C_US_RXDIS | AT91C_US_RSTTX | AT91C_US_RSTRX;
  AT91C_BASE_US0->US_MR = AT91C_US_CHMODE_NORMAL | AT91C_US_NBSTOP_1_BIT | AT91C_US_PAR_NONE | AT91C_US_CHRL_8_BITS | AT91C_US_USMODE_NORMAL;
  AT91C_BASE_US0->US_BRGR = BOARD_MCK / (16 * 200000); // TMC2209 auto-adjusts to baud rate; somewhat arbitrarily, use 200k baud

  AT91C_BASE_US1->US_CR = AT91C_US_TXDIS | AT91C_US_RXDIS | AT91C_US_RSTTX | AT91C_US_RSTRX;
  AT91C_BASE_US1->US_MR = AT91C_US_CHMODE_NORMAL | AT91C_US_NBSTOP_1_BIT | AT91C_US_PAR_NONE | AT91C_US_CHRL_8_BITS | AT91C_US_USMODE_NORMAL;
  AT91C_BASE_US1->US_BRGR = BOARD_MCK / (16 * 200000); // TMC2209 auto-adjusts to baud rate; somewhat arbitrarily, use 200k baud

  AT91C_BASE_PIOA->PIO_PDR = (1 << 21) | (1 << 20) | (1 << 19) | (1 << 18); // allocate pins to "peripheral A"
  AT91C_BASE_PIOA->PIO_OER = (1 << 20) | (1 << 18);

  // configure PA27 to output 12MHz clock for motor drivers
  AT91C_BASE_PMC->PMC_PCKR[0] = AT91C_PMC_CSS_MAIN_CLK | AT91C_PMC_PRES_CLK; // select main clock (12MHz) / 1
  AT91C_BASE_PMC->PMC_SCER = AT91C_PMC_PCK0; // enable clock output
  PIO_Configure(&TMC_COMMON_CLK, 1);

  PIO_Configure(STEP, PIO_LISTSIZE(STEP));
  PIO_Configure(&ZB_STEP, 1);
  PIO_Configure(DIRECTION, PIO_LISTSIZE(DIRECTION));
  PIO_Configure(&ZB_DIRECTION, 1);

  PIO_Configure(AXIS_SWITCHES, PIO_LISTSIZE(AXIS_SWITCHES));
  PIO_Configure(INDEX_WHEELS, PIO_LISTSIZE(INDEX_WHEELS));
  PIO_Configure(EXTRUDER_SCL, PIO_LISTSIZE(EXTRUDER_SCL));
  PIO_Configure(EXTRUDER_SDA, PIO_LISTSIZE(EXTRUDER_SDA));

  set_b_ms1(1);
  set_b_ms3(1);
  set_b_inhibit(1);

  for (i = 0; i < NUM_EXTRUDERS; i ++) {
    motion_hal_state.extruder_present[i] = !PIO_Get(&EXTRUDER_SDA[i]); // extruder connects sda to scl (driven 0), so sda will be low if extruder present, but high (due to pull-up) if extruder not present
    EXTRUDER_SCL[i].pio->PIO_ODR = EXTRUDER_SCL[i].mask; // now let scl float
  } /* endfor */

  delay(100e-6); // allow time for pull-up to charge stray capacitance

  for (i = 0; i < NUM_EXTRUDERS; i ++) {
    motion_hal_state.extruder_present[i] = PIO_Get(&EXTRUDER_SDA[i]) ? motion_hal_state.extruder_present[i] : 0; // now scl is floating, we expect sda to be 1 - but if rev1 extruder is present, it will still be 0 due to decoupling capacitor
    EXTRUDER_SCL[i].pio->PIO_OER = EXTRUDER_SCL[i].mask; // revert to driving low
  } /* endfor */

  motion_hal_state.gantry_rotation = 0;
  delay(10e-3); // TMC2209s seem to need a while after the power comes up to get their shit together
  turn_off_motors_r2();
}


void make_motor_active_r2(unsigned char axis, unsigned char reduced_current) {
  extern parameter_struct pa;

  reduced_current = reduced_current && ((axis == E_AXIS) || (axis == D_AXIS)); // reduced current only applies to extruders

  if (axis == B_AXIS) {
    set_head_power(1);
    set_b_high_current(pa.motion.motor_current[B_AXIS] > 64); // B is just switchable between high and low values
    set_b_inhibit(0);
  } else {
    if (!set_motor_driver(TMC2209S[axis], reduced_current ? pa.motion.reduced_current[axis] : pa.motion.motor_current[axis], pa.motion.hold_current[axis])) {
      report_error(ERROR_CODES[axis]);
    } /* endif */

    if (axis == Z_AXIS) { // for Z axis, there's a second motor driver to configure
      if (!set_motor_driver(ZB_TMC2209,  reduced_current ? pa.motion.reduced_current[axis] : pa.motion.motor_current[axis], pa.motion.hold_current[axis])) {
        report_error(ERROR_ZB_DRIVER);
      } /* endif */
    } /* endif */
  } /* endif */
}


void make_motor_inactive_r2(unsigned char axis) {
  extern parameter_struct pa;

  if (axis == B_AXIS) {
    set_b_high_current(pa.motion.hold_current[B_AXIS] > 64); // B is just switchable between high and low values
    set_b_inhibit(pa.motion.hold_current[B_AXIS] == 0);
  } /* endif */

  // axes other than B have TMC2209, which deals with switch to hold current automatically
}


void turn_off_motors_r2(void) {
	unsigned char        a;

	for (a = 0; a <= D_AXIS; a ++) {
    if (!set_motor_driver(TMC2209S[a], 0, 0)) {
      report_error(ERROR_CODES[a]);
    } /* endif */
	} /* endfor */

  if (!set_motor_driver(ZB_TMC2209, 0, 0)) {
    report_error(ERROR_ZB_DRIVER);
  } /* endif */

  set_b_inhibit(1);
}


__attribute__((always_inline)) void motor_setdir_r2(unsigned char axis, unsigned char dir) {
  extern t_motion_hal_state motion_hal_state;

  if (axis == Z_AXIS) {
    (dir != motion_hal_state.gantry_rotation) ? PIO_Set(&DIRECTION[Z_AXIS]) : PIO_Clear(&DIRECTION[Z_AXIS]);
    dir ? PIO_Set(&ZB_DIRECTION) : PIO_Clear(&ZB_DIRECTION);
  } else {
    dir ? PIO_Set(&DIRECTION[axis]) : PIO_Clear(&DIRECTION[axis]);
  } /* endif */
}


__attribute__((always_inline)) void motor_step_r2(unsigned char axis) {
  PIO_Set(&STEP[axis]);

  if (axis == Z_AXIS) {
    PIO_Set(&ZB_STEP);
  } /* endif */
}


__attribute__((always_inline)) void motor_unstep_r2(void) {
  PIO_Clear(&STEP[X_AXIS]);
  PIO_Clear(&STEP[Y_AXIS]);
  PIO_Clear(&STEP[Z_AXIS]);
  PIO_Clear(&ZB_STEP);
  PIO_Clear(&STEP[E_AXIS]);
  PIO_Clear(&STEP[D_AXIS]);
  PIO_Clear(&STEP[B_AXIS]);
}


unsigned char amps_to_dac_r2(float amps) {
  return(max(0, min(31, (unsigned char)((amps * 32.0 * 0.13 / 0.325) - 0.5))));
}


float dac_to_amps_r2(unsigned char v) {
  return((float)(v + 1) * 0.325 / (32 * 0.13));
}


inline unsigned char get_axis_switch_r2(unsigned char axis) {
  return(PIO_Get(&AXIS_SWITCHES[axis]));
}


// This routine should always be used to read the index wheel inputs.  It applies
// hysteresis by turning the on-chip pull-up on or off (there is also an off-chip
// pull-up resistor).
unsigned char get_index_wheel_r2(unsigned char axis) {
  static unsigned short index_debug[2] = {0};

  if (PIO_Get(&INDEX_WHEELS[axis])) {
    INDEX_WHEELS[axis].pio->PIO_PPUER = INDEX_WHEELS[axis].mask; // turn on pull-up
    return(1);
  } /* endif */

  if ((INDEX_WHEELS[axis].pio->PIO_PPUSR & INDEX_WHEELS[axis].mask) == 0) { // if pull-up enabled
    index_debug[axis] += (index_debug[axis] < 0xffff);
  } /* endif */

  INDEX_WHEELS[axis].pio->PIO_PPUDR = INDEX_WHEELS[axis].mask; // turn off pull-up
  return(0);
}


void check_motor_drivers_r2(void) {
  extern t_motion_hal_state motion_hal_state;
  static unsigned long      i = 0;
  static unsigned short     driver_flags[8] = {0}; // for debug
  static unsigned char      driver_reset_counters[8] = {0}, driver_err_counters[8] = {0}, driver_uv_counters[8] = {0}; // for debug
  unsigned char             chip, error_code;
  unsigned long             d;

  chip = (i <= D_AXIS) ? TMC2209S[i] : ZB_TMC2209;
  error_code = (i <= D_AXIS) ? ERROR_CODES[i] : ERROR_ZB_DRIVER;

  if (!read_tmc2209(chip, 0x01, &d)) { // GSTAT
    report_error(error_code);
  } else {
    if (d & 0xfffffffb) { // overlook uv_cp as it seems prone to occasional spurious trigger
      report_error(error_code);
    } /* endif */

    driver_reset_counters[chip] += ((d & 1) != 0) && (driver_reset_counters[chip] < 255);
    driver_err_counters[chip] += ((d & 2) != 0) && (driver_err_counters[chip] < 255);
    driver_uv_counters[chip] += ((d & 4) != 0) && (driver_uv_counters[chip] < 255);
  } /* endif */

  if (!read_tmc2209(chip, 0x6f, &d)) { // DRV_STATUS
    report_error(error_code);
  } else {
    driver_flags[chip] |= (d & 0x0f3f);
  } /* endif */

  i = (i <= D_AXIS) ? (i + 1) : 0;
}
