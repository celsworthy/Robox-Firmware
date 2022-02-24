#include <board.h>
#include <pio/pio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <irq/irq.h>
#include <tc/tc.h>
#include <math.h>
#include <float.h>

#include "util.h"
#include "command.h"
#include "gcode.h"
#include "parameters.h"
#include "samadc.h"
#include "heaters.h"
#include "file_system.h" // for nonvolatile data
#include "eeprom.h" // for head access
#include "motion.h" // for is_motion_buffer_empty()
#include "motion_hal.h" // for is_rev1_hw()
#include "main.h" // for do_everything_else()
#include "command.h" // for is_automaker_connected()

#define ABS_ZERO                                   -273.15

#define MAINS_DETECT_PERIOD_R1                     0.1 // seconds; if high voltage not detected in this period, we assume 115V; rev 1 hardware only
#define THRESHOLD_FREQUENCY_R2                     334.0 // Hz; for rev 2 hardware, the threshold for deciding whether mains is 115V or 230V

#define HEATER_PWM_FREQUENCY                       40 // Hz
#define HEATER_INTERRUPT_FREQUENCY                 (HEATER_PWM_FREQUENCY * 255)
#define HEAD_FAN_OVERRIDE_TEMPERATURE              60.0 // nozzle temperature at which to force the head fan to operate
#define HEAD_FAN_OVERRIDE_DUTY                     0.6  // ... and the PWM duty at which it is forced
#define HEAD_HOT_TEMPERATURE                       60.0 // head hours are counted when the nozzle is at or above this temperature
#define HEAD_DEFAULT_LIMIT                         300.0 // deg C; normally overridden by value from head EEPROM
#define HEAD_SHUTDOWN_THRESHOLD                    5.0  // deg C; if nozzle temperature exceeds max limit by this amount, then we turn off head power
#define BED_HOT_TEMPERATURE                        60.0 // used for cover interlock
#define BED_MAX_SLOPE                              1.0  // deg C/sec; if this is exceeded, force 230V mode as 230V detect circuit must be faulty
#define BED_TEMPERATURE_DROOP_THRESHOLD            5.0  // deg C; for ERROR_BED_TEMPERATURE_DROOP
#define MAX_BED_TARGET                             160.0
#define MAX_AMBIENT_TARGET                         60.0

// for estimation of time to reach target temperature
#define BED_FINAL_DELTA                            150.0 // deg C; the final temperature of the bed, as a delta from ambient temperature, if heated indefinitely
#define BED_TCONST                                 280.0 // secs; the time constant of the exponential approach to the final delta temperature
#define SINGLE_NOZZLE_FINAL_DELTA                  253.0 // deg C
#define SINGLE_NOZZLE_TCONST                       71.0  // secs
#define DUAL_NOZZLE_FINAL_DELTA                    276.0 // deg C
#define DUAL_NOZZLE_TCONST                         55.0  // secs
#define HIGH_TEMP_NOZZLE_FINAL_DELTA               434.0 // deg C
#define HIGH_TEMP_NOZZLE_TCONST                    84.0  // secs

#define LED_RAMP_TIME                              0.5 // secs

static const Pin BED_DC_HEAT[] =                  {{1 << 20, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT},  // post rev 1
                                                   {1 << 20, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}}; // rev 1
static const Pin BED_AC_HEAT[] =                  {{1 << 22, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT},  // post rev 1
                                                   {1 << 11, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}}; // rev 1
static const Pin AMBIENT_FAN =                     {1 << 18, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT};
static const Pin AMBIENT_FAN_CONTROL[] =          {{1 << 4,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT},  // post rev 1
                                                   {1 << 10, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}}; // rev 1
static const Pin ELECTRONICS_FAN =                 {1 << 28, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT};
static const Pin MAINS_DETECT =                    {1 << 6,  AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT};
static const Pin HEAD_POWER[] =                   {{1 << 7,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT},  // post rev 1
                                                   {1 << 18, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}}; // rev 1
static const Pin HEAD_POWER_DETECT_R2 =            {1 << 27, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT};     // not present on rev 1

static const Pin LEDS[][6] =                     {{{1 << 2,  AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}, // post rev 1
                                                   {1 << 26, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 3,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 14, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 21, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 30, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}},
                                                  {{1 << 2,  AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}, // rev1
                                                   {1 << 1,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 0,  AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 22, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 29, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT},
                                                   {1 << 28, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}}};

// access to head
#define set_nozzle0_heater(v)          set_head_output(3, (v)) // nozzle 0 heater is bit 3 in shift register
#define set_nozzle1_heater(v)          set_head_output(2, (v)) // nozzle 1 heater is bit 2 in shift register
#define set_head_fan(v)                set_head_output(4, (v)) // head fan is bit 4 in shift register


// Private routines...

// Estimates the time it will take to get to target temperature from temperature now.  The system
// is modelled as an exponential approach to (final_delta + ambient) with time constant tconst.
static float time_to_target(float target, float now, float ambient, float final_delta, float tconst) {
  double v;

  v = final_delta + ambient - now;

  if (v <= 0.0) {
    return(FLT_MAX);
  } /* endif */

  v = (target - now) / v;

  if (v <= 0.0) {
    return(0.0);
  } /* endif */

  v = 1.0 - v;

  if (v <= 0.0) {
    return(FLT_MAX);
  } /* endif */

  return(tconst * -log(v)); // NB: natural log
}


// The AC bed heater PWM (on BED_AC_HEAT) works differently from other PWM outputs because the triac can
// only be triggered in a small time window at each zero-crossing of the AC mains.
// If the mains is 230V: the maximum PWM duty (corresponding to tempcon_states.bed.pwm_duty = 255) is 25%.  This
// is done because the bed heater is rated for 115V, and 25% duty at 230V gives the same power as 100% duty at
// 115V.  MAINS_DETECT goes low during each +ve peak of the mains, and we use this to update the PWM output, with
// the result that the heater is only ever on for exactly one mains cycle, after which it will be off for at
// least three cycles.
// If the mains is 115V: maximum PWM duty (corresponding to tempcon_states.bed.pwm_duty = 255) is 100%.
// MAINS_DETECT is inactive, so we cannot use it to trigger synchronised PWM.  So, we use unsynchronised PWM
// updating every 100ms.  This means the heater is on for runs of around five mains cycles, so the uncertainty
// of one half-cycle can only result in a relatively small DC imbalance.  If the PWM updated at a higher frequency
// than the mains, there would be a danger of getting runs of +ve half cycles and no -ve half cycles, or
// vice-versa, resulting in a large DC imbalance.
// The value returned is 230V flag.
static unsigned char do_ac_bed_heater_r1(unsigned char duty, unsigned char force_230v_mode) {
  static unsigned short mains_detect_counter = (unsigned short)HEATER_INTERRUPT_FREQUENCY * MAINS_DETECT_PERIOD_R1;
  static unsigned short bed_acc = 0;
  static unsigned char  last_n230v = 0;
  static unsigned short bed_counter = 0;
  unsigned char         v;

  v = PIO_Get(&MAINS_DETECT);

  // update mains voltage detection; MAINS_DETECT pulses at mains frequency for 230V; always high for 115V
  if (!force_230v_mode && v) {
    mains_detect_counter -= (mains_detect_counter > 0);
  } else {
    mains_detect_counter = (unsigned short)(HEATER_INTERRUPT_FREQUENCY * MAINS_DETECT_PERIOD_R1);
  } /* endif */

  // AC bed heater PWM
  if ((bed_counter >= (unsigned short)(HEATER_INTERRUPT_FREQUENCY / 10.0)) || ((last_n230v == 1) && (v == 0))) { // at mains frequency (230V) or 10Hz (115V)...
    if (mains_detect_counter == 0) { // 115V mains; max pwm duty of 100%
      bed_acc += ((unsigned short)duty) << 2;
    } else { // 230V mains; max pwm duty of 25% (as we're operating 115V heater off 230V)
      bed_acc += duty;
    } /* endif */

    if (bed_acc & 0xfc00) {
      PIO_Set(&BED_AC_HEAT[1]);
    } else {
      PIO_Clear(&BED_AC_HEAT[1]);
    } /* endif */

    bed_acc &= 0x03ff;
    bed_counter = 0;
  } /* endif */

  last_n230v = v;
  bed_counter ++;
  return(mains_detect_counter != 0);
}


// Rev 2 hardware has a new mains detect circuit, where the frequency of MAINS_DETECT represents
// the mains voltage.  We set the threshold midway between the nominal frequencies for 115V and
// 230V.
// Rather than try to deduce the mains timing, we just use an update rate of 10Hz which avoids
// the possibility of large DC imbalance.
static unsigned char do_ac_bed_heater_r2(unsigned char duty, unsigned char force_230v_mode) {
  static unsigned short update_counter = 0;
  static unsigned short bed_acc = 0;
  static unsigned char  pulse_count = 0; // snapshot of pulse_counter for debug / observation
  static unsigned char  pulse_counter = 0;
  static unsigned char  is_230v = 1;

  if (MAINS_DETECT.pio->PIO_ISR & MAINS_DETECT.mask) { // use hardware rising edge detection to avoid danger of missing pulse, which is short
    pulse_counter += (pulse_counter < 255);
  } /* endif */

  update_counter ++;

  if (update_counter >= (unsigned short)(HEATER_INTERRUPT_FREQUENCY / 10.0)) {
    is_230v = (pulse_counter > (unsigned char)(THRESHOLD_FREQUENCY_R2 / 10.0)) || (pulse_counter < 10); // if pulse frequency suspiciously low, assume detect circuit is broken and err on the safe side
    pulse_count = pulse_counter;
    (void)pulse_count; // suppress unused variable warning
    pulse_counter = 0;

    if (is_230v || force_230v_mode) { // limit duty to 25%
      bed_acc += duty;
    } else { // allow 100% duty
      bed_acc += ((unsigned short)duty) << 2;
    } /* endif */

    if (bed_acc & 0xfc00) {
      PIO_Set(&BED_AC_HEAT[0]);
    } else {
      PIO_Clear(&BED_AC_HEAT[0]);
    } /* endif */

    bed_acc &= 0x03ff;
    update_counter = 0;
  } /* endif */

  return(is_230v);
}


// Timer 1 ISR; software PWM generation for heaters and fans.
// pwm_counter simply increments every interrupt, but rolls back to 0 after 254.  Most PWM outputs are generated
// by simply comparing the desired duty (in the range 0 to 255) against pwm_counter.
// For the ambient LED PWM, the value of pwm_counter is left-rotated, which has the effect of doubling the PWM
// frequency to avoid visible flicker.  For what it's worth, the resolution remains the same (averaged over 2
// cycles).
void TC1_IrqHandler(void) {
  extern t_tempcon_states tempcon_states;
  extern t_pwm_state      pwm_state;
  volatile unsigned short dummy;
  static unsigned char    pwm_counter = 0;
  static unsigned short   ambient_counter = 0;
  unsigned char           i;
  unsigned short          w;

  dummy = AT91C_BASE_TC1->TC_SR; // clear status bit to acknowledge interrupt
  (void)dummy; // suppresses unused variable warning

  // nozzle PWM
  set_nozzle0_heater((tempcon_states.nozzle[0].pwm_duty > pwm_counter) && !tempcon_states.force_nozzles_off);
  set_nozzle1_heater((tempcon_states.nozzle[1].pwm_duty > pwm_counter) && !tempcon_states.force_nozzles_off);

  // 2-wire ambient fan PWM
  if ((tempcon_states.ambient.pwm_duty > pwm_counter) || tempcon_states.force_ambient_fan) {
    PIO_Set(&AMBIENT_FAN);
  } else {
    PIO_Clear(&AMBIENT_FAN);
  } /* endif */

  // 3-wire ambient fan PWM
  ambient_counter += tempcon_states.ambient.pwm_duty;

  if ((ambient_counter > 0xff) || tempcon_states.force_ambient_fan) {
    PIO_Set(&AMBIENT_FAN_CONTROL[is_rev1_hw()]);
  } else {
    PIO_Clear(&AMBIENT_FAN_CONTROL[is_rev1_hw()]);
  } /* endif */

  ambient_counter &= 0xff;

  // head fan PWM
  set_head_fan(pwm_state.head_fan_duty > pwm_counter);

  // DC bed heater PWM
  if (tempcon_states.bed.pwm_duty > pwm_counter) {
    PIO_Set(&BED_DC_HEAT[is_rev1_hw()]);
  } else {
    PIO_Clear(&BED_DC_HEAT[is_rev1_hw()]);
  } /* endif */

  // deal with AC bed heater and mains voltage detection
  if (is_rev1_hw()) {
    pwm_state.is_230v = do_ac_bed_heater_r1(tempcon_states.bed.pwm_duty, pwm_state.force_230v_mode);
  } else {
    pwm_state.is_230v = do_ac_bed_heater_r2(tempcon_states.bed.pwm_duty, pwm_state.force_230v_mode);
  } /* endif */

  w = pwm_state.led_ramp_counter;
  pwm_state.led_ramp_counter += (unsigned short)(65536.0 * 255.0 / (LED_RAMP_TIME * HEATER_INTERRUPT_FREQUENCY));

  if (pwm_state.led_ramp_counter < w) { // rollover
    for (i = 0; i < 6; i ++) {
      if (pwm_state.led_duties[i] != (pwm_state.led_duties_ramped[i] >> 8)) {
        pwm_state.led_duties_ramped[i] += pwm_state.led_rates[i];
      } /* endif */
    } /* endfor */
  } /* endif */

  w = (((pwm_counter << 1) | (pwm_counter >= 0x80)) << 8) | 0xff; // has the effect of doubling pwm frequency for LEDs

  for (i = 0; i < 6; i ++) {
    if (pwm_state.led_duties_ramped[i] > w) {
      PIO_Set(&LEDS[is_rev1_hw()][i]);
    } else {
      PIO_Clear(&LEDS[is_rev1_hw()][i]);
    } /* endif */
  } /* endfor */

  // count from 0 to 254 then wrap back to 0; this means 255 gives continuous on
  if (pwm_counter < 254) {
    pwm_counter ++;
  } else {
    pwm_counter = 0;
  } /* endif */
}


// Init timer 1 for software PWM
static void ConfigureTc_1(void) {
  AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TC1; // enable peripheral clock

  TC_Configure(AT91C_BASE_TC1, 2 | AT91C_TC_CPCTRG); // MCK/32; reset counter on compare
  AT91C_BASE_TC1->TC_RC = (unsigned short)(BOARD_MCK / (32.0 * HEATER_INTERRUPT_FREQUENCY));

  IRQ_ConfigureIT(AT91C_ID_TC1, 6, TC1_IrqHandler); // configure and enable interrupt on timer
  AT91C_BASE_TC1->TC_IER = AT91C_TC_CPCS; // interrupt on compare
  IRQ_EnableIT(AT91C_ID_TC1);
  TC_Start(AT91C_BASE_TC1);
}


// PID controller.  Must be invoked at TEMPCON_SAMPLING_FREQUENCY.
static void temperature_controller(t_tempcon_state *s, t_tempcon_parameters *p, unsigned char invert) {
  float target, error, total;

  if (s->mode) {
    // mode=1 -> use target[0]; mode=2 -> use target[1]; mode=3 -> UNLOAD_NOZZLE_TEMPERATURE
    target = (s->mode == 1) ? s->target[0] : ((s->mode == 2) ? s->target[1] : UNLOAD_NOZZLE_TEMPERATURE);
    error = invert ? (s->measured - target) : (target - s->measured);
    total = (p->kp * error) + s->integrator + (p->kd * (s->last_measured - s->measured) * TEMPCON_SAMPLING_FREQUENCY);
    total = max(0.0, min(1.0, total));
    error = (total - s->integrator) / p->kp; // NB: Kd term not subtracted - this is important
    s->integrator += (p->ki * error / TEMPCON_SAMPLING_FREQUENCY);
    s->pwm_duty = (unsigned char)(255.0 * total);
  } else {
    s->pwm_duty = 0;
    s->integrator = 0.0;
  } /* endif */

  s->last_measured = s->measured;
}


static void set_temperature_parameter_handler(char *line, t_tempcon_parameters *p) {
  if (has_code(line, 'P')) {
    p->kp = get_float(line, 'P');
  } /* endif */

  if (has_code(line, 'I')) {
    p->ki = get_float(line, 'I');
  } /* endif */

  if (has_code(line, 'D')) {
    p->kd = get_float(line, 'D');
  } /* endif */

  if (has_code(line, 'B')) {
    p->beta = get_float(line, 'B');
  } /* endif */

  if (has_code(line, 'T')) {
    p->tcal = get_float(line, 'T');
  } /* endif */

  if (has_code(line, 'U')) {
    p->pullup = get_bool(line, 'U');
  } /* endif */
}


// The thermistor data sheet specifies beta and R0 @ T0 (kelvin)
// rinf is pre-calculated as  R0 * exp(-beta / T0)
// tcal is then pre-calculated as rpull / rinf
static float thermistor_to_celsius(unsigned short adc, float beta, float tcal, unsigned char pullup) {
  float x, celsius;

  adc = (pullup) ? 4095 - adc : adc;
  adc = (adc == 0) ? 1 : adc; // avoid divide by zero
  x = (float)(4095 - adc) / (float)adc;
  celsius = ABS_ZERO + (beta / log(x * tcal));
  return(celsius);
}


// tcal represents rpull / R0 (where R0 is resistance of sensor at 0 celsius).
// we solve quadratic at^2 + bt + c = 0, where a and b come from the datasheet and c = 1-(R/R0)
// it follows that c = 1-(tcal*(4095-adc)/adc)
static float prtd_to_celsius(unsigned short adc, float a, float b, float tcal, unsigned char pullup) {
  float c, x, celsius;

  adc = (pullup) ? 4095 - adc : adc;
  adc = (adc == 0) ? 1 : adc; // avoid divide by zero
  x = (float)(4095 - adc) / (float)adc;
  c = 1.0 - (tcal * x); // c as in at^2 + bt + c
  celsius = (-b + sqrt(max(0.0, (b * b) - (4.0 * a * c)))) / (2.0 * a); // (-b + sqrt(b^2 - 4ac)) / 2a
  return(celsius);
}


// This routine should be invoked each time the temperature controllers are invoked, ie. at TEMPCON_SAMPLING_FREQUENCY.
// It provides a backup means of 230V detection, based on checking for excessive +ve bed temperature slope.
// tempcon_states.bed_excessive_slope_lpf is a low-pass filtered version of the bed temperature.  The difference between
// the bed temperature and the lpf provides a filtered indication of the slope; if this difference exceeds a threshold,
// then pwm_state.force_230v_mode is set (and stays set until power-cycle or reset).
static void backup_230v_detector(void) {
  extern t_tempcon_states tempcon_states;
  extern t_pwm_state      pwm_state;
  float                   e;

  e = tempcon_states.bed.measured - tempcon_states.bed_excessive_slope_lpf;
  tempcon_states.bed_excessive_slope_lpf += e / (5.0 * TEMPCON_SAMPLING_FREQUENCY);
  pwm_state.force_230v_mode = pwm_state.force_230v_mode || (e >= (5.0 * BED_MAX_SLOPE));
}


// This routine provides protection against temperature run-away as a result of an open circuit thermistor fault.
// Unfortunately, at room temperature it is difficult to tell whether the thermistor is open-circuit, as its
// resistance is very high compared to the pull-down resistor.  Consequently, we allow a period of heating and
// then check whether the thermistor is open-circuit.
// Note the use of does_head_have_two_fans() to avoid spurious fault detection when nozzle1 PWM is being used
// to drive a fan rather than a heater.
static void thermistor_fault_detector(void) {
  extern t_tempcon_states tempcon_states;

  if (tempcon_states.bed.pwm_duty != 255) {
    tempcon_states.bed.fault_timer = get_time();
  } else if (has_period_elapsed(tempcon_states.bed.fault_timer, 60.0)) {
    if (tempcon_states.bed.measured < 20.0) {
      report_error(ERROR_BED_THERMISTOR);
      tempcon_states.bed.mode = 0;
    } /* endif */
  } /* endif */

  if (tempcon_states.nozzle[0].pwm_duty != 255) {
    tempcon_states.nozzle[0].fault_timer = get_time();
  } else if (has_period_elapsed(tempcon_states.nozzle[0].fault_timer, 20.0)) {
    if (tempcon_states.nozzle[0].measured < 20.0) {
      report_error(ERROR_NOZZLE0_THERMISTOR);
      tempcon_states.nozzle[0].mode = 0;
    } /* endif */
  } /* endif */

  if ((tempcon_states.nozzle[1].pwm_duty != 255) || does_head_have_two_fans()) {
    tempcon_states.nozzle[1].fault_timer = get_time();
  } else if (has_period_elapsed(tempcon_states.nozzle[1].fault_timer, 20.0)) {
    if (tempcon_states.nozzle[1].measured < 20.0) {
      report_error(ERROR_NOZZLE1_THERMISTOR);
      tempcon_states.nozzle[1].mode = 0;
    } /* endif */
  } /* endif */
}


// Public routines...

void initialise_heaters(void) {
  extern t_tempcon_states tempcon_states;
  extern t_pwm_state      pwm_state;
  unsigned char           i;

  PIO_Configure(&HEAD_POWER[is_rev1_hw()], 1);

  if (!is_rev1_hw()) {
    PIO_Configure(&HEAD_POWER_DETECT_R2, 1);
  } /* endif */

  set_head_power(1); // on post rev 1 hardware, checks for short before turning on power

  PIO_Configure(&AMBIENT_FAN, 1);
  PIO_Configure(&ELECTRONICS_FAN, 1);
  PIO_Configure(&BED_DC_HEAT[is_rev1_hw()], 1);
  PIO_Configure(&BED_AC_HEAT[is_rev1_hw()], 1);
  PIO_Configure(&AMBIENT_FAN_CONTROL[is_rev1_hw()], 1);
  PIO_Configure(LEDS[is_rev1_hw()], PIO_LISTSIZE(LEDS[0]));

  // set up rising edge detection (though it's only used on Rev 2 hardware)
  PIO_Configure(&MAINS_DETECT, 1);
  MAINS_DETECT.pio->PIO_AIMER = MAINS_DETECT.mask; // additional interrupt mode enable
  MAINS_DETECT.pio->PIO_ESR = MAINS_DETECT.mask; // edge detection
  MAINS_DETECT.pio->PIO_REHLSR = MAINS_DETECT.mask; // rising edge
  MAINS_DETECT.pio->PIO_AIMMR = MAINS_DETECT.mask; // select additional interrupt mode rather than default (both edges) mode

  for (i = 0; i < 2; i ++) {
    tempcon_states.nozzle[i].mode = 0;
    tempcon_states.nozzle[i].target[0] = 180.0;
    tempcon_states.nozzle[i].target[1] = 180.0;
    tempcon_states.nozzle[i].measured = 0;
    tempcon_states.nozzle[i].last_measured = 0;
    tempcon_states.nozzle[i].integrator = 0;
    tempcon_states.nozzle[i].pwm_duty = 0;
    tempcon_states.nozzle_mode_at_pause[i] = 0;
    tempcon_states.nozzle[i].fault_timer = get_time();
    tempcon_states.nozzle[i].unload_timer = get_time(); // used to determine when it's safe to unload filament
  } /* endfor */

  tempcon_states.bed.mode = 0;
  tempcon_states.bed.target[0] = 0.0;
  tempcon_states.bed.target[1] = 0.0;
  tempcon_states.bed.measured = 0;
  tempcon_states.bed.last_measured = 0;
  tempcon_states.bed.integrator = 0;
  tempcon_states.bed.pwm_duty = 0;
  tempcon_states.bed.fault_timer = get_time();

  tempcon_states.ambient.mode = 0;
  tempcon_states.ambient.target[0] = 0.0;
  tempcon_states.ambient.target[1] = 0.0;
  tempcon_states.ambient.measured = 0;
  tempcon_states.ambient.last_measured = 0;
  tempcon_states.ambient.integrator = 0;
  tempcon_states.ambient.pwm_duty = 0;
  tempcon_states.ambient.fault_timer = get_time();

  tempcon_states.timer = get_time();
  tempcon_states.force_ambient_fan = 0;
  tempcon_states.why_are_we_waiting = NOT_WAITING;
  tempcon_states.bed_excessive_slope_lpf = MAX_BED_TARGET;
  tempcon_states.force_nozzles_off = 0;
  tempcon_states.enable_nozzle_forcing = 0;
  tempcon_states.timing_pause = 0;
  tempcon_states.pause_timer = get_time();

  pwm_state.head_fan_duty = 0;
  pwm_state.head_fan_requested_duty = 0;

  for (i = 0; i < 6; i ++) {
    pwm_state.led_duties[i] = 0;
    pwm_state.led_duties_ramped[i] = 0;
    pwm_state.led_rates[i] = 0;
  } /* endfor */

  pwm_state.led_ramp_counter = 0;
  pwm_state.force_230v_mode = 0;

  ConfigureTc_1();
}


// Maintains temperature control; should be called frequently, relative to TEMPCON_SAMPLING_FREQUENCY.  When it
// is time for the next update of the temperature controllers, the ADC is triggered.  When ADC results are ready,
// the temperature controllers are run.
void maintain_heaters(void) {
  extern t_tempcon_states tempcon_states;
  extern t_pwm_state      pwm_state;
  extern parameter_struct pa;
  static float            bed_got_to = -FLT_MAX;
  float                   limit;

  if (are_adc_conversions_ready()) {
    // temperature limit is applied here rather than when the targets are set, as targets might get
    // set before the head type is known
    limit = get_head_max_temperature();
    tempcon_states.nozzle[0].target[0] = min(tempcon_states.nozzle[0].target[0], limit);
    tempcon_states.nozzle[0].target[1] = min(tempcon_states.nozzle[0].target[1], limit);
    tempcon_states.nozzle[1].target[0] = min(tempcon_states.nozzle[1].target[0], limit);
    tempcon_states.nozzle[1].target[1] = min(tempcon_states.nozzle[1].target[1], limit);

    tempcon_states.ambient.measured = thermistor_to_celsius(adc_read_ambient(), pa.ambient_tempcon.beta, pa.ambient_tempcon.tcal, pa.ambient_tempcon.pullup);
    temperature_controller(&tempcon_states.ambient, &pa.ambient_tempcon, 1); // invert flag set because fan is a cooler, not a heater

    if (does_head_have_prtd()) {
      tempcon_states.nozzle[0].measured = prtd_to_celsius(adc_read_nozzle0(), pa.nozzle_tempcon.prtd_a, pa.nozzle_tempcon.prtd_b, pa.nozzle_tempcon.prtd_tcal, pa.nozzle_tempcon.pullup);
    } else {
      tempcon_states.nozzle[0].measured = thermistor_to_celsius(adc_read_nozzle0(), pa.nozzle_tempcon.beta, pa.nozzle_tempcon.tcal, pa.nozzle_tempcon.pullup);
    } /* endif */

    if (!is_head_dual_material()) {
      tempcon_states.nozzle[1].measured = 0;
    } else if (does_head_have_prtd()) {
      tempcon_states.nozzle[1].measured = prtd_to_celsius(adc_read_nozzle1(), pa.nozzle_tempcon.prtd_a, pa.nozzle_tempcon.prtd_b, pa.nozzle_tempcon.prtd_tcal, pa.nozzle_tempcon.pullup);
    } else {
      tempcon_states.nozzle[1].measured = thermistor_to_celsius(adc_read_nozzle1(), pa.nozzle_tempcon.beta, pa.nozzle_tempcon.tcal, pa.nozzle_tempcon.pullup);
    } /* endif */

    temperature_controller(&tempcon_states.nozzle[0], &pa.nozzle_tempcon, 0);

    if (does_head_have_two_fans()) { // if there's a second fan, it replaces nozzle[1] heater
      tempcon_states.nozzle[1].pwm_duty = pwm_state.head_fan_requested_duty;
    } else {
      temperature_controller(&tempcon_states.nozzle[1], &pa.nozzle_tempcon, 0);
    } /* endif */

    tempcon_states.bed.measured = thermistor_to_celsius(adc_read_bed(), pa.bed_tempcon.beta, pa.bed_tempcon.tcal, pa.bed_tempcon.pullup);
    temperature_controller(&tempcon_states.bed, &pa.bed_tempcon, 0);

    if ((tempcon_states.nozzle[0].measured >= HEAD_FAN_OVERRIDE_TEMPERATURE) || (tempcon_states.nozzle[1].measured >= HEAD_FAN_OVERRIDE_TEMPERATURE)) {
      pwm_state.head_fan_duty = max(pwm_state.head_fan_requested_duty, (unsigned char)(255.0 * HEAD_FAN_OVERRIDE_DUTY));
    } else {
      pwm_state.head_fan_duty = pwm_state.head_fan_requested_duty;
    } /* endif */

    if (tempcon_states.nozzle[0].mode || tempcon_states.nozzle[1].mode || !is_motion_buffer_empty()) {
      PIO_Set(&ELECTRONICS_FAN);
    } else {
      PIO_Clear(&ELECTRONICS_FAN);
    } /* endif */

    // If nozzle temperature exceeds max limit plus margin, then shut down head power.
    if (((tempcon_states.nozzle[0].measured - get_eeprom_float(0, 0x28, HEAD_DEFAULT_LIMIT)) >= HEAD_SHUTDOWN_THRESHOLD) ||
        ((tempcon_states.nozzle[1].measured - get_eeprom_float(0, 0x28, HEAD_DEFAULT_LIMIT)) >= HEAD_SHUTDOWN_THRESHOLD)) {
      if (is_head_power_on()) {
        report_error(ERROR_HEAD_POWER_OVERTEMP);
      } /* endif */

      set_head_power(0);
    } /* endif */

    // These timers are used to judge when it's safe to unload filament
    if (tempcon_states.nozzle[0].measured < (UNLOAD_NOZZLE_TEMPERATURE * (1.0 - NOZZLE_TEMP_MARGIN))) {
      tempcon_states.nozzle[0].unload_timer = get_time();
    } /* endif */

    if (tempcon_states.nozzle[1].measured < (UNLOAD_NOZZLE_TEMPERATURE * (1.0 - NOZZLE_TEMP_MARGIN))) {
      tempcon_states.nozzle[1].unload_timer = get_time();
    } /* endif */

    if (tempcon_states.timing_pause) {
      if (has_period_elapsed(tempcon_states.pause_timer, NOZZLE_PAUSE_AUTO_OFF_TIME)) {
        tempcon_states.nozzle[0].mode = 0;
        tempcon_states.nozzle[1].mode = 0;
        tempcon_states.timing_pause = 0;
      } /* endif */
    } /* endif */

    // check for bed temperature droop
    if (tempcon_states.bed.mode > 0) {
      bed_got_to = min(tempcon_states.bed.target[tempcon_states.bed.mode - 1], max(bed_got_to, tempcon_states.bed.measured));

      if ((bed_got_to - tempcon_states.bed.measured) > BED_TEMPERATURE_DROOP_THRESHOLD) {
        report_error(ERROR_BED_TEMPERATURE_DROOP);
        bed_got_to = -FLT_MAX;
      } /* endif */
    } else {
      bed_got_to = -FLT_MAX;
    } /* endif */

    backup_230v_detector();
    thermistor_fault_detector();
    write_nonvolatile_data(0, (max(tempcon_states.nozzle[0].measured, tempcon_states.nozzle[1].measured) > HEAD_HOT_TEMPERATURE)); // for detection of ERROR_POWEROFF_WHILST_HOT
  } /* endif */

  if (has_period_elapsed(tempcon_states.timer, 1.0 / TEMPCON_SAMPLING_FREQUENCY)) {
    adc_start_conversion();
    tempcon_states.timer = get_time();
  } /* endif */
}


void turn_off_heaters(void) {
  extern t_tempcon_states tempcon_states;
  extern t_pwm_state      pwm_state;

  tempcon_states.timing_pause = 0;
  tempcon_states.nozzle[0].mode = 0;
  tempcon_states.nozzle[1].mode = 0;
  tempcon_states.bed.mode = 0;
  tempcon_states.ambient.mode = 0;
  pwm_state.head_fan_requested_duty = 0;
}


// Should be called on pause.  Simply turns off the nozzle heater, remembering its mode so it can be restored on resume.
void pause_heaters(void) {
  extern t_tempcon_states tempcon_states;

  tempcon_states.nozzle_mode_at_pause[0] = tempcon_states.nozzle[0].mode; // remember mode for resume_heaters()
  tempcon_states.nozzle_mode_at_pause[1] = tempcon_states.nozzle[1].mode; // remember mode for resume_heaters()
  tempcon_states.pause_timer = get_time();
  tempcon_states.timing_pause = 1; // enable auto turn-off of nozzle heaters if paused a long time
}


// Should be called on resume.  The mode of the nozzle temperature controller is restored to the value
// saved at pause.  Then, we wait for the nozzle and bed to get to temperature, if necessary.
void resume_heaters(void) {
  extern t_tempcon_states tempcon_states;

  tempcon_states.timing_pause = 0;
  tempcon_states.nozzle[0].mode = tempcon_states.nozzle_mode_at_pause[0];
  tempcon_states.nozzle[1].mode = tempcon_states.nozzle_mode_at_pause[1];
  wait_until_nozzle_is_at_temperature(1); // note ignore_pause; otherwise it would return immediately!
  wait_until_bed_is_at_temperature("", 1); // note ignore_pause; otherwise it would return immediately!
}


// Should be called before the filament is retracted from the head.  Gets the nozzle to the temperature
// which is desirable for filament retraction, with fan assistance if appropriate.
void heat_for_filament_unload(unsigned char do_nozzle0, unsigned char do_nozzle1) {
  extern t_tempcon_states tempcon_states;
  extern t_pwm_state      pwm_state;
  unsigned char           temp_f, done;

  temp_f = pwm_state.head_fan_requested_duty;
  pwm_state.head_fan_requested_duty = 0;

  if (do_nozzle0) {
    tempcon_states.nozzle[0].mode = 3; // selects UNLOAD_NOZZLE_TEMPERATURE

    if (tempcon_states.nozzle[0].measured > UNLOAD_NOZZLE_TEMPERATURE) {
      pwm_state.head_fan_requested_duty = 255; // if cooling needed, assist with fan
    } /* endif */
  } /* endif */

  if (do_nozzle1) {
    tempcon_states.nozzle[1].mode = 3; // selects UNLOAD_NOZZLE_TEMPERATURE

    if (tempcon_states.nozzle[1].measured > UNLOAD_NOZZLE_TEMPERATURE) {
      pwm_state.head_fan_requested_duty = 255; // if cooling needed, assist with fan
    } /* endif */
  } /* endif */

  tempcon_states.why_are_we_waiting = NOZZLE_AT_TEMPERATURE;

  do {
    do_everything_else();
    done = 1;

    if (do_nozzle0 && tempcon_states.nozzle[0].mode) {
      done = done && ((fabs(UNLOAD_NOZZLE_TEMPERATURE - tempcon_states.nozzle[0].measured) / UNLOAD_NOZZLE_TEMPERATURE) < NOZZLE_TEMP_MARGIN);
      done = done && has_period_elapsed(tempcon_states.nozzle[0].unload_timer, UNLOAD_NOZZLE_DWELL_TIME);
    } /* endif */

    if (do_nozzle1 && tempcon_states.nozzle[1].mode) {
      done = done && ((fabs(UNLOAD_NOZZLE_TEMPERATURE - tempcon_states.nozzle[1].measured) / UNLOAD_NOZZLE_TEMPERATURE) < NOZZLE_TEMP_MARGIN);
      done = done && has_period_elapsed(tempcon_states.nozzle[1].unload_timer, UNLOAD_NOZZLE_DWELL_TIME);
    } /* endif */

    done = done || is_abort();
    done = done || !is_head_power_on(); // if head power off, there's no point waiting!
  } while (!done);

  if (do_nozzle0) {
    tempcon_states.nozzle[0].mode = 0;
  } /* endif */

  if (do_nozzle1) {
    tempcon_states.nozzle[1].mode = 0;
  } /* endif */

  pwm_state.head_fan_requested_duty = temp_f;
  tempcon_states.why_are_we_waiting = NOT_WAITING;
}


// Updates selected nozzle's targets from reel (or if no parameters, both nozzles).
// Note that plumbing is swapped with dual reels.  So, nozzle 0 gets its values from reel 1
// if the head is dual-material, but needs to get them from reel 0 if the head is single-
// material.  Nozzle 1 only gets its value from reel 0.
void get_nozzle_targets_from_reel(char *line) {
  extern t_tempcon_states tempcon_states;
  float                   v;
  unsigned char           valid;

  if (has_code(line, 'S') || !has_code(line, 'T')) {
    if (is_head_dual_material()) {
      tempcon_states.nozzle[0].target[0] = get_eeprom_float(2, 0x30, tempcon_states.nozzle[0].target[0]); // normal; reel 1
      tempcon_states.nozzle[0].target[1] = get_eeprom_float(2, 0x28, tempcon_states.nozzle[0].target[1]); // first layer; reel 1
      valid = is_reel1_present();
    } else {
      tempcon_states.nozzle[0].target[0] = get_eeprom_float(1, 0x30, tempcon_states.nozzle[0].target[0]); // normal; reel 0
      tempcon_states.nozzle[0].target[1] = get_eeprom_float(1, 0x28, tempcon_states.nozzle[0].target[1]); // first layer; reel 0
      valid = is_reel0_present();
    } /* endif */

    // Automaker normally deals with flushing the nozzle when the material changes - but when Automaker isn't connected,
    // the firmware must report an error if a nozzle flush is required
    if (!is_automaker_connected() && valid) {
      v = get_eeprom_float(0, 0xb0, tempcon_states.nozzle[0].target[0]); // nozzle 0 material melt temp from head EEPROM

      if (fabs(v - tempcon_states.nozzle[0].target[0]) > NOZZLE_FLUSH_MARGIN_C) {
        report_error(ERROR_NOZZLE_FLUSH_NEEDED);
      } /* endif */
    } /* endif */
  } /* endif */

  if (has_code(line, 'T') || !has_code(line, 'S')) {
    tempcon_states.nozzle[1].target[0] = get_eeprom_float(1, 0x30, tempcon_states.nozzle[1].target[0]); // normal; reel 0
    tempcon_states.nozzle[1].target[1] = get_eeprom_float(1, 0x28, tempcon_states.nozzle[1].target[1]); // first layer; reel 0

    // Automaker normally deals with flushing the nozzle when the material changes - but when Automaker isn't connected,
    // the firmware must report an error if a nozzle flush is required
    if (!is_automaker_connected() && is_reel0_present() && is_head_dual_material()) {
      v = get_eeprom_float(0, 0xa8, tempcon_states.nozzle[1].target[0]); // nozzle 1 material melt temp from head EEPROM

      if (fabs(v - tempcon_states.nozzle[1].target[0]) > NOZZLE_FLUSH_MARGIN_C) {
        report_error(ERROR_NOZZLE_FLUSH_NEEDED);
      } /* endif */
    } /* endif */
  } /* endif */
}


// If there are two reels present, the bed targets are set to the higher of the two reels' values.
void get_bed_targets_from_reel(void) {
  extern t_tempcon_states tempcon_states;

  tempcon_states.bed.target[0] = get_eeprom_float(1, 0x40, tempcon_states.bed.target[0]); // normal, reel 0
  tempcon_states.bed.target[0] = max(tempcon_states.bed.target[0], get_eeprom_float(2, 0x40, tempcon_states.bed.target[0])); // normal, reel 1
  tempcon_states.bed.target[0] = min(tempcon_states.bed.target[0], MAX_BED_TARGET);

  tempcon_states.bed.target[1] = get_eeprom_float(1, 0x38, tempcon_states.bed.target[1]); // first layer, reel 0
  tempcon_states.bed.target[1] = max(tempcon_states.bed.target[1], get_eeprom_float(2, 0x38, tempcon_states.bed.target[1])); // first layer, reel 1
  tempcon_states.bed.target[1] = min(tempcon_states.bed.target[1], MAX_BED_TARGET);
}


// If there are two reels present, the ambient target is set to the higher of the two reels' values.
void get_ambient_target_from_reel(void) {
  extern t_tempcon_states tempcon_states;

  tempcon_states.ambient.target[0] = get_eeprom_float(1, 0x48, tempcon_states.ambient.target[0]); // reel 0
  tempcon_states.ambient.target[0] = max(tempcon_states.ambient.target[0], get_eeprom_float(2, 0x48, tempcon_states.ambient.target[0])); // reel 1
  tempcon_states.ambient.target[0] = min(tempcon_states.ambient.target[0], MAX_AMBIENT_TARGET);
  tempcon_states.ambient.target[1] = tempcon_states.ambient.target[0];
}


// Reads temperature controller targets from the reel EEPROM.  Note the use of the default parameter of
// get_eeprom_float(), to ensure the correct behaviour if the EEPROM has become unreadable.
// Note that bed targets are not updated if a job is in progress.  The rationale for this is that the bed
// temperature needs to suit the material which has already been printed, and should not change if a
// different material is introduced part way through the job.
void get_tempcon_targets_from_reel(void) {
  extern t_tempcon_states tempcon_states;

  if (!is_job_running()) {
    get_bed_targets_from_reel();
  } /* endif */

  get_nozzle_targets_from_reel("S T");
  get_ambient_target_from_reel();
}


// Implements the set_temperatures Robox command.
void set_tempcon_targets(unsigned char *d) {
  extern t_tempcon_states tempcon_states;
  char                    b[9];

  b[8] = 0;
  memcpy(b, &d[0], 8);
  tempcon_states.nozzle[0].target[0] = strtof(b, NULL);
  memcpy(b, &d[8], 8);
  tempcon_states.nozzle[0].target[1] = strtof(b, NULL);
  memcpy(b, &d[16], 8);
  tempcon_states.nozzle[1].target[0] = strtof(b, NULL);
  memcpy(b, &d[24], 8);
  tempcon_states.nozzle[1].target[1] = strtof(b, NULL);
  memcpy(b, &d[32], 8);
  tempcon_states.bed.target[0] = min(strtof(b, NULL), MAX_BED_TARGET); // normal
  memcpy(b, &d[40], 8);
  tempcon_states.bed.target[1] = min(strtof(b, NULL), MAX_BED_TARGET); // first layer
  memcpy(b, &d[48], 8);
  tempcon_states.ambient.target[0] = tempcon_states.ambient.target[1] = min(strtof(b, NULL), MAX_AMBIENT_TARGET);
}


// NB: does not wait if the heater is turned off, or if the head power has been turned off.
// The wait is abandoned in the event of an abort.  If ignore_pause is false, the wait is
// also abandoned in the event of a pause.
// When the head is dual-material, waits for the selected nozzle only.
void wait_until_nozzle_is_at_temperature(unsigned char ignore_pause) {
  extern t_tempcon_states tempcon_states;
  unsigned char nozzle;
  float  target, v;

  tempcon_states.why_are_we_waiting = NOZZLE_AT_TEMPERATURE;
  nozzle = is_head_dual_material() ? get_selected_tool() : 0;

  do {
    do_everything_else();
    v = 0;

    if ((!is_paused() || ignore_pause) && !is_abort() && is_head_power_on() && (tempcon_states.nozzle[nozzle].mode != 0)) {
      // mode=1 -> use target[0]; mode=2 -> use target[1]; mode=3 -> UNLOAD_NOZZLE_TEMPERATURE
      target = (tempcon_states.nozzle[nozzle].mode == 1) ? tempcon_states.nozzle[nozzle].target[0] : ((tempcon_states.nozzle[nozzle].mode == 2) ? tempcon_states.nozzle[nozzle].target[1] : UNLOAD_NOZZLE_TEMPERATURE);
      v = fabs(target - tempcon_states.nozzle[nozzle].measured) / max(1.0, target);
    } /* endif */
  } while (v > NOZZLE_TEMP_MARGIN);

  tempcon_states.why_are_we_waiting = NOT_WAITING;
}


// Does not wait if the heater is turned off, or if the target temperature is zero.
// The wait is abandoned in the event of an abort.  If ignore_pause is false, the wait
// is also abandoned in the event of a pause.
// If S or T parameters present, we stop waiting when it's time to start heating
// nozzle0 (S) or nozzle1 (T), with the objective that the bed and nozzle(s) reach
// their target temperatures at about the same time.
// If specified, P parameter causes the wait to end a specified number of secs early.
void wait_until_bed_is_at_temperature(char line[], unsigned char ignore_pause) {
  const char              PCHARS[] = "ST";
  extern t_tempcon_states tempcon_states;
  float                   bederr, v, tbed;
  unsigned char           i, done = 0;

  tempcon_states.why_are_we_waiting = BED_AT_TEMPERATURE;

  do {
    do_everything_else();

    if (tempcon_states.bed.mode > 0) {
      bederr = tempcon_states.bed.target[tempcon_states.bed.mode - 1] - tempcon_states.bed.measured;

      if (bederr > 0.0) {
        tbed = time_to_target(tempcon_states.bed.target[tempcon_states.bed.mode - 1], tempcon_states.bed.measured, tempcon_states.ambient.measured, BED_FINAL_DELTA, BED_TCONST);

        if (has_float(line, 'P')) {
          tbed -= get_float(line, 'P');
          done = done || (tbed < 0.0);
        } /* endif */

        for (i = 0; i < 2; i ++) {
          if (has_code(line, PCHARS[i])) {
            if (has_float(line, PCHARS[i])) {
              v = get_float(line, PCHARS[i]);
            } else {
              v = tempcon_states.nozzle[i].target[tempcon_states.bed.mode - 1]; // 1st layer selection assumed to be same as bed
            } /* endif */

            if (is_head_dual_material()) {
              v = time_to_target(v, tempcon_states.nozzle[i].measured, tempcon_states.ambient.measured, DUAL_NOZZLE_FINAL_DELTA, DUAL_NOZZLE_TCONST);
            } else if (does_head_have_prtd()) {
              v = time_to_target(v, tempcon_states.nozzle[i].measured, tempcon_states.ambient.measured, HIGH_TEMP_NOZZLE_FINAL_DELTA, HIGH_TEMP_NOZZLE_TCONST);
            } else {
              v = time_to_target(v, tempcon_states.nozzle[i].measured, tempcon_states.ambient.measured, SINGLE_NOZZLE_FINAL_DELTA, SINGLE_NOZZLE_TCONST);
            } /* endif */

            done = done || (v >= tbed); // it's time to start heating nozzle
          } /* endif */
        } /* endfor */
      } /* endif */

      done = done || (tempcon_states.bed.target[tempcon_states.bed.mode - 1] <= 0.0); // don't wait if target <= 0
      bederr = fabs(bederr / max(1.0, tempcon_states.bed.target[tempcon_states.bed.mode - 1])); // error as a proportion of target
      done = done || (bederr <= BED_TEMP_MARGIN); // bed has reached temperature
      done = done || (is_paused() && !ignore_pause);
      done = done || is_abort();
    } else {
      done = 1;
    } /* endif */
  } while (!done);

  tempcon_states.why_are_we_waiting = NOT_WAITING;
}


// For the purpose of cover interlock.
void wait_until_bed_is_cool(void) {
  extern t_tempcon_states tempcon_states;

  tempcon_states.why_are_we_waiting = BED_COOLING;

  while (!is_abort() && (tempcon_states.bed.measured >= BED_HOT_TEMPERATURE)) {
    do_everything_else();
  } /* endwhile */

  tempcon_states.why_are_we_waiting = NOT_WAITING;
}


// Indicates whether the nozzles are "hot", for the purposes of logging head hours.  If nozzle 1
// isn't present, this is OK as it will cause a low temperature to be measured.
unsigned char is_head_hot(void) {
  extern t_tempcon_states tempcon_states;
  return((tempcon_states.nozzle[0].measured >= HEAD_HOT_TEMPERATURE) || (tempcon_states.nozzle[1].measured >= HEAD_HOT_TEMPERATURE));
}


void set_nozzle_temperature(char *line, unsigned char first_layer) {
  extern t_tempcon_states tempcon_states;

  if (!has_code(line, 'S') && !has_code(line, 'T')) { // no parameters
    tempcon_states.nozzle[0].mode = tempcon_states.nozzle_mode_at_pause[0] = first_layer + 1; // ; turn on nozzle0 heater using present target

    if (is_head_dual_material()) {
      tempcon_states.nozzle[1].mode = tempcon_states.nozzle_mode_at_pause[1] = first_layer + 1; // ; if dual-material head, then turn on nozzle1 heater too
    } /* endif */
  } else {
    if (has_code(line, 'S')) {
      if (!has_float(line, 'S')) { // just 'S'; turn on nozzle0 heater only, using present target
        tempcon_states.nozzle[0].mode = tempcon_states.nozzle_mode_at_pause[0] = first_layer + 1;
      } else if (get_float(line, 'S') > 0.0) { // 'S' followed by a number > 0
        tempcon_states.nozzle[0].target[first_layer] = get_float(line, 'S');
        tempcon_states.nozzle[0].mode = tempcon_states.nozzle_mode_at_pause[0] = first_layer + 1;
      } else { // zero or -ve means "turn off heater"; in this case we want to retain last target
        tempcon_states.nozzle[0].mode = tempcon_states.nozzle_mode_at_pause[0] = 0;
      } /* endif */
    } /* endif */

    if (has_code(line, 'T')) {
      if (!has_float(line, 'T')) { // just 'T'; turn on nozzle1 heater only, using present target
        tempcon_states.nozzle[1].mode = tempcon_states.nozzle_mode_at_pause[1] = first_layer + 1;
      } else if (get_float(line, 'T') > 0.0) { // 'T' followed by a number > 0
        tempcon_states.nozzle[1].target[first_layer] = get_float(line, 'T');
        tempcon_states.nozzle[1].mode = tempcon_states.nozzle_mode_at_pause[1] = first_layer + 1;
      } else { // zero or -ve means "turn off heater"; in this case we want to retain last target
        tempcon_states.nozzle[1].mode = tempcon_states.nozzle_mode_at_pause[1] = 0;
      } /* endif */
    } /* endif */
  } /* endif */

  if (tempcon_states.nozzle[0].mode || tempcon_states.nozzle[1].mode) {
    set_head_power(1);
  } /* endif */
}


void set_bed_temperature(char *line, unsigned char first_layer) {
  extern t_tempcon_states tempcon_states;

  if (has_code(line, 'D')) {
    tempcon_states.bed.target[first_layer] = min(MAX_BED_TARGET, get_eeprom_float(2, (first_layer ? 0x38 : 0x40), tempcon_states.bed.target[first_layer])); // reel 1
  } /* endif */

  if (has_code(line, 'E')) {
    tempcon_states.bed.target[first_layer] = min(MAX_BED_TARGET, get_eeprom_float(1, (first_layer ? 0x38 : 0x40), tempcon_states.bed.target[first_layer])); // reel 0
  } /* endif */

  if (has_code(line, 'S')) {
    if (get_float(line, 'S') > 0.0) {
      tempcon_states.bed.target[first_layer] = min(MAX_BED_TARGET, get_float(line, 'S'));
      tempcon_states.bed.mode = first_layer + 1;
    } else { // zero or -ve means "turn off heater"; in this case we want to retain last target
      tempcon_states.bed.mode = 0;
    } /* endif */
  } else if (tempcon_states.bed.target[first_layer] > 0.0) {
    tempcon_states.bed.mode = first_layer + 1; // if no parameter, just turn on using present target
  } else {
    tempcon_states.bed.mode = 0;
  } /* endif */
}


void set_ambient_temperature(char *line) {
  extern t_tempcon_states tempcon_states;

  if (has_code(line, 'D')) {
  tempcon_states.ambient.target[0] = min(MAX_AMBIENT_TARGET, get_eeprom_float(2, 0x48, tempcon_states.ambient.target[0])); // reel 1
  } /* endif */

  if (has_code(line, 'E')) {
  tempcon_states.ambient.target[0] = min(MAX_AMBIENT_TARGET, get_eeprom_float(1, 0x48, tempcon_states.ambient.target[0])); // reel 0
  } /* endif */

  if (has_code(line, 'S')) {
    if (get_float(line, 'S') > 0.0) {
      tempcon_states.ambient.target[0] = min(MAX_AMBIENT_TARGET, get_float(line, 'S'));
      tempcon_states.ambient.mode = 1;
    } else { // zero or -ve means "turn off heater"; in this case we want to retain last target
      tempcon_states.ambient.mode = 0;
    } /* endif */
  } else {
    tempcon_states.ambient.mode = 1; // if no parameter, just turn on using present target
  } /* endif */
}


// Global hiding routine.
void force_ambient_fan(unsigned char on) {
  extern t_tempcon_states tempcon_states;
  tempcon_states.force_ambient_fan = on;
}


void set_head_fan_pwm_duty(unsigned char duty) {
  extern t_pwm_state pwm_state;

  if (duty > 0) {
    set_head_power(1);
  } /* endif */

  pwm_state.head_fan_requested_duty = duty;
}


void set_nozzle_parameters(char *line) {
  extern parameter_struct pa;
  set_temperature_parameter_handler(line, &pa.nozzle_tempcon);
}


void set_bed_parameters(char *line) {
  extern parameter_struct pa;
  set_temperature_parameter_handler(line, &pa.bed_tempcon);
}


void set_ambient_parameters(char *line) {
  extern parameter_struct pa;
  set_temperature_parameter_handler(line, &pa.ambient_tempcon);
}


void set_prtd_parameters(char *line) {
  extern parameter_struct pa;

  if (has_code(line, 'A')) {
    pa.nozzle_tempcon.prtd_a = get_float(line, 'A');
  } /* endif */

  if (has_code(line, 'B')) {
    pa.nozzle_tempcon.prtd_b = get_float(line, 'B');
  } /* endif */

  if (has_code(line, 'T')) {
    pa.nozzle_tempcon.prtd_tcal = get_float(line, 'T');
  } /* endif */
}


// Global hiding routine
void set_ambient_led_colour(unsigned char r, unsigned char g, unsigned char b) {
  extern t_pwm_state pwm_state;

  if ((r != pwm_state.led_duties[0]) || (g != pwm_state.led_duties[1]) || (b != pwm_state.led_duties[2])) {
    pwm_state.led_rates[0] = (signed short)r - (signed short)(pwm_state.led_duties_ramped[0] >> 8);
    pwm_state.led_rates[1] = (signed short)g - (signed short)(pwm_state.led_duties_ramped[1] >> 8);
    pwm_state.led_rates[2] = (signed short)b - (signed short)(pwm_state.led_duties_ramped[2] >> 8);
    pwm_state.led_duties[0] = r;
    pwm_state.led_duties[1] = g;
    pwm_state.led_duties[2] = b;
  } /* endif */
}


// Global hiding routine
void set_button_led_colour(unsigned char r, unsigned char g, unsigned char b, unsigned char no_ramp) {
  extern t_pwm_state pwm_state;

  if ((r != pwm_state.led_duties[3]) || (g != pwm_state.led_duties[4]) || (b != pwm_state.led_duties[5])) {
    pwm_state.led_rates[3] = (signed short)r - (signed short)(pwm_state.led_duties_ramped[3] >> 8);
    pwm_state.led_rates[4] = (signed short)g - (signed short)(pwm_state.led_duties_ramped[4] >> 8);
    pwm_state.led_rates[5] = (signed short)b - (signed short)(pwm_state.led_duties_ramped[5] >> 8);
    pwm_state.led_duties[3] = r;
    pwm_state.led_duties[4] = g;
    pwm_state.led_duties[5] = b;
  } /* endif */

  if (no_ramp) {
    pwm_state.led_duties_ramped[3] = ((unsigned short)r) << 8;
    pwm_state.led_duties_ramped[4] = ((unsigned short)g) << 8;
    pwm_state.led_duties_ramped[5] = ((unsigned short)b) << 8;
  } /* endif */
}


// Returns response to M105 (show temperatures).
void show_temperatures(char *response) {
  extern t_tempcon_states tempcon_states;
  extern t_pwm_state      pwm_state;
  sprintf(response, "S:%d @%u T:%d @%u B:%d %c%u A:%d *%u\r\n",
          (int)tempcon_states.nozzle[0].measured, tempcon_states.nozzle[0].pwm_duty,
          (int)tempcon_states.nozzle[1].measured, tempcon_states.nozzle[1].pwm_duty,
          (int)tempcon_states.bed.measured, !pwm_state.is_230v ? '$' : '^', !pwm_state.is_230v ? tempcon_states.bed.pwm_duty : (tempcon_states.bed.pwm_duty >> 2),
          (int)tempcon_states.ambient.measured, tempcon_states.ambient.pwm_duty);
}


// Returns temperature controller info which forms part of the Robox status report.
void get_tempcon_states(unsigned char *d) {
  extern t_tempcon_states tempcon_states;
  extern t_pwm_state      pwm_state;

  memset(d, 0, 94);
  d[0] = tempcon_states.nozzle[0].mode + '0';
  sprintf((char *)&d[1], "%f", tempcon_states.nozzle[0].measured);
  sprintf((char *)&d[9], "%f", (float)tempcon_states.nozzle[0].target[0]);
  sprintf((char *)&d[17], "%f", (float)tempcon_states.nozzle[0].target[1]);
  d[25] = tempcon_states.nozzle[1].mode + '0';
  sprintf((char *)&d[26], "%f", tempcon_states.nozzle[1].measured);
  sprintf((char *)&d[34], "%f", (float)tempcon_states.nozzle[1].target[0]);
  sprintf((char *)&d[42], "%f", (float)tempcon_states.nozzle[1].target[1]);
  d[50] = tempcon_states.bed.mode + '0';
  sprintf((char *)&d[51], "%f", tempcon_states.bed.measured);
  sprintf((char *)&d[59], "%f", (float)tempcon_states.bed.target[0]);
  sprintf((char *)&d[67], "%f", (float)tempcon_states.bed.target[1]);
  d[75] = tempcon_states.ambient.mode + '0';
  sprintf((char *)&d[76], "%f", tempcon_states.ambient.measured);
  sprintf((char *)&d[84], "%f", (float)tempcon_states.ambient.target[0]);
  d[92] = (pwm_state.head_fan_requested_duty > 0) ? '1' : '0';
  d[93] = tempcon_states.why_are_we_waiting + '0';
}


// returns head power info which forms part of the Robox status report.
void get_head_power_state(unsigned char *d) {
  d[0] = PIO_Get(&HEAD_POWER[is_rev1_hw()]) ? '1' : '0';
}


unsigned char is_head_power_on(void) {
  return(PIO_Get(&HEAD_POWER[is_rev1_hw()]));
}


void shut_down_head_due_to_eeprom(void) {
  if (is_head_power_on()) {
    report_error(ERROR_HEAD_POWER_EEPROM);
  } /* endif */

  set_head_power(0);
}


// This is a ghastly work-around for a hardware problem.  It allows the nozzle heaters to be forced
// off when the B axis is going to be moved.
void force_nozzles_off(unsigned char force) {
  extern t_tempcon_states tempcon_states;

  force = force && tempcon_states.enable_nozzle_forcing;
  tempcon_states.force_nozzles_off = force;

  if (force) {
    set_nozzle0_heater(0);
    set_nozzle1_heater(0);
  } /* endif */
}


// The ghastly work-around is not enabled by default, but this routine is used to enable it
// if errors arise which suggest that the work-around is needed.
void enable_nozzle_forcing(void) {
  extern t_tempcon_states tempcon_states;
  tempcon_states.enable_nozzle_forcing = 1;
}


unsigned char is_nozzle_forcing_enabled(void) {
  extern t_tempcon_states tempcon_states;
  return(tempcon_states.enable_nozzle_forcing);
}


void check_for_poweroff_whilst_hot(void) {
  unsigned char d;

  if (read_nonvolatile_data(0, &d)) { // silently do nothing if no SD card
    if (d) {
      report_error(ERROR_POWEROFF_WHILST_HOT);
    } /* endif */
  } /* endif */
}


// Note check of HEAD_POWER_DETECT (post rev 1 hardware only) to avoid turning
// on head power if it's shorted.  The hardware line is read multiple times
// so as to ignore occasional glitches.
void set_head_power(unsigned char p) {
  static unsigned char head_shorted_counter; // static so it can be observed
  unsigned char        i;
  
  head_shorted_counter = 0;
  
  if (!is_rev1_hw()) {
    if (p) {
      for (i = 0; i < 255; i ++) {
        head_shorted_counter += PIO_Get(&HEAD_POWER_DETECT_R2);
      } /* endfor */
      
      if (head_shorted_counter < 128) {
        report_error(ERROR_HEAD_SHORTED);
        p = 0;
      } /* endif */
    } /* endif */
  } /* endif */

  if (p) {
    PIO_Set(&HEAD_POWER[is_rev1_hw()]);
  } else {
    PIO_Clear(&HEAD_POWER[is_rev1_hw()]);
  } /* endif */
}


// structs to store state of temperature controllers
t_tempcon_states tempcon_states;
t_pwm_state      pwm_state;
