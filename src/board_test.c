#include <board.h>
#include <pio/pio.h>
#include <string.h>
#include <pio/pio.h>
#include <adc/adc12.h>
#include <stdio.h>
#include "motion_hal.h"
#include "util.h"
#include "sd_mmc.h"
#include "command.h"

#define ALLOWED_CURRENT_ERROR  0.2 // +/- 20%
#define CURRENT_LSB            (3.3 * 1047 / (1000 * 4095 * 0.264)) // LSB of current measurement via test fixture; 0.264 from ACS722 data sheet; 47R, 1K divide ratio; 3.3V ADC full scale
#define CURRENT_FIDDLE_FACTOR  (1.6 / 2.0) // historically, a setting of 2A has always given approximately 1.6A
#define CURRENT_SETTLING_TIME  2.0e-3
#define TEST_MICROSTEPS        64
#define EXTRUDER_FULL_SCALE    (4095 * 0.812 * 1000 / (3.3 * 1047)) // ADC measurement for full-scale digital pot (extruder current setting outputs)
#define ALLOWED_EXTRUDER_ERROR (0.1 * EXTRUDER_FULL_SCALE)
#define PORT_SETTLING_TIME     2.0e-3

void send_ad5206(unsigned short d);

typedef struct {
  Pin           a;
  Pin           b;
  unsigned char type;
} t_port_pair;

typedef struct {
  char             errors[256];
  unsigned char    fail;
} t_board_test_state;


#define N_PORT_PAIRS (sizeof(PORT_PAIRS) / sizeof(t_port_pair))

const t_port_pair PORT_PAIRS[] = {
  {{1 << 10, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_DEFAULT},    {1 << 24, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT}, 0}, // HEAD_LOAD_A/B
  {{1 << 9,  AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_DEFAULT},    {1 << 31, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT}, 0}, // HEAD_SDA_A/B
  {{1 << 15, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT},    {1 << 12, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_DEFAULT}, 0}, // NOZZLE_HOME_SWITCH_A/B
  {{1 << 29, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT},    {1 << 0,  AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT}, 0}, // HEAD_MOTOR_STEP_A/B
  {{1 << 25, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT},    {1 << 11, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT}, 0}, // HEAD_MOTOR_DIR_A/B
  {{1 << 17, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_DEFAULT},    {1 << 13, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT}, 0}, // HEAD_SCL_A/B
  {{1 << 16, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT},    {1 << 23, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_DEFAULT}, 0}, // X_END_STOP_A/B
  {{1 << 12, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_DEFAULT},    {1 << 27, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT}, 0}, // Z_STOP_A/B
  {{1 << 26, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT},    {1 << 21, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT}, 0}, // EXTRUDER_MOTOR_DIR_E1/2
  {{1 << 21, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_DEFAULT},    {1 << 14, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_DEFAULT}, 0}, // EXTRUDER_MOTOR_STEP_E1/2
  {{1 << 30, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT},    {1 << 5,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_DEFAULT}, 0}, // EXTRUDER_MOTOR_N_ENABLE_E1/2
  {{1 << 14, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT},    {1 << 23, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT}, 0}, // EXTRUDER_INDEX_E1/2
  {{1 << 17, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT},    {1 << 21, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_DEFAULT}, 0}, // EXTRUDER_SWITCH_E1/2
  {{1 << 29, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_DEFAULT},    {1 << 7,  AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT}, 0}, // Z+_STOP/CASE_OPEN
  {{1 << 28, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}, {1 << 6,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_DEFAULT}, 1}, // ELECTRONICS_COOLING_FAN/LID_LATCH_DETECT
  {{1 << 18, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}, {1 << 24, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_DEFAULT}, 1}, // AMBIENT_FAN/Y_STOP
  {{1 << 22, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}, {1 << 25, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP},  1}, // LED_EJECT_R/REEL_EEPROM_SCL
  {{1 << 29, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}, {1 << 24, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP},  1}, // LED_EJECT_G/REEL_EEPROM_SDA
  {{1 << 28, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}, {1 << 30, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP},  1}, // LED_EJECT_B/EJECT_SWITCH
  {{1 << 2,  AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}, {1 << 17, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_DEFAULT}, 1}, // LED_AMB_R/THERMISTOR_AMB_HEAD_A
  {{1 << 1,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}, {1 << 15, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_DEFAULT}, 1}, // LED_AMB_G/THERMISTOR_AMB_HEAD_B
  {{1 << 0,  AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}, {1 << 3,  AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT}, 1}, // LED_AMB_B/AMBIENT_THERMISTOR_AUX
};

typedef struct {
  Pin           nen;
  Pin           step;
  Pin           dir;
} t_motor_port;

#define N_MOTOR_DRIVERS (sizeof(MOTOR_PORTS) / sizeof(t_motor_port))

const t_motor_port MOTOR_PORTS[] = {
  {{1 << 31, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT}, {1 << 28, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}, {1 << 8,  AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}}, // X
  {{1 << 2,  AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT}, {1 << 23, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}, {1 << 31, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}}, // Y
  {{1 << 13, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT}, {1 << 27, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}, {1 << 27, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}}, // Z1
  {{1 << 20, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT}, {1 << 7,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}, {1 << 8,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}}  // Z2
};

const Pin BED_DETECT   = {1 << 30, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_DEFAULT};    // normally bed thermistor input
const Pin RESULT_PASS  = {1 << 29, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}; // green LED on test fixture
const Pin RESULT_FAIL  = {1 << 22, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}; // red LED on test fixture
const Pin HEAD_POWER_L = {1 << 18, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT};
const Pin BED_AC_HEAT  = {1 << 11, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT};


// Log a test error.
void test_error(unsigned short n) {
  extern t_board_test_state board_test_state;

  board_test_state.errors[n] = '1';
  board_test_state.fail = 1;
}


// Make averaged ADC measurements.
static void measure(signed long *a, signed long *b, signed long *e, signed long *d) {
  unsigned char i;

  *a = *b = *e = *d = 0;

  for (i = 0; i < 32; i ++) {
    ADC12_StartConversion(AT91C_BASE_ADC12B);

    while ((ADC12_GetStatus(AT91C_BASE_ADC12B) & 0xff) != (ADC12_GetChannelStatus(AT91C_BASE_ADC12B) & 0xff)) {
    } /* endwhile */

    *a += ADC12_GetConvertedData(AT91C_BASE_ADC12B, ADC12_CHANNEL_3); // motor a measurement on thermistor 1a
    *b += ADC12_GetConvertedData(AT91C_BASE_ADC12B, ADC12_CHANNEL_5); // motor b measurement on thermistor 2a
    *e += ADC12_GetConvertedData(AT91C_BASE_ADC12B, ADC12_CHANNEL_0); // ref e on thermistor 1b
    *d += ADC12_GetConvertedData(AT91C_BASE_ADC12B, ADC12_CHANNEL_7); // ref d on thermistor 2b
  } /* endfor */

  *a = *a >> 5;
  *b = *b >> 5;
  *e = *e >> 5;
  *d = *d >> 5;
}


// Checks two pairs of ports linked by the test fixture, but not linked in normal use.
// If either pair of ports is linked, we conclude that we must be connected to the test
// fixture, in which case we wait for the 24V power to come up then return true.
// Otherwise, we return false straight away.
static unsigned char is_tester_connected(void) {
  extern const Pin VMOT_PIN;
  static const Pin INS[] =  {{1 << 25, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_PULLUP}, {1 << 26, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_PULLUP}};
  static const Pin OUTS[] = {{1 << 11, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}, {1 << 21, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}};
  unsigned char i, yes;

  PIO_Configure(&VMOT_PIN, 1);
  PIO_Configure(&RESULT_PASS, 1);
  PIO_Configure(&RESULT_FAIL, 1);
  yes = 0;

  for (i = 0; i < 2; i ++) {
    PIO_Configure(&INS[i], 1);
    PIO_Configure(&OUTS[i], 1);
    PIO_Clear(&OUTS[i]);

    if (PIO_Get(&INS[i]) == 0) {
      PIO_Set(&OUTS[i]);

      if (PIO_Get(&INS[i]) == 1) {
        yes = 1;
      } /* endif */
    } /* endif */
  } /* endfor */

  if (yes) {
    while (!PIO_Get(&VMOT_PIN)) {
    } /* endwhile */

    return(1);
  } /* endif */

  return(0);
}


// Performs a walking 1 test and a walking 0 test on the port pairs.
static void test_ports(void) {
  unsigned char i, dir, d, p;

  for (i = 0; i < N_PORT_PAIRS; i ++) {
    PIO_Configure(&PORT_PAIRS[i].a, 1);
    PIO_Configure(&PORT_PAIRS[i].b, 1);
  } /* endfor */

  for (dir = 0; dir < 2; dir ++) {
    for (d = 0; d < 2; d ++) {
      // drive motor step and direction outputs to maximise the chances of spotting shorts to other I/Os; however do not enable motor drivers
      for (i = 0; i < N_MOTOR_DRIVERS; i ++) {
        if (d) {
          PIO_Clear(&MOTOR_PORTS[i].step);
          PIO_Clear(&MOTOR_PORTS[i].dir);
        } else {
          PIO_Set(&MOTOR_PORTS[i].step);
          PIO_Set(&MOTOR_PORTS[i].dir);
        } /* endif */
      } /* endfor */

      for (i = 0; i < N_PORT_PAIRS; i ++) {
        for (p = 0; p < N_PORT_PAIRS; p ++) {
          if (dir && (PORT_PAIRS[p].type == 0)) {
            PORT_PAIRS[p].a.pio->PIO_ODR = PORT_PAIRS[p].a.mask; // make input
            PORT_PAIRS[p].b.pio->PIO_OER = PORT_PAIRS[p].b.mask; // make output

            if (d == (p == i)) {
              PORT_PAIRS[p].b.pio->PIO_SODR = PORT_PAIRS[p].b.mask; // output 1
            } else {
              PORT_PAIRS[p].b.pio->PIO_CODR = PORT_PAIRS[p].b.mask; // output 0
            } /* endif */
          } else {
            PORT_PAIRS[p].b.pio->PIO_ODR = PORT_PAIRS[p].b.mask; // make input
            PORT_PAIRS[p].a.pio->PIO_OER = PORT_PAIRS[p].a.mask; // make output

            if (d == (p == i)) {
              PORT_PAIRS[p].a.pio->PIO_SODR = PORT_PAIRS[p].a.mask; // output 1
            } else {
              PORT_PAIRS[p].a.pio->PIO_CODR = PORT_PAIRS[p].a.mask; // output 0
            } /* endif */
          } /* endif */
        } /* endfor */

        delay(PORT_SETTLING_TIME);

        for (p = 0; p < N_PORT_PAIRS; p ++) {
          if (PORT_PAIRS[p].type == 0) {
            if (dir) {
              if (PIO_Get(&PORT_PAIRS[p].a) != (d == (p == i))) {
                test_error(p);
              } /* endif */
            } else {
              if (PIO_Get(&PORT_PAIRS[p].b) != (d == (p == i))) {
                test_error(p);
              } /* endif */
            } /* endif */
          } else {
            if (PIO_Get(&PORT_PAIRS[p].b) == (d == (p == i))) {
              test_error(p);
            } /* endif */
          } /* endif */
        } /* endfor */
      } /* endfor */
    } /* endfor */
  } /* endfor */

  for (i = 0; i < N_PORT_PAIRS; i ++) {
    PIO_Configure(&PORT_PAIRS[i].a, 1);
    PIO_Configure(&PORT_PAIRS[i].b, 1);
  } /* endfor */
}


// Sets the current, and microsteps the motor driver selected by axis, with the other
// motor drivers not enabled.  For each microstep, measures the A and B phase currents,
// which the test fixture provides at ADC inputs normally used for thermistors.
// We check the sum of the squares of the currents, which should remain constant.
// Also determines the step state, and counts the state changes to verify that the
// driver is being stepped, and stepped in the correct direction.
// A desirable feature of this test is that it doesn't need to know the initial state
// of the motor driver, which is undefined as there's no access to its reset signal.
static void test_motor_driver(unsigned char axis, unsigned char reverse, float current) {
  static const unsigned short AD5206_CHS[] =  {3 << 8, 1 << 8, 4 << 8, 0 << 8}; // channel numbers in AD5206 digital potentiometer IC
  unsigned long               sum_sq, sum_sq_min, sum_sq_max;
  signed long                 a_current, b_current, a_zero, b_zero, trash0, trash1;
  unsigned char               i;
  signed short                start_count, count, state;

  start_count = count = 0; // has no effect, other than to suppress unwanted compiler warning

  for (i = 0; i < N_MOTOR_DRIVERS; i ++) {
    PIO_Set(&MOTOR_PORTS[i].nen);
  } /* endfor */

  PIO_Clear(&MOTOR_PORTS[axis].step);

  if (reverse) {
    PIO_Set(&MOTOR_PORTS[axis].dir);
  } else {
    PIO_Clear(&MOTOR_PORTS[axis].dir);
  } /* endif */

  send_ad5206(AD5206_CHS[axis] | amps_to_dac_r1(current));
  current *= CURRENT_FIDDLE_FACTOR;
  sum_sq_min = (unsigned long)(current * current * (1.0 - ALLOWED_CURRENT_ERROR) * (1.0 - ALLOWED_CURRENT_ERROR) / (CURRENT_LSB * CURRENT_LSB));
  sum_sq_max = (unsigned long)(current * current * (1.0 + ALLOWED_CURRENT_ERROR) * (1.0 + ALLOWED_CURRENT_ERROR) / (CURRENT_LSB * CURRENT_LSB));

  delay(CURRENT_SETTLING_TIME);
  measure(&a_zero, &b_zero, &trash0, &trash1);
  PIO_Clear(&MOTOR_PORTS[axis].nen);

  for (i = 0; i < TEST_MICROSTEPS; i ++) {
    delay(CURRENT_SETTLING_TIME);
    measure(&a_current, &b_current, &trash0, &trash1);
    a_current -= a_zero;
    b_current -= b_zero;
    sum_sq = (unsigned long)((a_current * a_current) + (b_current * b_current));

    if (sum_sq > sum_sq_max) {
      test_error(0 + 32 + (axis * 8)); // current too high
    } else if (sum_sq < sum_sq_min) {
      test_error(1 + 32 + (axis * 8)); // current too low
    } /* endif */

    state = ((a_current < 0) << 1) | (b_current < 0);
    state = (state & 2) ? (state ^ 1) : state; // convert natural Gray coding to binary

    if (i == 0) {
      start_count = count = state;
    } else if (((state - count) & 3) == 1) {
      count ++;
    } else if (((state - count) & 3) == 3) {
      count --;
    } else if (((state - count) & 3) != 0) { // a jump of 2 Gray states; can't happen when things are working properly
      test_error(2 + 32 + (axis * 8)); // full step skipped
    } /* endif */

    PIO_Set(&MOTOR_PORTS[axis].step);
    delay(1e-6);
    PIO_Clear(&MOTOR_PORTS[axis].step);
  } /* endfor */

  PIO_Set(&MOTOR_PORTS[axis].nen);
  count = (reverse) ? (start_count - count) : (count - start_count);

  if (count < 0) {
    test_error(3 + 32 + (axis * 8)); // bad direction
  } else if ((count < ((TEST_MICROSTEPS / 16) - 1)) || (count > ((TEST_MICROSTEPS / 16) + 1))) {
    test_error(4 + 32 + (axis * 8)); // bad step sequence
  } /* endif */
}


// Sets several digital pot values for the extruder current references, and checks the
// voltages looped back to thermistor inputs by the test fixture.
static void test_extruder_refs(void) {
  static const unsigned short AD5206_CHS[] =  {5 << 8, 2 << 8}; // channel numbers in AD5206 digital potentiometer IC
  unsigned short              e, d;
  signed long                 eadc, dadc, v, trash0, trash1;

  for (e = 0; e < 256; e += 255) {
    for (d = 0; d < 256; d += 255) {
      send_ad5206(AD5206_CHS[0] | e);
      send_ad5206(AD5206_CHS[1] | d);
      delay(CURRENT_SETTLING_TIME);
      measure(&trash0, &trash1, &eadc, &dadc);
      v = ((signed long)EXTRUDER_FULL_SCALE * (signed long)e) / 255;

      if ((eadc - v) > ALLOWED_EXTRUDER_ERROR) {
        test_error(0 + 64); // E1 voltage too high
      } else if ((v - eadc) > ALLOWED_EXTRUDER_ERROR) {
        test_error(1 + 64); // E1 voltage too low
      } /* endif */

      v = ((signed long)EXTRUDER_FULL_SCALE * (signed long)d) / 255;

      if ((dadc - v) > ALLOWED_EXTRUDER_ERROR) {
        test_error(4 + 64); // E2 voltage too high
      } else if ((v - dadc) > ALLOWED_EXTRUDER_ERROR) {
        test_error(5 + 64); // E2 voltage too low
      } /* endif */
    } /* endfor */
  } /* endfor */
}


// The test fixture uses the head power output to drive what would normally be mains live.
// It loops back the AC bed heater output to what would normally be the bed thermistor
// input.  Here, we check them all.
static void test_bed_heater_driver(void) {
  PIO_Configure(&HEAD_POWER_L, 1);
  PIO_Configure(&BED_AC_HEAT, 1);
  PIO_Configure(&BED_DETECT, 1);

  PIO_Set(&BED_AC_HEAT);
  delay(PORT_SETTLING_TIME);

  if (PIO_Get(&BED_DETECT)) {
    test_error(72); // head power won't turn off
  } else {
    PIO_Set(&HEAD_POWER_L);
    delay(PORT_SETTLING_TIME);

    if (!PIO_Get(&BED_DETECT)) {
      test_error(73); // head power / bed AC heat won't turn on
    } else {
      PIO_Clear(&HEAD_POWER_L);
      delay(50e-3); // allow plenty of time for FET to turn off; board measured took about 12ms

      if (PIO_Get(&BED_DETECT)) {
        test_error(74); // head power won't turn off (2)
      } else {
        PIO_Clear(&BED_AC_HEAT);
        delay(PORT_SETTLING_TIME);
        PIO_Set(&HEAD_POWER_L);
        delay(PORT_SETTLING_TIME);

        if (PIO_Get(&BED_DETECT)) {
          test_error(75); // bed AC heat won't turn off
        } /* endif */
      } /* endif */
    } /* endif */
  } /* endif */

  PIO_Clear(&HEAD_POWER_L);
  PIO_Clear(&BED_AC_HEAT);
}


static void test_sd_card(void) {
  extern const Pin SD_CD_PIN;

  PIO_Configure(&SD_CD_PIN, 1);
  delay(1e-3); // give pull-up time to work

  if (PIO_Get(&SD_CD_PIN)) {
    test_error(80); // SD card not detected
  } else if (!sd_mmc_init()) {
    test_error(81); // SD card error
  } /* endif */
}


// Public routines...

// This should be called from main() at start-up.  In normal use, it simply returns.
// However, if it finds that the board test fixture is connected, then the board tests
// are run.  When the tests are finished, the pass or fail LED on the test fixture is
// turned on, then maintain_commands() is called repetitively so that test results may
// be retrieved via USB.  Note that we never return in this case.
void board_test(void) {
  extern const Pin          MOSI, SCK, NCS, MS1, MS2;
  extern t_board_test_state board_test_state;
  float                     current;
  unsigned char             i, reverse;

  board_test_state.fail = 0;
  memset(board_test_state.errors, '0', sizeof(board_test_state.errors));

  if (!is_tester_connected()) {
    return;
  } /* endif */

  PIO_Configure(&HEAD_POWER_L, 1); // undo default of 1 in ResetException(), so it has time to decay before test_bed_heater_driver()
  PIO_Configure(&BED_AC_HEAT, 1);
  PIO_Set(&BED_AC_HEAT); // try to hasten decay of head power, though triac may not trigger until voltage has reduced

  for (i = 0; i < N_MOTOR_DRIVERS; i ++) {
    PIO_Configure(&MOTOR_PORTS[i].nen, 1);
    PIO_Configure(&MOTOR_PORTS[i].step, 1);
    PIO_Configure(&MOTOR_PORTS[i].dir, 1);
  } /* endfor */

  PIO_Configure(&MOSI, 1);
  PIO_Configure(&SCK, 1);
  PIO_Configure(&NCS, 1);
  PIO_Configure(&MS1, 1);
  PIO_Configure(&MS2, 1);
  PIO_Set(&MS1);
  PIO_Set(&MS2);

  ADC12_Initialize(AT91C_BASE_ADC12B,
                   AT91C_ID_ADC12B,
                   AT91C_ADC_TRGEN_DIS,
                   0,
                   AT91C_ADC_SLEEP_NORMAL_MODE,
                   AT91C_ADC_LOWRES_12_BIT,
                   BOARD_MCK,
                   1000000,
                   10,               // start-up time (us)
                   10000);           // sample & hold time (ns)
  ADC12_EnableChannel(AT91C_BASE_ADC12B, ADC12_CHANNEL_0); // PA22; thermistor 1B
  //ADC12_EnableChannel(AT91C_BASE_ADC12B, ADC12_CHANNEL_1); // PA30; bed
  //ADC12_EnableChannel(AT91C_BASE_ADC12B, ADC12_CHANNEL_2); // PB3; ambient (wired)
  ADC12_EnableChannel(AT91C_BASE_ADC12B, ADC12_CHANNEL_3); // PB4; thermistor 1A
  //ADC12_EnableChannel(AT91C_BASE_ADC12B, ADC12_CHANNEL_4); // PC15; amb head B
  ADC12_EnableChannel(AT91C_BASE_ADC12B, ADC12_CHANNEL_5); // PC16; thermistor 2A
  //ADC12_EnableChannel(AT91C_BASE_ADC12B, ADC12_CHANNEL_6); // PC17; amb head A
  ADC12_EnableChannel(AT91C_BASE_ADC12B, ADC12_CHANNEL_7); // PC18; thermistor 2B

  test_ports();

  for (current = 1.0; current <= 2.5; current += 1.0) {
    for (reverse = 0; reverse < 2; reverse ++) {
      for (i = 0; i < N_MOTOR_DRIVERS; i ++) {
        test_motor_driver(i, reverse, current);
      } /* endfor */
    } /* endfor */
  } /* endfor */

  test_extruder_refs();
  test_bed_heater_driver();
  test_sd_card();

  PIO_Configure(&RESULT_PASS, 1);
  PIO_Configure(&RESULT_FAIL, 1);

  if (board_test_state.fail) {
    PIO_Set(&RESULT_FAIL);
    PIO_Clear(&RESULT_PASS);
  } else {
    PIO_Set(&RESULT_PASS);
    PIO_Clear(&RESULT_FAIL);
  } /* endif */

  initialise_commands();

  while (1) {
    maintain_commands(); // allows test results to be retrieved via USB
  } /* endwhile */
}


// Retrieves the board test results for debug info report.
void get_board_test_report(unsigned char *d) {
  extern t_board_test_state board_test_state;
  memcpy((char *)d, board_test_state.errors, sizeof(board_test_state.errors));
}


t_board_test_state board_test_state;
