#include <board.h>
#include <pio/pio.h>
#include <irq/irq.h>
#include <tc/tc.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

#include "util.h"
#include "command.h"
#include "parameters.h"
#include "motion_hal.h"
#include "main.h"
#include "gcode.h"
#include "heaters.h"
#include "motion.h"

const char axis_codes[NUM_AXIS] =       {'X', 'Y', 'Z', 'E', 'D', 'B'};

const Pin LID_SWITCH[] =               {{1 << 29, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_PULLUP},  // post rev 1
                                        {1 << 6,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP}}; // rev 1
const Pin Z_TOP_SWITCH[] =             {{1 << 0,  AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP},  // post rev 1
                                        {1 << 29, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP}}; // rev 1


// Private routines...

// efficient integer square root
static unsigned long isqrt(unsigned long x) {
  unsigned long res = 0;
  unsigned long bit = 0x40000000; // the second-to-top bit is set

  while (bit != 0) {
    if (x >= (res + bit)) {
      x -= res + bit;
      res += bit << 1;
    } /* endif */

    res >>= 1;
    bit >>= 2;
  } /* endwhile */

  return(res);
}


// calculate the previous buffer index
static inline unsigned char prev_buffer_index(unsigned char v) {
  return((v + (BLOCK_BUFFER_SIZE - 1)) & (BLOCK_BUFFER_SIZE - 1));
}


// calculate the next buffer index
static inline unsigned char next_buffer_index(unsigned char v) {
  return((v + 1) & (BLOCK_BUFFER_SIZE - 1));
}


// Returns the state of the switch selected by motion_state.switch_select.  However, a "distance timeout"
// is implemented using motion_state.switch_abandon_counter, and when the counter reaches zero we return
// true regardless of the switch.  We also return true if an abort is in force, or if a filament slip has
// occurred and motion_state.slip_select enables sensitivity to filament slip.
// In normal operation, motion_state.switch_abandon_counter = 0 so we just return 1.
static inline unsigned char check_switches(void) {
  extern t_motion_state      motion_state;

  if (is_abort() || (motion_state.switch_abandon_counter == 0)) {
    return(1);
  } /* endif */

  motion_state.switch_abandon_counter --;

  if (motion_state.slip_select && motion_state.filament_slip) {
    return(1);
  } /* endif */

  return(get_axis_switch(motion_state.switch_select) != motion_state.switch_invert);
}


// Calculates the allowable speed at the end of "this" block, based on what's coming next.
// The speed limit resulting from the change of direction at the junction of this block
// and the next block requires some explanation:
//
// At the block junction, each axis experiences a step change in target speed given by:
//
//   speed * (this_block.inc[axis] - last_block.inc[axis])
//
// Let's call the allowable position error in (micro)steps "e".  We limit the speed at the
// block junction so that no axis over/undershoots by more than e.  The over/undershoot is
// of course temporary and the motor eventually catches up, though e shouldn't exceed a
// full step otherwise step skipping is likely.
//
// Let's call the allowable step change in target axis speed "x", and the acceleration "a".
// For now, assume x is in steps/sec and a is in steps/sec^2.  When the target speed changes
// by x, the motor takes time (x / a) to catch up, and at the moment the speed has caught up,
// the distance will lag by:
//
//   e = 0.5 * x * x / a
//
// Consequently,
//
//   x = sqrt(2 * a * e)
//
// Now we know the allowable step change in axis target speed, we simply choose the axis
// which has the largest change in its inc value, and using this we determine the max
// speed as:
//
//   sqrt(2 * a * e) / abs(this.inc[axis] - next.inc[axis])
//
// ... at least, that's conceptually what happens - in reality there's some scaling to
// account for the actual units of acceleration, speed and inc.
//
static unsigned short calculate_end_speed(t_block *this, t_block *next, unsigned char acceleration, unsigned long max_speed_step) {
  unsigned long end_speed, v;
  unsigned char i;

  // calculate allowable speed entering next block = sqrt((2 * length * acceleration) + (end_speed * end_speed))
  v = min(0x00800000L, next->length); // limit very large length to prevent overflow
  v = (v << 1) * (unsigned long)acceleration;
  v += ((unsigned long)next->end_speed * (unsigned long)next->end_speed) >> 4;
  v = isqrt(v);
  end_speed = min((unsigned long)next->speed, v << 2);

  // calculate allowable speed at junction
  v = 0;

  for (i = 0; i < NUM_AXIS; i ++) {
    v = max(v, (unsigned long)abs(this->inc[i] - next->inc[i])); // find the worst
  } /* endfor */

  if (v > 0) { // if v is zero, there's no implied speed limit (so we avoid divide by zero)
    end_speed = min(end_speed, max_speed_step / v);
  } /* endif */

  end_speed = max(MIN_SPEED, min(MAX_SPEED, end_speed));
  return((unsigned short)end_speed); // can't exceed 0xffff so cast is safe
}


// Monitors the index wheel inputs for the purposes of autoload and slip detection.  Most of the work of slip
// detection is done in the ISR, but we decide whether to report the error here.  motion_state.acc[] is updated
// by the ISR, with a change of +0x10000 representing one microstep forwards.  The ISR uses acc[] to keep track of
// when to issue a step pulse for each axis, but only the low 16 bits are needed for this purpose.  Consequently,
// the ISR can zero the upper 16 bits of acc when it sees an index wheel edge.  So, the upper 16 bits of acc
// constitute a signed count of the number of microsteps since the last index wheel edge.  If it exceeds the
// slip detection threshold, we report an error, but only if the filament switch indicates that a filament is
// present.
// Error reporting is suppressed if motion_state.what_are_we_doing == MOTION_UNTIL_SLIP, as this implies that
// we are expecting the filament to slip.  Note that when motion_state.what_are_we_doing == MOTION_UNTIL_SLIP,
// once we detect a slip we keep zeroing the upper bits of acc so as to avoid the possibility of leaving it on
// the brink of slip detection when normal operation resumes, which could cause a spurious slip error to be
// reported.
static void monitor_index_wheels(void) {
  extern parameter_struct pa;
  extern t_motion_state   motion_state;
  signed long             slip_threshold;
  float                   temp;
  unsigned char           i;

  for (i = 0; i < NUM_EXTRUDERS; i ++) {
    if (get_axis_switch(i + E_AXIS) == pa.motion.invert_switch[i + E_AXIS]) { // if filament not present
      if (!has_period_elapsed(motion_state.filament_holdoff_timer, FILAMENT_HOLDOFF_TIME) || !is_motion_buffer_empty()) {
        motion_state.filament_autoload_counter[i] = 0;
      } else if (get_index_wheel(i) != (motion_state.filament_autoload_counter[i] & 1)) {
        motion_state.filament_autoload_counter[i] ++;
      } /* endif */

      if (motion_state.filament_autoload_counter[i] >= FILAMENT_AUTOLOAD_THRESHOLD) {
        execute_gcode((unsigned char *)(i ? "M120 D\n" : "M120 E\n"), 7); // auto filament load due to index wheel activity
        motion_state.filament_autoload_counter[i] = 0;
      } /* endif */

      __sync_fetch_and_and(&motion_state.acc[i + E_AXIS], 0x0000ffffL); // clear upper 16 bits of acc; doesn't affect generation of step signals
    } else {
      motion_state.filament_autoload_counter[i] = 0;

      if (motion_state.what_are_we_doing == MOTION_UNTIL_SLIP) {
        temp = pa.motion.steps_per_unit[i + E_AXIS] * pa.motion.filament_until_slip_threshold * get_filament_multiplier(i) * 65536.0;
        slip_threshold = (temp < (float)0x7fffffff) ? (signed long)temp : 0x7fffffff; // NB: 0x7fffffff has the effect of turning off slip detection, as a signed long can never exceed it

        if (motion_state.filament_slip) {
          // if we're expecting slip and we've detected it, then keep clearing the acc
          __sync_fetch_and_and(&motion_state.acc[i + E_AXIS], 0x0000ffffL); // clear upper 16 bits of acc; doesn't affect generation of step signals
        } else if (abs((signed long)__sync_fetch_and_or(&motion_state.acc[i + E_AXIS], 0)) > slip_threshold) { // ensure atomic read, as motion_state.acc changes in ISR
          __sync_fetch_and_and(&motion_state.acc[i + E_AXIS], 0x0000ffffL); // clear upper 16 bits of acc; doesn't affect generation of step signals
          motion_state.filament_slip = 1;
        } /* endif */
      } else {
        temp = pa.motion.steps_per_unit[i + E_AXIS] * pa.motion.filament_slip_threshold * get_filament_multiplier(i) * 65536.0;
        slip_threshold = (temp < (float)0x7fffffff) ? (signed long)temp : 0x7fffffff; // NB: 0x7fffffff has the effect of turning off slip detection, as a signed long can never exceed it

        if (abs((signed long)__sync_fetch_and_or(&motion_state.acc[i + E_AXIS], 0)) > slip_threshold) { // ensure atomic read, as motion_state.acc changes in ISR
          __sync_fetch_and_and(&motion_state.acc[i + E_AXIS], 0x0000ffffL); // clear upper 16 bits of acc; doesn't affect generation of step signals
          motion_state.filament_slip = 1;

          if (motion_state.what_are_we_doing == MOTION_LOADING) {
            report_error(i ? ERROR_D_LOAD_SLIP : ERROR_E_LOAD_SLIP);
          } else if (motion_state.what_are_we_doing == MOTION_UNLOADING) {
            report_error(i ? ERROR_D_UNLOAD_SLIP : ERROR_E_UNLOAD_SLIP);
          } else if (motion_state.what_are_we_doing == MOTION_NORMAL) {
            motion_state.filament_slip_error[i] = 1;
          } /* endif */
          // NB: if motion_state.what_are_we_doing == MOTION_UNTIL_SLIP, then slip is expected and we don't want to report an error
        } /* endif */
      } /* endif */
    } /* endif */
  } /* endfor */
}


// Z top switch input is vulnerable to glitching low when in the high (switch open) state, especially on RoboxPro
// where the wiring is long.  This routine provides a deglitched version.
static unsigned char is_z_top_switch_actuated(void) {
  extern t_motion_state   motion_state;
  return(motion_state.z_top_switch_deglitch_counter >= 10);
}


// Each move in the buffer has a check_z_top_switch flag.  If this flag is set, it means that the Z position
// throughout the move is less than the Z position at which the Z top switch should be actuated.  So, if there's
// a move in progress and its check_z_top_switch flag is set, we report an error if the Z top switch is actuated.
static void monitor_z_top_switch(void) {
  extern t_motion_state   motion_state;
  unsigned char           b;

  motion_state.z_top_switch_deglitch_counter = PIO_Get(&Z_TOP_SWITCH[is_rev1_hw()]) ? 0 : (motion_state.z_top_switch_deglitch_counter + (motion_state.z_top_switch_deglitch_counter < 0xff));
  b = motion_state.buffer_out; // take a copy because it may change at any time

  if (b != motion_state.buffer_in) {
    if (motion_state.block[b].check_z_top_switch && is_z_top_switch_actuated()) {
      report_error(ERROR_Z_TOP_SWITCH);
    } /* endif */
  } /* endif */
}


// Checks the block buffer to see which axes are idle for all blocks in the buffer.  make_motor_inactive() is called
// for each idle axis, to set the motor current to its hold value (unless the motor is off, in which case it stays
// that way).
static void manage_motor_currents(void) {
  extern t_motion_state   motion_state;
  unsigned char           i, n;
  unsigned char           is_active[NUM_AXIS];

  for (i = 0; i < NUM_AXIS; i ++) {
    is_active[i] = 0;
  } /* endfor */

  n = motion_state.buffer_out; // be careful; motion_state.buffer_out is volatile

  while (n != motion_state.buffer_in) {
    for (i = 0; i < NUM_AXIS; i ++) {
      is_active[i] = is_active[i] || (motion_state.block[n].inc[i] != 0);
    } /* endfor */

    n = next_buffer_index(n);
  } /* endwhile */

  for (i = 0; i < NUM_AXIS; i ++) {
    if (!is_active[i]) {
      make_motor_inactive(i);
    } /* endif */
  } /* endfor */

  // part of a ghastly work-around for a hardware problem; the nozzle heaters are forced off whilst the B axis is moving
  if (!is_active[B_AXIS]) {
    force_nozzles_off(0);
  } /* endif */
}


// This routine is used to keep the buffer contents to a minimum whilst not limiting the speed.  This is
// desirable as it makes pause and abort more responsive.
// We calculate the total length of buffered moves, including the move which is in progress.  If this length is
// sufficient for deceleraton from the target speed of the last move to a standstill, then the speed is not
// being limited, so we return false indicating that it is not yet time to queue a new move.  Obviously we
// also return false if the buffer is full!  Otherwise (including when the buffer is empty) we return true,
// indicating that a new move should be queued.
// Deceleration distance is:
//
// (speed * speed) / (2 * acceleration)
//
// However, we also add a small time margin to make allowance for the time taken to get the next move queued.
// So, this makes the distance:
//
// ((speed * speed) / (2 * acceleration)) + (time * speed)
//
// For efficiency this is rearranged as:
//
// ((speed / (2 * acceleration)) + time) * speed
//
// In practise some scaling is applied to account for the units of acceleration, speed and distance, and the
// divide by 2 is lumped in with this.
static unsigned char is_buffer_ready_for_more(void) {
  extern parameter_struct    pa;
  extern t_motion_state      motion_state;
  unsigned long              length, v;
  unsigned char              b, n;

  b = motion_state.buffer_out;
  n = (motion_state.buffer_in - b) & (BLOCK_BUFFER_SIZE - 1);

  if (n >= (BLOCK_BUFFER_SIZE - 1)) {
    return(0); // buffer is full, so definitely not ready for more!
  } else if (n == 0) {
    return(1); // buffer is empty, so definitely ready!
  } /* endif */

  length = 0;

  while (n) {
    length += motion_state.block[b].length;
    b = next_buffer_index(b);
    n --;
  } /* endwhile */

  v = (unsigned long)motion_state.block[prev_buffer_index(motion_state.buffer_in)].speed;
  v /= (unsigned long)pa.motion.acceleration;
  v += (unsigned long)(32.0 * SPEED_UNITS * 0.1); // gives 0.1 secs margin
  v *= (unsigned long)motion_state.block[prev_buffer_index(motion_state.buffer_in)].speed;
  v = v >> 5;
  return(length < v);
}


// Public routines...

// This routine must be called when the acceleration setting is changed.
// max_period_for_ramping is simply a function of acceleration, pre-calculated for efficiency.  The purpose of this
// is as follows:
//
// In normal operation, the speed ramps up by ((period * acceleration) >> 8) each step.  However, when starting off
// from very low speed, period may be very large, causing an unacceptably large jump in speed.  To avoid this problem,
// we place a limit on the value of period used for ramping purposes.  max_period_for_ramping is that limit.
//
// Assume speed is 0 when the first step pulse is issued, and speed is "v" when the second pulse is issued.  We'll call
// the period between the step pulses "t", and the acceleration "a".  The average speed is 0.5 * v, and consequently we
// can say:
//
// t = 1 / (0.5 * v)
//
// As we started off at zero speed, we also know that:
//
// v = t * a
//
// so by substitution, we obtain:
//
// t = 1 / (0.5 * t * a)      so      t * t = 2 / a      so      t = sqrt(2 / a)
//
// This routine simply finds sqrt(2 / a), corrected according to the units of acceleration and period.
// We also pre-calculate max_speed_step; see calculate_end_speed() for description.
void set_acceleration(char line[]) {
  extern parameter_struct pa;
  extern t_motion_state   motion_state;

  if (has_code(line, 'S')) {
    pa.motion.acceleration = max(1, min(255, get_uint(line, 'S')));
  } /* endif */

  if (has_code(line, 'T')) {
    pa.motion.allowable_error = max(1, min(255, get_uint(line, 'T')));
  } /* endif */

  motion_state.max_period_for_ramping = (unsigned short)(0x10000L / isqrt(((unsigned long)pa.motion.acceleration) << 3));
  // max_speed_step = 2^18 * sqrt(2 * a* e); see calculate_end_speed()
  motion_state.max_speed_step = max(1.0, min((float)0xffffffff, 262144.0 * sqrt(2.0 * (float)pa.motion.acceleration * (float)pa.motion.allowable_error)));
}


void initialise_motion(void) {
  extern parameter_struct pa;
  extern t_motion_state   motion_state;
  unsigned char           i;

  PIO_Configure(&LID_SWITCH[is_rev1_hw()], 1);
  PIO_Configure(&Z_TOP_SWITCH[is_rev1_hw()], 1);

  set_acceleration(""); // initialises motion_state.max_period_for_ramping and motion_state.max_speed_step
  motion_state.speed_x4k = ((unsigned long)MIN_SPEED) << 12;
  motion_state.timer_period = 0x10000L / MIN_SPEED;

  motion_state.switch_select = 0;
  motion_state.slip_select = 0;
  motion_state.switch_abandon_counter = 0;
  motion_state.filament_slip = 0;
  motion_state.what_are_we_doing = MOTION_NORMAL;

  motion_state.buffer_in = 0;
  motion_state.buffer_out = 0;

  for (i = 0; i < NUM_AXIS; i ++) {
    motion_state.acc[i] = 0;
  } /* endfor */

  AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TC0; // enable peripheral clock
  TC_Configure(AT91C_BASE_TC0, 2 | AT91C_TC_CPCTRG); // MCK/32; reset counter on compare
  AT91C_BASE_TC0->TC_RC = motion_state.timer_period;

  IRQ_ConfigureIT(AT91C_ID_TC0, 0, TC0_IrqHandler); // configure and enable interrupt on timer

  AT91C_BASE_TC0->TC_IER = AT91C_TC_CPCS; // interrupt on compare
  IRQ_EnableIT(AT91C_ID_TC0);
  TC_Start(AT91C_BASE_TC0);

  // leave until end so pull-ups have had some time to take effect...
  for (i = 0; i < NUM_EXTRUDERS; i ++) {
    motion_state.filament_autoload_counter[i] = get_index_wheel(i);
    motion_state.filament_slip_error[i] = 0;
    motion_state.index_wheel_moved[i] = 0;
  } /* endfor */

  motion_state.filament_holdoff_timer = get_time();
  motion_state.z_top_switch_deglitch_counter = 0;
}


void maintain_motion(void) {
  monitor_index_wheels();
  monitor_z_top_switch();
  manage_motor_currents();
  check_motor_drivers();
}


// Timer0 ISR (the "stepper driver interrupt").  Takes blocks from the block buffer and
// executes them by pulsing the step pins appropriately.
void TC0_IrqHandler(void) {
  extern parameter_struct    pa;
  extern t_motion_state      motion_state;
  static unsigned char       last_index_wheel[NUM_EXTRUDERS]; // NB: doesn't need initialisation
  unsigned long              v;
  unsigned char              i;
  volatile unsigned short    dummy;
  static unsigned char       active = 0;

  dummy = AT91C_BASE_TC0->TC_SR; // clear status bit to acknowledge interrupt
  (void)dummy; // suppresses unused variable warning

  if (!active) {
    if (motion_state.buffer_in != motion_state.buffer_out) { // if there's a block in the buffer
      if (motion_state.block[motion_state.buffer_out].length != 0) {
        active = 1;
      } else { // a precaution for robustness; should never happen, as zero length moves should never be queued
        motion_state.buffer_out = next_buffer_index(motion_state.buffer_out);
      } /* endif */
    } /* endif */

    if (active) {
      for (i = 0; i < NUM_AXIS; i ++) {
        motion_state.acc[i] = (motion_state.acc[i] & 0xffff0000L) | 0x00008000; // "half" a step
        motor_setdir(i, (motion_state.block[motion_state.buffer_out].inc[i] < 0) != pa.motion.invert_direction[i]);
      } /* endfor */
    } else { // idle
      motion_state.speed_x4k = ((unsigned long)MIN_SPEED) << 12;
    } /* endif */
  } /* endif */

  // filament slip detection
  for (i = 0; i < NUM_EXTRUDERS; i ++) {
    if (get_index_wheel(i) != last_index_wheel[i]) {
      last_index_wheel[i] = !last_index_wheel[i];
      motion_state.acc[i + E_AXIS] &= 0x0000ffff;
      motion_state.index_wheel_moved[i] = 1;
    } /* endif */
  } /* endfor */

  if (active) {
    // step motors
    for (i = 0; i < NUM_AXIS; i ++) {
      v = motion_state.acc[i];
      motion_state.acc[i] += (unsigned long)motion_state.block[motion_state.buffer_out].inc[i];

      if ((v ^ motion_state.acc[i]) & 0xffff0000L) { // if upper 16 bits changed
        motor_step(i);
      } /* endif */
    } /* endfor */

    // calculate allowable speed = sqrt((2 * remaining_length * acceleration) + (end_speed * end_speed))
    v = min(0x00800000L, motion_state.block[motion_state.buffer_out].length - 1); // limit very large length to prevent overflow
    v = (v << 1) * (unsigned long)pa.motion.acceleration;
    v += ((unsigned long)motion_state.block[motion_state.buffer_out].end_speed * (unsigned long)motion_state.block[motion_state.buffer_out].end_speed) >> 4;
    v = isqrt(v);

    // do speed ramping
    motion_state.speed_x4k += (unsigned long)min(motion_state.timer_period, motion_state.max_period_for_ramping) * (unsigned long)pa.motion.acceleration;
    motion_state.speed_x4k = min(v << 14, min(motion_state.speed_x4k, ((unsigned long)motion_state.block[motion_state.buffer_out].speed) << 12));
    motion_state.speed_x4k = max(((unsigned long)MIN_SPEED) << 12, motion_state.speed_x4k);

    motion_state.block[motion_state.buffer_out].length -= check_switches();

    if (motion_state.block[motion_state.buffer_out].length == 0) { // we've just finished a block
      motion_state.buffer_out = next_buffer_index(motion_state.buffer_out);
      active = 0;
    } /* endif */
  } /* endif */

  // calculate timer period
  motion_state.timer_period = (unsigned short)((0x10000L << 12) / motion_state.speed_x4k);

  AT91C_BASE_TC0->TC_RC = motion_state.timer_period; // NB: don't make this the last thing in the ISR - it seems to cause occasional immediate re-entry
  motor_unstep();
}


// Add block to the buffer to make the moves specified by moves[].
// The return value of moves[] indicates the exact movement made.
void add_block_to_buffer(float feed_rate, float speed_limit, unsigned char check_z_top_switch, unsigned char reduced_current, float moves[]) {
  extern parameter_struct    pa;
  extern t_motion_state      motion_state;
  signed long                steps[NUM_AXIS];
  float                      min_duration, v;
  unsigned long              length;
  unsigned char              i, n, b;

  // Calculate length (defines the number of times the ISR will run, so needs to be the largest number of steps issued to any one axis).
  // Also calculate number of steps each axis should move.
  length = 0;

  for (i = 0; i < B_AXIS; i ++) {
    steps[i] = (signed long)floor((moves[i] * pa.motion.steps_per_unit[i]) + 0.5);
    length = max(length, (unsigned long)abs(steps[i]));
  } /* endfor */

  if (is_nozzle_forcing_enabled()) {
    // A work-around for a hardware problem which causes the B motor driver's translator to reset.  By moving in 64 microstep
    // chunks, we prevent translator resets between moves from causing any cumulative positioning error.
    steps[B_AXIS] = 64 * (signed long)floor((moves[B_AXIS] * pa.motion.steps_per_unit[B_AXIS] / 64.0) + 0.5);
  } else {
    steps[B_AXIS] = (signed long)floor((moves[B_AXIS] * pa.motion.steps_per_unit[B_AXIS]) + 0.5);
  } /* endif */

  length = max(length, (unsigned long)abs(steps[B_AXIS]));

  if (length < 1) { // if it's too short to bother with, just leave it to get absorbed into the next block
    for (i = 0; i < NUM_AXIS; i ++) {
      moves[i] = 0.0;
    } /* endfor */

    return;
  } /* endif */

  motion_state.block[motion_state.buffer_in].length = length;
  motion_state.block[motion_state.buffer_in].end_speed = 0; // this may be increased later by planning
  motion_state.block[motion_state.buffer_in].check_z_top_switch = check_z_top_switch;

  // calculate speed limit for this block
  if (feed_rate > 0.0) {
    min_duration = 0.0;

    // calculate min duration based on axis speed limits
    for (i = 0; i < NUM_AXIS; i ++) {
      min_duration = max(min_duration, fabs(moves[i]) / pa.motion.max_speed[i]);
    } /* endfor */

    // adjust min_duration according to feed_rate and 3-dimensional x/y/z distance
    v = 0.0;

    for (i = X_AXIS; i <= Z_AXIS; i ++) {
      v += moves[i] * moves[i];
    } /* endfor */

    v = sqrt(v); // v is now 3-dimensional distance
    min_duration = max(min_duration, v / feed_rate);

    for (i = E_AXIS; i < NUM_AXIS; i ++) {
      min_duration = max(min_duration, fabs(moves[i]) / speed_limit);
    } /* endfor */

    // calculate speed based on min_duration and length
    motion_state.block[motion_state.buffer_in].speed = max(MIN_SPEED, (unsigned short)min((float)MAX_SPEED, (float)length / (SPEED_UNITS * min_duration)));
  } else {
    motion_state.block[motion_state.buffer_in].speed = MIN_SPEED;
  } /* endif */

  // Calculate inc for each axis; ISR issues inc/65536 steps per interrupt.
  // Also update moves[] according to the exact number of steps each axis will take.
  // This allows the small residual error to be carried over to the next block.
  for (i = 0; i < NUM_AXIS; i ++) {
    motion_state.block[motion_state.buffer_in].inc[i] = (signed long)floor(65536.0 * (float)steps[i] / (float)length); // range -65536 to +65536
    moves[i] = floor((((float)motion_state.block[motion_state.buffer_in].inc[i] * (float)length) + 32768.0) / 65536.0) / pa.motion.steps_per_unit[i]; // + 32768.0 to take start value of acc[] into account

    // Make sure that all axes that move during this block are on and not at hold current
    if (motion_state.block[motion_state.buffer_in].inc[i] != 0) {
      make_motor_active(i, reduced_current);
    } /* endif */
  } /* endfor */

  // part of a ghastly work-around for a hardware problem; the nozzle heaters are forced off whilst the B axis is moving
  if (motion_state.block[motion_state.buffer_in].inc[B_AXIS] != 0) {
    force_nozzles_off(1);
  } /* endif */

  n = (motion_state.buffer_in - motion_state.buffer_out) & (BLOCK_BUFFER_SIZE - 1);
  b = motion_state.buffer_in;
  motion_state.buffer_in = next_buffer_index(motion_state.buffer_in);

  // Do "planning" - this just involves updating the end speeds of all blocks to take account of the block we've just added.
  // It would be obvious to loop until b reached motion_state.buffer_out.  However, this would be
  // dangerous as motion_state.buffer_out is changed by the ISR.  By calculating n at the start,
  // we avoid the problem; it's possible we may update the end speed of a block which the ISR has
  // just finished, but this doesn't do any harm.
  while (n > 0) {
    motion_state.block[prev_buffer_index(b)].end_speed = calculate_end_speed(&motion_state.block[prev_buffer_index(b)], &motion_state.block[b], pa.motion.acceleration, motion_state.max_speed_step);
    b = prev_buffer_index(b);
    n --;
  } /* endwhile */

  // wait until the buffer's ready for more...
  while (!is_buffer_ready_for_more()) {
    do_everything_else();
  } /* endwhile */
}


void wait_until_buffer_empty(void) {
  extern t_motion_state motion_state;

  while (motion_state.buffer_out != motion_state.buffer_in) {
    do_everything_else();
  } /* endwhile */
}


unsigned char is_motion_buffer_empty(void) {
  extern t_motion_state motion_state;
  return(motion_state.buffer_out == motion_state.buffer_in);
}


// Returns the distance moved during homing.
float home_axis(unsigned char axis, float home_distance, float abandon_distance) {
  extern parameter_struct pa;
  extern t_motion_state   motion_state;
  float                   distance_moved, v, moves[NUM_AXIS];
  unsigned char           i;

  for (i = 0; i < NUM_AXIS; i ++) {
    moves[i] = 0.0;
  } /* endfor */

  distance_moved = 0.0;
  wait_until_buffer_empty();

  if (is_abort()) {
    return(distance_moved);
  } /* endif */

  motion_state.switch_select = axis;
  motion_state.slip_select = 0;
  motion_state.switch_invert = pa.motion.invert_switch[axis];

  if (get_axis_switch(axis) != pa.motion.invert_switch[axis]) { // if home switch is already actuated, we need to back off
    moves[axis] = -1.5 * home_distance;
    distance_moved += moves[axis];
    add_block_to_buffer(FLT_MAX, FLT_MAX, 0, 0, moves);
    wait_until_buffer_empty();
  } /* endif */

  if (is_abort()) {
    return(distance_moved);
  } /* endif */

  moves[axis] = home_distance;
  motion_state.switch_abandon_counter = (unsigned long)(abandon_distance * pa.motion.steps_per_unit[axis]);
  add_block_to_buffer(FLT_MAX, FLT_MAX, 0, 0, moves);
  wait_until_buffer_empty();
  v = abandon_distance - ((float)motion_state.switch_abandon_counter / pa.motion.steps_per_unit[axis]);
  distance_moved += (home_distance >= 0.0) ? v : -v;
  motion_state.switch_abandon_counter = 0;
  return(distance_moved);
}


void make_z_safe(void) {
  float         moves[NUM_AXIS];
  unsigned char i;

  for (i = 0; i < NUM_AXIS; i ++) {
    moves[i] = 0.0;
  } /* endfor */

  wait_until_buffer_empty();
  moves[Z_AXIS] = is_z_top_switch_actuated() ? Z_SAFE_LOWER_DISTANCE : Z_SAFE_RAISE_DISTANCE;
  add_block_to_buffer(FLT_MAX, FLT_MAX, 0, 0, moves);
  wait_until_buffer_empty();
}


void rotate_gantry(float distance) {
  float         moves[NUM_AXIS];
  unsigned char i;

  for (i = 0; i < NUM_AXIS; i ++) {
    moves[i] = 0.0;
  } /* endfor */

  wait_until_buffer_empty();
  moves[Z_AXIS] = 0.5 * distance;
  enable_gantry_rotation(1);
  add_block_to_buffer(FLT_MAX, FLT_MAX, 0, 0, moves);
  wait_until_buffer_empty();
  enable_gantry_rotation(0);
}


// Does selection of nozzle, with auto-homing (the switch changes state somewhere between the nozzles, so we might
// as well make use of the information every time).  distance will normally have been obtained from the head EEPROM,
// and it represents the distance from the switch changing state to the point where the nozzle is about to open.
// A +ve distance has the effect of selecting nozzle 0; a -ve distance has the effect of selecting nozzle 1.
// If the switch indicates that the nozzle we want to select is already selected, then we back off until the switch
// changes state.  Once the switch is in the desired state, we just do a normal homing operation on the B axis.
// The distance moved to get to the point where the switch changes state is returned for diagnostic purposes.
float select_nozzle(float distance, float abandon_distance) {
  extern parameter_struct pa;
  extern t_motion_state   motion_state;
  float                   moves[NUM_AXIS], speed, distance_moved;
  unsigned char           i, distance_measured;

  distance_moved = 0.0;
  distance_measured = 0;

  for (i = 0; i < NUM_AXIS; i ++) {
    moves[i] = 0.0;
  } /* endfor */

  wait_until_buffer_empty();

  if (is_abort()) {
    return(distance_moved);
  } /* endif */

  motion_state.switch_select = B_AXIS;
  motion_state.slip_select = 0;
  i = 4; // allows 3 retries
  speed = pa.motion.max_speed[B_AXIS];

  // if necessary, back off first
  while ((i > 0) && ((get_axis_switch(B_AXIS) == pa.motion.invert_switch[B_AXIS]) == (distance < 0.0))) {
    motion_state.switch_abandon_counter = abandon_distance * pa.motion.steps_per_unit[B_AXIS];
    moves[B_AXIS] = -0.25 * distance;
    motion_state.switch_invert = ((distance < 0.0) == pa.motion.invert_switch[B_AXIS]);
    add_block_to_buffer(FLT_MAX, speed, 0, 0, moves);
    wait_until_buffer_empty();

    if (!distance_measured) {
      distance_moved = abandon_distance - ((float)motion_state.switch_abandon_counter / pa.motion.steps_per_unit[B_AXIS]);
      distance_moved -= fabs(moves[B_AXIS]); // remove the overtravel
      distance_measured = 1;
    } /* endif */

    motion_state.switch_abandon_counter = 0;

    if ((get_axis_switch(B_AXIS) == pa.motion.invert_switch[B_AXIS]) == (distance < 0.0)) { // if we didn't get there
      i --;
      speed *= 0.5;

      if (i <= 1) { // if we're down to the last retry
        enable_nozzle_forcing();
        report_error(ERROR_B_POSITION_LOST);
      } /* endif */
    } /* endif */

    if (is_abort()) {
      return(distance_moved);
    } /* endif */
  } /* endwhile */

  while ((i > 0) && ((get_axis_switch(B_AXIS) == pa.motion.invert_switch[B_AXIS]) != (distance < 0.0))) {
    motion_state.switch_abandon_counter = abandon_distance * pa.motion.steps_per_unit[B_AXIS];
    moves[B_AXIS] = distance;
    motion_state.switch_invert = ((distance < 0.0) != pa.motion.invert_switch[B_AXIS]);
    add_block_to_buffer(FLT_MAX, speed, 0, 0, moves);
    wait_until_buffer_empty();

    if (!distance_measured) {
      distance_moved = abandon_distance - ((float)motion_state.switch_abandon_counter / pa.motion.steps_per_unit[B_AXIS]);
      distance_moved -= fabs(moves[B_AXIS]); // remove the overtravel
      distance_measured = 1;
    } /* endif */

    motion_state.switch_abandon_counter = 0;

    if ((get_axis_switch(B_AXIS) == pa.motion.invert_switch[B_AXIS]) != (distance < 0.0)) { // if we didn't get there
      i --;
      speed *= 0.5;

      if (i <= 1) { // if we're down to the last retry
        enable_nozzle_forcing();
        report_error(ERROR_B_POSITION_LOST);
      } /* endif */
    } /* endif */

    if (is_abort()) {
      return(distance_moved);
    } /* endif */
  } /* endwhile */

  if (i == 0) {
    report_error(ERROR_B_STUCK);
  } /* endif */

  return(distance_moved);
}


// Performs the first part of filament loading.  If the filament switch is already actuated,
// do nothing.  Otherwise...
// Do a short move which tries to end up switch_overtravel past the point where the filament
// switch was actuated.  This move times out at abandon_distance.
// The return value indicates whether the filament switch is actuated at the end.
unsigned char grab_filament(unsigned char axis, float switch_overtravel, float abandon_distance) {
  extern parameter_struct pa;
  extern t_motion_state   motion_state;
  float                   moves[NUM_AXIS];
  unsigned char           i;

  for (i = 0; i < NUM_AXIS; i ++) {
    moves[i] = 0.0;
  } /* endfor */

  wait_until_buffer_empty();

  if (is_abort()) {
    return(0);
  } /* endif */

  motion_state.what_are_we_doing = MOTION_LOADING;
  motion_state.switch_select = axis;
  motion_state.filament_slip = 0;
  motion_state.slip_select = 0;
  motion_state.switch_invert = pa.motion.invert_switch[axis];

  if (get_axis_switch(axis) == pa.motion.invert_switch[axis]) { // if filament switch not actuated
    motion_state.switch_abandon_counter = abandon_distance * pa.motion.steps_per_unit[axis];
    moves[axis] = switch_overtravel;
    add_block_to_buffer(FLT_MAX, FLT_MAX, 0, 0, moves);
    wait_until_buffer_empty();
  } /* endif */

  motion_state.switch_abandon_counter = 0;
  motion_state.what_are_we_doing = MOTION_NORMAL;
  return(get_axis_switch(axis) != pa.motion.invert_switch[axis]);
}


// Performs the main part of filament loading.  Assumes grab_filament() has already been
// successful in positioning the filament switch_overtravel past the point where the
// filament switch was actuated.  Moves the filament so it ends up distance past the
// point where the filament switch was actuated.
// The return value indicates whether the filament switch is actuated at the end.
// The reasons for splitting filament load into several parts are:
//  1] avoids an excessively long timeout if the user "teases" the index wheel with the
//     filament, but the drive mechanism is not able to grab the filament.
//  2] allows the filament to be grabbed quickly before we do other stuff (eg. move the
//     X-carriage to straighten the Bowden tube).
unsigned char load_filament(unsigned char axis, float switch_overtravel, float distance) {
  extern parameter_struct pa;
  extern t_motion_state   motion_state;
  float                   moves[NUM_AXIS];
  unsigned char           i;

  for (i = 0; i < NUM_AXIS; i ++) {
    moves[i] = 0.0;
  } /* endfor */

  wait_until_buffer_empty();

  if (is_abort()) {
    return(0);
  } /* endif */

  motion_state.what_are_we_doing = MOTION_LOADING;
  motion_state.switch_select = axis;
  motion_state.switch_invert = !pa.motion.invert_switch[axis];
  motion_state.filament_slip = 0;
  motion_state.slip_select = 1;
  motion_state.switch_abandon_counter = (distance - (2.0 * switch_overtravel)) * pa.motion.steps_per_unit[axis];
  moves[axis] = switch_overtravel;
  add_block_to_buffer(FLT_MAX, FLT_MAX, 0, 0, moves);
  wait_until_buffer_empty();

  motion_state.switch_abandon_counter = 0;
  motion_state.what_are_we_doing = MOTION_NORMAL;
  return(!motion_state.filament_slip && (get_axis_switch(axis) != pa.motion.invert_switch[axis]));
}


// Retracts the filament to overtravel_distance past the point where the filament
// switch becomes unactuated.  Times out at abandon_distance.
void unload_filament(unsigned char axis, float overtravel_distance, float abandon_distance) {
  extern parameter_struct pa;
  extern t_motion_state   motion_state;
  float                   moves[NUM_AXIS];
  unsigned char           i;

  for (i = 0; i < NUM_AXIS; i ++) {
    moves[i] = 0.0;
  } /* endfor */

  wait_until_buffer_empty();

  if (is_abort()) {
    return;
  } /* endif */

  motion_state.what_are_we_doing = MOTION_UNLOADING;
  motion_state.filament_slip = 0;
  motion_state.slip_select = 1;
  motion_state.switch_select = axis;
  motion_state.switch_invert = !pa.motion.invert_switch[axis];
  motion_state.switch_abandon_counter = abandon_distance * pa.motion.steps_per_unit[axis];
  moves[axis] = -overtravel_distance;
  add_block_to_buffer(FLT_MAX, FLT_MAX, 0, 0, moves);
  wait_until_buffer_empty();
  motion_state.switch_abandon_counter = 0;
  motion_state.filament_holdoff_timer = get_time();
  motion_state.what_are_we_doing = MOTION_NORMAL;
}


// Moves axis until a filament slip is detected, timing out at abandon_distance.  overtravel_distance indicates
// the distance allowed for deceleration, its sign determining the direction of travel.  abandon_distance should
// always be +ve.  The return value indicates how far the filament actually travelled, so that an accurate update
// of the remaining filament may be made.
float move_filament_until_slip(unsigned char axis, float overtravel_distance, float abandon_distance, float speed_limit) {
  extern parameter_struct pa;
  extern t_motion_state   motion_state;
  float                   moves[NUM_AXIS], distance_moved;
  unsigned char           i;

  for (i = 0; i < NUM_AXIS; i ++) {
    moves[i] = 0.0;
  } /* endfor */

  distance_moved = 0.0;
  wait_until_buffer_empty();

  if (is_abort() || (get_axis_switch(axis) == pa.motion.invert_switch[axis])) {
    return(distance_moved);
  } /* endif */

  motion_state.what_are_we_doing = MOTION_UNTIL_SLIP;
  motion_state.filament_slip = 0;
  motion_state.slip_select = 1;
  motion_state.switch_select = axis;
  motion_state.switch_invert = !pa.motion.invert_switch[axis];
  motion_state.switch_abandon_counter = abandon_distance * pa.motion.steps_per_unit[axis];
  moves[axis] = overtravel_distance;
  add_block_to_buffer(FLT_MAX, speed_limit, 0, 1, moves); // NB: select reduced motor current
  wait_until_buffer_empty();
  distance_moved = abandon_distance - ((float)motion_state.switch_abandon_counter / pa.motion.steps_per_unit[axis]);
  distance_moved = (overtravel_distance >= 0.0) ? distance_moved : -distance_moved;
  motion_state.switch_abandon_counter = 0;
  motion_state.what_are_we_doing = MOTION_NORMAL;

  return(distance_moved);
}


unsigned char is_filament_slip_error(unsigned char extruder) {
  extern t_motion_state motion_state;
  unsigned char         r;

  r = motion_state.filament_slip_error[extruder];
  motion_state.filament_slip_error[extruder] = 0;
  return(r);
}


unsigned char has_index_wheel_moved(unsigned char extruder) {
  extern t_motion_state motion_state;
  unsigned char         r;

  r = motion_state.index_wheel_moved[extruder];
  motion_state.index_wheel_moved[extruder] = 0;
  return(r);
}


void show_switch_state(char *response) {
  extern parameter_struct pa;

  sprintf(response, "M119 X:%d Y:%d Z:%d Z+:%d E:%d D:%d B:%d Eindex:%d Dindex:%d\r\n",
          get_axis_switch(X_AXIS) != pa.motion.invert_switch[X_AXIS],
          get_axis_switch(Y_AXIS) != pa.motion.invert_switch[Y_AXIS],
          get_axis_switch(Z_AXIS) != pa.motion.invert_switch[Z_AXIS],
          is_z_top_switch_actuated(),
          get_axis_switch(E_AXIS) != pa.motion.invert_switch[E_AXIS],
          get_axis_switch(D_AXIS) != pa.motion.invert_switch[D_AXIS],
          get_axis_switch(B_AXIS) != pa.motion.invert_switch[B_AXIS],
          get_index_wheel(E_AXIS - E_AXIS),
          get_index_wheel(D_AXIS - E_AXIS));
}


void get_switch_states(unsigned char *d) {
  extern parameter_struct pa;
  extern t_motion_state   motion_state;
  unsigned char           i;

  for (i = 0; i < NUM_AXIS; i ++) {
    d[i] = (get_axis_switch(i) != pa.motion.invert_switch[i]) ? '1' : '0';
  } /* endfor */

  d[NUM_AXIS] = PIO_Get(&LID_SWITCH[is_rev1_hw()]) ? '1' : '0';
  // d[NUM_AXIS + 1] is space for button switch
  d[NUM_AXIS + 2] = get_index_wheel(0) ? '1' : '0';
  d[NUM_AXIS + 3] = get_index_wheel(1) ? '1' : '0';
  d[NUM_AXIS + 4] = is_z_top_switch_actuated() ? '1' : '0';
}


unsigned char get_selected_nozzle(void) {
  extern parameter_struct pa;
  return(get_axis_switch(B_AXIS) == pa.motion.invert_switch[B_AXIS]);
}


t_motion_state      motion_state;
