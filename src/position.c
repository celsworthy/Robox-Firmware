#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

#include "util.h"
#include "command.h"
#include "parameters.h"
#include "motion_hal.h"
#include "gcode.h"
#include "heaters.h"
#include "motion.h"
#include "main.h"
#include "eeprom.h"
#include "position.h"


// private...

// At z homing, this routine is called to add a new levelling point.  If the list of
// levelling points is already full, the oldest entry is discarded to make way for
// the new entry.
void add_levelling_point(float x, float y, float z_delta) {
  extern t_position_state position_state;
  unsigned char i;

  // shift the points up, adjusting according to z_delta, and discarding the oldest point if the list is already full
  for (i = NUM_LEVELLING_POINTS - 1; i > 0; i --) {
    position_state.levelling_points[i].x = position_state.levelling_points[i - 1].x;
    position_state.levelling_points[i].y = position_state.levelling_points[i - 1].y;
    position_state.levelling_points[i].z = position_state.levelling_points[i - 1].z - z_delta;
  } /* endfor */

  // enter the new point, with z of zero
  position_state.levelling_points[0].x = x;
  position_state.levelling_points[0].y = y;
  position_state.levelling_points[0].z = 0.0;
  position_state.levelling_point_count = min(position_state.levelling_point_count + 1, NUM_LEVELLING_POINTS);
}


// Uses inverse-distance-squared interpolation to determine the bed levelling correction
// for the Z axis, based on the list of levelling points.  The correction is "washed out"
// according to the Z position.
float get_z_correction(float x_move, float y_move, float z_dest) {
  extern t_position_state position_state;
  float         sum, acc, x, y, v;
  unsigned char i;

  if (position_state.levelling_point_count == 0) { // no levelling points, so no correction possible
    return(0.0);
  } /* endif */

  x = position_state.position[X_AXIS] + x_move - position_state.tool_offset[X_AXIS];
  y = position_state.position[Y_AXIS] + y_move - position_state.tool_offset[Y_AXIS];
  sum = acc = 0.0;

  for (i = 0; i < position_state.levelling_point_count; i ++) {
    v = ((position_state.levelling_points[i].x - x) * (position_state.levelling_points[i].x - x));
    v += ((position_state.levelling_points[i].y - y) * (position_state.levelling_points[i].y - y));

    if (v < 1.0e-10) { // avoid divide by zero if we're right at, or extremely close to one of the levelling points
      return(position_state.levelling_points[i].z);
    } /* endif */

    v = 1.0 / v;
    sum += v;
    acc += (v * position_state.levelling_points[i].z);
  } /* endfor */

  acc /= sum; // acc is now the interpolated correction
  v = max(0.0, min(1.0, 1.0 - (z_dest * position_state.z_washout)));
  return(acc * v);
}


// Calculates move required to bring positions back into line with destinations, then calls add_block_to_buffer()
// to queue it.  Note that add_block_to_buffer() returns actual distances, which are used to update the positions.
// Any fractional-microstep remainders therefore get carried over rather than being lost.
void do_move(float feed_rate, float speed_limit) {
  extern parameter_struct pa;
  extern t_position_state position_state;
  float                   moves[NUM_AXIS];
  unsigned char           i, check_z_top_switch;

  // note that move is limited so that the travel of the axes is not exceeded...
  for (i = X_AXIS; i <= Y_AXIS; i ++) {
    if (position_state.homing_done[i]) {
      moves[i] = max(0.0, min(pa.position.travel[i], position_state.destination[i] + position_state.tool_offset[i])) - position_state.position[i];
    } else {
      moves[i] = 0.0;
    } /* endif */
  } /* endfor */

  // because Z axis is nozzle-relative, we limit destination at a min of 0, but limit position at a max of travel
  if (position_state.homing_done[Z_AXIS]) {
    moves[Z_AXIS] = min(pa.position.travel[Z_AXIS], max(0.0, position_state.destination[Z_AXIS]) + position_state.tool_offset[Z_AXIS] + get_z_correction(moves[X_AXIS], moves[Y_AXIS], position_state.destination[Z_AXIS])) - position_state.position[Z_AXIS];
  } else {
    moves[Z_AXIS] = 0.0;
  } /* endif */

  // if Z remains < Z top switch min position throughout the move, we can enable checking of the Z top switch
  check_z_top_switch = (position_state.position[Z_AXIS] + max(moves[Z_AXIS], 0.0)) < pa.position.z_top_switch_min_position;

  // E & D axes have unlimited travel
  for (i = E_AXIS; i <= D_AXIS; i ++) {
    moves[i] = position_state.destination[i] - position_state.position[i];
  } /* endfor */

  // B axis direction is flipped according to the "tool" (ie. nozzle) selection, and "travel" is always 0 to 1
  moves[B_AXIS] = max(0.0, min(1.0, position_state.destination[B_AXIS]));
  moves[B_AXIS] = (position_state.tool == 0) ? moves[B_AXIS] : -moves[B_AXIS];
  moves[B_AXIS] -= position_state.position[B_AXIS];
  moves[B_AXIS] = position_state.homing_done[B_AXIS] ? moves[B_AXIS] : 0.0;

  add_block_to_buffer(feed_rate, speed_limit, check_z_top_switch, 0, moves); // actual distances returned in moves[]

  // update position according to actual distances, so that remainders are carried over to the next move
  for (i = 0; i < NUM_AXIS; i ++) {
    position_state.position[i] += moves[i];
  } /* endfor */
}


// public...

void initialise_position(void) {
  extern t_position_state position_state;
  unsigned char           i;

  position_state.feed_rate = DEFAULT_FEED_RATE;
  position_state.feed_rate_multiplier[0] = position_state.feed_rate_multiplier[1] = 1.0;
  position_state.feed_rate_multiplier_select = 0;
  position_state.tool = get_selected_nozzle();
  position_state.unpark_tool = 0;
  position_state.relative_moves = 0;
  position_state.unpark_relative = 0;
  position_state.z_made_safe = 0;

  for (i = 0; i < NUM_AXIS; i ++) {
    position_state.homing_done[i] = 0;
    position_state.destination[i] = 0.0;
    position_state.position[i] = 0.0;
    position_state.tool_offset[i] = 0.0;
    position_state.unpark_destination[i] = 0.0;
    position_state.home_delta[i] = 0.0;
  } /* endfor */

  for (i = 0; i < NUM_EXTRUDERS; i ++) {
    position_state.filament_diameter[i] = DEFAULT_FILAMENT_DIAMETER;
    position_state.filament_multiplier[i] = 1.0;
  } /* endfor */

  position_state.levelling_point_count = 0;
  position_state.z_washout = 0.0;
  position_state.what_are_we_doing = NORMAL;
}


// Handles G0 and G1 commands, parsing line for parameters and updating axis destinations and feed rate
// accordingly.  do_move() is then called to queue move(s) which will bring the positions into line with
// the destinations.
// If rapid is true, do_move() is passed FLT_MAX, so that no speed limit is imposed.  Otherwise, if there's
// an X, Y or Z parameter, then the feed rate is applied to the 3-dimensional distance.  Otherwise, the
// the feed rate is used as a speed limit for the E, D and B axes.
// If necessary, the move is fragmented so that the 3-dimensional distance doesn't exceed G0_G1_MAX_SEGMENT.
// This allows Z correction for bed levelling to be applied to each fragment.
void do_g0_g1(char *line, unsigned char rapid) {
  extern t_position_state position_state;
  float                   moves[NUM_AXIS], distance;
  unsigned long           segments;
  unsigned char           i, has_xyz, has_edb;

  has_edb = has_xyz = 0;
  distance = 0.0;

  for (i = 0; i < NUM_AXIS; i ++) {
    moves[i] = 0.0;
  } /* endfor */

  // X/Y/Z moves may be absolute or relative...
  for (i = X_AXIS; i <= Z_AXIS; i ++) {
    if (has_code(line, axis_codes[i])) {
      if (position_state.relative_moves) {
        moves[i] = get_float(line, axis_codes[i]);
      } else {
        moves[i] = get_float(line, axis_codes[i]) - position_state.destination[i];
      } /* endif */

      has_xyz = 1;
      distance += moves[i] * moves[i];
    } /* endif */
  } /* endfor */

  distance = sqrt(distance); // 3-dimensional distance of X/Y/Z

  // E/D axis moves are always relative, and are in units of mm^3 (ie. volumetric).  Conversion to mm is done by dividing by the
  // cross-sectional area of the filament.  The "multiplier" tweak is also applied here.
  for (i = E_AXIS; i <= D_AXIS; i ++) {
    if (has_code(line, axis_codes[i])) {
      moves[i] = get_float(line, axis_codes[i]) * position_state.filament_multiplier[i - E_AXIS] / (position_state.filament_diameter[i - E_AXIS] * position_state.filament_diameter[i - E_AXIS] * M_PI / 4.0);
      position_state.feed_rate_multiplier_select = i - E_AXIS; // 0 for E, 1 for D
      has_edb = 1;
    } /* endif */
  } /* endfor */

  // B axis moves are always absolute...
  if (has_code(line, axis_codes[B_AXIS])) {
    moves[B_AXIS] = get_float(line, axis_codes[B_AXIS]) - position_state.destination[B_AXIS];
    has_edb = 1;
  } /* endif */

  if (has_code(line, 'F')) {
    position_state.feed_rate = get_float(line, 'F') / 60.0; // F is in units/min, but internally we work in units/sec
  } /* endif */

  segments = 1 + (unsigned long)(distance / G0_G1_MAX_SEGMENT);

  for (i = 0; i < NUM_AXIS; i ++) {
    moves[i] /= (float)segments;
  } /* endfor */

  while (segments > 0) {
    for (i = 0; i < NUM_AXIS; i ++) {
      position_state.destination[i] += moves[i];
    } /* endfor */

    if (rapid) {
      do_move(FLT_MAX, FLT_MAX);
    } else if (has_xyz) {
      if (has_edb) {
        do_move(position_state.feed_rate * position_state.feed_rate_multiplier[position_state.feed_rate_multiplier_select], FLT_MAX); // feed rate applied to X/Y/Z 3d distance
      } else {
        do_move(position_state.feed_rate, FLT_MAX); // feed rate applied to X/Y/Z 3d distance
      } /* endif */
    } else {
      do_move(FLT_MAX, position_state.feed_rate); // no X/Y/Z move, so apply feed rate as speed limit on individual axes
    } /* endif */

    segments --;
  } /* endwhile */
}


// Moves the B axis to select the specified tool (that is, nozzle).  Moves the X, Y and Z
// axes according to the tool offsets read from the head EEPROM.  To avoid fouling the job,
// the Z axis is moved before X & Y if it's going upwards, or after X & Y if it's going
// downwards.  Likewise, the new tool is selected after the X & Y moves if it's lower-
// hanging than the old tool.  On the other hand, if the old tool is lower-hanging, the
// new tool is selected before the X & Y moves.
// The T0/T1 command can have optional X/Y/Z/B parameters, in which case these moves are
// amalgamated with the moves according to the tool offsets.  Automaker can use this
// feature to improve efficiency, when it knows the position at which the new nozzle is
// is going to start is different from the position at which the previous nozzle left off.
// NB: the reason tool is passed in, rather than being obtained from parsing line, is
// that this facilitates the invocation of do_tool_change() from unpark().
void do_tool_change(signed long tool, char *line) {
  extern parameter_struct pa;
  extern t_position_state position_state;
  float                   z_move, z_diff, opening, delta;
  unsigned short          a;
  unsigned char           i, pass;

  tool = max(0, min(2, tool)); // user might specify any tool number!
  a = 0x40 + (position_state.tool * 0x30); // address of start of tool offsets in head EEPROM
  z_diff = (position_state.tool < 2) ? get_eeprom_float(0, a + 16, 0.0) : 0.0; // Z offset of currently selected tool
  a = 0x40 + (tool * 0x30); // address of start of tool offsets in head EEPROM
  z_diff = ((tool < 2) ? get_eeprom_float(0, a + 16, 0.0) : 0.0) - z_diff; // Z offset of newly selected tool
  z_move = z_diff;

  if (has_code(line,axis_codes[Z_AXIS])) {
    z_move += get_float(line, axis_codes[Z_AXIS]);

    if (!position_state.relative_moves) {
      z_move -= position_state.destination[Z_AXIS];
    } /* endif */
  } /* endif */

  for (pass = 0; pass < 5; pass ++) {
    // if Z is moving upwards, then make the Z move first; otherwise, make the Z move last
    if (((pass == 0) && (z_move >= 0.0)) || ((pass == 4) && (z_move < 0.0))) {
      if (has_code(line, axis_codes[Z_AXIS])) {
        if (position_state.relative_moves) {
          position_state.destination[Z_AXIS] += get_float(line, axis_codes[Z_AXIS]);
        } else {
          position_state.destination[Z_AXIS] = get_float(line, axis_codes[Z_AXIS]);
        } /* endif */
      } /* endif */

      position_state.tool_offset[Z_AXIS] = (tool < 2) ? get_eeprom_float(0, a + 16, 0.0) : 0.0;
      do_move(FLT_MAX, FLT_MAX);
    } /* endif */

    // if the previous tool is lower-hanging than the new tool, then select the new tool before moving X and Y;
    // otherwise, select the new tool after moving X and Y
    if (((pass == 1) && (z_diff < 0.0)) || ((pass == 3) && (z_diff >= 0.0))) {
      if (does_head_have_b_axis()) {
        delta = max(0.0, min(1.0, position_state.destination[B_AXIS])); // previous opening
        delta += fabs(position_state.tool_offset[B_AXIS]); // previous B offset

        if (has_code(line, axis_codes[B_AXIS])) {
          position_state.destination[B_AXIS] = get_float(line, axis_codes[B_AXIS]);
        } /* endif */

        position_state.tool_offset[B_AXIS] = (tool < 2) ? get_eeprom_float(0, a + 24, 0.0) : 0.0;
        opening = max(0.0, min(1.0, position_state.destination[B_AXIS]));
        opening = (tool == 0) ? opening : -opening;
        delta = select_nozzle(opening + position_state.tool_offset[B_AXIS], pa.position.travel[B_AXIS]) - delta;

        if (position_state.homing_done[B_AXIS]) {
          if (fabs(delta) > B_POSITION_LOST_TOLERANCE) {
            enable_nozzle_forcing();
            report_error(ERROR_B_POSITION_LOST);
          }
          else if (fabs(delta) > B_POSITION_WARNING_TOLERANCE)
          {
            report_error(ERROR_B_POSITION_WARNING);
          }

          position_state.home_delta[B_AXIS] = delta;
        } /* endif */

        position_state.position[B_AXIS] = opening;
        position_state.homing_done[B_AXIS] = 1;
      } /* endif */

      position_state.tool = (unsigned char)tool;
    } /* endif */

    // move X and Y...
    if (pass == 2) {
      for (i = X_AXIS; i <= Y_AXIS; i ++) {
        if (has_code(line, axis_codes[i])) {
          if (position_state.relative_moves) {
            position_state.destination[i] += get_float(line, axis_codes[i]);
          } else {
            position_state.destination[i] = get_float(line, axis_codes[i]);
          } /* endif */
        } /* endif */
      } /* endfor */

      position_state.tool_offset[X_AXIS] = ((tool < 2) ? get_eeprom_float(0, a, 0.0) : 0.0) + pa.position.x_offset;
      position_state.tool_offset[Y_AXIS] = ((tool < 2) ? get_eeprom_float(0, a + 8, 0.0) : 0.0) + pa.position.y_offset;
      do_move(FLT_MAX, FLT_MAX);
    } /* endif */
  } /* endfor */
}


void do_homing(char *line) {
  extern parameter_struct pa;
  extern t_position_state position_state;
  float                   z_home_tweak; // compensation for deformation of mechanism as a result of nozzle force required to open Z home contact
  unsigned short          a;
  unsigned char           i;

  // The tool offsets should have been set up by do_tool_change(), but just in case it wasn't done, we do it here.
  // This avoids the danger of confusing behaviour.
  a = 0x40 + (position_state.tool * 0x30); // address of start of tool offsets in head EEPROM
  position_state.tool_offset[X_AXIS] = ((position_state.tool < 2) ? get_eeprom_float(0, a, 0.0) : 0.0) + pa.position.x_offset;
  position_state.tool_offset[Y_AXIS] = ((position_state.tool < 2) ? get_eeprom_float(0, a + 8, 0.0) : 0.0) + pa.position.y_offset;
  position_state.tool_offset[Z_AXIS] = (position_state.tool < 2) ? get_eeprom_float(0, a + 16, 0.0) : 0.0;
  z_home_tweak = (position_state.tool < 2) ? 0.5 * (get_eeprom_float(0, 0x50, 0.0) + get_eeprom_float(0, 0x80, 0.0)) : 0.0; // mean of Z tool offsets

  if (!position_state.z_made_safe) {
    make_z_safe();
    position_state.z_made_safe = 1;
  } /* endif */

  // X&Y homing is absolute, so we zero position and set destination according to tool offset
  for (i = X_AXIS; i <= Y_AXIS; i ++) {
    if (has_code(line, axis_codes[i])) {
      position_state.position[i] += home_axis(i, pa.position.home_distance[i], 1.5 * pa.position.travel[i]);
      position_state.home_delta[i] = (pa.position.home_distance[i] >= 0.0) ? (position_state.position[i] - pa.position.travel[i]) : position_state.position[i];

      if (!has_code(line, '?')) { // '?' for measure only
        position_state.position[i] = (pa.position.home_distance[i] >= 0.0) ? pa.position.travel[i] : 0.0; // range of axis always 0 to travel
        position_state.destination[i] = position_state.position[i] - position_state.tool_offset[i];
        position_state.homing_done[i] = 1;
      } /* endif */

      do_move(FLT_MAX, FLT_MAX); // if measure only, go back to where we were; otherwise do nothing
    } /* endif */
  } /* endfor */

  // Z homing is nozzle-relative, so we zero destination and set position according to tool offset
  if (has_code(line, axis_codes[Z_AXIS])) {
    position_state.position[Z_AXIS] += home_axis(Z_AXIS, pa.position.home_distance[Z_AXIS], 1.5 * pa.position.travel[Z_AXIS]);
    position_state.home_delta[Z_AXIS] = position_state.position[Z_AXIS]; // where we are
    position_state.home_delta[Z_AXIS] -= pa.position.home_distance[Z_AXIS] + z_home_tweak + position_state.tool_offset[Z_AXIS]; // where we "should" be

    if (!has_code(line, '?')) { // '?' for measure only of Z
      position_state.position[Z_AXIS] = pa.position.home_distance[Z_AXIS] + z_home_tweak + position_state.tool_offset[Z_AXIS];
      position_state.destination[Z_AXIS] = 0.0;
      position_state.homing_done[Z_AXIS] = 1;
      add_levelling_point(position_state.position[X_AXIS] - position_state.tool_offset[X_AXIS], position_state.position[Y_AXIS] - position_state.tool_offset[Y_AXIS], position_state.home_delta[Z_AXIS]);
    } /* endif */

    do_move(FLT_MAX, FLT_MAX); // undo overtravel and correct for Z home tweak
  } /* endif */
}


// Homes the Y axis if necessary, then moves the Y axis to a position beyond its normal travel which unlocks the
// cover.  Note that this is an absolute position, unaffected by the tool offset.
void unlock_cover(char *line) {
  extern parameter_struct pa;
  extern t_position_state position_state;
  float                   moves[NUM_AXIS];
  unsigned char           i;

  if (!position_state.homing_done[Y_AXIS]) {
    do_homing("Y");
  } /* endif */

  if (!has_code(line, 'S')) { // S means just do it
    turn_off_heaters();
    force_ambient_fan(1); // speeds cooling of bed, somewhat
    wait_until_bed_is_cool();
    force_ambient_fan(0);
  } /* endif */

  if (!is_abort()) {
    for (i = 0; i < NUM_AXIS; i ++) {
      moves[i] = 0.0;
    } /* endfor */

    moves[Y_AXIS] = pa.position.cover_unlock_y_position - position_state.position[Y_AXIS];
    add_block_to_buffer(FLT_MAX, FLT_MAX, 0, 0, moves); // actual distances returned in moves[]
    position_state.position[Y_AXIS] += moves[Y_AXIS];
    position_state.destination[Y_AXIS] = position_state.position[Y_AXIS] - position_state.tool_offset[Y_AXIS];
  } /* endif */
}


// Uses the most recent two levelling points to level the gantry.  Once this is done, the z
// values of the two points are made zero, as there's no longer any compensation needed.
// Any additional levelling points are discarded, to avoid the complication of trying to
// adjust them.  Intended usage is that 2 Z homes are done, then the gantry is levelled,
// then additional Z homes are done to populate the levelling points as required.
void level_gantry(void) {
  extern parameter_struct pa;
  extern t_position_state position_state;
  float  x_delta, z_delta, adj;

  if (position_state.levelling_point_count < 2) {
    return;
  } /* endif */

  position_state.levelling_point_count = 2;
  x_delta = position_state.levelling_points[0].x - position_state.levelling_points[1].x;
  z_delta = position_state.levelling_points[0].z - position_state.levelling_points[1].z;

  if (x_delta < LEVELLING_MIN_X_DELTA) {
    return;
  } /* endif */

  adj = z_delta * (pa.position.zb_leadscrew_x_position - pa.position.za_leadscrew_x_position) / x_delta;

  if (fabs(adj) > MAX_GANTRY_ADJUSTMENT) {
    report_error(ERROR_MAX_GANTRY_ADJUSTMENT);
    adj = max(-MAX_GANTRY_ADJUSTMENT, min(MAX_GANTRY_ADJUSTMENT, adj));
  } /* endif */

  rotate_gantry(adj);
  position_state.levelling_points[1].z = 0.0; // now that we've levelled the gantry, the two points in the list should both have z values of zero (point[0].z will already be zero)

  // compensate Z position according to where last Z home was done (if, for example, it was done midway between the leadscrews, then no adjustment is required)
  position_state.position[Z_AXIS] -= adj * ((0.5 * (pa.position.za_leadscrew_x_position + pa.position.zb_leadscrew_x_position)) - position_state.levelling_points[0].x) / (pa.position.zb_leadscrew_x_position - pa.position.za_leadscrew_x_position);
  do_move(FLT_MAX, FLT_MAX);
}


// Now that bed levelling is full-time, this routine has no effect other than to
// allow the washout to be adjusted, and to provide a means of clearing the levelling
// points.
void level_bed(char *line) {
  extern t_position_state position_state;

  if (has_code(line, 'S')) {
    position_state.z_washout = get_float(line, 'S');
  } else {
    position_state.levelling_point_count = 0;
  } /* endif */
}


// Handles load filament M command, parsing line for E and/or D axis letters.  First,
// grab_filament() is used to get the filament(s) to the point where the filament switch
// is actuated.  Then, we move the X-carriage to the right to straighten the Bowden tube,
// minimising the risk of it getting pierced by the filament.  Next, load_filament()
// advances the filament a specific distance using full motor current.
// Finally, move_filament_until_slip() advances the filament until it slips, using
// reduced motor current.  This ensures that the filament reaches the nozzle, allowing
// for variation in the length of the Bowden tube from unit to unit.
void do_filament_load(char *line) {
  extern parameter_struct pa;
  extern t_position_state position_state;
  unsigned char           i, started[NUM_AXIS];

  for (i = E_AXIS; i <= D_AXIS; i ++) {
    started[i] = 0;

    if (has_code(line, axis_codes[i])) {
      position_state.what_are_we_doing = (i == D_AXIS) ? LOADING_D_FILAMENT : LOADING_E_FILAMENT;
      started[i] = grab_filament(i, FILAMENT_LOAD_SWITCH_OVERTRAVEL, FILAMENT_LOAD_ABANDON_DISTANCE);
    } /* endif */
  } /* endfor */

  if (!is_abort() && (started[E_AXIS] || started[D_AXIS])) {
    if (!position_state.homing_done[X_AXIS]) {
      do_homing("X");
    } /* endif */

    if (!is_abort()) {
      position_state.destination[X_AXIS] = min(pa.position.filament_load_x_position, pa.position.travel[X_AXIS]);
      do_move(FLT_MAX, FLT_MAX); // move X-carriage fully to the right, to ease filament's travel through Bowden tube
    } /* endif */
  } /* endif */

  for (i = E_AXIS; i <= D_AXIS; i ++) {
    if (started[i] && !is_abort()) {
      position_state.what_are_we_doing = (i == D_AXIS) ? LOADING_D_FILAMENT : LOADING_E_FILAMENT;

      if (load_filament(i, FILAMENT_LOAD_SWITCH_OVERTRAVEL, pa.position.home_distance[i])) {
        move_filament_until_slip(i, FILAMENT_SLIP_OVERTRAVEL, FILAMENT_LOAD_DISTANCE_TO_SLIP, FILAMENT_LOAD_SPEED_TO_SLIP);
      } /* endif */
    } /* endif */
  } /* endfor */

  position_state.what_are_we_doing = NORMAL;
}


// Handles unload filament M command, parsing line for E and/or D axis letters
void do_filament_unload(char *line, unsigned char pause) {
  extern parameter_struct pa;
  extern t_position_state position_state;
  unsigned char           i;

  for (i = E_AXIS; i <= D_AXIS; i ++) {
    if (has_code(line, axis_codes[i])) {
      position_state.what_are_we_doing = (i == D_AXIS) ? UNLOADING_D_FILAMENT : UNLOADING_E_FILAMENT;

      if (pause) {
        pause_and_park(); // if already paused and parked, or if no job running, has no effect
      } /* endif */

      if (!is_abort()) {
        if (!position_state.homing_done[X_AXIS]) {
          do_homing("X");
        } /* endif */
      } /* endif */

      if (!is_abort()) {
        position_state.destination[X_AXIS] = min(pa.position.filament_load_x_position, pa.position.travel[X_AXIS]);
        do_move(FLT_MAX, FLT_MAX); // move X-carriage fully to the right, to ease filament's travel through Bowden tube
      } /* endif */
  
      if (is_head_dual_material()) { // unfortunately, with dual material E gets plumbed to nozzle 1 and D gets plumbed to nozzle 0!
        heat_for_filament_unload(has_code(line, axis_codes[D_AXIS]), has_code(line, axis_codes[E_AXIS]));
      } else {
        heat_for_filament_unload(has_code(line, axis_codes[E_AXIS]), has_code(line, axis_codes[D_AXIS]));
      } /* endif */

      if (!is_abort()) {
        unload_filament(i, FILAMENT_UNLOAD_DISTANCE, FILAMENT_UNLOAD_ABANDON_DISTANCE);
      } /* endif */
    } /* endif */
  } /* endfor */

  position_state.what_are_we_doing = NORMAL;
}


// Handles G36 (move filament until slip) command.  Note that the 'F' feed rate parameter (if specified) only
// affects this command; it doesn't change the feed rate used by G01.  However, feed rate units are mm/min for
// consistency with the feed rate used by G01.
void do_move_filament_until_slip(char *line) {
  extern parameter_struct pa;
  extern t_position_state position_state;
  unsigned char           i;
  float                   overtravel, v, speed;

  speed = FLT_MAX;

  if (has_code(line, 'F')) {
    speed = get_float(line, 'F') / 60.0; // F is in units/min, but internally we work in units/sec
  } /* endif */

  for (i = E_AXIS; i <= D_AXIS; i ++) {
    if (has_code(line, axis_codes[i])) {
      v = get_float(line, axis_codes[i]);
      overtravel = min(FILAMENT_SLIP_OVERTRAVEL, fabs(v));
      overtravel = (v < 0.0) ? -overtravel : overtravel;
      v = move_filament_until_slip(i, overtravel, fabs(v), speed);
      position_state.destination[i] += v; // for keeping track of remaining filament length
      position_state.position[i] += v;    // for keeping track of remaining filament length
    } /* endif */
  } /* endfor */
}


void park(void) {
  extern t_position_state position_state;
  unsigned char i;

  wait_until_buffer_empty();
  position_state.unpark_relative = position_state.relative_moves;
  position_state.unpark_feed_rate = position_state.feed_rate;
  position_state.unpark_tool = position_state.tool;

  for (i = 0; i < NUM_AXIS; i ++) {
    position_state.unpark_destination[i] = position_state.destination[i];
  } /* endfor */

  position_state.destination[B_AXIS] = 0.0; // close nozzle
  do_move(FLT_MAX, FLT_MAX);
  position_state.destination[Z_AXIS] += PARKING_Z_DISTANCE; // move nozzle clear of workpiece
  do_move(FLT_MAX, FLT_MAX);
  wait_until_buffer_empty();
  pause_heaters();
}


void unpark(unsigned char reselect_tool) {
  extern t_position_state position_state;

  resume_heaters(); // waits until up to temperature

  if (is_abort()) {
    return;
  } /* endif */

  position_state.relative_moves = position_state.unpark_relative;
  position_state.feed_rate = position_state.unpark_feed_rate;
  position_state.destination[X_AXIS] = position_state.unpark_destination[X_AXIS];
  position_state.destination[Y_AXIS] = position_state.unpark_destination[Y_AXIS];
  position_state.destination[Z_AXIS] = position_state.unpark_destination[Z_AXIS];
  position_state.destination[B_AXIS] = min(position_state.destination[B_AXIS], position_state.unpark_destination[B_AXIS]); // if B is closing might as well do it concurrently with other axes

  if (reselect_tool || (position_state.tool != position_state.unpark_tool)) {
    do_tool_change(position_state.unpark_tool, "");
  } else {
    do_move(FLT_MAX, FLT_MAX);
  } /* endif */

  position_state.destination[B_AXIS] = position_state.unpark_destination[B_AXIS];
  do_move(FLT_MAX, FLT_MAX);
}


unsigned char do_filament_slip_retry(unsigned char extruder) {
  extern t_position_state position_state;

  wait_until_buffer_empty();
  has_index_wheel_moved(extruder); // clear move detector
  position_state.destination[E_AXIS + extruder] -= FILAMENT_SLIP_RETRY_DISTANCE;
  do_move(FLT_MAX, FLT_MAX);
  wait_until_buffer_empty();

  if (has_index_wheel_moved(extruder)) {
    delay_aborting(5.0);
    position_state.destination[E_AXIS + extruder] += FILAMENT_SLIP_RETRY_DISTANCE;
    do_move(FLT_MAX, FLT_MAX);
    wait_until_buffer_empty();
    return(1);
  } /* endif */

  position_state.destination[E_AXIS + extruder] -= FILAMENT_SLIP_RETRY_DISTANCE;
  do_move(FLT_MAX, FLT_MAX);
  wait_until_buffer_empty();
  return(0);
}


void set_position(char *line) {
  extern t_position_state position_state;
  unsigned char           i;

  wait_until_buffer_empty();

  for (i = X_AXIS; i <= Z_AXIS; i ++) {
    if (has_code(line, axis_codes[i])) {
      position_state.destination[i] = get_float(line, axis_codes[i]);
      position_state.position[i] = position_state.destination[i] + position_state.tool_offset[i];
      position_state.homing_done[i] = 1;

      if (i == Z_AXIS) {
        position_state.position[Z_AXIS] += get_z_correction(0.0, 0.0, position_state.destination[Z_AXIS]);
      } /* endif */
    } /* endif */
  } /* endfor */
}


// Note that this only affects X/Y/Z axes.  E/D are always relative.  B is always absolute.
void select_relative_moves(unsigned char en) {
  extern t_position_state position_state;
  position_state.relative_moves = en;
}


void show_position(char *response) {
  extern t_position_state position_state;
  sprintf(response, "X:%f Y:%f Z:%f B:%f T:%d\r\n",
          position_state.position[X_AXIS] - position_state.tool_offset[X_AXIS],
          position_state.position[Y_AXIS] - position_state.tool_offset[Y_AXIS],
          position_state.position[Z_AXIS] - (position_state.tool_offset[Z_AXIS] + get_z_correction(0.0, 0.0, position_state.destination[Z_AXIS])),
          (position_state.tool == 0) ? position_state.position[B_AXIS] : -position_state.position[B_AXIS],
          position_state.tool);
}


void show_delta(unsigned char axis, char *response) {
  extern t_position_state position_state;
  extern const char       axis_codes[];
  sprintf(response, "%cdelta:%f\r\n", axis_codes[axis], position_state.home_delta[axis]);
}


void get_positions(unsigned char *d) {
  extern t_position_state position_state;

  memset(d, 0, 32);
  sprintf((char *)&d[0], "%f", position_state.position[X_AXIS] - position_state.tool_offset[X_AXIS]);
  sprintf((char *)&d[8], "%f", position_state.position[Y_AXIS] - position_state.tool_offset[Y_AXIS]);
  sprintf((char *)&d[16], "%f", position_state.position[Z_AXIS] - (position_state.tool_offset[Z_AXIS] + get_z_correction(0.0, 0.0, position_state.destination[Z_AXIS])));
  sprintf((char *)&d[24], "%f", (position_state.tool == 0) ? position_state.position[B_AXIS] : -position_state.position[B_AXIS]);
  d[32] = position_state.tool + '0';
}


void get_filament_info(unsigned char *d) {
  extern t_position_state position_state;

  memset(d, 0, 48);
  sprintf((char *)&d[0], "%f", position_state.filament_diameter[0]);
  sprintf((char *)&d[8], "%f", position_state.filament_multiplier[0]);
  sprintf((char *)&d[16], "%f", position_state.filament_diameter[1]);
  sprintf((char *)&d[24], "%f", position_state.filament_multiplier[1]);
  sprintf((char *)&d[32], "%f", position_state.feed_rate_multiplier[1]);
  sprintf((char *)&d[40], "%f", position_state.feed_rate_multiplier[0]);
}


void set_feed_rate_multiplier(unsigned char extruder, unsigned char *d) {
  extern t_position_state position_state;
  char                    b[9];

  b[8] = 0;
  memcpy(b, &d[0], 8);
  position_state.feed_rate_multiplier[extruder] = max(0.0, strtof(b, NULL));
}


void set_filament_info(unsigned char extruder, unsigned char *d) {
  extern t_position_state position_state;
  char                    b[9];

  b[8] = 0;
  memcpy(b, &d[0], 8);
  position_state.filament_diameter[extruder] = max(0.1, strtof(b, NULL)); // don't allow divide by zero problems to arise
  memcpy(b, &d[8], 8);
  position_state.filament_multiplier[extruder] = max(0.0, strtof(b, NULL));
}


void get_filament_info_from_reel(void) {
  extern t_position_state position_state;

  position_state.filament_diameter[0] = get_eeprom_float(1, 0x50, position_state.filament_diameter[0]);
  position_state.filament_multiplier[0] = get_eeprom_float(1, 0x58, position_state.filament_multiplier[0]);
  position_state.filament_diameter[1] = get_eeprom_float(2, 0x50, position_state.filament_diameter[1]);
  position_state.filament_multiplier[1] = get_eeprom_float(2, 0x58, position_state.filament_multiplier[1]);
  position_state.feed_rate_multiplier[0] = get_eeprom_float(1, 0x60, position_state.feed_rate_multiplier[0]);
  position_state.feed_rate_multiplier[1] = get_eeprom_float(2, 0x60, position_state.feed_rate_multiplier[1]);
}


// The E/D axes have destination and position variables for consistency with the other axes, but absolute position
// of E/D is not meaningful, except for the purposes of updating the remaining filament field of the reel EEPROM.
// It is important that E/D destination & position are not allowed to reach large values (eg. 240000 if a whole
// reel of filament is extruded) as the dynamic range of float variables starts to be a problem.
// It's important that we do not try to make small adjustments to the remaining filament value as it also has
// limited precision, being represented by an 8 character text string.
// This routine deals with both limitations.  If destination (position) is in the range 0 to 1, no action is taken.
// If destination (position) is outside this range, it is adjusted by an integer amount to bring it back into the
// range 0 to 1, and the remaining filament field of the EEPROM is adjusted by the same amount.
void update_reel_filament_length(void) {
  extern t_position_state position_state;
  float  v;

  v = floor(position_state.destination[E_AXIS]); // update in whole mm; avoids error build-up due to limited precision of length
  write_eeprom_float(1, 0xb8, get_eeprom_float(1, 0xb8, 0.0) - v);
  position_state.destination[E_AXIS] -= v; // keep small to avoid loss of precision
  position_state.position[E_AXIS] -= v; // keep small to avoid loss of precision

  v = floor(position_state.destination[D_AXIS]); // update in whole mm; avoids error build-up due to limited precision of length
  write_eeprom_float(2, 0xb8, get_eeprom_float(2, 0xb8, 0.0) - v);
  position_state.destination[D_AXIS] -= v; // keep small to avoid loss of precision
  position_state.position[D_AXIS] -= v; // keep small to avoid loss of precision
}


// Global hiding routine
unsigned char is_loading_e_filament(void) {
  extern t_position_state position_state;
  return(position_state.what_are_we_doing == LOADING_E_FILAMENT);
}


// Global hiding routine
unsigned char is_unloading_e_filament(void) {
  extern t_position_state position_state;
  return(position_state.what_are_we_doing == UNLOADING_E_FILAMENT);
}


// Global hiding routine
unsigned char is_loading_d_filament(void) {
  extern t_position_state position_state;
  return(position_state.what_are_we_doing == LOADING_D_FILAMENT);
}


// Global hiding routine
unsigned char is_unloading_d_filament(void) {
  extern t_position_state position_state;
  return(position_state.what_are_we_doing == UNLOADING_D_FILAMENT);
}


// Global hiding routine
unsigned char get_selected_tool(void) {
  extern t_position_state position_state;
  return(position_state.tool);
}


// Global hiding routine
float get_filament_multiplier(unsigned char n) {
  extern t_position_state position_state;
  return(position_state.filament_multiplier[n]);
}


void set_z_leadscrew_x_positions(char *line) {
  extern parameter_struct pa;

  if (has_code(line, 'A')) {
    pa.position.za_leadscrew_x_position = get_float(line, 'A');
  } /* endif */

  if (has_code(line, 'B')) {
    pa.position.zb_leadscrew_x_position = get_float(line, 'B');
  } /* endif */
}


void set_filament_load_x_position(char *line) {
  extern parameter_struct pa;

  if (has_code(line, 'X')) {
    pa.position.filament_load_x_position = get_float(line, 'X');
  } /* endif */
}


void set_cover_unlock_y_position(char *line){
  extern parameter_struct pa;

  if (has_code(line, 'Y')) {
    pa.position.cover_unlock_y_position = get_float(line, 'Y');
  } /* endif */
}


void set_z_top_switch_min_position(char *line) {
  extern parameter_struct pa;

  if (has_code(line, 'Z')) {
    pa.position.z_top_switch_min_position = get_float(line, 'Z');
  } /* endif */
}


void set_xy_offset(char *line) {
  extern parameter_struct pa;

  if (has_code(line, 'X')) {
    pa.position.x_offset = get_float(line, 'X');
  } /* endif */

  if (has_code(line, 'Y')) {
    pa.position.y_offset = get_float(line, 'Y');
  } /* endif */
}


t_position_state position_state;
