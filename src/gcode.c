/*
Implemented Codes
-------------------
 G0  - Rapid coordinated movement X Y Z E D B
 G1  - Coordinated movement X Y Z E D B
 G4  - Dwell S<seconds> or P<milliseconds>
 G28 - Home
 G36 - Move filament E/D until slip
 G37 - Unlock cover [S]
 G38 - Level gantry
 G39 - Clear bed levelling points [S set bed levelling washout]
 G90 - Absolute X/Y/Z moves
 G91 - Relative X/Y/Z moves
 G92 - Set current position to coordinates given X/Y/Z

 T0  - Tool (nozzle) 0
 T1  - Tool (nozzle) 1

 M1   - Pause
 M84  - Disable motors until next move
 M92  - Set axis_steps_per_unit - same syntax as G92
 M103 - Set nozzle first layer target temperature S T
 M104 - Set nozzle target temperature S T
 M105 - Show temperatures
 M106 - Head Fan on
 M107 - Head Fan off
 M109 - Wait for nozzle temperature to reach target
 M111 - Show X delta
 M112 - Show Y delta
 M113 - Show Z delta
 M114 - Display current position
 M115 - Show firmware revision
 M119 - Show switch state
 M120 - Load filament E/D
 M121 - Unload filament E/D
 M122 - Unload filament E/D without pause
 M126 - Head power off
 M127 - Head power on
 M128 - Head lights off
 M129 - Head lights on
 M139 - Set bed first layer target temperature S
 M140 - Set bed target temperature S
 M150 - Get nozzle target temperatures from reel S, T
 M154 - Get bed target temperatures from reel
 M157 - Get ambient target temperature from reel
 M170 - Set ambient target temperature
 M190 - Wait for bed temperature to reach target [S, T, P]
 M201 - Set maximum acceleration for moves (M201 S10)
 M202 - Set maximum speed that each axis can sustain (M203 X200 Y200 Z300 E100 D100 B69) in mm/sec

 M301 - Set nozzle heater parameters P, I, D, L, B, T, U
 M302 - Set bed heater parameters P, I, D, L, B, T, U
 M303 - Set ambient temperature control parameters P, I, D, L, B, T, U
 M304 - Set parameters for nozzle prtd A, B, T

 M400 - Finish all moves

 M500 - Store parameters to EEPROM
 M501 - Read parameters from EEPROM (if you need to reset them after you changed them temporarily).
 M502 - Revert to the default "factory settings". You still need to store them in EEPROM afterwards if you want to.
 M503 - Show settings

 M510 - Invert axis, 0=false, 1=true (M510 X0 Y0 Z0 E1 D1)
 M520 - Set axis travel (M520 X200 Y200 Z150)
 M526 - Invert switch inputs 0=false, 1=true (M526 X0 Y0 Z0 E0 D0 B0)
 M527 - Set home distance
 M530 - Set Z leadscrew X positions (M530 A-37.546 B251.50)
 M531 - Set filament load X position (M531 X190)
 M532 - Set cover unlock Y position (M532 Y161.5)
 M533 - Set Z top switch min position (M533 Z79)
 M534 - Set X/Y offsets (M534 X0 Y0)

 M906 - Set motor current (Amps) (M906 X1.6 Y1.6 Z1.0 E1.6 D1.6 B1.0)
 M907 - Set motor hold current (Amps) (M907 X0.4 Y0.4 Z0.4 E0.4 D0.4 B0.0)
 M908 - Set motor reduced current (Amps) (M908 E0.8 D0.8)

 M909 - Set filament slip threshold (mm) (M909 S10.0 T4.0)
 M999 - Show B distance (debug only - probably temporary)
*/

#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>

#include <board.h>

#include "util.h"
#include "command.h"
#include "eeprom.h"
#include "parameters.h"
#include "heaters.h"
#include "position.h"
#include "motion.h"
#include "motion_hal.h"
#include "file_system.h"
#include "main.h"
#include "gcode.h"

#define IDLE_PAUSE_TIME 600  // secs; if job running and no activity for this time, do auto pause
#define IDLE_ABORT_TIME 3600 // secs; if no activity for this time, do auto abort

#define GET_AXES(line,var,type,count)  { int cnt_c; for (cnt_c = 0;cnt_c < count;cnt_c++) { if (has_code(line, axis_codes[cnt_c])) var[cnt_c] = get_##type(line, axis_codes[cnt_c]); } }
#define GET_ALL_AXES(line,var,type)    GET_AXES(line,var,type,NUM_AXIS)
#define GCODE_BUFFER_SIZE              1024

#define set_head_lights(v)             set_head_output(5, (v)) // head lights is bit 5 in shift register

typedef struct {
  unsigned char  execute_buffer[GCODE_BUFFER_SIZE + 1]; // extra byte is space for terminating strings with \0
  unsigned char  job_buffer[GCODE_BUFFER_SIZE + 1]; // extra byte is space for terminating strings with \0
  unsigned short execute_buffer_bytes;
  unsigned short job_buffer_bytes;
  unsigned long  line_number;
  unsigned char  abort_flag;
  unsigned char  pause;
  unsigned char  parked;
  unsigned char  job_running;
  unsigned long  idle_timer;
  unsigned char  filament_slip_count[NUM_EXTRUDERS];
  unsigned long  filament_slip_count_timer;
} t_gcode_state;


// private routines

// For debug only!
static void memory_read(char line[], char response[]) {
  unsigned char *m = 0;
  char          *ptr, *next;

  response[0] = 0;

  ptr = strchr(line, 'M');

  if (ptr == NULL) {
    return;
  } /* endif */

  ptr ++;
  strtol(ptr, &next, 10); // skip M code

  if (next <= ptr) {
    return;
  } /* endif */

  m = (unsigned char *)strtoul(next, &ptr, 16); // get address

  if (ptr <= next) {
    return;
  } /* endif */

  put_hex(*m, 2, (unsigned char *)&response[0]);
  put_hex(*(m + 1), 2, (unsigned char *)&response[2]);
  put_hex(*(m + 2), 2, (unsigned char *)&response[4]);
  put_hex(*(m + 3), 2, (unsigned char *)&response[6]);
  response[8] = '\n';
  response[9] = 0;
}


// If there's no CR or LF, then it's not a complete GCode line so zero is returned.
// Otherwise, the length of the line plus termination is returned, where "termination"
// can be:
//
// 1] CR [whitespace]
// 2] LF [whitespace]
// 3] CR LF [whitespace]
// 4] LF CR [whitespace]
//
// If there's trailing whitespace, it is mopped up so that it can't lurk in the buffer,
// giving the appearance that there's still GCode to be executed.
static unsigned short get_gcode_line_length(char *line) {
  unsigned short i;

  i = 0;

  while (line[i] && (line[i] != '\n') && (line[i] != '\r')) {
    i ++;
  } /* endwhile */

  if (!line[i]) { // if not terminated with CR or LF, it's not a complete GCode command
    return(0);
  } /* endif */

  i += (((line[i] == '\n') && (line[i + 1] == '\r')) || ((line[i] == '\r') && (line[i + 1] == '\n'))); // ensure CR, LF (or LF, CR) only counts as one line
  i ++;

  while ((line[i] == ' ') || (line[i] == '\t')) { // skip any trailing whitespace
    i ++;
  } /* endwhile */

  return(i);
}


static void pause_command(char line[]) {
  extern t_gcode_state gcode_state;

  if (gcode_state.job_running) { // don't allow pause when no job
    gcode_state.pause = has_code(line, 'C') ? 2 : 1; // C selects "selfie" pause
  } /* endif */
}


// process a G-code command line
static void gcode_process_line(char *line, char *response) {
  extern parameter_struct pa;
  char   *comment;

  // the first occurrence of cr, lf or a comment character (if any) is changed to zero so as to terminate the string early
  if ((comment = strpbrk(line, "\r\n;(")) != NULL) {
    *comment = 0;
  } /* endif */

  strcpy(response, "");

  if (has_code(line, 'G')) {
    switch (get_int(line, 'G')) {
    case 0:
      do_g0_g1(line, 1); // rapid
      break;
    case 1:
      do_g0_g1(line, 0); // normal feed rate
      break;
    case 4: // dwell
      wait_until_buffer_empty();  // wait for all movements to finish

      if (has_code(line, 'S')) {
        delay_aborting(get_float(line, 'S')); // seconds to wait
      } else if (has_code(line, 'P')) {
        delay_aborting(get_float(line, 'P') / 1000.0); // milliseconds to wait
      } /* endif */

      break;
    case 28: // home
      do_homing(line);
      break;
    case 36: // move filament until slip
      do_move_filament_until_slip(line);
      break;
    case 37: // unlock cover
      unlock_cover(line);
      break;
    case 38: // level gantry
      level_gantry();
      break;
    case 39: // level bed
      level_bed(line);
      break;
    case 90: // absolute
      select_relative_moves(0);
      break;
    case 91: // relative
      select_relative_moves(1);
      break;
    case 92: // set position
      set_position(line);
      break;
    } /* endswitch */
  } else if (has_code(line, 'M')) {
    switch(get_int(line, 'M')) {
    case 1:
      pause_command(line);
      break;
    case 84:
      wait_until_buffer_empty(); // wait for all movements to finish
      turn_off_motors();
      break;
    case 92: // M92
      GET_ALL_AXES(line, pa.motion.steps_per_unit, float);
      break;
    case 103: // M103 set nozzle first layer temperature
      set_nozzle_temperature(line, 1);
      break;
    case 104: // M104 set nozzle temperature
      set_nozzle_temperature(line, 0);
      break;
    case 105: // M105 show temperatures
      show_temperatures(response);
      break;
    case 106: //M106 Head Fan On
      if (has_code(line, 'S')) {
        set_head_fan_pwm_duty(min(255, get_uint(line, 'S')));
      } else {
        set_head_fan_pwm_duty(255);
      } /* endif */

      break;
    case 107: //M107 Head Fan Off
      set_head_fan_pwm_duty(0);
      break;
    case 109: // M109 - Wait for nozzle temperature to reach target
      wait_until_nozzle_is_at_temperature(0);
      break;
    case 111: // M111 show X delta
      show_delta(X_AXIS, response);
      break;
    case 112: // M112 show Y delta
      show_delta(Y_AXIS, response);
      break;
    case 113: // M113 show Z delta
      show_delta(Z_AXIS, response);
      break;
    case 114: // M114 show current position
      show_position(response);
      break;
    case 115: // M115
      sprintf(response, "FIRMWARE_NAME: robox "FIRMWARE_REVISION"\r\n");
      break;
    case 119: // M119 show switch state
      show_switch_state(response);
      break;
    case 120: // M120 load filament
      do_filament_load(line);
      break;
    case 121: // M121 unload filament
      do_filament_unload(line, 1);
      break;
    case 122: // M122 unload filament without pause
      do_filament_unload(line, 0);
      break;
    case 126: // M126 head power off
      set_head_power(0);
      break;
    case 127: // M127 head power on
      set_head_power(1);
      break;
    case 128: // M128 head lights off
      set_head_lights(0);
      break;
    case 129: // M129 head lights on
      set_head_power(1);
      set_head_lights(1);
      break;
    case 139: // M139 set bed first layer temperature
      set_bed_temperature(line, 1);
      break;
    case 140: // M140 set bed temperature
      set_bed_temperature(line, 0);
      break;
    case 150: // M150 get nozzle target temperatures from reel
      get_nozzle_targets_from_reel(line);
      break;
    case 154: // M154 get bed target temperatures from reel
      get_bed_targets_from_reel();
      break;
    case 157: // M157 get ambient target temperature from reel
      get_ambient_target_from_reel();
      break;
    case 170: // M170 set ambient temperature
      set_ambient_temperature(line);
      break;
    case 190: // M190 - Wait for bed temperature to reach target
      wait_until_bed_is_at_temperature(line, 0);
      break;
    case 201: // M201  Set acceleration; applies to all axes
      set_acceleration(line);
      break;
    case 202: // M202 max speed mm/sec
      GET_ALL_AXES(line, pa.motion.max_speed, float);
      break;
    case 301: // M301
      set_nozzle_parameters(line);
      break;
    case 302: // M302
      set_bed_parameters(line);
      break;
    case 303: // M303
      set_ambient_parameters(line);
      break;
    case 304: // M304
      set_prtd_parameters(line);
      break;
    case 400: // M400 finish all moves
      wait_until_buffer_empty();
      break;
    case 500: // M500 - stores parameters in EEPROM
      store_parameters();
      break;
    case 501: // M501 - reads parameters from EEPROM (if you need to reset them after you changed them temporarily).
      load_parameters();
      break;
    case 502: // M502 - reverts to the default "factory settings". You still need to store them in EEPROM afterwards if you want to.
      set_default_parameters();
      break;
    case 503: // M503 show settings
      show_parameters(response);
      break;
    case 510: // M510 Axis invert
      GET_ALL_AXES(line, pa.motion.invert_direction, bool);
      break;
    case 520: // M520 Axis Travel
      GET_ALL_AXES(line, pa.position.travel, float);
      break;
    case 526: // M526 Switch Invert
      GET_ALL_AXES(line, pa.motion.invert_switch, bool);
      break;
    case 527: // M527 Home Distance
      GET_ALL_AXES(line, pa.position.home_distance, float);
      break;
    case 530:
      set_z_leadscrew_x_positions(line);
      break;
    case 531:
      set_filament_load_x_position(line);
      break;
    case 532:
      set_cover_unlock_y_position(line);
      break;
    case 533:
      set_z_top_switch_min_position(line);
      break;
    case 534:
      set_xy_offset(line);
      break;
    case 906: { // set motor currents in Amps using axis codes
      int cnt_c;

      for (cnt_c = 0; cnt_c < NUM_AXIS; cnt_c++) {
        if (has_code(line, axis_codes[cnt_c])) {
          pa.motion.motor_current[cnt_c] = amps_to_dac(get_float(line, axis_codes[cnt_c]));
        } /* endif */
      } /* endfor */

      break;
      }
    case 907: { // set motor hold currents in Amps using axis codes
      int cnt_c;

      for (cnt_c = 0; cnt_c < NUM_AXIS; cnt_c++) {
        if (has_code(line, axis_codes[cnt_c])) {
          pa.motion.hold_current[cnt_c] = amps_to_dac(get_float(line, axis_codes[cnt_c]));
        } /* endif */
      } /* endfor */

      break;
      }
    case 908: { // set motor reduced currents in Amps using axis codes
      int cnt_c;

      for (cnt_c = E_AXIS; cnt_c <= D_AXIS; cnt_c++) {
        if (has_code(line, axis_codes[cnt_c])) {
          pa.motion.reduced_current[cnt_c] = amps_to_dac(get_float(line, axis_codes[cnt_c]));
        } /* endif */
      } /* endfor */

      break;
      }
    case 909:
      if (has_code(line, 'S')) {
        pa.motion.filament_slip_threshold = get_float(line, 'S');
      } /* endif */

      if (has_code(line, 'T')) {
        pa.motion.filament_until_slip_threshold = get_float(line, 'T');
      } /* endif */

      break;
    case 997:
      memory_read(line, response);
      break;
    case 998:
      fake_an_error(line);
      break;
    case 999: // M999 show B distance
      show_delta(B_AXIS, response);
      break;
    } /* endswitch */
  } else if (has_code(line, 'T')) {
    do_tool_change(get_int(line, 'T'), line);
  } /* endif */

  strcat(response, "ok\r\n"); // sprinter-style acknowledgement for compatibility mode
}


// public routines...

unsigned short gcode_handler(char *line, char *response) {
  unsigned short n;

  n = get_gcode_line_length(line);

  if (n) {
    gcode_process_line(line, response);
    return(n);
  } /* endif */

  strcpy(response, "");
  return(0);
}


void initialise_job(void) {
  extern t_gcode_state gcode_state;
  unsigned char i;

  gcode_state.job_buffer_bytes = 0;
  gcode_state.execute_buffer_bytes = 0;
  gcode_state.abort_flag = 0;
  gcode_state.pause = 0;
  gcode_state.parked = 0;
  gcode_state.line_number = 0;
  gcode_state.job_running = 0;
  gcode_state.idle_timer = get_time();

  for (i = 0; i < NUM_EXTRUDERS; i ++) {
    gcode_state.filament_slip_count[i] = 0;
  } /* endfor */

  gcode_state.filament_slip_count_timer = get_time();
}


void start_job(unsigned char *id) {
  extern t_gcode_state gcode_state;

  gcode_state.line_number = 0;
  gcode_state.pause = 0;
  gcode_state.parked = 0;
  gcode_state.job_running = 1;
  gcode_state.job_buffer_bytes = 0;
  gcode_state.idle_timer = get_time();
  start_read_file(id);
}


// Deals with top level functionality.
void the_boss(void) {
  extern t_gcode_state gcode_state;
  unsigned char i;

  // filament slip handling
  if (!gcode_state.job_running) {
    for (i = 0; i < NUM_EXTRUDERS; i ++) {
      gcode_state.filament_slip_count[i] = 0;
    } /* endfor */

    gcode_state.filament_slip_count_timer = get_time();
  } else {
    if (has_period_elapsed(gcode_state.filament_slip_count_timer, 300.0)) {
      for (i = 0; i < NUM_EXTRUDERS; i ++) {
        gcode_state.filament_slip_count[i] -= (gcode_state.filament_slip_count[i] > 0);
      } /* endfor */

      gcode_state.filament_slip_count_timer = get_time();
    } /* endif */

    for (i = 0; i < NUM_EXTRUDERS; i ++) {
      if (is_filament_slip_error(i) && !gcode_state.pause) {
        gcode_state.filament_slip_count[i] ++;

        if (gcode_state.filament_slip_count[i] >= 4) {
          report_error(i ? ERROR_D_FILAMENT_SLIP : ERROR_E_FILAMENT_SLIP);
        } else {
          park();

          if (do_filament_slip_retry(i)) {
            unpark(1); // force tool reselection
          } else {
            gcode_state.filament_slip_count[i] = 0; // start with a clean slate after unloading the filament,
            gcode_state.pause = 1;
            gcode_state.parked = 1;
            report_error(i ? ERROR_D_NO_FILAMENT : ERROR_E_NO_FILAMENT);
            execute_gcode(i ? (unsigned char *)"M121 D\n" : (unsigned char *)"M121 E\n", 7); // queue up unload filament
          } /* endif */
        } /* endif */

        is_filament_slip_error(i);
      } /* endif */
    } /* endfor */
  } /* endif */

  // auto pause on error
  if (gcode_state.job_running && is_error()) {
    gcode_state.pause = 1;
  } /* endif */

  // auto pause / abort on long period of idleness
  if (gcode_state.job_running && has_period_elapsed(gcode_state.idle_timer, IDLE_PAUSE_TIME)) {
    gcode_state.pause = 1;
  } else if (has_period_elapsed(gcode_state.idle_timer, IDLE_ABORT_TIME)) {
    abort_job(); // NB: do this even if job not running - it ensures the heaters get turned off
    gcode_state.idle_timer = get_time();
  } /* endif */
}


void maintain_job(void) {
  extern t_gcode_state gcode_state;
  unsigned short n;

  the_boss();
  gcode_state.execute_buffer[gcode_state.execute_buffer_bytes] = 0; // make sure string is zero terminated
  gcode_state.job_buffer[gcode_state.job_buffer_bytes] = 0; // make sure string is zero terminated

  if (gcode_state.abort_flag) {
    if (is_motion_buffer_empty()) {
      turn_off_heaters();
      gcode_state.abort_flag = 0;
      gcode_state.pause = 0;
      gcode_state.parked = 0;
      gcode_state.job_running = 0;
      gcode_state.line_number = 0;
      get_tempcon_targets_from_reel();
      get_filament_info_from_reel();
    } /* endif */
  } else if ((n = get_gcode_line_length((char *)gcode_state.execute_buffer)) > 0) {
    gcode_process_line((char *)gcode_state.execute_buffer, NULL); // NULL discards response

    if (gcode_state.execute_buffer_bytes >= n) { // abort may have been received while we were in gcode_process_line()
      gcode_state.execute_buffer_bytes -= n;
      memmove(gcode_state.execute_buffer, &gcode_state.execute_buffer[n], gcode_state.execute_buffer_bytes);
    } /* endif */

    gcode_state.idle_timer = get_time();
  } else if (!gcode_state.job_running) {
    // do nothing
  } else if (gcode_state.pause) {
    if (!gcode_state.parked) {
      park();
      gcode_state.parked = 1;
    } /* endif */
  } else if (gcode_state.parked) {
    unpark(0);
    gcode_state.parked = 0;
  } else if ((n = get_gcode_line_length((char *)gcode_state.job_buffer)) > 0) {
    gcode_state.line_number ++;
    gcode_process_line((char *)gcode_state.job_buffer, NULL); // NULL discards response

    if (gcode_state.job_buffer_bytes >= n) { // abort may have been received while we were in gcode_process_line()
      gcode_state.job_buffer_bytes -= n;
      memmove(gcode_state.job_buffer, &gcode_state.job_buffer[n], gcode_state.job_buffer_bytes);
    } /* endif */

    gcode_state.idle_timer = get_time();
  } else if (gcode_state.job_buffer_bytes >= (GCODE_BUFFER_SIZE / 2)) {
    report_error(ERROR_GCODE_LINE_TOO_LONG);
    abort_job();
  } else if (is_eof()) {
    if (is_motion_buffer_empty()) {
      close_read_file();
      turn_off_heaters();
      gcode_state.pause = 0;
      gcode_state.job_running = 0;
      gcode_state.job_buffer_bytes = 0;
      gcode_state.line_number = 0;
      get_tempcon_targets_from_reel();
      get_filament_info_from_reel();
    } /* endif */
  } else if ((n = read_file(&gcode_state.job_buffer[gcode_state.job_buffer_bytes])) == 0) { // we're waiting for Automaker to send rest of the file...
    gcode_state.pause = 1;
  } else {
    gcode_state.job_buffer_bytes += n;
  } /* endif */
}


// This is called prior to unloading the filament.  If a job is running, it's paused
// and a park is done.  Otherwise, no action is necessary.
void pause_and_park(void) {
  extern t_gcode_state gcode_state;

  if (gcode_state.job_running) {
    gcode_state.pause = 1;

    if (!gcode_state.parked) {
      park();
      gcode_state.parked = 1;
    } /* endif */
  } /* endif */
}


void pause_resume_job(unsigned char *d) {
  extern t_gcode_state gcode_state;

  gcode_state.pause = (d[0] == '1') && gcode_state.job_running; // don't allow pause when no job
  gcode_state.idle_timer = get_time(); // ensure idle auto-pause can be cancelled!
}


void invert_pause(void) {
  extern t_gcode_state gcode_state;

  gcode_state.pause = !gcode_state.pause && gcode_state.job_running; // don't allow pause when no job
  gcode_state.idle_timer = get_time(); // ensure idle auto-pause can be cancelled!
}


void abort_job(void) {
  extern t_gcode_state gcode_state;

  gcode_state.abort_flag = 1;
  gcode_state.job_buffer_bytes = 0;
  gcode_state.execute_buffer_bytes = 0;
  close_read_file();
}


void execute_gcode(unsigned char *d, unsigned short n) {
  extern t_gcode_state gcode_state;

  if ((gcode_state.execute_buffer_bytes + n) >= GCODE_BUFFER_SIZE) { // if insufficient space, then ignore execute gcode command
    report_error(ERROR_GCODE_BUFFER_OVERRUN);
    return;
  } /* endif */

  memmove(&gcode_state.execute_buffer[gcode_state.execute_buffer_bytes], d, n);
  gcode_state.execute_buffer_bytes += n;
}


void get_job_state(unsigned char *d) {
  extern t_gcode_state gcode_state;

  put_hex(gcode_state.line_number, 8, &d[0]);

  if (gcode_state.pause == 0) {
    d[8] = gcode_state.parked ? '3' : '0';
  } else if (gcode_state.pause == 1) {
    d[8] = gcode_state.parked ? '2' : '1';
  } else { // selfie pause
    d[8] = gcode_state.parked ? '4' : '1';
  } /* endif */

  if (is_loading_e_filament()) {
    d[9] = '2';
  } else if (is_unloading_e_filament()) {
    d[9] = '3';
  } else if (is_loading_d_filament()) {
    d[9] = '4';
  } else if (is_unloading_d_filament()) {
    d[9] = '5';
  } else {
    d[9] = ((gcode_state.execute_buffer_bytes == 0) && is_motion_buffer_empty() && (gcode_state.pause || !gcode_state.job_running)) ? '0' : '1';
  } /* endif */
}


unsigned char is_abort(void) {
  extern t_gcode_state gcode_state;
  return(gcode_state.abort_flag);
}


unsigned char is_job_running(void) {
  extern t_gcode_state gcode_state;
  return(gcode_state.job_running);
}


unsigned char is_paused(void) {
  extern t_gcode_state gcode_state;
  return(gcode_state.pause > 0);
}


unsigned char is_non_selfie_paused(void) {
  extern t_gcode_state gcode_state;
  return(gcode_state.pause == 1);
}


// returns true if the line contains a reporting command
unsigned char is_gcode_reporting(unsigned char *d, unsigned short n) {
  signed long cmd;
  unsigned char temp;

  temp = d[n];
  d[n] = 0; // ensure string is terminated
  cmd = get_int((char *)d, 'M');
  d[n] = temp;
  return((cmd == 105) || (cmd == 111) || (cmd == 112) || (cmd == 113) || (cmd == 114) || (cmd == 115) || (cmd == 119) || (cmd == 503) || (cmd == 997) || (cmd == 999));
}


signed long get_int(char *line, char c) {
  char *ptr;

  ptr = strchr(line, c);
  return(ptr ? strtol(ptr + 1, NULL, 10) : 0);
}


unsigned long get_uint(char *line, char c) {
  char *ptr;

  ptr = strchr(line, c);
  return(ptr ? strtoul(ptr + 1, NULL, 10) : 0);
}


float get_float(char *line, char c) {
  char *ptr;

  ptr = strchr(line, c);
  return(ptr ? strtod(ptr + 1, NULL) : 0);
}


unsigned char has_float(char *line, char c) {
  char *ps, *pe;

  ps = strchr(line, c);

  if (ps == NULL) {
    return(0);
  } /* endif */

  strtod(ps + 1, &pe); // pe will point to the start of the string if no valid float found
  return(pe > (ps + 1));
}


unsigned char get_bool(char *line, char c) {
  return(get_int(line, c) ? 1 : 0);
}


char *get_str(char *line, char c) {
  char *ptr;

  ptr = strchr(line, c);
  return(ptr ? ptr + 1 : NULL);
}


unsigned char has_code(char *line, char c) {
  return(strchr(line, c) != NULL);
}


t_gcode_state gcode_state;
