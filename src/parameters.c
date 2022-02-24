#include <board.h>
#include "cmsis/core_cm3.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "util.h"
#include "parameters.h"
#include "command.h"
#include "motion_hal.h"
#include "gcode.h"             // for is_job_running(), is_paused()

#define PARAMETERS_PAGE0   ((AT91C_IFLASH_SIZE / AT91C_IFLASH_PAGE_SIZE) - 5)
#define PARAMETERS_PAGE1   ((AT91C_IFLASH_SIZE / AT91C_IFLASH_PAGE_SIZE) - 4)
#define PARAMETERS_ADDRESS (AT91C_IFLASH + (PARAMETERS_PAGE0 * AT91C_IFLASH_PAGE_SIZE))

#define HOURS_PAGE         ((AT91C_IFLASH_SIZE / AT91C_IFLASH_PAGE_SIZE) - 3)
#define HOURS_ADDRESS      (AT91C_IFLASH + (HOURS_PAGE * AT91C_IFLASH_PAGE_SIZE))

// parameters used to be ((AT91C_IFLASH_SIZE / AT91C_IFLASH_PAGE_SIZE) - 2)

#define UNIQUE_ID_PAGE     ((AT91C_IFLASH_SIZE / AT91C_IFLASH_PAGE_SIZE) - 1)
#define UNIQUE_ID_ADDRESS  (AT91C_IFLASH + (UNIQUE_ID_PAGE * AT91C_IFLASH_PAGE_SIZE))

typedef struct {
  unsigned long  last_timer;
  unsigned long  run_time;
} t_hours_state;


// Private routines...

void __attribute__((section(".ramfunc"))) program_page(unsigned short page, unsigned char *data, unsigned short bytes) {
  unsigned long i, d[AT91C_IFLASH_PAGE_SIZE / 4];
  unsigned long *a;

  a = (unsigned long *)(AT91C_IFLASH + (page * AT91C_IFLASH_PAGE_SIZE));

  if (memcmp(a, data, bytes) != 0) { // if it needs to be programmed
    // make a local copy for zero padding, and to ensure long word alignment
    memset(d, 0, AT91C_IFLASH_PAGE_SIZE);
    memcpy(d, data, min(AT91C_IFLASH_PAGE_SIZE, bytes));

    __disable_irq();
    AT91C_BASE_EFC->EFC_FCR = (0x5aL << 24) | (page << 8) | AT91C_EFC_FCMD_CLB; // unlock page

    while (!(AT91C_BASE_EFC->EFC_FSR & AT91C_EFC_FRDY)) {
    } /* endwhile */

    for (i = 0; i < (AT91C_IFLASH_PAGE_SIZE / 4); i ++) {
      *(a ++) = d[i];
    } /* endfor */

    AT91C_BASE_EFC->EFC_FCR = (0x5aL << 24) | (page << 8) | AT91C_EFC_FCMD_EWP; // erase & write page

    while (!(AT91C_BASE_EFC->EFC_FSR & AT91C_EFC_FRDY)) {
    } /* endwhile */

    AT91C_BASE_EFC->EFC_FCR = (0x5aL << 24) | (page << 8) | AT91C_EFC_FCMD_SLB; // lock page

    while (!(AT91C_BASE_EFC->EFC_FSR & AT91C_EFC_FRDY)) {
    } /* endwhile */

    __enable_irq();
  } /* endif */
}


// Public routines...

void set_default_parameters(void) {
  extern parameter_struct pa;
  extern unsigned long    _antichecksum;

  pa.version = _antichecksum;

  // Motion
  pa.motion.acceleration = 4;
  pa.motion.allowable_error = 4; // microsteps

  pa.motion.steps_per_unit[X_AXIS] = 133.7;
  pa.motion.steps_per_unit[Y_AXIS] = 133.7;
  pa.motion.steps_per_unit[Z_AXIS] = 6400.0;
  pa.motion.steps_per_unit[E_AXIS] = 747.0;
  pa.motion.steps_per_unit[D_AXIS] = 747.0;
  pa.motion.steps_per_unit[B_AXIS] = 2500.0;

  pa.motion.max_speed[X_AXIS] = 200.0; // NB: units/sec, not units/min
  pa.motion.max_speed[Y_AXIS] = 200.0;
  pa.motion.max_speed[Z_AXIS] = 3.5;
  pa.motion.max_speed[E_AXIS] = 35.0;
  pa.motion.max_speed[D_AXIS] = 35.0;
  pa.motion.max_speed[B_AXIS] = 6.0;

  pa.motion.invert_direction[X_AXIS] = 1;
  pa.motion.invert_direction[Y_AXIS] = 1;
  pa.motion.invert_direction[Z_AXIS] = 1;
  pa.motion.invert_direction[E_AXIS] = 0;
  pa.motion.invert_direction[D_AXIS] = 0;
  pa.motion.invert_direction[B_AXIS] = 1;

  pa.motion.invert_switch[X_AXIS] = 0;
  pa.motion.invert_switch[Y_AXIS] = 0;
  pa.motion.invert_switch[Z_AXIS] = 0;
  pa.motion.invert_switch[E_AXIS] = 1;
  pa.motion.invert_switch[D_AXIS] = 1;
  pa.motion.invert_switch[B_AXIS] = 0;

  pa.motion.motor_current[X_AXIS] = amps_to_dac(1.1);
  pa.motion.motor_current[Y_AXIS] = amps_to_dac(1.1);
  pa.motion.motor_current[Z_AXIS] = amps_to_dac(0.9);
  pa.motion.motor_current[E_AXIS] = amps_to_dac(1.3);
  pa.motion.motor_current[D_AXIS] = amps_to_dac(1.3);
  pa.motion.motor_current[B_AXIS] = 128; // exact value unimportant; selects high current

  // only relevant to E/D axes
  pa.motion.reduced_current[X_AXIS] = 0;
  pa.motion.reduced_current[Y_AXIS] = 0;
  pa.motion.reduced_current[Z_AXIS] = 0;
  pa.motion.reduced_current[E_AXIS] = amps_to_dac(0.7);
  pa.motion.reduced_current[D_AXIS] = amps_to_dac(0.7);
  pa.motion.reduced_current[B_AXIS] = 0;

  pa.motion.hold_current[X_AXIS] = amps_to_dac(0.5);
  pa.motion.hold_current[Y_AXIS] = amps_to_dac(0.5);
  pa.motion.hold_current[Z_AXIS] = amps_to_dac(0.3);
  pa.motion.hold_current[E_AXIS] = amps_to_dac(0.5);
  pa.motion.hold_current[D_AXIS] = amps_to_dac(0.5);
  pa.motion.hold_current[B_AXIS] = 13; // exact value unimportant; selects hold current

  pa.motion.filament_slip_threshold = 10.0; // mm
  pa.motion.filament_until_slip_threshold = 4.0; // mm

  // Position
  pa.position.home_distance[X_AXIS] = -0.5;
  pa.position.home_distance[Y_AXIS] = -0.5;
  pa.position.home_distance[Z_AXIS] = -0.2;
  pa.position.home_distance[E_AXIS] = 350.0;
  pa.position.home_distance[D_AXIS] = 350.0;
  pa.position.home_distance[B_AXIS] = 0.0; // unused

  pa.position.travel[X_AXIS] = 226.0;
  pa.position.travel[Y_AXIS] = 155.0;
  pa.position.travel[Z_AXIS] = 100.2;
  pa.position.travel[E_AXIS] = 0.0; // unused
  pa.position.travel[D_AXIS] = 0.0; // unused
  pa.position.travel[B_AXIS] = 7.0;

  pa.position.za_leadscrew_x_position = -37.546;
  pa.position.zb_leadscrew_x_position = 251.50;
  pa.position.filament_load_x_position = 190.0;
  pa.position.cover_unlock_y_position = 161.5;
  pa.position.z_top_switch_min_position = 79.0;
  pa.position.x_offset = 0.0;
  pa.position.y_offset = 0.0;

  // Temperature controllers
  pa.nozzle_tempcon.kp = 0.08;
  pa.nozzle_tempcon.ki = 0.012;
  pa.nozzle_tempcon.kd = 0.55;
  pa.nozzle_tempcon.beta = 4380.0;
  pa.nozzle_tempcon.tcal = 19193.0;
  pa.nozzle_tempcon.pullup = 0;
  pa.nozzle_tempcon.prtd_a = -5.775e-7; // coefficient in t^2 (IEC60751 value from Heraeus doc; confusingly they call this "B")
  pa.nozzle_tempcon.prtd_b = 3.9083e-3; // coefficient in t (IEC60751 value from Heraeus doc; confusingly they call this "A")
  pa.nozzle_tempcon.prtd_tcal = 1.0; // rpull / R0

  pa.bed_tempcon.kp = 0.05;
  pa.bed_tempcon.ki = 0.006;
  pa.bed_tempcon.kd = 1.4;
  pa.bed_tempcon.beta = 3977.0;
  pa.bed_tempcon.tcal = 62508.0;
  pa.bed_tempcon.pullup = 0;

  pa.ambient_tempcon.kp = 0.1;
  pa.ambient_tempcon.ki = 0.0;
  pa.ambient_tempcon.kd = 0.0;
  pa.ambient_tempcon.beta = 3977.0;
  pa.ambient_tempcon.tcal = 318790.0;
  pa.ambient_tempcon.pullup = 0;

  if (is_robox_pro()) {
    pa.motion.steps_per_unit[Z_AXIS] = 1600.0;
    pa.motion.max_speed[X_AXIS] = 250.0;
    pa.motion.max_speed[Y_AXIS] = 250.0;
    pa.motion.max_speed[Z_AXIS] = 8.0;
    pa.motion.motor_current[X_AXIS] = amps_to_dac(1.2);
    pa.motion.motor_current[Y_AXIS] = amps_to_dac(1.4);
    pa.motion.motor_current[Z_AXIS] = amps_to_dac(0.8);
    pa.motion.hold_current[X_AXIS] = amps_to_dac(0.4);
    pa.motion.hold_current[Y_AXIS] = amps_to_dac(0.4);
    pa.motion.hold_current[Z_AXIS] = amps_to_dac(0.25);
    pa.motion.hold_current[E_AXIS] = amps_to_dac(0.25);
    pa.motion.hold_current[D_AXIS] = amps_to_dac(0.25);

    pa.position.travel[X_AXIS] = 330.0;
    pa.position.travel[Y_AXIS] = 220.0;
    pa.position.travel[Z_AXIS] = 400.0;
    pa.position.home_distance[E_AXIS] = 510.0;
    pa.position.home_distance[D_AXIS] = 510.0;
    pa.position.za_leadscrew_x_position = -53.0;
    pa.position.zb_leadscrew_x_position = 395.0;
    pa.position.filament_load_x_position = 155.0;
    pa.position.cover_unlock_y_position = 225.0;
    pa.position.z_top_switch_min_position = 392.0;
    pa.position.x_offset = 7.5;
    pa.position.y_offset = 5.0;

    pa.bed_tempcon.beta = 3455.5;
    pa.bed_tempcon.tcal = 10900.0;
    pa.bed_tempcon.kp = 2.0;
    pa.bed_tempcon.ki = 0.7;
    pa.bed_tempcon.kd = 0.0;
  } /* endif */
}


void init_parameters(void) {
  extern parameter_struct pa;
  extern unsigned long    _antichecksum;

  if (((parameter_struct *)PARAMETERS_ADDRESS)->version == _antichecksum) { // if what's in flash is compatible
    memcpy((char *)&pa, (char *)PARAMETERS_ADDRESS, sizeof(pa)); // get parameters from flash
  } else {
    set_default_parameters();
  } /* endif */
}


void store_parameters(void) {
  extern parameter_struct pa;
  program_page(PARAMETERS_PAGE0, (unsigned char *)&pa, 256);
  program_page(PARAMETERS_PAGE1, 256 + (unsigned char *)&pa, 256);
}


void load_parameters(void) {
  extern parameter_struct pa;
  extern unsigned long    _antichecksum;

  if (((parameter_struct *)PARAMETERS_ADDRESS)->version == _antichecksum) { // if what's in flash is compatible
    memcpy((char *)&pa, (char *)PARAMETERS_ADDRESS, sizeof(pa)); // get parameters from flash
  } /* endif */
}


void show_parameters(char *response) {
  extern parameter_struct pa;
  unsigned short n;

  n = 0;
  n += sprintf(&response[n], "Ver: %08x\r\n", (int)pa.version);
  n += sprintf(&response[n], "uSteps / Unit:\r\n  M92 X%f Y%f Z%f E%f D%f B%f\r\n", pa.motion.steps_per_unit[X_AXIS], pa.motion.steps_per_unit[Y_AXIS], pa.motion.steps_per_unit[Z_AXIS], pa.motion.steps_per_unit[E_AXIS], pa.motion.steps_per_unit[D_AXIS], pa.motion.steps_per_unit[B_AXIS]);
  n += sprintf(&response[n], "Accel / allowable err:\r\n  M201 S%u T%u\r\n", pa.motion.acceleration, pa.motion.allowable_error);
  n += sprintf(&response[n], "Max Speed (mm/s):\r\n  M202 X%f Y%f Z%f E%f D%f B%f\r\n", pa.motion.max_speed[X_AXIS], pa.motion.max_speed[Y_AXIS], pa.motion.max_speed[Z_AXIS], pa.motion.max_speed[E_AXIS], pa.motion.max_speed[D_AXIS], pa.motion.max_speed[B_AXIS]);
  n += sprintf(&response[n], "Nozzle:\r\n  M301 P%e I%e D%e B%f T%f U%d\r\n", pa.nozzle_tempcon.kp, pa.nozzle_tempcon.ki, pa.nozzle_tempcon.kd, pa.nozzle_tempcon.beta, pa.nozzle_tempcon.tcal, pa.nozzle_tempcon.pullup);
  n += sprintf(&response[n], "Bed:\r\n  M302 P%e I%e D%e B%f T%f U%d\r\n", pa.bed_tempcon.kp, pa.bed_tempcon.ki, pa.bed_tempcon.kd, pa.bed_tempcon.beta, pa.bed_tempcon.tcal, pa.bed_tempcon.pullup);
  n += sprintf(&response[n], "Ambient:\r\n  M303 P%e I%e D%e B%f T%f U%d\r\n", pa.ambient_tempcon.kp, pa.ambient_tempcon.ki, pa.ambient_tempcon.kd, pa.ambient_tempcon.beta, pa.ambient_tempcon.tcal, pa.ambient_tempcon.pullup);
  n += sprintf(&response[n], "Nozzle PRTD:\r\n  M304 A%e B%e T%e\r\n", pa.nozzle_tempcon.prtd_a, pa.nozzle_tempcon.prtd_b, pa.nozzle_tempcon.prtd_tcal);
  n += sprintf(&response[n], "Axis Invert:\r\n  M510 X%d Y%d Z%d E%d D%d B%d\r\n", pa.motion.invert_direction[X_AXIS], pa.motion.invert_direction[Y_AXIS], pa.motion.invert_direction[Z_AXIS], pa.motion.invert_direction[E_AXIS], pa.motion.invert_direction[D_AXIS], pa.motion.invert_direction[B_AXIS]);
  n += sprintf(&response[n], "Axis Travel:\r\n  M520 X%f Y%f Z%f B%f\r\n", pa.position.travel[X_AXIS], pa.position.travel[Y_AXIS], pa.position.travel[Z_AXIS], pa.position.travel[B_AXIS]);
  n += sprintf(&response[n], "Switch Invert:\r\n  M526 X%d Y%d Z%d E%d D%d B%d\r\n", pa.motion.invert_switch[X_AXIS], pa.motion.invert_switch[Y_AXIS], pa.motion.invert_switch[Z_AXIS], pa.motion.invert_switch[E_AXIS], pa.motion.invert_switch[D_AXIS], pa.motion.invert_switch[B_AXIS]);
  n += sprintf(&response[n], "Home Dist:\r\n  M527 X%f Y%f Z%f E%f D%f\r\n", pa.position.home_distance[X_AXIS], pa.position.home_distance[Y_AXIS], pa.position.home_distance[Z_AXIS], pa.position.home_distance[E_AXIS], pa.position.home_distance[D_AXIS]);
  n += sprintf(&response[n], "Z Leadscrew X Pos:\r\n  M530 A%f B%f\r\n", pa.position.za_leadscrew_x_position, pa.position.zb_leadscrew_x_position);
  n += sprintf(&response[n], "Filament Load X Pos:\r\n  M531 X%f\r\n", pa.position.filament_load_x_position);
  n += sprintf(&response[n], "Unlock Y Pos:\r\n  M532 Y%f\r\n", pa.position.cover_unlock_y_position);
  n += sprintf(&response[n], "Z Top Sw Min Pos:\r\n  M533 Z%f\r\n", pa.position.z_top_switch_min_position);
  n += sprintf(&response[n], "X/Y Offsets:\r\n  M534 X%f Y%f\r\n", pa.position.x_offset, pa.position.y_offset);
  n += sprintf(&response[n], "Motor I (A):\r\n  M906 X%f Y%f Z%f E%f D%f B%f\r\n", dac_to_amps(pa.motion.motor_current[X_AXIS]), dac_to_amps(pa.motion.motor_current[Y_AXIS]), dac_to_amps(pa.motion.motor_current[Z_AXIS]), dac_to_amps(pa.motion.motor_current[E_AXIS]), dac_to_amps(pa.motion.motor_current[D_AXIS]), dac_to_amps(pa.motion.motor_current[B_AXIS]));
  n += sprintf(&response[n], "Hold I (A):\r\n  M907 X%f Y%f Z%f E%f D%f B%f\r\n", dac_to_amps(pa.motion.hold_current[X_AXIS]), dac_to_amps(pa.motion.hold_current[Y_AXIS]), dac_to_amps(pa.motion.hold_current[Z_AXIS]), dac_to_amps(pa.motion.hold_current[E_AXIS]), dac_to_amps(pa.motion.hold_current[D_AXIS]), dac_to_amps(pa.motion.hold_current[B_AXIS]));
  n += sprintf(&response[n], "Reduced I (A):\r\n  M908 E%f D%f\r\n", dac_to_amps(pa.motion.reduced_current[E_AXIS]), dac_to_amps(pa.motion.reduced_current[D_AXIS]));
  n += sprintf(&response[n], "Filament slip thr (mm):\r\n  M909 S%f T%f\r\n", pa.motion.filament_slip_threshold, pa.motion.filament_until_slip_threshold);
}


void write_unique_id(unsigned char *id) {
  program_page(UNIQUE_ID_PAGE, id, AT91C_IFLASH_PAGE_SIZE);
}


void get_unique_id(unsigned char *id) {
  unsigned short i;

  memcpy(id, (char *)UNIQUE_ID_ADDRESS, AT91C_IFLASH_PAGE_SIZE);

  for (i = 0; i < AT91C_IFLASH_PAGE_SIZE; i ++) {
    id[i] &= 0x7f; // constrain to ASCII range, just in case flash contains garbage
  } /* endfor */
}


void initialise_hours_counter(void) {
  extern t_hours_state hours_state;

  hours_state.run_time = 0;
  hours_state.last_timer = get_time();
}


void maintain_hours_counter(void) {
  extern t_hours_state hours_state;
  float                hours;
  unsigned long        now;
  char                 temp[32];

  now = get_time();

  if (is_job_running()) {
    if (!is_paused()) {
      hours_state.run_time += now - hours_state.last_timer;
    } /* endif */
  } else if (hours_state.run_time > 0) {
    memcpy(temp, (char *)HOURS_ADDRESS, 8);
    temp[8] = 0; // ensure string is terminated
    hours = strtof(temp, NULL);
    hours += (float)hours_state.run_time / (3600.0 * SYSTICK_FREQUENCY);
    sprintf(temp, "%f        ", hours);

    if (is_motion_buffer_empty()) { // need to inhibit interrupts to write flash, so don't do it during motion
      program_page(HOURS_PAGE, (unsigned char *)temp, 8);
      hours_state.run_time = 0;
    } /* endif */
  } /* endif */

  hours_state.last_timer = now;
}


void get_hours_counter(unsigned char *d) {
  extern t_hours_state hours_state;
  unsigned char        i;

  memcpy(d, (char *)HOURS_ADDRESS, 8);

  for (i = 0; i < 8; i ++) {
    d[i] &= 0x7f; // constrain to ASCII range, just in case flash contains garbage
  } /* endfor */
}


void write_hours_counter(unsigned char *d) {
  extern t_hours_state hours_state;

  program_page(HOURS_PAGE, d, 8);
  hours_state.run_time = 0;
}


unsigned char is_robox_pro(void) {
  return(memcmp((char *)UNIQUE_ID_ADDRESS, "RBX10", 5) == 0);
}


parameter_struct pa;
t_hours_state    hours_state;
