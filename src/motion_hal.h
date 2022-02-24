#ifndef MOTION_HAL_H
#define MOTION_HAL_H

#include "motion.h" // for NUM_EXTRUDERS

typedef struct {
  unsigned char gantry_rotation;
  unsigned char extruder_present[NUM_EXTRUDERS];
  unsigned char hardware_is_rev1;
} t_motion_hal_state;

// access to B axis motor driver controls
#define set_b_high_current(v)      set_head_output(7, (v)) // B axis motor driver high current select is bit 7 in shift register
#define set_b_inhibit(v)           set_head_output(6, (v)) // B axis motor driver inhibit is bit 6 in shift register
#define set_b_ms1(v)               set_head_output(1, (v)) // B axis motor driver ms1 is bit 1 in shift register
#define set_b_ms3(v)               set_head_output(0, (v)) // B axis motor driver ms3 is bit 1 in shift register

// motion HAL calls which are not specific to the hardware revision
void enable_gantry_rotation(unsigned char v);
void get_extruder_presence(unsigned char *d);

// motion HAL calls which are specific to the hardware revision
#define initialise_motors()        if (is_rev1_hw()) initialise_motors_r1(); else initialise_motors_r2()
#define make_motor_active(a, r)    if (is_rev1_hw()) make_motor_active_r1(a, r); else make_motor_active_r2(a, r)
#define make_motor_inactive(a)     if (is_rev1_hw()) make_motor_inactive_r1(a); else make_motor_inactive_r2(a)
#define turn_off_motors()          if (is_rev1_hw()) turn_off_motors_r1(); else turn_off_motors_r2()
#define motor_setdir(a, d)         if (is_rev1_hw()) motor_setdir_r1(a, d); else motor_setdir_r2(a, d)
#define motor_step(a)              if (is_rev1_hw()) motor_step_r1(a); else motor_step_r2(a)  
#define motor_unstep()             if (is_rev1_hw()) motor_unstep_r1(); else motor_unstep_r2()  
#define amps_to_dac(v)             (is_rev1_hw() ? amps_to_dac_r1(v) : amps_to_dac_r2(v))
#define dac_to_amps(v)             (is_rev1_hw() ? dac_to_amps_r1(v) : dac_to_amps_r2(v))
#define get_axis_switch(a)         (is_rev1_hw() ? get_axis_switch_r1(a) : get_axis_switch_r2(a))
#define get_index_wheel(a)         (is_rev1_hw() ? get_index_wheel_r1(a) : get_index_wheel_r2(a))
#define check_motor_drivers()      if (!is_rev1_hw()) check_motor_drivers_r2()

void initialise_motors_r1(void);
void initialise_motors_r2(void);
void make_motor_active_r1(unsigned char axis, unsigned char reduced_current);
void make_motor_active_r2(unsigned char axis, unsigned char reduced_current);
void make_motor_inactive_r1(unsigned char axis);
void make_motor_inactive_r2(unsigned char axis);
void turn_off_motors_r1(void);
void turn_off_motors_r2(void);
void motor_setdir_r1(unsigned char axis, unsigned char dir);
void motor_setdir_r2(unsigned char axis, unsigned char dir);
void motor_step_r1(unsigned char axis);
void motor_step_r2(unsigned char axis);
void motor_unstep_r1(void);
void motor_unstep_r2(void);
unsigned char amps_to_dac_r1(float amps);
unsigned char amps_to_dac_r2(float amps);
float dac_to_amps_r1(unsigned char v);
float dac_to_amps_r2(unsigned char v);
unsigned char get_axis_switch_r1(unsigned char axis);
unsigned char get_axis_switch_r2(unsigned char axis);
unsigned char get_index_wheel_r1(unsigned char axis);
unsigned char get_index_wheel_r2(unsigned char axis);
void check_motor_drivers_r2(void);

// not motion related, but here because presence of motor driver is used to detect hardware rev
void detect_hardware_rev(void);
unsigned char is_rev1_hw(void);
void get_hardware_rev(unsigned char *d);

#endif
