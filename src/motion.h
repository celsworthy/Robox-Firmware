#ifndef MOTION_H
#define MOTION_H

#define NUM_AXIS                               6
#define NUM_EXTRUDERS                          2

#define X_AXIS                                 0
#define Y_AXIS                                 1
#define Z_AXIS                                 2
#define E_AXIS                                 3
#define D_AXIS                                 4
#define B_AXIS                                 5

#define BLOCK_BUFFER_SIZE                      16 // must be power of 2
#define MIN_SPEED                              2
#define SPEED_UNITS                            (BOARD_MCK / (65536.0 * 32.0)) // steps/sec

#define FILAMENT_AUTOLOAD_THRESHOLD            8   // encoder edges
#define FILAMENT_HOLDOFF_TIME                  4.0 // secs; prevents index wheel from triggering filament load immediately after unload
#define Z_SAFE_RAISE_DISTANCE                  3.0   // mm; initial distance Z is raised if Z top switch not actuated
#define Z_SAFE_LOWER_DISTANCE                  -20.0 // mm; initial distance Z is lowered if Z top switch is actuated


// if compiler optimization is not on, we must reduce MAX_SPEED to avoid getting swamped by motion interrupts
#ifdef OPTIMIZATION
#define MAX_SPEED          1200
#else
#define MAX_SPEED          600
#endif


typedef enum {
  MOTION_NORMAL        = 0,
  MOTION_LOADING       = 1,
  MOTION_UNLOADING     = 2,
  MOTION_UNTIL_SLIP    = 3,
} t_what_motion_is_doing;


typedef struct {
  unsigned long          length;
  unsigned short         speed;
  unsigned short         end_speed;
  signed long            inc[NUM_AXIS];
  unsigned char          check_z_top_switch;
} t_block;


typedef struct {
  unsigned char          acceleration;
  unsigned char          allowable_error;
  float                  steps_per_unit[NUM_AXIS];
  float                  max_speed[NUM_AXIS];
  unsigned char          invert_direction[NUM_AXIS];
  unsigned char          invert_switch[NUM_AXIS];
  unsigned char          motor_current[NUM_AXIS]; // used when moving
  unsigned char          reduced_current[NUM_AXIS]; // only relevant to E/D axes
  unsigned char          hold_current[NUM_AXIS];  // used to hold position when axis idle
  float                  filament_slip_threshold;       // normal filament slip threshold
  float                  filament_until_slip_threshold; // filament slip threshold used during move until slip
} t_motion_parameters;


typedef struct {
  // per-block stuff used by isr
  unsigned long          speed_x4k;
  unsigned short         timer_period;
  unsigned long          acc[NUM_AXIS];
  unsigned long          switch_abandon_counter;
  unsigned char          switch_select;
  unsigned char          switch_invert;
  unsigned char          slip_select;            // selects sensitivity to filament slip in addition to switch
  unsigned short         max_period_for_ramping; // a function of acceleration; pre-calculated for efficiency
  unsigned long          max_speed_step;         // pre-calculated as a function of acceleration and allowable_error

  // buffer
  unsigned char          buffer_in;
  volatile unsigned char buffer_out; // "volatile" is very important as this is incremented in an ISR and polled elsewhere
  t_block                block[BLOCK_BUFFER_SIZE];

  // misc
  unsigned char          filament_autoload_counter[NUM_EXTRUDERS];
  unsigned char          filament_slip_error[NUM_EXTRUDERS];
  unsigned char          index_wheel_moved[NUM_EXTRUDERS];
  unsigned char          filament_slip;
  t_what_motion_is_doing what_are_we_doing;
  unsigned long          filament_holdoff_timer; // prevents index wheel triggered filament load immediately after unload
  unsigned char          z_top_switch_deglitch_counter;
} t_motion_state;


extern const char axis_codes[];


void set_acceleration(char *line);
void initialise_motion(void);
void maintain_motion(void);
void add_block_to_buffer(float feed_rate, float speed_limit, unsigned char check_z_top_switch, unsigned char reduced_current, float moves[]);
void TC0_IrqHandler(void);
void wait_until_buffer_empty(void);
unsigned char is_motion_buffer_empty(void);
float home_axis(unsigned char axis, float home_distance, float abandon_distance);
void make_z_safe(void);
void rotate_gantry(float distance);
float select_nozzle(float distance, float abandon_distance);
unsigned char grab_filament(unsigned char axis, float switch_overtravel, float abandon_distance);
unsigned char load_filament(unsigned char axis, float switch_overtravel, float distance);
void unload_filament(unsigned char axis, float overtravel_distance, float abandon_distance);
float move_filament_until_slip(unsigned char axis, float overtravel_distance, float abandon_distance, float speed_limit);
unsigned char is_filament_slip_error(unsigned char extruder);
unsigned char has_index_wheel_moved(unsigned char extruder);
void show_switch_state(char *response);
void get_switch_states(unsigned char *d);
unsigned char get_selected_nozzle(void);

#endif
