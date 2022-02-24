#ifndef POSITION_H
#define POSITION_H

#define NUM_AXIS                         6
#define NUM_EXTRUDERS                    2

#define DEFAULT_FEED_RATE                10.0    // NB: units/sec, not units/min
#define G0_G1_MAX_SEGMENT                2.0     // mm; G0/G1 moves are split into segments no greater than this, to allow bed levelling to do its thing
#define FILAMENT_LOAD_ABANDON_DISTANCE   150.0   // mm
#define FILAMENT_LOAD_SWITCH_OVERTRAVEL  5.0     // mm
#define FILAMENT_LOAD_DISTANCE_TO_SLIP   100.0   // mm
#define FILAMENT_LOAD_SPEED_TO_SLIP      6.66    // mm/sec
#define FILAMENT_UNLOAD_ABANDON_DISTANCE 550.0   // mm
#define FILAMENT_UNLOAD_DISTANCE         35.0    // mm; unload moves this distance after filament switch goes false
#define FILAMENT_SLIP_OVERTRAVEL         5.0     // mm; deceleration distance for do_move_filament_until_slip()
#define FILAMENT_SLIP_RETRY_DISTANCE     8.3     // mm; used by filament slip retry algorithm
#define DEFAULT_FILAMENT_DIAMETER        1.75    // mm
#define MAX_GANTRY_ADJUSTMENT            10.0    // mm
#define LEVELLING_MIN_X_DELTA            50.0    // mm; if Z homes were not separated by at least this much X distance, don't attempt gantry levelling
#define NUM_LEVELLING_POINTS             9       // max number of points stored for bed levelling
#define PARKING_Z_DISTANCE               2.0     // mm
#define B_POSITION_WARNING_TOLERANCE     0.15    // B position units; if discrepancy on tool change exceeds this, ERROR_B_POSITION_WARNING is reported
#define B_POSITION_LOST_TOLERANCE        0.4     // B position units; if discrepancy on tool change exceeds this, ERROR_B_POSITION_LOST is reported

typedef enum {
  NORMAL                = 0,
  LOADING_E_FILAMENT    = 1,
  UNLOADING_E_FILAMENT  = 2,
  LOADING_D_FILAMENT    = 3,
  UNLOADING_D_FILAMENT  = 4
} t_what_are_we_doing;

typedef struct {
  float          x;
  float          y;
  float          z;
} t_3d_point;

typedef struct {
  float                feed_rate;
  float                feed_rate_multiplier[2]; // one for each extruder
  unsigned char        feed_rate_multiplier_select; // keeps track of which extruder was most recently used
  unsigned char        tool;
  unsigned char        relative_moves;          // only affects X/Y/Z
  unsigned char        z_made_safe;             // shows it is safe to home X/Y
  unsigned char        homing_done[NUM_AXIS];   // E/D/B not used
  float                home_delta[NUM_AXIS];    // remembers result of home / probe; only X/Y/Z/B used
  float                destination[NUM_AXIS];   // where we should be
  float                position[NUM_AXIS];      // where we actually are when queued moves are done
  float                tool_offset[NUM_AXIS];
  float                unpark_destination[NUM_AXIS]; // remembers position on pausing, so we can return there on resuming; E/D not used
  float                unpark_feed_rate;             // remembers feed rate on pausing, so it can be reinstated on resume
  unsigned char        unpark_relative;              // remembers state of relative_moves on pausing, so it can be reinstated on resume
  unsigned char        unpark_tool;                  // remembers state of tool on pausing, so tool selection can be reinstated on resume
  t_3d_point           levelling_points[NUM_LEVELLING_POINTS];
  unsigned char        levelling_point_count;
  float                z_washout;               // the rate at which z_per_y fades as z increases; eg. z_washout = 0.2 means z_per_y has faded out completely when z gets to 5.0
  float                filament_diameter[NUM_EXTRUDERS];
  float                filament_multiplier[NUM_EXTRUDERS];
  t_what_are_we_doing  what_are_we_doing;       // used by is_unloading_filament() and to report loading / unloading filament status
} t_position_state;

typedef struct {
  float          travel[NUM_AXIS];        // for "software endstops"
  float          home_distance[NUM_AXIS]; // home switch overtravel
  float          za_leadscrew_x_position; // used in gantry levelling calculation
  float          zb_leadscrew_x_position; // used in gantry levelling calculation
  float          filament_load_x_position; // load and unload of filament done at this X position
  float          cover_unlock_y_position; // Y position for unlocking the cover
  float          z_top_switch_min_position; // the Z top switch should never be actuated at Z positions below this value
  float          x_offset; // the tool offset gets the sum of this value and the offset read from the head EEPROM; 0 for RBX01
  float          y_offset; // the tool offset gets the sum of this value and the offset read from the head EEPROM; 0 for RBX01
} t_position_parameters;


void initialise_position(void);
void do_g0_g1(char *line, unsigned char rapid);
void do_tool_change(signed long tool, char *line);
void do_homing(char *line);
void unlock_cover(char *line);
void level_gantry(void);
void level_bed(char *line);
void do_filament_load(char *line);
void do_filament_unload(char *line, unsigned char pause);
void do_move_filament_until_slip(char *line);
void park(void);
void unpark(unsigned char reselect_tool);
unsigned char do_filament_slip_retry(unsigned char extruder);
void set_position(char *line);
void select_relative_moves(unsigned char en);
void show_position(char *response);
void show_delta(unsigned char axis, char *response);
void get_positions(unsigned char *d);
void get_filament_info(unsigned char *d);
void set_feed_rate_multiplier(unsigned char extruder, unsigned char *d);
void set_filament_info(unsigned char extruder, unsigned char *d);
void get_filament_info_from_reel(void);
void update_reel_filament_length(void);
unsigned char is_loading_e_filament(void);
unsigned char is_unloading_e_filament(void);
unsigned char is_loading_d_filament(void);
unsigned char is_unloading_d_filament(void);
unsigned char get_selected_tool(void);
float get_filament_multiplier(unsigned char n);
void set_z_leadscrew_x_positions(char *line);
void set_filament_load_x_position(char *line);
void set_cover_unlock_y_position(char *line);
void set_z_top_switch_min_position(char *line);
void set_xy_offset(char *line);

#endif
