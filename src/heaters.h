#ifndef HEATERS_H
#define HEATERS_H

#define TEMPCON_SAMPLING_FREQUENCY   1.0   // sampling rate of temperature controllers (Hz)
#define NOZZLE_TEMP_MARGIN           0.017 // +/- range, ie. temperature between 0.983 target and 1.017 target considered "close" for M109 etc
#define BED_TEMP_MARGIN              0.03  // +/- range, ie. temperature between 0.97 target and 1.03 target considered "close" for M190 etc
#define NOZZLE_FLUSH_MARGIN_C        15.0  // (C°) +/- range used to decide whether nozzle needs flushing
#define UNLOAD_NOZZLE_TEMPERATURE    140.0 // (C°)
#define UNLOAD_NOZZLE_DWELL_TIME     5.0   // secs; if approach UNLOAD_NOZZLE_TEMPERATURE from below, need to dwell
#define NOZZLE_PAUSE_AUTO_OFF_TIME   300.0 // secs

void initialise_heaters(void);
void maintain_heaters(void);
void turn_off_heaters(void);
void pause_heaters(void);
void resume_heaters(void);
void heat_for_filament_unload(unsigned char do_nozzle0, unsigned char do_nozzle1);
void get_nozzle_targets_from_reel(char *line);
void get_bed_targets_from_reel(void);
void get_ambient_target_from_reel(void);
void get_tempcon_targets_from_reel(void);
void set_tempcon_targets(unsigned char *d);
void wait_until_nozzle_is_at_temperature(unsigned char ignore_pause);
void wait_until_bed_is_at_temperature(char *line, unsigned char ignore_pause);
void wait_until_bed_is_cool(void);
unsigned char is_head_hot(void);
void set_nozzle_temperature(char *line, unsigned char first_layer);
void set_bed_temperature(char *line, unsigned char first_layer);
void set_ambient_temperature(char *line);
void force_ambient_fan(unsigned char on);
void set_head_fan_pwm_duty(unsigned char duty);
void set_nozzle_parameters(char *line);
void set_bed_parameters(char *line);
void set_ambient_parameters(char *line);
void set_prtd_parameters(char *line);
void set_ambient_led_colour(unsigned char r, unsigned char g, unsigned char b);
void set_button_led_colour(unsigned char r, unsigned char g, unsigned char b, unsigned char no_ramp);
void show_temperatures(char *response);
void get_tempcon_states(unsigned char *d);
void get_head_power_state(unsigned char *d);
unsigned char is_head_power_on(void);
void shut_down_head_due_to_eeprom(void);
void force_nozzles_off(unsigned char force);
void enable_nozzle_forcing(void);
unsigned char is_nozzle_forcing_enabled(void);
void check_for_poweroff_whilst_hot(void);
void set_head_power(unsigned char p);

typedef enum {
  NOT_WAITING           = 0,
  BED_COOLING           = 1,
  BED_AT_TEMPERATURE    = 2,
  NOZZLE_AT_TEMPERATURE = 3
} t_why_are_we_waiting;

typedef struct {
  unsigned char mode; // 0->off; 1->use target[0]; 2->use target[1]
  float         target[2];
  float         measured;
  float         last_measured;
  float         integrator;
  unsigned char pwm_duty;
  unsigned long fault_timer;
  unsigned long unload_timer; // only used for nozzles
} t_tempcon_state;

typedef struct {
  t_tempcon_state      nozzle[2];
  t_tempcon_state      bed;
  t_tempcon_state      ambient;
  unsigned long        timer;
  unsigned char        nozzle_mode_at_pause[2]; // used by resume_heaters()
  unsigned char        force_ambient_fan;
  t_why_are_we_waiting why_are_we_waiting; // used in status report
  float                bed_excessive_slope_lpf; // provides backup 230V detection
  unsigned char        force_nozzles_off;       // ghastly work-around for hardware problem
  unsigned char        enable_nozzle_forcing;   // ghastly work-around for hardware problem
  unsigned char        timing_pause;            // indicates we're in pause and waiting to turn off nozzle heaters
  unsigned long        pause_timer;             // timer for turning off nozzle heaters in pause
} t_tempcon_states;

typedef struct {
  float         kp;
  float         ki;
  float         kd;
  float         beta; // for thermistor
  float         tcal; // for thermistor; pre-calculated as rpull / rinf
  unsigned char pullup;
  float         prtd_tcal; // for platinum rtd sensor
  float         prtd_a; // for platinum rtd sensor
  float         prtd_b; // for platinum rtd sensor
} t_tempcon_parameters;

typedef struct {
  unsigned char  head_fan_duty;
  unsigned char  head_fan_requested_duty;
  unsigned char  led_duties[6];
  unsigned short led_duties_ramped[6];
  signed short   led_rates[6];
  unsigned short led_ramp_counter;
  unsigned char  is_230v;
  unsigned char  force_230v_mode;
} t_pwm_state;

#endif
