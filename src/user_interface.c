#include <board.h>
#include <pio/pio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "command.h"
#include "eeprom.h"
#include "position.h"
#include "gcode.h"
#include "heaters.h"
#include "motion.h"
#include "motion_hal.h"
#include "position.h"
#include "util.h"
#include "user_interface.h"

#define BUTTON_DEBOUNCE_TIME       0.03 // secs
#define BUTTON_LONG_PRESS_TIME     0.8  // secs
#define BUTTON_DOUBLE_PRESS_TIME   0.3 // secs
#define LED_THROB_PERIOD           1.0 // secs

typedef struct {
  unsigned long button_timer;
  unsigned long led_throb_timer;
  unsigned char last_button;
  unsigned char pending;
  unsigned char been_unpressed;
  unsigned char pressed_at_start;
  unsigned char ambient_colour_r;
  unsigned char ambient_colour_g;
  unsigned char ambient_colour_b;
  unsigned char override_button_led;
  unsigned char flipper;
} t_ui_state;

const Pin BUTTON[] =    {{1 << 29, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP},  // post rev 1
                         {1 << 30, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP}}; // rev 1


// Should be called as start-up.  Configures the i/o ports and initialises ui_state.
void initialise_user_interface(void) {
  extern t_ui_state ui_state;

  PIO_Configure(&BUTTON[is_rev1_hw()], 1);
  ui_state.pending = 0;
  ui_state.last_button = 0;
  ui_state.been_unpressed = 0;

  // set pressed_at_start if the button is pressed and the last reset was power-on (as opposed to software-triggered)
  delay(1e-3); // make sure button pull-up has had time to do its thing
  ui_state.pressed_at_start = !PIO_Get(&BUTTON[is_rev1_hw()]) && ((AT91C_BASE_RSTC->RSTC_RSR & AT91C_RSTC_RSTTYP) == AT91C_RSTC_RSTTYP_GENERAL);

  ui_state.button_timer = ui_state.led_throb_timer = get_time();

  // default to white
  ui_state.ambient_colour_r = 0xff;
  ui_state.ambient_colour_g = 0xd7;
  ui_state.ambient_colour_b = 0x64;

  ui_state.override_button_led = 0;
  ui_state.flipper = 0;
}


// Should be called regularly.  Checks the button - a short button press toggles pause state, whereas a long
// button press pauses the job then unloads the E filament.  If no job is running, then a double short button
// press runs the newest job file.  Note the use of been_unpressed so that holding the button at start-up is
// ignored here, though obviously it affects was_button_pressed_at_start().
// Also handles the button LED.
void maintain_user_interface(void) {
  extern t_ui_state ui_state;

  if (PIO_Get(&BUTTON[is_rev1_hw()]) != ui_state.last_button) {
    ui_state.button_timer = get_time();
  } /* endif */

  if (has_period_elapsed(ui_state.button_timer, BUTTON_DEBOUNCE_TIME)) {
    if (PIO_Get(&BUTTON[is_rev1_hw()])) { // not pressed
      if (is_job_running()) {
        if (ui_state.pending) {
          if (is_paused()) {
            clear_resumable_errors(); // allow certain errors to be cleared by resume from the button
          } /* endif */

          invert_pause();
          ui_state.pending = 0;
        } /* endif */
      } else {
        if (ui_state.pending > 1) {
          if (is_head_power_on() && !is_unloading_e_filament() && !is_unloading_d_filament() &&
              is_reel0_present() && (is_reel1_present() || !is_head_dual_material())) {
            start_job((unsigned char *)""); // empty file id means open job file most recently run
          } /* endif */

          ui_state.pending = 0;
        } /* endif */
      } /* endif */

      if (has_period_elapsed(ui_state.button_timer, BUTTON_DOUBLE_PRESS_TIME)) {
        ui_state.pending = 0;
      } /* endif */

      ui_state.been_unpressed = 1;
    } else { // pressed
      if (!has_period_elapsed(ui_state.button_timer, BUTTON_LONG_PRESS_TIME)) {
        ui_state.pending += ui_state.been_unpressed;
        ui_state.been_unpressed = 0;
      } else if (ui_state.pending) {
        if (!is_unloading_e_filament() && !is_unloading_d_filament()) {
          if (is_reel1_present()) {
            execute_gcode((unsigned char *)"M121 D\n", 7); // unload D filament
          } else {
            execute_gcode((unsigned char *)"M121 E\n", 7); // unload E filament
          } /* endif */
        } /* endif */

        ui_state.pending = 0;
      } /* endif */
    } /* endif */
  } /* endif */

  if (!is_non_selfie_paused()) {
    set_ambient_led_colour(ui_state.ambient_colour_r, ui_state.ambient_colour_g, ui_state.ambient_colour_b); // user-selected colour
  } else if (!has_period_elapsed(ui_state.led_throb_timer, LED_THROB_PERIOD * 0.5)) {
    set_ambient_led_colour(0, 0, 0); // off
  } else if (ui_state.flipper) {
    set_ambient_led_colour(ui_state.ambient_colour_r, ui_state.ambient_colour_g, ui_state.ambient_colour_b); // user-selected colour
  } else if (is_error()) {
    set_ambient_led_colour(0xff, 0, 0); // red
  } else {
    set_ambient_led_colour(0, 0xff, 0); // green
  } /* endif */

  if (!ui_state.override_button_led) {
    if (is_unloading_e_filament() || is_unloading_d_filament()) {
      if (has_period_elapsed(ui_state.led_throb_timer, LED_THROB_PERIOD * 0.5)) {
        set_button_led_colour(0, 0, 0xff, 0); // blue
      } else {
        set_button_led_colour(0, 0, 0, 0); // off
      } /* endif */
    } else if (!PIO_Get(&BUTTON[is_rev1_hw()])) { // button pressed
      set_button_led_colour(0, 0, 0xff, 1); // blue; no_ramp for instant response
    } else if (!ui_state.last_button) { // button just released
      set_button_led_colour(0, 0, 0, 1); // off; no_ramp for instant response
    } else if (is_job_running() && is_error()) {
      if (!is_paused() || has_period_elapsed(ui_state.led_throb_timer, LED_THROB_PERIOD * 0.5)) {
        set_button_led_colour(0xff, 0, 0, 0); // red
      } else {
        set_button_led_colour(0, 0, 0, 0); // off
      } /* endif */
    } else if (is_job_running()) {
      if (!is_non_selfie_paused() || has_period_elapsed(ui_state.led_throb_timer, LED_THROB_PERIOD * 0.5)) {
        set_button_led_colour(0, 0xff, 0, 0); // green
      } else {
        set_button_led_colour(0, 0, 0, 0); // off
      } /* endif */
    } else {
      set_button_led_colour(0, 0, 0, 0); // off
    } /* endif */
  } /* endif */

  if (has_period_elapsed(ui_state.led_throb_timer, LED_THROB_PERIOD)) {
    ui_state.led_throb_timer = get_time();
    ui_state.flipper = !ui_state.flipper;
  } /* endif */

  ui_state.last_button = PIO_Get(&BUTTON[is_rev1_hw()]);
}


// Implements the set_ambient_led Robox command.
void set_ambient_led(unsigned char *d) {
  extern t_ui_state ui_state;

  ui_state.ambient_colour_r = get_hex(&d[0], 2);
  ui_state.ambient_colour_g = get_hex(&d[2], 2);
  ui_state.ambient_colour_b = get_hex(&d[4], 2);
}


// Implements the set_button_led Robox command.
void set_button_led(unsigned char *d) {
  extern t_ui_state ui_state;

  ui_state.override_button_led = 1;
  set_button_led_colour(get_hex(&d[0], 2), get_hex(&d[2], 2), get_hex(&d[4], 2), 0);
}


// Global hiding routine used in the assembly of a Robox status report.
void get_button_state(unsigned char *d) {
  *d = PIO_Get(&BUTTON[is_rev1_hw()]) ? '1' : '0';
}


// Global hiding routine to find whether the button was pressed at power-on reset
unsigned char was_button_pressed_at_start(void) {
  extern t_ui_state ui_state;
  return(ui_state.pressed_at_start);
}


t_ui_state ui_state;
