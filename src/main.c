#include <board.h>
#include <pio/pio.h>
#include <irq/irq.h>
#include <tc/tc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "parameters.h"
#include "command.h"
#include "samadc.h"
#include "motion.h"
#include "position.h"
#include "gcode.h"
#include "file_system.h"
#include "heaters.h"
#include "motion_hal.h"
#include "eeprom.h"
#include "reprogram.h"
#include "user_interface.h"
#include "board_test.h"
#include "util.h"


// this should be called whenever the foreground is waiting on something
void do_everything_else(void) {
  maintain_motion();
  maintain_eeproms();
  maintain_file_system();
  maintain_commands();
  maintain_user_interface();
  maintain_heaters();
  maintain_hours_counter();
}


int main(void) {
  detect_hardware_rev(); // important to do this first, so that is_rev1_hw() starts to report the correct result
  initialise_timer();
  board_test(); // doesn't do anything unless board is connected to board tester
  initialise_user_interface(); // needs to precede wait_for_vmot()
  wait_for_vmot();
  init_parameters();
  initialise_hours_counter();
  initialise_eeproms();
  initialise_commands();
  initialise_job();
  init_adc();
  initialise_motors();
  initialise_heaters();
  initialise_motion();
  initialise_position();
  initialise_file_system();
  flash_checksum_test();
  check_for_poweroff_whilst_hot();

  while (1) {
    do_everything_else();
    maintain_job();
  } /* endwhile */
}
