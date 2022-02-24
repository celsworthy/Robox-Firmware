#include <board.h>
#include <pio/pio.h>
#include <irq/irq.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "util.h"
#include "gcode.h"
#include "file_system.h"
#include "eeprom.h"
#include "motion.h"
#include "motion_hal.h"
#include "heaters.h"
#include "reprogram.h"
#include "board.h"
#include "parameters.h"
#include "user_interface.h"
#include "board_test.h"
#include "command.h"


typedef enum {
  USB_DISCONNECTED = 0,
  USB_PENDING      = 1,
  USB_CONNECTED    = 2
} t_usb_state;

// Size in bytes of the buffers used for reading data from / sending data to the USB
#define RX_BUFFER_SIZE    1024
#define TX_BUFFER_SIZE    1024

typedef struct {
  unsigned char  rx_buffer[RX_BUFFER_SIZE + 1]; // extra byte is space for terminating strings with \0
  unsigned short rx_byte_count;
  unsigned short received;
  unsigned char  robox_mode;
  unsigned char  tx_buffer[TX_BUFFER_SIZE];
  unsigned short tx_byte_count;
  uint64_t       error_flags;
  unsigned long  timer;
  t_usb_state    usb_state;
} t_command_state;


const Pin  USB_VBUS_PIN =              {1 << 19, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_DEFAULT}; // detects USB power
const Pin  VMOT_PIN =                  {1 << 5,  AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT}; // detects 24V supply
const Pin  USB_HUB_NRESET_R2 =         {1 << 30, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}; // not present on rev 1 hardware


// Invoked when the USB device leaves the Suspended state.
void USBDCallbacks_Resumed(void) {
}

// Invoked when the USB device gets suspended.
void USBDCallbacks_Suspended(void) {
}


// Callback when USB data received.
static void cb_usb_data_received(unsigned int unused, unsigned char status, unsigned int received, unsigned int remaining) {
  extern t_command_state command_state;

  if (status == USBD_STATUS_SUCCESS) { // Check that data has been received successfully
    command_state.received = received;
  } else {
    report_error(ERROR_USB_RX);
  } /* endif */
}


// Callback when USB tx completed.
static void cb_usb_write_complete(void* pArg, unsigned char status, unsigned int received, unsigned int remaining) {
  extern t_command_state command_state;

  command_state.tx_byte_count = 0;

  if (status != USBD_STATUS_SUCCESS) {
    report_error(ERROR_USB_TX);
  } /* endif */
}


// Transmits the first tx_byte_count bytes of tx_buffer on USB.
void usb_send(void) {
  extern t_command_state command_state;

  if (command_state.tx_byte_count > 0) {
    if (CDCDSerialDriver_Write((void *)command_state.tx_buffer, command_state.tx_byte_count, cb_usb_write_complete, 0) != USBD_STATUS_SUCCESS) {
      report_error(ERROR_USB_TX);
      command_state.tx_byte_count = 0;
    } /* endif */
  } /* endif */
}


// This should not be used in normal operation; it's specifically for commands which cause reset,
// where we want to ensure the response has been sent on USB before we reset the processor.
// Commands which cause reset are the update_firmware command, and (you guessed it) the reset command.
void wait_until_usb_tx_complete(void) {
  extern t_command_state command_state;
  unsigned long timer;

  timer = get_time();

  while (((volatile unsigned short)command_state.tx_byte_count > 0) && !has_period_elapsed(timer, 0.5)) {
  } /* endwhile */
}


// Permits read / write of any address.  Definitely for debug purposes only!
// n represents the length of d[], so it is double the number of bytes to write.
static void do_memory_rw(unsigned long address, unsigned short n, unsigned char *d) {
  extern t_command_state command_state;
  unsigned short         i;
  unsigned char          *p;

  // do write, if any
  p = (unsigned char *)address;
  n = n >> 1; // number of bytes to write

  for (i = 0; i < n; i ++) {
    *p = (unsigned char)get_hex(d, 2);
    p ++;
    d += 2;
  } /* endfor */

  // do read
  p = (unsigned char *)address;

  if (command_state.tx_byte_count != 0) {
    report_error(ERROR_USB_TX);
  } /* endif */

  command_state.tx_buffer[0] = REPORT_MEMORY_RW;

  for (i = 1; i <= 257; i += 2) {
    put_hex(*p, 2, &command_state.tx_buffer[i]);
    p ++;
  } /* endfor */

  command_state.tx_byte_count = 257;
  usb_send();
}


// It's intended that this routine can be easily hacked to make any globals observable for debug purposes.
// The usual rule of not accessing variables belonging to other "objects" is waived here.
static void report_bens_info(void) {
  extern t_command_state  command_state;
  extern t_position_state position_state; // for now, the thing we want to observe
  extern t_tempcon_states tempcon_states; // for now, the thing we want to observe
  unsigned short i, n;

  if (command_state.tx_byte_count != 0) {
    report_error(ERROR_USB_TX);
  } /* endif */

  memset(command_state.tx_buffer, 0, 257);
  command_state.tx_buffer[0] = REPORT_BENS_INFO;

  n = 1;
  n += sprintf((char *)&command_state.tx_buffer[n], "ghastly:%d ", tempcon_states.enable_nozzle_forcing);

  for (i = 0; i < position_state.levelling_point_count; i ++) {
    n += sprintf((char *)&command_state.tx_buffer[n], "z[%d]:%f ", i, position_state.levelling_points[i].z);
  } /* endfor */

  command_state.tx_byte_count = 257;
  usb_send();
}


// Reports debug info from board test.
static void report_debug_info(void) {
  extern t_command_state  command_state;

  if (command_state.tx_byte_count != 0) {
    report_error(ERROR_USB_TX);
  } /* endif */

  command_state.tx_buffer[0] = REPORT_DEBUG_INFO;
  get_board_test_report(&command_state.tx_buffer[1]);
  command_state.tx_byte_count = 257;
  usb_send();
}


// Issues a status report.  Various global hiding routines are called to gather the information.
static void report_status(void) {
  extern t_command_state command_state;

  if (command_state.tx_byte_count != 0) {
    report_error(ERROR_USB_TX);
  } /* endif */

  command_state.tx_buffer[0] = REPORT_STATUS;

  get_read_file_id(&command_state.tx_buffer[1]);
  get_job_state(&command_state.tx_buffer[17]);

  get_switch_states(&command_state.tx_buffer[27]);
  get_extruder_presence(&command_state.tx_buffer[38]);
  get_button_state(&command_state.tx_buffer[34]);
  get_tempcon_states(&command_state.tx_buffer[40]);

  get_eeprom_states(&command_state.tx_buffer[134]);
  get_sd_card_state(&command_state.tx_buffer[138]);

  get_positions(&command_state.tx_buffer[139]);
  get_filament_info(&command_state.tx_buffer[172]);

  get_head_power_state(&command_state.tx_buffer[220]);
  get_hardware_rev(&command_state.tx_buffer[221]);

  command_state.tx_byte_count = 222;
  usb_send();
}


static void report_sd_card_state(void) {
  extern t_command_state command_state;

  if (command_state.tx_byte_count != 0) {
    report_error(ERROR_USB_TX);
  } /* endif */

  command_state.tx_buffer[0] = REPORT_SD_CARD_STATE;
  get_sd_card_state(&command_state.tx_buffer[1]);

  command_state.tx_byte_count = 2;
  usb_send();
}


// Issues a file list report, which lists the files on the SD card.
static void report_file_list(void) {
  extern t_command_state command_state;

  if (command_state.tx_byte_count != 0) {
    report_error(ERROR_USB_TX);
  } /* endif */

  command_state.tx_buffer[0] = REPORT_FILE_LIST;
  command_state.tx_byte_count = get_file_list(&command_state.tx_buffer[3]);
  put_hex(command_state.tx_byte_count >> 4, 2, &command_state.tx_buffer[1]); // length parameter
  command_state.tx_byte_count += 3;
  usb_send();
}


// Issues a send file report, which indicates the id of the file open for writing, and
// the next chunk number.
static void report_send_file(void) {
  extern t_command_state command_state;

  if (command_state.tx_byte_count != 0) {
    report_error(ERROR_USB_TX);
  } /* endif */

  command_state.tx_buffer[0] = REPORT_SEND_FILE;
  get_send_file_report(&command_state.tx_buffer[1]);
  command_state.tx_byte_count += 25;
  usb_send();
}


// Issues an EEPROM report, which shows the contents of the head EEPROM (eeprom_sel = 0)
// or the reel EEPROM (eeprom_sel = 1).
static void report_eeprom(unsigned char eeprom_sel) {
  extern t_command_state command_state;

  if (command_state.tx_byte_count != 0) {
    report_error(ERROR_USB_TX);
  } /* endif */

  command_state.tx_buffer[0] = (eeprom_sel == 2) ? REPORT_REEL1_EEPROM : ((eeprom_sel == 1) ? REPORT_REEL0_EEPROM : REPORT_HEAD_EEPROM);
  command_state.tx_byte_count = get_eeprom_read(eeprom_sel, &command_state.tx_buffer[1]);
  command_state.tx_byte_count += 1;
  usb_send();
}


// Issues an ack/errors report, which simply contains the error flags.  In addition to
// being used for a report_errors command, it's also used as a general purpose acknowledgement
// for commands which do not return data via one of the other report types - hence the name.
// Note the resetting of timer, which is used to detect that Automaker is connected and is
// being attentive to errors.
static void report_ack_errors(void) {
  extern t_command_state command_state;
  unsigned char i;

  command_state.timer = get_time();

  if (command_state.tx_byte_count != 0) {
    report_error(ERROR_USB_TX);
  } /* endif */

  command_state.tx_buffer[0] = REPORT_ACK_ERRORS;

  for (i = 0; i < 64; i ++) {
    command_state.tx_buffer[1 + i] = (command_state.error_flags & (((uint64_t)1) << i)) ? '1' : '0';
  } /* endfor */

  command_state.tx_byte_count = 65;
  usb_send();
}


// Issues a report of the firmware revision.  It's important that the format of this report
// remains unchanged, so the software can always detect the firmware revision at start-up.
// It can then decide to update the firmware if necessary.
static void report_firmware_revision(void) {
  extern t_command_state command_state;

  if (command_state.tx_byte_count != 0) {
    report_error(ERROR_USB_TX);
  } /* endif */

  command_state.tx_buffer[0] = REPORT_FIRMWARE_REVISION;
  memset(&command_state.tx_buffer[1], 0, 8);
  strcpy((char *)&command_state.tx_buffer[1], FIRMWARE_REVISION);
  command_state.tx_byte_count = 9;
  usb_send();
}


// Reports the unique ID string which is stored in the flash.
static void report_unique_id(void) {
  extern t_command_state command_state;

  if (command_state.tx_byte_count != 0) {
    report_error(ERROR_USB_TX);
  } /* endif */

  command_state.tx_buffer[0] = REPORT_UNIQUE_ID;
  get_unique_id(&command_state.tx_buffer[1]);
  command_state.tx_byte_count = 257;
  usb_send();
}


static void report_hours_counter(void) {
  extern t_command_state command_state;

  if (command_state.tx_byte_count != 0) {
    report_error(ERROR_USB_TX);
  } /* endif */

  command_state.tx_buffer[0] = REPORT_HOURS_COUNTER;
  get_hours_counter(&command_state.tx_buffer[1]);
  command_state.tx_byte_count = 9;
  usb_send();
}


// Issues a gcode_response report, responding to an execute_gcode command.  Is also
// involved in the execution of "reporting" Gcode commands.
// When execute_gcode is used to execute a "non-reporting" Gcode command, that is, a Gcode
// command which makes no response, the command is queued.  This routine is called with n=0
// and an empty gcode_response report is issued.  The gcode_response report indicates that
// the command has been queued, not that it has finished executing.
// However, if execute_gcode is used to execute a "reporting" Gcode command, it's passed in
// with n indicating the length of the string.  We call gcode_handler() to execute the
// Gcode command straight away, and report the response it returns.
// This approach only works because it happens that all reporting Gcode commands 1] complete
// immediately, and 2] do not affect the state of the Robox.  Consequently, it is safe to
// bypass the queuing system which is used for non-reporting commands.
// Arguably, reporting Gcode commands are not needed in Robox mode, as we could easily define
// a Robox command to return whatever information is required.  Reporting Gcode commands
// might still be desirable in non-Robox (Pronterface compatible) mode, but in this case
// the Robox command / report system is bypassed anyway.
static void report_gcode(unsigned char *gcode, unsigned short n) {
  extern t_command_state command_state;
  unsigned char temp;

  if (command_state.tx_byte_count != 0) {
    report_error(ERROR_USB_TX);
  } /* endif */

  command_state.tx_buffer[0] = REPORT_GCODE_RESPONSE;

  if (n) {
    temp = gcode[n];
    gcode[n] = 0; // ensure string is terminated
    gcode_handler((char *)gcode, (char *)&command_state.tx_buffer[5]);
    gcode[n] = temp;
    command_state.tx_byte_count = strlen((char *)&command_state.tx_buffer[5]);
  } else {
    command_state.tx_byte_count = 0;
  } /* endif */

  put_hex(command_state.tx_byte_count, 4, &command_state.tx_buffer[1]);
  command_state.tx_byte_count += 5;
  usb_send();
}


// Simple global-hiding routine to clear all error flags.
static void clear_errors(void) {
  extern t_command_state command_state;

  command_state.error_flags = 0L;
}


// Checks the first l bytes of b[] to see if they contain a complete Robox command.  If so,
// the command is executed and the return value indicates the number of bytes "consumed".
// If there's an incomplete Robox command, then we return zero, indicating that no bytes
// have been consumed yet (the command can be executed later, once the rest of it has
// arrived).
// If l is non-zero and b[0] doesn't contain a valid command code, we report a bad_command
// error and return 1, so that the bad command byte is "consumed".  Since the parameter
// bytes of a Robox command are always ASCII characters < 0x80, whereas Robox command
// codes are all >= 0x80, synchronisation is automatically re-established after an error
// (which is not to say that errors should be expected).
static unsigned short robox_command_handler(unsigned char b[], unsigned short l) {
  unsigned short n;

  if (l == 0) {
    return(0);
  } /* endif */

  if (b[0] == COMMAND_SHOW_STATUS) {
    report_status();
    return(1);
  } else if (b[0] == COMMAND_REPORT_ERRORS) {
    report_ack_errors();
    return(1);
  } else if (b[0] == COMMAND_CLEAR_ERRORS) {
    clear_errors();
    report_ack_errors();
    return(1);
  } else if (b[0] == COMMAND_REPORT_FIRMWARE_REVISION) {
    report_firmware_revision();
    return(1);
  } else if (b[0] == COMMAND_LIST_FILES) {
    report_file_list();
    return(1);
  } else if (b[0] == COMMAND_FORMAT_SD_CARD) {
    format_sd_card();
    report_ack_errors();
    return(1);
  } else if (b[0] == COMMAND_ABORT_JOB) {
    abort_job();
    report_ack_errors();
    return(1);
  } else if (b[0] == COMMAND_PAUSE_JOB) {
    if (l >= 2) {
      pause_resume_job(&b[1]);
      report_ack_errors();
      return(2);
    } else {
      return(0);
    } /* endif */
  } else if ((b[0] == COMMAND_START_FILE) || (b[0] == COMMAND_START_JOB_FILE)) {
    if (l >= 17) {
      start_write_file(&b[1], (b[0] == COMMAND_START_JOB_FILE));
      report_ack_errors();
      return(17);
    } else {
      return(0);
    } /* endif */
  } else if (b[0] == COMMAND_FILE_CHUNK) {
    if (l >= 521) {
      write_file(get_hex(&b[1], 8), &b[9]);
      report_ack_errors();
      return(521);
    } else {
      return(0);
    } /* endif */
  } else if (b[0] == COMMAND_END_FILE) {
    if (l >= 13) {
      n = (unsigned short)get_hex(&b[9], 4); // length parameter

      if (l >= (n + 13)) {
        end_write_file(get_hex(&b[1], 8), n, &b[13]);
        report_ack_errors();
        return(n + 13);
      } else {
        return(0);
      } /* endif */
    } else {
      return(0);
    } /* endif */
  } else if (b[0] == COMMAND_READ_SEND_FILE_REPORT) {
    report_send_file();
    return(1);
  } else if (b[0] == COMMAND_RUN_JOB) {
    if (l >= 17) {
      start_job(&b[1]);
      report_ack_errors();
      return(17);
    } else {
      return(0);
    } /* endif */
  } else if (b[0] == COMMAND_UPDATE_FIRMWARE) {
    if (l >= 17) {
      if (is_firmware_file_ok(&b[1])) {
        report_ack_errors();
        wait_until_usb_tx_complete();
        reprogram(&b[1]); // doesn't return
      } else {
        report_ack_errors(); // will include a bad firmware file error
      } /* endif */

      return(17);
    } else {
      return(0);
    } /* endif */
  } else if (b[0] == COMMAND_RESET) {
    report_ack_errors();
    wait_until_usb_tx_complete();
    delay(50e-3);
    AT91C_BASE_RSTC->RSTC_RCR = AT91C_RSTC_KEY | AT91C_RSTC_PERRST | AT91C_RSTC_PROCRST; // reset
    return(1); // should never be reached
  } else if (b[0] == COMMAND_EXECUTE_GCODE) {
    if (l >= 5) {
      n = (unsigned short)get_hex(&b[1], 4); // length parameter

      if (l >= (n + 5)) {
        if (!is_gcode_reporting(&b[5], n)) {
          execute_gcode(&b[5], n); // queue it up
          report_gcode(NULL, 0); // empty response
        } else {
          report_gcode(&b[5], n); // do it now and return response
        } /* endif */

        return(n + 5);
      } else {
        return(0);
      } /* endif */
    } else {
      return(0);
    } /* endif */
  } else if ((b[0] == COMMAND_WRITE_HEAD_EEPROM) || (b[0] == COMMAND_WRITE_REEL0_EEPROM) || (b[0] == COMMAND_WRITE_REEL1_EEPROM)) {
    if (l >= 5) {
      n = get_hex(&b[3], 2); // length parameter

      if (l >= (5 + n)) {
        write_eeprom((b[0] == COMMAND_WRITE_REEL1_EEPROM) ? 2 : ((b[0] == COMMAND_WRITE_REEL0_EEPROM) ? 1 : 0), get_hex(&b[1], 2), n, &b[5]);
        report_ack_errors();
        return(5 + n);
      } else {
        return(0);
      } /* endif */
    } else {
      return(0);
    } /* endif */
  } else if ((b[0] == COMMAND_READ_HEAD_EEPROM) || (b[0] == COMMAND_READ_REEL0_EEPROM) || (b[0] == COMMAND_READ_REEL1_EEPROM)) {
    report_eeprom((b[0] == COMMAND_READ_REEL1_EEPROM) ? 2 : ((b[0] == COMMAND_READ_REEL0_EEPROM) ? 1 : 0));
    return(1);
  } else if ((b[0] == COMMAND_FORMAT_HEAD_EEPROM) || (b[0] == COMMAND_FORMAT_REEL0_EEPROM) || (b[0] == COMMAND_FORMAT_REEL1_EEPROM)) {
    format_eeprom((b[0] == COMMAND_FORMAT_REEL1_EEPROM) ? 2 : ((b[0] == COMMAND_FORMAT_REEL0_EEPROM) ? 1 : 0));
    report_ack_errors();
    return(1);
  } else if (b[0] == COMMAND_READ_UNIQUE_ID) {
    report_unique_id();
    return(1);
  } else if (b[0] == COMMAND_WRITE_UNIQUE_ID) {
    if (l >= 257) {
      write_unique_id(&b[1]);
      report_ack_errors();
      return(257);
    } else {
      return(0);
    } /* endif */
  } else if (b[0] == COMMAND_READ_HOURS_COUNTER) {
    report_hours_counter();
    return(1);
  } else if (b[0] == COMMAND_SHOW_SD_CARD_STATE) {
    report_sd_card_state();
    return(1);
  } else if (b[0] == COMMAND_WRITE_HOURS_COUNTER) {
    if (l >= 9) {
      write_hours_counter(&b[1]);
      report_ack_errors();
      return(9);
    } else {
      return(0);
    } /* endif */
  } else if (b[0] == COMMAND_AMBIENT_LED) {
    if (l >= 7) {
      set_ambient_led(&b[1]);
      report_ack_errors();
      return(7);
    } else {
      return(0);
    } /* endif */
  } else if (b[0] == COMMAND_BUTTON_LED) {
    if (l >= 7) {
      set_button_led(&b[1]);
      report_ack_errors();
      return(7);
    } else {
      return(0);
    } /* endif */
  } else if (b[0] == COMMAND_SET_TEMPERATURES) {
    if (l >= 57) {
      set_tempcon_targets(&b[1]);
      report_ack_errors();
      return(57);
    } else {
      return(0);
    } /* endif */
  } else if ((b[0] == COMMAND_SET_E_FEED_RATE_MULTIPLIER) || (b[0] == COMMAND_SET_D_FEED_RATE_MULTIPLIER)) {
    if (l >= 9) {
      set_feed_rate_multiplier((b[0] == COMMAND_SET_D_FEED_RATE_MULTIPLIER), &b[1]);
      report_ack_errors();
      return(9);
    } else {
      return(0);
    } /* endif */
  } else if ((b[0] == COMMAND_SET_E_FILAMENT_INFO) || (b[0] == COMMAND_SET_D_FILAMENT_INFO)) {
    if (l >= 17) {
      set_filament_info((b[0] == COMMAND_SET_D_FILAMENT_INFO), &b[1]);
      report_ack_errors();
      return(17);
    } else {
      return(0);
    } /* endif */
  } else if (b[0] == COMMAND_SET_HEAD_POWER) {
    if (l >= 2) {
      set_head_power(b[1] == '1');
      report_ack_errors();
      return(2);
    } else {
      return(0);
    } /* endif */
  } else if (b[0] == COMMAND_READ_DEBUG_INFO) {
    report_debug_info();
    return(1);
  } else if (b[0] == COMMAND_MEMORY_RW) {
    if (l >= 13) {
      n = (unsigned short)get_hex(&b[9], 4); // length parameter

      if (l >= (n + 13)) {
        do_memory_rw(get_hex(&b[1], 8), n, &b[13]);
        return(n + 13);
      } else {
        return(0);
      } /* endif */
    } else {
      return(0);
    } /* endif */
  } else if (b[0] == COMMAND_READ_BENS_INFO) {
    report_bens_info();
    return(1);
  } /* endif */

  // if we get here, it's probably because we've lost command alignment - so just skip to the next byte...
  report_error(ERROR_BAD_COMMAND);
  return(1);
}


// Public routines...

// Should be called once at start-up, before maintain_commands() is invoked.
void initialise_commands(void) {
  extern t_command_state command_state;

  PIO_Configure(&USB_VBUS_PIN, 1);
  PIO_Configure(&VMOT_PIN, 1);

  if (!is_rev1_hw()) {
    PIO_Configure(&USB_HUB_NRESET_R2, 1); // not present on rev 1 hardware
  } /* endif */

  command_state.usb_state = USB_DISCONNECTED;
  command_state.rx_byte_count = 0;
  command_state.received = 0;
  command_state.robox_mode = 0;
  command_state.tx_byte_count = 0;
  command_state.error_flags = 0L;
  command_state.timer = get_time();
}


void wait_for_vmot(void) {
  while (!PIO_Get(&VMOT_PIN) && !was_button_pressed_at_start()) {
  }
}


// This routine should be called frequently.  It handles USB disconnection / connection and
// calls robox_command_handler() (or gcode_handler() in Pronterface compatibility mode) to
// handle data received from USB.
// All Robox command codes are >= 0x80, so the receipt of any byte >= 0x80 causes us to enter
// Robox mode, which persists until reset.  When operating with Pronterface, all bytes will
// be ASCII characters < 0x80 so we do not enter Robox mode.
// "Playing dead" (that is, pretending not to be powered up in the absence of the 24V supply,
// even though USB power is present) is implemented here.  This is because when the 24V
// supply goes away, we need to disconnect USB before resetting the processor.
// Note the use of was_button_pressed_at_start(); this allows "playing dead" to be
// circumvented for test / debug purposes, by holding the button at power-up.
void maintain_commands(void) {
  extern t_command_state command_state;
  unsigned short n, i;

  // done here because of the need to disconnect USB before resetting due to absence of 24V supply
  if ((command_state.usb_state == USB_DISCONNECTED) && !PIO_Get(&VMOT_PIN) && !was_button_pressed_at_start()) {
    AT91C_BASE_RSTC->RSTC_RCR = AT91C_RSTC_KEY | AT91C_RSTC_PERRST | AT91C_RSTC_PROCRST; // reset peripherals and processor
  } /* endif */

  if (PIO_Get(&USB_VBUS_PIN) && (PIO_Get(&VMOT_PIN) || was_button_pressed_at_start())) {
    if (!is_rev1_hw()) {
      PIO_Set(&USB_HUB_NRESET_R2);
    } /* endif */

    if (command_state.usb_state == USB_DISCONNECTED) {
      CDCDSerialDriver_Initialize();
      USBD_Connect();
      command_state.usb_state = USB_PENDING;
    } else if (command_state.usb_state == USB_PENDING) {
      if (USBD_GetState() == USBD_STATE_CONFIGURED) {
        command_state.usb_state = USB_CONNECTED;
        CDCDSerialDriver_Read(&command_state.rx_buffer, RX_BUFFER_SIZE, (TransferCallback)cb_usb_data_received, 0); // start receiving data on the USB
      } /* endif */
    } else if (USBD_GetState() != USBD_STATE_CONFIGURED) {
      USBD_Disconnect();
      command_state.usb_state = USB_DISCONNECTED;
    } /* endif */
  } else {
    if (command_state.usb_state != USB_DISCONNECTED) {
      USBD_Disconnect();
      command_state.usb_state = USB_DISCONNECTED;
    } /* endif */
  } /* endif */

  if (command_state.usb_state != USB_CONNECTED) {
    command_state.received = 0;
    command_state.rx_byte_count = 0;
    command_state.tx_byte_count = 0;
  } /* endif */

  if (command_state.received != 0) {
    command_state.rx_byte_count += command_state.received;
    command_state.rx_buffer[command_state.rx_byte_count] = 0; // gcode_handler() expects a zero-terminated string
    command_state.received = 0;
    i = 0;

    do {
      if (command_state.rx_byte_count > i) {
        command_state.robox_mode = command_state.robox_mode || (command_state.rx_buffer[i] >= 0x80); // Robox command codes are all >= 0x80; in Pronterface compatible mode, all bytes received should be ASCII characters < 0x80
      } /* endif */

      if (command_state.robox_mode) {
        n = robox_command_handler(&command_state.rx_buffer[i], command_state.rx_byte_count - i);
      } else {
        if (command_state.tx_byte_count != 0) {
          report_error(ERROR_USB_TX);
        } /* endif */

        n = gcode_handler((char *)&command_state.rx_buffer[i], (char *)command_state.tx_buffer);
        command_state.tx_byte_count = strlen((char *)command_state.tx_buffer);
        usb_send();
      } /* endif */

      i += n;
    } while (n > 0);

    // Normally, i = command_state.rx_byte_count at this point, so the memmove doesn't need to move anything.
    // However, we allow for the possibility that a partial command is left over...
    memmove(command_state.rx_buffer, &command_state.rx_buffer[i], command_state.rx_byte_count - i);
    command_state.rx_byte_count -= i;

    CDCDSerialDriver_Read(&command_state.rx_buffer[command_state.rx_byte_count], RX_BUFFER_SIZE - command_state.rx_byte_count, (TransferCallback)cb_usb_data_received, 0);
  } /* endif */
}


// Automaker is considered connected if the USB state is connected, and an error report has been requested within
// the timeout period.
unsigned char is_automaker_connected(void) {
  extern t_command_state command_state;

  return((command_state.usb_state == USB_CONNECTED) && !has_period_elapsed(command_state.timer, AUTOMAKER_TIMEOUT));
}


// Global-hiding routine to set an error flag.
void report_error(unsigned char error_number) {
  extern t_command_state command_state;

  command_state.error_flags |= (((uint64_t)1) << error_number);
}


// Indicates whether an error exists.  Note that some errors are always ignored, and some
// are ignored only when Automaker is considered to be "connected".
unsigned char is_error(void) {
  extern t_command_state command_state;
  uint64_t               e;

  e = command_state.error_flags & ~ERROR_PAUSE_MASK;

  if (is_automaker_connected()) {
    e &= ~ERROR_PAUSE_MASK_AM;
  } /* endif */

  return(e != 0);
}


// Allows certain errors to be cancelled by resume from the button, but only when Automaker
// is not connected.
void clear_resumable_errors(void) {
  extern t_command_state command_state;

  if (!is_automaker_connected()) {
    command_state.error_flags &= ~ERROR_RESUMABLE_MASK;
  } /* endif */
}


// Allows errors to be faked, to facilitate testing of Automaker
void fake_an_error(char line[]) {
  extern t_command_state command_state;

  if (has_code(line, 'S')) {
    command_state.error_flags |= (((uint64_t)1) << get_int(line, 'S'));
  } /* endif */
}


t_command_state command_state;
