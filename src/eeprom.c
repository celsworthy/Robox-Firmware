#include <pio/pio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "util.h"
#include "command.h"
#include "file_system.h"
#include "heaters.h"
#include "position.h"
#include "eeprom.h"
#include "motion_hal.h"

#define EEPROM_VIRTUAL_LENGTH 0x0c0 // the EEPROM size presented to the outside world
#define BANK_LENGTH           (EEPROM_VIRTUAL_LENGTH + 4) // adds 4 for crc
#define EEPROM_BANK0_START    0x000 // physical EEPROM address for the start of bank 0
#define EEPROM_BANK1_START    0x100 // physical EEPROM address for the start of bank 1

#define EEPROM_UPDATE_PERIOD  7.2   // secs; = 0.002 hours
#define EEPROM_TIMEOUT        0.2   // secs

#define pause()               delay(1e-6)

typedef enum {
  NOT_PRESENT = 0,
  PRESENT     = 1,
  VALID       = 2
} t_eeprom_status;

typedef struct {
  unsigned char   buffer[BANK_LENGTH];
  t_eeprom_status status;
  unsigned char   writing;
  unsigned char   working_bank;
  unsigned char   address_counter;
  unsigned long   timer;
} t_eeprom_state;

typedef struct {
  unsigned long update_timer;
  unsigned char i2c_mux_present;
} t_eeprom_misc;

typedef struct {
  unsigned char desired;
  unsigned char currently;
  unsigned char locked;
} t_head_state;

typedef struct {
  Pin           scl;
  Pin           sda;
  unsigned char sel;
} t_eeprom_hw;

const t_eeprom_hw HEAD_EEPROM  = {{1 << 17, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_PULLUP},
                                  {1 << 9,  AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP},
                                  0xff}; // 0xff means no i2c mux
const t_eeprom_hw REEL0_EEPROM = {{1 << 25, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_PULLUP},
                                  {1 << 24, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP},
                                   0}; // i2c mux chan 0 in dual reel adaptor
const t_eeprom_hw REEL1_EEPROM = {{1 << 25, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_PULLUP},
                                  {1 << 24, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP},
                                   1}; // i2c mux chan 1 in dual reel adaptor
const Pin HEAD_SHIFTER_LOAD[] =  {{1 << 6,  AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT},  // post rev 1
                                  {1 << 10, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT}}; // rev 1


// We manage the EEPROMs so that they may be disconnected / reconnected at any time without
// data corruption.  This is achieved by storing two copies of the data (two "banks").
// Each copy is protected by a crc.
// Whilst no EEPROM is present, our status flag is NOT_PRESENT and we keep checking for an EEPROM.
// When an EEPROM becomes present, we keep trying to load from alternating banks until we
// get a good crc.  If we've tried both banks and neither was valid, then the status flag becomes
// PRESENT.  When we get a good crc, the status flag becomes VALID.  Note that in normal operation,
// we transition from NOT_PRESENT straight to VALID, so as to enure that Automaker is not
// inadvertently led to reformat the EEPROM.
// Whilst the status is VALID, we keep reading alternating banks from the EEPROM, comparing
// with the buffer.  Whenever we have verified that a bank agrees with the buffer (including CRC),
// we move on to the other bank.
// If write_eeprom() modifies the buffer, it recalculates the CRC and zeros the address
// counter, so as to restart the process of verifying the bank against the buffer.
// The total length of each bank is 196 bytes (192 bytes of data + 4 bytes of crc).
// Therefore, only 392 of the EEPROM's 512 bytes are actually used.


// private...

// Updates crc according to the byte d.
void do_crc32(unsigned char d, unsigned long *crc) {
  const unsigned long CRCPOLY = 0xedb88320;
  int i;

  *crc ^= d;

  for (i = 0; i < 8; i ++) {
    *crc = (*crc >> 1) ^ ((*crc & 1) ? CRCPOLY : 0);
  } /* endfor */
}


inline void open_drain(const Pin *p, unsigned char v) {
  if (v) {
    p->pio->PIO_ODR = p->mask; // make input
  } else {
    PIO_Clear(p);
    p->pio->PIO_OER = p->mask; // make output
  } /* endif */
}


// Updates the head shift register if necessary, being careful only to change the data
// when the clock is low, so that the EEPROM doesn't see a start condition.
void update_head_shift_register(void) {
  extern t_head_state head_state;
  unsigned char m;

  if (head_state.currently != head_state.desired) {
    head_state.currently = head_state.desired;
    PIO_Clear(&HEAD_EEPROM.scl);
    pause();
    open_drain(&HEAD_EEPROM.sda, 0); // make output

    for (m = 0x80; m > 0; m = m >> 1) {
      if (head_state.currently & m) {
        PIO_Set(&HEAD_EEPROM.sda);
      } else {
        PIO_Clear(&HEAD_EEPROM.sda);
      } /* endif */

      pause();
      PIO_Set(&HEAD_EEPROM.scl);
      pause();
      PIO_Clear(&HEAD_EEPROM.scl);
      pause();
    } /* endfor */

    PIO_Set(&HEAD_SHIFTER_LOAD[is_rev1_hw()]);
    pause();
    PIO_Clear(&HEAD_SHIFTER_LOAD[is_rev1_hw()]);
    pause();
    open_drain(&HEAD_EEPROM.sda, 1);
    pause();
    PIO_Set(&HEAD_EEPROM.scl);
    pause();
  } /* endif */
}


unsigned char send_byte(const t_eeprom_hw *hw, unsigned char b) {
  unsigned char m, r;

  for (m = 0x80; m != 0; m = m >> 1) {
    open_drain(&hw->sda, b & m);
    pause();
    PIO_Set(&hw->scl);
    pause();
    PIO_Clear(&hw->scl);
    pause();
  } /* endfor */

  open_drain(&hw->sda, 1);
  pause();
  PIO_Set(&hw->scl);
  pause();
  r = !PIO_Get(&hw->sda);
  pause();
  PIO_Clear(&hw->scl);
  pause();
  return(r);
}


unsigned char get_byte(const t_eeprom_hw *hw, unsigned char ack) {
  unsigned char m, b;

  open_drain(&hw->sda, 1);
  b = 0;

  for (m = 0x80; m != 0; m = m >> 1) {
    PIO_Set(&hw->scl);
    pause();
    b |= PIO_Get(&hw->sda) ? m : 0;
    pause();
    PIO_Clear(&hw->scl);
    pause();
  } /* endfor */

  open_drain(&hw->sda, !ack);
  pause();
  PIO_Set(&hw->scl);
  pause();
  PIO_Clear(&hw->scl);
  pause();
  open_drain(&hw->sda, 1);
  pause();
  return(b);
}


// Deals with the i2c mux of a dual-reel adaptor.  If we're trying to access reel 1
// and the mux isn't present (doesn't acknowledge), we return false, as it's not
// possible to access reel 1 without a dual-reel adaptor.
// If we're trying to access reel 0 and the mux isn't present, we return true, as
// it is possible to access reel 0 without a dual-reel adaptor.
// eeprom_misc.i2c_mux_present is updated for reporting purposes.
unsigned char i2c_mux_select(const t_eeprom_hw *hw) {
  extern t_eeprom_misc eeprom_misc;

  if (hw->sel > 1) {
    return(1); // no i2c mux to deal with
  } /* endif */

  // start condition
  open_drain(&hw->sda, 0);
  pause();
  PIO_Clear(&hw->scl);
  pause();

  eeprom_misc.i2c_mux_present = send_byte(hw, 0xe0); // slave address of i2c mux

  if (eeprom_misc.i2c_mux_present) {
    eeprom_misc.i2c_mux_present = send_byte(hw, hw->sel | 0x04); // channel select
  } /* endif */

  // stop condition
  open_drain(&hw->sda, 0);
  pause();
  PIO_Set(&hw->scl);
  pause();
  open_drain(&hw->sda, 1);
  pause();

  return(eeprom_misc.i2c_mux_present || (hw->sel != 1));
}


unsigned char write_one_byte(const t_eeprom_hw *hw, unsigned short address, unsigned char data) {
  unsigned char r;

  if (!i2c_mux_select(hw)) { // deal with i2c mux (only present with dual-reel adaptor)
    return(0);
  } /* endif */

  // start condition
  open_drain(&hw->sda, 0);
  pause();
  PIO_Clear(&hw->scl);
  pause();

  r = send_byte(hw, 0xa0 | (((unsigned char)(address >> 7)) & 0x06));

  if (r) {
    r = send_byte(hw, (unsigned char)address);
  } /* endif */

  if (r) {
    r = send_byte(hw, data);
  } /* endif */

  // stop condition
  open_drain(&hw->sda, 0);
  pause();
  PIO_Set(&hw->scl);
  pause();
  open_drain(&hw->sda, 1);
  pause();

  return(r);
}


unsigned char read_one_byte(const t_eeprom_hw *hw, unsigned short address, unsigned char *data) {
  unsigned char r;

  if (!i2c_mux_select(hw)) { // deal with i2c mux (only present with dual-reel adaptor)
    return(0);
  } /* endif */

  // start condition
  open_drain(&hw->sda, 0);
  pause();
  PIO_Clear(&hw->scl);
  pause();

  r = send_byte(hw, 0xa0 | (((unsigned char)(address >> 7)) & 0x06));

  if (r) {
    r = send_byte(hw, (unsigned char)address);
  } /* endif */

  if (r) {
    PIO_Set(&hw->scl);
    pause();

    // start condition
    open_drain(&hw->sda, 0);
    pause();
    PIO_Clear(&hw->scl);
    pause();

    r = send_byte(hw, 0xa1 | (((unsigned char)(address >> 7)) & 0x06));
    *(data ++) = get_byte(hw, 0); // don't ack
  } /* endif */

  // stop condition
  open_drain(&hw->sda, 0);
  pause();
  PIO_Set(&hw->scl);
  pause();
  open_drain(&hw->sda, 1);
  pause();

  return(r);
}


unsigned char is_crc_good(unsigned char *d) {
  unsigned long  crc;
  unsigned short i;

  crc = 0x535e01fe;

  for (i = 0; i < BANK_LENGTH; i ++) {
    do_crc32(d[i], &crc);
  } /* endfor */

  return(crc == 0);
}


void insert_crc(unsigned char *d) {
  unsigned long crc;
  unsigned short i;

  crc = 0x535e01fe;

  for (i = 0; i < EEPROM_VIRTUAL_LENGTH; i ++) {
    do_crc32(d[i], &crc);
  } /* endfor */

  memcpy(&d[EEPROM_VIRTUAL_LENGTH], &crc, 4);
}


void maintain_eeprom(const t_eeprom_hw *hw, t_eeprom_state *e) {
  unsigned char d;

  if (e->writing) {
    if (write_one_byte(hw, (e->working_bank ? EEPROM_BANK1_START : EEPROM_BANK0_START) + e->address_counter, e->buffer[e->address_counter])) {
      e->timer = get_time();
      e->writing = 0;
    } /* endif */
  } else {
    if (read_one_byte(hw, (e->working_bank ? EEPROM_BANK1_START : EEPROM_BANK0_START) + e->address_counter, &d)) {
      e->timer = get_time();

      if (e->status != VALID) {
        e->buffer[e->address_counter] = d;
        e->address_counter ++;

        if (e->address_counter >= BANK_LENGTH) {
          if (is_crc_good(e->buffer)) {
            e->status = VALID;
          } else if (e->working_bank) { // avoid going to PRESENT state on the way to VALID; only go to PRESENT when we've tried both banks and neither was valid
            e->status = PRESENT;
          } /* endif */

          e->working_bank = !e->working_bank; // if we loaded successfully, then the other bank is the working bank; if we didn't load successfully, try the other bank next time
          e->address_counter = 0;
        } /* endif */
      } else {
        if (d == e->buffer[e->address_counter]) {
          e->address_counter ++;

          if (e->address_counter >= BANK_LENGTH) {
            e->working_bank = !e->working_bank;
            e->address_counter = 0;
          } /* endif */
        } else {
          e->writing = 1;
        } /* endif */
      } /* endif */
    } /* endif */
  } /* endif */

  if (has_period_elapsed(e->timer, EEPROM_TIMEOUT)) {
    e->status = NOT_PRESENT;
    e->writing = 0;
    e->address_counter = 0;
    e->working_bank = 0;
  } /* endif */
}


// public...
void initialise_eeproms(void) {
  extern t_eeprom_state eeprom_states[];
  extern t_head_state   head_state;
  extern t_eeprom_misc  eeprom_misc;
  unsigned char         i;

  PIO_Configure(&HEAD_EEPROM.scl, 1);
  PIO_Configure(&HEAD_EEPROM.sda, 1);
  PIO_Configure(&REEL0_EEPROM.scl, 1);
  PIO_Configure(&REEL0_EEPROM.sda, 1);
  // no need to do REEL1_EEPROM; it uses the same pins
  PIO_Configure(&HEAD_SHIFTER_LOAD[is_rev1_hw()], 1);

  for (i = 0; i < 3; i ++) {
    eeprom_states[i].status = NOT_PRESENT;
    eeprom_states[i].writing = 0;
    eeprom_states[i].address_counter = 0;
    eeprom_states[i].working_bank = 0;
    eeprom_states[i].timer = get_time();
  } /* endfor */

  eeprom_misc.update_timer = get_time();
  eeprom_misc.i2c_mux_present = 0;

  head_state.locked = 1;
  head_state.desired = 0x43;
  head_state.currently = ~head_state.desired; // just make sure they are not equal!
  update_head_shift_register();
  head_state.locked = 0;
}


// Loading of temperature targets from the reel EEPROM is only triggered at
// start-up, and on introduction of a reel.
void maintain_eeproms(void) {
  extern t_eeprom_state  eeprom_states[];
  extern t_head_state    head_state;
  extern t_eeprom_misc   eeprom_misc;
  static t_eeprom_status status[3] = {NOT_PRESENT, NOT_PRESENT, NOT_PRESENT};

  head_state.locked = 1;
  maintain_eeprom(&HEAD_EEPROM, &eeprom_states[0]);
  update_head_shift_register(); // only does it if head_state.desired has changed whilst we've had head_state.locked set
  head_state.locked = 0;

  maintain_eeprom(&REEL0_EEPROM, &eeprom_states[1]);
  maintain_eeprom(&REEL1_EEPROM, &eeprom_states[2]);

  // When head EEPROM appears...
  if ((eeprom_states[0].status == VALID) && (status[0] != VALID)) {
    check_for_head_change(); // prevents reprints if the type of head changes
  } /* endif */

  // When either reel appears, get information from the reel EEPROMs.  Note that this is also done when the head EEPROM
  // appears, because the "routing" of the information in the reel EEPROMs depends on whether the head is dual-material.
  if (((eeprom_states[0].status == VALID) && (status[0] != VALID)) ||
      ((eeprom_states[1].status == VALID) && (status[1] != VALID)) ||
      ((eeprom_states[2].status == VALID) && (status[2] != VALID))) {
    get_tempcon_targets_from_reel();
    get_filament_info_from_reel();
  } /* endif */

  if (has_period_elapsed(eeprom_states[0].timer, EEPROM_TIMEOUT)) {
    shut_down_head_due_to_eeprom();
  } /* endif */

  if (has_period_elapsed(eeprom_misc.update_timer, EEPROM_UPDATE_PERIOD)) {
    if (is_head_hot()) {
      write_eeprom_float(0, 0xb8, get_eeprom_float(0, 0xb8, 0.0) + (EEPROM_UPDATE_PERIOD / 3600)); // head hours counter
    } /* endif */

    update_reel_filament_length();
    eeprom_misc.update_timer = get_time();
  } /* endif */

  status[0] = eeprom_states[0].status;
  status[1] = eeprom_states[1].status;
  status[2] = eeprom_states[2].status;
}


unsigned short get_eeprom_read(unsigned char eeprom_sel, unsigned char *d) {
  extern t_eeprom_state eeprom_states[];
  t_eeprom_state *e;
  unsigned short i;

  e = &eeprom_states[eeprom_sel];

  if (e->status != VALID) {
    report_error((eeprom_sel == 2) ? ERROR_REEL1_EEPROM : ((eeprom_sel == 1) ? ERROR_REEL0_EEPROM : ERROR_HEAD_EEPROM));
    memset(d, 0, EEPROM_VIRTUAL_LENGTH);
  } else {
    for (i = 0; i < EEPROM_VIRTUAL_LENGTH; i ++) {
      d[i] = e->buffer[i] & 0x7f; // avoid returning anything non-ASCII
    } /* endfor */
  } /* endif */

  return(EEPROM_VIRTUAL_LENGTH);
}


float get_eeprom_float(unsigned char eeprom_sel, unsigned short address, float def) {
  extern t_eeprom_state eeprom_states[];
  t_eeprom_state *e;
  char temp[9];

  e = &eeprom_states[eeprom_sel];

  if (e->status != VALID) {
    return(def);
  } /* endif */

  memcpy(temp, &e->buffer[address], 8);
  temp[8] = 0; // ensure string is terminated
  return(strtof(temp, NULL));
}


void write_eeprom_float(unsigned char eeprom_sel, unsigned short address, float v) {
  extern t_eeprom_state eeprom_states[];
  t_eeprom_state        *e;
  char                  temp[32];

  e = &eeprom_states[eeprom_sel];

  if ((e->status != VALID) || (address > (EEPROM_VIRTUAL_LENGTH - 8))) {
    return;
  } /* endif */

  sprintf(temp, "%f        ", v);
  memcpy(&e->buffer[address], temp, 8);
  e->address_counter = 0;
  e->writing = 0;
  insert_crc(e->buffer);
}


// n is the number of characters to be written.  d contains the characters to be written.
// Data extending beyond address EEPROM_VIRTUAL_LENGTH is ignored.  If the buffer is
// modified, then we recalculate the CRC and zero the address counter, forcing a restart
// of the checking / updating of the working bank.
void write_eeprom(unsigned char eeprom_sel, unsigned short address, unsigned short n, unsigned char *d) {
  extern t_eeprom_state eeprom_states[];
  t_eeprom_state *e;
  unsigned short i;
  unsigned char  modified;

  e = &eeprom_states[eeprom_sel];

  if (e->status != VALID) {
    report_error((eeprom_sel == 2) ? ERROR_REEL1_EEPROM : ((eeprom_sel == 1) ? ERROR_REEL0_EEPROM : ERROR_HEAD_EEPROM));
    return;
  } /* endif */

  modified = 0;

  for (i = 0; (i < n) && (address < EEPROM_VIRTUAL_LENGTH); i ++) {
    if ((d[i] & 0x7f) != e->buffer[address]) { // don't allow anything non-ASCII
      modified = 1;
      e->buffer[address] = d[i] & 0x7f; // don't allow anything non-ASCII
    } /* endif */

    address ++;
  } /* endfor */

  if (modified) {
    e->address_counter = 0;
    e->writing = 0;
    insert_crc(e->buffer);

    if ((eeprom_sel == 1) || (eeprom_sel == 2)) { // if a reel EEPROM
      get_tempcon_targets_from_reel();
      get_filament_info_from_reel();
    } /* endif */
  } /* endif */
}


// Allows an unformatted or corrupted EEPROM to be made usable.  Fills the buffer
// with zeros, and sets the status to VALID.  This will cause stores to the EEPROM,
// after which the EEPROM should be usable.
void format_eeprom(unsigned char eeprom_sel) {
  extern t_eeprom_state eeprom_states[];
  t_eeprom_state *e;

  e = &eeprom_states[eeprom_sel];
  memset(e->buffer, 0, EEPROM_VIRTUAL_LENGTH);
  insert_crc(e->buffer);
  e->status = VALID;
  e->address_counter = 0;
  e->writing = 0;
}


// Provides EEPROM states for the purposes of a status report
void get_eeprom_states(unsigned char *d) {
  extern t_eeprom_state eeprom_states[];
  extern t_eeprom_misc  eeprom_misc;

  d[0] = '0' + (unsigned char)eeprom_states[0].status;
  d[1] = '0' + (unsigned char)eeprom_states[1].status;
  d[2] = '0' + (unsigned char)eeprom_states[2].status;
  d[3] = '0' + eeprom_misc.i2c_mux_present;
}


// Because the head shift register shares pins with the head EEPROM, it makes sense
// to keep this routine local.  This routine may be called by an ISR, so head_state.locked
// is used to prevent re-entrancy.  Also, atomic functions are used to modify head_state.byte,
// ensuring that a change made by an ISR won't subsequently get undone by the foreground.
void set_head_output(unsigned char bitno, unsigned char value) {
  extern t_head_state head_state;

  if (value) {
    __sync_fetch_and_or(&head_state.desired, 1 << bitno);
  } else {
    __sync_fetch_and_and(&head_state.desired, ~(1 << bitno));
  } /* endif */

  if (!head_state.locked) {
    head_state.locked = 1;
    update_head_shift_register();
    head_state.locked = 0;
  } /* endif */
}


unsigned char is_head_eeprom_valid(void) {
  extern t_eeprom_state eeprom_states[];
  return(eeprom_states[0].status == VALID);
}


unsigned char is_head_dual_material(void) {
  extern t_eeprom_state eeprom_states[];

  if (eeprom_states[0].status != VALID) {
    return(0);
  } /* endif */

  // old naming scheme
  if (memcmp(&eeprom_states[0].buffer[0], "RBX01-DM", 8) == 0) {
    return(1);
  } /* endif */

  // new naming scheme
  return((memcmp(&eeprom_states[0].buffer[0], "RX", 2) == 0) && (((eeprom_states[0].buffer[7] - '0') & 3) == 2));
}


unsigned char does_head_have_two_fans(void) {
  extern t_eeprom_state eeprom_states[];

  if (eeprom_states[0].status != VALID) {
    return(0);
  } /* endif */

  // old naming scheme
  if (memcmp(&eeprom_states[0].buffer[0], "RBXDV-S3", 8) == 0) {
    return(1);
  } /* endif */

  // new naming scheme
  return((memcmp(&eeprom_states[0].buffer[0], "RX", 2) == 0) && (((eeprom_states[0].buffer[7] - '0') & 4) != 0));
}


unsigned char does_head_have_b_axis(void) {
  extern t_eeprom_state eeprom_states[];

  if (eeprom_states[0].status != VALID) {
    return(0);
  } /* endif */

  // old naming scheme
  if ((memcmp(&eeprom_states[0].buffer[0], "RBX01-SM", 8) == 0) || (memcmp(&eeprom_states[0].buffer[0], "RBX01-S2", 8) == 0) || (memcmp(&eeprom_states[0].buffer[0], "RBX01-DM", 8) == 0)) {
    return(1);
  } /* endif */

  // new naming scheme
  return((memcmp(&eeprom_states[0].buffer[0], "RX", 2) == 0) && (((eeprom_states[0].buffer[6] - '0') & 1) != 0));
}


unsigned char does_head_have_prtd(void) {
  extern t_eeprom_state eeprom_states[];

  if (eeprom_states[0].status != VALID) {
    return(0);
  } /* endif */

  // old naming scheme
  if (memcmp(&eeprom_states[0].buffer[0], "RBXDV-S2", 8) == 0) {
    return(1);
  } /* endif */

  // new naming scheme
  return((memcmp(&eeprom_states[0].buffer[0], "RX", 2) == 0) && (((eeprom_states[0].buffer[6] - '0') & 4) == 4));
}


float get_head_max_temperature(void) {
  extern t_eeprom_state eeprom_states[];
  int                   t;

  if (eeprom_states[0].status == VALID) {
    // old naming scheme
    if ((memcmp(&eeprom_states[0].buffer[0], "RBX01-S2", 8) == 0) || (memcmp(&eeprom_states[0].buffer[0], "RBX01-DM", 8) == 0)) {
      return(280.0);
    } /* endif */

    if (memcmp(&eeprom_states[0].buffer[0], "RBXDV-S1", 8) == 0) {
      return(300.0);
    } /* endif */

    if (memcmp(&eeprom_states[0].buffer[0], "RBXDV-S2", 8) == 0) {
      return(500.0);
    } /* endif */

    // new naming scheme
    if (memcmp(&eeprom_states[0].buffer[0], "RX", 2) == 0) {
      t = eeprom_states[0].buffer[4];

      if ((t >= '1') && (t <= '9')) {
        return((float)(200 + (5 * (t - '1'))));
      } else if ((t >= 'A') && (t <= 'Z')) {
        return((float)(245 + (5 * (t - 'A'))));
      } else if ((t >= 'a') && (t <= 'z')) {
        return((float)(375 + (5 * (t - 'a'))));
      } /* endif */
    } /* endif */
  } /* endif */

  return(260.0);
}


unsigned char is_reel0_present(void) {
  extern t_eeprom_state eeprom_states[];
  return(eeprom_states[1].status == VALID);
}


unsigned char is_reel1_present(void) {
  extern t_eeprom_state eeprom_states[];
  return(eeprom_states[2].status == VALID);
}


t_eeprom_state eeprom_states[3];
t_head_state   head_state;
t_eeprom_misc  eeprom_misc;
