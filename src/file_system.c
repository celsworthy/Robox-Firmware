#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <pio/pio.h>
#include "util.h"
#include "sd_mmc.h"
#include "command.h"
#include "eeprom.h"
#include "file_system.h"


#define FILE_START_BLOCK          1    // directory occupies just block 0
#define CONTIGUOUS_BLOCKS         1024 // a file <= this length (ie. a firmware file) must not wrap around from end to start of SD card

#define NO_FILE                   0xff
#define N_FILES                   16 // must be a power of 2
#define wrap_file_number(f)       ((f) & (N_FILES - 1))
#define is_no_file(f)             ((f) >= N_FILES)

const Pin  SD_CD_PIN = {1 << 1, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP};

typedef struct {
  unsigned char id[16];
  unsigned char is_job; // allows double button press "run last job" to ignore files that are not job files
  unsigned char unused[3];
  unsigned long start_block;
  unsigned long length;
} t_file_info;


typedef struct {
  unsigned long capacity; // in blocks
  unsigned long next_write_block;
  unsigned char first_file;
  unsigned char next_file;
  unsigned char last_job;
  unsigned char nonvolatile_data[2];
  unsigned char padding[51];
  t_file_info   file_info[N_FILES];
} t_directory;


typedef struct {
  t_directory   directory;
  unsigned long next_write_block;
  unsigned long next_read_block;
  unsigned long read_byte_counter;
  unsigned char write_file;
  unsigned char read_file;
  unsigned char ok;
} t_fs_state;


// Very rarely sd_mmc_read_block() was failing here, causing ERROR_SD_CARD.  This was
// found to be due to hsmci_read_word() failing because the hardware CRC error flag was
// set.  The reason for the CRC errors is not known, but we work around the problem
// here by doing a retry.
static void read_one_block(unsigned long block, void *d) {
  extern t_fs_state fs_state;

  if (!fs_state.ok) {
    report_error(ERROR_SD_CARD);
  } else if (!sd_mmc_read_block(block, d)) {
    // re-try
    if (!sd_mmc_init()) {
      initialise_file_system();
      report_error(ERROR_SD_CARD);
    } else if (!sd_mmc_read_block(block, d)) {
      initialise_file_system();
      report_error(ERROR_SD_CARD);
    } /* endif */
  } /* endif */
}


static void write_one_block(unsigned long block, void *d) {
  extern t_fs_state fs_state;

  if (!fs_state.ok) {
    report_error(ERROR_SD_CARD);
  } else if (!sd_mmc_write_block(block, d)) {
    initialise_file_system();
    report_error(ERROR_SD_CARD);
  } /* endif */
}


// Checks whether the SD card is full (ie. we have caught up with the first file).
// If so, we make space by deleting the first file, which is done by advancing
// fs_state.directory.first_file.  We then store the updated directory to the SD
// card, to ensure that the deletion is permanent.
// If we have caught up with the first file and we are still writing the first file,
// then the file must be larger that the SD card; in this case we just report an
// error.
static void make_space(void) {
  extern t_fs_state fs_state;

  if (fs_state.next_write_block == fs_state.directory.file_info[fs_state.directory.first_file].start_block) { // if we've caught up with the first file
    if (fs_state.directory.first_file == fs_state.directory.next_file) { // card full... the file must be bigger than the SD card!
      report_error(ERROR_FILE_TOO_LARGE);
    } else {
      if (fs_state.read_file == fs_state.directory.first_file) {
        fs_state.read_file = NO_FILE;
        report_error(ERROR_FILE_READ_CLOBBERED);
      } /* endif */

      if (fs_state.directory.last_job == fs_state.directory.first_file) {
        fs_state.directory.last_job = NO_FILE;
      } /* endif */

      fs_state.directory.first_file = wrap_file_number(fs_state.directory.first_file + 1);
      write_one_block(0L, (void *)&fs_state.directory); // store advance of first_file to SD just in case we don't make it to end_write_file()
      sd_mmc_finish(); // make sure write is flushed to SD card
    } /* endif */
  } /* endif */
}


// Deletes the most recent file if it's not a job file, and it's not open for read.
// Intended for use by start_write_file(), the idea being to prevent job files from
// being displaced from the file system by a multitude of non-job files (most likely
// macros).  NB: no need to update last_job, as we will never delete a job file.
static void delete_last_file_if_not_job(void) {
  extern t_fs_state fs_state;
  unsigned char f;

  if (fs_state.directory.next_file == fs_state.directory.first_file) { // if no files
    return;
  } /* endif */

  f = wrap_file_number(fs_state.directory.next_file - 1);

  if ((fs_state.directory.file_info[f].is_job) || (fs_state.read_file == f)) { // if last file is a job, or is open for read
    return;
  } /* endif */

  fs_state.directory.next_write_block = fs_state.directory.file_info[f].start_block;
  memset(&fs_state.directory.file_info[f], 0, sizeof(t_file_info));
  fs_state.directory.next_file = f;

  // make sure the directory on SD card reflects the deletion before we start overwriting the deleted file's data
  write_one_block(0L, (void *)&fs_state.directory);
  sd_mmc_finish(); // make sure write is flushed to SD card
}


static void do_format(t_directory *directory) {
  memset((void *)directory, 0, sizeof(t_directory));
  directory->last_job = NO_FILE;
  directory->capacity = sd_mmc_get_capacity() << 1; // convert Kbytes to blocks
  directory->next_write_block = FILE_START_BLOCK;
  write_one_block(0L, (void *)directory);
  sd_mmc_finish(); // make sure write is flushed to SD card
}


static unsigned char prepare_sd_card(void) {
  extern t_fs_state fs_state;
  unsigned long v;
  unsigned char f, needs_format = 0;

  if (fs_state.ok) {
    return(1);
  } /* endif */

  if (!sd_mmc_init()) {
    return(0);
  } /* endif */

  fs_state.ok = 1;
  v = sd_mmc_get_capacity() << 1; // convert Kbytes to blocks
  read_one_block(0L, (unsigned char *)&fs_state.directory);

  if ((fs_state.directory.capacity != v) || (fs_state.directory.next_write_block < FILE_START_BLOCK) || (fs_state.directory.next_write_block >= v) ||
      (fs_state.directory.first_file >= N_FILES) || (fs_state.directory.next_file >= N_FILES)) { // directory not valid
    needs_format = 1;
  } else {
    v = fs_state.directory.file_info[fs_state.directory.first_file].start_block;

    for (f = fs_state.directory.first_file; f != fs_state.directory.next_file; f = wrap_file_number(f + 1)) {
      needs_format = needs_format || (fs_state.directory.file_info[f].start_block != v);
      v += (fs_state.directory.file_info[f].length + 511) >> 9; // convert bytes to blocks
      v = (fs_state.directory.capacity < (v + CONTIGUOUS_BLOCKS)) ? FILE_START_BLOCK : v;
    } /* endfor */
  } /* endif */

  if (needs_format) {
    do_format(&fs_state.directory);
  } /* endif */

  fs_state.read_file = fs_state.write_file = NO_FILE;
  fs_state.next_write_block = fs_state.directory.next_write_block;
  return(1);
}


// public...

void initialise_file_system(void) {
  extern t_fs_state fs_state;

  PIO_Configure(&SD_CD_PIN, 1);
  memset((void *)&fs_state.directory, 0, sizeof(t_directory));
  fs_state.directory.last_job = NO_FILE;
  fs_state.write_file = NO_FILE;
  fs_state.read_file = NO_FILE;
  fs_state.ok = 0;
}


void maintain_file_system(void) {
  extern t_fs_state fs_state;

  if (fs_state.ok && PIO_Get(&SD_CD_PIN)) {
    initialise_file_system();
  } /* endif */
}


void format_sd_card(void) {
  extern t_fs_state fs_state;

  fs_state.write_file = NO_FILE;
  fs_state.read_file = NO_FILE;

  if (!prepare_sd_card()) {
    report_error(ERROR_SD_CARD);
    return;
  } /* endif */

  do_format(&fs_state.directory);
  fs_state.next_write_block = fs_state.directory.next_write_block;
}


// Initialises the "next" directory entry, setting the file id to be written.  If
// we are close to the end of the SD card, then we go back to the start.  This
// ensures that a firmware file (which is always short) will never wrap around.
// Other files, which may be arbitrarily large, are allowed to wrap around.
void start_write_file(unsigned char *id, unsigned char is_job) {
  extern t_fs_state fs_state;
  unsigned char i;

  if (!prepare_sd_card()) {
    report_error(ERROR_SD_CARD);
    return;
  } /* endif */

  delete_last_file_if_not_job(); // avoid accumulation of non-job files, so they can't displace job files from the file system
  fs_state.next_write_block = fs_state.directory.next_write_block;

  if (fs_state.directory.capacity < (fs_state.next_write_block + CONTIGUOUS_BLOCKS)) {
    fs_state.next_write_block = FILE_START_BLOCK;
    make_space(); // if we've caught up with the first file, delete the first file
  } /* endif */

  if (fs_state.read_file == fs_state.directory.next_file) {
    fs_state.read_file = NO_FILE;
    report_error(ERROR_FILE_READ_CLOBBERED);
  } /* endif */

  memset(&fs_state.directory.file_info[fs_state.directory.next_file], 0, sizeof(t_file_info));

  for (i = 0; (i < 16) && id[i]; i ++) {
    fs_state.directory.file_info[fs_state.directory.next_file].id[i] = id[i];
  } /* endfor */

  fs_state.directory.file_info[fs_state.directory.next_file].is_job = is_job;
  fs_state.directory.file_info[fs_state.directory.next_file].start_block = fs_state.next_write_block;
  fs_state.directory.file_info[fs_state.directory.next_file].length = 0L;
  fs_state.write_file = fs_state.directory.next_file;
}


// If sequence_number is correct, writes a block of data to the SD card.  If sequence_number
// is less than the number of blocks already written, then we do nothing.  If sequence_number
// is greater than the number of blocks already written, we report an error as a block must
// have gone missing.
// If we've caught up with the start of the first (oldest) file, then the first file is discarded
// (by advancing fs_state.directory.first_file).  Note that the directory needs to be written
// to SD in this case, to ensure we don't leave the SD in a bad state if we carry on writing
// but for some reason (eg. power fail) we never get to end_write_file().
void write_file(unsigned long sequence_number, unsigned char *d) {
  extern t_fs_state fs_state;

  if (!prepare_sd_card()) {
    report_error(ERROR_SD_CARD);
    return;
  } else if (is_no_file(fs_state.write_file)) {
    report_error(ERROR_CHUNK_SEQUENCE);
    return;
  } else if (sequence_number < (fs_state.directory.file_info[fs_state.directory.next_file].length >> 9)) {
    return;
  } else if (sequence_number > (fs_state.directory.file_info[fs_state.directory.next_file].length >> 9)) {
    report_error(ERROR_CHUNK_SEQUENCE);
    return;
  } /* endif */

  write_one_block(fs_state.next_write_block ++, d);
  fs_state.next_write_block = (fs_state.next_write_block >= fs_state.directory.capacity) ? FILE_START_BLOCK : fs_state.next_write_block;
  make_space(); // if we've caught up with the first file, delete the first file
  fs_state.directory.file_info[fs_state.directory.next_file].length += 512L;
}


// Completes the write of the file.  The remaining data is padded to a full block and
// written to SD, then the directory is updated to include the file.  Note that if
// this causes the directory to be full, we discard the oldest entry to make space.
void end_write_file(unsigned long sequence_number, unsigned short n, unsigned char *d) {
  extern t_fs_state fs_state;
  unsigned char b[512];

  if (!prepare_sd_card()) {
    report_error(ERROR_SD_CARD);
    return;
  } else if (is_no_file(fs_state.write_file)) {
    report_error(ERROR_CHUNK_SEQUENCE);
    return;
  } else if (sequence_number != (fs_state.directory.file_info[fs_state.directory.next_file].length >> 9)) {
    report_error(ERROR_CHUNK_SEQUENCE);
    return;
  } /* endif */

  n = min(512, n);

  if (n > 0) {
    memcpy(b, d, n);
    memset(&b[n], 0, 512 - n); // pad the file with zeros
    write_file(sequence_number, b); // advances length by 512
    fs_state.directory.file_info[fs_state.directory.next_file].length -= (512 - n); // correct length according to n
  } /* endif */

  if (fs_state.directory.file_info[fs_state.directory.next_file].length > 0) { // don't allow zero-length file
    fs_state.directory.next_file = wrap_file_number(fs_state.directory.next_file + 1);
    fs_state.directory.first_file = wrap_file_number(fs_state.directory.first_file + (fs_state.directory.first_file == fs_state.directory.next_file));
    fs_state.directory.next_write_block = fs_state.next_write_block;
  } /* endif */

  fs_state.directory.last_job = (fs_state.directory.last_job == fs_state.directory.next_file) ? NO_FILE : fs_state.directory.last_job;
  fs_state.write_file = NO_FILE;
  write_one_block(0L, (void *)&fs_state.directory);
  sd_mmc_finish(); // make sure write is flushed to SD card
}


// Attempts to open a file with specified id for read.  If a file is currently being
// written, it also is eligible.  Note that we start with the most recent file and work
// backwards, so if there are duplicate files with the same id, then the newest is opened.
// last_job is updated to allow subsequent "run last job".
// If id is an empty string, then we attempt to open the last job.
// Returns 1 if successful, 0 otherwise.
unsigned char start_read_file(unsigned char *id) {
  extern t_fs_state fs_state;
  unsigned char f, n;

  fs_state.read_file = NO_FILE;

  if (!prepare_sd_card()) {
    report_error(ERROR_SD_CARD);
    return(0);
  } /* endif */

  f = fs_state.directory.next_file;
  n = wrap_file_number(fs_state.directory.next_file - fs_state.directory.first_file);

  if (id[0] && (f == fs_state.write_file)) { // if id[] is not empty and there's a file being written, extend the search to include it
    n ++;
  } else {
    f = wrap_file_number(f - 1);
  } /* endif */

  while (n) {
    if ((memcmp((char *)fs_state.directory.file_info[f].id, (char *)id, 16) == 0) || ((f == fs_state.directory.last_job) && fs_state.directory.file_info[f].is_job && !id[0])) {
      fs_state.read_file = f;
      fs_state.next_read_block = fs_state.directory.file_info[f].start_block;
      fs_state.read_byte_counter = 0L;

      // update last_job if necessary
      if (fs_state.directory.file_info[f].is_job && (f != fs_state.directory.last_job)) {
        fs_state.directory.last_job = f;

        // save last_job to SD card; but if it's the file being written, then leave the save to end_write_file(), in case end_write_file() never gets called
        if (fs_state.directory.last_job != fs_state.write_file) {
          write_one_block(0L, (void *)&fs_state.directory);
          sd_mmc_finish(); // make sure write is flushed to SD card
        } /* endif */
      } /* endif */

      return(1);
    } /* endif */

    f = wrap_file_number(f - 1);
    n --;
  } /* endwhile */

  return(0);
}


// Attempts to read the next block.  If no file is open, or there's no more data to
// be read, returns 0.  Otherwise, returns the number of bytes that have been put in
// d.  d must be large enough to accept 512 bytes.  Note that "no more data to be read"
// may be a temporary situation if the file is still being written, so the return value
// of 0 should not be taken to indicate that we've finished reading the file; use
// is_eof() for this.
unsigned short read_file(unsigned char *d) {
  extern t_fs_state fs_state;
  unsigned long n;

  if (!prepare_sd_card()) {
    report_error(ERROR_SD_CARD);
    return(0);
  } else if (is_no_file(fs_state.read_file)) { // no file
    return(0);
  } else if (fs_state.directory.file_info[fs_state.read_file].length <= fs_state.read_byte_counter) {
    return(0);
  } /* endif */

  read_one_block(fs_state.next_read_block ++, d);
  fs_state.next_read_block = (fs_state.next_read_block >= fs_state.directory.capacity) ? FILE_START_BLOCK : fs_state.next_read_block;
  n = min(512, fs_state.directory.file_info[fs_state.read_file].length - fs_state.read_byte_counter);
  fs_state.read_byte_counter += n;
  return((unsigned short)n);
}


void close_read_file(void) {
  extern t_fs_state fs_state;

  fs_state.read_file = NO_FILE;
}


// Returns 1 if the end of file has been reached, or if no file is open for reading.
// If the read file is currently being written, we assume we haven't reached the end
// yet and return 0.
unsigned char is_eof(void) {
  extern t_fs_state fs_state;

  if (is_no_file(fs_state.read_file)) { // if no file, report eof
    return(1);
  } else if (fs_state.read_file == fs_state.write_file) { // if file not yet finished, then don't report eof
    return(0);
  } /* endif */

  return(fs_state.read_byte_counter >= fs_state.directory.file_info[fs_state.read_file].length);
}


// Note that this listing only includes files which have been committed to the SD card.
// If a file is currently being written, it is not listed.
unsigned short get_file_list(unsigned char *d) {
  extern t_fs_state fs_state;
  unsigned char  f;
  unsigned short n;

  if (!prepare_sd_card()) {
    report_error(ERROR_SD_CARD);
    return(0);
  } /* endif */

  n = 0;

  for (f = fs_state.directory.first_file; f != fs_state.directory.next_file; f = wrap_file_number(f + 1)) {
    memcpy(&d[n], fs_state.directory.file_info[f].id, 16);
    n += 16;
  } /* endfor */

  return(n);
}


// Reports the id of the file being sent, and the next chunk number.  If no file is open
// for write, reports empty id and zero chunk number.
void get_send_file_report(unsigned char *d) {
  extern t_fs_state fs_state;

  if (fs_state.write_file == NO_FILE) {
    memset(d, 0, 16); // empty id
    put_hex(0, 8, &d[16]); // zero length
  } else {
    memcpy(d, fs_state.directory.file_info[fs_state.write_file].id, 16);
    put_hex(fs_state.directory.file_info[fs_state.write_file].length >> 9, 8, &d[16]);
  } /* endif */
}


// Gets the id of the current read file; zeros id if no file open.
void get_read_file_id(unsigned char *id) {
  extern t_fs_state fs_state;

  if (is_no_file(fs_state.read_file)) {
    memset(id, 0, 16);
  } else {
    memcpy(id, fs_state.directory.file_info[fs_state.read_file].id, 16);
  } /* endif */
}


// Starts a multi-block read which doesn't use DMA; subsequently, the reprogramming
// routine can simply use hsmci_read_word() repeatedly to obtain the entire firmware
// image.
// We start the search with the most recent file and work backwards, as is the case in
// start_read_file().  Unlike start_read_file(), we do not include any file which is
// currently open for write in the search.
// Returns the file length (in bytes) if successful; otherwise returns 0.
unsigned long start_read_firmware(unsigned char *id) {
  extern t_fs_state fs_state;
  unsigned char f, n;

  fs_state.read_file = NO_FILE;

  if (!prepare_sd_card()) {
    report_error(ERROR_SD_CARD);
    return(0L);
  } /* endif */

  f = fs_state.directory.next_file;
  n = wrap_file_number(fs_state.directory.next_file - fs_state.directory.first_file);

  while (n) {
    f = wrap_file_number(f - 1);

    if (memcmp((char *)fs_state.directory.file_info[f].id, (char *)id, 16) == 0) {
      if (!sd_mmc_read_block(fs_state.directory.file_info[f].start_block, NULL)) {
        return(0L);
      } /* endif */

      return(fs_state.directory.file_info[f].length);
    } /* endif */

    n --;
  } /* endwhile */

  return(0L);
}


void get_sd_card_state(unsigned char *d) {
  *d = PIO_Get(&SD_CD_PIN) ? '0' : '1';
}


unsigned char write_nonvolatile_data(unsigned char a, unsigned char d) {
  extern t_fs_state fs_state;

  if (!prepare_sd_card()) { // fail silently if SD card not present
    return(0);
  } /* endif */

  if (d != fs_state.directory.nonvolatile_data[a]) { // if update needed
    fs_state.directory.nonvolatile_data[a] = d;
    write_one_block(0L, (void *)&fs_state.directory);
    sd_mmc_finish(); // make sure write is flushed to SD card
  } /* endif */

  return(1);
}


unsigned char read_nonvolatile_data(unsigned char a, unsigned char *d) {
  extern t_fs_state fs_state;

  if (!prepare_sd_card()) { // fail if SD card not present
    return(0);
  } /* endif */

  *d = fs_state.directory.nonvolatile_data[a];
  return(1);
}


// Checks whether the head type has changed, and if so clears last_job so it can't be reprinted
// using the wrong type of head.
void check_for_head_change(void) {
  extern t_fs_state fs_state;
  unsigned char v;

  if (read_nonvolatile_data(1, &v)) { // silently do nothing if no SD card
    if (is_head_dual_material() != v) {
      fs_state.directory.last_job = NO_FILE;
      write_nonvolatile_data(1, is_head_dual_material());
    } /* endif */
  } /* endif */
}


t_fs_state fs_state;
