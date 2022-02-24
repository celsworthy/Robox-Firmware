#ifndef REPROGRAM_H
#define REPROGRAM_H

unsigned char is_firmware_file_ok(unsigned char *id);
void reprogram(unsigned char *id);
void flash_checksum_test(void);

#endif
