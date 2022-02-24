// This program inserts a 32 bit antichecksum into a .bin file.  The antichecksum's
// address offset is specified as a parameter.
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define FLASH_SIZE            0x20000


int main(int argc, char *argv[]) {
  unsigned char d[FLASH_SIZE];
  unsigned char c;
  unsigned long i, n, sum, offset;
  FILE          *f;

  if (argc < 3) {
    fprintf(stderr, "Must specify filename antichecksum_offset_hex!\n");
    return(1);
  } /* endif */

  if (sscanf(argv[2], "%lx", &offset) != 1) {
    fprintf(stderr, "Bad antichecksum offset!\n");
    return(1);
  } /* endif */

  if ((f = fopen(argv[1], "rb")) == NULL) {
    fprintf(stderr, "Can't open %s for read!\n", argv[1]);
    return(1);
  } /* endif */

  // get data...
  c = getc(f);

  for (n = 0; !feof(f); n ++) {
    d[min(n, FLASH_SIZE - 1)] = c;
    c = getc(f);
  } /* endfor */

  fclose(f);

  if (n > FLASH_SIZE) {
    fprintf(stderr, "Data is larger than the flash!\n");
    return(1);
  } else if (n & 0x000000ff) {
    fprintf(stderr, "Number of bytes is not a multiple of 256!\n");
    return(1);
  } else if (n < (offset + 4)) {
    fprintf(stderr, "Antichecksum location is not encompassed by %s!\n", argv[1]);
    return(1);
  } else if ((d[offset] != 0) || (d[offset + 1] != 0) || (d[offset + 2] != 0) || (d[offset + 3] != 0)) {
    fprintf(stderr, "Antichecksum would overwrite a non-zero location!\n");
    return(1);
  } /* endif */

  // calculate antichecksum and insert it... NB: do it a byte at a time; don't make assumptions about PC endian-ness
  sum = 0;

  for (i = 0; i < n; i ++) {
    sum += ((unsigned long)d[i]) << ((i & 3) << 3); // we're calculating long checksum so shift up 0, 8, 16 or 24
  } /* endfor */

  sum = (~sum) + 1;

  for (i = 0; i < 4; i ++) {
    d[offset + i] = (unsigned char)sum;
    sum = sum >> 8;
  } /* endfor */

  // output data...
  if ((f = fopen(argv[1], "wb")) == NULL) {
    fprintf(stderr, "Can't open %s for write!\n", argv[1]);
    return(1);
  } /* endif */

  for (i = 0; i < n; i ++) {
    putc(d[i], f);
  } /* endfor */

  fclose(f);
  return(0);
}
