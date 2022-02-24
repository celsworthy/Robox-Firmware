#ifndef SD_MMC_H
#define SD_MMC_H

//! This SD MMC stack uses the maximum block size autorized (512 bytes)
#define SD_MMC_BLOCK_SIZE          512

unsigned char sd_mmc_init(void);
unsigned long sd_mmc_get_capacity(void); // in KB
void sd_mmc_finish(void);
unsigned char sd_mmc_read_block(unsigned long block, unsigned char *d);
unsigned char sd_mmc_write_block(unsigned long block, unsigned char *d);

#endif
