#ifndef FILE_SYSTEM_H
#define FILE_SYSTEM_H

void initialise_file_system(void);
void maintain_file_system(void);
void format_sd_card(void);
void start_write_file(unsigned char *id, unsigned char is_job);
void write_file(unsigned long sequence_number, unsigned char *d);
void end_write_file(unsigned long sequence_number, unsigned short n, unsigned char *d);
unsigned char start_read_file(unsigned char *id);
unsigned short read_file(unsigned char *d);
void close_read_file(void);
unsigned char is_eof(void);
unsigned short get_file_list(unsigned char *d);
void get_send_file_report(unsigned char *d);
void get_read_file_id(unsigned char *id);
unsigned long start_read_firmware(unsigned char *id);
void get_sd_card_state(unsigned char *d);
unsigned char write_nonvolatile_data(unsigned char a, unsigned char d);
unsigned char read_nonvolatile_data(unsigned char a, unsigned char *d);
void check_for_head_change(void);

#endif
