#ifndef GCODE_H
#define GCODE_H

unsigned short gcode_handler(char *line, char *response);
void initialise_job(void);
void start_job(unsigned char *id);
void maintain_job(void);
void pause_and_park(void);
void pause_resume_job(unsigned char *d);
void invert_pause(void);
void abort_job(void);
void execute_gcode(unsigned char *d, unsigned short n);
void get_job_state(unsigned char *d);
unsigned char is_abort(void);
unsigned char is_job_running(void);
unsigned char is_paused(void);
unsigned char is_non_selfie_paused(void);
unsigned char is_gcode_reporting(unsigned char *d, unsigned short n);

signed long get_int(char *line, char c);
unsigned long get_uint(char *line, char c);
float get_float(char *line, char c);
unsigned char has_float(char *line, char c);
unsigned char get_bool(char *line, char c);
char* get_str(char *line, char c);
unsigned char has_code(char *line, char c);

#endif
