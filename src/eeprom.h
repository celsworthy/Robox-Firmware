#ifndef EEPROM_H
#define EEPROM_H

void initialise_eeproms(void);
void maintain_eeproms(void);
unsigned short get_eeprom_read(unsigned char eeprom_sel, unsigned char *d);
float get_eeprom_float(unsigned char eeprom_sel, unsigned short address, float def);
void write_eeprom_float(unsigned char eeprom_sel, unsigned short address, float v);
void write_eeprom(unsigned char eeprom_sel, unsigned short address, unsigned short n, unsigned char *d);
void format_eeprom(unsigned char eeprom_sel);
void get_eeprom_states(unsigned char *d);
void set_head_output(unsigned char bitno, unsigned char value);
unsigned char is_head_eeprom_valid(void);
unsigned char is_head_dual_material(void);
unsigned char does_head_have_two_fans(void);
unsigned char does_head_have_b_axis(void);
unsigned char does_head_have_prtd(void);
float get_head_max_temperature(void);
unsigned char is_reel0_present(void);
unsigned char is_reel1_present(void);

#endif
