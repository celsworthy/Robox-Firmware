#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

void initialise_user_interface(void);
void maintain_user_interface(void);
void set_ambient_led(unsigned char *d);
void set_button_led(unsigned char *d);
void get_button_state(unsigned char *d);
unsigned char was_button_pressed_at_start(void);

#endif
