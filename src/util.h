#ifndef UTIL_H
#define UTIL_H

#define SYSTICK_FREQUENCY 1000 // Hz; not critical, just a tradeoff between systick interrupt overhead and timer resolution

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

// Always use these macros, so that the calling context can specify times in seconds and doesn't need to know how
// the timer is implemented.
#define delay(p)                (((p) < 2.0e-3) ? short_delay_i((unsigned long)((p) * BOARD_MCK / 12.0)) : delay_i((unsigned long)((p) * SYSTICK_FREQUENCY)))
#define delay_aborting(p)       delay_aborting_i((unsigned long)((p) * SYSTICK_FREQUENCY))
#define has_period_elapsed(t,p) has_period_elapsed_i(t, (unsigned long)((p) * SYSTICK_FREQUENCY))

unsigned long get_hex(unsigned char s[], unsigned char n);
void put_hex(unsigned long v, unsigned char n, unsigned char s[]);
void delay_i(unsigned long v); // don't call this direct; use delay() macro
void delay_aborting_i(unsigned long v); // don't call this direct; use delay_aborting() macro
unsigned long get_time(void);
unsigned char has_period_elapsed_i(unsigned long start_time, unsigned long v); // don't call this direct; use has_period_elapsed() macro
void initialise_timer(void);
void short_delay_i(unsigned long v); // don't call this direct; use short_delay() macro

#endif
