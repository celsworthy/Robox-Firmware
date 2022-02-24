#include <stdlib.h>
#include <systick/systick.h>
#include "board.h"
#include "gcode.h"
#include "main.h"
#include "util.h"


// Gets a hex value from the first n characters of string s
unsigned long get_hex(unsigned char s[], unsigned char n) {
  unsigned long v;
  unsigned char temp;

  temp = s[n];
  s[n] = 0;
  v = strtoul((char *)s, NULL, 16);
  s[n] = temp;
  return(v);
}


// Puts an n character long hex string in s representing the value of v
void put_hex(unsigned long v, unsigned char n, unsigned char s[]) {
  while (n) {
    n --;

    if ((v & 0x0f) < 0x0a) {
      s[n] = '0' + (v & 0x0f);
    } else {
      s[n] = ('a' - 10) + (v & 0x0f);
    } /* endif */

    v = v >> 4;
  } /* endwhile */
}


// Waits for a time determined by v.  Note that the calculation works correctly even if timestamp has rolled
// over from 0xffffffff to 0x00000000.  It is intended that this routine is invoked via the delay() macro, so
// that the calling context can specify the time in seconds.
void delay_i(unsigned long v) {
  extern volatile unsigned long timestamp;
  unsigned long t;

  t = timestamp;

  while ((timestamp - t) < v) {
    __asm volatile("nop");
  } /* endwhile */
}


// Waits for a time determined by v, repeatedly calling do_everything _else(), and abandoning the wait on abort.
// Note that the calculation works correctly even if timestamp has rolled over from 0xffffffff to 0x00000000.
// It is intended that this routine is invoked via the delay() macro, so that the calling context can specify
// the time in seconds.
void delay_aborting_i(unsigned long v) {
  extern volatile unsigned long timestamp;
  unsigned long t;

  t = timestamp;

  while (!is_abort() && ((timestamp - t) < v)) {
    do_everything_else();
  } /* endwhile */
}


// Simply returns the current value of timestamp, for use in subsequent calls to has_period_elapsed().
unsigned long get_time(void) {
  extern volatile unsigned long timestamp;
  return(timestamp);
}


// Returns true if the period between now and start_time exceeds v.  Note that the calculation works correctly
// even if timestamp has rolled over from 0xffffffff to 0x00000000.  It is intended that this routine is invoked
// via the has_period_elapsed() macro, so that the calling contect can specify the period in seconds.
unsigned char has_period_elapsed_i(unsigned long start_time, unsigned long v) {
  extern volatile unsigned long timestamp;
  return((timestamp - start_time) >= v);
}


// Systick interrupt service routine.  It would be nice to use a hardware timer instead, and the SAM3U2E does
// have a 32 bit "real-time timer" (RTT).  Unfortunately, in the absence of a 32.768KHz crystal, it can only be
// clocked by the inaccurate RC oscillator, and even then it's not a clean solution as it is up to the firmware
// to detect bad reads caused by the timer changing asynchronously!  All in all it's easier to do it this way...
void SysTick_Handler(void) {
  extern volatile unsigned long timestamp;
  timestamp ++;
}


// This routine should be invoked at start-up to configure the systick hardware and enable the interrupt.
void initialise_timer(void) {
  extern volatile unsigned long timestamp;

  timestamp = 0;
  NVIC_SetPriority(SysTick_IRQn, 15); // make lower priority than motor stepping
  SysTick_Configure(1, BOARD_MCK / SYSTICK_FREQUENCY, SysTick_Handler);
}


void short_delay_i(unsigned long v) {
  while (v > 0) {
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    __asm volatile("nop");
    v --;
  } /* endwhile */
}


volatile unsigned long timestamp;
