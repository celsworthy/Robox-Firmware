/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

#include <string.h>
#include "board.h"
#include "exceptions.h"
#include "board_lowlevel.h"
#include <pio/pio.h>


//------------------------------------------------------------------------------
// This is the code that gets called on processor reset.  It initialises
// the device, initialises RAM variables then calls main().
//------------------------------------------------------------------------------

void ResetException(void) {
  extern unsigned long _sinit, _srelocate, _erelocate, _szero, _ezero;
  extern int main(void);

  // By getting the head power turned on ASAP (whilst the 24V supply is still on the way up) we
  // avoid causing severe ground bounce at the head, which can leave the B motor driver in a
  // shut down state...
  // Rev 2 hardware doesn't need this work-around, and in fact it would be undesirable to turn on
  // the head power as it defeats the object of the head short detection circuit on rev 2.
  // Fortunately PA18 does something different on rev 2, and there is no harm in setting it!
  AT91C_BASE_PIOA->PIO_SODR = 1 << 18;
  AT91C_BASE_PIOA->PIO_OER = 1 << 18;
  AT91C_BASE_PIOA->PIO_PER = 1 << 18;

  LowLevelInit();
  memcpy(&_srelocate, &_sinit, (unsigned long)&_erelocate - (unsigned long)&_srelocate);
  memset(&_szero, 0, (unsigned long)&_ezero - (unsigned long)&_szero);
  main();
}


//------------------------------------------------------------------------------
//         Exception Table
//------------------------------------------------------------------------------

// Stack top
extern unsigned long _estack;

__attribute__((section(".vectors"))) IntFunc exception_table[] = {
    // Configure Initial Stack Pointer, using linker-generated symbols
    (IntFunc)&_estack,
    ResetException,

    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,             // Reserved
    SVC_Handler,
    DebugMon_Handler,
    0,                      // Reserved
    PendSV_Handler,
    SysTick_Handler,

    // Configurable interrupts
    SUPC_IrqHandler,    // 0  SUPPLY CONTROLLER
    RSTC_IrqHandler,    // 1  RESET CONTROLLER
    RTC_IrqHandler,     // 2  REAL TIME CLOCK
    RTT_IrqHandler,     // 3  REAL TIME TIMER
    WDT_IrqHandler,     // 4  WATCHDOG TIMER
    PMC_IrqHandler,     // 5  PMC
    EFC0_IrqHandler,    // 6  EFC0
    EFC1_IrqHandler,    // 7  EFC1
    DBGU_IrqHandler,    // 8  DBGU
    HSMC4_IrqHandler,   // 9  HSMC4
    PIOA_IrqHandler,    // 10 Parallel IO Controller A
    PIOB_IrqHandler,    // 11 Parallel IO Controller B
    PIOC_IrqHandler,    // 12 Parallel IO Controller C
    USART0_IrqHandler,  // 13 USART 0
    USART1_IrqHandler,  // 14 USART 1
    USART2_IrqHandler,  // 15 USART 2
    USART3_IrqHandler,  // 16 USART 3
    MCI0_IrqHandler,    // 17 Multimedia Card Interface
    TWI0_IrqHandler,    // 18 TWI 0
    TWI1_IrqHandler,    // 19 TWI 1
    SPI0_IrqHandler,    // 20 Serial Peripheral Interface
    SSC0_IrqHandler,    // 21 Serial Synchronous Controller 0
    TC0_IrqHandler,     // 22 Timer Counter 0
    TC1_IrqHandler,     // 23 Timer Counter 1
    TC2_IrqHandler,     // 24 Timer Counter 2
    PWM_IrqHandler,     // 25 Pulse Width Modulation Controller
    ADCC0_IrqHandler,   // 26 ADC controller0
    ADCC1_IrqHandler,   // 27 ADC controller1
    HDMA_IrqHandler,    // 28 HDMA
    UDPD_IrqHandler,   // 29 USB Device High Speed UDP_HS
    IrqHandlerNotUsed   // 30 not used
};
