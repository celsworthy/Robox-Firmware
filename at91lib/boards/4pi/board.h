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

#ifndef BOARD_H
#define BOARD_H

#include "at91sam3u4/chip.h"
#include "at91sam3u4/AT91SAM3U4.h"


/// Master clock frequency (when using board_lowlevel.c).
#define BOARD_MCK               96000000

/// Chip has a UDP controller.
#define BOARD_USB_UDPHS

/// Indicates the D+ pull-up is external.
#define BOARD_USB_PULLUP_INTERNAL

/// Number of endpoints in the USB controller.
#define BOARD_USB_NUMENDPOINTS              7

/// Returns the maximum packet size of the given endpoint.
#define BOARD_USB_ENDPOINTS_MAXPACKETSIZE(i) (((i == 0) || (i == 3) || (i == 4)) ? 64 :\
                                             (((i == 1) || (i == 2)) ? 512 : 1024))

/// Returns the number of FIFO banks for the given endpoint.
#define BOARD_USB_ENDPOINTS_BANKS(i)        ((i == 0) ? 1 : ((i == 1) || (i == 2)) ? 2 : 3)

/// USB attributes configuration descriptor (bus or self powered, remote wakeup)
#define BOARD_USB_BMATTRIBUTES              USBConfigurationDescriptor_SELFPOWERED_RWAKEUP
//#define BOARD_USB_BMATTRIBUTES            USBConfigurationDescriptor_SELFPOWERED_NORWAKEUP


/// Startup time max, return from Idle mode (in µs)
#define ADC_STARTUP_TIME_MAX       15
/// Track and hold Acquisition Time min (in ns)
#define ADC_TRACK_HOLD_TIME_MIN  1200
/// ADC_AD0 pin definition.
#define PIN_ADC0_AD0 {1 << 22, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_DEFAULT}
/// ADC_AD1 pin definition.
#define PIN_ADC0_AD1 {1 << 30, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_DEFAULT}
/// ADC_AD2 pin definition.
#define PIN_ADC0_AD2 {1 << 3, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT}
/// ADC_AD3 pin definition.
#define PIN_ADC0_AD3 {1 << 4, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_DEFAULT}
/// ADC_AD4 pin definition.
#define PIN_ADC0_AD4 {1 << 15, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_DEFAULT}
/// ADC_AD5 pin definition.
#define PIN_ADC0_AD5 {1 << 16, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_DEFAULT}
/// ADC_AD6 pin definition.
#define PIN_ADC0_AD6 {1 << 17, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_DEFAULT}
/// ADC_AD7 pin definition.
#define PIN_ADC0_AD7 {1 << 18, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_DEFAULT}
/// Pins ADC
#define PINS_ADC PIN_ADC0_AD0, PIN_ADC0_AD1, PIN_ADC0_AD2, PIN_ADC0_AD3, PIN_ADC0_AD4, PIN_ADC0_AD5, PIN_ADC0_AD6, PIN_ADC0_AD7


/// DBGU pins (DTXD and DRXD) definitions, PA11,12.
#define PINS_DBGU  {0x00001800, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}


// HSMCI
#define PINS_HSMCI {0x1f8, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_A, PIO_PULLUP}


/// USART1
#define PIN_USART1_RXD    {0x1 << 21, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_USART1_TXD    {0x1 << 20, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_USART1_CTS    {0x1 << 23, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
#define PIN_USART1_RTS    {0x1 << 22, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}
#define PIN_USART1_SCK    {0x1 << 24, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT}

// This seems to be a repeat of the ...IFLASH0 values defined in AT91SAM3U4.h...
#define AT91C_IFLASH                   (0x80000)
#define AT91C_IFLASH_SIZE              (0x20000)
#define AT91C_IFLASH_PAGE_SIZE             (256) // Internal FLASH 0 Page Size: 256 bytes
#define AT91C_IFLASH_NB_OF_PAGES           (512) // Internal FLASH 0 Number of Pages: 512
#define AT91C_IFLASH_LOCK_REGION_SIZE     (8192) // Internal FLASH 0 Lock Region Size: 8 Kbytes
#define AT91C_IFLASH_NB_OF_LOCK_BITS        (16) // Internal FLASH 0 Number of Lock Bits: 32

// Indicates chip has an EFC.
#define AT91C_BASE_EFC    AT91C_BASE_EFC0

// Interrupt source (seems like this really belongs in AT91SAM3U4.h)
typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ***************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                             */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M3 Memory Management Interrupt              */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M3 Bus Fault Interrupt                      */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M3 Usage Fault Interrupt                    */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M3 SV Call Interrupt                       */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M3 Debug Monitor Interrupt                 */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M3 Pend SV Interrupt                       */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M3 System Tick Interrupt                   */

/******  AT91SAM3U4 specific Interrupt Numbers *********************************************************/
 IROn_SUPC                = AT91C_ID_SUPC , // SUPPLY CONTROLLER
 IROn_RSTC                = AT91C_ID_RSTC , // RESET CONTROLLER
 IROn_RTC                 = AT91C_ID_RTC  , // REAL TIME CLOCK
 IROn_RTT                 = AT91C_ID_RTT  , // REAL TIME TIMER
 IROn_WDG                 = AT91C_ID_WDG  , // WATCHDOG TIMER
 IROn_PMC                 = AT91C_ID_PMC  , // PMC
 IROn_EFC0                = AT91C_ID_EFC0 , // EFC0
 IROn_EFC1                = AT91C_ID_EFC1 , // EFC1
 IROn_DBGU                = AT91C_ID_DBGU , // DBGU
 IROn_HSMC4               = AT91C_ID_HSMC4, // HSMC4
 IROn_PIOA                = AT91C_ID_PIOA , // Parallel IO Controller A
 IROn_PIOB                = AT91C_ID_PIOB , // Parallel IO Controller B
 IROn_PIOC                = AT91C_ID_PIOC , // Parallel IO Controller C
 IROn_US0                 = AT91C_ID_US0  , // USART 0
 IROn_US1                 = AT91C_ID_US1  , // USART 1
 IROn_US2                 = AT91C_ID_US2  , // USART 2
 IROn_US3                 = AT91C_ID_US3  , // USART 3
 IROn_MCI0                = AT91C_ID_MCI0 , // Multimedia Card Interface
 IROn_TWI0                = AT91C_ID_TWI0 , // TWI 0
 IROn_TWI1                = AT91C_ID_TWI1 , // TWI 1
 IROn_SPI0                = AT91C_ID_SPI0 , // Serial Peripheral Interface
 IROn_SSC0                = AT91C_ID_SSC0 , // Serial Synchronous Controller 0
 IROn_TC0                 = AT91C_ID_TC0  , // Timer Counter 0
 IROn_TC1                 = AT91C_ID_TC1  , // Timer Counter 1
 IROn_TC2                 = AT91C_ID_TC2  , // Timer Counter 2
 IROn_PWMC                = AT91C_ID_PWMC , // Pulse Width Modulation Controller
 IROn_ADCC0               = AT91C_ID_ADC12B, // ADC controller0
 IROn_ADCC1               = AT91C_ID_ADC, // ADC controller1
 IROn_HDMA                = AT91C_ID_HDMA , // HDMA
 IROn_UDPHS               = AT91C_ID_UDPHS // USB Device High Speed
} IRQn_Type;


#endif //#ifndef BOARD_H

