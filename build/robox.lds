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

/*------------------------------------------------------------------------------
 *      Linker script for running in internal FLASH on AT91SAM3U4E
 *      Should also work on AT91SAM3U2E
 *----------------------------------------------------------------------------*/

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(ResetException)


/* Memory Spaces Definitions */
MEMORY
{
    sram0  (W!RX) : ORIGIN = 0x2007C000, LENGTH = 0x00008000 /* Srams0 & 1 as contiguous block, 32K */
    flash0 (W!RX) : ORIGIN = 0x00080000, LENGTH = 0x00020000 /* Flash0, 128K */
}

SECTIONS
{
    .fixed :
    {
        _sprog = .;
        KEEP(*(.vectors))
        _antichecksum = .;
        LONG(0);               /* Space for antichecksum to be inserted */
        *(.text*)
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)
        . = ALIGN(256);        /* So the programming file is an integer number of flash pages long */
        _sinit = .;            /* Start of initialising values */
    } >flash0

    .relocate : AT (_sinit)
    {
        . = ALIGN(4);
        _srelocate = .;
        *(.ramfunc);
        *(.data);
        *(.data.*);
        . = ALIGN(4);
        _erelocate = .;
        . = ALIGN(256);        /* So the programming file is an integer number of flash pages long */
    } >sram0

    .bss (NOLOAD) : {
        . = ALIGN(4);
        _szero = .;
        *(.bss)
        *(COMMON)
        . = ALIGN(4);
        _ezero = .;
    } >sram0

    _eprog = _sinit + SIZEOF(.relocate); /* End of used area of flash */

    /* Stack at the end of SRAM */
    _estack = 0x20084000;
}
end = .;
PROVIDE(__HEAP_START = end );
