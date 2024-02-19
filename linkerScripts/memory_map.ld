/*
******************************************************************************
**
**  File        : LinkerScript.ld
**
**  Author      : STM32CubeIDE
**
**  Abstract    : Linker script for STM32H7 series
**                2048Kbytes FLASH and 1056Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** Copyright (c) 2022 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
****************************************************************************
*/

/* Specify the memory areas */
MEMORY
{
  sharedram (rwx) : ORIGIN = 0x20000000, LENGTH = 10K
  bl1 (rw)        : ORIGIN = 0x08000000, LENGTH = 100K
  app1 (rx)       : ORIGIN = 0x08040000, LENGTH = 256K
  app2 (rx)       : ORIGIN = 0x08080000, LENGTH = 256K
  commondata (rwx): ORIGIN = 0x080C0000, LENGTH = 40K
  DTCMRAM (xrw)   : ORIGIN = 0x20002800, LENGTH = 128K - LENGTH(sharedram)
  RAM_D1 (xrw)    : ORIGIN = 0x24000000, LENGTH = 512K
  RAM_D2 (xrw)    : ORIGIN = 0x30000000, LENGTH = 288K
  RAM_D3 (xrw)    : ORIGIN = 0x38000000, LENGTH = 64K
  ITCMRAM (xrw)   : ORIGIN = 0x00000000, LENGTH = 64K
}

/* Define output sections */
SECTIONS
{
    .shared_memory (NOLOAD) : {
        KEEP(*(.shared_memory))
    } >sharedram
}

SECTIONS
{
    .common_data (NOLOAD) :
    {
        KEEP(*(.common_data))
    } > commondata
}

__sharedram_start__ = ORIGIN(sharedram);
__sharedram_size__ = LENGTH(sharedram);
__bootloader_start__ = ORIGIN(bl1);
__bootloader_size__ = LENGTH(bl1);
__app1_start__ = ORIGIN(app1);
__app1_size__ = LENGTH(app1);
__app2_start__ = ORIGIN(app2);
__app2_size__ = LENGTH(app2);
__commondata_start__ = ORIGIN(commondata);
