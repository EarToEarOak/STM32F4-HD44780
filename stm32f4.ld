/*
 * STM32F4-HD44780
 *
 * An interrupt driven library for HD44780-based LCD displays
 *
 * http://eartoearoak.com/software/stm32f4-hd44780
 *
 * Copyright 2013 Al Brown
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Minimum Stack and heap sizes */
Heap_Size = 0;
Stack_Size = 1024;

/* Stack at end of RAM */
_estack = 0x20020000;

MEMORY
{
	VECTOR (rx)		: ORIGIN = 0x08000000, LENGTH = 16k
	CONFIG (rw)		: ORIGIN = 0x08004000, LENGTH = 16k
	FLASH (rx)		: ORIGIN = 0x08008000, LENGTH = 1024k - 16k - 16k
	RAM (rw)		: ORIGIN = 0x20000000, LENGTH = 128k
}

ENTRY(Reset_Handler)

SECTIONS
{
	/* Interrupts */
	.isr_vector :
	{
		KEEP(*(.isr_vector))
	} >VECTOR

	/* Config */
	.config :
	{
		_config_start = .;
	} >CONFIG

	/* Code and constants */
	.text :
	{
		*(.text)
		*(.text*)
		*(.rodata)
		*(.rodata*)
		*(.glue_7)
		*(.glue_7t)
		*(.eh_frame)
		KEEP (*(.init))
		KEEP (*(.fini))
		. = ALIGN(4);
	} >FLASH

	.preinit_array     :
	{
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array*))
		PROVIDE_HIDDEN (__preinit_array_end = .);
	} >FLASH

	.init_array :
	{
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array*))
		PROVIDE_HIDDEN (__init_array_end = .);
	} >FLASH

	.fini_array :
	{
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(.fini_array*))
		KEEP (*(SORT(.fini_array.*)))
		PROVIDE_HIDDEN (__fini_array_end = .);
	} >FLASH

	_sidata = .;

	/* Initialised data  */
	.data :
	{
		. = ALIGN(4);
		_sdata = .;
		*(.data)
		*(.data*)
		. = ALIGN(4);
		_edata = .;
	} >RAM AT> FLASH

	/* Uninitialised data */
	.bss :
	{
		. = ALIGN(4);
		_sbss = .;
		__bss_start__ = _sbss;
		*(.bss)
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
		__bss_end__ = _ebss;
	} >RAM

	/* Heap */
	.heap :
	{
		. = ALIGN(4);
		end = .;
		. = . + Heap_Size;
	} >RAM

	/* Stack */
	.stack :
	{
		. = . + Stack_Size;
		. = ALIGN(4);
	} >RAM

    /DISCARD/ :
	{
		libc.a ( * )
		libm.a ( * )
		libgcc.a ( * )
		libnosys.a ( * )
	}
}
