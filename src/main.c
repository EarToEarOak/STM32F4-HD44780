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

#include <stdbool.h>

#include "hd44780.h"
#include "stm32f4xx_conf.h"

#include "main.h"

#define UDG	0

int main(void) {

	char udg[] = { 0x00, 0x00, 0x0a, 0x00, 0x11, 0x0e, 0x00, 0x00 };

	hd44780_init(GPIOE, GPIO_Pin_7, GPIO_Pin_8, GPIO_Pin_9, GPIO_Pin_10,
			GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_13, HD44780_LINES_2,
			HD44780_FONT_5x8);

	hd44780_cgram(UDG, udg);
	hd44780_position(0, 1);
	hd44780_print("Hello World! ");
	hd44780_put(UDG);
	hd44780_display(true, false, false);

	while (1)
		;

	return (0);
}

#ifdef USE_FULL_ASSERT

void assert_failed(u8* file, u32 line) {

	(void) file;
	(void) line;

	while (1)
		;
}

#endif
