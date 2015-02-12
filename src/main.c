/*
 * STM32F4-HD44780
 *
 * An interrupt driven library for HD44780-based LCD displays
 *
 * http://eartoearoak.com/software/stm32f4-hd44780
 *
 * Copyright 2013 - 2015 Al Brown
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

/*
 * Pins:
 *
 *		STM32			LCD
 *
 * 		5V		 -		5V
 *		5V		 -		A
 *		GND		 -		GND
 *		PE7		 ->		RS
 *		PE8		 ->		RW
 *		PE9		 ->		E
 *		PE10	<->		DB4
 *		PE11	<->		DB5
 *		PE12	<->		DB6
 *		PE13	<->		DB7
 *	 	PB15	 ->		K
 *	 	PA5		 ->		VO
 *
 */

#include <stdbool.h>
#include <stm32f4xx.h>
#include <stm32f4_discovery.h>

#include "hd44780.h"

#include "main.h"

#define UDG	0

int main(void) {

	char udg[] = { 0x00, 0x00, 0x0a, 0x00, 0x11, 0x0e, 0x00, 0x00 };

	hd44780_init(GPIOE, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10,
			GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, HD44780_LINES_2,
			HD44780_FONT_5x8);

	hd44780_init_brightness();
	hd44780_brightness(85);
	hd44780_init_contrast();
	hd44780_contrast(50);

	hd44780_cgram(UDG, udg);
	hd44780_position(0, 1);
	hd44780_print("Hello World! ");
	hd44780_put(UDG);
	hd44780_display(true, false, false);

	while (1)
		;

	return (0);
}
