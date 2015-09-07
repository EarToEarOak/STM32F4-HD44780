/**
 * @file
 * @defgroup 	HD44780 HD44780 4-Bit LCD Driver
 * @{
 * @brief		An interrupt driven library for HD44780-based LCD displays
 * @author		Al Brown
 * @copyright	Copyright &copy; 2013 - 2015 Al Brown
 */

/*
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

#ifndef __HD44780_H
#define __HD44780_H

#include <stdbool.h>
#include "stm32f4xx.h"

#define HD44780_QUEUE_FREQ  5000	/**< Update Frequency */
#define HD44780_QUEUE_SIZE  500		/**< Queue Size */

/**
 * Lines
 */
typedef enum {
	HD44780_LINES_1 = 0x00, 		/**< 1 Line */
	HD44780_LINES_2 = 0x08			/**< 2 or 4 Lines */
} hd44780_lines_type;

/**
 * Font
 */
typedef enum {
	HD44780_FONT_5x8 = 0x00,		/**< Standard Font */
	HD44780_FONT_5x10 = 0x04 		/**< Large Font */
} hd44780_font_type;

void hd44780_brightness(const uint8_t brightness);
void hd44780_contrast(const uint8_t contrast);
void hd44780_clear(void);
void hd44780_home(void);
void hd44780_display(const bool enable, const bool cursor, const bool blink);
void hd44780_position(const uint8_t row, const uint8_t col);
void hd44780_cgram(const uint8_t pos, const char row[8]);
void hd44780_put(const char chr);
void hd44780_print(const char* string);
void hd44780_printf(const char *fmt, ...);
void hd44780_init(GPIO_TypeDef* port, const uint16_t rs, const uint16_t rw,
		const uint16_t e, const uint16_t db4, const uint16_t db5,
		const uint16_t db6, const uint16_t db7, const hd44780_lines_type lines,
		const hd44780_font_type font);
void hd44780_init_brightness();
void hd44780_init_contrast();

#endif

/** @}*/
