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

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

#include "hd44780.h"
#include "stm32f4xx.h"

#define HD44780_TIMER 		TIM7
#define HD44780_PRIORITY 	6

#define BKL_TIMER 			TIM12
#define BKL_CHANNEL			TIM_CHANNEL_2

#define BKL_TIME			1000
#define BKL_FREQ			125000
#define BKL_PERIOD			(SystemCoreClock / 2) / BKL_FREQ

#define BKL_GPIO			GPIOB
#define BKL_PIN				GPIO_PIN_15
#define BKL_AF 				GPIO_AF9_TIM12

#define MAX_CONTRAST		3000

#define NONE 0

#define HD44780_4BIT 		0x00
#define HDD44780_ENTRY_LEFT	0x02
#define HD44780_DISPLAY_ON	0x04
#define HD44780_CURSOR_ON	0x02
#define HD44780_BLINK_ON	0x01

#define HD44780_CLEAR		0x0001
#define HD44780_HOME		0x0002
#define HD44780_ENTRY		0x0004
#define HD44780_DISPLAY		0x0008
#define HD44780_CURSOR		0x0010
#define HD44780_FUNCTION	0x0020
#define HD44780_CGRAM		0x0040
#define HD44780_DGRAM		0x0080

typedef enum {
	HD44780_WAIT = 1, HD44780_WAIT_NOT_BUSY, HD44780_WRITE
} hd44780_command_t;

typedef struct {
	hd44780_command_t command;
	bool reg;
	uint16_t data;
	uint8_t nibble;
} hd44780_task_t;

typedef struct {
	GPIO_TypeDef* gpio;
	uint32_t rs;
	uint32_t rw;
	uint32_t e;
	uint32_t db4;
	uint32_t db5;
	uint32_t db6;
	uint32_t db7;
	uint8_t lines;
	uint8_t font;
} hd44780_conf_t;

static TIM_HandleTypeDef TIM_Handle_Lcd;
TIM_HandleTypeDef TIM_Handle_Brigt;

static hd44780_conf_t Lcd_Conf;
static volatile hd44780_task_t Queue[HD44780_QUEUE_SIZE];
static volatile uint16_t Queue_Head = 0;
static volatile uint16_t Queue_Tail = 0;

static void delay(uint8_t delay) {

	while (delay != 0)
		delay--;
}

static void set_output(const bool output) {

	GPIO_InitTypeDef GPIO_InitStruct;
	uint32_t pins;
	uint8_t dir;

	pins = Lcd_Conf.db4 | Lcd_Conf.db5 | Lcd_Conf.db6 | Lcd_Conf.db7;
	dir = GPIO_MODE_INPUT;

	if (output) {
		pins = pins | Lcd_Conf.rs | Lcd_Conf.rw | Lcd_Conf.e;
		dir = GPIO_MODE_OUTPUT_PP;
	}

	GPIO_InitStruct.Pin = pins;
	GPIO_InitStruct.Mode = dir;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;

	HAL_GPIO_Init(Lcd_Conf.gpio, &GPIO_InitStruct);
}

static void enable(const bool pulse) {

	if (pulse) {
		HAL_GPIO_TogglePin(Lcd_Conf.gpio, Lcd_Conf.e);
		delay(150);
	}
	HAL_GPIO_TogglePin(Lcd_Conf.gpio, Lcd_Conf.e);
	delay(150);
}

static void write(const uint8_t data, const bool reg) {

	set_output(true);

	HAL_GPIO_WritePin(Lcd_Conf.gpio, Lcd_Conf.rw, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Lcd_Conf.gpio, Lcd_Conf.rs, !reg);
	HAL_GPIO_WritePin(Lcd_Conf.gpio, Lcd_Conf.db7, (data & 0x8) >> 3);
	HAL_GPIO_WritePin(Lcd_Conf.gpio, Lcd_Conf.db6, (data & 0x4) >> 2);
	HAL_GPIO_WritePin(Lcd_Conf.gpio, Lcd_Conf.db5, (data & 0x2) >> 1);
	HAL_GPIO_WritePin(Lcd_Conf.gpio, Lcd_Conf.db4, (data & 0x1));
	enable(true);
}

static bool read_busy(void) {

	uint8_t data;

	set_output(false);
	HAL_GPIO_WritePin(Lcd_Conf.gpio, Lcd_Conf.rs | Lcd_Conf.db7,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Lcd_Conf.gpio, Lcd_Conf.rw, GPIO_PIN_SET);
	enable(false);
	data = HAL_GPIO_ReadPin(Lcd_Conf.gpio, Lcd_Conf.db7);
	enable(false);
	enable(true);

	if (data == 1)
		return (true);

	return (false);

}

static void task_add(const hd44780_command_t command, const bool reg,
		const uint16_t data, uint8_t nibbles) {

	if (command == HD44780_WAIT_NOT_BUSY)
		nibbles = 2;

	nibbles--;

	if (Queue_Head != (Queue_Tail + 1) % (HD44780_QUEUE_SIZE - 1)) {
		Queue[Queue_Tail].command = command;
		Queue[Queue_Tail].reg = reg;
		Queue[Queue_Tail].data = data;
		Queue[Queue_Tail].nibble = nibbles;
		Queue_Tail = (Queue_Tail + 1) % (HD44780_QUEUE_SIZE - 1);
	} else
		assert_param(false);
}

static void task_del(void) {

	if (Queue_Head != Queue_Tail) {
		Queue_Head = (Queue_Head + 1) % (HD44780_QUEUE_SIZE - 1);
	}
}

static void exec(void) {

	volatile hd44780_task_t *task;

	if (Queue_Head != Queue_Tail) {
		task = &Queue[Queue_Head];
		switch (task->command) {
		case HD44780_WAIT:
			if (task->data > 0)
				task->data--;
			else {
				task_del();
				exec();
			}
			break;
		case HD44780_WAIT_NOT_BUSY:
			if (!read_busy()) {
				task_del();
				exec();
			}
			break;
		case HD44780_WRITE:
			if (task->nibble == 1)
				write(task->data >> 4, task->reg);
			write(task->data & 0xf, task->reg);
			task_del();
			exec();
			break;
		}
	}
}

/**
 * Set the brightness
 *
 * @param brightness	0-100
 *
 */
void hd44780_brightness(const uint8_t brightness) {

	uint16_t bright;

	bright = (BKL_PERIOD * brightness) / 100;
	__HAL_TIM_SetCompare(&TIM_Handle_Brigt, BKL_CHANNEL, bright);
}

/**
 * Clear the LCD
 */
void hd44780_clear(void) {

	task_add(HD44780_WRITE, true, HD44780_CLEAR, 2);
	task_add(HD44780_WAIT_NOT_BUSY, NONE, NONE, NONE);
}

/**
 * Move cursor home
 */
void hd44780_home(void) {

	task_add(HD44780_WRITE, true, HD44780_HOME, 2);
	task_add(HD44780_WAIT_NOT_BUSY, NONE, NONE, NONE);
}

/**
 * Display settings
 *
 * @param enable	LCD on/off
 * @param cursor	Cursor on/off
 * @param blink		Blinking cursor on/off
 *
 */
void hd44780_display(const bool enable, const bool cursor, const bool blink) {

	uint16_t command = HD44780_DISPLAY;

	if (enable)
		command |= HD44780_DISPLAY_ON;
	else {
		Queue_Head = Queue_Tail = 0;
	}
	if (cursor)
		command |= HD44780_CURSOR_ON;
	if (blink)
		command |= HD44780_BLINK_ON;

	task_add(HD44780_WRITE, true, command, 2);
	task_add(HD44780_WAIT_NOT_BUSY, NONE, NONE, NONE);
}

/**
 * Position the cursor
 *
 * @param row	Row
 * @param col	Column
 */
void hd44780_position(const uint8_t row, const uint8_t col) {

	uint16_t command = HD44780_DGRAM;
	const uint8_t offsets[] = { 0x00, 0x40, 0x14, 0x54 };

	command |= col + offsets[row];
	task_add(HD44780_WRITE, true, command, 2);
	task_add(HD44780_WAIT_NOT_BUSY, NONE, NONE, NONE);
}

/**
 * Set a UDG
 *
 * @param pos	UDG number
 * @param udg	UDG definition
 */
void hd44780_cgram(const uint8_t pos, const char udg[8]) {

	uint8_t i;
	uint16_t command = HD44780_CGRAM;

	assert_param(pos < 8);

	command |= pos * 8;

	task_add(HD44780_WRITE, true, command, 2);
	for (i = 0; i < 8; i++) {
		task_add(HD44780_WRITE, false, udg[i], 2);
		task_add(HD44780_WAIT_NOT_BUSY, NONE, NONE, NONE);
	}
}

/**
 * Send a single character to the LCD
 *
 * @param chr	Character to send
 */
void hd44780_put(const char chr) {

	task_add(HD44780_WRITE, false, chr, 2);
	task_add(HD44780_WAIT_NOT_BUSY, NONE, NONE, NONE);
}

/**
 * Send a string to the LCD
 *
 * @param string	String to send
 */
void hd44780_print(const char* string) {

	uint8_t i = 0;

	while (string[i]) {
		task_add(HD44780_WRITE, false, string[i], 2);
		task_add(HD44780_WAIT_NOT_BUSY, NONE, NONE, NONE);
		i++;
	}
}

/**
 * Send a formatted string to the LCD
 *
 * @param fmt	String format
 * @param ...	Variable arguments (see printf())
 */
void hd44780_printf(const char *fmt, ...) {

	uint16_t i;
	uint16_t size;
	uint8_t character;
	char buffer[32];
	va_list args;

	va_start(args, fmt);
	size = vsprintf(buffer, fmt, args);

	for (i = 0; i < size; i++) {
		character = buffer[i];

		if (character == 10)
			break;
		else
			hd44780_put(character);
	}
}

/**
 * Initialise the LCD
 *
 * @param gpio	GPIO Port
 * @param rs	RS line
 * @param rw	RW line
 * @param e		E line
 * @param db4	DB4 line
 * @param db5	DB5 line
 * @param db6	DB6 line
 * @param db7	DB7 line
 * @param lines	Lines
 * @param font	Font
 */
void hd44780_init(GPIO_TypeDef* gpio, const uint16_t rs, const uint16_t rw,
		const uint16_t e, const uint16_t db4, const uint16_t db5,
		const uint16_t db6, const uint16_t db7, const hd44780_lines_t lines,
		const hd44780_font_t font) {

	assert_param(IS_GPIO_ALL_INSTANCE(gpio));
	assert_param(IS_GPIO_PIN(rs));
	assert_param(IS_GPIO_PIN(rw));
	assert_param(IS_GPIO_PIN(e));
	assert_param(IS_GPIO_PIN(db4));
	assert_param(IS_GPIO_PIN(db5));
	assert_param(IS_GPIO_PIN(db6));
	assert_param(IS_GPIO_PIN(db7));

	Lcd_Conf.gpio = gpio;
	Lcd_Conf.rs = rs;
	Lcd_Conf.rw = rw;
	Lcd_Conf.e = e;
	Lcd_Conf.db4 = db4;
	Lcd_Conf.db5 = db5;
	Lcd_Conf.db6 = db6;
	Lcd_Conf.db7 = db7;
	Lcd_Conf.lines = lines;
	Lcd_Conf.font = font;

	if (gpio == GPIOA)
		__GPIOA_CLK_ENABLE();
	else if (gpio == GPIOB)
		__GPIOB_CLK_ENABLE();
	else if (gpio == GPIOC)
		__GPIOC_CLK_ENABLE();
	else if (gpio == GPIOD)
		__GPIOD_CLK_ENABLE();
	else if (gpio == GPIOE)
		__GPIOE_CLK_ENABLE();
	else
		return;

	__TIM7_CLK_ENABLE();
	HAL_NVIC_SetPriority(TIM7_IRQn, 1, HD44780_PRIORITY);
	HAL_NVIC_EnableIRQ(TIM7_IRQn);

	TIM_Handle_Lcd.Instance = HD44780_TIMER;
	TIM_Handle_Lcd.Init.Period = (62500 / HD44780_QUEUE_FREQ) - 1;
	TIM_Handle_Lcd.Init.Prescaler = ((SystemCoreClock / 2) / 62500) - 1;
	TIM_Handle_Lcd.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM_Handle_Lcd.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_Base_Init(&TIM_Handle_Lcd);
	HAL_TIM_Base_Start_IT(&TIM_Handle_Lcd);

	task_add(HD44780_WAIT, true, 1000, 1);
	task_add(HD44780_WRITE, true, 3, 1);
	task_add(HD44780_WAIT, NONE, 5, NONE);
	task_add(HD44780_WRITE, true, 3, 1);
	task_add(HD44780_WAIT, NONE, 1, NONE);
	task_add(HD44780_WRITE, true, 3, 1);
	task_add(HD44780_WAIT, NONE, 1, NONE);
	task_add(HD44780_WRITE, true, 2, 1);
	task_add(HD44780_WAIT, NONE, 1, NONE);
	task_add(HD44780_WRITE, true,
	HD44780_FUNCTION | HD44780_4BIT | Lcd_Conf.lines | Lcd_Conf.font, 2);
	task_add(HD44780_WAIT, NONE, 1, NONE);
	task_add(HD44780_WRITE, true,
	HD44780_DISPLAY | HD44780_DISPLAY_ON | !HD44780_CURSOR_ON, 2);
	task_add(HD44780_WAIT, NONE, 1, NONE);
	task_add(HD44780_WRITE, true, HD44780_ENTRY | HDD44780_ENTRY_LEFT, 2);
	task_add(HD44780_WAIT_NOT_BUSY, NONE, NONE, NONE);
	hd44780_clear();
	hd44780_home();
}

/**
 * Initialise the brightness control
 *
 */
void hd44780_init_brightness(void) {

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_OC_InitTypeDef TIM_OCInitStructure;

	__GPIOB_CLK_ENABLE();
	GPIO_InitStructure.Pin = BKL_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
	GPIO_InitStructure.Alternate = BKL_AF;
	HAL_GPIO_Init(BKL_GPIO, &GPIO_InitStructure);

	__TIM12_CLK_ENABLE();

	TIM_Handle_Brigt.Instance = BKL_TIMER;
	TIM_Handle_Brigt.Init.Period = (SystemCoreClock / 2) / BKL_FREQ;
	TIM_Handle_Brigt.Init.Prescaler = 1 - 1;
	TIM_Handle_Brigt.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM_Handle_Brigt.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(&TIM_Handle_Brigt);

	TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
	TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
	TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	TIM_OCInitStructure.Pulse = 300;
	TIM_OCInitStructure.OCPolarity = TIM_OCNPOLARITY_HIGH;

	HAL_TIM_PWM_ConfigChannel(&TIM_Handle_Brigt, &TIM_OCInitStructure,
	BKL_CHANNEL);
	HAL_TIM_PWM_Start(&TIM_Handle_Brigt, BKL_CHANNEL);
}

void TIM7_IRQHandler(void) {

	exec();
	HAL_TIM_IRQHandler(&TIM_Handle_Lcd);
}
