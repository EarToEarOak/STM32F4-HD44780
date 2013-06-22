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


#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

#include "hd44780.h"
#include "stm32f4xx_conf.h"

#define HD44780_TIMER 		TIM7
#define HD44780_PRIORITY 	6

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
	u16 data;
	int8_t nibble;
} hd44780_task_t;

typedef struct {
	GPIO_TypeDef* gpio;
	u32 rs;
	u32 rw;
	u32 e;
	u32 db4;
	u32 db5;
	u32 db6;
	u32 db7;
	u8 lines;
	u8 font;
} hd44780_conf_t;

static hd44780_conf_t Lcd_Conf;
static volatile hd44780_task_t Queue[HD44780_QUEUE_SIZE];
static volatile u16 Queue_Head = 0;
static volatile u16 Queue_Tail = 0;

static void delay(u8 delay) {

	while (delay != 0)
		delay--;
}

static void set_output(const bool output) {

	GPIO_InitTypeDef GPIO_InitStruct;
	u32 pins;
	u8 dir;

	pins = Lcd_Conf.db4 | Lcd_Conf.db5 | Lcd_Conf.db6 | Lcd_Conf.db7;
	dir = GPIO_Mode_IN;

	if (output) {
		pins = pins | Lcd_Conf.rs | Lcd_Conf.rw | Lcd_Conf.e;
		dir = GPIO_Mode_OUT;
	}

	GPIO_InitStruct.GPIO_Pin = pins;
	GPIO_InitStruct.GPIO_Mode = dir;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;

	GPIO_Init(Lcd_Conf.gpio, &GPIO_InitStruct);
}

static void enable(const bool pulse) {

	if (pulse) {
		GPIO_ToggleBits(Lcd_Conf.gpio, Lcd_Conf.e);
		delay(150);
	}
	GPIO_ToggleBits(Lcd_Conf.gpio, Lcd_Conf.e);
	delay(150);
}

static void write(const u8 data, const bool reg) {

	set_output(true);

	GPIO_ResetBits(Lcd_Conf.gpio, Lcd_Conf.rw);
	GPIO_WriteBit(Lcd_Conf.gpio, Lcd_Conf.rs, !reg);
	GPIO_WriteBit(Lcd_Conf.gpio, Lcd_Conf.db7, (data & 0x8) >> 3);
	GPIO_WriteBit(Lcd_Conf.gpio, Lcd_Conf.db6, (data & 0x4) >> 2);
	GPIO_WriteBit(Lcd_Conf.gpio, Lcd_Conf.db5, (data & 0x2) >> 1);
	GPIO_WriteBit(Lcd_Conf.gpio, Lcd_Conf.db4, (data & 0x1));
	enable(true);
}

static bool read_busy(void) {

	u8 data;

	set_output(false);
	GPIO_ResetBits(Lcd_Conf.gpio, Lcd_Conf.rs | Lcd_Conf.db7);
	GPIO_SetBits(Lcd_Conf.gpio, Lcd_Conf.rw);
	enable(false);
	data = GPIO_ReadInputDataBit(Lcd_Conf.gpio, Lcd_Conf.db7);
	enable(false);
	enable(true);

	if (data == 1)
		return (true);

	return (false);

}

static void task_add(const hd44780_command_t command, const bool reg,
		const u16 data, u8 nibbles) {

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

	u16 command = HD44780_DISPLAY;

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
void hd44780_position(const u8 row, const u8 col) {

	u16 command = HD44780_DGRAM;
	const u8 offsets[] = { 0x00, 0x40, 0x14, 0x54 };

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
void hd44780_cgram(const u8 pos, const char udg[8]) {

	u8 i;
	u16 command = HD44780_CGRAM;

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

	u8 i = 0;

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

	u16 i;
	u16 size;
	u8 character;
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
void hd44780_init(GPIO_TypeDef* gpio, const u16 rs, const u16 rw, const u16 e,
		const u16 db4, const u16 db5, const u16 db6, const u16 db7,
		const hd44780_lines_t lines, const hd44780_font_t font) {

	u32 periph;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_InitStructure;

	assert_param(IS_GPIO_ALL_PERIPH(gpio));
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

	if (gpio == GPIOA )
		periph = RCC_AHB1Periph_GPIOA;
	else if (gpio == GPIOB )
		periph = RCC_AHB1Periph_GPIOB;
	else if (gpio == GPIOC )
		periph = RCC_AHB1Periph_GPIOC;
	else if (gpio == GPIOD )
		periph = RCC_AHB1Periph_GPIOD;
	else if (gpio == GPIOE )
		periph = RCC_AHB1Periph_GPIOE;
	else
		return;

	RCC_AHB1PeriphClockCmd(periph, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1 );
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = HD44780_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_InitStructure.TIM_Period = (62500 / HD44780_QUEUE_FREQ) - 1;
	TIM_InitStructure.TIM_Prescaler = ((SystemCoreClock / 2) / 62500) - 1;
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(HD44780_TIMER, &TIM_InitStructure);
	TIM_ITConfig(HD44780_TIMER, TIM_IT_Update, ENABLE);
	TIM_Cmd(HD44780_TIMER, ENABLE);

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
			HD44780_FUNCTION | HD44780_4BIT | Lcd_Conf.lines | Lcd_Conf.font,
			2);
	task_add(HD44780_WAIT, NONE, 1, NONE);
	task_add(HD44780_WRITE, true,
			HD44780_DISPLAY | HD44780_DISPLAY_ON | !HD44780_CURSOR_ON, 2);
	task_add(HD44780_WAIT, NONE, 1, NONE);
	task_add(HD44780_WRITE, true, HD44780_ENTRY | HDD44780_ENTRY_LEFT, 2);
	task_add(HD44780_WAIT_NOT_BUSY, NONE, NONE, NONE);
	hd44780_clear();
	hd44780_home();
}

void TIM7_IRQHandler(void) {

	exec();
	TIM_ClearITPendingBit(HD44780_TIMER, TIM_IT_Update );
}
