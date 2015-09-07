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

#include <stm32f4xx.h>

#include "hd44780.h"

#define BKL_FREQ			250000
#define CON_FREQ			BKL_FREQ

#define HD44780_TIMER 		TIM7
#define HD44780_CLK_EN		__TIM7_CLK_ENABLE
#define HD44780_PRIORITY 	1
#define HD44780_IRQ 		TIM7_IRQn
#define HD44780_HANDLER		TIM7_IRQHandler

#define BKL_GPIO_EN			__GPIOB_CLK_ENABLE
#define BKL_GPIO			GPIOB
#define BKL_PIN				GPIO_PIN_15
#define BKL_AF 				GPIO_AF9_TIM12
#define BKL_TIMER 			TIM12
#define BKL_CHANNEL			TIM_CHANNEL_2
#define BKL_CLK_EN			__TIM12_CLK_ENABLE
#define BKL_PERIOD			(HAL_RCC_GetHCLKFreq() /2) / BKL_FREQ

#define CON_GPIO_EN			__GPIOB_CLK_ENABLE
#define CON_GPIO			GPIOB
#define CON_PIN				GPIO_PIN_9
#define CON_AF 				GPIO_AF3_TIM11
#define CON_TIMER 			TIM11
#define CON_CHANNEL			TIM_CHANNEL_1
#define CON_CLK_EN			__TIM11_CLK_ENABLE
#define CON_PERIOD			HAL_RCC_GetHCLKFreq() / CON_FREQ

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

#define SYSTICK_US			(SystemCoreClock / 1000000)

typedef enum {
	HD44780_WAIT = 1, HD44780_WAIT_NOT_BUSY, HD44780_WRITE
} hd44780_command_type;

typedef struct {
	hd44780_command_type command;
	bool reg;
	uint16_t data;
	uint8_t nibble;
} hd44780_task_type;

typedef struct {
	GPIO_TypeDef *gpio;
	uint16_t rs;
	uint16_t rw;
	uint16_t e;
	uint16_t db4;
	uint16_t db5;
	uint16_t db6;
	uint16_t db7;
	uint8_t lines;
	uint8_t font;
} hd44780_conf_type;

void HD44780_HANDLER(void);

static TIM_HandleTypeDef Handle_Lcd;
static TIM_HandleTypeDef Handle_Brigt;
static TIM_HandleTypeDef Handle_Con;

static hd44780_conf_type Lcd_Conf;
static volatile hd44780_task_type Queue[HD44780_QUEUE_SIZE];
static volatile uint16_t Queue_Head = 0;
static volatile uint16_t Queue_Tail = 0;

static void init_delay(void) {

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint8_t delay) {

	uint32_t ccr_start = DWT->CYCCNT;

	while (1) {
		uint32_t ccr = DWT->CYCCNT;
		long elapsed = ccr - ccr_start;

		if (elapsed < 0)
			elapsed = ccr_start + ccr;

		if ((elapsed / SYSTICK_US) >= delay)
			break;
	}

}

static void set_output(const bool output) {

	GPIO_InitTypeDef gpio_init;
	uint32_t pins;
	uint8_t dir;

	pins = Lcd_Conf.db4 | Lcd_Conf.db5 | Lcd_Conf.db6 | Lcd_Conf.db7;
	dir = GPIO_MODE_INPUT;

	if (output) {
		pins = pins | Lcd_Conf.rs | Lcd_Conf.rw | Lcd_Conf.e;
		dir = GPIO_MODE_OUTPUT_PP;
	}

	gpio_init.Pin = pins;
	gpio_init.Mode = dir;
	gpio_init.Pull = GPIO_NOPULL;
	gpio_init.Speed = GPIO_SPEED_MEDIUM;

	HAL_GPIO_Init(Lcd_Conf.gpio, &gpio_init);
}

static void enable(const bool pulse) {

	if (pulse) {
		HAL_GPIO_TogglePin(Lcd_Conf.gpio, Lcd_Conf.e);
		delay_us(50);
	}
	HAL_GPIO_TogglePin(Lcd_Conf.gpio, Lcd_Conf.e);
	delay_us(50);
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

static void task_add(const hd44780_command_type command, const bool reg,
		const uint16_t data, uint8_t nibbles) {

	if (command == HD44780_WAIT_NOT_BUSY)
		nibbles = 2;

	nibbles--;

	if (Queue_Head != (Queue_Tail + 1) % (HD44780_QUEUE_SIZE - 1)) {
		Queue[Queue_Tail].command = command;
		Queue[Queue_Tail].reg = reg;
		Queue[Queue_Tail].data = data;
		Queue[Queue_Tail].nibble = nibbles;
		Queue_Tail = (uint16_t) (Queue_Tail + 1) % (HD44780_QUEUE_SIZE - 1);
	} else
		assert_param(false);
}

static void task_del(void) {

	if (Queue_Head != Queue_Tail) {
		Queue_Head = (uint16_t) (Queue_Head + 1) % (HD44780_QUEUE_SIZE - 1);
	}
}

static void exec(void) {

	volatile hd44780_task_type *task;

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
				write((uint8_t) (task->data >> 4), task->reg);
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

	uint32_t bright;

	bright = (uint32_t) (100 - brightness);
	bright *= BKL_PERIOD;
	bright /= 100;
	__HAL_TIM_SetCompare(&Handle_Brigt, BKL_CHANNEL, bright);
}

/**
 * Set the contrast
 *
 * @param contrast	0-100
 *
 */
void hd44780_contrast(const uint8_t contrast) {

	uint32_t cont;

	cont = (uint32_t) (100 - contrast);
	cont *= BKL_PERIOD * 2;
	cont /= 100;
	__HAL_TIM_SetCompare(&Handle_Con, CON_CHANNEL, cont);
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

	command |= (uint16_t) (col + offsets[row]);
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

	command |= (uint16_t) (pos * 8);

	task_add(HD44780_WRITE, true, command, 2);
	task_add(HD44780_WAIT, true, 1, 1);
	for (i = 0; i < 8; i++) {
		task_add(HD44780_WRITE, false, (uint16_t) udg[i], 2);
		task_add(HD44780_WAIT_NOT_BUSY, NONE, NONE, NONE);
	}
}

/**
 * Send a single character to the LCD
 *
 * @param chr	Character to send
 */
void hd44780_put(const char chr) {

	task_add(HD44780_WRITE, false, (uint16_t) chr, 2);
	task_add(HD44780_WAIT_NOT_BUSY, NONE, NONE, NONE);
}

/**
 * Send a string to the LCD
 *
 * @param string	String to send
 */
void hd44780_print(const char *string) {

	uint8_t i = 0;

	while (string[i]) {
		task_add(HD44780_WRITE, false, (uint16_t) string[i], 2);
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

	int i;
	int size;
	char character;
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
	va_end(args);
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
void hd44780_init(GPIO_TypeDef *gpio, const uint16_t rs, const uint16_t rw,
		const uint16_t e, const uint16_t db4, const uint16_t db5,
		const uint16_t db6, const uint16_t db7, const hd44780_lines_type lines,
		const hd44780_font_type font) {

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

	init_delay();

	HD44780_CLK_EN();
	HAL_NVIC_SetPriority(HD44780_IRQ, 0, HD44780_PRIORITY);
	HAL_NVIC_EnableIRQ(HD44780_IRQ);

	Handle_Lcd.Instance = HD44780_TIMER;
	Handle_Lcd.Init.Period = (62500 / HD44780_QUEUE_FREQ) - 1;
	Handle_Lcd.Init.Prescaler = ((HAL_RCC_GetHCLKFreq() / 2) / 62500) - 1;
	Handle_Lcd.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Handle_Lcd.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_Base_Init(&Handle_Lcd);
	HAL_TIM_Base_Start_IT(&Handle_Lcd);

	task_add(HD44780_WAIT, true, 100, 1);
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

	GPIO_InitTypeDef gpio;
	TIM_OC_InitTypeDef oc;

	BKL_GPIO_EN();
	gpio.Pin = BKL_PIN;
	gpio.Mode = GPIO_MODE_AF_OD;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_MEDIUM;
	gpio.Alternate = BKL_AF;
	HAL_GPIO_Init(BKL_GPIO, &gpio);

	BKL_CLK_EN();
	Handle_Brigt.Instance = BKL_TIMER;
	Handle_Brigt.Init.Period = BKL_PERIOD;
	Handle_Brigt.Init.Prescaler = 1 - 1;
	Handle_Brigt.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Handle_Brigt.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(&Handle_Brigt);

	oc.OCMode = TIM_OCMODE_PWM1;
	oc.OCIdleState = TIM_OCIDLESTATE_SET;
	oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	oc.Pulse = 0;
	oc.OCPolarity = TIM_OCPOLARITY_HIGH;
	oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;

	HAL_TIM_PWM_ConfigChannel(&Handle_Brigt, &oc,
	BKL_CHANNEL);
	HAL_TIM_PWM_Start(&Handle_Brigt, BKL_CHANNEL);
}

/**
 * Initialise the contrast control
 *
 */
void hd44780_init_contrast(void) {

	GPIO_InitTypeDef gpio;
	TIM_OC_InitTypeDef oc;

	CON_GPIO_EN();
	gpio.Pin = CON_PIN;
	gpio.Mode = GPIO_MODE_AF_OD;
	gpio.Pull = GPIO_PULLDOWN;
	gpio.Speed = GPIO_SPEED_MEDIUM;
	gpio.Alternate = CON_AF;
	HAL_GPIO_Init(CON_GPIO, &gpio);

	CON_CLK_EN();
	Handle_Con.Instance = CON_TIMER;
	Handle_Con.Init.Period = CON_PERIOD;
	Handle_Con.Init.Prescaler = 1 - 1;
	Handle_Con.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Handle_Con.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_PWM_Init(&Handle_Con);

	oc.OCMode = TIM_OCMODE_PWM1;
	oc.OCIdleState = TIM_OCIDLESTATE_SET;
	oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	oc.Pulse = 0;
	oc.OCPolarity = TIM_OCPOLARITY_HIGH;
	oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;

	HAL_TIM_PWM_ConfigChannel(&Handle_Con, &oc,
	CON_CHANNEL);
	HAL_TIM_PWM_Start(&Handle_Con, CON_CHANNEL);
}

void HD44780_HANDLER(void) {

	exec();
	HAL_TIM_IRQHandler(&Handle_Lcd);
}
