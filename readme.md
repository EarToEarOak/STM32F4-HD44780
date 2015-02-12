# STM32F4-HD44780 #

Interrupt driven software for the ST Microelectronics STMF4 Cortex-M4 microcontroller to interface with a Hitachi-compatible HD44780 LCD module.  Works in 4bit mode using 7 pins and is 5v compatible, tested on a STM32F4 Discovery with a variety of LCD modules.

Full details can be found [here](http://eartoearoak.com/software/stm32f4-hd44780-lcd-driver)

Copyright 2013 - 2015 Al Brown

al [at] eartoearoak.com

##Usage##

Complete documentation can be found here.

A brief example is included in the source code.

Initialise the driver using:

`hd44780_init(gpio, rs, rw, e, db4, db5, db6, db7, lines, font)`

where:

`gpio` - GPIO Port (GPIOA to GPIOE)

`rs, rw, e, db4, db5, db6, db7` - Pins (`GPIO_Pin_0` to `GPIO_Pin_15`)

`lines` - LCD lines (`HD44780_LINES_1` or `HD44780_LINES_2`)

`font`- Font size (`HD44780_FONT_5x8` or `HD44780_FONT_5x10`)


## License ##

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
