/*
 * led.h
 *
 * Copyright 2014 Edward V. Emelianoff <eddy@sao.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */
#pragma once
#ifndef __LED_H__
#define __LED_H__
#include "stm8l.h"

void set_display_buf(char *str);
void show_buf_digit(U8 N);
void show_next_digit(void);
void display_int(int i);
void display_DP_at_pos(U8 i);

/**
 * Initialize ports
 * PA2, PB5, PC3|4|5|6|7, PD1|2|3
 *
#define LED_init()	do{ \
		PA_DDR = 0x08; PB_DDR = 0x30; PC_DDR = 0xf8; PD_DDR = 0x1e; \
		PA_CR1 = 0x08; PB_CR1 = 0x30; PC_CR1 = 0xf8; PD_CR1 = 0x1e; \
	}while(0)
*/
/*
PA_DDR = 0x04;\
		PB_DDR = 0x20;\
		PC_DDR = 0xf8;\
		PD_DDR = 0x07;\*/

// PA1|3, PB4|5, PC3|4|5|6|7, PD1|4|6
#define LED_init()	do{ \
		PA_DDR = 0xff;\
		PB_DDR = 0x20;\
		PC_DDR = 0xf8;\
		PD_DDR = 0x0f;\
		PA_CR1 = 0xff;\
		PB_CR1 = 0xff;\
		PC_CR1 = 0xff;\
		PD_CR1 = 0xff; \
	}while(0)

#endif // __LED_H__
