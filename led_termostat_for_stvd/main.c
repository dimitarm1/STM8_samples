/*
 * main.c
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


#include "stm8l.h"
#include "led.h"

/*	Authorize interrupts.
 */
//#define enableInterrupts() _asm("rim")

#define DIGIT_PER 1000
unsigned long Global_time = 0L; // global time in ms
int ADC_value = 0; // value of last ADC measurement
U8 LED_delay = 1; // one digit emitting time


@interrupt void HandledInterrupt (void)
{
	/* in order to detect unexpected events during development, 
	   it is recommended to set a breakpoint on the following instruction
	*/
	return;
}

@interrupt void NonHandledInterrupt (void)
{
	/* in order to detect unexpected events during development, 
	   it is recommended to set a breakpoint on the following instruction
	*/
	return;
}

@interrupt void TIM1_UPD_OVF_TRG_BRK_IRQHandler (void)
{
	if(TIM1_SR1 & TIM_SR1_UIF){ // update interrupt
		Global_time++; // increase timer
	}
	TIM1_SR1 = 0; // clear all interrupt flags
	return;
}


/*	Send a char to the SCI.
 */
void outch(char c)
	{
//	while (!(USART_SR & TRDE))	/* wait for READY */
	//	;
//	USART_DR = c;			/* send it */
	}

/*	Character reception routine.
 *	This routine is called on interrupt.
 *	It puts the received char in the buffer.
 */
@interrupt void recept(void)
	{
//	USART_SR;			/* clear interrupt */
//	*ptecr++ = USART_DR;		/* get the char */
//	if (ptecr >= &buffer[SIZE])	/* put it in buffer */
//		ptecr = buffer;
	}

int main() {
	unsigned long T_LED = 0L;  // time of last digit update
	unsigned long T_time = 0L; // timer
	int i = 00;
	Global_time = 0L; // global time in ms
	ADC_value = 0; // value of last ADC measurement
	LED_delay = 1; // one digit emitting time

	// Configure clocking
	CLK_CKDIVR = 0; // F_HSI = 16MHz, f_CPU = 16MHz
	// Configure pins
	CFG_GCR |= 1; // disable SWIM
	LED_init();
	// Configure Timer1
	// prescaler = f_{in}/f_{tim1} - 1
	// set Timer1 to 1MHz: 1/1 - 1 = 15
	TIM1_PSCRH = 0;
	TIM1_PSCRL = 15; // LSB should be written last as it updates prescaler
	// auto-reload each 1ms: TIM_ARR = 1000 = 0x03E8
	TIM1_ARRH = 0x03;
	TIM1_ARRL = 0xE8;
	// interrupts: update
	TIM1_IER = TIM_IER_UIE;
	// auto-reload + interrupt on overflow + enable
	TIM1_CR1 = TIM_CR1_APRE | TIM_CR1_URS | TIM_CR1_CEN;
	// configure ADC

	_asm("rim");    // enable interrupts
	
	display_int(i++);
	
	show_next_digit(); // show zero
	
	// Loop
	do {
		if(((unsigned int)(Global_time - T_time) > DIGIT_PER) || (T_time > Global_time)) // set next timer value
		{
			T_time = Global_time;
			i++;
			if((i % 100) > 59)
			{
				i += 40;
			}
			display_int(i);
			
			if(i > 9999) i = -1200;
			// check ADC value to light up DPs proportionaly
		
			// values less than 206 == 0
		}
		if((U8)(Global_time - T_LED) > LED_delay)
		{
			T_LED = Global_time;
			show_next_digit();
			
		}
	} while(1);
}

