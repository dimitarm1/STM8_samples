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

#define KEY_0_PRESSED  0x01
#define KEY_1_PRESSED  0x02
#define KEY_2_PRESSED  0x04
#define KEY_3_PRESSED  0x08
#define KEY_PRESSED    0x0F

#define KEY_0_RELEASED 0x10
#define KEY_1_RELEASED 0x20
#define KEY_2_RELEASED 0x40
#define KEY_3_RELEASED 0x80
#define KEY_RELEASED   0xF0

unsigned long Global_time = 0L; // global time in ms
int ADC_value = 0; // value of last ADC measurement
U8 LED_delay = 1; // one digit emitting time
U16 keys_scan_buffer[4];
U8 key_state;
typedef struct settings_t{
	U8 pre_time;
	U8 cool_time;
}settings_t;
typedef struct {
	U8 minutes;
	U8 hours_L;
	U8 hours_H;
}work_hours_t;
work_hours_t *work_hours;


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

/*	Scan pushbuttons routine.
 *	This routine is called in the main loop.
 *	It scans for key states, sets the new state 
 *  and returns a mask of buttons with changed state.
 *  Newly pressed in the bits 0-3, newly released in bits 4-7
 */
U8 scan_keys(void)
{
	U8 i, result = 0;
	keys_scan_buffer[0] = (keys_scan_buffer[0]<<1) | (!(PB_IDR & (1<<4))); // Up (start)
	keys_scan_buffer[1] = (keys_scan_buffer[1]<<1) | (!(PD_IDR & (1<<6))); // Dn (start)
	keys_scan_buffer[2] = (keys_scan_buffer[2]<<1) | (!(PA_IDR & (1<<1))); // Up (stop)
	keys_scan_buffer[3] = (keys_scan_buffer[3]<<1) | (!(PA_IDR & (1<<3))); // Dn (stop)
	for(i = 0; i<4; i++)
	{
		if(keys_scan_buffer[i] == 0xffff)
		{
			if(!(key_state & (1<<i)))
			{
				key_state = key_state | (1<<i);
				result = result | (0x01<<i);
			}
		}
		if(keys_scan_buffer[i] == 0)
		{
			if((key_state & (1<<i)))
			{
				key_state = key_state & (~((U8)(0x11<<i)));
				result = result | (0x10<<i);
			}
		}
		
	}
	return result;
}

/*
 * add_minutes_to_eeprom(U8 minutes)
 * This function updates worked hours stored in EEPROM 
 *
*/
void add_minutes_to_eeprom(U8 minutes)
{
	U8 tmp = 0;
	
	if(minutes > 59)
	{
		tmp = minutes/60;
		minutes = minutes % 60;
	}
	minutes = work_hours->minutes + minutes;
  if(minutes > 59)
	{
		minutes = minutes - 60;
		tmp+=1;
	}
	//
  //  Check if the EEPROM is write-protected.  If it is then unlock the EEPROM.
  //
	if ((FLASH_IAPSR & FLASH_IAPSR_DUL) == 0)
	{
		FLASH_DUKR = EEPROM_KEY1;
		FLASH_DUKR = EEPROM_KEY2;
	}
	//
	//  Write the data to the EEPROM.
	//
	work_hours->minutes = minutes;
	tmp = tmp + work_hours->hours_L;
	if (tmp > 99)
	{
		tmp -= 100;
		work_hours->hours_H = work_hours->hours_H + 1;
	}
	work_hours->hours_L = tmp;
	//
  //  Now write protect the EEPROM.
	//
	FLASH_IAPSR = FLASH_IAPSR & ~FLASH_IAPSR_DUL;
}

/*
 * void clear_eeprom()
 * clear the eeprom used 
 *
*/
void clear_eeprom(void)
{	
	//
  //  Check if the EEPROM is write-protected.  If it is then unlock the EEPROM.
  //
	if ((FLASH_IAPSR & FLASH_IAPSR_DUL) == 0)
	{
		FLASH_DUKR = EEPROM_KEY1;
		FLASH_DUKR = EEPROM_KEY2;
	}
	//
	//  Write the data to the EEPROM.
	//
	work_hours->minutes = 0;
	work_hours->hours_L = 0;
	work_hours->hours_H = 0;
	
	//
  //  Now write protect the EEPROM.
	//
	FLASH_IAPSR = FLASH_IAPSR & ~FLASH_IAPSR_DUL;
}

int main() {
	unsigned long T_LED = 0L;  // time of last digit update
	unsigned long T_time = 0L; // timer
	int i = 00, j = 00;
	U8 beep_delay = 0;
	int show_time_delay = 0;
	U8 counter_enabled = 0;
	key_state = 0;
	work_hours = (work_hours_t*)EEPROM_START_ADDR;	

	keys_scan_buffer[0] = keys_scan_buffer[1] = keys_scan_buffer[2] = keys_scan_buffer[3] = 0;
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
  BEEP_CSR = 0x1e; // Configure BEEP

	_asm("rim");    // enable interrupts
		
	display_int(i);
	
	show_next_digit(); // show zero
	
	// Loop
	do 
	{
		U8 *test = (U8*)0x4010;
		U8 result;		
		if(((unsigned int)(Global_time - T_time) > DIGIT_PER) || (T_time > Global_time)) // set next timer value
		{
			T_time = Global_time;
			if(i && counter_enabled == 2)
			{
				PA_ODR |= (1<<2); // Relay is on
				i--;
				if(!i)
				{
					counter_enabled = 0;					
				}
			}
			else
			{
				PA_ODR &= ~(1<<2); // Relay is off
			}
			if((i % 100) > 59)
			{
				i -= 40;
			}
			if(counter_enabled == 1)
			{
				if(j)
				{
					j--;
					if(!j)
				  {
						i++;
					}
					if((j % 100) > 59)
					{
						j -= 40;
					}
				}
				if(!j)
				{
					counter_enabled = 2;
					add_minutes_to_eeprom(i/100);
				}
		  }
			if(counter_enabled == 1)
			{
				display_int(-j);
			}
			else
			{
				display_int(i);
			}
			
			//if(i > 9999) i = -1200;
			// check ADC value to light up DPs proportionaly
		
			// values less than 206 == 0
		}
		if((U8)(Global_time - T_LED) > LED_delay)
		{
			T_LED = Global_time;
			show_next_digit();	
			if(beep_delay)
			{
				beep_delay--;
				if(!beep_delay)
				{
					BEEP_CSR = 0x1e; // Configure BEEP
				}
			}
			if(show_time_delay)
			{
			  show_time_delay--;
				if(!show_time_delay)
				{
					i = 0; // Stop show time
				}
				
			}
		}
		result = scan_keys();		
		
		if(result & KEY_0_PRESSED) // Start
		{
			if(counter_enabled == 1 && j < 859) // 859 - wait 3 sec from 1-st start press
			{
				BEEP_CSR = 0xbe;
				beep_delay = 200;
				counter_enabled = 2;
				add_minutes_to_eeprom(i/100);
				i++;
				j = 0;
			}
			if(!counter_enabled)
			{
				BEEP_CSR = 0xbe;
				beep_delay = 10;
				if(show_time_delay == 0 && i>0)
				{
					counter_enabled = 1;
					j = 901; // 6 minutes pre time 
				}
				else
				{
					// Show Time
					i = work_hours->minutes + (int)(work_hours->hours_L) * 100 + (int)(work_hours->hours_H) * 10000;
					show_time_delay = 2450;
				}
		  }
		}
		else 
		{			
			if(result && show_time_delay)
			{
				i = 0;
				show_time_delay = 0;
			}			
			if(!counter_enabled)
			{
				if(result & KEY_3_PRESSED)
				{
					i+=100;
					display_int(i);
					BEEP_CSR = 0xbe;
					beep_delay = 10;
				}
				if(result & KEY_2_PRESSED)
				{
					if(i >= 100)
					{
						i-=100;
						display_int(i);
					}
					BEEP_CSR = 0xbe;
					beep_delay = 10;
				}
			}
		}
		
		if(result & KEY_1_PRESSED) //Stop
		{
			counter_enabled = 0;
			j = i = 0;
			display_int(i);
			BEEP_CSR = 0xbe;
			beep_delay = 40;
			show_time_delay = 0;			
		}
		if((result & KEY_PRESSED) == KEY_PRESSED && Global_time < 1000)
		{
			BEEP_CSR = 0xbe;
			beep_delay = 40;		
			clear_eeprom();
		}
		
		
	} while(1);
}

