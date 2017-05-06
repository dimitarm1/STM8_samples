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

#define STATUS_FREE    0
#define STATUS_WORKING 1
#define STATUS_COOLING 2
#define STATUS_WAITING 3

#define STATE_WAIT_COMMAND 0
#define STATE_WAIT_PRE_TIME 1
#define STATE_WAIT_MAIN_TIME 2
#define STATE_WAIT_COOL_TIME 3
#define STATE_WAIT_CHECKSUM 4
#define STATE_WAIT_VALIDATE_START 5

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

#define BUFFER_SIZE 5
unsigned char buffer[BUFFER_SIZE];
unsigned char* ptecr = buffer;
unsigned char* ptecr_prev = buffer;

unsigned char device = 255;
unsigned char status;
unsigned char result_1;
unsigned char result_2;
int pre_time = 0;
int main_time = 0;
int cool_time = 0;
unsigned char device_status = STATUS_FREE;
unsigned char prescaler =  60;
int receiver_timeout;
char receiver_state;

// Function prototypes


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

#define UART_PORT		PD
#define UART_TX_PIN		GPIO_PIN5
void uart_init(void){
	// PD5 - UART2_TX
	PD_DDR |= UART_TX_PIN;
	PD_CR1 |= UART_TX_PIN;	
	UART1_CR2 = 0;
// Configure UART
	// 8 bit, no parity, 1 stop (UART_CR1/3 = 0 - reset value)
	// 1200 on 16MHz: BRR1=0x41, BRR2=0x35
	UART1_BRR1 = 0xCD; UART1_BRR2 = 0x33; //taka stava!!
	UART1_CR2 = UART_CR2_TEN | UART_CR2_REN | UART_CR2_RIEN; // Allow RX/TX, 
	 // Allow RX/TX, generate ints on rx
	//UART1_CR1 = 0; // Enable UART?	 
}

/**
 * Send one byte through UART
 * @param byte - data to send
 */
void UART_send_byte(U8 byte){
	while(!(UART1_SR & UART_SR_TXE)); // wait until previous byte transmitted
	UART1_DR = byte;
}

/*	Character reception routine.
 *	This routine is called on interrupt.
 *	It puts the received char in the buffer.
 */
@interrupt void recept(void)
{
	char tmp;
	tmp = UART1_SR;			/* clear interrupt */
	*ptecr++ = tmp = UART1_DR;		/* get the char */
	if (ptecr >= &buffer[BUFFER_SIZE])	/* put it in buffer */
	{
		  ptecr = buffer;
	}
	UART_send_byte(tmp);
}

int ToBCD(int value)
{  
  int digits[6];
  int result;
  digits[0] = value %10;
  digits[1] = (value/10) % 10;
  digits[2] = (value/100) % 10;
	digits[3] = (value/1000)%10;
  digits[4] = (value/10000) % 10;
  digits[5] = (value/100000) % 10;
  result = digits[0] | (digits[1]<<4) | (digits[2]<<8);
  return result;
}

int FromBCD(int value){
  int digits[6];
  int result;
  digits[0] = value & 0x0F;
  digits[1] = (value>>4) & 0x0F;
  digits[2] = (value>>8) & 0x0F;
	digits[3] = (value>>12) & 0x0F;
  digits[4] = (value>>16) & 0x0F;
  digits[5] = (value>>20) & 0x0F;
  result = digits[0] + digits[1]*10 + digits[2]*100 + digits[3]*1000 + digits[4]*10000 + digits[5]*100000;
  return result;
}

void updateDeviceStatus(void)
{
	if(cool_time > 0)
	{
		if(pre_time > 0)
		{
				device_status = STATUS_WAITING;
		}
		else if(main_time > 0)
		{
				device_status = STATUS_WORKING;
		}
		else 
		{
				device_status = STATUS_COOLING;
		}
	}
  else
  {
      device_status = STATUS_FREE;
  }
  
  if( device_status == STATUS_WORKING)
  {
			PA_ODR |= (1<<2); // Relay is on
  }
  else
  {
      PA_ODR &= ~(1<<2); // Relay is off
  }
//  if( device_status == STATUS_WORKING || device_status == STATUS_COOLING)
//  {
//      digitalWrite(OUPTUT_OHLAJDANE_PIN,1);
//  }
//  else
//  {
//      digitalWrite(OUPTUT_OHLAJDANE_PIN,0);
//  }
//  Serial.print(F("\nNew device status:"));
//  Serial.print( device_status);
}


//----------------------------- MAIN --------------------------
int main() {
	unsigned long T_LED = 0L;  // time of last digit update
	unsigned long T_time = 0L; // timer	
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
	CLK_CKDIVR = 2<<3; // F_HSI = 16MHz, f_CPU = 16MHz
	// Configure pins
	CFG_GCR |= 1; // disable SWIM
	LED_init();
	uart_init();
	// Configure Timer1
	// prescaler = f_{in}/f_{tim1} - 1
	// set Timer1 to 1MHz: 1/1 - 1 = 15
	TIM1_PSCRH = 0;
	TIM1_PSCRL = 3; // LSB should be written last as it updates prescaler
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
		
	display_int(0);
	
	show_next_digit(); // show zero
	
	// Loop
	do 
	{
		U8 *test = (U8*)0x4010;
		U8 result;		
		if(((unsigned int)(Global_time - T_time) > DIGIT_PER) || (T_time > Global_time)) // set next timer value
		{
			// Each second -->
			T_time = Global_time;	
			switch(device_status){
				case STATUS_FREE:
					if(show_time_delay == 0) display_int(main_time);
				break;
				case STATUS_WAITING:
					if(pre_time) pre_time--;
					if(!pre_time)
				  {
						main_time++;
					}
					display_int(-pre_time);
				break;
				case STATUS_WORKING:
					if(main_time)	main_time--;
					display_int(main_time);
				break;
				case STATUS_COOLING:
					if(cool_time)	cool_time--;
					display_int(cool_time);
				break;
				default:
				break;
			}
	
			//if(i > 9999) i = -1200;
			// check ADC value to light up DPs proportionaly
		
			// values less than 206 == 0
		}
		if((U8)(Global_time - T_LED) > LED_delay)
		{
			// Each ~3 mSec -->
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
					display_int(main_time); // Stop show time
				}				
			}
			
			result = scan_keys();		
			
			if(result & KEY_0_PRESSED) // Start
			{
				if(device_status == STATUS_WAITING && pre_time < 9*60) // - wait 3 sec from 1-st start press
				{
					BEEP_CSR = 0xbe;
					beep_delay = 200;
					counter_enabled = 2;
					add_minutes_to_eeprom(main_time/60);
					main_time++;
					pre_time = 0;
				}
				if(device_status == STATUS_FREE)
				{
					BEEP_CSR = 0xbe;
					beep_delay = 10;
					if(show_time_delay == 0 && main_time>0)
					{						
						if(pre_time > 0)
						{
							cool_time = 3*60; // Indicate manual start using Cool Time							
						}
						else
						{
							pre_time = 9*60; // Initial wait itme for manual operation
							device_status == STATUS_WAITING;
						}
					}
					else
					{						
						// Show Time
						int i = work_hours->minutes + (int)(work_hours->hours_L) * 60 + (int)(work_hours->hours_H) * 60*60;
						display_int(i);
						show_time_delay = 6000;
					}
				}
			}
			else 
			{			
				if(result && show_time_delay)
				{
					main_time = 0;
					show_time_delay = 0;
				}			
				if(device_status == STATUS_FREE)
				{
					if(result & KEY_3_PRESSED)// plus
					{
						if(main_time < 60*30)	main_time+=60;
						display_int(main_time);
						BEEP_CSR = 0xbe;
						beep_delay = 10;
						pre_time = 9*60;
					}
					if(result & KEY_2_PRESSED)// minus
					{
						if(main_time >= 60)
						{
							main_time -=60;
							display_int(main_time);
						}
						BEEP_CSR = 0xbe;
						beep_delay = 10;
					}
				}
			}
			
			if(result & KEY_1_PRESSED) //Stop
			{				
				main_time = pre_time = cool_time = 0;
				if(device_status != STATUS_FREE)
				{
					cool_time = 3*60;
					display_int(cool_time);
				}
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
		}	
		updateDeviceStatus();
	} while(1);
}

