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

#define    ResetIWDG    (IWDG_KR=0xAA)
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


U8 second_elapsed = 0;
volatile unsigned long Global_time = 0L; // global time in ms
volatile unsigned Local_time = 0L; // local time in ms
int ADC_value = 0; // value of last ADC measurement
U8 LED_delay = 1; // one digit emitting time
U16 keys_scan_buffer[4];
U8 key_state;
typedef struct settings_t{
	U8 pre_time;
	U8 cool_time;
	U8 address;
}settings_t;
typedef struct {
	U8 minutes;
	U8 hours_L;
	U8 hours_H;
}work_hours_t;
work_hours_t *work_hours;
tmp_test_buff[4];
settings_t *settings;
#ifdef DISTANCIONNO_SERIAL
	#warning "PC Type distancionno compilation!!!"
#endif

volatile unsigned char device = 255;
volatile unsigned char status;
volatile unsigned char result_1;
volatile unsigned char result_2;
volatile unsigned char pre_time_serial = 0;
volatile unsigned char main_time_serial = 0;
volatile unsigned char cool_time_serial = 0;
volatile int pre_time = 0;
volatile int main_time = 0;
volatile int cool_time = 0;
volatile int curr_status = -1;
volatile int curr_time = 0;
volatile unsigned char device_status = STATUS_FREE;
volatile unsigned char prescaler =  60;
volatile int receiver_timeout;
volatile char receiver_state;
char data;
char checksum;
char time_in_hex;
char data_received;

// Function prototypes
int ToBCD(int value);
int FromBCD(int value);
void updateDeviceStatus(void);
void increment_address_in_EEPROM(void);
void ping_status(void);
void send_start(void);
void send_time(void);


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
		Local_time++;
		if(Local_time >= DIGIT_PER)
		{
			Local_time = 0;
			second_elapsed = 1;
		}		
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

void increment_address_in_EEPROM(void)
{
	U8 tmp = settings->address & 0x0f;
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
	settings->address = (tmp+1)&0x0f;
	//
  //  Now write protect the EEPROM.
	//
	FLASH_IAPSR = FLASH_IAPSR & ~FLASH_IAPSR_DUL;
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
	
	data = UART1_SR;			/* clear interrupt */
	if(!(data & UART_SR_RXNE)) return;
	data = UART1_DR;		/* get the char */
#ifdef DISTANCIONNO_SERIAL
	data_received = 1;

#else
  if(data & 0x80)  // Command received
    {
      device = (data & 0x78)>>3;     
      receiver_timeout = 40;       
      /*
 * commads:
       * 0 - status 0-free, 1-Working, 2-COOLING, 3-WAITING
       * 1 - start
       * 2 - set pre-time
       * 3 - set cool-time
       * 4 - stop - may be not implemented in some controllers
       * 5 - set main time
       */
		if (device == settings->address)
      switch(data & 0x07)
      {
      case 0: // ststus
        data = device_status<<6;
        switch (device_status)
        {
          case STATUS_FREE:
            break;
          case STATUS_WORKING:
            data = data | ToBCD(!!(main_time%60) + main_time/60);
            break;
          case STATUS_COOLING:
            data = data | ToBCD(!!(cool_time%60) + cool_time/60);
            break;
          case STATUS_WAITING:
            data = data | ToBCD(!!(pre_time%60) + pre_time/60);
            break;
          default:
            break;
        }
        UART_send_byte(data);
        receiver_state = STATE_WAIT_COMMAND;
        break;
      case 1: // start
        receiver_state = STATE_WAIT_VALIDATE_START;                        
        break;
      case 2: // set pre_time
        receiver_state = STATE_WAIT_PRE_TIME;                      
        break;
      case 3: // set cool_time
        receiver_state = STATE_WAIT_COOL_TIME;                        
        break;
      case 4: // stop
        receiver_state = STATE_WAIT_COMMAND;
        if(pre_time == 0 && main_time > 0)
        {
          main_time = 0;                          
        }
        else
        {
          cool_time = 0;
          main_time = 0;
          pre_time = 0;                            
        }
        if(device_status > 0)
        {
          //updateDeviceStatus();
        }
        break;
      case 5: // set main time
        receiver_state = STATE_WAIT_MAIN_TIME;            
        break;
      default:
        break;
      }
    }
    else // Data recevied ?
    if(receiver_state != STATE_WAIT_COMMAND)
    {
      switch(receiver_state)
      {
        case STATE_WAIT_PRE_TIME:          
          if(data>9)data = 9;
          pre_time_serial = (data);          
          receiver_state = STATE_WAIT_COMMAND;
          break;
        case STATE_WAIT_MAIN_TIME:                                        
          main_time_serial = FromBCD(data);       
          UART_send_byte(main_time_serial);
          receiver_state = STATE_WAIT_COMMAND;
          break;
        case STATE_WAIT_COOL_TIME:       
          if(data>9) data = 9;
          cool_time_serial = (data);
          time_in_hex = ToBCD(main_time_serial);
          checksum = (pre_time_serial + cool_time_serial - time_in_hex - 5) & 0x7F;
					UART_send_byte(checksum);
          receiver_state = STATE_WAIT_CHECKSUM;                                                          
          break;
        case STATE_WAIT_CHECKSUM:   
          time_in_hex = ToBCD(main_time_serial);
          checksum = (pre_time_serial + cool_time_serial - time_in_hex - 5) & 0x7F;     
          if((char)data == (char)checksum)
          {
            if(main_time_serial > 60) main_time_serial = 60;            
						main_time = main_time_serial*60;
						pre_time = pre_time_serial*60;
						cool_time = cool_time_serial*60;
            //updateDeviceStatus();
          }
          receiver_state = STATE_WAIT_COMMAND;
          break;
        case STATE_WAIT_VALIDATE_START:        
          if(data == 0x55) //validate start
          {
            pre_time = 0;
            if(device_status > 0)
            {                           
              //updateDeviceStatus();
            }                                
          }
          receiver_state = STATE_WAIT_COMMAND;                                    
          break;
        default:
          break;
      }
    }	
#endif
}


unsigned char USART_Receive(int *timeout)
{
	data_received = 0;
	while(*timeout && !data_received)
	{
		*timeout--;
	}
	return data;
}
int ToBCD(int value)
{  
  int digits[3];
  int result;
  digits[0] = value %10;
  digits[1] = (value/10) % 10;
  digits[2] = (value/100) % 10;	
  result = digits[0] | (digits[1]<<4) | (digits[2]<<8);
  return result;
}

int FromBCD(int value){
  int digits[3];
  int result;
  digits[0] = value & 0x0F;
  digits[1] = (value>>4) & 0x0F;
  digits[2] = (value>>8) & 0x0F;
  result = digits[0] + digits[1]*10 + digits[2]*100;
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
				PB_ODR &= ~(1<<5); // Relay is on
		}
	}
  else
  {
      device_status = STATUS_FREE;			
			PB_ODR |= (1<<5); // Relay is off
  }
  
  if( device_status == STATUS_WORKING)
  {
			PA_ODR |= (1<<2); // Relay is on
			PB_ODR &= ~(1<<5); // Relay is on			
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

void InitIWDG(void)
{
  IWDG_KR  = 0x55;  // Erisim ac?l?yor. 
  IWDG_PR  = 0x06;  // Onbolucu degeri 256 olarak ayarlan?yor.
  IWDG_RLR = 0xFF;  // Say?c? 0xFF ten geriye dogru sayacak.
  IWDG_KR  = 0x00;  // Erisim kapat?l?yor.
  IWDG_KR  = 0xCC;  // Say?c? saymaya basl?yor.
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

	settings = (settings_t*)(EEPROM_START_ADDR + sizeof(work_hours_t));

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
	InitIWDG();
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
		
	display_int_sec(0);
	
	show_next_digit(); // show zero
	pre_time = main_time = cool_time = 0;
	updateDeviceStatus();
		
	// Loop
	do 
	{
		U8 *test = (U8*)0x4010;
		U8 result;		
		ResetIWDG;
		if(second_elapsed) // set next timer value
		{
			// Each second -->		
			second_elapsed = 0;
#ifdef werwer
			if(show_time_delay == 0)
				display_int_sec(curr_time/100);
#else
			switch(device_status){
				case STATUS_FREE:
					if(show_time_delay == 0) display_int_sec(main_time);
				break;
				case STATUS_WAITING:
					if(pre_time) 
					{
						pre_time--;
						if(!pre_time)
						{
							BEEP_CSR = 0xbe;
							beep_delay = 300;
							counter_enabled = 2;
							add_minutes_to_eeprom(main_time/60);						
							pre_time = 0;
							main_time++;
						}
					}
					display_int_sec(-pre_time);
				break;
				case STATUS_WORKING:
					if(main_time)	main_time--;
					display_int_sec(main_time);
				break;
				case STATUS_COOLING:
					if(cool_time)	cool_time--;
					display_int_sec(cool_time);
				break;
				default:
				break;
			}
#endif
			LED_init(); // EMC or just Paranoya...
#ifdef DISTANCIONNO_SERIAL
			ping_status();
#else
			updateDeviceStatus();
#endif
			//if(i > 9999) i = -1200;
			// check ADC value to light up DPs proportionaly
		
			// values less than 206 == 0
			
		}
		if((U8)(Global_time - T_LED) > LED_delay)
		{
			if(receiver_state)
			{
				if(receiver_timeout)
				{
					receiver_timeout--;
				}
				else
				{
					receiver_state = STATE_WAIT_COMMAND;
				}
			}
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
					display_int_sec(main_time); // Stop show time
				}				
			}
			
			result = scan_keys();		
			
			if(result & KEY_0_PRESSED) // Start
			{
				if(device_status == STATUS_WAITING && pre_time < settings->address*60) // - wait 3 sec from 1-st start press
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
							pre_time = settings->address*60; // Initial wait itme for manual operation
							device_status == STATUS_WAITING;
						}
					}
					else
					{						
						// Show Time
						//int i = work_hours->minutes + (int)(work_hours->hours_L) * 60 + (int)(work_hours->hours_H) * 60*60;
						S32 i = (S32)work_hours->minutes + 
						   (S32)(work_hours->hours_L) * 100 +  
							 (S32)(work_hours->hours_H) * 10000;
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
						display_int_sec(main_time);
						BEEP_CSR = 0xbe;
						beep_delay = 10;
						pre_time = settings->address*60;
					}
					if(result & KEY_2_PRESSED)// minus
					{
						if(main_time >= 60)
						{
							main_time -=60;
							display_int_sec(main_time);
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
					display_int_sec(cool_time);
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
			if((result & KEY_PRESSED) == KEY_2_PRESSED && Global_time < 1000)
			{
				increment_address_in_EEPROM();
				display_int_sec(settings->address);
			}
		}	
		
	} while(1);
}


#ifdef DISTANCIONNO_SERIAL
void ping_status(void){

	static int ping_index = 0;
	volatile static int status_codes[4];
	// Ping solarium for status
	{
		int sts;
		ping_index = (ping_index + 1) & 0x03;
		status_codes[ping_index] = get_controller_status(settings->address);
		if(status_codes[ping_index]==status_codes[0] && status_codes[ping_index]==status_codes[1] &&
				status_codes[ping_index]==status_codes[2] && status_codes[ping_index]==status_codes[3] ){
			sts = status_codes[ping_index];
		}
		else {
			return;
		}

		if(sts != -1){
			curr_status = (sts & 0xC0)>>6;
			curr_time = sts & 0x3F;
		} else {
			curr_status = -1;
			curr_time = 0;
		}

		curr_time = FromBCD(curr_time);
		if(curr_status != -1)
		{
			device_status = curr_status;

		}
	}
}

void send_time(void){
	// Ping solarium for status
	if(settings->address >15){
		curr_status = -1;
		curr_time = 0;
	} else {
		int retry = 0;
		int timeout2 = 10000;
		char data;
		int time_in_hex = ToBCD(main_time);
		pre_time = pre_time & 0x7F;
		// checksum: CoolTime + Pre-Time - 5 - MainTime

		while(retry < 20){
			// clear in FIFO
			volatile unsigned char checksum = (pre_time + cool_time  - time_in_hex - 5) & 0x7F;
			volatile unsigned char  remote_check_sum = 220;
//			while(USART_GetFlagStatus(USART1,USART_FLAG_RXNE))	USART_ReceiveData(USART1); // Flush input
//			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // wAIT UNTIL TX BUFFER IS EMPTY

			USART_SendData(0x80U | ((settings->address & 0x0fU)<<3U) | 2U); //Command 2 == Pre_time_set
			SystickDelay(2);
			USART_SendData(pre_time);

			SystickDelay(2);
			USART_SendData(0x80U | ((settings->address & 0x0fU)<<3U) | 5U); //Command 5 == Main time set

			SystickDelay(2);
			USART_SendData(time_in_hex);

//			SystickDelay(2);


//			USART_ReceiveData(USART1); //"Read" old time

			SystickDelay(2);
			USART_SendData(0x80U | ((settings->address & 0x0fU)<<3U) | 3U); //Command 3 == Cool Time set

			SystickDelay(2);
			USART_SendData(cool_time);

			SystickDelay(4);
			timeout2 = 10000;

			remote_check_sum = USART_ReceiveData(&timeout2); //"Read" checksum

			SystickDelay(20);
//			checksum = remote_check_sum;
			if(remote_check_sum == checksum){
				SystickDelay(2);
				USART_SendData(checksum);
			}
			SystickDelay(100);
			USART_SendData(0x80U | ((settings->address & 0x0f)<<3U) | 0); //Command 0 - Get status
			SystickDelay(20);
			timeout2 = 10000;
			data = USART_ReceiveData(&timeout2); //"Read" status
			if ((data >> 6 ) != 0 ) retry = 22;
			retry ++;
		}

		/**************** OLD Pascal code as example
		CheckSum:=PreTime+CoolTime-DataSent-5;
		    CheckSum:=CheckSum mod 128 ;
		    PurgeComm(hDevice,(PURGE_TXCLEAR or PURGE_RXCLEAR	));
		    while retry<20 do
		     begin
		      for i:=1 to 10 do IOResult:=ReadFile(hDevice,IOByte,1,IOCount,NIL);
		      Data1:=128+Chanel*8+2;
		      IOResult:=WriteFile(hDevice,Data1,1,IOCount,NIL);
		      sleep(2);             // Set PRE-Time
		      IOResult:=WriteFile(hDevice,PreTime,1,IOCount,NIL);
		      Data1:=128+Chanel*8+5;
		      sleep(2);              // Set main time
		      IOResult:=WriteFile(hDevice,Data1,1,IOCount,NIL);
		      sleep(2);
		      IOResult:=WriteFile(hDevice,DataSent,1,IOCount,NIL);
		      sleep(2);
		      IOResult:=ReadFile(hDevice,IOByte,1,IOCount,NIL);   // get old main time
		      Data1:=128+Chanel*8+3; //  Set cool time
		      IOResult:=WriteFile(hDevice,Data1,1,IOCount,NIL);
		      sleep(2);
		      IOResult:=WriteFile(hDevice,CoolTime,1,IOCount,NIL);
		      sleep(4);
		      IOResult:=ReadFile(hDevice,IOByte,1,IOCount,NIL); // Get checksum?
		      if IOResult and (IOByte=CheckSum) then
		      IOResult:=WriteFile(hDevice,CheckSum,1,IOCount,NIL);
		      sleep(100);
		      Data1:=128+Chanel*8; // Get status command for selected chanel
		      IOResult:=WriteFile(hDevice,Data1,1,IOCount,NIL);
		      sleep(5);
		      IOResult:=ReadFile(hDevice,IOByte,1,IOCount,NIL);
		      IOByte:= IOByte div 64;
		      if IOResult and (IOByte <>0) and
		        ((DataSent=0) or (PreTime > 0) or ((IOByte = 1) and (DataSent >0)))    then
		        begin
		          retry:=22;
		        end
		      else retry:= retry+1;
		      sleep(1);
		      MainForm.Gauge2.Progress:=retry;
		     end;
		     IOResult:=ReadFile(hDevice,IOByte,1,IOCount,NIL);
		     IOResult:=ReadFile(hDevice,IOByte,1,IOCount,NIL);

		*/
	}
}

void send_start(void){
	// Ping solarium for status
	if(settings->address >15){
		curr_status = -1;
		curr_time = 0;
	} else {
//			while(USART_GetFlagStatus(USART1,USART_FLAG_RXNE))	USART_ReceiveData(USART1); // Flush input
//			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // wAIT UNTIL TX BUFFER IS EMPTY

			USART_SendData(0x80 | ((settings->address & 0x0f)<<3) | 1); //Command 1 == start
			SystickDelay(2);
			USART_SendData(0x55);
	}
}
#endif