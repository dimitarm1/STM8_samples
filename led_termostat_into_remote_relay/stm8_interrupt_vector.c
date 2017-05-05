/*	INTERRUPT VECTORS TABLE FOR STM8
 *	Copyright (c) 2007 by COSMIC Software
 */
/*	should be the name of a dummy interrupt routine
 *	in order to catch properly unexpected interrupts
 */
 
#include "stm8l.h" 
//#define NULL 0

extern void _stext();		/* startup routine */
extern void recept();		/* character receive handler */

extern void NonHandledInterrupt (void);

extern void HandledInterrupt (void);
void TIM1_UPD_OVF_TRG_BRK_IRQHandler (void);


typedef void @far (*interrupt_handler_t)(void);

struct interrupt_vector {
	unsigned char interrupt_instruction;
	interrupt_handler_t interrupt_handler;
};
//void (* const _vectab[32])()
struct interrupt_vector const _vectab[] = {

	{0x82, (interrupt_handler_t)_stext}, /* reset */

	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* trap  */
		// Top Level Interrupt
  // INTERRUPT_HANDLER(TLI_IRQHandler, 0){}
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq0  */
	// Auto Wake Up Interrupt
  // INTERRUPT_HANDLER(AWU_IRQHandler, 1){}
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq1  */
  // Clock Controller Interrupt
  // INTERRUPT_HANDLER(CLK_IRQHandler, 2){}
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq2  */
	// External Interrupt PORTA
  // INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3){}
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq3  */
	// External Interrupt PORTB
  // INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4){}
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq4  */
	// External Interrupt PORTC
  // INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5){}
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq5  */
	// External Interrupt PORTD
  // INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 6){}
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq6  */
	// External Interrupt PORTE
  // INTERRUPT_HANDLER(EXTI_PORTE_IRQHandler, 7){}
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq7  */
  // reserved
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq8  */
	// reserved
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq9  */
	// SPI Interrupt routine.
  // INTERRUPT_HANDLER(SPI_IRQHandler, 10){}
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq10 */
	// Timer1 Update/Overflow/Trigger/Break Interrupt
  // INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11){
	{0x82, (interrupt_handler_t)TIM1_UPD_OVF_TRG_BRK_IRQHandler}, /* irq11 */
	// Timer1 Capture/Compare Interrupt routine.
  // INTERRUPT_HANDLER(TIM1_CAP_COM_IRQHandler, 12){}
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq12 */
	// reserved
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq13 */
	// reserved
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq14 */
	// reserved
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq15 */
  // reserved
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq16 */
	// UART1 TX Interrupt
  // INTERRUPT_HANDLER(UART1_TX_IRQHandler, 17){}
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq17 */
	// UART1 RX Interrupt
  // INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18){}
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq18 */
	// I2C Interrupt
  // INTERRUPT_HANDLER(I2C_IRQHandler, 19){}
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq19 */
	// reserved
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq20 */
	// reserved
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq21 */
	// ADC1 interrupt
  // INTERRUPT_HANDLER(ADC1_IRQHandler, 22){}
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq22 */
	// reserved
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq23 */
	// Eeprom EEC Interrupt
  // INTERRUPT_HANDLER(EEPROM_EEC_IRQHandler, 24){}
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq24 */
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq25 */
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq26 */
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq27 */
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq28 */
	{0x82, (interrupt_handler_t)NonHandledInterrupt}, /* irq29 */
};
#ifdef test


void (* const _vectab[32])() = {
	_stext,			/* RESET       */
	NonHandledInterrupt,			/* TRAP        */
	NonHandledInterrupt,			/* TLI         */
	NonHandledInterrupt,			/* AWU         */
	NonHandledInterrupt,			/* CLK         */
	NonHandledInterrupt,			/* EXTI PORTA  */
	NonHandledInterrupt,			/* EXTI PORTB  */
	NonHandledInterrupt,			/* EXTI PORTC  */
	NonHandledInterrupt,			/* EXTI PORTD  */
	NonHandledInterrupt,			/* EXTI PORTE  */
	NonHandledInterrupt,			/* CAN RX      */
	NonHandledInterrupt,			/* CAN TX      */
	NonHandledInterrupt,			/* SPI         */
	NonHandledInterrupt,			/* TIMER 1 OVF */
	NonHandledInterrupt,			/* TIMER 1 CAP */
	NonHandledInterrupt,			/* TIMER 2 OVF */
	NonHandledInterrupt,			/* TIMER 2 CAP */
	NonHandledInterrupt,			/* TIMER 3 OVF */
	NonHandledInterrupt,			/* TIMER 3 CAP */
	NonHandledInterrupt,			/* USART TX    */
	recept,			/* USART RX    */
	NonHandledInterrupt,			/* I2C         */
	NonHandledInterrupt,			/* LINUART TX  */
	NonHandledInterrupt,			/* LINUART RX  */
	NonHandledInterrupt,			/* ADC         */
	NonHandledInterrupt,			/* TIMER 4 OVF */
	NonHandledInterrupt,			/* EEPROM ECC  */
	NonHandledInterrupt,			/* Reserved    */
	NonHandledInterrupt,			/* Reserved    */
	NonHandledInterrupt,			/* Reserved    */
	NonHandledInterrupt,			/* Reserved    */
	NonHandledInterrupt,			/* Reserved    */
	};
#endif