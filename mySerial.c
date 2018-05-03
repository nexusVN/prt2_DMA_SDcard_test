/*
 * mySerial.c
 *
 *  Created on: Mar 1, 2017
 *      Author: Nhat Tan
 */

/*-----system include----------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <ctype.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "utils/ringbuf.h"


/*-----my include-------------------*/
#include "mySerial.h"
#include "myIO.h"
#include "myFIFO.h"

MySerial serialPC;
FIFO serialPcFIFO;

//void UART_PC_Init(){
//	UART3_Init();
//}
void UART2_Init(){
	//
	// Enable Peripheral Clocks
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	//
	// Enable pin PD6 for UART2 U2RX
	//
	GPIOPinConfigure(GPIO_PD6_U2RX);
	GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6);

	//
	// Enable pin PD7 for UART2 U2TX
	// First open the lock and select the bits we want to modify in the GPIO commit register.
	//
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;
	//
	// Now modify the configuration of the pins that we unlocked.
	//
	GPIOPinConfigure(GPIO_PD7_U2TX);
	GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_7);

	UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	IntMasterEnable(); //enable processor interrupts
	IntEnable(INT_UART2); //enable the UART interrupt

	UARTFIFOLevelSet(UART2_BASE,UART_FIFO_TX1_8,UART_FIFO_RX4_8);
	UARTIntEnable(UART2_BASE, UART_INT_RX|UART_INT_RT);
}
void UART3_Init(){
	//
	// Enable Peripheral Clocks
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	//
	// Enable pin PC7 for UART3 U3TX
	//
	GPIOPinConfigure(GPIO_PC7_U3TX);
	GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_7);

	//
	// Enable pin PC6 for UART3 U3RX
	//
	GPIOPinConfigure(GPIO_PC6_U3RX);
	GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6);

	UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 1500000,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	IntMasterEnable(); //enable processor interrupts
	IntEnable(INT_UART3); //enable the UART interrupt

	UARTFIFOLevelSet(UART3_BASE,UART_FIFO_TX1_8,UART_FIFO_RX4_8);
	UARTIntEnable(UART3_BASE, UART_INT_RX|UART_INT_RT);

	myFIFO_init(&serialPcFIFO);
}

void UART3_Interrupt_Handler()
{
	uint32_t interrupt_status = UARTIntStatus(UART3_BASE,true);
	//	if(interrupt_status&UART_INT_RX)
	//	{
	//		UARTIntClear(UART3_BASE,UART_INT_RX);//|UART_INT_RX
	//		//		SerialGetStr(UART0_BASE,Uart.Command_Data);
	//		toggle_led[0]^=1;
	//		led(LED_RED,toggle_led[0]);
	//	}
	//	else if(interrupt_status&UART_INT_RT)
	//	{
	//		UARTIntClear(UART3_BASE,UART_INT_RT);//|UART_INT_RX
	//		serialGetData(UART3_BASE,serialPC.Command_Data);
	//		serialPC.Flag_receive=1;
	//
	//		toggle_led[1]^=1;
	//		led(LED_BLUE,toggle_led[1]);
	//	}

	/*
	 * FIFO version using ringbuffer
	 */
	//	if(interrupt_status&UART_INT_RX)
	//	{
	//		UARTIntClear(UART3_BASE,UART_INT_RX);
	//		toggle_led[0]^=1;
	//					led(LED_RED,toggle_led[0]);
	//		while(UARTCharsAvail(UART3_BASE))
	//		{
	//			char charData=(char)UARTCharGet(UART3_BASE);
	//			if(charData==0xFF){
	//
	//				FIFO_Rx_clear(&serialPcFIFO);
	//			}
	//			FIFO_Rx_CharPut(&serialPcFIFO,charData);
	//		}
	//		//		serialGetData(UART3_BASE,serialPC.Command_Data);
	//		toggle_led[0]^=1;
	//				led(LED_RED,toggle_led[0]);
	//	}
	//	else if(interrupt_status&UART_INT_RT)
	//	{
	//		UARTIntClear(UART3_BASE,UART_INT_RT);
	//		toggle_led[1]^=1;
	//				led(LED_BLUE,toggle_led[1]);
	//		while(UARTCharsAvail(UART3_BASE))
	//		{
	//			char charData=(char)UARTCharGet(UART3_BASE);
	//			if(charData==0xFF){
	//				FIFO_Rx_clear(&serialPcFIFO);
	//			}
	//			FIFO_Rx_CharPut(&serialPcFIFO,charData);
	//		}
	//
	//		serialPC.Flag_receive=1;
	//		toggle_led[1]^=1;
	//		led(LED_BLUE,toggle_led[1]);
	//	}

	/*
	 * my buffer version  version
	 */
	if(interrupt_status&UART_INT_RX)
	{
		UARTIntClear(UART3_BASE,UART_INT_RX);
		toggle_led[0]^=1;
		led(LED_RED,toggle_led[0]);
		while(UARTCharsAvail(UART3_BASE))
		{
			char charData=(char)UARTCharGet(UART3_BASE);
			if(charData==PC_HEADER_){
				serialPC.dataIndex=0;
				//				serialPC.dataCount=0;
				//					FIFO_Rx_clear(&serialPcFIFO);
			}
			else if(charData==PC_TERMINATOR_){
				serialPC.Flag_receive=1;
			}
			if(serialPC.dataIndex<SERIAL_BUFFER_SIZE_)
				serialPC.Command_Data[serialPC.dataIndex++]=charData;
			else
				serialPC.dataIndex=0;
			//				FIFO_Rx_CharPut(&serialPcFIFO,charData);
		}
		//		serialGetData(UART3_BASE,serialPC.Command_Data);
		toggle_led[0]^=1;
		led(LED_RED,toggle_led[0]);
	}
	else if(interrupt_status&UART_INT_RT)
	{
		UARTIntClear(UART3_BASE,UART_INT_RT);
//		toggle_led[1]^=1;
//		led(LED_BLUE,toggle_led[1]);
		while(UARTCharsAvail(UART3_BASE))
		{
			char charData=(char)UARTCharGet(UART3_BASE);
			if(charData==PC_HEADER_){
				//					FIFO_Rx_clear(&serialPcFIFO);
				serialPC.dataIndex=0;
				//				serialPC.dataCount=0;
			}
			else if(charData==PC_TERMINATOR_){
				serialPC.Flag_receive=1;
			}

			if(serialPC.dataIndex<SERIAL_BUFFER_SIZE_)
				serialPC.Command_Data[serialPC.dataIndex++]=charData;
			else
				serialPC.dataIndex=0;
			//				FIFO_Rx_CharPut(&serialPcFIFO,charData);
		}
//		toggle_led[1]^=1;
//		led(LED_BLUE,toggle_led[1]);
	}
}

void UART4_Init(){
	//
	// Enable pin PC4 for UART4 U4RX
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	//
	// Enable pin PC4 for UART4 U4RX
	//
	GPIOPinConfigure(GPIO_PC4_U4RX);
	GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4);

	//
	// Enable pin PC5 for UART4 U4TX
	//
	GPIOPinConfigure(GPIO_PC5_U4TX);
	GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_5);

	UARTConfigSetExpClk(UART4_BASE, SysCtlClockGet(), 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	IntMasterEnable(); //enable processor interrupts
	IntEnable(INT_UART4); //enable the UART interrupt

	UARTFIFOLevelSet(UART4_BASE,UART_FIFO_TX1_8,UART_FIFO_RX4_8);
	UARTIntEnable(UART4_BASE, UART_INT_RX|UART_INT_RT);

}
void UART7_Init(){
	//
	// Enable Peripheral Clocks
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//
	// Enable pin PE0 for UART7 U7RX
	//
	GPIOPinConfigure(GPIO_PE0_U7RX);
	GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0);

	//
	// Enable pin PE1 for UART7 U7TX
	//
	GPIOPinConfigure(GPIO_PE1_U7TX);
	GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_1);

	UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	IntMasterEnable(); //enable processor interrupts
	IntEnable(INT_UART7); //enable the UART interrupt

	UARTFIFOLevelSet(UART7_BASE,UART_FIFO_TX1_8,UART_FIFO_RX4_8);
	UARTIntEnable(UART7_BASE, UART_INT_RX|UART_INT_RT);
}


void serialGetData(uint32_t ui32Base,char *uart_str)
{
	while(UARTCharsAvail(ui32Base))
	{
		*uart_str++=(char)UARTCharGet(ui32Base);
	}
}


