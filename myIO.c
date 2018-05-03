/*
 * myIO.c
 *
 *  Created on: Jun 20, 2015
 *      Author: NhatTan
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

#include "myIO.h"


volatile uint8_t toggle_led[3];
void myIO_init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//	SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

//	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2);
//	GPIOPinTypeGPIOOutput(GPIO_PORTF_AHB_BASE,GPIO_PIN_3);

//	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
//	GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_STRENGTH_8MA,GPIO_PIN_TYPE_STD_WPU);

}

void led(uint8_t led_color,uint8_t state)
{
	if(state==1)
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3)|led_color);
	else if(state==0)
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3)&~led_color);
	else
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3,0);
}

bool button()
{
	if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)!=GPIO_PIN_4)
		return true;
	else
		return false;
}
