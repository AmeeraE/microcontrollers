#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

uint8_t ui8PinData=2;

int main(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

	while(1)
	{
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, ui8PinData);
		SysCtlDelay(8000000);//changed to half a second
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
		SysCtlDelay(8000000);//changed to half a second
		if(ui8PinData==2) {ui8PinData=8;}//if red go to green
		else if(ui8PinData==8) {ui8PinData=4;}//if green go to blue
		else if(ui8PinData==4) {ui8PinData=10;}//if blue go to red/green
		else if(ui8PinData==10) {ui8PinData=6;}//if red/green go to red blue
		else if(ui8PinData==6) {ui8PinData=12;}//if red/blue go to green blue
		else if(ui8PinData==12) {ui8PinData=14;}//if green/blue go to red/green/blue
	    else if (ui8PinData==14) {ui8PinData=2;}//if red/green/blue go to red

		//2 8 4 10 6 12 14
		//2 =red  8=green   4=blue
	}
 }

