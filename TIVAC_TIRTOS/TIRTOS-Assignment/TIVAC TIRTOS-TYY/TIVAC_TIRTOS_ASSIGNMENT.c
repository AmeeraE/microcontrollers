#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include <xdc/runtime/Log.h>
#include <xdc/cfg/global.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

void hardware_init(void);
void ledToggle(void);
void Timer_ISR(void);
void initADC();
void getADC3(void);
void InitConsole(void);
void UARTdisplayADC(void);

volatile int16_t i16ToggleCount = 0;
volatile int16_t i16InstanceCount = 0;

uint32_t ADCValues[1];

// variable used to store the output of the ADC C3
uint32_t adc3 ;

void main(void)
{
   hardware_init();
   initADC();
   InitConsole();
   BIOS_start();
}

void hardware_init(void)
{
	uint32_t ui32Period;

	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

	// Turn on LED
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 8);

	// Timer 2 setup
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);

	ui32Period = (SysCtlClockGet() / 20);
	TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period);

	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER2_BASE, TIMER_A);
}

void UARTdisplayADC(void) {
    while(1) {
        Semaphore_pend(UARTSem, BIOS_WAIT_FOREVER);
        UARTprintf("ADC CH3 Value: %d\n\n", adc3);
    }
}

void ledToggle(void)
{
	while(1)
	{
	    Semaphore_pend(LEDSem, BIOS_WAIT_FOREVER);

	    // LED values: 8=GREEN
	        if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2))
	        {
	            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
	        }
	        else
	        {
	            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 8);
	        }
	        i16ToggleCount += 1;
	        Log_info1("LED TOGGLED [%u] TIMES",i16ToggleCount);
	}
}

void Timer_ISR(void)
{
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    if(i16InstanceCount == 10)
    {
        Semaphore_post(ADC3Sem);
    }

    else if (i16InstanceCount == 20)
    {
        Semaphore_post(UARTSem);
    }

    else if(i16InstanceCount == 30)
    {
        Semaphore_post(LEDSem);
        i16InstanceCount = 0;
    }
    i16InstanceCount++;
}

void InitConsole(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, 16000000);
}

// Initialize ADC0
void initADC() {
                SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
                SysCtlDelay(3);
                SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
                SysCtlDelay(3);
                GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
                ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
                ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH3  | ADC_CTL_IE | ADC_CTL_END);
                ADCSequenceEnable(ADC0_BASE, 3);
                ADCIntClear(ADC0_BASE, 3);
}

// Receives value from ADC0 CH 3

void getADC3(void) {

    while(1) {
        Semaphore_pend(ADC3Sem, BIOS_WAIT_FOREVER);
        ADCProcessorTrigger(ADC0_BASE, 3);

        while(!ADCIntStatus(ADC0_BASE, 3, false))
        {
        }

        ADCIntClear(ADC0_BASE, 3);

        ADCSequenceDataGet(ADC0_BASE, 3, ADCValues);
        adc3 = ADCValues[0];
    }
}
