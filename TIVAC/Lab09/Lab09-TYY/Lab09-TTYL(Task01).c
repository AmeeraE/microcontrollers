#include <stdint.h>
#include <stdbool.h>
#include <math.h> //uses sinf() function
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h" //support for Floating Point Unit
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"

#ifndef M_PI  //defines M_PI
#define M_PI                    3.14159265358979323846
#endif

#define SERIES_LENGTH 100   //creates depth of data buffer

float gSeriesData[SERIES_LENGTH]; //creates an array of floats SERIES_LENGTH long

int32_t i32DataCount = 0;   //computation loop counter

int main(void)
{
    float fRadians;  //used to calculate sine

    ROM_FPULazyStackingEnable();    //turn on Lazy Stacking
    ROM_FPUEnable();    //turn on FPU, from reset it is off

    //set up system clock to 50MHz
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    fRadians = ((2 * M_PI) / SERIES_LENGTH); //creates full sine wave cycle

    while(i32DataCount < SERIES_LENGTH)
    {
        //loop to calculate the sine value for each
        //of the 100 values of the angle
        //then places them in the array
        gSeriesData[i32DataCount] = sinf(fRadians * i32DataCount); //calculation
        i32DataCount++;  //add to array count
    }

    while(1)
    {
    }
}
