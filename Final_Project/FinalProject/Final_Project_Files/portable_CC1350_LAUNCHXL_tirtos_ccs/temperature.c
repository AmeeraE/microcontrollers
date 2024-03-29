/*
 * Copyright (c) 2016-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== temperature.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

#include <ti/display/Display.h>

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <time.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>

/* Example/Board Header files */
#include "Board.h"

/* ======== Si7021 Registers ======== */
#define Si7021_TMP_REG 0xE3
#define Si7021_HUM_REG 0xE5
#define Si7021_ADDR 0x40

/*
 *  ======== HIGH_TEMP ========
 *  Send alert when this temperature (in Celsius) is exceeded
 */
#define HIGH_TEMP 30

/*
 *  ======== TMP Registers ========
 */
#define TMP006_REG          0x0001  /* Die Temp Result Register for TMP006 */
#define TMP116_REG          0x0000  /* Die Temp Result Register for TMP116 */

/*
 *  The CC32XX LaunchPads come with an on-board TMP006 or TMP116 temperature
 *  sensor depending on the revision. Newer revisions come with the TMP116.
 *  The Build Automation Sensors (BOOSTXL-BASSENSORS) BoosterPack
 *  contains a TMP116.
 *
 *  We are using the DIE temperature because it's cool!
 *
 *  Additionally: no calibration is being done on the TMPxxx device to simplify
 *  the example code.
 */
#define TMP006_ADDR         0x41;
#define TMP116_BP_ADDR      0x48;
#define TMP116_LP_ADDR      0x49;

/* Temperature written by the temperature thread and read by console thread */
volatile float temperatureC;
volatile float temperatureF;
volatile float temperaturef;
volatile float temperature;
volatile float temp;
volatile float sample;

Display_Handle display;

/* Mutex to protect the reading/writing of the temperature variables */
extern pthread_mutex_t temperatureMutex;

/*
 *  ======== clearAlert ========
 *  Clear the LED
 */
//static void clearAlert(float temperature)
//{
//    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_OFF);
//}

/*
 *  ======== sendAlert ========
 *  Okay, just light a LED in this example, but with the SimpleLink SDK,
 *  you could send it out over the radio to something cool!
 */
//static void sendAlert(float temperature)
//{
//    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
//}

/*
 *  ======== postSem ========
 *  Function called when the timer (created in setupTimer) expires.
 */
static void postSem(union sigval val)
{
    sem_t *sem = (sem_t*)(val.sival_ptr);

    sem_post(sem);
}

/*
 *  ======== setupTimer ========
 *  Create a timer that will expire at the period specified by the
 *  time arguments. When the timer expires, the passed in semaphore
 *  will be posted by the postSem function.
 *
 *  A non-zero return indicates a failure.
 */
int setupTimer(sem_t *sem, timer_t *timerid, time_t sec, long nsec)
{
    struct sigevent   sev;
    struct itimerspec its;
    int               retc;

    retc = sem_init(sem, 0, 0);
    if (retc != 0) {
        return(retc);
    }

    /* Create the timer that wakes up the thread that will pend on the sem. */
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_value.sival_ptr = sem;
    sev.sigev_notify_function = &postSem;
    sev.sigev_notify_attributes = NULL;
    retc = timer_create(CLOCK_MONOTONIC, &sev, timerid);
    if (retc != 0) {
        return(retc);
    }

    /* Set the timer to go off at the specified period */
    its.it_interval.tv_sec = sec;
    its.it_interval.tv_nsec = nsec;
    its.it_value.tv_sec = sec;
    its.it_value.tv_nsec = nsec;
    retc = timer_settime(*timerid, 0, &its, NULL);
    if (retc != 0) {
        timer_delete(*timerid);
        return(retc);
    }

    return(0);
}

/*
 *  ======== temperatureThread ========
 *  This thread reads the temperature every second via I2C and sends an
 *  alert if it goes above HIGH_TEMP.
 */
void *temperatureThread(void *arg0)
{
    uint8_t         txBuffer[1];
    uint8_t         rxBuffer[2];
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;
    sem_t           semTimer;
//    timer_t         timerid;
//    int             retc;

    /* Configure the LED and if applicable, the TMP116_EN pin */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
#ifdef Board_GPIO_TMP116_EN
    GPIO_setConfig(Board_GPIO_TMP116_EN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
    /* 1.5 ms reset time for the TMP116 */
    sleep(1);
#endif

    /*
     *  Create/Open the I2C that talks to the TMP sensor
     */
    I2C_init();
    Display_init();

    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
        while (1);
    }

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf    = rxBuffer;
    i2cTransaction.readCount  = 2;

    /*
     * Determine which I2C sensor is present.
     * We will prefer sensors in this order: TMP116 (on BoosterPacks),
     * TMP116 (on-board CC32XX LaunchPads), and last TMP006
     * (on older CC32XX LaunchPads).
     */
    /*
    // Try TMP116 values
    txBuffer[0] = TMP116_REG;
    i2cTransaction.slaveAddress = TMP116_BP_ADDR;
    if (!I2C_transfer(i2c, &i2cTransaction)) {
        // Not BP TMP116, try LP TMP116
        i2cTransaction.slaveAddress = TMP116_LP_ADDR;
        if (!I2C_transfer(i2c, &i2cTransaction)) {
            // Not a TMP116 try TMP006
            txBuffer[0] = TMP006_REG;
            i2cTransaction.slaveAddress = TMP006_ADDR;
            if (!I2C_transfer(i2c, &i2cTransaction)) {
                // Could not resolve a sensor, error
                while(1);
            }
        }
    }
*/
    // Try Si7021
    txBuffer[0] = Si7021_TMP_REG;
    i2cTransaction.slaveAddress = Si7021_ADDR;
    if (!I2C_transfer(i2c, &i2cTransaction))
    {
        // Could not resolve a sensor, error
        Display_printf(display, 0, 0, "Error. No TMP sensor found!");
        while(1);
    }


    else
    {
        Display_printf(display, 0, 0, "Detected Si7021 sensor.");
    }

    // Take 20 samples and print them out onto the console
    for (sample = 0; sample < 100; sample++)
    {
        if (I2C_transfer(i2c, &i2cTransaction))
        {
            //
            // Extract degrees C from the received data;
            // see Si7021 datasheet
            //
            temp = (rxBuffer[0] << 8) | (rxBuffer[1]);
            temperature = (((175.72 * temp)/ 65536) - 46.85); // celsius
            temperaturef = (temperature * (1.8)) + 32; //farenheit
            Display_printf(display, 0, 0, "Sample %u: %d (C)", sample, temperaturef);
        }

        else
        {
            Display_printf(display, 0, 0, "I2C Bus fault.");
        }
    }

    /*
     *  The temperature thread blocks on the semTimer semaphore, which the
     *  timerId timer will post every second. The timer is created in the
     *  setupTimer function. It's returned so the thread could change the
     *  period or delete it if desired.
     */
//    retc = setupTimer(&semTimer, &timerid, 1, 0);
//    if (retc != 0) {
//        while (1);
//    }

//    while (1)
//    {
//        if (I2C_transfer(i2c, &i2cTransaction)) {
//            /*
//             *  Extract degrees C from the received data; see sensor datasheet.
//             *  Make sure we are updating the global temperature variables
//             *  in a thread-safe manner.
//             */
//            pthread_mutex_lock(&temperatureMutex);
//            temperatureC = (rxBuffer[0] << 6) | (rxBuffer[1] >> 2);
//            temperatureC *= 0.03125;
//            temperatureF = temperatureC * 9 / 5 + 32;
//            pthread_mutex_unlock(&temperatureMutex);
//
//            /*  Send an alert if the temperature is too high!! */
//            if ((int)temperatureC >= HIGH_TEMP) {
//                sendAlert(temperatureC);
//            }
//            else {
//                clearAlert(temperatureC);
//            }
//    }

//        /* Block until the timer posts the semaphore. */
//        retc = sem_wait(&semTimer);
//        if (retc == -1) {
//            while (1);
//        }
//    }
}
