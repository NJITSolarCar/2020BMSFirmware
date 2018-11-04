/*
 * pinconfig.c
 *
 *  Created on: Nov 4, 2018
 *      Author: Duemmer
 */

#include "pinconfig.h"

#include <stdint.h>

#include <driverlib/gpio.h>
#include <driverlib/adc.h>
#include <driverlib/can.h>
#include <driverlib/sysctl.h>
#include <driverlib/ssi.h>
#include <driverlib/uart.h>
#include <driverlib/i2c.h>
#include <driverlib/fpu.h>
#include <driverlib/timer.h>

void sys_init()
{
    // Set the clock to active operation
    SysCtlClockSet(
            MCU_XTAL |
            MCU_VCO |
            MCU_PLL_DIV |
    );

    // Power all the peripherals
    for(int i=0; i<(sizeof(SYSCTL_PERIPHS) / sizeof(SYSCTL_PERIPHS[0]); i++)
        SysCtlPeripheralEnable(SYSCTL_PERIPHS[i]);
}
