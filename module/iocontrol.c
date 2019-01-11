/*
 * iocontrol.c
 *
 *  Created on: Jan 4, 2019
 *      Author: Duemmer
 */


#include <stdbool.h>
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

#include <inc/hw_memmap.h>

#include "iocontrol.h"
#include "util.h"
#include "pinconfig.h"


static void ioctl_setPinConfigurations()
{
    // Set regular peripheral pin configurations
    GPIOPinConfigure(PINCFG_PWM1_PINMAP);
    GPIOPinConfigure(PINCFG_PWM2_PINMAP);
    GPIOPinConfigure(PINCFG_BQUART_RX_PINCONFIG);
    GPIOPinConfigure(PINCFG_BQUART_TX_PINCONFIG);
    GPIOPinConfigure(PINCFG_RS485UART_RX_PINCONFIG);
    GPIOPinConfigure(PINCFG_RS485UART_TX_PINCONFIG);
    GPIOPinConfigure(PINCFG_SDSPI_RX_PINCONFIG);
    GPIOPinConfigure(PINCFG_SDSPI_TX_PINCONFIG);
    GPIOPinConfigure(PINCFG_SDSPI_CLK_PINCONFIG);
    GPIOPinConfigure(PINCFG_SDSPI_FSS_PINCONFIG);
    GPIOPinConfigure(PINCFG_GPSSI_RX_PINCONFIG);
    GPIOPinConfigure(PINCFG_GPSSI_TX_PINCONFIG);
    GPIOPinConfigure(PINCFG_GPSSI_CLK_PINCONFIG);
    GPIOPinConfigure(PINCFG_GPSSI_FSS_PINCONFIG);
    GPIOPinConfigure(PINCFG_GPI2C_SDA_PINCONFIG);
    GPIOPinConfigure(PINCFG_GPI2C_SCL_PINCONFIG);
    GPIOPinConfigure(PINCFG_GPUART_RX_PINCONFIG);
    GPIOPinConfigure(PINCFG_GPUART_TX_PINCONFIG);
    GPIOPinConfigure(PINCFG_CAN_TX_PINCONFIG);
    GPIOPinConfigure(PINCFG_CAN_RX_PINCONFIG);

    // ADC pin configurations
    GPIOPinTypeADC(PINCFG_CURRENT1POS_PORT, PINCFG_CURRENT1POS_PIN);
    GPIOPinTypeADC(PINCFG_CURRENT1NEG_PORT, PINCFG_CURRENT1NEG_PIN);
    GPIOPinTypeADC(PINCFG_CURRENT2POS_PORT, PINCFG_CURRENT2POS_PIN);
    GPIOPinTypeADC(PINCFG_CURRENT2NEG_PORT, PINCFG_CURRENT2NEG_PIN);
    GPIOPinTypeADC(PINCFG_PACKVOLTAGE_PORT, PINCFG_PACKVOLTAGE_PIN);
    GPIOPinTypeADC(PINCFG_THERM1_PORT, PINCFG_THERM1_PIN);
    GPIOPinTypeADC(PINCFG_THERM2_PORT, PINCFG_THERM2_PIN);
    GPIOPinTypeADC(PINCFG_THERM3_PORT, PINCFG_THERM3_PIN);
}



static void ioctl_configADC()
{
    // Current sense sequence config. Continuous sample, run c1p, c1n, c2p, c2n
    ADCSequenceConfigure(
            PINCFG_CURRENTSENSE_MODULE,
            PINCFG_CURRENTSENSE_SEQUENCE,
            ADC_TRIGGER_ALWAYS,
            1
    );

    // Current 1 positive
    ADCSequenceStepConfigure(PINCFG_CURRENTSENSE_MODULE,
                             PINCFG_CURRENTSENSE_SEQUENCE,
                             0,
                             PINCFG_CURRENT1POS_CHANNEL);

    // Current 1 negative
    ADCSequenceStepConfigure(PINCFG_CURRENTSENSE_MODULE,
                             PINCFG_CURRENTSENSE_SEQUENCE,
                             1,
                             PINCFG_CURRENT1NEG_CHANNEL);

    // Current 2 positive
    ADCSequenceStepConfigure(PINCFG_CURRENTSENSE_MODULE,
                             PINCFG_CURRENTSENSE_SEQUENCE,
                             2,
                             PINCFG_CURRENT2POS_CHANNEL);

    // Current 2 negative
    ADCSequenceStepConfigure(PINCFG_CURRENTSENSE_MODULE,
                             PINCFG_CURRENTSENSE_SEQUENCE,
                             3,
                             PINCFG_CURRENT2NEG_CHANNEL);

    // Sequence End
    ADCSequenceStepConfigure(PINCFG_CURRENTSENSE_MODULE,
                             PINCFG_CURRENTSENSE_SEQUENCE,
                             4,
                             ADC_CTL_IE | ADC_CTL_END);

    // Run the proper oversampling
    ADCHardwareOversampleConfigure(PINCFG_CURRENTSENSE_MODULE,
                                   IOCTL_CURRENTSENSE_OVERSAMPLE);


    /* ===== Auxillary channel sampling config ====== */

    // Config pack voltage sense. Run from a timer trigger, assert an interrupt
    ADCSequenceConfigure(PINCFG_AUX_MODULE, PINCFG_PV_SEQUENCE, ADC_TRIGGER_TIMER, 1);
    ADCSequenceStepConfigure(PINCFG_AUX_MODULE, PINCFG_PV_SEQUENCE, 0, PINCFG_PV_CHANNEL);
    ADCSequenceStepConfigure(PINCFG_AUX_MODULE,
                             PINCFG_PV_SEQUENCE,
                             1,
                             ADC_CTL_IE | ADC_CTL_END);
    // Setup timer
    TimerConfigure(PINCFG_PV_TIMER, TIMER_CFG_PERIODIC);
    TimerClockSourceSet(PINCFG_PV_TIMER, TIMER_CLOCK_SYSTEM);
    TimerPrescaleSet(PINCFG_PV_TIMER, TIMER_BOTH, IOCTL_PV_TIMER_PRESCALE);
    TimerLoadSet(PINCFG_PV_TIMER, TIMER_BOTH, IOCTL_PV_TIMER_LOAD);


    // Config thermistor read. Read each thermistor, run from a timer trigger, assert an interrupt
    ADCSequenceConfigure(PINCFG_AUX_MODULE, PINCFG_PV_SEQUENCE, ADC_TRIGGER_TIMER, 1);
    ADCSequenceStepConfigure(PINCFG_AUX_MODULE, PINCFG_THERM_SEQUENCE, 0, PINCFG_THERM1_CHANNEL);
    ADCSequenceStepConfigure(PINCFG_AUX_MODULE, PINCFG_THERM_SEQUENCE, 1, PINCFG_THERM2_CHANNEL);
    ADCSequenceStepConfigure(PINCFG_AUX_MODULE, PINCFG_PV_SEQUENCE, 2, PINCFG_THERM3_CHANNEL);
    ADCSequenceStepConfigure(PINCFG_AUX_MODULE,
                             PINCFG_THERM_SEQUENCE,
                             3,
                             ADC_CTL_IE | ADC_CTL_END);
    // Setup timer
    TimerConfigure(PINCFG_THERM_TIMER, TIMER_CFG_PERIODIC);
    TimerClockSourceSet(PINCFG_THERM_TIMER, TIMER_CLOCK_SYSTEM);
    TimerPrescaleSet(PINCFG_THERM_TIMER, TIMER_BOTH, IOCTL_THERM_TIMER_PRESCALE);
    TimerLoadSet(PINCFG_THERM_TIMER, TIMER_BOTH, IOCTL_THERM_TIMER_LOAD);

    // Run the proper oversampling for aux
    ADCHardwareOversampleConfigure(PINCFG_AUX_MODULE, IOCTL_AUX_OVERSAMPLE);
}





void ioctl_configGPIO() {
    // Configure each pin's direction
    GPIOPinTypeGPIOInput(PINCFG_BQFAULT_PORT, PINCFG_BQFAULT_PIN);
    GPIOPinTypeGPIOInput(PINCFG_USERSWITCH_PORT, PINCFG_USERSWITCH_PIN);

    GPIOPinTypeGPIOOutput(PINCFG_CANSTB_PORT, PINCFG_CANSTB_PIN);
    GPIOPinTypeGPIOOutput(PINCFG_DEBUGLED1_PORT, PINCFG_DEBUGLED1_PIN);
    GPIOPinTypeGPIOOutput(PINCFG_DEBUGLED2_PORT, PINCFG_DEBUGLED2_PIN);

    // Set interrupt directions
    GPIOIntTypeSet(PINCFG_BQFAULT_PORT, PINCFG_BQFAULT_PIN, PINCFG_BQFAULT_INT_LEVEL);
    GPIOIntTypeSet(PINCFG_USERSWITCH_PORT, PINCFG_USERSWITCH_PIN, PINCFG_USERSWITCH_INT_LEVEL);
}




void ioctl_initialize() {
    // Set the clock to active operation
    SysCtlClockSet(
            SYSCTL_OSC_MAIN |
            MCU_XTAL |
            MCU_VCO |
            MCU_PLL_DIV
    );

    // Power all the peripherals
    for(int i=0; i<(sizeof(SYSCTL_PERIPHS) / sizeof(SYSCTL_PERIPHS[0])); i++)
        SysCtlPeripheralEnable(SYSCTL_PERIPHS[i]);

    // Start the system timer, at 0
    TimerConfigure(PINCFG_SYSTIME_TIMER, TIMER_CFG_ONE_SHOT_UP);
    TimerPrescaleSet(PINCFG_SYSTIME_TIMER, TIMER_BOTH, MCU_ACTIVE_FREQ / 1000000);
    TimerClockSourceSet(PINCFG_SYSTIME_TIMER, TIMER_CLOCK_SYSTEM);
    TimerLoadSet64(PINCFG_SYSTIME_TIMER, 0);
    TimerEnable(PINCFG_SYSTIME_TIMER, TIMER_BOTH);


    setPinConfigurations();
    configADC();
    configGPIO();
    FPUEnable();
    FPULazyStackingEnable();


}
