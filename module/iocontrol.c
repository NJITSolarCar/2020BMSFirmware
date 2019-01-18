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
#include <driverlib/interrupt.h>

#include <inc/hw_memmap.h>

#include "iocontrol.h"
#include "calculation.h"
#include "util.h"
#include "pinconfig.h"
#include "flag.h"

// Utility inline ISRs. Since timer triggering of ADC samples doesn't seem to work
// with more than 1 timer / sequence, just run a simple ISR to trigger the ADC
void _ioctl_isr_currSenTrig() {
    ADCProcessorTrigger(PINCFG_CURRSEN_MODULE, PINCFG_CURRSEN_SEQUENCE);
}

void _ioctl_isr_pvReadTrig() {
    ADCProcessorTrigger(PINCFG_AUX_MODULE, PINCFG_PV_SEQUENCE);
}

void _ioctl_isr_mcuThermTrig() {
    ADCProcessorTrigger(PINCFG_AUX_MODULE, PINCFG_THERM_SEQUENCE);
}


// Current Sensor ADC conversion done
void ioctl_isr_currentSense() {
    uint32_t puiBuf[4];
    if(ADCSequenceDataGet(PINCFG_CURRSEN_MODULE, PINCFG_CURRSEN_SEQUENCE, puiBuf) == 4) {
        // Sample order is c1p, c1n, c2p, c2n
        g_fCurrent = calc_adcToAmps(puiBuf[0], puiBuf[1], puiBuf[2], puiBuf[3]);
        g_fCurrentValid = true;
    } else {
        // TODO Treat as a fault?
    }
}




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
    GPIOPinTypeADC(PINCFG_PV_PORT, PINCFG_PV_PIN);
    GPIOPinTypeADC(PINCFG_THERM1_PORT, PINCFG_THERM1_PIN);
    GPIOPinTypeADC(PINCFG_THERM2_PORT, PINCFG_THERM2_PIN);
    GPIOPinTypeADC(PINCFG_THERM3_PORT, PINCFG_THERM3_PIN);
}



static void ioctl_configADC()
{
    // Current sense sequence config. Processor trigger (from timer ISR),
    // run c1p, c1n, c2p, c2n
    ADCSequenceConfigure(
            PINCFG_CURRSEN_MODULE,
            PINCFG_CURRSEN_SEQUENCE,
            ADC_TRIGGER_PROCESSOR,
            1
    );

    // Current 1 positive
    ADCSequenceStepConfigure(PINCFG_CURRSEN_MODULE,
                             PINCFG_CURRSEN_SEQUENCE,
                             0,
                             PINCFG_CURRENT1POS_CHANNEL);

    // Current 1 negative
    ADCSequenceStepConfigure(PINCFG_CURRSEN_MODULE,
                             PINCFG_CURRSEN_SEQUENCE,
                             1,
                             PINCFG_CURRENT1NEG_CHANNEL);

    // Current 2 positive
    ADCSequenceStepConfigure(PINCFG_CURRSEN_MODULE,
                             PINCFG_CURRSEN_SEQUENCE,
                             2,
                             PINCFG_CURRENT2POS_CHANNEL);

    // Current 2 negative
    ADCSequenceStepConfigure(PINCFG_CURRSEN_MODULE,
                             PINCFG_CURRSEN_SEQUENCE,
                             3,
                             PINCFG_CURRENT2NEG_CHANNEL);

    // Sequence End
    ADCSequenceStepConfigure(PINCFG_CURRSEN_MODULE,
                             PINCFG_CURRSEN_SEQUENCE,
                             4,
                             ADC_CTL_IE | ADC_CTL_END);

    // Run the proper oversampling
    ADCHardwareOversampleConfigure(PINCFG_CURRSEN_MODULE,
                                   IOCTL_CURRSEN_OVERSAMPLE);


    /* ===== Auxillary channel sampling config ====== */

    // Config pack voltage sense. Run from a processor trigger, assert an interrupt
    ADCSequenceConfigure(PINCFG_AUX_MODULE, PINCFG_PV_SEQUENCE, ADC_TRIGGER_PROCESSOR, 1);
    ADCSequenceStepConfigure(PINCFG_AUX_MODULE, PINCFG_PV_SEQUENCE, 0, PINCFG_PV_CHANNEL);
    ADCSequenceStepConfigure(PINCFG_AUX_MODULE,
                             PINCFG_PV_SEQUENCE,
                             1,
                             ADC_CTL_IE | ADC_CTL_END);

    // Config thermistor read. Read each thermistor, run from a timer trigger, assert an interrupt
    ADCSequenceConfigure(PINCFG_AUX_MODULE, PINCFG_PV_SEQUENCE, ADC_TRIGGER_PROCESSOR, 1);
    ADCSequenceStepConfigure(PINCFG_AUX_MODULE, PINCFG_THERM_SEQUENCE, 0, PINCFG_THERM1_CHANNEL);
    ADCSequenceStepConfigure(PINCFG_AUX_MODULE, PINCFG_THERM_SEQUENCE, 1, PINCFG_THERM2_CHANNEL);
    ADCSequenceStepConfigure(PINCFG_AUX_MODULE, PINCFG_PV_SEQUENCE, 2, PINCFG_THERM3_CHANNEL);
    ADCSequenceStepConfigure(PINCFG_AUX_MODULE,
                             PINCFG_THERM_SEQUENCE,
                             3,
                             ADC_CTL_IE | ADC_CTL_END);

    // Run the proper oversampling for aux
    ADCHardwareOversampleConfigure(PINCFG_AUX_MODULE, IOCTL_AUX_OVERSAMPLE);
}



void ioctl_configADCTimers() {
    uint32_t ui32ClkMHz = SysCtlClockGet() / 1000000;

    // Current Sense timer
    TimerConfigure(PINCFG_TIMER_CURRSEN, PINCFG_TIMER_CFG_CURRSEN);
    TimerClockSourceSet(PINCFG_TIMER_CURRSEN, TIMER_CLOCK_SYSTEM);
    TimerPrescaleSet(PINCFG_TIMER_CURRSEN, PINCFG_TIMER_PART_CURRSEN, ui32ClkMHz);
    IntRegister(PINCFG_VEC_CURRSEN_TRIG, _ioctl_isr_currSenTrig);
    IntPrioritySet(PINCFG_VEC_CURRSEN_TRIG, 0);
    IntEnable(PINCFG_VEC_CURRSEN_TRIG);

    // PV read timer
    TimerConfigure(PINCFG_TIMER_PV, PINCFG_TIMER_CFG_PV);
    TimerClockSourceSet(PINCFG_TIMER_PV, TIMER_CLOCK_SYSTEM);
    TimerPrescaleSet(PINCFG_TIMER_PV, PINCFG_TIMER_PART_PV, ui32ClkMHz);
    IntRegister(PINCFG_VEC_PV_READ_TRIG, _ioctl_isr_pvReadTrig);
    IntPrioritySet(PINCFG_VEC_PV_READ_TRIG, 0);
    IntEnable(PINCFG_VEC_PV_READ_TRIG);

    // MCU Thermo timer
    TimerConfigure(PINCFG_TIMER_MCU_THERM, PINCFG_TIMER_CFG_MCU_THERM);
    TimerClockSourceSet(PINCFG_TIMER_MCU_THERM, TIMER_CLOCK_SYSTEM);
    TimerPrescaleSet(PINCFG_TIMER_MCU_THERM, PINCFG_TIMER_PART_MCU_THERM, ui32ClkMHz);
    IntRegister(PINCFG_VEC_MCU_THERM_TRIG, _ioctl_isr_mcuThermTrig);
    IntPrioritySet(PINCFG_VEC_MCU_THERM_TRIG, 0);
    IntEnable(PINCFG_VEC_MCU_THERM_TRIG);
}





void ioctl_configGPIO() {
    // Configure each pin's direction
    GPIOPinTypeGPIOInput(PINCFG_BQFAULT_PORT, PINCFG_BQFAULT_PIN);
    GPIOPinTypeGPIOInput(PINCFG_SW1_PORT, PINCFG_SW1_PIN);

    GPIOPinTypeGPIOOutput(PINCFG_CANSTB_PORT, PINCFG_CANSTB_PIN);
    GPIOPinTypeGPIOOutput(PINCFG_DEBUGLED1_PORT, PINCFG_DEBUGLED1_PIN);
    GPIOPinTypeGPIOOutput(PINCFG_DEBUGLED2_PORT, PINCFG_DEBUGLED2_PIN);

    // Set interrupt directions
    GPIOIntTypeSet(PINCFG_BQFAULT_PORT, PINCFG_BQFAULT_PIN, PINCFG_BQFAULT_INT_LEVEL);
    GPIOIntTypeSet(PINCFG_SW1_PORT, PINCFG_SW1_PIN, PINCFG_SW1_INT_LEVEL);
}




void ioctl_initialize() {
    // Set the clock to active operation
    SysCtlClockSet(
            SYSCTL_OSC_MAIN |
            MCU_XTAL |
            MCU_VCO |
            MCU_PLL_DIV
    );

    uint32_t ui32Clk = SysCtlClockGet();

    // Power all the peripherals
    for(int i=0; i<(sizeof(SYSCTL_PERIPHS) / sizeof(SYSCTL_PERIPHS[0])); i++)
        SysCtlPeripheralEnable(SYSCTL_PERIPHS[i]);

    FPUEnable();
    FPULazyStackingEnable();

    // Start the system timer, and start counting
    TimerConfigure(PINCFG_TIMER_SYSTIME, PINCFG_TIMER_CFG_SYSTIME);
    TimerPrescaleSet(PINCFG_TIMER_SYSTIME, PINCFG_TIMER_PART_SYSTIME, ui32Clk / 1000000);
    TimerClockSourceSet(PINCFG_TIMER_SYSTIME, TIMER_CLOCK_SYSTEM);
    TimerLoadSet64(PINCFG_TIMER_SYSTIME, 0);
    TimerEnable(PINCFG_TIMER_SYSTIME, PINCFG_TIMER_PART_SYSTIME);

    // Initialize the different subsets of the system
    ioctl_setPinConfigurations();
    ioctl_configADC();
    ioctl_configGPIO();
    ioctl_configADCTimers();


}
