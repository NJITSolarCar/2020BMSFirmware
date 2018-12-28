/*
 * pinconfig.c
 *
 *  Created on: Nov 4, 2018
 *      Author: Duemmer
 */

#include "pinconfig.h"
#include "system.h"
#include "util.h"
#include "iocontrol.h"
#include "config.h"
#include "bq76.h"
#include "calculation.h"

#include <stdint.h>
#include <stdbool.h>

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


static tConf *psConfig;

void setPinConfigurations()
{
    // Set regular peripheral pin configurations
    GPIOPinConfigure(PWM1_PINMAP);
    GPIOPinConfigure(PWM2_PINMAP);
    GPIOPinConfigure(BQUART_RX_PINCONFIG);
    GPIOPinConfigure(BQUART_TX_PINCONFIG);
    GPIOPinConfigure(RS485UART_RX_PINCONFIG);
    GPIOPinConfigure(RS485UART_TX_PINCONFIG);
    GPIOPinConfigure(SDSPI_RX_PINCONFIG);
    GPIOPinConfigure(SDSPI_TX_PINCONFIG);
    GPIOPinConfigure(SDSPI_CLK_PINCONFIG);
    GPIOPinConfigure(SDSPI_FSS_PINCONFIG);
    GPIOPinConfigure(GPSSI_RX_PINCONFIG);
    GPIOPinConfigure(GPSSI_TX_PINCONFIG);
    GPIOPinConfigure(GPSSI_CLK_PINCONFIG);
    GPIOPinConfigure(GPSSI_FSS_PINCONFIG);
    GPIOPinConfigure(GPI2C_SDA_PINCONFIG);
    GPIOPinConfigure(GPI2C_SCL_PINCONFIG);
    GPIOPinConfigure(GPUART_RX_PINCONFIG);
    GPIOPinConfigure(GPUART_TX_PINCONFIG);
    GPIOPinConfigure(CAN_TX_PINCONFIG);
    GPIOPinConfigure(CAN_RX_PINCONFIG);

    // ADC pin configurations
    GPIOPinTypeADC(CURRENT1POS_PORT, CURRENT1POS_PIN);
    GPIOPinTypeADC(CURRENT1NEG_PORT, CURRENT1NEG_PIN);
    GPIOPinTypeADC(CURRENT2POS_PORT, CURRENT2POS_PIN);
    GPIOPinTypeADC(CURRENT2NEG_PORT, CURRENT2NEG_PIN);
    GPIOPinTypeADC(PACKVOLTAGE_PORT, PACKVOLTAGE_PIN);
    GPIOPinTypeADC(THERM1_PORT, THERM1_PIN);
    GPIOPinTypeADC(THERM2_PORT, THERM2_PIN);
    GPIOPinTypeADC(THERM3_PORT, THERM3_PIN);
}



void configADC()
{
    // Current sense sequence config. Always runs,
    // 64x oversample, asserts an interrupt, and reads each current sensor ADC line
    ADCSequenceConfigure(
            CURRENT_MODULE,
            CURRENT_SEQUENCE,
            ADC_TRIGGER_ALWAYS,
            1
    );
    ADCSequenceStepConfigure(
            CURRENT_MODULE,
            CURRENT_SEQUENCE,
            CURRENT_STEP,
            CURRENT1POS_CHANNEL |
            CURRENT1NEG_CHANNEL |
            CURRENT2POS_CHANNEL |
            CURRENT2NEG_CHANNEL |
            ADC_CTL_IE |
            ADC_CTL_END
    );
    ADCHardwareOversampleConfigure(CURRENT_MODULE, CURRENT_OVERSAMPLE);

    // Temperature / pack voltage sensor sequence. Triggered by timer 5,
    // 64x oversample, asserts interrupt, reads pack voltage / thermistors
    ADCSequenceConfigure(
            THERMPV_MODULE,
            THERMPV_SEQUENCE,
            ADC_TRIGGER_TIMER,
            0
    );
    ADCSequenceStepConfigure(
            THERMPV_MODULE,
            THERMPV_SEQUENCE,
            THERMPV_STEP,
            PACKVOLTAGE_CHANNEL |
            THERM1_CHANNEL |
            THERM2_CHANNEL |
            THERM3_CHANNEL
    );
    ADCHardwareOversampleConfigure(THERMPV_MODULE, THERMPV_OVERSAMPLE);
    // Setup ADC timer
    TimerConfigure(THERMPV_TIMER, THERMPV_TIMERCFG);
    TimerLoadSet(
            THERMPV_TIMER,
            TIMER_BOTH,
            MCU_ACTIVE_FREQ / THERMPV_SAMPLE_FREQ
    );
}



void configGPIO() {
    // Configure each pin's direction
    GPIOPinTypeGPIOInput(BQFAULT_PORT, BQFAULT_PIN);
    GPIOPinTypeGPIOInput(USERSWITCH_PORT, USERSWITCH_PIN);

    GPIOPinTypeGPIOOutput(CANSTB_PORT, CANSTB_PIN);
    GPIOPinTypeGPIOOutput(DEBUGLED1_PORT, DEBUGLED1_PIN);
    GPIOPinTypeGPIOOutput(DEBUGLED2_PORT, DEBUGLED2_PIN);

    // Set interrupt directions
    GPIOIntTypeSet(BQFAULT_PORT, BQFAULT_PIN, BQFAULT_INT_LEVEL);
    GPIOIntTypeSet(USERSWITCH_PORT, USERSWITCH_PIN, USERSWITCH_INT_LEVEL);
}



void sys_init()
{
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

    setPinConfigurations();
    configADC();
    configGPIO();
    FPUEnable();
    FPUStackingDisable();
}



static uint64_t g_ui64SysTickLastCall = 0;
void system_tick(uint64_t ui64NumCalls) {

    // Get timing intervals
    uint64_t ui64Now = util_usecs();
    uint64_t ui64dt = ui64Now - g_ui64SysTickLastCall;
    g_ui64SysTickLastCall = ui64Now;


    ////////////////////// Variables //////////////////////
    // Raw current sensor readings
    uint32_t ui32C1p;
    uint32_t ui32C1n;
    uint32_t ui32C2p;
    uint32_t ui32C2n;

    // Auxiliary adc readings
    uint32_t ui32PackVolts;
    uint32_t ui32Therm1;
    uint32_t ui32Therm2;
    uint32_t ui32Therm3;

    // Intermediate cell voltage readings
    uint32_t ui32NumCells = config_totalNumcells(psConfig);
    uint16_t pui16CellSamples[ui32NumCells];


    ////////////////////// Acquire sampled data //////////////////////
    ioctl_sampledCurrent(&ui32C1p, &ui32C1n, &ui32C2p, &ui32C2n);
    ioctl_sampledAux(&ui32PackVolts, &ui32Therm1, &ui32Therm2, &ui32Therm3);
    bq76_readSampledVoltages(pui16CellSamples, ui32NumCells);

    // check thermals if needed
    bool bDoThermo = !(ui64NumCalls % SYSTEM_THERMO_PART);
    if(bDoThermo) {
        // Intermediate thermistor readings for the BQ
        uint16_t pui16ThermoSamples[psConfig->ui8NumBQModules*BQ_NUM_THERMISTOR];
        bq76_readThermistors(pui16ThermoSamples);

        //
    }


    ////////////////////// Do calculations //////////////////////
    g_fCurrent = ((float)calc_adcToMilliAmps(
            ui32C1p,
            ui32C1n,
            ui32C2p,
            ui32C2n)) * 1E-3;

}























