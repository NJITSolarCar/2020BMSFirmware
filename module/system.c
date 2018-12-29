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
#include "types.h"
#include "flag.h"
#include "fault.h"

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


static tConf *g_psConfig;


static void system_readThermoData(uint32_t ui32Therm1, uint32_t ui32Therm2,
                           uint32_t ui32Therm3)
{
    // Intermediate thermistor readings for the BQ
    uint32_t ui32TotalNumTherm = g_psConfig->ui32NumBQModules
            * BQ_NUM_THERMISTOR;

    uint16_t pui16ThermoSamples[ui32TotalNumTherm];
    bq76_readThermistors(pui16ThermoSamples);

    // First 3 thermistors are on the main board, with a 12 bit ADC
    g_pfThermoTemperatures[0] = calc_thermistorTemp(
            ((float) ui32Therm1) / 4096.0);
    g_pfThermoTemperatures[1] = calc_thermistorTemp(
            ((float) ui32Therm2) / 4096.0);
    g_pfThermoTemperatures[2] = calc_thermistorTemp(
            ((float) ui32Therm3) / 4096.0);

    // The others are on the BQ boards, which gives a 16 bit result
    float fTmp;
    for (uint32_t i = 0; i < ui32TotalNumTherm; i++)
    {
        fTmp = calc_thermistorTemp(((float) pui16ThermoSamples[i]) / 65536.0);
        g_pfThermoTemperatures[i + 3] = fTmp;
    }
}




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
    uint32_t ui32NumCells = config_totalNumcells(g_psConfig);
    uint16_t pui16CellSamples[ui32NumCells];

    // Pack voltage from different ways
    float fPVFromCells = 0.0;
    float fPVFromADC;

    // maximum and minimum cell voltages
    float fMinCellVolts = 10.0;
    float fMaxCellVolts = -1.0;
    uint16_t ui16MinCellIdx = 0;
    uint16_t ui16MaxCellIdx = 0;


    // Get timing intervals
    uint64_t ui64Now = util_usecs();
    uint64_t ui64dt = ui64Now - g_ui64SysTickLastCall;
    g_ui64SysTickLastCall = ui64Now;


    ////////////////////// Acquire sampled data //////////////////////
    ioctl_sampledCurrent(&ui32C1p, &ui32C1n, &ui32C2p, &ui32C2n);
    ioctl_sampledAux(&ui32PackVolts, &ui32Therm1, &ui32Therm2, &ui32Therm3);
    bq76_readSampledVoltages(pui16CellSamples, ui32NumCells);

    // Calculate thermals (a slow task) only if needed
    bool bDoThermo = !(ui64NumCalls % SYSTEM_THERMO_PART);
    if(bDoThermo)
        system_readThermoData(ui32Therm1, ui32Therm2, ui32Therm3);


    ////////////////////// Do calculations //////////////////////
    // Determine current
    g_fCurrent = ((float)calc_adcToMilliAmps(
            ui32C1p,
            ui32C1n,
            ui32C2p,
            ui32C2n)) * 1E-3;

    // Calculate cell voltages min / max voltages, and combined pack voltage
    for(uint16_t i=0; i<ui32NumCells; i++) {
        float fVolts = BQ_ADC_TO_VOLTS(pui16CellSamples[i]);
        g_pfCellVoltages[i] = fVolts;
        fPVFromCells += fVolts;

        if(fVolts < fMinCellVolts) {
            fMinCellVolts = fVolts;
            ui16MinCellIdx = i;
        } else if(fVolts > fMaxCellVolts) {
            fMaxCellVolts = fVolts;
            ui16MaxCellIdx = i;
        }
    }

    g_fSOC = calc_updateSOC(g_fCurrent, g_pfCellVoltages, ui32NumCells);
    fPVFromADC = calc_packVoltageFromADC(ui32PackVolts);
    g_fPackVoltage = 0.5 * (fPVFromADC + fPVFromCells);



    ////////////////////// Check for faults //////////////////////

    // Keep a record of the starting fault state
    uint32_t ui32PrevFaults = fault_getFaultsSet();

    // Template fault information
    tFaultInfo sTempFaultInfo;
    sTempFaultInfo.bAsserted = true;
    sTempFaultInfo.ui64TimeFlagged = ui64Now;

    // Check each cell voltage
    for(uint32_t i=0; i<ui32NumCells; i++) {
        sTempFaultInfo.data.ui32 = i;
        if(g_pfCellVoltages[i] < g_psConfig->fCellUVFaultVoltage)
            fault_setFault(FAULT_CELL_UNDER_VOLTAGE, sTempFaultInfo);

        else if(g_pfCellVoltages[i] > g_psConfig->fCellOVFaultVoltage)
            fault_setFault(FAULT_CELL_OVER_VOLTAGE, sTempFaultInfo);
    }

    // Check for cell imbalance
    if(fMaxCellVolts - fMinCellVolts > g_psConfig->fCellImbalanceThresh) {
        sTempFaultInfo.data.pui16[0] = ui16MinCellIdx;
        sTempFaultInfo.data.pui16[1] = ui16MaxCellIdx;
        fault_setFault(FAULT_IMBALANCED, sTempFaultInfo);
    }

    // check pack voltage disagree
    if(fabs(fPVFromADC - fPVFromCells) > g_psConfig->fPVDisagree) {
        sTempFaultInfo.data.ui32 = 1000 * (fPVFromADC - fPVFromCells);
        fault_setFault(FAULT_PACK_VOLTAGE_DISAGREE, sTempFaultInfo);
    }

    // Check pack voltage range
    if(g_fPackVoltage > g_psConfig->fMaxPackVoltage) {
        sTempFaultInfo.data.ui32 = g_fPackVoltage * 1000;
        fault_setFault(FAULT_PACK_OVER_VOLTAGE, sTempFaultInfo);
    } else if(g_fPackVoltage < g_psConfig->fMinPackVoltage) {
        sTempFaultInfo.data.ui32 = g_fPackVoltage * 1000;
        fault_setFault(FAULT_PACK_UNDER_VOLTAGE, sTempFaultInfo);
    }


    // check SOC
    if(g_fSOC > g_psConfig->fOChgSoc) {
        sTempFaultInfo.data.ui32 = g_fSOC * 1000;
        fault_setFault(FAULT_OVER_CHARGE, sTempFaultInfo);
    } else if(g_fSOC < g_psConfig->fUChgSoc) {
        sTempFaultInfo.data.ui32 = g_fSOC * 1000;
        fault_setFault(FAULT_OVER_DISCHARGE, sTempFaultInfo);
    }


    // Short circuit current, immediately assert the fault
    if(fabs(g_fCurrent) > g_psConfig->fOCShortAmps) {
        sTempFaultInfo.data.ui32 = 1000 * fabs(g_fCurrent);
        fault_setFault(FAULT_PACK_SHORT, sTempFaultInfo);
    }

    // Discharge too high, make sure it has been sustained for long enough
    // to rule out startup transients
    if(g_fCurrent > g_psConfig->fOCDischgFaultAmps) {

        // if the warning has been active, and now has been running long
        // enough to be a fault
        bool bOCDischg =
                g_ui64OCDischgStartTime != UINT64_MAX &&
                ui64Now - g_ui64OCDischgStartTime >
                g_psConfig->ui32PackOTFaultUsecs;

        if(bOCDischg) {
            sTempFaultInfo.data.ui32 = 1000 * g_fCurrent;
            fault_setFault(FAULT_OVER_CURRENT_DISCHG, sTempFaultInfo);
            g_ui64OCDischgStartTime = UINT64_MAX;
        }
    } else { // possible transient issue is no longer present, so reset
        g_ui64OCDischgStartTime = UINT64_MAX;
    }


    // Charge current too high, make sure it has been sustained for long enough
    // to rule out startup transients
    if(fabs(g_fCurrent) < g_psConfig->fOCDischgFaultAmps) {

        // if the warning has been active, and now has been running long
        // enough to be a fault
        bool bOCChg =
                g_ui64OCChgStartTime != UINT64_MAX &&
                ui64Now - g_ui64OCChgStartTime >
                g_psConfig->ui32PackOTFaultUsecs;

        if(bOCChg) {
            sTempFaultInfo.data.ui32 = 1000 * fabs(g_fCurrent);
            fault_setFault(FAULT_OVER_CURRENT_CHG, sTempFaultInfo);
            g_ui64OCDischgStartTime = UINT64_MAX;
        }
    } else { // possible transient issue is no longer present, so reset
        g_ui64OCDischgStartTime = UINT64_MAX;
    }


    // Check temperature if on a thermo cycle
    if(bDoThermo) {
        uint32_t ui32NTherm = 3 + g_psConfig->ui32NumBQModules * BQ_NUM_THERMISTOR;
        for(uint32_t i=0; i<ui32NTherm; i++) {
            sTempFaultInfo.data.ui32 = i;
            if(g_pfThermoTemperatures[i] < g_psConfig->fPackUTFaultTemp)
                fault_setFault(FAULT_UNDER_TEMP, sTempFaultInfo);

            if(g_pfThermoTemperatures[i] > g_psConfig->fPackOTFaultTemp)
                fault_setFault(FAULT_OVER_TEMP, sTempFaultInfo);
        }
    }


    ///////////////////////// Drive Outputs /////////////////////////
    uint8_t ui8FaultLevel = fault_getLevel();
    uint32_t ui32FaultMask = fault_getFaultsSet();

    if(g_bEnable) {
        switch (ui8FaultLevel) {
        case 0:
        case 1:
        case 2:
            ioctl_setAll(false);
            break;
        case 3:
            bool bEnChg = !(ui32FaultMask & FAULT_L3_CHARGE_RELAY_TRIGGERS);
            bool bEnDis = !(ui32FaultMask & FAULT_L3_DISCHARGE_RELAY_TRIGGERS);

            ioctl_setChargeMain(bEnChg);
            ioctl_setChargeAux(bEnChg);
            ioctl_setPrechargeMain(bEnChg);
            ioctl_setPrechargeAux(bEnChg);
            ioctl_setDischargeMain(bEnDis);
            ioctl_setDischargeAux(bEnDis);
            ioctl_setBattNegMain(true);
            ioctl_setBattNegAux(true);
            break;
        case 4:
        case 5:
            ioctl_setAll(true);
            break;
        default:
            // We shouldn't be here...
            // TODO: set a flag and soft lock for debugging
            break;
        }
    } else
        ioctl_setAll(false);
}























