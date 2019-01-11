/*
 * system.c
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
#include "watchdog.h"

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




/**
 * Performs a detailed fault check of the BQ76 stack, and diagnoses any faults it
 * detects. Will read all the registers it needs, and will update the global fault
 * status of the BMS. Returns true if sampling from the BMS can continue.
 */
inline bool system_handleBQFaults(uint64_t ui64Now) {
    uint16_t ui16AggregateFaults = 0;
    bool bReadSampled = true;

    tFaultInfo sTempFaultInfo;
    sTempFaultInfo.bAsserted = true;
    sTempFaultInfo.ui64TimeFlagged = ui64Now;

    uint16_t pui16FaultSum[g_psConfig->ui32NumBQModules];
    uint8_t ui8NumModulesRead = bq76_readFaultSum(pui16FaultSum);

    for(uint32_t i=0; i<ui8NumModulesRead; i++)
        ui16AggregateFaults |= pui16FaultSum[i];

    // Cell under voltage
    if(ui16AggregateFaults & (BQ_UV_FAULT_SUM | BQ_CMPUV_FAULT_SUM)) {
        sTempFaultInfo.data.ui32 = 0;
        bq76_getCellsUV(sTempFaultInfo.data.pui16, sizeof(sTempFaultInfo.data)/2);
        fault_setFault(FAULT_CELL_UNDER_VOLTAGE, sTempFaultInfo);
    }

    // Cell over voltage
    if(ui16AggregateFaults & (BQ_OV_FAULT_SUM | BQ_CMPOV_FAULT_SUM)) {
        sTempFaultInfo.data.ui32 = 0;
        bq76_getCellsOV(sTempFaultInfo.data.pui16, sizeof(sTempFaultInfo.data)/2);
        fault_setFault(FAULT_CELL_OVER_VOLTAGE, sTempFaultInfo);
    }

    // Communications fault
    if(ui16AggregateFaults & BQ_COMM_FAULT_SUM) {
        sTempFaultInfo.data.ui32 = 0;
        bq76_getComFaults(sTempFaultInfo.data.pui16, sizeof(sTempFaultInfo.data)/2);
        fault_setFault(FAULT_BQ_COM, sTempFaultInfo);
    }

    // Chip fault
    if(ui16AggregateFaults & BQ_COMM_FAULT_SUM) {
        sTempFaultInfo.data.ui32 = 0;
        bq76_getChipFaults(sTempFaultInfo.data.pui16, sizeof(sTempFaultInfo.data)/2);
        fault_setFault(FAULT_BQ_CHIP_FAULT, sTempFaultInfo);
        bReadSampled = false; // the BQ is broken, samples could be erroneous
    }

    return bReadSampled;
}








inline void system_bqDataLoop() {

    static uint32_t ui32CallCount = 0;
    static bool bCanSample = true;
    uint64_t ui64Now = util_usecs();

    // Diagnose any faults
    if(bq76_faultPinActive())
        bCanSample = system_handleBQFaults(ui64Now);

    if(bCanSample) {

        // Build a sufficiently sized buffer for whatever may be sampled
        uint32_t ui32SampleBufSize = util_ui32Max(
                config_totalNumcells(g_psConfig),
                BQ_NUM_THERMISTOR * 2 * g_psConfig->ui32NumBQModules);
        uint16_t pui16SampleBuf[ui32SampleBufSize];
        uint32_t ui32nSmaples = bq76_readSampledVoltages(pui16SampleBuf, ui32SampleBufSize);

        // Do a thermo 1 sample. Need to copy and move chunks to allow for the
        // proper ordering of the thermistor buffer, as the muxing of the
        // thermistor lines means we can't sample all thermistors on a board at
        // once. Instead, half are sampled at once, then the results are spaced out
        // in the temperature buffer. If there are n total thermistors on each
        // BQ module, then the buffer would be divided into segments of length n,
        // one for each BQ module. When copying thermo 1 data to that buffer, each
        // module's n/2 thermistors would be copied to the first n/2 slots of the
        // module's part of the buffer, and the thermo 2 samples would fill the
        // other half, like so:
        // | mod0therm1 | mod0therm2 | mod1therm1 | mod1therm2 | ...
        if(ui32CallCount % SYSTEM_BQ_THERMO_PART == 0) {
            uint32_t ui32Idx;
            float fVoltFrac;

            // for each module...
            for(uint32_t i=0; i<g_psConfig->ui32NumBQModules; i++) {

                // For each sample of that module...
                for(uint32_t j=0; j<(BQ_NUM_THERMISTOR/2); j++) {

                    // Copy to the buffer
                    ui32Idx = BQ_NUM_THERMISTOR*i+j+SYSTEM_NUM_MCU_THERMO;
                    fVoltFrac = BQ_ADC_TO_VOLT_FRAC(j+(BQ_NUM_THERMISTOR/2)*i);
                    g_pfThermoTemperatures[ui32Idx] =
                            calc_thermistorTemp(fVoltFrac, &g_psConfig->sThermoParams);

                }
            }

            // Sample thermo 2 next
            bq76_startThermoSample(true);
        }

        // Thermo 2 sample. Similar to thermo 1, but stores to the latter half
        // of each module's buffer space
        else if(ui32CallCount+1 % SYSTEM_BQ_THERMO_PART == 0) {
            uint32_t ui32Idx;
            float fVoltFrac;

            // for each module...
            for(uint32_t i=0; i<g_psConfig->ui32NumBQModules; i++) {

                // For each sample of that module...
                for(uint32_t j=0; j<(BQ_NUM_THERMISTOR/2); j++) {

                    // Copy to the buffer
                    ui32Idx = (BQ_NUM_THERMISTOR/2) +
                            BQ_NUM_THERMISTOR*i +
                            j+SYSTEM_NUM_MCU_THERMO;

                    fVoltFrac = BQ_ADC_TO_VOLT_FRAC(j+(BQ_NUM_THERMISTOR/2)*i);
                    g_pfThermoTemperatures[ui32Idx] =
                            calc_thermistorTemp(fVoltFrac, &g_psConfig->sThermoParams);
                }
            }

            // Sample cell voltage next
            bq76_StartCellVoltageSample();
        }

        // Cell voltage sample
        else {
            for(uint32_t i=0; i<ui32nSmaples; i++)
                g_pfCellVoltages[i] = BQ_ADC_TO_VOLTS(pui16SampleBuf[i]);

            // Sample the correct thing next
            if(ui32CallCount+1 % SYSTEM_BQ_THERMO_PART == 0) // thermo 1 in next
                bq76_startThermoSample(false);
            bq76_StartCellVoltageSample();
        }
    }
    ui32CallCount++;
}







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

    // Pack voltage from different ways
    float fPVFromCells = 0.0;
    float fPVFromADC;

    // maximum and minimum cell voltages
    float fMinCellVolts = 10.0;
    float fMaxCellVolts = -1.0;
    uint32_t ui16MinCellIdx = 0;
    uint32_t ui32MaxCellIdx = 0;


    // Get timing intervals
    uint64_t ui64Now = util_usecs();


    ////////////////////// Acquire sampled data //////////////////////
    ioctl_sampledCurrent(&ui32C1p, &ui32C1n, &ui32C2p, &ui32C2n);
    ioctl_sampledAux(&ui32PackVolts, &ui32Therm1, &ui32Therm2, &ui32Therm3);

    bool bDoThermo = !(ui64NumCalls % SYSTEM_THERMO_PART);
    if(bDoThermo) {
        // First 3 thermistors are on the main board, with a 12 bit ADC
        g_pfThermoTemperatures[0] = calc_thermistorTemp(
                ((float) ui32Therm1) / 4096.0, &g_psConfig->sThermoParams);
        g_pfThermoTemperatures[1] = calc_thermistorTemp(
                ((float) ui32Therm2) / 4096.0, &g_psConfig->sThermoParams);
        g_pfThermoTemperatures[2] = calc_thermistorTemp(
                ((float) ui32Therm3) / 4096.0, &g_psConfig->sThermoParams);
    }


    ////////////////////// Do calculations //////////////////////
    // Determine current
    g_fCurrent = ((float)calc_adcToMilliAmps(
            ui32C1p,
            ui32C1n,
            ui32C2p,
            ui32C2n)) * 1E-3;

    // Calculate cell voltages min / max voltages, and combined pack voltage
    for(uint32_t i=0; i<ui32NumCells; i++) {
        fPVFromCells += g_pfCellVoltages[i];

        if(g_pfCellVoltages[i] < fMinCellVolts) {
            fMinCellVolts = g_pfCellVoltages[i];
            ui16MinCellIdx = i;
        } else if(g_pfCellVoltages[i] > fMaxCellVolts) {
            fMaxCellVolts = g_pfCellVoltages[i];
            ui32MaxCellIdx = i;
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
        sTempFaultInfo.data.pui16[1] = ui32MaxCellIdx;
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

    // Current faults, set the template fault info to just read abs(milliamps)
    sTempFaultInfo.data.ui32 = 1000 * fabs(g_fCurrent);

    // Short circuit current, immediately assert the fault
    if(fabs(g_fCurrent) > g_psConfig->fOCShortAmps)
        fault_setFault(FAULT_PACK_SHORT, sTempFaultInfo);

    // Discharge too high, make sure it has been sustained for long enough
    // to rule out startup transients
    if(g_fCurrent > g_psConfig->fOCDischgFaultAmps)
        fault_setFault(FAULT_TRANSIENT_DISCHG_OC, sTempFaultInfo);

    else // possible transient issue is no longer present, so reset
        fault_clearFault(FAULT_TRANSIENT_DISCHG_OC);


    // Charge current too high, make sure it has been sustained for long enough
    // to rule out startup transients
    if(g_fCurrent < -g_psConfig->fOCDischgFaultAmps)
        fault_setFault(FAULT_TRANSIENT_CHG_OC, sTempFaultInfo);

    else // possible transient issue is no longer present, so reset
        fault_clearFault(FAULT_TRANSIENT_CHG_OC);

    // Check if the transient faults have been active long enough to upgrade
    uint64_t ui64TimeTrans;
    bool bChgOC =
            fault_timeAsserted(FAULT_TRANSIENT_CHG_OC, &ui64TimeTrans) &&
            ui64Now - ui64TimeTrans > g_psConfig->ui32OCShortUsecs;

    bool bDischgOC =
            fault_timeAsserted(FAULT_TRANSIENT_DISCHG_OC, &ui64TimeTrans) &&
            ui64Now - ui64TimeTrans > g_psConfig->ui32OCShortUsecs;

    if(bChgOC)
        fault_setFault(FAULT_OVER_CURRENT_CHG, sTempFaultInfo);

    if(bDischgOC)
            fault_setFault(FAULT_OVER_CURRENT_DISCHG, sTempFaultInfo);



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
        case 3: {
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
        }
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


    ///////////////////////// Prepare for Next Cycle /////////////////////////
    watchdog_feed();
}







/**
 * ============================================================================
 * ============================================================================
 */

void system_initialize()
{

}


/**
 * Computes the next state of the system state machine
 */
tSystemState system_nextState(tSystemState eNow, bool *pbDataValid) {
    // If set, is in the startup sequence vs. normal operation
    static bool bInStartup;
    static uint32_t ui32CellSampleCount = 0;
    static bool bThermo2;

    // revert to the start sequence if needed
    if(eNow == INITIALIZE || eNow == TEST || eNow == RESET)
        bInStartup = true;

    // just walk through the sequence
    if(bInStartup) {
        switch(eNow) {
            case INITIALIZE:    return RESET;
            case RESET:         return FAULT_CLEAR;
            case FAULT_CLEAR:   return TEST;
            case TEST:          return AUX_SAMPLE;
            case AUX_SAMPLE:    return CELL_SAMPLE;
            case CELL_SAMPLE:   return CURRENT_READ;
            case CURRENT_READ:  return AUX_READ;
            case AUX_READ:      return CELL_READ;
            case CELL_READ:     return VOLT_FAULTS;
            case VOLT_FAULTS:   return THERM1_SAMPLE;
            case THERM1_SAMPLE: return THERM1_READ;
            case THERM1_READ:   return THERM2_SAMPLE;
            case THERM2_SAMPLE: return THERM2_READ;
            case THERM2_READ:   return BRANCH_PONT;
            case BRANCH_PONT:
                bInStartup = false;
                *pbDataValid = true;
                return CELL_SAMPLE;
        }
    } else { // main operations state loop
        ui32CellSampleCount++;
        switch(eNow) {
            case CELL_SAMPLE:
                ui32CellSampleCount++;
                return CURRENT_READ;

            // need a branch as we could be here in either the cell, thermo 1,
            // or thermo 2 sample cycle. if the sample count != 0,
            // we are in a cell cycle
            case CURRENT_READ:
                if(bq76_isSampling())
                    return CURRENT_READ;
                else if(ui32CellSampleCount)    return CELL_READ;
                else if(bThermo2)               return THERM2_READ;
                else                            return THERM1_READ;

            case CELL_READ:     return VOLT_FAULTS;
            case VOLT_FAULTS:   return BRANCH_PONT;

            case BRANCH_PONT:
                if(ui32CellSampleCount > SYSTEM_THERMO_PART) {
                    ui32CellSampleCount = 0;
                    return THERM1_SAMPLE;
                }
                else
                    return CELL_SAMPLE;

            case THERM1_SAMPLE:
                bThermo2 = false;
                return AUX_SAMPLE;

            case AUX_SAMPLE:    return AUX_READ;
            case AUX_READ:
                if(bq76_isSampling())
                    return CURRENT_READ;
                else
                    return THERM1_READ;

            case THERM1_READ:   return THERM2_SAMPLE;
            case THERM2_SAMPLE:
                bThermo2 = true;
                return CURRENT_READ;

            case THERM2_READ:   return THERM_FAULTS;
            case THERM_FAULTS:  return BRANCH_PONT;

        }
    }

    // Shouldn't get here, if we do, it's a state machine error
    return INVALID;
}



void system_parseThermo2() {
    uint32_t ui32BufSize = g_psConfig->ui32NumBQModules*(BQ_NUM_THERMISTOR/2);
    uint16_t pui16Buf[ui32BufSize];

    float fTmpVolt;
    if(bq76_readSampledVoltages(pui16Buf, ui32BufSize)) {

        // For each module's data
        for(uint32_t i=0; i<g_psConfig->ui32NumBQModules; i++) {

            // for each thermistor sample in that module
            for(uint32_t j=0; j<BQ_NUM_THERMISTOR/2; j++) {

                // determine the voltage of that sample
                fTmpVolt = BQ_ADC_TO_VOLT_FRAC(pui16Buf[j + i*BQ_NUM_THERMISTOR/2]);

                // convert that volt fraction to a temperature, and copy to the
                // right spot in the temperature buffer
                g_pfThermoTemperatures[j + i*BQ_NUM_THERMISTOR + BQ_NUM_THERMISTOR/2] =
                        calc_thermistorTemp(fTmpVolt, &g_psConfig->sThermoParams);
            }
        }
    } else {
        // TODO: throw a comm fault, maybe something more
        tFaultInfo sInfo;
        sInfo.bAsserted = true;
        sInfo.ui64TimeFlagged = util_usecs();
        sInfo.data.ui32 = 0;
        fault_setFault(FAULT_BQ_COM, sInfo);
    }
}



void system_parseThermo1() {
    uint32_t ui32BufSize = g_psConfig->ui32NumBQModules*(BQ_NUM_THERMISTOR/2);
    uint16_t pui16Buf[ui32BufSize];

    float fTmpVolt;
    if(bq76_readSampledVoltages(pui16Buf, ui32BufSize)) {

        // For each module's data
        for(uint32_t i=0; i<g_psConfig->ui32NumBQModules; i++) {

            // for each thermistor sample in that module
            for(uint32_t j=0; j<BQ_NUM_THERMISTOR/2; j++) {

                // determine the voltage of that sample
                fTmpVolt = BQ_ADC_TO_VOLT_FRAC(pui16Buf[j + i*BQ_NUM_THERMISTOR/2]);

                // convert that volt fraction to a temperature, and copy to the
                // right spot in the temperature buffer
                g_pfThermoTemperatures[j + i*BQ_NUM_THERMISTOR] =
                        calc_thermistorTemp(fTmpVolt, &g_psConfig->sThermoParams);
            }
        }
    } else {
        // TODO: throw a comm fault, maybe something more
        tFaultInfo sInfo;
        sInfo.bAsserted = true;
        sInfo.ui64TimeFlagged = util_usecs();
        sInfo.data.ui32 = 0;
        fault_setFault(FAULT_BQ_COM, sInfo);
    }
}



void system_parseCells() {
    uint32_t ui32BufSize = config_totalNumcells(g_psConfig);
    uint16_t pui16Buf[ui32BufSize];

    if(bq76_readSampledVoltages(pui16Buf, ui32BufSize)) {
        for(uint32_t i=0; i<ui32BufSize; i++)
            g_pfCellVoltages[i] = BQ_ADC_TO_VOLTS(pui16Buf[i]);
    } else {
        // TODO: throw a comm fault, maybe something more
        tFaultInfo sInfo;
        sInfo.bAsserted = true;
        sInfo.ui64TimeFlagged = util_usecs();
        sInfo.data.ui32 = 0;
        fault_setFault(FAULT_BQ_COM, sInfo);
    }
}



void system_parseAux() {
    uint32_t pv, t1, t2, t3;

    // wait for the sample to finish
    while(!ioctl_sampledAux(&pv, &t1, &t2, &t3))
        ;

    g_fPackVoltage = calc_packVoltageFromADC(pv);

    g_pfThermoTemperatures[0] = calc_thermistorTemp(
            IOCTL_ADC_TO_VOLT_FRAC(t1),
            &g_psConfig->sThermoParams);

    g_pfThermoTemperatures[1] = calc_thermistorTemp(
            IOCTL_ADC_TO_VOLT_FRAC(t2),
            &g_psConfig->sThermoParams);

    g_pfThermoTemperatures[2] = calc_thermistorTemp(
            IOCTL_ADC_TO_VOLT_FRAC(t3),
            &g_psConfig->sThermoParams);
}




void system_determineOutputs() {
    uint8_t ui8FaultLevel = fault_getLevel();
    uint32_t ui32FaultMask = fault_getFaultsSet();

    if(g_bEnable) {
        switch (ui8FaultLevel) {
        case 0:
        case 1:
        case 2:
            ioctl_setAll(false);
            break;
        case 3: {
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
        }
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



void system_voltFaults() {
    float fMinV = 10.0, fMaxV = -1.0;
    float fPVFromCells = 0;

    uint32_t ui32MaxIdx, ui32MinIdx;
    uint32_t ui32NumCells = config_totalNumcells(g_psConfig);

    tFaultInfo sTempFaultInfo;
    sTempFaultInfo.bAsserted = true;
    sTempFaultInfo.ui64TimeFlagged = util_usecs();

    float fVolts;
    for(uint32_t i=0; i<ui32NumCells; i++) {
        fVolts = g_pfCellVoltages[i];
        fPVFromCells += fVolts;

        // Get min / max voltages
        if(fVolts < fMinV) {
            fMinV = fVolts;
            ui32MinIdx = i;
        } else if(fVolts > fMaxV) {
            fMaxV = fVolts;
            ui32MaxIdx = i;
        }

        // Throw cell uv / ov faults if needed
        sTempFaultInfo.data.ui32 = i;
        if(fVolts < g_psConfig->fCellUVFaultVoltage)
            fault_setFault(FAULT_CELL_UNDER_VOLTAGE, sTempFaultInfo);

        else if(fVolts > g_psConfig->fCellOVFaultVoltage)
            fault_setFault(FAULT_CELL_OVER_VOLTAGE, sTempFaultInfo);
        }

    // Check for imbalance
    if(fMaxV - fMinV > g_psConfig->fCellImbalanceThresh) {
        sTempFaultInfo.data.pui16[0] = ui32MinIdx;
        sTempFaultInfo.data.pui16[1] = ui32MaxIdx;
        fault_setFault(FAULT_IMBALANCED, sTempFaultInfo);
    }

    // Check for disagree
    if(fabs(fPVFromCells - g_fPackVoltage) > g_psConfig->fPVDisagree) {
        sTempFaultInfo.data.ui32 = 1000.0 * fabs(fPVFromCells - g_fPackVoltage);
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
}



void system_currentFaults() {
    // Template fault information
        tFaultInfo sTempFaultInfo;
        sTempFaultInfo.bAsserted = true;
        sTempFaultInfo.ui64TimeFlagged = util_usecs();

        uint64_t ui64Now = util_usecs();

        // Current faults, set the template fault info to just read abs(milliamps)
        sTempFaultInfo.data.ui32 = 1000 * fabs(g_fCurrent);

        // Short circuit current, immediately assert the fault
        if(fabs(g_fCurrent) > g_psConfig->fOCShortAmps)
            fault_setFault(FAULT_PACK_SHORT, sTempFaultInfo);

        // Discharge too high, make sure it has been sustained for long enough
        // to rule out startup transients
        if(g_fCurrent > g_psConfig->fOCDischgFaultAmps)
            fault_setFault(FAULT_TRANSIENT_DISCHG_OC, sTempFaultInfo);

        else // possible transient issue is no longer present, so reset
            fault_clearFault(FAULT_TRANSIENT_DISCHG_OC);


        // Charge current too high, make sure it has been sustained for long enough
        // to rule out startup transients
        if(g_fCurrent < -g_psConfig->fOCDischgFaultAmps)
            fault_setFault(FAULT_TRANSIENT_CHG_OC, sTempFaultInfo);

        else // possible transient issue is no longer present, so reset
            fault_clearFault(FAULT_TRANSIENT_CHG_OC);

        // Check if the transient faults have been active long enough to upgrade
        uint64_t ui64TimeTrans;
        bool bChgOC =
                fault_timeAsserted(FAULT_TRANSIENT_CHG_OC, &ui64TimeTrans) &&
                ui64Now - ui64TimeTrans > g_psConfig->ui32OCShortUsecs;

        bool bDischgOC =
                fault_timeAsserted(FAULT_TRANSIENT_DISCHG_OC, &ui64TimeTrans) &&
                ui64Now - ui64TimeTrans > g_psConfig->ui32OCShortUsecs;

        if(bChgOC)
            fault_setFault(FAULT_OVER_CURRENT_CHG, sTempFaultInfo);

        if(bDischgOC)
            fault_setFault(FAULT_OVER_CURRENT_DISCHG, sTempFaultInfo);
}



void system_thermFaults() {
    uint32_t ui32NTherm = 3 + g_psConfig->ui32NumBQModules * BQ_NUM_THERMISTOR;

    // Template fault information
    tFaultInfo sTempFaultInfo;
    sTempFaultInfo.bAsserted = true;
    sTempFaultInfo.ui64TimeFlagged = util_usecs();

    for(uint32_t i=0; i<ui32NTherm; i++) {
        sTempFaultInfo.data.ui32 = i;
        if(g_pfThermoTemperatures[i] < g_psConfig->fPackUTFaultTemp)
            fault_setFault(FAULT_UNDER_TEMP, sTempFaultInfo);

        if(g_pfThermoTemperatures[i] > g_psConfig->fPackOTFaultTemp)
            fault_setFault(FAULT_OVER_TEMP, sTempFaultInfo);
    }
}











