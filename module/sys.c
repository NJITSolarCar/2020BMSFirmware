/*
 * sys.c
 *
 *  Created on: Jan 15, 2019
 *      Author: Duemmer
 */

#include <stdbool.h>
#include <stdint.h>

#include <driverlib/timer.h>
#include <driverlib/interrupt.h>
#include <driverlib/adc.h>

#include "sys.h"
#include "iocontrol.h"
#include "flag.h"
#include "fault.h"
#include "bq76.h"
#include "pinconfig.h"
#include "config.h"
#include "util.h"
#include "calculation.h"


void system_selfTest() {
    // The methods and scope of self testing have not yet been determined.
    // For now, do nothing here, as true self tests will be implemented
    // at a future time.

    return;
}




void system_initISRs() {
    // Current Sense
    IntRegister(PINCFG_VEC_CURRSEN, ioctl_isr_currentSense);
    IntPrioritySet(PINCFG_VEC_CURRSEN, SYSTEM_PRI_CURRSEN);
    IntEnable(PINCFG_VEC_CURRSEN);

    // PV Read
    IntRegister(PINCFG_VEC_PV_READ, ioctl_isr_pvRead);
    IntPrioritySet(PINCFG_VEC_PV_READ, SYSTEM_PRI_PV_READ);
    IntEnable(PINCFG_VEC_PV_READ);

    // MCU Thermo Read
    IntRegister(PINCFG_VEC_MCU_THERM, ioctl_isr_mcuTherm);
    IntPrioritySet(PINCFG_VEC_MCU_THERM, SYSTEM_PRI_MCU_THERM);
    IntEnable(PINCFG_VEC_MCU_THERM);

    // BQ Parse
    IntRegister(PINCFG_VEC_BQPARSE, system_isr_bqParse);
    IntPrioritySet(PINCFG_VEC_BQPARSE, SYSTEM_PRI_BQPARSE);
    IntEnable(PINCFG_VEC_BQPARSE);

    // BQ Sample timer
    IntRegister(PINCFG_VEC_BQ_SAMPLE_TIMER, system_isr_bqTimer);
    IntPrioritySet(PINCFG_VEC_BQ_SAMPLE_TIMER, SYSTEM_PRI_BQ_SAMPLE_TIMER);
    IntEnable(PINCFG_VEC_BQ_SAMPLE_TIMER);
}




void system_startSampling() {

    // Load up the timers
    TimerLoadSet(PINCFG_TIMER_PV, PINCFG_TIMER_PART_PV, SYSTEM_PERIOD_PV);
    TimerMatchSet(PINCFG_TIMER_PV, PINCFG_TIMER_PART_PV, SYSTEM_PERIOD_PV);

    TimerLoadSet(PINCFG_TIMER_MCU_THERM, PINCFG_TIMER_PART_MCU_THERM, SYSTEM_PERIOD_MCU_THERM);
    TimerMatchSet(PINCFG_TIMER_MCU_THERM, PINCFG_TIMER_PART_MCU_THERM, SYSTEM_PERIOD_MCU_THERM);

    TimerLoadSet(PINCFG_TIMER_BQ_SAMPLE, PINCFG_TIMER_PART_BQ_SAMPLE, SYSTEM_PERIOD_BQ_SAMPLE);
    TimerMatchSet(PINCFG_TIMER_BQ_SAMPLE, PINCFG_TIMER_PART_BQ_SAMPLE, SYSTEM_PERIOD_BQ_SAMPLE);

    TimerLoadSet(PINCFG_TIMER_CURRSEN, PINCFG_TIMER_PART_CURRSEN, SYSTEM_PERIOD_CURRSEN);
    TimerMatchSet(PINCFG_TIMER_CURRSEN, PINCFG_TIMER_PART_CURRSEN, SYSTEM_PERIOD_CURRSEN);

    // Start them running
    TimerEnable(PINCFG_TIMER_PV, PINCFG_TIMER_PART_PV);
    TimerEnable(PINCFG_TIMER_MCU_THERM, PINCFG_TIMER_PART_MCU_THERM);
    TimerEnable(PINCFG_TIMER_BQ_SAMPLE, PINCFG_TIMER_PART_BQ_SAMPLE);
    TimerEnable(PINCFG_TIMER_CURRSEN, PINCFG_TIMER_PART_CURRSEN);
}



//////////////////////// ISR Utilities /////////////////////////
static inline void system_bqParseCells(
        uint32_t ppui32Samples[][BQ_MAX_SAMPLE],
        bool *pbDataValid,
        uint32_t *pui32nSamples,
        uint32_t ui32nModules) {
    float fTmp;

    // Flag for any potential faults
    tFaultInfo sFlt;
    sFlt.ui64TimeFlagged = util_usecs();

    // Min / max cell voltage / index
    float fMinV, fMaxV;
    uint32_t ui32MinCell, ui32MaxCell;
    bool bHasMin = false;
    bool bHasMax = false;

    // Parse voltages, find the min and max
    uint32_t ui32Offset = 0;
    uint32_t ui32CellNum;
    bool bAllDataValid = true;
    for(int m=0; m<ui32nModules; m++) {
        if(pbDataValid[m]) { // Don't bother copying or running min/max on bad data
            for(int s=0; s<pui32nSamples[m]; s++) {
                ui32CellNum = s+ui32Offset;
                fTmp = BQ_ADC_TO_VOLTS(ppui32Samples[m][s]);
                g_pfCellVoltages[ui32CellNum] = fTmp;

                if(fTmp < fMinV || !bHasMin) {
                    fMinV = fTmp;
                    ui32MinCell = ui32CellNum;
                    bHasMin = true;

                } else if(fTmp > fMaxV || !bHasMax) {
                    fMaxV = fTmp;
                    ui32MaxCell = ui32CellNum;
                    bHasMax = true;
                }
            }
        } else
            bAllDataValid = false;
        ui32Offset += pui32nSamples[m];
    }

    // *All* of the data has to be proper before this flag can be reliably set
    g_pfCellVoltagesValid = bAllDataValid;

    // Check imbalance. Don't need to check UV / OV, the BQ
    // will take care of that and notify us separately
    if(fMaxV - fMinV > g_psConfig->fCellImbalanceThresh) {
        sFlt.data.pui16[0] = ui32MinCell;
        sFlt.data.pui16[1] = ui32MaxCell;

        fault_setFault(FAULT_IMBALANCED, sFlt);
    }
}



static inline void system_bqParseThermo(
        uint32_t ppui32Samples[][BQ_MAX_SAMPLE],
        bool *pbDataValid,
        uint32_t *pui32nSamples,
        uint32_t ui32nModules,
        bool bIsTherm2) {
    float fTmp, fVoltFrac;

    // Flag for any potential faults
    tFaultInfo sFlt;
    sFlt.ui64TimeFlagged = util_usecs();

    // Min and max temperature / index
    float fMinT, fMaxT;
    int32_t i32MinIdx = -1, i32MaxIdx = -1;
    bool bHasMin = false, bHasMax = false;

    // True if all modules' data is valid
    bool bAllDataValid = true;

    // Parse temperatures, find the min and max, and store to the right place.
    // s is sample number, m is module number
    uint32_t ui32Idx;
    for(int m=0; m<ui32nModules; m++) {
        if(pbDataValid[m]) { // Don't bother copying or running min/max on bad data
            for(int s=0; s<pui32nSamples[m]; s++) {

                // Determine the dest. index in the final buffer
                ui32Idx = SYSTEM_NUM_MCU_THERMO + s +
                        BQ_NUM_THERMISTOR * m;

                if(bIsTherm2)
                    ui32Idx += BQ_NUM_THERMISTOR/2;

                // Only check for faults on and record the
                // thermistor is populated
                if((1 << s) & g_psConfig->bqCals[m].ui16ThermoMask) {
                    fVoltFrac = BQ_ADC_TO_VOLT_FRAC(ppui32Samples[m][s]);
                    fTmp = calc_thermistorTemp(fVoltFrac, &g_psConfig->sThermoParams);

                    g_pfThermTemps[ui32Idx] = fTmp;

                    if(fTmp < fMinT || !bHasMin) {
                        fMinT = fTmp;
                        i32MinIdx = ui32Idx;
                        bHasMin = false;

                    } else if(fTmp > fMaxT || !bHasMax) {
                        fMaxT = fTmp;
                        i32MaxIdx = ui32Idx;
                        bHasMax = false;
                    }
                }
            }
        } else
            bAllDataValid = false;
    }

    if(bIsTherm2)
        g_pfTherm2Valid = bAllDataValid;
    else
        g_pfTherm1Valid = bAllDataValid;

    // Check temperature thresholds
    if(fMaxT > g_psConfig->fPackOTFaultTemp) {
        sFlt.data.ui32 = i32MaxIdx;
        fault_setFault(FAULT_OVER_TEMP, sFlt);
    }

    if(fMinT < g_psConfig->fPackUTFaultTemp) {
        sFlt.data.ui32 = i32MinIdx;
        fault_setFault(FAULT_UNDER_TEMP, sFlt);
    }
}



//////////////////////// ISRs /////////////////////////

void system_isr_bqParse() {

    // Temporary sample buffer
    uint32_t ppui32Samples[BQ_MAX_NUM_MODULE][BQ_MAX_SAMPLE];
    uint32_t pui32nSamples[BQ_MAX_NUM_MODULE];
    bool pbDataValid[BQ_MAX_NUM_MODULE];
    uint32_t ui32nModules;

    // Flag for any potential faults
    tFaultInfo sFlt;
    sFlt.ui64TimeFlagged = util_usecs();

    IntPendClear(PINCFG_VEC_BQPARSE);

    if(bq76_readBufToVoltages(ppui32Samples, pbDataValid, pui32nSamples, &ui32nModules)) {
        switch(bq76_samplingMode()) {
        case CELL: {
            system_bqParseCells(ppui32Samples, pbDataValid, pui32nSamples, ui32nModules);
            break;
        }
        case THERM1: {
            system_bqParseThermo(ppui32Samples, pbDataValid, pui32nSamples, ui32nModules, false);
            break;
        }
        case THERM2: {
            system_bqParseThermo(ppui32Samples, pbDataValid, pui32nSamples, ui32nModules, true);
            break;
        }
        default: {
            // TODO: Throw an error
        }
        }
    } else {
        // TODO: Throw an error
    }
}





void system_isr_bqTimer() {
    static uint32_t ui32State = 0;
    bool bStarted = false;

    IntPendClear(PINCFG_VEC_BQ_SAMPLE_TIMER);

    // Do the first half of thermo samples
    if(ui32State % SYSTEM_BQ_THERMO_PART == 0)
        bStarted = bq76_startThermoSample(false);

    // we just did the first half last time, now onto the second half
    else if(ui32State+1 % SYSTEM_BQ_THERMO_PART == 0)
        bStarted = bq76_startThermoSample(true);

    // Default to voltage sample
    else
        bStarted = bq76_StartCellVoltageSample();

    // It's possible reading a sample took too long and we aren't ready for
    // the next one. In that case, don't advance the counter
    if(bStarted)
        ui32State++;
}





