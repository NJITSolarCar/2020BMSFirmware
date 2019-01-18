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
#include "bq76.h"

#include <stdint.h>
#include <stdbool.h>

#define SAMPLE_BUF_SIZE         64

// how long the watchdog lives, in usec
#define WDT_PERIOD              1000





void bms() {
    uint32_t ui32NumModules = 0;

    // Relay states
    bool bBatNeg, bChg, bDisch;

    // Startup sequence
    flag_init();
    config_load(g_psConfig);
    ioctl_initialize();
    ui32NumModules = bq76_autoAddress();

    if(ui32NumModules != g_psConfig->ui32NumBQModules) {
        // TODO: Assert BQ comm fault, and just lock up the system
        for(;;);
    }

    system_selfTest();
    system_initISRs();
    system_startSampling();
    watchdog_begin(WDT_PERIOD, system_abort);

    // Idle loop, should execute at a (minimum) 1kHz
    for(;;) {

        // Determine SOC
        if(g_fCurrentValid && g_pfCellVoltagesValid) {
            g_fSOC = calc_updateSOC(g_fCurrent, g_pfCellVoltages, config_totalNumcells(g_psConfig));
            g_fSOCValid = true;
        }

        // Drive outputs
        if(g_bEnable && flag_dataValid()) {
            fault_determineRelays(&bBatNeg, &bChg, &bDisch);
            ioctl_setBattNegEn(bBatNeg);
            ioctl_setChargeEn(bChg);
            ioctl_setDischargeEn(bDisch);
        } else
            ioctl_setAll(false);

        watchdog_feed();
    }
}










int main(void)
{
    tSystemState eState = INITIALIZE;

    bool bDataValid = false;

    for(;;) {
        switch(eState) {
        case INITIALIZE:
            system_initialize();
            break;
        case TEST:
            system_self_test();
            break;
        case RESET:
            system_reset();
            break;
        case FAULT_CLEAR:
            fault_clearNonPersistent();
            break;
        case CELL_SAMPLE:
            bq76_StartCellVoltageSample();
            break;
        case CELL_READ:
            system_parseCells();
            break;
        case THERM1_SAMPLE:
            bq76_startThermoSample(false);
            break;
        case THERM1_READ:
            system_parseTherm1();
            break;
        case THERM2_SAMPLE:
            bq76_startThermoSample(true);
            break;
        case THERM2_READ:
            system_parseTherm2();
            break;
        case CURRENT_READ: {
            uint32_t c1p, c1n, c2p, c2n;
            ioctl_sampledCurrent(&c1p, &c1n, &c2p, &c2n);
            g_fCurrent = calc_adcToAmps(c1p, c1n, c2p, c2n);
            system_currentFaults();
            break;
        }
        case AUX_READ: {
            uint32_t pv, t1, t2, t3;
            ioctl_sampledAux(&pv, &t1, &t2, &t3);

            break;
        }
        case AUX_SAMPLE:
            ioctl_startAuxSample();
            break;
        case BRANCH_PONT:
            // TODO: allow external triggering from here
            break;
        case VOLT_FAULTS:
            system_voltFaults();
            break;
        case THERM_FAULTS:
            system_thermFaults();
            break;
        case INVALID:
        default:
            // TODO: Throw a state machine error
            break;
        }

        // Calculate faults and drive outputs if we have good data
        if(bDataValid) {
            system_determineOutputs();
        } else
            ioctl_setAll(false);

        // Determine the next state
        eState = system_nextState(eState, &bDataValid);
        watchdog_feed();
    }
}
