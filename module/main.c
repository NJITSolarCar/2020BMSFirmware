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

#define SAMPLE_BUF_SIZE         64


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
                system_parseThermo1();
                break;
            case THERM2_SAMPLE:
                bq76_startThermoSample(true);
                break;
            case THERM2_READ:
                system_parseThermo2();
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
