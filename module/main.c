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
	uint16_t pui16CellSampleBuf[SAMPLE_BUF_SIZE];

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
                bq76_readSampledVoltages(pui16CellSampleBuf, SAMPLE_BUF_SIZE);
                system_parseCells(pui16CellSampleBuf, SAMPLE_BUF_SIZE);
                break;
            case THERM1_SAMPLE:
                bq76_startThermoSample(false);
                break;
            case THERM1_READ:
                bq76_readSampledVoltages(pui16CellSampleBuf, SAMPLE_BUF_SIZE);
                system_parseThermo1(pui16CellSampleBuf, SAMPLE_BUF_SIZE);
                break;
            case THERM2_SAMPLE:
                bq76_startThermoSample(true);
                break;
            case THERM2_READ:
                bq76_readSampledVoltages(pui16CellSampleBuf, SAMPLE_BUF_SIZE);
                system_parseThermo2(pui16CellSampleBuf, SAMPLE_BUF_SIZE);
                break;
            case CURRENT_READ: {
                uint32_t c1p, c1n, c2p, c2n;
                ioctl_sampledCurrent(&c1p, &c1n, &c2p, &c2n);
                g_fCurrent = calc_adcToAmps(c1p, c1n, c2p, c2n);
                break;
            }
            case AUX_READ: {
                uint32_t pv, t1, t2, t3;
                ioctl_sampledAux(&pv, &t1, &t2, &t3);
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
                break;
            }
            case IDLE: // Do nothing
                break;
	    }

	    // Calculate faults and drive outputs if we have good data
	    if(bDataValid) {
	        system_determine_faults();
	        system_determine_outputs();
	    }

	    // Determine the next state
	    eState = system_nextState(eState, &bDataValid);
	    watchdog_feed();
	}
}
