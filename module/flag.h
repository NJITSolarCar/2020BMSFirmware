/*
 * flag.h
 *
 * Provides a reference to all status and informational flags in the
 * system. No functions should be available, just a global information
 * about the system.
 *
 *  Created on: Dec 23, 2018
 *      Author: Duemmer
 */

#ifndef FLAG_H_
#define FLAG_H_

#include "config.h"

extern uint32_t g_ui32Faults;
extern uint32_t g_ui32FaultData;

// Data metrics
extern float g_fCurrent;
extern float g_fSOC;
extern float g_fPackVoltage;
extern float *g_pfCellVoltages;
extern float *g_pfThermoTemperatures;

// warning status start times. Will read UINT64_MAX if inactive
extern uint64_t g_ui64OCDischgStartTime;
extern uint64_t g_ui64OCChgStartTime;

#endif /* FLAG_H_ */
