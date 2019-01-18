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

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

extern uint32_t g_ui32Faults;
extern uint32_t g_ui32FaultData;

// Data metrics
extern float g_fCurrent;
extern bool g_fCurrentValid;

extern float g_fSOC;
extern bool g_fSOCValid;

extern float g_fPVFromADC;
extern bool g_fPVFromADCValid;

extern float g_fPVFromSum;
extern bool g_fPVFromSumValid;

extern float *g_pfCellVoltages;
extern bool g_pfCellVoltagesValid;

extern float *g_pfThermTemps;
extern bool g_pfTherm1Valid;
extern bool g_pfTherm2Valid;
extern bool g_pfThermMCUValid;

// Control flags
extern bool g_bEnable;
extern tConf *g_psConfig;

// Utility Functions

// Initializes the flag variables to a safe default state
void flag_init();

// Returns true if all the data flags are valid, false otherwise
bool flag_dataValid();

#endif /* FLAG_H_ */
