/*
 * system.h
 *
 * Contains the core application code and loop. All primary initialization and
 * system control should be performed from here. In addition, the lower level
 * modules should be used for the majority of HAL operations and computations
 *
 *  Created on: Nov 4, 2018
 *      Author: Duemmer
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <stdint.h>
#include <stdbool.h>

#include "types.h"

#include <driverlib/sysctl.h>

// Asserts that the BMS is in a ready state for operation
void sys_init();


/** System Settings */

// Fraction of the time certain samples run. Numbers are "weird" so the calls
// don't intersect as often
#define SYSTEM_THERMO_PART      100
#define SYSTEM_BQ_THERMO_PART   101
#define SYSTEM_BQ_INTERNAL_PART 99973

#define SYSTEM_NUM_MCU_THERMO   3



/**
 * This method will be called periodically during operation. This is the "superloop"
 * Where all of the more advanced calculations, and decisions are made. This
 * should run fast, and not perform and blocking operations such as I/O if it can be
 * avoided.
 */
void system_tick();


/**
 * This method will be called by the watchdog on its death. This method should
 * unconditionally and immediately disable all contactors, balancing, etc. and
 * put itself in a safe state. From there it should set the proper fault
 * indicators and record as much data as possible to the SD card. Finally,
 * it will hang up the system, so it cannot leave this state until a manual reset.
 */
void system_abort();




//////////////////////////////////////////////////////////////////////
void system_initialize();
void system_reset();
void system_self_test();

// Calculates and sets the output state based on fault
void system_determineOutputs();
bool system_parseCells();
bool system_parseTherm1();
bool system_parseTherm2();
void system_voltFaults();
void system_thermFaults();
void system_currentFaults();

tSystemState system_nextState(tSystemState eNow, bool *pbDataValid);
void system_start_sampling();

#endif /* SYSTEM_H_ */
















