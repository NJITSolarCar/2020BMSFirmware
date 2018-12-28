/*
 * fault.h
 *
 * Manages all fault handling of the system. This should provide a set of
 * functions to detect and respond to faults based on input data passed to
 * the functions. When faults are detected, the appropriate flags should be
 * set, outputs updated, messages sent, etc. This includes running fault
 * status LEDs, etc.
 *
 *
 *  Created on: Dec 23, 2018
 *      Author: Duemmer
 */

#ifndef FAULT_H_
#define FAULT_H_

#include <stdint.h>

// If the data for a fault is invalid, it should be set to this
#define FAULT_NO_DATA               UINT32_MAX

// Fault Codes
#define FAULT_PACK_OVER_VOLTAGE     0x4000
#define FAULT_PACK_UNDER_VOLTAGE    0x2000
#define FAULT_CELL_OVER_VOLTAGE     0x1000
#define FAULT_CELL_UNDER_VOLTAGE    0x800
#define FAULT_OVER_DISCHARGE        0x400
#define FAULT_OVER_CHARGE           0x200
#define FAULT_OVER_TEMP             0x100
#define FAULT_UNDER_TEMP            0x80
#define FAULT_PACK_OPEN_CIRCUIT     0x40
#define FAULT_IMBALANCED            0x20
#define FAULT_PACK_SHORT            0x10
#define FAULT_OVER_CURRENT          0x8
#define FAULT_COMMUNICATIONS        0x4
#define FAULT_BQ_CHIP_FAULT         0x2
#define FAULT_PACK_GENERAL          0x1


/**
 * Registers this fault with the system. ui32Data should be for the
 * highest level fault
 */
void fault_setFault(uint32_t ui32Faults, uint32_t ui32Data);


/**
 * Clears the specified faults from the system. Note that if the highest fault
 * is cleared, then the fault data flag will be set to FAULT_NO_DATA
 */
void fault_clearFaults(uint32_t ui32Mask);


/**
 * Sets the operational fault level of the system. This can be any number from 0-5,
 * inclusive. Level 0 is a software crash or watchdog death, 1 is a fatal but non-
 * halting fault that immediately shuts down the battery system. Level 2 is a
 * dangerous issue that will automatically disable effected systems, Level
 * 3 is a persistent fault, but it doesn't shut anything down, Level 4 is like
 * level 3 but it doesn't remain after a reset, and level 5 means no fault.
 */
void fault_assertFaultLevel(uint8_t ui8State);


/**
 * Reads any persistent faults saved to EEPROM. pui32Data is a description of additional
 * data associated with the most severe fault. For example, a cell OV fault
 * might store the cell number, etc. Returns a nonzero value if valid fault
 * data was read from EEPROM, 0 otherwise.
 */
uint8_t fault_getPersistentFaults(uint32_t *pui32FaultSum, uint32_t *pui32Data);


/**
 * Writes this fault flag to EEPROM. Note that it will not write the exact fault
 * summary, but will mask any non-persistent faults. Even if the highest priority
 * fault is masked, pui32Data will not be changed or cleared.
 */
uint8_t fault_writePersistentFaults(uint32_t *pui32FaultSum, uint32_t *pui32Data);


#endif /* FAULT_H_ */
