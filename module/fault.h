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

// L1 Faults
#define FAULT_L1_CUTOFF

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
