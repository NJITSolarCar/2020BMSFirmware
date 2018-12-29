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
#include <stdbool.h>

#include "types.h"

// Default fault data object to indicate that there is no data present
// #define FAULT_DATA_NONE             { UINT32_MAX }

// Convert fault index to bitmask
#define FAULT_MASK(x)               (1 << x)

// Fault level thresholds
#define FAULT_L0_THRESH             0 // 0x1
#define FAULT_L1_THRESH             2 // 0x4
#define FAULT_L2_THRESH             8 // 0x100
#define FAULT_L3_THRESH             17 // 0x20000
#define FAULT_L4_THRESH             19 // 0x100000

// Specific fault bits
// L0 Faults
#define FAULT_BMS_CRASH             0 // 0x1

// L1 Faults
#define FAULT_BQ_CHIP_FAULT         1 // 0x2
#define FAULT_BQ_COM                2 // 0x4

// L2 Faults
#define FAULT_PACK_SHORT            3 // 0x8
#define FAULT_PACK_OPEN_CIRCUIT     4 // 0x10
#define FAULT_IMBALANCED            5 // 0x20
#define FAULT_OVER_TEMP             6 // 0x40
#define FAULT_UNDER_TEMP            7 // 0x80
#define FAULT_PACK_VOLTAGE_DISAGREE 8 // 0x100
#define FAULT_PACK_GENERAL          9 // 0x200

// L3 Faults
#define FAULT_OVER_CURRENT_DISCHG   10 // 0x400
#define FAULT_OVER_CURRENT_CHG      11 // 0x800
#define FAULT_OVER_DISCHARGE        12 // 0x1000
#define FAULT_OVER_CHARGE           13 // 0x2000
#define FAULT_PACK_OVER_VOLTAGE     14 // 0x4000
#define FAULT_PACK_UNDER_VOLTAGE    15 // 0x8000
#define FAULT_CELL_OVER_VOLTAGE     16 // 0x10000
#define FAULT_CELL_UNDER_VOLTAGE    17 // 0x20000

// L4 Faults
#define FAULT_TRANSIENT_CHG_OC      18 // 0x40000
#define FAULT_TRANSIENT_DISCHG_OC   19 // 0x80000
#define FAULT_GEN_COM               20 // 0x100000


// Some fault class masks to determine what L3 faults disable certain relays
#define FAULT_L3_CHARGE_RELAY_TRIGGERS        \
    ( FAULT_MASK(FAULT_OVER_CHARGE)         | \
    FAULT_MASK(FAULT_OVER_CURRENT_CHG)      | \
    FAULT_MASK(FAULT_PACK_OVER_VOLTAGE) )

#define FAULT_L3_DISCHARGE_RELAY_TRIGGERS     \
    ( FAULT_MASK(FAULT_OVER_DISCHARGE)      | \
    FAULT_MASK(FAULT_OVER_CURRENT_DISCHG)   | \
    FAULT_MASK(FAULT_PACK_UNDER_VOLTAGE) )


/**
 * Registers this fault with the system.
 */
void fault_setFault(uint32_t ui32Faults, tFaultInfo uFaultInfo);


/**
 * Returns the system fault level classification based on the current fault
 * states
 */
uint8_t fault_getLevel();


/**
 * Returns a bit packed integer detailing which faults are set at this moment
 */
uint32_t fault_getFaultsSet();


/**
 * Determines the time which a certain fault was asserted. Note that only
 * a single bit in ui32Fault should be set, corresponding to a single fault.
 * Returns true if the fault is asserted and a valid time was retrieved, false
 * otherwise.
 */
bool fault_timeAsserted(uint32_t ui32Fault, uint64_t *pui64Time);


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
