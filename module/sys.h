/*
 * sys.h
 *
 *  Created on: Jan 15, 2019
 *      Author: Duemmer
 */

#ifndef MODULE_SYS_H_
#define MODULE_SYS_H_

// General system info
#define SYSTEM_NUM_MCU_THERMO           3

#define SYSTEM_BQ_THERMO_PART           100

// Sampling periods, in usecs. Make them weird numbers so
// they don't intersect as much
#define SYSTEM_PERIOD_CURRSEN           1001
#define SYSTEM_PERIOD_PV                10581
#define SYSTEM_PERIOD_MCU_THERM         99230
#define SYSTEM_PERIOD_BQ_SAMPLE         5069


// ISR priorities
#define SYSTEM_PRIORITY_HIGH            2
#define SYSTEM_PRIORITY_LOW             6
#define SYSTEM_PRIORITY_MED             4

#define SYSTEM_PRI_CURRSEN              SYSTEM_PRIORITY_HIGH
#define SYSTEM_PRI_PV_READ              SYSTEM_PRIORITY_MED
#define SYSTEM_PRI_MCU_THERM            SYSTEM_PRIORITY_LOW

#define SYSTEM_PRI_CAN_RX               SYSTEM_PRIORITY_MED
#define SYSTEM_PRI_RS485_RX             SYSTEM_PRIORITY_MED

#define SYSTEM_PRI_BQUART               SYSTEM_PRIORITY_HIGH
#define SYSTEM_PRI_BQFAULT              SYSTEM_PRIORITY_HIGH
#define SYSTEM_PRI_BQPARSE              SYSTEM_PRIORITY_LOW
#define SYSTEM_PRI_BQ_SAMPLE_TIMER      SYSTEM_PRIORITY_MED


//////////////////////// REGULAR FUNCTIONS /////////////////////////

/**
 * Performs a self diagnostic test of the entire BMS system. Checks
 * only the hardware, and largely ignores higher leveled issues such as
 * overcurrent, imbalance, etc. Instead will check things like communication,
 * BQ hardware errors, and possibly cell tap open circuits.
 */
void system_selfTest();


/**
 * Initializes the Interrupt Service Routines (ISRs) used by the BMS. This
 * will enable the vectors, assign them priorities, and assign them handlers.
 */
void system_initISRs();


/**
 * Starts the timers required for routine sampling operation on the ADCs and
 * BQ76 stack. Note that this won'tprepare the ISRs themselves, they will be
 * configured in system_initISRs()
 */
void system_startSampling();


//////////////////////// ISRs /////////////////////////

void system_isr_bqParse();
void system_isr_bqTimer();

#endif /* MODULE_SYS_H_ */













