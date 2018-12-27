/*
 * watchdog.h
 *
 * Provides a simple background safety system. The purpose of this system is to
 * watch over the application code and detect critical software faults, hangs,
 * and malfunctions. If anything is detected, the entire system will be frozen
 * and shut down.
 *
 *  Created on: Dec 23, 2018
 *      Author: Duemmer
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include <stdint.h>

/**
 * Starts the watchdog running, and will start loaded to period. Feeding
 * the watchdog will restore the timer to period. If the watchdog expires,
 * control is passed to pfnOnDeath, which should put the system in a
 * safe and halted state
 */
void watchdog_begin(uint32_t period, void (*pfnOnDeath)());

/**
 * Restores the watchdog's down counter to the original value set by period in
 * watchdog_begin. After starting the watchdog, this must be called often enough
 * so that the down counter never expires
 */
void watchdog_feed();


/**
 * Instantly clears the watchdog's down counter and runs the OnDeath function
 */
void watchdog_kill();

#endif /* WATCHDOG_H_ */
