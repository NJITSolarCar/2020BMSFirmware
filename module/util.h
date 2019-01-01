/*
 * util.h
 *
 * Contains a set of utility functions that dpn't have a place anywhere else.
 *
 *  Created on: Dec 25, 2018
 *      Author: Duemmer
 */

#ifndef MODULE_UTIL_H_
#define MODULE_UTIL_H_

/**
 * Reports the current system uptime in microseconds
 */
uint64_t util_usecs();
uint32_t util_ui32Max(uint32_t a, uint32_t b);

#endif /* MODULE_UTIL_H_ */
