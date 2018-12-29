/*
 * iocontrol.h
 *
 * Handles all low level direct IO on the system, such as gpio, analog input, and
 * timer PWM.
 *
 *  Created on: Dec 23, 2018
 *      Author: Duemmer
 */

#ifndef IOCONTROL_H_
#define IOCONTROL_H_

#include <stdint.h>
#include <stdbool.h>

// Relay I/O
void ioctl_setChargeMain(bool bEnable);
void ioctl_setChargeAux(bool bEnable);
void ioctl_setDischargeMain(bool bEnable);
void ioctl_setDischargeAux(bool bEnable);
void ioctl_setDischargeMain(bool bEnable);
void ioctl_setBattNegMain(bool bEnable);
void ioctl_setBattNegAux(bool bEnable);
void ioctl_setPrechargeMain(bool bEnable);
void ioctl_setPrechargeAux(bool bEnable);

/**
 * Sets the state of all relays in the system to a certain value
 */
void ioctl_setAll(bool bEnable);

/**
 * Reads the sampled ADC readings. Returns a nonzero value if
 * there is data sampled, false otherwise.
 */
uint8_t ioctl_sampledCurrent(
        uint32_t *pui32C1p,
        uint32_t *pui32C1n,
        uint32_t *pui32C2p,
        uint32_t *pui32C2n);

uint8_t ioctl_sampledAux(
        uint32_t *pui32PackVolts,
        uint32_t *pui32Therm1,
        uint32_t *pui32Therm2,
        uint32_t *pui32Therm3);

#endif /* IOCONTROL_H_ */
