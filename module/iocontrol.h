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


#endif /* IOCONTROL_H_ */
