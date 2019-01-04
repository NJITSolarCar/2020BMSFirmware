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

// Peripheral Units
const uint32_t SYSCTL_PERIPHS[] =
{
     SYSCTL_PERIPH_ADC0,
     SYSCTL_PERIPH_ADC1,
     SYSCTL_PERIPH_CAN0,
     SYSCTL_PERIPH_GPIOA,
     SYSCTL_PERIPH_GPIOB,
     SYSCTL_PERIPH_GPIOC,
     SYSCTL_PERIPH_GPIOD,
     SYSCTL_PERIPH_GPIOE,
     SYSCTL_PERIPH_GPIOF,
     SYSCTL_PERIPH_GPIOG,
     SYSCTL_PERIPH_I2C0,
     SYSCTL_PERIPH_SSI0,
     SYSCTL_PERIPH_SSI1,
     SYSCTL_PERIPH_TIMER0,
     SYSCTL_PERIPH_TIMER1,
     SYSCTL_PERIPH_TIMER5,
     SYSCTL_PERIPH_UART0,
     SYSCTL_PERIPH_UART2,
     SYSCTL_PERIPH_UART4,
     SYSCTL_PERIPH_WDOG0
};

/** System Settings */

// Fraction of the time certain samples run. Numbers are "weird" so the calls
// don't intersect as often
#define SYSTEM_THERMO_PART      100
#define SYSTEM_BQ_THERMO_PART   101
#define SYSTEM_BQ_INTERNAL_PART 99973

#define SYSTEM_NUM_MCU_THERMO   3

// Clock settings, configured for 80 MHz. See datasheet page 208 for divider reference
#define MCU_XTAL                SYSCTL_XTAL_8MHZ
#define MCU_VCO                 SYSCTL_CFG_VCO_480
#define MCU_PLL_DIV             SYSCTL_SYSDIV_2_5
#define MCU_ACTIVE_FREQ         80000000



// Analog Sampling Settings

// Current Sensor Sequence. Will be set to always sample
#define CURRENT_MODULE          ADC0_BASE
#define CURRENT_SEQUENCE        2
#define CURRENT_STEP            0
#define CURRENT_HYSTERESIS      4
#define CURRENT_OVERSAMPLE      64

// Thermistor / pack voltage sequence
#define THERMPV_MODULE          ADC1_BASE
#define THERMPV_SEQUENCE        2
#define THERMPV_STEP            0
#define THERMPV_OVERSAMPLE      64
#define THERMPV_SAMPLE_FREQ     100


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
void system_determineOutputs();
void system_parseCells();
void system_parseThermo1();
void system_parseThermo2();
void system_voltFaults();
void system_thermFaults();
void system_currentFaults();

tSystemState system_nextState(tSystemState eNow, bool *pbDataValid);

#endif /* SYSTEM_H_ */
















