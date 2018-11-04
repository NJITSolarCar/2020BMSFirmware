/*
 * system.h
 *
 *  Created on: Nov 4, 2018
 *      Author: Duemmer
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

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
     SYSCTL_PERIPH_UART4
};

/** System Settings */

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

#endif /* SYSTEM_H_ */
















