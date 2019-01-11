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

#include <driverlib/sysctl.h>


// Clock settings, configured for 80 MHz. See datasheet page 208 for divider reference
#define MCU_XTAL                SYSCTL_XTAL_8MHZ
#define MCU_VCO                 SYSCTL_CFG_VCO_480
#define MCU_PLL_DIV             SYSCTL_SYSDIV_2_5
#define MCU_ACTIVE_FREQ         80000000

/* Analog Sampling Settings */
#define IOCTL_CURRENTSENSE_OVERSAMPLE   64
#define IOCTL_AUX_OVERSAMPLE            64

// Timer settings for analog sampling
// PV read - fires @ 1 kHz
#define IOCTL_PV_TIMER_PRESCALE         80
#define IOCTL_PV_TIMER_LOAD             1000

// Thermistor read - fires @ 5 Hz
#define IOCTL_THERM_TIMER_PRESCALE      256
#define IOCTL_THERM_TIMER_LOAD          62500


// Utility macros
#define IOCTL_VCC                   3.3
#define IOCTL_ADC_TO_VOLT_FRAC(x)   ((1.0/4096.0) * (float)(x))
#define IOCTL_ADC_TO_VOLTS(x)       ((IOCTL_VCC/4096.0) * (float)(x))

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

// Relay I/O
void ioctl_setChargeMain(bool bEnable);
void ioctl_setChargeAux(bool bEnable);
void ioctl_setDischargeMain(bool bEnable);
void ioctl_setDischargeAux(bool bEnable);
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

/////////////////////////////////////////////////////////////
void ioctl_startAuxSample();
void ioctl_configGPIO();
void ioctl_configADC();
void ioctl_initialize();


#endif /* IOCONTROL_H_ */
