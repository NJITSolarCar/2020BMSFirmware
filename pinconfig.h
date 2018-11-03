/*
 * pinconfig.h
 *
 * Defines all of the pin settings and configurations on the microcontroller
 *  Created on: Nov 2, 2018
 *      Author: Duemmer
 */

#ifndef PINCONFIG_H_
#define PINCONFIG_H_

#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/adc>

#include <inc/hw_memmap.h>



/** General Purpose pins */

// General Timer PWM pin 1
#define PWM1_PORT               GPIO_PORTB_BASE
#define PWM1_PIN                GPIO_PIN_6
#define PWM1_PINMAP             GPIO_PB6_T0CCP0

// General Timer PWM pin 2
#define PWM2_PORT               GPIO_PORTB_BASE
#define PWM2_PIN                GPIO_PIN_7
#define PWM2_PINMAP             GPIO_PB7_T0CCP1

// General Timer PWM pin 3
#define PWM3_PORT               GPIO_PORTF_BASE
#define PWM3_PIN                GPIO_PIN_4
#define PWM3_PINMAP             GPIO_PF4_T2CCP0

// General GPIO 1
#define GIO1_PORT               GPIO_PORTG_BASE
#define GIO1_PIN                GPIO_PIN_0

// General GPIO 2
#define GIO2_PORT               GPIO_PORTG_BASE
#define GIO2_PIN                GPIO_PIN_1

// General GPIO 3
#define GIO3_PORT               GPIO_PORTG_BASE
#define GIO3_PIN                GPIO_PIN_2



/* Digital I/O lines */

// BQ module fault
#define BQFAULT_PORT            GPIO_PORTC_BASE
#define BQFAULT_PIN             GPIO_PIN_6

// CAN interface STB
#define CANSTB_PORT             GPIO_PORTB_BASE
#define CANSTB_PIN              GPIO_PIN_4

// User Switch
#define USERSWITCH_PORT         GPIO_PORTB_BASE
#define USERSWITCH_PIN          PIO_PIN_3

// Debug LEDs
#define DEBUGLED1_PORT          GPIO_PORTB_BASE
#define DEBUGLED1_PIN           GPIO_PIN_1



/** Analog Inputs */

// Current 1 positive
#define CURRENT1POS_PORT        GPIO_PORTE_BASE
#define CURRENT1POS_PIN         GPIO_PIN_3
#define CURRENT1POS_CHANNEL     ADC_CTL_CH0

// Current 1 negative
#define CURRENT1NEG_PORT        GPIO_PORTE_BASE
#define CURRENT1NEG_PIN         GPIO_PIN_2
#define CURRENT1NEG_CHANNEL     ADC_CTL_CH1

// Current 2 positive
#define CURRENT2POS_PORT        GPIO_PORTE_BASE
#define CURRENT2POS_PIN         GPIO_PIN_1
#define CURRENT2POS_CHANNEL     ADC_CTL_CH2

// Current 2 negative
#define CURRENT2NEG_PORT        GPIO_PORTE_BASE
#define CURRENT2NEG_PIN         GPIO_PIN_0
#define CURRENT2NEG_CHANNEL     ADC_CTL_CH3

// Pack voltage sense
#define PACKVOLTAGE_PORT        GPIO_PORTD_BASE
#define PACKVOLTAGE_PIN         GPIO_PIN_3
#define PACKVOLTAGE_CHANNEL     ADC_CTL_CH4

// Thermistor 1
#define THERM1_PORT             GPIO_PORTD_BASE
#define THERM1_PIN              GPIO_PIN_2
#define THERM1_CHANNEL          ADC_CTL_CH5

// Thermistor 2
#define THERM2_PORT             GPIO_PORTD_BASE
#define THERM2_PIN              GPIO_PIN_1
#define THERM2_CHANNEL          ADC_CTL_CH6

// Thermistor 3
#define THERM3_PORT             GPIO_PORTD_BASE
#define THERM3_PIN              GPIO_PIN_0
#define THERM3_CHANNEL          ADC_CTL_CH7



/** Communication Ports */

// BQ Module communications
#define BQUART_MODULE           UART4_BASE
#define BQUART_RX_PINCONFIG     GPIO_PC4_U4RX
#define BQUART_TX_PINCONFIG     GPIO_PC5_U4TX

// RS485 Communications
#define RS485UART_MODULE        UART2_BASE
#define RS485UART_RX_PINCONFIG



#endif /* PINCONFIG_H_ */









