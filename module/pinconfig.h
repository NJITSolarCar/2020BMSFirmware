/*
 * pinconfig.h
 *
 * Defines all of the pin settings and configurations on the microcontroller
 *  Created on: Nov 2, 2018
 *      Author: Duemmer
 */

#ifndef PINCONFIG_H_
#define PINCONFIG_H_

#include <stdint.h>

#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/adc.h>

#include <inc/hw_memmap.h>

/** General use peripherals */
#define BQLOOP_TIMER            TIMER1_BASE

/** General Purpose pins */

// General Timer PWM pin 1
#define PWM1_PORT               GPIO_PORTB_BASE
#define PWM1_PIN                GPIO_PIN_6
#define PWM1_PINMAP             GPIO_PB6_T0CCP0

// General Timer PWM pin 2
#define PWM2_PORT               GPIO_PORTB_BASE
#define PWM2_PIN                GPIO_PIN_7
#define PWM2_PINMAP             GPIO_PB7_T0CCP1

// General GPIO 1
#define GIO1_PORT               GPIO_PORTG_BASE
#define GIO1_PIN                GPIO_PIN_0

// General GPIO 2
#define GIO2_PORT               GPIO_PORTD_BASE
#define GIO2_PIN                GPIO_PIN_7



/* Digital I/O lines */

// BQ module fault
#define BQFAULT_PORT            GPIO_PORTF_BASE
#define BQFAULT_PIN             GPIO_PIN_4
#define BQFAULT_INT_LEVEL       GPIO_RISING_EDGE

// BQ wakeup
#define BQWAKE_PORT             GPIO_PORTC_BASE
#define BQWAKE_PIN              GPIO_PIN_6

// CAN interface STB
#define CANSTB_PORT             GPIO_PORTB_BASE
#define CANSTB_PIN              GPIO_PIN_4

// User Switch
#define USERSWITCH_PORT         GPIO_PORTB_BASE
#define USERSWITCH_PIN          GPIO_PIN_5
#define USERSWITCH_INT_LEVEL    GPIO_FALLING_EDGE

// Debug LEDs
#define DEBUGLED1_PORT          GPIO_PORTD_BASE
#define DEBUGLED1_PIN           GPIO_PIN_6

#define DEBUGLED2_PORT          GPIO_PORTC_BASE
#define DEBUGLED2_PIN           GPIO_PIN_7

// Relay controls
#define CHARGEMAIN_PORT         GPIO_PORTB_BASE
#define CHARGEMAIN_PIN          GPIO_PIN_3

#define CHARGEAUX_PORT          GPIO_PORTB_BASE
#define CHARGEAUX_PIN           GPIO_PIN_2

#define DISCHARGEMAIN_PORT      GPIO_PORTB_BASE
#define DISCHARGEMAIN_PIN       GPIO_PIN_1

#define DISCHARGEAUX_PORT       GPIO_PORTB_BASE
#define DISCHARGEAUX_PIN        GPIO_PIN_0

#define BATTNEGMAIN_PORT        GPIO_PORTD_BASE
#define BATTNEGMAIN_PIN         GPIO_PIN_5

#define BATTNEGAUX_PORT         GPIO_PORTD_BASE
#define BATTNEGAUX_PIN          GPIO_PIN_4

#define PRECHARGEMAIN_PORT      GPIO_PORTG_BASE
#define PRECHARGEMAIN_PIN       GPIO_PIN_1

#define PRECHARGEAUX_PORT       GPIO_PORTG_BASE
#define PRECHARGEAUX_PIN        GPIO_PIN_2



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

#define BQ_RECV_TIMER           TIMER5_BASE
#define BQ_RECV_TIMER_PART      TIMER_A

// RS485 Communications
#define RS485UART_MODULE        UART2_BASE
#define RS485UART_RX_PINCONFIG  GPIO_PG4_U2RX
#define RS485UART_TX_PINCONFIG  GPIO_PG5_U2TX
#define RS485UART_DE_PORT       GPIO_PORTG_BASE
#define RS485UART_DE_PIN        GPIO_PIN_3

// SD Card Communications
#define SDSPI_MODULE            SSI1_BASE
#define SDSPI_RX_PINCONFIG      GPIO_PF0_SSI1RX
#define SDSPI_TX_PINCONFIG      GPIO_PF1_SSI1TX
#define SDSPI_CLK_PINCONFIG     GPIO_PF2_SSI1CLK
#define SDSPI_FSS_PINCONFIG     GPIO_PF3_SSI1FSS

// General SSI
#define GPSSI_MODULE            SSI0_BASE
#define GPSSI_RX_PINCONFIG      GPIO_PA4_SSI0RX
#define GPSSI_TX_PINCONFIG      GPIO_PA5_SSI0TX
#define GPSSI_CLK_PINCONFIG     GPIO_PA2_SSI0CLK
#define GPSSI_FSS_PINCONFIG     GPIO_PA3_SSI0FSS

// General I2C
#define GPI2C_MODULE            I2C1_BASE
#define GPI2C_SDA_PINCONFIG     GPIO_PA7_I2C1SDA
#define GPI2C_SCL_PINCONFIG     GPIO_PA6_I2C1SCL

// General UART
#define GPUART_MODULE           UART0_BASE
#define GPUART_RX_PINCONFIG     GPIO_PA0_U0RX
#define GPUART_TX_PINCONFIG     GPIO_PA1_U0TX

// CAN bus
#define CAN_MODULE              CAN0_BASE
#define CAN_TX_PINCONFIG        GPIO_PE5_CAN0TX
#define CAN_RX_PINCONFIG        GPIO_PE4_CAN0RX



#endif /* PINCONFIG_H_ */









