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
#include <inc/hw_ints.h>

/** General use peripherals */
#define PINCFG_TIMER_SYSTIME            WTIMER0_BASE
#define PINCFG_TIMER_PART_SYSTIME       TIMER_BOTH
#define PINCFG_TIMER_CFG_SYSTIME        TIMER_CFG_ONE_SHOT_UP

#define PINCFG_TIMER_PV                 WTIMER1_BASE
#define PINCFG_TIMER_PART_PV            TIMER_A
#define PINCFG_TIMER_CFG_PV             TIMER_CFG_A_PERIODIC

#define PINCFG_TIMER_MCU_THERM          WTIMER1_BASE
#define PINCFG_TIMER_PART_MCU_THERM     TIMER_B
#define PINCFG_TIMER_CFG_MCU_THERM      TIMER_CFG_B_PERIODIC

#define PINCFG_TIMER_BQ_SAMPLE          WTIMER2_BASE
#define PINCFG_TIMER_PART_BQ_SAMPLE     TIMER_A
#define PINCFG_TIMER_CFG_BQ_SAMPLE      TIMER_CFG_A_PERIODIC

#define PINCFG_TIMER_CURRSEN            WTIMER2_BASE
#define PINCFG_TIMER_PART_CURRSEN       TIMER_B
#define PINCFG_TIMER_CFG_CURRSEN        TIMER_CFG_B_PERIODIC

// Interrupt vector numbers
#define PINCFG_VEC_CURRSEN              INT_ADC0SS1
#define PINCFG_VEC_PV_READ              INT_ADC1SS3
#define PINCFG_VEC_MCU_THERM            INT_ADC1SS1
#define PINCFG_VEC_CAN_RX               INT_CAN0
#define PINCFG_VEC_RS485_RX             INT_UART2
#define PINCFG_VEC_BQUART               INT_UART4
#define PINCFG_VEC_BQFAULT              INT_GPIOF
#define PINCFG_VEC_BQ_SAMPLE_TIMER      INT_WTIMER2A

// Trigger ISR vectors
#define PINCFG_VEC_CURRSEN_TRIG         INT_WTIMER2B
#define PINCFG_VEC_PV_READ_TRIG         INT_WTIMER1A
#define PINCFG_VEC_MCU_THERM_TRIG       INT_WTIMER1B

// IRQ25 is unused by hardware on this device. It can still function as a
// SoftWare triggered Interrupt (SWI)
#define PINCFG_VEC_BQPARSE              25


/** General Purpose pins */

// General Timer PWM pin 1
#define PINCFG_PWM1_PORT                GPIO_PORTB_BASE
#define PINCFG_PWM1_PIN                 GPIO_PIN_6
#define PINCFG_PWM1_PINMAP              GPIO_PB6_T0CCP0

// General Timer PWM pin 2
#define PINCFG_PWM2_PORT                GPIO_PORTB_BASE
#define PINCFG_PWM2_PIN                 GPIO_PIN_7
#define PINCFG_PWM2_PINMAP              GPIO_PB7_T0CCP1

// General GPIO 1
#define PINCFG_GIO1_PORT                GPIO_PORTG_BASE
#define PINCFG_GIO1_PIN                 GPIO_PIN_0

// General GPIO 2
#define PINCFG_GIO2_PORT                GPIO_PORTD_BASE
#define PINCFG_GIO2_PIN                 GPIO_PIN_7



/* Digital I/O lines */

// BQ module fault
#define PINCFG_BQFAULT_PORT             GPIO_PORTF_BASE
#define PINCFG_BQFAULT_PIN              GPIO_PIN_4
#define PINCFG_BQFAULT_INT_LEVEL        GPIO_RISING_EDGE

// BQ wakeup
#define PINCFG_BQWAKE_PORT              GPIO_PORTC_BASE
#define PINCFG_BQWAKE_PIN               GPIO_PIN_6

// CAN interface STB
#define PINCFG_CANSTB_PORT              GPIO_PORTB_BASE
#define PINCFG_CANSTB_PIN               GPIO_PIN_4

// User Switch
#define PINCFG_SW1_PORT                 GPIO_PORTB_BASE
#define PINCFG_SW1_PIN                  GPIO_PIN_5
#define PINCFG_SW1_INT_LEVEL            GPIO_FALLING_EDGE

// Debug LEDs
#define PINCFG_DEBUGLED1_PORT           GPIO_PORTD_BASE
#define PINCFG_DEBUGLED1_PIN            GPIO_PIN_6

#define PINCFG_DEBUGLED2_PORT           GPIO_PORTC_BASE
#define PINCFG_DEBUGLED2_PIN            GPIO_PIN_7

// Relay controls
#define PINCFG_CHARGEMAIN_PORT          GPIO_PORTB_BASE
#define PINCFG_CHARGEMAIN_PIN           GPIO_PIN_3

#define PINCFG_CHARGEAUX_PORT           GPIO_PORTB_BASE
#define PINCFG_CHARGEAUX_PIN            GPIO_PIN_2

#define PINCFG_DISCHARGEMAIN_PORT       GPIO_PORTB_BASE
#define PINCFG_DISCHARGEMAIN_PIN        GPIO_PIN_1

#define PINCFG_DISCHARGEAUX_PORT        GPIO_PORTB_BASE
#define PINCFG_DISCHARGEAUX_PIN         GPIO_PIN_0

#define PINCFG_BATTNEGMAIN_PORT         GPIO_PORTD_BASE
#define PINCFG_BATTNEGMAIN_PIN          GPIO_PIN_5

#define PINCFG_BATTNEGAUX_PORT          GPIO_PORTD_BASE
#define PINCFG_BATTNEGAUX_PIN           GPIO_PIN_4

#define PINCFG_PRECHARGEMAIN_PORT       GPIO_PORTG_BASE
#define PINCFG_PRECHARGEMAIN_PIN        GPIO_PIN_1

#define PINCFG_PRECHARGEAUX_PORT        GPIO_PORTG_BASE
#define PINCFG_PRECHARGEAUX_PIN         GPIO_PIN_2



/** Analog Inputs */

// Module data
#define PINCFG_CURRSEN_MODULE           ADC0_BASE
#define PINCFG_CURRSEN_SEQUENCE         1

#define PINCFG_AUX_MODULE               ADC1_BASE
#define PINCFG_PV_SEQUENCE              3
#define PINCFG_THERM_SEQUENCE           1

// Current 1 positive
#define PINCFG_CURRENT1POS_PORT         GPIO_PORTE_BASE
#define PINCFG_CURRENT1POS_PIN          GPIO_PIN_3
#define PINCFG_CURRENT1POS_CHANNEL      ADC_CTL_CH0

// Current 1 negative
#define PINCFG_CURRENT1NEG_PORT         GPIO_PORTE_BASE
#define PINCFG_CURRENT1NEG_PIN          GPIO_PIN_2
#define PINCFG_CURRENT1NEG_CHANNEL      ADC_CTL_CH1

// Current 2 positive
#define PINCFG_CURRENT2POS_PORT         GPIO_PORTE_BASE
#define PINCFG_CURRENT2POS_PIN          GPIO_PIN_1
#define PINCFG_CURRENT2POS_CHANNEL      ADC_CTL_CH2

// Current 2 negative
#define PINCFG_CURRENT2NEG_PORT         GPIO_PORTE_BASE
#define PINCFG_CURRENT2NEG_PIN          GPIO_PIN_0
#define PINCFG_CURRENT2NEG_CHANNEL      ADC_CTL_CH3

// Pack voltage sense
#define PINCFG_PV_PORT                  GPIO_PORTD_BASE
#define PINCFG_PV_PIN                   GPIO_PIN_3
#define PINCFG_PV_CHANNEL               ADC_CTL_CH4

// Thermistor 1
#define PINCFG_THERM1_PORT              GPIO_PORTD_BASE
#define PINCFG_THERM1_PIN               GPIO_PIN_2
#define PINCFG_THERM1_CHANNEL           ADC_CTL_CH5

// Thermistor 2
#define PINCFG_THERM2_PORT              GPIO_PORTD_BASE
#define PINCFG_THERM2_PIN               GPIO_PIN_1
#define PINCFG_THERM2_CHANNEL           ADC_CTL_CH6

// Thermistor 3
#define PINCFG_THERM3_PORT              GPIO_PORTD_BASE
#define PINCFG_THERM3_PIN               GPIO_PIN_0
#define PINCFG_THERM3_CHANNEL           ADC_CTL_CH7



/** Communication Ports */

// BQ Module communications
#define PINCFG_BQUART_MODULE            UART4_BASE
#define PINCFG_BQUART_RX_PINCONFIG      GPIO_PC4_U4RX
#define PINCFG_BQUART_TX_PINCONFIG      GPIO_PC5_U4TX

// RS485 Communications
#define PINCFG_RS485UART_MODULE         UART2_BASE
#define PINCFG_RS485UART_RX_PINCONFIG   GPIO_PG4_U2RX
#define PINCFG_RS485UART_TX_PINCONFIG   GPIO_PG5_U2TX
#define PINCFG_RS485UART_DE_PORT        GPIO_PORTG_BASE
#define PINCFG_RS485UART_DE_PIN         GPIO_PIN_3

// SD Card Communications
#define PINCFG_SDSPI_MODULE             SSI1_BASE
#define PINCFG_SDSPI_RX_PINCONFIG       GPIO_PF0_SSI1RX
#define PINCFG_SDSPI_TX_PINCONFIG       GPIO_PF1_SSI1TX
#define PINCFG_SDSPI_CLK_PINCONFIG      GPIO_PF2_SSI1CLK
#define PINCFG_SDSPI_FSS_PINCONFIG      GPIO_PF3_SSI1FSS

// General SSI
#define PINCFG_GPSSI_MODULE             SSI0_BASE
#define PINCFG_GPSSI_RX_PINCONFIG       GPIO_PA4_SSI0RX
#define PINCFG_GPSSI_TX_PINCONFIG       GPIO_PA5_SSI0TX
#define PINCFG_GPSSI_CLK_PINCONFIG      GPIO_PA2_SSI0CLK
#define PINCFG_GPSSI_FSS_PINCONFIG      GPIO_PA3_SSI0FSS

// General I2C
#define PINCFG_GPI2C_MODULE             I2C1_BASE
#define PINCFG_GPI2C_SDA_PINCONFIG      GPIO_PA7_I2C1SDA
#define PINCFG_GPI2C_SCL_PINCONFIG      GPIO_PA6_I2C1SCL

// General UART
#define PINCFG_GPUART_MODULE            UART0_BASE
#define PINCFG_GPUART_RX_PINCONFIG      GPIO_PA0_U0RX
#define PINCFG_GPUART_TX_PINCONFIG      GPIO_PA1_U0TX

// CAN bus
#define PINCFG_CAN_MODULE               CAN0_BASE
#define PINCFG_CAN_TX_PINCONFIG         GPIO_PE5_CAN0TX
#define PINCFG_CAN_RX_PINCONFIG         GPIO_PE4_CAN0RX



#endif /* PINCONFIG_H_ */









