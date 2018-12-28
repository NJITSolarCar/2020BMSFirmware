/*
 * calculation.h
 *
 * Defines the floating point calculations that the BMS must perform
 * at a high level. Examples include State of charge, real current, real voltage, etc.
 * Also included are methods to perform high level checks of voltages, currents,
 * charge, etc. Note: these methods should be completely independent from any hardware
 * access!
 *
 *  Created on: Dec 23, 2018
 *      Author: Duemmer
 */

#ifndef CALCULATION_H_
#define CALCULATION_H_

#include <stdint.h>
#include "types.h"

/**
 * Calculates an updated state of charge, based on the current system state.
 * This should be called as often as current and cell voltage are
 */
float calc_updateSOC(float fAmps, float *pfCellVoltages, uint32_t ui32CellCount);


/**
 * Evaluates a polynomial function defined by pfCoeffs on x
 */
float calc_evalPoly(float *pfCoeffs, uint32_t ui32NumCoeffs, float x);


/**
 * Converts a thermistor voltage to a temperature. fVoltFraction is the ratio
 * of the measured voltage to the max voltage, or v_measured / v_max.
 */
float calc_thermistorTemp(float fVoltFraction);


/**
 * Converts the raw ADC readings into a real current draw, in milliamps. This
 * compares the 4 raw inputs, and deciphers which one to use, based on which
 * inputs are saturated. In addition, it applies whatever scalars and shifts
 * are necessary.
 */
int32_t calc_adcToMilliAmps(
        uint16_t ui16Curr1Pos,
        uint16_t ui16Curr1Neg,
        uint16_t ui16Curr2Pos,
        uint16_t ui16Curr2Neg);





#endif /* CALCULATION_H_ */
