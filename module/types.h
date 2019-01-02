/*
 * types.h
 *
 * Contains the definitions for the different types and structures used by the
 * system
 *
 *  Created on: Dec 28, 2018
 *      Author: Duemmer
 */

#ifndef MODULE_TYPES_H_
#define MODULE_TYPES_H_

// Maximun number of supported BQ76 Modules
#define CONF_MAX_BQ_MODULES                     4

///////////////////////////////// Types ////////////////////////////////

typedef enum {
    INITIALIZE,
    TEST,
    RESET,
    FAULT_CLEAR,
    FAULT_CALC,
    CELL_SAMPLE,
    CELL_READ,
    THERM1_SAMPLE,
    THERM1_READ,
    THERM2_SAMPLE,
    THERM2_READ,
    CURRENT_READ,
    AUX_READ,
    IDLE
} tSystemState;

// Additional representation of fault data
typedef union {
    uint32_t ui32;
    uint16_t pui16[2];
    uint8_t pui8[4];
} tFaultData;



// Additional information about asserted faults
typedef struct
{
    bool bAsserted;
    uint64_t ui64TimeFlagged;
    tFaultData data;
} tFaultInfo;



// Calibrations for a single BQ76 module
typedef struct
{
    // BQ76 Sampling settings
    bool bCollateCellSamples;
    uint8_t ui8NumOverSample;
    int8_t i8CellGEC;
    int8_t i8CellOEC;

    // usecs for cell sample periods
    float fFirstSampleTime;
    float fOtherSampleTime;

    uint8_t ui8NumCells;

    // Comm timeouts
    uint32_t ui32WriteNoRespTimeout;
    uint32_t ui32WriteRespTimeout;
} tBQBoardCal;



// Steinhart-hart approximation parameters
typedef struct {
    float fBallast;
    float fNominalR;
    float fA;
    float fB;
    float fC;
    float fD;
} tSHParams;



// Represents all configuration data in the system, packed
// for easy storage and manipulation
typedef struct
{
    // Multiplier to convert raw inputs to amps
    float fCurr1PosScl;
    float fCurr1NegScl;
    float fCurr2PosScl;
    float fCurr2NegScl;

    // Constant shift to apply to the calculated current
    float fCurr1PosOff;
    float fCurr1NegOff;
    float fCurr2PosOff;
    float fCurr2NegOff;

    // If an adc input is at least this many counts
    // away from an extreme, it is considered saturated
    uint16_t ui16ADCSaturationThresh;

    tSHParams sThermoParams;


    // Cell voltage fault levels
    float fCellUVFaultVoltage;
    float fCellOVFaultVoltage;
    float fCellImbalanceThresh;

    // max disagreement between measured pack voltage and sum of cell voltages
    float fPVDisagree;
    float fMaxPackVoltage;
    float fMinPackVoltage;

    // Sustained overcurrent fault
    float fOCDischgFaultAmps;
    float fOCChgFaultAmps;

    // Transient overcurrent fault / timing
    float fOCShortAmps;
    uint32_t ui32OCShortUsecs;

    // Overtemperature fault threshold
    float fPackOTFaultTemp;
    float fPackUTFaultTemp;
    uint32_t ui32PackOTFaultUsecs;

    // Charging faults
    float fOChgSoc;
    float fUChgSoc;


    // BQ Configurations
    uint32_t ui32NumBQModules;
    tBQBoardCal bqCals[CONF_MAX_BQ_MODULES];

    // polynomial to scale voltage to SOC
    float pfVoltsToSocPoly[8];

} tConf;

#endif /* MODULE_TYPES_H_ */
