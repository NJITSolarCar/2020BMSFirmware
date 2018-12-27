/*
 * config.h
 *
 * Provides utilities to permit persistent storage and retrieval of BMS configuration
 * settings, using the onboard EEPROM storage. This should include all constants
 * that can vary in the code, and that can be changed at runtime.
 * Examples include fault thresholds, timeouts, cell populations, etc. In addition,
 * a set of default configurations should be defined at compile time, to be
 * used whenever necessary, and at the discretion of the caller.
 *
 *  Created on: Dec 23, 2018
 *      Author: Duemmer
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

// Maximun number of supported BQ76 Modules
#define CONF_MAX_BQ_MODULES                     4

///////////////////////////////// DEFAULT CONFIGURATIONS //////////////////////////////
#define CONF_DEFAULT_NUM_CELLS                  16
#define CONF_DEFAULT_COLLATE_CELL_SAMPLES       true

// BQ76 module configurations

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

    uint8_t ui8NumCells

    // Comm timeouts
    uint32_t ui32WriteNoRespTimeout;
    uint32_t ui32WriteRespTimeout
} tBQBoardCal;



// Represents all configuration data in the system, packed
// for easy sorage and manipulation
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


    // Cell voltage fault levels
    float fCellUVFaultVoltage;
    float fCellOVFaultVoltage;

    // Sustained overcurrent fault
    float fOCFaultAmps;

    // Transient overcurrent fault / timing
    float fOCTransientFaultAmps;
    uint32_t ui32OCTransientFaultUsecs;

    // Overtemperature fault threshold
    float fPackOTFaultTemp;
    uint32_t ui32PackOTFaultUsecs;

    // Charging faults
    float fOChgSoc;
    float fUChgSoc;
    float fChgOCFault;


    // BQ Configurations
    uint8_t ui8NumBQModules;
    tBQBoardCal bqCals[CONF_MAX_BQ_MODULES];

    // polynomial to scale voltage to SOC
    float pfVoltsToSocPoly[8];

} tConf;


/**
 * Writes the specified config to eeprom. Will also save a checksum
 * at the end of the configuration, to detect if data is corrupted, or
 * no data has been saved to this mcu. Checksums will be done with the
 * onboard crc module. Blocks until is is complete, and returns true
 * if successful, false otherwise
 */
uint8_t config_save(tConf *psConfig);


/**
 * Loads the current config saved in EEPROM. Will verify the checksum, and will
 * return false if it fails. Returns true iff data was successfully read from
 * EEPROM, and the checksum was valid. If EEPROM data could not be verified, will
 * populate the config with default configuration in addition to returning false.
 */
uint8_t config_load(tConf *psConfig);


/**
 * Populates psConfig with the default configuration data for the system,
 * which is defined at compile time
 */
void config_default(tConf *psConfig);

#endif /* CONFIG_H_ */





