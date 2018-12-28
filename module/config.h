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

///////////////////////////////// DEFAULT CONFIGURATIONS //////////////////////////////
#define CONF_DEFAULT_NUM_CELLS                  16
#define CONF_DEFAULT_COLLATE_CELL_SAMPLES       true

// BQ76 module configurations




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

/**
 * Utility function to determine the total number of cells in the system
 */
uint32_t config_totalNumcells(tConf *psConfig);

#endif /* CONFIG_H_ */





