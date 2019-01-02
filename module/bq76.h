/*
 * bq76.h
 *
 * Contains Drivers for the BQ76PL455A Battery Management IC. The stack of
 * these forms the core functionality of the BMS, and the connection to the
 * cells. This interface should expose high level routines for all basic
 * functionalities, such as reading cell and pack voltages, detecting cell
 * faults, and passive balancing.
 *
 *  Created on: Nov 10, 2018
 *      Author: Duemmer
 */

#ifndef BQ76_H_
#define BQ76_H_

#include "system.h"
#include <stdint.h>
#include <stdbool.h>

#define BQ_NUM_THERMISTOR       16
#define BQ_WRITE_NORESP_TIMEOUT 250 // usecs
#define BQ_WRITE_RESP_TIMEOUT   2000 // usecs

// Sampling modes
#define BQ_SAMPLE_INVALID       0
#define BQ_SAMPLE_CELL          1
#define BQ_SAMPLE_THERMO1       2
#define BQ_SAMPLE_THERMO2       3

// Frame initialization byte settings
#define BQ_FRM_TYPE_RESPONSE    0x00 // Response Frame
#define BQ_FRM_TYPE_COMMAND     0x80 // Command Frame

#define BQ_REQ_TYPE_SGL_RESP    0x00 // Single Device Write with Response
#define BQ_REQ_TYPE_SGL_NORESP  0x10 // Single Device Write without Response
#define BQ_REQ_TYPE_GRP_RESP    0x20 // Group Write with Response
#define BQ_REQ_TYPE_GRP_NORESP  0x30 // Group Write without Response
#define BQ_REQ_TYPE_BC_RESP     0x40 // Broadcast Write with Response
#define BQ_REQ_TYPE_BC_NORESP   0x50 // Broadcast Write without Response

#define BQ_ADDR_SIZE_8          0x00 // 8-bit Register Address
#define BQ_ADDR_SIZE_16         0x08 // 16-bit Register Address

// All the other frame lengths but this one just use
// the same number (e.g a length of 1 is 1, length of 2 is 2, etc.),
// So explicitly specify this one
#define BQ_DATA_SIZE_8          0x07 // 8 byte packet

// Addresses of commonly used registers. If you need to write to it and it
// isn't listed here, add it!
#define BQ_REG_CMD              0x02
#define BQ_REG_ADDR             0x0A
#define BQ_REG_DEV_CTRL         0x0C
#define BQ_REG_DEVCONFIG        0x0E
#define BQ_REG_COMCONFIG        0x10
#define BQ_REG_FAULT_SUM        0x52


// Command Register Commands
#define BQ_CMD_SAMPLE           0x00
#define BQ_CMD_READ             0x20

// Oversampling counts
#define BQ_OVSMP_NONE           0
#define BQ_OVSMP_2              0
#define BQ_OVSMP_4              0
#define BQ_OVSMP_8              0
#define BQ_OVSMP_16             0
#define BQ_OVSMP_32             0

// Fault summary bits
#define BQ_UV_FAULT_SUM         (1 << 15)
#define BQ_OV_FAULT_SUM         (1 << 14)
#define BQ_AUXUV_FAULT_SUM      (1 << 13)
#define BQ_AUXOV_FAULT_SUM      (1 << 12)
#define BQ_CMPUV_FAULT_SUM      (1 << 11)
#define BQ_CMPOV_FAULT_SUM      (1 << 10)
#define BQ_COMM_FAULT_SUM       (1 << 9)
#define BQ_SYS_FAULT_SUM        (1 << 8)
#define BQ_CHIP_FAULT_SUM       (1 << 7)
#define BQ_GPI_FAULT_SUM        (1 << 6)


// Baud rates to use for UART. The BQ76 defaults to BQBAUD_INIT, and that
// is changed to BQBAUD_RUNNING during initialization to boost performance
#define BQ_BAUD_INIT            250000
#define BQ_BAUD_INIT_CODE       (1 << 12)

#define BQ_BAUD_RUNNING         1000000
#define BQ_BAUD_RUNNING_CODE    (3 << 12)

// Misc. Buffer sizes
#define BQ_WRITE_BUF_SIZE       16
#define BQ_READ_BUF_SIZE        256
#define BQ_WRITEREG_MAX_MSG     7 // Max data length allowed in bq76_writeReg(...)

// Converts a direct reading from the AFE to a voltage
#define BQ_ADC_VMAX             5.0
#define BQ_ADC_TO_VOLTS(x)      ((BQ_ADC_VMAX/65536.0) * ((float)(x)))
#define BQ_ADC_TO_VOLT_FRAC(x)  ((1.0/65536.0) * ((float)(x)))


/*
 * Initialization
 */
void bq76_connect();
uint8_t bq76_autoAddress();


/*
 * Low Level Communication
 */
uint8_t bq76_write(
        uint8_t ui8Flags,
        uint8_t ui8Len,
        uint8_t ui8Addr,
        uint8_t *data);

uint8_t bq76_writeRegMultiple(
        uint8_t ui8Flags,
        uint8_t ui8Addr,
        uint16_t ui16Reg,
        uint8_t *pui8Data,
        uint8_t ui8DataLength);

uint8_t bq76_writeRegSingle(
        uint8_t ui8Flags,
        uint8_t ui8Addr,
        uint16_t ui16Reg,
        uint8_t ui8Data);

uint8_t bq76_readRegSingle(
        uint8_t ui8Addr,
        uint16_t ui16Reg,
        uint8_t ui8Len,
        uint8_t *pui8Response);

uint8_t bq76_waitResponse(uint32_t ui32Timeout);
void bq76_waitResponse();

uint8_t bq76_parseResponse(uint8_t ui8Start, uint8_t *pui8Len);
uint8_t bq76_parseMultiple(
        uint8_t ui8NumResponses,
        uint8_t *pui8StartPtrs,
        uint8_t *pui8Lengths,
        uint8_t *pui8ChecksumGood);




/*
 * High Level Configuration
 */
uint8_t bq76_setCellVoltageThreshold(
        uint8_t ui8ReqType,
        uint8_t ui8Addr,
        float fUnder,
        float fOver);

uint8_t bq76_setOversampling(
        uint8_t ui8ReqType,
        uint8_t ui8Addr,
        bool bCollateSamples,
        uint8_t ui8Count);

uint8_t bq76_setCellsPopulated(uint8_t ui8Addr, uint16_t ui16CellMask);


/*
 * High Level Functionality
 */

uint8_t bq76_StartCellVoltageSample();
uint8_t bq76_startThermoSample(bool bMuxState);

uint8_t bq76_waitSampleDone(uint32_t ui32Timeout);
uint32_t bq76_readSampledVoltages(uint16_t *pui16Buf, uint32_t ui32BufSize);


/**
 * Reads the aggregate fault status of each bq76 module on the stack. Returns
 * the number of modules that have been read, or 0 on an error.
 */
uint8_t bq76_readFaultSum(uint16_t *pui16FaultSum);
uint8_t bq76_getCellsUV(uint16_t *pui16CellMasks, uint8_t ui8BufSize);
uint8_t bq76_getCellsOV(uint16_t *pui16CellMasks, uint8_t ui8BufSize);
uint8_t bq76_getComFaults(uint16_t *pui16FaultMasks, uint8_t ui8BufSize);
uint8_t bq76_getChipFaults(uint16_t *pui16FaultMasks, uint8_t ui8BufSize);

bool bq76_faultPinActive();

/**
 * Reads the state of all the thermistors on the BQ76 modules. Because reading
 * all the thermistors requires 2 sampling cycles, this call will wait for
 * the bq76 chips to sample on the false mux setting, read the data, then do it
 * again for the true mux setting.
 */
uint8_t bq76_readThermistors(uint16_t *pui16Buf);

/*
 * Utility
 */
uint16_t bq76_checksum(uint8_t *pui8Buf, uint16_t ui16Len);
uint8_t bq76_enabled();

///////////////////////////////////////////////////////////
bool bq76_isSampling();

#endif /* BQ76_H_ */
