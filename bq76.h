/*
 * bq76.h
 *
 *  Created on: Nov 10, 2018
 *      Author: Duemmer
 */

#ifndef BQ76_H_
#define BQ76_H_

#include "system.h"
#include <stdint.h>

#define BQ_NUM_MODULES          2
#define BQ_WRITE_NORESP_TIMEOUT 250 // usecs
#define BQ_WRITE_RESP_TIMEOUT   2000 // usecs

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



void bq76_connect();
uint8_t bq76_faultStat();
uint8_t bq76_autoAddress();
uint8_t bq76_enabled();

uint8_t bq76_write(
        uint8_t ui8Flags,
        uint8_t ui8Len,
        uint8_t ui8Addr,
        uint8_t *data);

uint8_t bq76_writeReg(
        uint8_t ui8Flags,
        uint8_t ui8Addr,
        uint16_t ui16Reg,
        uint8_t *ui8Data,
        uint8_t ui8DataLength);

uint8_t bq76_writeReg(
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

uint16_t bq76_checksum(uint8_t *pui8Buf, uint16_t ui16Len);

void bq76_readRawCellVolts(uint16_t *pui16buf);
float bq76_rawCellToVolts(uint16_t ui16CellVal);

#endif /* BQ76_H_ */
