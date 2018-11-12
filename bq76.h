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

// Frame initialization byte settings
#define BQ_FRM_TYPE_RESPONSE    0x00 // Response Frame
#define BQ_FRM_TYPE_COMMAND     0x80 // Command Frame

#define BQ_REQ_TYPE_SGL_RESP    0x00 // Single Device Write with Response
#define BQ_REQ_TYPE_SGL_NORESP  0x10 // Single Device Write without Response
#define BQ_REQ_TYPE_GRP_RESP    0x20 // Group Write with Response
#define BQ_REQ_TYPE_GRP_NORESP  0x30 // Group Write without Response
#define BQ_REQ_TYPE_BC_RESP     0x00 // Broadcast Write with Response
#define BQ_REQ_TYPE_BC_NORESP   0x10 // Broadcast Write without Response

#define BQ_ADDR_SIZE_8          0x00 // 8-bit Register Address
#define BQ_ADDR_SIZE_16         0x08 // 16-bit Register Address

#define BQ_DATA_SIZE_0          0x00 // 0 byte packet
#define BQ_DATA_SIZE_1          0x01 // 1 byte packet
#define BQ_DATA_SIZE_2          0x02 // 2 byte packet
#define BQ_DATA_SIZE_3          0x03 // 3 byte packet
#define BQ_DATA_SIZE_4          0x04 // 4 byte packet
#define BQ_DATA_SIZE_5          0x05 // 5 byte packet
#define BQ_DATA_SIZE_6          0x06 // 6 byte packet
#define BQ_DATA_SIZE_8          0x07 // 8 byte packet


// Baud rates to use for UART. The BQ76 defaults to BQBAUD_INIT, and that
// is changed to BQBAUD_RUNNING during initialization to boost performance
#define BQ_BAUD_INIT            250000
#define BQ_BAUD_RUNNING         1000000

// Addresses for the 2 modules
#define BQ_MOD0_ADDR            0
#define BQ_MOD1_ADDR            1

void bq76_connect();
uint8_t bq76_faultStat();
uint8_t bq76_enabled();
uint32_t bq76_write(uint8_t ui8FrameInit, uint8_t ui8Addr, uint8_t *data);
uint16_t bq76_checksum(uint8_t *pui8Buf, uint16_t ui16Len);
void bq76_readRawCellVolts(uint16_t *pui16buf);
float bq76_rawCellToVolts(uint16_t ui16CellVal);

#endif /* BQ76_H_ */
