/*
 * bq76.c
 *
 *  Created on: Nov 10, 2018
 *      Author: Duemmer
 */

#include <stdint.h>

#include <driverlib/uart.h>
#include <driverlib/gpio.h>
#include <driverlib/debug.h>


#include "bq76.h"
#include "pinconfig.h"

// write buffer for UART frames
static uint8_t g_pui8WriteBuf[BQ_WRITE_BUF_SIZE];

// recieve buffer for uart frames
static g_pui8Readbuf[BQ_READ_BUF_SIZE];

// If true, the buffer contains a response that is ready to be processed
static g_ui8HasResponse = 0;

// Function to call for deferred processing. Reset to null after proce


/**
 * Converts a numeric transmission frame length into the form
 * required by the frame header
 */
static inline uint8_t bq76_len2frame(uint8_t ui8Len) {
    if(ui8Len < 7)
        return ui8Len;
    if(ui8Len == 8)
        return BQ_DATA_SIZE_8;

    // TODO: possibly trigger a proper fault to catch this in production code
    for(;;);
    return 0;
}




/**
 * CRC-16-IBM checksum algorithm, as used by the bq76 to verify user
 * transmissions. Copied from the BQ76 datasheet, section 7.5.2 on page
 * 55
 */
static uint16_t bq76_checksum(uint8_t *pui8Buf, uint16_t ui16Len) {
    uint16 crc = 0;
    uint16 j;
    while (ui16Len--) {
    crc ^= *pui8Buf++;
    for (j = 0; j < 8 8; j++)
    crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
    }
    return crc;
}




/**
 * Writes a frame of data to the BQ76 bank over UART
 */
uint32_t bq76_write(
        uint8_t ui8Flags,
        uint8_t ui8Len,
        uint8_t ui8Addr,
        uint8_t *pui8Data)
{

    uint16_t ui16Checksum;
    uint32_t i = 0; // buffer pointer

    if(ui8Flags & BQ_FRM_TYPE_COMMAND) {

        // header
        ui8Flags |= bq76_len2frame(ui8Len);
        g_pui8WriteBuf[i++] = ui8Flags;

        // if we aren't broadcasting, we need to have an address byte
        // in the packet, so add it in
        if(
            !(ui8Flags | BQ_REQ_TYPE_BC_RESP)  &&
            !(ui8Flags | BQ_REQ_TYPE_BC_NORESP)
        ) {
            g_pui8WriteBuf[i++] = ui8Addr;
        }

        // data
        for(int j=0; j<ui8Len; j++)
            g_pui8WriteBuf[i++] = pui8Data[j];

        // checksum
        ui16Checksum = bq76_checksum(g_pui8WriteBuf, ui8Len);
        g_pui8WriteBuf[i++] = ui16Checksum >> 8;
        g_pui8WriteBuf[i++] = ui16Checksum & 0xFF;

        // write the buffer
        for(int j=0; j<i; j++)
            UARTCharPut(BQUART_MODULE, g_pui8WriteBuf[j]);
    } else {} // TODO: throw a fault, we shouldn't be transmitting without the command flag

}



/**
 * Writes a value to a register of one or more bq76 modules
 */
void bq76_writeReg(
        uint8_t ui8Flags,
        uint8_t ui8Addr,
        uint16_t ui16Reg,
        uint8_t ui8Data)
{
    uint8_t data[3];
    int i=0;

    if(ui8Flags & BQ_ADDR_SIZE_16)
        data[i++] = ui16Reg >> 8;
    data[i++] = ui16Reg & 0xFF;
    data[i++] = ui8Data;

    bq76_write(ui8Flags, i, ui8Addr, data);
}




/**
 * Executes the auto addressing sequence to the onboard BQ76 modules
 */
uint8_t bq76_autoAddress() {

}









