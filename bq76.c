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
#include "system.h"

// Read buffer for UART frames
static uint8_t g_pui8ReadBuf[BQ_READ_BUF_SIZE];

// If true, a command is currently in progress
static uint8_t g_ui8CmdInProgress = 0;


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
    uint8_t pui8Readbuf[BQ_READ_BUF_SIZE];
    uint16_t ui16Checksum;
    uint32_t i = 0; // buffer pointer

    if(ui8Flags & BQ_FRM_TYPE_COMMAND) {

        // header
        ui8Flags |= bq76_len2frame(ui8Len);
        pui8Readbuf[i++] = ui8Flags;

        // if we aren't broadcasting, we need to have an address byte
        // in the packet, so add it in
        if(
            !(ui8Flags | BQ_REQ_TYPE_BC_RESP)  &&
            !(ui8Flags | BQ_REQ_TYPE_BC_NORESP)
        ) {
            pui8Readbuf[i++] = ui8Addr;
        }

        // data
        for(int j=0; j<ui8Len; j++)
            pui8Readbuf[i++] = pui8Data[j];

        // checksum
        ui16Checksum = bq76_checksum(g_pui8WriteBuf, ui8Len);
        pui8Readbuf[i++] = ui16Checksum >> 8;
        pui8Readbuf[i++] = ui16Checksum & 0xFF;

        // write the buffer
        for(int j=0; j<i; j++)
            UARTCharPut(BQUART_MODULE, pui8Readbuf[j]);
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



uint8_t bq76_waitResponse(uint32_t ui32Timeout) {

}





/**
 * Executes the auto addressing sequence to the onboard BQ76 modules.
 * Taken from the  BQ76 Software Developer's reference, pg. 2
 */
uint8_t bq76_autoAddress() {
    // Fully Enable Differential Interfaces and Select Auto-Addressing Mode
    uint16_t ui16ComCfgDat =
            BQ_BAUD_INIT_CODE   |
            (1 << 7)            |   // UART transmission enable
            (1 << 6)            |   // high side receiver enable
            (1 << 5);               // low side transmitter enable

    bq76_writeReg(
            BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_BC_NORESP,
            0,
            BQ_REG_COMCONFIG,
            ui16ComCfgDat >> 8);

    bq76_writeReg(
            BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_BC_NORESP,
            0,
            BQ_REG_COMCONFIG+1,
            ui16ComCfgDat & 0xFF);

    // Put Devices into Auto-Address Learning Mode
    bq76_writeReg(
            BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_BC_NORESP,
            0,
            BQ_REG_DEVCONFIG,
            1 << 4);

    bq76_writeReg(
            BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_BC_NORESP,
            0,
            BQ_REG_DEV_CTRL,
            1 << 3);

    // Assign addresses to each device
    for(int i=0; i<BQ_NUM_MODULES; i++) {
        bq76_writeReg(
            BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_BC_NORESP,
            0,
            BQ_REG_ADDR,
            i);
    }

    // Verify that the modules are responding correctly. If any one doesn't respond,
    // assert a level 1 (non-halting fatal) fault
}









