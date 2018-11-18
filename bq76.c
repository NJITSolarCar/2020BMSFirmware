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
#include <driverlib/timer.h>


#include "bq76.h"
#include "pinconfig.h"
#include "system.h"

static uint8_t g_pui8ReadBuf[BQ_READ_BUF_SIZE]; // Read buffer for UART frames

static uint32_t g_ui32ReadBufPtr = 0; // Relative pinter to tail of the read buffer

static uint8_t g_ui8CmdInProgress = 0; // If true, a command is currently in progress

static uint8_t g_ui8CmdHasResponse = 0; // If true, the current command expects a response


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
 * General interrupt routine for processing interrupts
 * from the bq76 UART module.
 */
void _bq76_uartISR() {
    uint8_t data;

    // if we make it this far we won't time out
    TimerDisable(BQ_RECV_TIMER, BQ_RECV_TIMER_PART);

    // Copy data to read buffer
    while(UARTCharsAvail(BQUART_MODULE) != -1)
        g_pui8ReadBuf[g_ui32ReadBufPtr++] = UARTCharGet(BQUART_MODULE) & 0xFF;

    // Receive timed out, so we can assume its done
    if(UARTIntStatus(BQUART_MODULE, 1) & UART_INT_RT) {
        g_ui8CmdInProgress = 0;
    }
}


/**
 * ISR that runs whenever the timeout period on a command expires.
 * For commands without response, this will happen normally, after
 * every command, and serves to give the bq76 chips some time to
 * process the command before receiving a new one. For systems with
 * response, however, the UART receive ISR should stop the timer, therefore
 * if this runs after a response command, we have a communication fault.
 */
void _bq76_timerISR() {
    g_ui8CmdHasResponse = 0;
    if(g_ui8CmdHasResponse)
        ; // TODO: Assert level 1 fault
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
    uint32_t i = 0; // write buffer pointer

    // Wait until we're ready to run a command, then reset global flags
    bq76_waitResponse();
    g_ui32ReadBufPtr = 0;
    g_ui8CmdInProgress = 1;
    g_ui8CmdHasResponse =
            !(ui8Flags & BQ_REQ_TYPE_SGL_NORESP)    &&
            !(ui8Flags & BQ_REQ_TYPE_GRP_NORESP)    &&
            !(ui8Flags & BQ_REQ_TYPE_BC_NORESP);

    // Build write buffer
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

        // Load timeout timer
        uint32_t ui32Timeout = g_ui8CmdHasResponse ?
                BQ_WRITE_RESP_TIMEOUT :
                BQ_WRITE_NORESP_TIMEOUT;

        TimerLoadSet(BQ_RECV_TIMER, BQ_RECV_TIMER_PART, ui32Timeout);
        TimerEnable(BQ_RECV_TIMER, BQ_RECV_TIMER_PART);
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
 * Waits until the g_ui8CmdInProgress flag goes low,
 * or a timeout occurs. The flag should be automatically set
 * after a command is sent, and reset after either the entire
 * response is received (if applicable), or a certain amount
 * of time has elapsed (for no response commands)
 *
 * Returns a nonzero value if it timed out waiting for a response,
 * zero if the flag was cleared.
 *
 * NOTE: This will never affect flag or rx buffer state!
 */
uint8_t bq76_waitResponse(uint32_t ui32Timeout) {
    while(g_ui8CmdInProgress) {
        // TODO: Implement a systick system and provide a callable
        // function to get usec time, to implement timeout
    }
    return 0;
}



void bq76_waitResponse() {
    while(g_ui8CmdInProgress);
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









