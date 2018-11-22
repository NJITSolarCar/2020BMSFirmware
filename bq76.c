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
 * Writes a frame of data to the BQ76 bank over UART. At the moment,
 * this always returns 1, but this may change as more fault handling
 * capabilities are added.
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
    return 1; // TODO: add proper return exception handling
}



/**
 * Writes a value to a register of one or more bq76 modules. Can
 * write a variable number of bytes to the register, from 0 to
 * BQ_WRITEREG_MAX_MSG. If ui8DataLength is greater than BQ_WRITEREG_MAX_MSG,
 * then the write will abort and return 0. Otherwise, it will return
 * whatever bq76_write returns.
 */
uint8_t bq76_writeReg(
        uint8_t ui8Flags,
        uint8_t ui8Addr,
        uint16_t ui16Reg,
        uint8_t *ui8Data,
        uint8_t ui8DataLength)
{
    if(ui8DataLength > BQ_WRITEREG_MAX_MSG) {
        // TODO: Assert proper software fault, this is illegal
        return;
    }

    // 2 for register address, the rest for the max possible message length
    uint8_t pui8WriteBuf[2 + BQ_WRITEREG_MAX_MSG];
    int i=0;

    if(ui8Flags & BQ_ADDR_SIZE_16)
        ui8WriteBuf[i++] = ui16Reg >> 8;
    ui8WriteBuf[i++] = ui16Reg & 0xFF;

    for(int j=0; j<ui8DataLength; j++)
        pui8WriteBuf[i++] = ui8Data[j];

    return bq76_write(ui8Flags, i, ui8Addr, pui8WriteBuf);
}



/**
 * Utility method to make single byte register writes easier
 */
uint8_t bq76_writeReg(
        uint8_t ui8Flags,
        uint8_t ui8Addr,
        uint16_t ui16Reg,
        uint8_t ui8Data)
{
    return bq76_writeReg(ui8Flags, ui8Addr, ui16Reg, &ui8Data, 1);
}



/**
 * Extracts information from a raw response stored in the read buffer,
 * starting with ui8Start
 * Checks that the response checksum is correct, and that a command with response
 * is not in progress, and provides the caller with the response
 * length.
 * Returns a nonzero value if the checksum is correct and the command
 * in progress flag is false, zero otherwise. If a zero flag is
 * returned, there is no guarantee that the calculated length is correct
 */
uint8_t bq76_parseResponse(uint8_t ui8Start, uint8_t *pui8Len) {

    // if a response command is in progress, the buffer may be clobbered
    if(g_ui8CmdInProgress && g_ui8CmdHasResponse)
        return 0;

    *pui8Len = g_pui8ReadBuf[ui8Start];

    // shift checksum address by 1 to account for the frame init byte
    uint16_t ui16RespChecksum = g_pui8ReadBuf[*pui8Len + 1 + ui8Start] << 8;
    ui16RespChecksum |= g_pui8ReadBuf[*pui8Len + 2 + ui8Start];

    // Now compare it with the calculated checksum
    uint16_t ui16CalcChecksum = bq76_checksum(g_pui8ReadBuf+1+ui8Start, *pui8Len);
    return ui16CalcChecksum == ui16RespChecksum;

}


/**
 * Parses the response of commands from multiple targets, from either broadcast
 * or group writes with responses. This will loop over the buffer, extracting the
 * lengths and start pointers of each block, up to ui8NumResponses.
 * In addition, it will individually verify the checksum of each response, returning
 * a 0 for a mismatch. Note that a read fault, and missed checksum,
 * could possibly shift the data, therefore corrupting all responses
 * past that point. Returns 0 if a command with response is in progress
 * or the read buffer is overrun, false otherwise.
 */
uint8_t bq76_parseMultiple(
        uint8_t ui8NumResponses,
        uint8_t *pui8StartPtrs,
        uint8_t *pui8Lengths,
        uint8_t *pui8ChecksumGood)
{
    if(g_ui8CmdInProgress && g_ui8CmdHasResponse)
        return 0;

    int iBufPtr = 0;
    uint8_t ui8RespOn = 0;
    while(ui8RespOn < ui8NumResponses &&
            iBufPtr < BQ_READ_BUF_SIZE)
    {
        // Verify the checksum, parse the block, and get its length
        pui8ChecksumGood[ui8RespOn] =
                bq76_parseResponse(iBufPtr, pui8Lengths+ui8RespOn);

        pui8StartPtrs[ui8RespOn] = (uint8_t) iBufPtr;

        // Advance the buffer pointer
        // add 3 bytes on top of data length: 2 for checksum,
        // 1 for frame initializer
        iBufPtr += pui8Lengths[ui8RespOn] + 3;
        ui8RespOn++;
    }

    // Verify any overruns
    return iBufPtr < BQ_READ_BUF_SIZE;
}




/**
 * Reads the value of a single (variable length) register on a single
 * bq76 module. Returns a nonzero value if the register was read
 * with a correct checksum, zero otherwise
 */
uint8_t bq76_readRegSingle(
        uint8_t ui8Addr,
        uint16_t ui16Reg,
        uint8_t ui8Len,
        uint8_t *pui8Response)
{
    // Single device write with response, either 8 or 16 bit
    // register addressing, based on register address size
    uint8_t ui8Flags =
            BQ_FRM_TYPE_COMMAND     |
            BQ_REQ_TYPE_SGL_RESP    |
            (ui16Reg & 0xFF00) ? BQ_ADDR_SIZE_16 :BQ_ADDR_SIZE_8;

    uint8_t pui8WriteBuf[5];
    int i=0;

    if(ui8Flags & BQ_ADDR_SIZE_16)
        pui8WriteBuf[i++] = ui16Reg >> 8;
    pui8WriteBuf[i++] = ui16Reg & 0xFF;
    pui8WriteBuf[i++] = ui8Len-1;

    bq76_write(ui8Flags, i, ui8Addr, pui8WriteBuf);

    uint8_t ui8RespLen;
    if(!bq76_waitResponse(BQ_WRITE_RESP_TIMEOUT) &&
        bq76_parseResponse(0, &ui8RespLen))
    { // got a response, copy it over
        for(int j=0; j<ui8RespLen; j++)
            pui8Response[j] = g_pui8ReadBuf[j+1];

        // if the lengths don't match, that's probably an issue,
        // so flag it in the return
        return ui8Len == ui8RespLen;
    }

    return 0; // Bad checksum or response timeout
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



/**
 * Waits unconditionally on a response to occur. Should be used with caution as
 * to avoid a lockup
 */
void bq76_waitResponse() {
    while(g_ui8CmdInProgress);
}





/**
 * Executes the auto addressing sequence to the onboard BQ76 modules.
 * Taken from the  BQ76 Software Developer's reference, pg. 2. This
 * will assign addresses starting from 0 up to BQ_NUM_MODULES-1, with
 * 0 being the device on the bottom of the stack (connected to the UART)
 * and BQ_NUM_MODULES-1 being the device at the top of the stack
 * (with no high side differential transmitter). In addition, this will
 * also clear all faults asserted on the modules.
 *
 * Returns a nonzero value if addressing was successful, 0 otherwise
 */
uint8_t bq76_autoAddress() {
    uint8_t pui8RegData[4]; // temp buffer for multi-byte register writes

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
            (uint8_t *) &ui16ComCfgDat,
            2);

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
    uint8_t ui8AddrResponse;
    for(int i=0; i<BQ_NUM_MODULES; i++) {
        if(!bq76_readRegSingle( // Read failed
                i,
                BQ_REG_ADDR,
                0,
                &ui8AddrResponse))
        {
            // Failed to address the modules.
            // TODO: assert a fault
            return 0
        }
    }

    // if we make it here, everything is addressed correctly

    // Turn off the high side of the differential interface on
    // the device on the top of the stack. See Section 1.2.5 of
    // the software developer's manual for reference on these data
    // values
    pui8RegData[0] = 0x10;
    pui8RegData[1] = 0x20;
    bq76_writeReg(
        BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_SGL_NORESP,
        BQ_NUM_MODULES-1,
        BQ_REG_COMCONFIG,
        pui8RegData,
        2);

    // Turn off the low side of the differential interface on
    // the device on the bottom of the stack. See Section 1.2.6 of
    // the software developer's manual for reference on these data
    // values
    pui8RegData[0] = 0x10;
    pui8RegData[1] = 0xC0;
    bq76_writeReg(
        BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_SGL_NORESP,
        0,
        BQ_REG_COMCONFIG,
        pui8RegData,
        2);

    // Clear all faults on each device, starting from the top down.
    // See Section 1.2.6 of the software developer's manual for
    // reference on these data values
    pui8RegData[0] = 0xFF;
    pui8RegData[1] = 0xC0;
    for(int i=0; i<BQ_NUM_MODULES; i++) {
        bq76_writeReg(
            BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_SGL_NORESP,
            i,
            BQ_REG_FAULT_SUM,
            pui8RegData,
            2);
    }

    // At this point the devices should be ready to go with further
    // configuration
    return 1;
}









