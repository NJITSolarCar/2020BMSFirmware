/*
 * bq76.c
 *
 *  Created on: Nov 10, 2018
 *      Author: Duemmer
 */

#include <stdint.h>
#include <stdbool.h>

#include <driverlib/uart.h>
#include <driverlib/gpio.h>
#include <driverlib/debug.h>
#include <driverlib/timer.h>


#include "bq.h"
#include "fault.h"
#include "types.h"
#include "util.h"

static uint8_t g_pui8ReadBuf[BQ_READ_BUF_SIZE]; // Read buffer for UART frames

static uint32_t g_ui32ReadBufPtr = 0; // Relative pinter to tail of the read buffer

static bool g_bCmdInProgress = false; // If true, a command is currently in progress

static bool g_bCmdHasResponse = false; // If true, the current command expects a response

static tSampleType g_sSampleType = NONE; // Current voltage sampling mode

// MCU peripheral data
static uint32_t g_ui32ReadDoneVec; // IRQ vector number to trigger when a UART read is finished
static uint32_t g_ui32UARTBase; // Base address of the UART peripheral used for bq76 communications
static uint32_t g_ui32WTimerBase; // Base address of the general purpose wide timer allocated for BQ use


/**
 * Converts a numeric transmission frame length into the form
 * required by the frame header
 */
static inline uint8_t bq76_len2frame(uint8_t ui8Len) {
    if(ui8Len < 7)
        return ui8Len;
    else
        return BQ_DATA_SIZE_8;
}




/**
 * CRC-16-IBM checksum algorithm, as used by the bq76 to verify user
 * transmissions. Copied from the BQ76 datasheet, section 7.5.2 on page
 * 55
 */
uint16_t bq76_checksum(uint8_t *pui8Buf, uint16_t ui16Len) {
    uint16_t crc = 0;
    while (ui16Len--) {
    crc ^= *pui8Buf++;
    for (uint16_t j = 0; j < 8; j++)
    crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
    }
    return crc;
}


/**
 * General interrupt routine for processing interrupts
 * from the bq76 UART module.
 */
void _bq76_uartISR() {
    // if we make it this far we won't time out
    TimerDisable(g_ui32WTimerBase, TIMER_A);

    // Copy data to read buffer
    while(UARTCharsAvail(g_ui32UARTBase))
        g_pui8ReadBuf[g_ui32ReadBufPtr++] = UARTCharGet(g_ui32UARTBase) & 0xFF;

    // Receive timed out, so we can assume its done
    if(UARTIntStatus(g_ui32UARTBase, 1) & UART_INT_RT) {
        g_bCmdInProgress = false;
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
    g_bCmdInProgress = false;
    if(g_bCmdHasResponse) {
        tFaultInfo sInfo;
        sInfo.ui64TimeFlagged = util_usecs();
        sInfo.data.ui32 = 0; // Not really any data we can package with this
        fault_setFault(FAULT_BQ_COM, sInfo);
    }
}




/**
 * Writes a frame of data to the BQ76 bank over UART. At the moment,
 * this always returns true, but this may change as more fault handling
 * capabilities are added.
 */
bool bq76_write(
        uint8_t ui8Flags,
        uint8_t ui8Len,
        uint8_t ui8Addr,
        uint8_t *pui8Data)
{
    uint8_t pui8Writebuf[BQ_WRITE_BUF_SIZE];
    uint16_t ui16Checksum;
    uint32_t i = 0; // write buffer pointer

    // Wait until we're ready to run a command, then reset global flags
    bq76_waitResponse();
    g_ui32ReadBufPtr = 0;
    g_bCmdInProgress = true;
    g_bCmdHasResponse =
            !(ui8Flags & BQ_REQ_TYPE_SGL_NORESP)    &&
            !(ui8Flags & BQ_REQ_TYPE_GRP_NORESP)    &&
            !(ui8Flags & BQ_REQ_TYPE_BC_NORESP);

    // Build write buffer
    if(ui8Flags & BQ_FRM_TYPE_COMMAND) {

        // header
        ui8Flags |= bq76_len2frame(ui8Len);
        pui8Writebuf[i++] = ui8Flags;

        // if we aren't broadcasting, we need to have an address byte
        // in the packet, so add it in
        if(
            !(ui8Flags | BQ_REQ_TYPE_BC_RESP)  &&
            !(ui8Flags | BQ_REQ_TYPE_BC_NORESP)
        ) {
            pui8Writebuf[i++] = ui8Addr;
        }

        // data
        for(int j=0; j<ui8Len; j++)
            pui8Writebuf[i++] = pui8Data[j];

        // checksum
        ui16Checksum = bq76_checksum(pui8Writebuf, ui8Len);
        pui8Writebuf[i++] = ui16Checksum >> 8;
        pui8Writebuf[i++] = ui16Checksum & 0xFF;

        // write the buffer
        for(int j=0; j<i; j++)
            UARTCharPut(g_ui32UARTBase, pui8Writebuf[j]);

        // Run the timeout timer
        TimerLoadSet(g_ui32UARTBase, TIMER_A, BQ_WRITE_TIMEOUT);
        TimerEnable(g_ui32UARTBase, TIMER_A);
    } else {} // TODO: throw a fault, we shouldn't be transmitting without the command flag
    return true; // TODO: add proper return exception handling
}



/**
 * Writes a value to a register of one or more bq76 modules. Can
 * write a variable number of bytes to the register, from 0 to
 * BQ_WRITEREG_MAX_MSG. If ui8DataLength is greater than BQ_WRITEREG_MAX_MSG,
 * then the write will abort and return 0. Otherwise, it will return
 * whatever bq76_write returns.
 */
bool bq76_writeReg(
        uint8_t ui8Flags,
        uint8_t ui8Addr,
        uint16_t ui16Reg,
        uint8_t ui8Len,
        uint8_t *pui8Data)
{
    if(ui8Len > BQ_WRITEREG_MAX_MSG) {
        // TODO: Assert proper software fault, this is illegal
        return false;
    }

    // 2 for register address, the rest for the max possible message length
    uint8_t pui8WriteBuf[2 + BQ_WRITEREG_MAX_MSG];
    int i=0;

    if(ui8Flags & BQ_ADDR_SIZE_16)
        pui8WriteBuf[i++] = ui16Reg >> 8;
    pui8WriteBuf[i++] = ui16Reg & 0xFF;

    for(int j=0; j<ui8Len; j++)
        pui8WriteBuf[i++] = pui8Data[j];

    return bq76_write(ui8Flags, i, ui8Addr, pui8WriteBuf);
}




/**
 * Extracts information from a raw response stored in the read buffer,
 * starting with ui8Start
 * Checks that the response checksum is correct, and that a command with response
 * is not in progress, and provides the caller with the response
 * length.
 * Returns true if the checksum is correct and the command
 * in progress flag is false, false otherwise. If a zero flag is
 * returned, there is no guarantee that the calculated length is correct
 */
bool bq76_parseResponse(uint8_t ui8Start, uint8_t *pui8Len) {

    // if a response command is in progress, the buffer may be clobbered
    if(g_bCmdInProgress && g_bCmdHasResponse)
        return false;

    *pui8Len = g_pui8ReadBuf[ui8Start];

    // Extract the checksum
    uint16_t ui16RespChecksum = g_pui8ReadBuf[*pui8Len + ui8Start] << 8;
    ui16RespChecksum |= g_pui8ReadBuf[*pui8Len + 1 + ui8Start];

    // Now compare it with the calculated checksum
    uint16_t ui16CalcChecksum = bq76_checksum(g_pui8ReadBuf+ui8Start, *pui8Len);
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
bool bq76_parseMultiple(
        uint8_t ui8NumResponses,
        uint8_t *pui8StartPtrs,
        uint8_t *pui8Lengths,
        uint8_t *pui8ChecksumGood)
{
    if(g_bCmdInProgress && g_bCmdHasResponse)
        return false;

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
            (ui16Reg & 0xFF00) ? BQ_ADDR_SIZE_16 : BQ_ADDR_SIZE_8;

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

    return false; // Bad checksum or response timeout
}




/**
 * Waits until the g_ui8CmdInProgress flag goes low,
 * or a timeout occurs. The flag should be automatically set
 * after a command is sent, and reset after either the entire
 * response is received (if applicable), or a certain amount
 * of time has elapsed (for no response commands).
 *
 * The timeout is determined based on whether or not the command in
 * progress expects a response
 *
 * Returns true if it timed out waiting for a response,
 * false if the flag was cleared.
 *
 * NOTE: This will never affect flag or rx buffer state!
 */
bool bq76_waitResponse() {
    uint64_t ui64Started = util_usecs();
    uint64_t ui64Timeout = BQ_WRITE_TIMEOUT;
    while(g_bCmdInProgress) {
        if(util_usecs() + ui64Timeout > ui64Started)
            return true;
    }
    return false;
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
bool bq76_autoAddress(uint32_t ui32nModules) {
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
            2,
            (uint8_t *) &ui16ComCfgDat);

    // Put Devices into Auto-Address Learning Mode
    pui8RegData[0] = 1 << 4;
    bq76_writeReg(
            BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_BC_NORESP,
            0,
            BQ_REG_DEVCONFIG,
            1,
            pui8RegData);

    pui8RegData[0] = 1 << 3;
    bq76_writeReg(
            BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_BC_NORESP,
            0,
            BQ_REG_DEV_CTRL,
            1,
            pui8RegData);

    // Assign addresses to each potential device
    for(int i=0; i<BQ_MAX_NUM_MODULE; i++) {
        pui8RegData[0] = i;
        bq76_writeReg(
            BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_BC_NORESP,
            0,
            BQ_REG_ADDR,
            1,
            pui8RegData);
    }

    // Verify that the modules are responding correctly. If any one doesn't respond,
    // assert a level 1 (non-halting fatal) fault
    uint8_t ui8AddrResponse;
    for(int i=0; i<ui32nModules; i++) {
        if(!bq76_readRegSingle( // Read failed
                i,
                BQ_REG_ADDR,
                0,
                &ui8AddrResponse))
        {
            // Failed to address the modules.
            // TODO: assert a fault
            return false;
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
        ui32nModules-1,
        BQ_REG_COMCONFIG,
        2,
        pui8RegData);

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
        2,
        pui8RegData);

    // Clear all faults on each device, starting from the top down.
    // See Section 1.2.6 of the software developer's manual for
    // reference on these data values
    pui8RegData[0] = 0xFF;
    pui8RegData[1] = 0xC0;
    for(int i=0; i<BQ_MAX_NUM_MODULE; i++) {
        bq76_writeReg(
            BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_SGL_NORESP,
            i,
            BQ_REG_FAULT_SUM,
            2,
            pui8RegData);
    }

    // At this point the devices should be ready to go with further
    // configuration
    return true;
}


/**
 * Tries to start running a cell voltage sample with response on the
 * BQ76 stack. Will Configure the sampling channels, and send a
 * broadcast write request, using the current BQ configurations.
 * will return true if sampling is started, false otherwise
 */
bool bq76_StartCellVoltageSample(tConf *psConf) {
    if(g_bCmdInProgress)
        return false;

    bool bALlGood = true;

    // Set the cells to sample on each device. Broadcast a read all
    // channels select, then on the last one, round it out to the correct
    // number of cells
    uint32_t ui32SampleConf = 0xFFFF0000;
    bALlGood = bq76_writeReg(
        BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_BC_NORESP,
        0,
        BQ_REG_CHANNELS,
        4,
        (uint8_t *)&ui32SampleConf);

    // Determine the mask for the last module
    uint32_t ui32nCellsLast = psConf->ui32nCells % BQ_MAX_SAMPLE;
    ui32SampleConf = 0;
    for(int i=0; i<ui32nCellsLast; i++)
        ui32SampleConf |= 1 << (i+16);

    bALlGood = bq76_writeReg(
        BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_SGL_NORESP,
        psConf->ui32NumBQModules-1,
        BQ_REG_CHANNELS,
        4,
        (uint8_t *)&ui32SampleConf);

    if(!bALlGood)
        return false;

    // Send the broadcast cell sample command on all devices
    uint8_t ui8Data = BQ_CMD_SAMPLE | (psConf->ui32NumBQModules & 0xF);
    bALlGood = bq76_writeReg(
        BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_BC_RESP,
        0,
        BQ_REG_CMD,
        1,
        &ui8Data);

    return bALlGood;
}




/**
 * Tries to start running a thermo voltage sample with response on the
 * BQ76 stack. Will Configure the sampling channels, and send a
 * broadcast write request, using the current BQ configurations.
 * will return true if sampling is started, false otherwise
 */
bool bq76_startThermoSample(tConf *psConf, bool bMuxState) {
    if(g_bCmdInProgress)
        return false;

    bool bALlGood = true;

    // Set the channels to sample on each device
    uint32_t  ui32SampleConf = 0xFF00;

    bALlGood = bq76_writeReg(
        BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_BC_NORESP,
        0,
        BQ_REG_CHANNELS,
        4,
        (uint8_t *)&ui32SampleConf);

    if(!bALlGood)
        return false;

    // Send the broadcast cell sample command on all devices
    uint8_t ui8Data = BQ_CMD_SAMPLE | (psConf->ui32NumBQModules & 0xF);
    bALlGood = bq76_writeReg(
        BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_BC_RESP,
        0,
        BQ_REG_CMD,
        1,
        &ui8Data);

    if(bALlGood)
        g_sSampleType = bMuxState ? THERM2 : THERM1;
    else
        g_sSampleType = NONE;

    return bALlGood;
}


bool bq76_setCellVoltageThreshold(float fUnder, float fOver) {

    // The registers are adjacent, can do it in one write
    uint16_t pui16Conf[2];
    pui16Conf[0] = BQ_VOLTS_TO_ADC(fUnder);
    pui16Conf[1] = BQ_VOLTS_TO_ADC(fOver);

    return bq76_writeReg(
            BQ_FRM_TYPE_COMMAND | BQ_REQ_TYPE_BC_NORESP,
            0,
            BQ_REG_CELL_UV,
            4,
            (uint8_t *)pui16Conf);
}


