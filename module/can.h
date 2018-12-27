/*
 * can.h
 *
 * Provides a driver for CAN communications using the CAN interface in
 * the mcu, along with the TCAN1042H transceiver chip. This should provide
 * basic HAL, initialization, and messaging features, but should not
 * worry about implementing the formal solar car CAN protocol, which will be
 * implemented externally.
 *
 *  Created on: Dec 23, 2018
 *      Author: Duemmer
 */

#ifndef CAN_H_
#define CAN_H_

#include <stdint.h>

typedef struct
{
    uint16_t id;
    uint16_t ide;
    struct {
        int8_t rtr : 1;
        int8_t ide : 1;
        uint8_t length : 4;
    } header;
    uint8_t data[8];
} tCAN;


/**
 * Initializes the CAN communication system at the specified
 * baud rate. Returns a nonzero number on success, 0 otherwise
 */
uint8_t can_init(uint32_t ui32Baud);


/**
 * Flushes any buffers, closes the interface, and restarts the interface
 */
uint8_t can_reset();


/**
 * Transmits a frame onto the bus. Returns a nonzero value on success,
 * 0 otherwise
 */
uint8_t can_sendFrame(tCAN *frame);


/**
 * Registers a function to call when frames are received on the bus. This should
 * run quickly, and will only be called on valid frames.
 */
void can_registerFrameHandler(void (*handler)(*tCAN) pfnHandler);

#endif /* CAN_H_ */
