/*
 * log.h
 *
 * Provides functions to periodically report system data, flags, and metrics to
 * outside resources. These will include file logging on the SD card, and possibly
 * broadcasting the information over the communication interfaces.
 *
 * LOGGING FORMAT:
 * %HH:mm:SS.ssssss [MSG_TYPE]\t"MSG"\n
 *
 * Where the timestamp represents the uptime
 *
 *  Created on: Dec 23, 2018
 *      Author: Duemmer
 */

#ifndef LOG_H_
#define LOG_H_

/**
 * Opens a new log file and prepares for writing to it
 */
uint8_t log_startLogging();

/**
 * Writes a message to the log file at the current time, with the specified
 * message type and message to display. Message / message type are null terminated
 * strings.
 */
uint8_t log_addMsg(const char *pcmsgType, char *pcMsg);


/**
 * Formats a message into a character buffer that can be written to the log file.
 * Returns the length of the data.
 */
uint32_t log_fmtMsg(
        char *pcDst,
        uint32_t ui32BufSize,
        uint64_t ui64Time,
        const char *pcMsgType,
        char *pcMsg);

#endif /* LOG_H_ */
