/*
 * @file serial.h
 * @brief Implements a simple serial protocol.
 *
 * Here we consider that this device is a slave and replies to commands.
 *
 * A serial protocol is implemented here through a state machine. The
 * protocol is as follows:
 * 	- Start byte (1 byte - 0x55)
 * 	- ID - or cmd (4 bytes)
 * 	- Data size (4 bytes)
 * 	- Data (N bytes)
 * 	- Stop byte (1 byte - 0x77)
 *
 * Whenever data is successfully received, the state machine will call the
 * function corresponding to the ID received.
 *
 *  Created on: Mar 11, 2019
 *      Author: marco
 *
 */

#ifndef SERIAL_H_
#define SERIAL_H_

//===========================================================================
/*------------------------------- Includes --------------------------------*/
//===========================================================================
#include <stdint.h>

//===========================================================================

//===========================================================================
/*---------------------------- Critical section ---------------------------*/
//===========================================================================
#include "device.h"

#define SERIAL_CRITICAL_ENTER   DINT /**< Enters critical section. */
#define SERIAL_CRITICAL_EXIT    EINT    /**< Exits critical section. */

//===========================================================================

//===========================================================================
/*-------------------------------- Defines --------------------------------*/
//===========================================================================
/* Error codes */
#define SERIAL_ERR_INVALID_ID               -0x01
#define SERIAL_ERR_EXCEEDED_MAX_ID          -0x02
#define SERIAL_ERR_WRITE                    -0x03

#define SERIAL_CONFIG_SUPPORT_16BIT_BUFFER  1

#define SERIAL_CONFIG_IDS                   30
#define SERIAL_CONFIG_RX_TO                 1000
#define SERIAL_CONFIG_TX_TO                 1000

#define SERIAL_CONFIG_START_BYTE            0x55
#define SERIAL_CONFIG_STOP_BYTE             0x77
//===========================================================================

//===========================================================================
/*------------------------------- Data types ------------------------------*/
//===========================================================================
typedef struct{
    uint32_t size;
    uint8_t *buffer;
#if ( SERIAL_CONFIG_SUPPORT_16BIT_BUFFER == 1 )
    uint8_t bufferMode;
#endif
}serialDataExchange_t;

/*
 * Function executed when data is received. The buffer holding the data
 * received and the number of bytes received are passed to the function on
 * the data structure. Additionally, should any message be sent back, this
 * function should return 1, and write the number of bytes and the buffer
 * on the data structure. If there is no reply, this function must return 0.
 */
typedef uint32_t(*serialHandle_t)(serialDataExchange_t *data);

/*
 * Function to read a byte from the hardware peripheral. We expect the
 * function to take as argument a pointer and the TO. If the function was
 * able to read one byte from the hardware peripheral, then we expect this
 * function to return 0, and any other value if the no data was read from
 * the peripheral.
 */
typedef int32_t(*serialHWRead_t)(uint8_t *buffer, uint32_t to);

/*
 * Function to write a byte to the hardware peripheral. We expect the
 * function to take as argument a pointer and the TO. If the function was
 * able to write one byte to the hardware peripheral, then we expect this
 * function to return 0, and any other value if the no data was written to
 * the peripheral.
 */
typedef int32_t(*serialHWWrite_t)(uint8_t *buffer, uint32_t to);
//===========================================================================

//===========================================================================
/*------------------------------- Functions -------------------------------*/
//===========================================================================
void serialInitialize(uint8_t *buffer, uint32_t size, serialHWRead_t hwRead, serialHWWrite_t hwWrite);
uint8_t serialRun(void);
int32_t serialRegisterHandle(uint32_t id, serialHandle_t handle);
//===========================================================================

#endif /* SERIAL_H_ */
