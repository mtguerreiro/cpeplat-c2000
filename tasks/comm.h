/*
 * comm.h
 *
 *  Created on: 06.05.2021
 *      Author: local_marco
 */

#ifndef TASKS_COMM_H_
#define TASKS_COMM_H_

//=============================================================================
/*-------------------------------- Includes ---------------------------------*/
//=============================================================================
#include <xdc/std.h>

//=============================================================================

//=============================================================================
/*--------------------------------- Defines ---------------------------------*/
//=============================================================================
#define COMM_CONFIG_SERIAL_BUFFER_SIZE      50
//=============================================================================

//=============================================================================
/*---------------------------------- Task -----------------------------------*/
//=============================================================================
void comm(UArg a0, UArg a1);
//=============================================================================

#endif /* TASKS_COMM_H_ */
