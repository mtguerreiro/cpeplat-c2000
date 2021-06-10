/*
 * plat_defs.h
 *
 *  Created on: 10.06.2021
 *      Author: mguerreiro
 */

#ifndef PLAT_DEFS_H_
#define PLAT_DEFS_H_

#include "device.h"

//=============================================================================
/*--------------------------------- Defines ---------------------------------*/
//=============================================================================


/* CPU1 commands */
typedef enum{
    PLAT_CMD_CPU1_BLINK = 1,
    PLAT_CMD_CPU1_CPU2_BLINK,
    PLAT_CMD_CPU1_CPU2_GPIO,
    PLAT_CMD_CPU1_END
}platCPU1CommandsEnum_t;

///* CPU1 to CPU2 commands */
//typedef enum{
//    PLAT_CMD_C1C2_BLINK = 0,
//    PLAT_CMD_C1C2_GPIO = 1,
//    PLAT_CMD_C1C2_END
//}platC1C2CommandsEnum_t;

/* CPU2 commands */
typedef enum{
    PLAT_CMD_CPU2_BLINK = 0,
    PLAT_CMD_CPU2_GPIO = 1,
    PLAT_CMD_CPU2_END
}platCPU2CommandsEnum_t;


/* GPIOs for CPU1 */
#define PLAT_CPU1_LED           31U

/* GPIOs for CPU2 */
#define PLAT_CPU2_LED           34U
#define PLAT_CPU2_GPIO_0        8U
#define PLAT_CPU2_GPIO_1        9U

/* IPC flag for commands */
#define PLAT_IPC_FLAG_CMD       0U
#define PLAT_IPC_FLAG_INIT      17U
//=============================================================================

#endif /* PLAT_DEFS_H_ */