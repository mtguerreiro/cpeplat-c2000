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
    PLAT_CMD_CPU1_CPU2_PWM_ENABLE,
    PLAT_CMD_CPU1_CPU2_PWM_DISABLE,
    PLAT_CMD_CPU1_ADC_B4_READ,
    PLAT_CMD_CPU1_READ_RAM,
    PLAT_CMD_CPU1_END
}platCPU1CommandsEnum_t;

/* CPU2 commands */
typedef enum{
    PLAT_CMD_CPU2_BLINK = 0,
    PLAT_CMD_CPU2_GPIO,
    PLAT_CMD_CPU2_PWM_ENABLE,
    PLAT_CMD_CPU2_PWM_DISABLE,
    PLAT_CMD_CPU2_END
}platCPU2CommandsEnum_t;

/* GPIOs for CPU1 */
#define PLAT_CPU1_LED               31U

/* GPIOs for CPU2 */
#define PLAT_CPU2_LED               34U
#define PLAT_CPU2_GPIO_0            8U
#define PLAT_CPU2_GPIO_1            9U

/* IPC flag for commands */
#define PLAT_IPC_FLAG_CMD           0U
#define PLAT_IPC_FLAG_CPU1_INIT     16U
#define PLAT_IPC_FLAG_CPU2_INIT     17U
#define PLAT_IPC_FLAG_MEM_OWN       18U

/* RAM section for CPU2->CPU1 data exchange */
#define PLAT_CPU2_CPU1_RAM_SEC      (MEMCFG_SECT_GS14 | MEMCFG_SECT_GS15)
#define PLAT_CPU2_CPU1_RAM_ADD      0x0001A000
#define PLAT_CPU2_CPU1_RAM_SIZE     8192
//=============================================================================

#endif /* PLAT_DEFS_H_ */
