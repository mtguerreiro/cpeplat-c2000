/*
 * main.c
 *
 *  Created on: 01.12.2020
 *      Author: mguerreiro
 */

//=============================================================================
/*-------------------------------- Includes ---------------------------------*/
//=============================================================================
/* Kernel */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* Device */
#include "inc/hw_ipc.h"
#include "memcfg.h"

/* Tasks */
#include "tasks/blink.h"
#include "tasks/comm.h"
#include "tasks/ctl.h"

/* Libs */
#include "plat_defs.h"
//=============================================================================

//=============================================================================
/*------------------------------- Prototypes --------------------------------*/
//=============================================================================
static void mainInitialize(void);
static void mainInitializeCPU2(void);
//=============================================================================

//=============================================================================
/*----------------------------------- Main ----------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
void main(void){

    Task_Params taskParams;

    /* Hardware and software initializations */
    mainInitialize();

    /* Creates tasks */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 1024;
    taskParams.priority = 5;
    Task_create((Task_FuncPtr)blink, &taskParams, NULL);

    Task_Params_init(&taskParams);
    taskParams.stackSize = 512;
    taskParams.priority = 10;
    Task_create((Task_FuncPtr)comm, &taskParams, NULL);

    Task_Params_init(&taskParams);
    taskParams.stackSize = 512;
    taskParams.priority = 12;
    Task_create((Task_FuncPtr)ctl, &taskParams, NULL);

    /* Starts the scheduler */
    BIOS_start();

    while(1);
}
//-----------------------------------------------------------------------------
//=============================================================================

//=============================================================================
/*---------------------------- Static functions -----------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
static void mainInitialize(void){

    /* Initialize device clock and peripherals */
    Device_init();

    Device_initGPIO();

    mainInitializeCPU2();
}
//-----------------------------------------------------------------------------
static void mainInitializeCPU2(void){

    /* Tells CPU2 to boot from flash */
    Device_bootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);

    /* Initializes second LED for CPU2 */
    GPIO_setPadConfig(PLAT_CPU2_LED, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(PLAT_CPU2_LED, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(PLAT_CPU2_LED, GPIO_CORE_CPU2);

    /* Initializes CPU2's GPIOs */
    GPIO_setPadConfig(PLAT_CPU2_GPIO_0, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(PLAT_CPU2_GPIO_0, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(PLAT_CPU2_GPIO_0, GPIO_CORE_CPU2);

    GPIO_setPadConfig(PLAT_CPU2_GPIO_1, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(PLAT_CPU2_GPIO_1, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(PLAT_CPU2_GPIO_1, GPIO_CORE_CPU2);

    /* Gives ownership of RAM sections GS14 and GS15 to CPU2 */
    MemCfg_setGSRAMMasterSel(MEMCFG_SECT_GS14 | MEMCFG_SECT_GS15, MEMCFG_GSRAMMASTER_CPU2);

    /* Signals given ownership of RAM section GS14 and GS15 to CPU 2 */
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_MEM_OWN;

    /* Wait until CPU2 is ready */
    while( !(HWREG(IPC_BASE + IPC_O_STS) & (1UL << PLAT_IPC_FLAG_CPU2_INIT)) );

    /* Acks the IPC flag */
    HWREG(IPC_BASE + IPC_O_ACK) = 1UL << PLAT_IPC_FLAG_CPU2_INIT;
    HWREG(IPC_BASE + IPC_O_CLR) = 1UL << PLAT_IPC_FLAG_CPU2_INIT;
}
//-----------------------------------------------------------------------------
//=============================================================================
