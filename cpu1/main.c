/*
 * main.c
 *
 *  Created on: 01.12.2020
 *      Author: LRS
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



#include "F2837xD_GlobalVariableDefs.c"         //adjust linker File  according to "F2837xD_Headers_nonBIOS_cpu1.cmd"
//=============================================================================

//=============================================================================
/*------------------------------- Prototypes --------------------------------*/
//=============================================================================
static void mainInitialize(void);
static void mainInitializeCPU2(void);
static void mainInitializeCPU2GPIO(void);
static void mainInitializeCPU2PWM(void);
static void mainInitializeCPU2ADC(void);
static void mainInitializeCPU2Memory(void);
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
    taskParams.stackSize = BLINK_CONFIG_TASK_STACK_SIZE;
    taskParams.priority = BLINK_CONFIG_TASK_PRIO;
    Task_create((Task_FuncPtr)blink, &taskParams, NULL);

    Task_Params_init(&taskParams);
    taskParams.stackSize = COMM_CONFIG_TASK_STACK_SIZE;
    taskParams.priority = COMM_CONFIG_TASK_PRIO;
    Task_create((Task_FuncPtr)comm, &taskParams, NULL);

    Task_Params_init(&taskParams);
    taskParams.stackSize = CTL_CONFIG_TASK_STACK_SIZE;
    taskParams.priority = CTL_CONFIG_TASK_PRIO;
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
    //Device_bootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);

    /* CPU2's peripherals initialization */
    mainInitializeCPU2GPIO();
    mainInitializeCPU2PWM();
    mainInitializeCPU2ADC();
    mainInitializeCPU2Memory();

    /* Acks and clears CPU2->CPU1 data IPC flag */
    HWREG(IPC_BASE + IPC_O_ACK) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
    HWREG(IPC_BASE + IPC_O_CLR) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;

    /*
     * Signals to CPU2 that CPU1 initialized already (all required peripheral
     * ownerships were given).
     */
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU1_INIT;

    /* Now, wait until CPU2 is ready */
    while( !(HWREG(IPC_BASE + IPC_O_STS) & (1UL << PLAT_IPC_FLAG_CPU2_INIT)) );

    /* Acks the IPC flag */
    HWREG(IPC_BASE + IPC_O_ACK) = 1UL << PLAT_IPC_FLAG_CPU2_INIT;
    HWREG(IPC_BASE + IPC_O_CLR) = 1UL << PLAT_IPC_FLAG_CPU2_INIT;
}
//-----------------------------------------------------------------------------
static void mainInitializeCPU2GPIO(void){

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
}
//-----------------------------------------------------------------------------
static void mainInitializeCPU2PWM(void){

    /* Transfer ownerships of EPWM2 and EPWM4 to CPU2 */
    EALLOW;

    DevCfgRegs.CPUSEL0.bit.EPWM2 = 1;
    DevCfgRegs.CPUSEL0.bit.EPWM4 = 1;

    //
    // Disable internal pull-up for the selected output pins
    //   for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up on GPIO6 (EPWM4A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;    // Disable pull-up on GPIO7 (EPWM4B)
    // GpioCtrlRegs.GPEPUD.bit.GPIO151 = 1;    // Disable pull-up on GPIO151 (EPWM4A)
    // GpioCtrlRegs.GPEPUD.bit.GPIO152 = 1;    // Disable pull-up on GPIO152 (EPWM4B)

     //
     // Configure EPWM-4 pins using GPIO regs
     // This specifies which of the possible GPIO pins will be EPWM4 functional
     // pins.
     // Comment out other unwanted lines.
     //
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B
    // GpioCtrlRegs.GPEMUX2.bit.GPIO151 = 1;   // Configure GPIO151 as EPWM4A
    // GpioCtrlRegs.GPEMUX2.bit.GPIO152 = 1;   // Configure GPIO152 as EPWM4B

    EDIS;
}
//-----------------------------------------------------------------------------
static void mainInitializeCPU2ADC(void){

    /* Transfers ownership of ADC to CPU2 */
    EALLOW;

    DevCfgRegs.CPUSEL11.bit.ADC_A = 1;
    DevCfgRegs.CPUSEL11.bit.ADC_B = 1;
    DevCfgRegs.CPUSEL11.bit.ADC_C = 1;

    EDIS;
}
//-----------------------------------------------------------------------------
static void mainInitializeCPU2Memory(void){

    /* Gives ownership of selected RAM sections to CPU2 */
    MemCfg_setGSRAMMasterSel(PLAT_CPU2_BUFFER_RAM_SEC, MEMCFG_GSRAMMASTER_CPU2);
}
//-----------------------------------------------------------------------------
//=============================================================================
