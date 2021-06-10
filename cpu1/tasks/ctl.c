/*
 * ctl.c
 *
 *  Created on: 19.05.2021
 *      Author: mguerreiro
 */

/*
 * TODO: instead of having the relay commands, maybe it would be more useful to
 * have a generic set/reset GPIO command. The GPIO and states are sent through
 * the serial interface, and the board sets/resets the GPIO. The same could be
 * done to initialize a GPIO.
 */
//=============================================================================
/*-------------------------------- Includes ---------------------------------*/
//=============================================================================
#include "ctl.h"

/* Kernel */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Hwi.h>

/* Device and drivers */
#include "driverlib.h"
#include "inc/hw_ipc.h"

/* Libs */
#include "clibs/serial.h"
#include "plat_defs.h"

/* Tasks */
#include "tasks/blink.h"
//#include "clibs/adc.h"
//=============================================================================

//=============================================================================
/*--------------------------------- Globals ---------------------------------*/
//=============================================================================

//============================================================================

//=============================================================================
/*-------------------------------- Prototypes -------------------------------*/
//=============================================================================
static void ctlInitialize(void);

static uint32_t ctlCommandCPU1Blink(serialDataExchange_t *data);
static uint32_t ctlCommandCPU2Blink(serialDataExchange_t *data);
static uint32_t ctlCommandCPU2GPIO(serialDataExchange_t *data);

static void ctlIPCCommand(uint32_t command, uint32_t data);
//static uint32_t ctlCommandRelay1(serialDataExchange_t *data);
//static uint32_t ctlCommandRelay2(serialDataExchange_t *data);
//static uint32_t ctlCommandStartAcq(serialDataExchange_t *data);
//static uint32_t ctlCommandReadADCData(serialDataExchange_t *data);
//=============================================================================

//=============================================================================
/*---------------------------------- Task -----------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
void ctl(UArg a0, UArg a1){

    ctlInitialize();

    while(1){
        Task_sleep(1000);
    }
}
//-----------------------------------------------------------------------------
//=============================================================================

//=============================================================================
/*---------------------------- Static functions -----------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
static void ctlInitialize(void){

    /* Register commands */
    serialRegisterHandle(PLAT_CMD_CPU1_BLINK, ctlCommandCPU1Blink);
    serialRegisterHandle(PLAT_CMD_CPU1_CPU2_BLINK, ctlCommandCPU2Blink);
    serialRegisterHandle(PLAT_CMD_CPU1_CPU2_GPIO, ctlCommandCPU2GPIO);
}
//-----------------------------------------------------------------------------
static uint32_t ctlCommandCPU1Blink(serialDataExchange_t *data){

    uint32_t period;

    period = *data->buffer++ << 8;
    period = period | *data->buffer;

    blinkPeriodUpdate(period);

    return 0;
}
//-----------------------------------------------------------------------------
static uint32_t ctlCommandCPU2Blink(serialDataExchange_t *data){

    uint32_t period;

    period = *data->buffer++ << 8;
    period = period | *data->buffer;

    ctlIPCCommand(PLAT_CMD_CPU2_BLINK, period);

    return 0;
}
//-----------------------------------------------------------------------------
static uint32_t ctlCommandCPU2GPIO(serialDataExchange_t *data){

    uint32_t gpio;
    uint32_t state;
    uint32_t cmd;

    gpio = *data->buffer++;
    state = *data->buffer;
    cmd = gpio | (state << 16);

    ctlIPCCommand(PLAT_CMD_CPU2_GPIO, cmd);

    return 0;
}
//-----------------------------------------------------------------------------
//static uint32_t ctlCommandRelay1(serialDataExchange_t *data){
//
//    uint32_t val = 1;
//
//    if( *data->buffer == 0 ) val = 0;
//
//    GPIO_writePin(CTL_CONFIG_RELAY_1, val);
//
//    return 0;
//}
//-----------------------------------------------------------------------------
//static uint32_t ctlCommandRelay2(serialDataExchange_t *data){
//
//    uint32_t val = 1;
//
//    if( *data->buffer == 0 ) val = 0;
//
//    GPIO_writePin(CTL_CONFIG_RELAY_2, val);
//
//    return 0;
//}
//-----------------------------------------------------------------------------
//static uint32_t ctlCommandStartAcq(serialDataExchange_t *data){
//
//    adcStartAcq();
//
//    return 0;
//}
//-----------------------------------------------------------------------------
//static uint32_t ctlCommandReadADCData(serialDataExchange_t *data){
//
//    uint8_t *pADC;
//
//    pADC = adcGetBuffer();
//    data->buffer = pADC;
//    data->size = CTL_CONFIG_ADC_SAMPLES;
//
//    return 1;
//}
//-----------------------------------------------------------------------------
static void ctlIPCCommand(uint32_t command, uint32_t data){

    HWREG(IPC_BASE + IPC_O_SENDCOM) = command;
    HWREG(IPC_BASE + IPC_O_SENDDATA) = data;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CMD;
}
//-----------------------------------------------------------------------------
//=============================================================================
