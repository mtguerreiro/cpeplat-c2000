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
/*--------------------------------- Defines ---------------------------------*/
//=============================================================================

/* CPU1 commands */
typedef struct{
    uint16_t adcA1[100];
    uint16_t adcB4[100];
    uint16_t adcA5[100];
    uint16_t i;
}ctlADCBuffer_t;

//=============================================================================

//=============================================================================
/*--------------------------------- Globals ---------------------------------*/
//=============================================================================
ctlADCBuffer_t ctlADCBuffer = {.i = 0};
//============================================================================

//=============================================================================
/*-------------------------------- Prototypes -------------------------------*/
//=============================================================================
static void ctlInitialize(void);

static uint32_t ctlCommandCPU1Blink(serialDataExchange_t *data);
static uint32_t ctlCommandCPU2Blink(serialDataExchange_t *data);
static uint32_t ctlCommandCPU2GPIO(serialDataExchange_t *data);
static uint32_t ctlCommandCPU2PWMEnable(serialDataExchange_t *data);
static uint32_t ctlCommandCPU2PWMDisable(serialDataExchange_t *data);
static uint32_t ctlCommandCPU1ReadADCB4(serialDataExchange_t *data);

static uint32_t ctlCommandReadMem(serialDataExchange_t *data);

static void ctlIPCCommand(uint32_t command, uint32_t data);
//static uint32_t ctlCommandRelay1(serialDataExchange_t *data);
//static uint32_t ctlCommandRelay2(serialDataExchange_t *data);
//static uint32_t ctlCommandStartAcq(serialDataExchange_t *data);
//static uint32_t ctlCommandReadADCData(serialDataExchange_t *data);

static __interrupt void ctlADCISR(void);
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
    serialRegisterHandle(PLAT_CMD_CPU1_CPU2_PWM_ENABLE, ctlCommandCPU2PWMEnable);
    serialRegisterHandle(PLAT_CMD_CPU1_CPU2_PWM_DISABLE, ctlCommandCPU2PWMDisable);
    serialRegisterHandle(PLAT_CMD_CPU1_ADC_B4_READ, ctlCommandCPU1ReadADCB4);

    serialRegisterHandle(PLAT_CMD_CPU1_READ_RAM, ctlCommandReadMem);



    /*
     * Enable ADC ISR. We don't want this interrupt to go through the
     * operating system, so we add it manually to the interrupt table. The
     * interrupt function should then have the __interrupt attribute in its
     * prototype, so all the context saving is done by the compiler.
     */
    Interrupt_register(INT_ADCA1, ctlADCISR);
    Interrupt_enable(INT_ADCA1);
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
static uint32_t ctlCommandCPU2PWMEnable(serialDataExchange_t *data){

    ctlADCBuffer.i = 0;

    ctlIPCCommand(PLAT_CMD_CPU2_PWM_ENABLE, 0);

    return 0;
}
//-----------------------------------------------------------------------------
static uint32_t ctlCommandCPU2PWMDisable(serialDataExchange_t *data){

    ctlIPCCommand(PLAT_CMD_CPU2_PWM_DISABLE, 0);

    return 0;
}
//-----------------------------------------------------------------------------
static uint32_t ctlCommandCPU1ReadADCB4(serialDataExchange_t *data){

    data->bufferMode = 1;
    data->buffer = (uint8_t *)ctlADCBuffer.adcB4;
    data->size = 100;

    return 1;
}
//-----------------------------------------------------------------------------
static uint32_t ctlCommandReadMem(serialDataExchange_t *data){

    data->buffer = (uint8_t *)0x0001A000;
    data->bufferMode = 1;
    data->size = 100;

    return 1;
}
//-----------------------------------------------------------------------------
static void ctlIPCCommand(uint32_t command, uint32_t data){

    HWREG(IPC_BASE + IPC_O_SENDCOM) = command;
    HWREG(IPC_BASE + IPC_O_SENDDATA) = data;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CMD;
}
//-----------------------------------------------------------------------------
//=============================================================================


//=============================================================================
/*---------------------------------- IRQs -----------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
static __interrupt void ctlADCISR(void){

    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

    if( ctlADCBuffer.i < 100 ){
        ctlADCBuffer.adcA1[ctlADCBuffer.i] = ADC_readResult(ADCA_BASE, (ADC_SOCNumber)0);
        ctlADCBuffer.adcA5[ctlADCBuffer.i] = ADC_readResult(ADCA_BASE, (ADC_SOCNumber)2);
        ctlADCBuffer.adcB4[ctlADCBuffer.i] = ADC_readResult(ADCB_BASE, (ADC_SOCNumber)0);
        ctlADCBuffer.i++;
    }
}
//-----------------------------------------------------------------------------
//=============================================================================
