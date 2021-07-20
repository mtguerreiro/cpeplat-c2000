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

#define CTL_CONFIG_ADC_N    6

typedef struct{
    uint16_t *buffer;
    uint32_t i;
    uint32_t size;
}ctlADCBuffer_t;
//=============================================================================

//=============================================================================
/*--------------------------------- Globals ---------------------------------*/
//=============================================================================
ctlADCBuffer_t ctlADCBuffer[CTL_CONFIG_ADC_N];
//=============================================================================

//=============================================================================
/*-------------------------------- Prototypes -------------------------------*/
//=============================================================================
static void ctlInitialize(void);

static uint32_t ctlADCBufferUpdate(void);

static void ctlIPCCommand(uint32_t command, uint32_t data);
static uint32_t ctlIPCDataReceive(uint32_t flag, uint32_t *data);

static uint32_t ctlCommandCPU1Blink(serialDataExchange_t *data);

static uint32_t ctlCommandCPU2Status(serialDataExchange_t *data);
static uint32_t ctlCommandCPU2StatusClear(serialDataExchange_t *data);
static uint32_t ctlCommandCPU2Blink(serialDataExchange_t *data);

static uint32_t ctlCommandCPU2GPIO(serialDataExchange_t *data);

static uint32_t ctlCommandCPU2PWMEnable(serialDataExchange_t *data);
static uint32_t ctlCommandCPU2PWMDisable(serialDataExchange_t *data);

static uint32_t ctlCommandCPU1ADCBufferSet(serialDataExchange_t *data);
static uint32_t ctlCommandCPU1ADCBufferRead(serialDataExchange_t *data);

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

    uint32_t k;

    /* Register commands */
    serialRegisterHandle(PLAT_CMD_CPU1_BLINK, ctlCommandCPU1Blink);

    serialRegisterHandle(PLAT_CMD_CPU1_CPU2_STATUS, ctlCommandCPU2Status);
    serialRegisterHandle(PLAT_CMD_CPU1_CPU2_STATUS_CLEAR, ctlCommandCPU2StatusClear);
    serialRegisterHandle(PLAT_CMD_CPU1_CPU2_BLINK, ctlCommandCPU2Blink);
    serialRegisterHandle(PLAT_CMD_CPU1_CPU2_GPIO, ctlCommandCPU2GPIO);

    serialRegisterHandle(PLAT_CMD_CPU1_CPU2_PWM_ENABLE, ctlCommandCPU2PWMEnable);
    serialRegisterHandle(PLAT_CMD_CPU1_CPU2_PWM_DISABLE, ctlCommandCPU2PWMDisable);

    serialRegisterHandle(PLAT_CMD_CPU1_ADC_BUFFER_SET, ctlCommandCPU1ADCBufferSet);
    serialRegisterHandle(PLAT_CMD_CPU1_ADC_BUFFER_READ, ctlCommandCPU1ADCBufferRead);

    /*
     * Enable ADC ISR. We don't want this interrupt to go through the
     * operating system, so we add it manually to the interrupt table. The
     * interrupt function should then have the __interrupt attribute in its
     * prototype, so all the context saving is done by the compiler.
     */
    Interrupt_register(INT_ADCA1, ctlADCISR);
    Interrupt_enable(INT_ADCA1);

    /* Initializes ADC buffer */
    for(k = 0; k < CTL_CONFIG_ADC_N; k++){
        ctlADCBuffer[k].buffer = (uint16_t *)PLAT_CPU1_ADC_RAM_ADD;
        ctlADCBuffer[k].i = 0;
        ctlADCBuffer[k].size = 0;
    }
}
//-----------------------------------------------------------------------------
static void ctlIPCCommand(uint32_t command, uint32_t data){

    HWREG(IPC_BASE + IPC_O_SENDCOM) = command;
    HWREG(IPC_BASE + IPC_O_SENDDATA) = data;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CMD;
}
//-----------------------------------------------------------------------------
static uint32_t ctlIPCDataReceive(uint32_t flag, uint32_t *data){

    /* Wait until CPU2 sets the selected flag */
    while( !(HWREG(IPC_BASE + IPC_O_STS) & (1UL << flag)) );

    /* Acks the IPC flag */
    HWREG(IPC_BASE + IPC_O_ACK) = 1UL << flag;
    HWREG(IPC_BASE + IPC_O_CLR) = 1UL << flag;

    /* Saves the data */
    *data = HWREG(IPC_BASE + IPC_O_RECVDATA);

    return 0;
}
//-----------------------------------------------------------------------------
static uint32_t ctlADCBufferUpdate(void){

    uint32_t k;
    uint16_t *p;
    uint32_t size;

    /* Check if the overall adc buffer size does not exceed memory size */
    size = 0;
    for(k = 0; k < CTL_CONFIG_ADC_N; k++){
        size += ctlADCBuffer[k].size;
    }

    if( size > PLAT_CPU1_ADC_RAM_SIZE ) return 1;

    p = (uint16_t *)PLAT_CPU1_ADC_RAM_ADD;
    for(k = 0; k < CTL_CONFIG_ADC_N; k++){
        ctlADCBuffer[k].buffer = p;
        ctlADCBuffer[k].i = 0;
        p += ctlADCBuffer[k].size;
    }

    return 0;
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
static uint32_t ctlCommandCPU2Status(serialDataExchange_t *data){

    uint32_t status;
    uint32_t dataStatus;

    ctlIPCCommand(PLAT_CMD_CPU2_STATUS, 0);
    dataStatus = ctlIPCDataReceive(PLAT_IPC_FLAG_CPU2_CPU1_DATA, &status);

    if( dataStatus != 0 ){
        data->buffer[0] = PLAT_CMD_CPU1_ERR_CPU2_UNRESPONSIVE;
        data->size = 1;
    }
    else{
        data->buffer[0] = 0;
        data->buffer[1] = (uint8_t)(status >> 8);
        data->buffer[2] = (uint8_t)(status & 0xFF);
        data->size = 3;
    }

    data->bufferMode = 0;

    return 1;
}
//-----------------------------------------------------------------------------
static uint32_t ctlCommandCPU2StatusClear(serialDataExchange_t *data){

    uint32_t status;
    uint32_t dataStatus;

    ctlIPCCommand(PLAT_CMD_CPU2_STATUS_CLEAR, 0);
    dataStatus = ctlIPCDataReceive(PLAT_IPC_FLAG_CPU2_CPU1_DATA, &status);

    if( dataStatus != 0 ){
        data->buffer[0] = PLAT_CMD_CPU1_ERR_CPU2_UNRESPONSIVE;
        data->size = 1;
    }
    else{
        data->buffer[0] = 0;
        data->buffer[1] = (uint8_t)(status >> 8);
        data->buffer[2] = (uint8_t)(status & 0xFF);
        data->size = 3;
    }

    data->bufferMode = 0;

    return 1;
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

    if( ctlIPCDataReceive(PLAT_IPC_FLAG_CPU2_CPU1_DATA, &cmd) != 0 ){
        data->buffer[0] = PLAT_CMD_CPU1_ERR_CPU2_UNRESPONSIVE;
        data->size = 1;
    }
    else{
        data->buffer[0] = 0;
        data->buffer[1] = (uint8_t)(cmd >> 8);
        data->buffer[2] = (uint8_t)(cmd & 0xFF);
        data->size = 3;
    }
    data->bufferMode = 0;

    return 1;
}
//-----------------------------------------------------------------------------
static uint32_t ctlCommandCPU2PWMEnable(serialDataExchange_t *data){

    uint16_t dc;
    uint32_t status;

    if( ctlADCBufferUpdate() != 0 ){
        data->buffer[0] = PLAT_CMD_CPU1_PWM_ENABLE_ERR_BUFFER;
        data->size = 1;
        data->bufferMode = 0;
        return 1;
    }

    dc = data->buffer[0] << 8;
    dc = dc | data->buffer[1];

    ctlIPCCommand(PLAT_CMD_CPU2_PWM_ENABLE, dc);

    if( ctlIPCDataReceive(PLAT_IPC_FLAG_CPU2_CPU1_DATA, &status) != 0 ){
        data->buffer[0] = PLAT_CMD_CPU1_ERR_CPU2_UNRESPONSIVE;
        data->size = 1;
    }
    else{
        data->buffer[0] = 0;
        data->buffer[1] = (uint8_t)(status >> 8);
        data->buffer[2] = (uint8_t)(status & 0xFF);
        data->size = 3;
    }

    data->bufferMode = 0;

    return 1;
}
//-----------------------------------------------------------------------------
static uint32_t ctlCommandCPU2PWMDisable(serialDataExchange_t *data){

    uint32_t status;

    ctlIPCCommand(PLAT_CMD_CPU2_PWM_DISABLE, 0);

    if( ctlIPCDataReceive(PLAT_IPC_FLAG_CPU2_CPU1_DATA, &status) != 0 ){
        data->buffer[0] = PLAT_CMD_CPU1_ERR_CPU2_UNRESPONSIVE;
        data->size = 1;
    }
    else{
        data->buffer[0] = 0;
        data->buffer[1] = (uint8_t)(status >> 8);
        data->buffer[2] = (uint8_t)(status & 0xFF);
        data->size = 3;
    }

    data->bufferMode = 0;

    return 1;
}
//-----------------------------------------------------------------------------
static uint32_t ctlCommandCPU1ADCBufferSet(serialDataExchange_t *data){

    uint32_t adc, size, sizeOriginal;

    adc = data->buffer[0];

    if( (adc + 1) > CTL_CONFIG_ADC_N ){
        data->buffer[0] = PLAT_CMD_CPU1_ADC_BUFFER_SET_ERR_ADC;
        data->size = 1;
        data->bufferMode = 0;
        return 1;
    }

    size = data->buffer[1] << 8;
    size = size | data->buffer[2];

    sizeOriginal = ctlADCBuffer[adc].size;
    ctlADCBuffer[adc].size = size;

    if( ctlADCBufferUpdate() != 0 ){
        ctlADCBuffer[adc].size = sizeOriginal;
        ctlADCBufferUpdate();
        data->buffer[0] = PLAT_CMD_CPU1_ADC_BUFFER_SET_ERR_SIZE;
    }
    else{
        data->buffer[0] = 0;
    }

    data->size = 1;
    data->bufferMode = 0;

    return 1;
}
//-----------------------------------------------------------------------------
static uint32_t ctlCommandCPU1ADCBufferRead(serialDataExchange_t *data){

    uint32_t adc;

    adc = data->buffer[0];

    if( (adc + 1) > CTL_CONFIG_ADC_N ){
        data->buffer[0] = PLAT_CMD_CPU1_ADC_BUFFER_SET_ERR_ADC;
        data->size = 1;
        data->bufferMode = 0;
        return 1;
    }

    data->bufferMode = 1;
    data->buffer = (uint8_t *)ctlADCBuffer[adc].buffer;
    data->size = ctlADCBuffer[adc].i << 1;

    return 1;
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

    if( ctlADCBuffer[0].i < ctlADCBuffer[0].size ){
        ctlADCBuffer[0].buffer[ctlADCBuffer[0].i] = ADC_readResult(ADCARESULT_BASE, (ADC_SOCNumber)0);
        ctlADCBuffer[0].i++;
    }

    if( ctlADCBuffer[1].i < ctlADCBuffer[1].size ){
        ctlADCBuffer[1].buffer[ctlADCBuffer[1].i] = ADC_readResult(ADCARESULT_BASE, (ADC_SOCNumber)1);
        ctlADCBuffer[1].i++;
    }

    if( ctlADCBuffer[2].i < ctlADCBuffer[2].size ){
        ctlADCBuffer[2].buffer[ctlADCBuffer[2].i] = ADC_readResult(ADCBRESULT_BASE, (ADC_SOCNumber)0);
        ctlADCBuffer[2].i++;
    }

    if( ctlADCBuffer[3].i < ctlADCBuffer[3].size ){
        ctlADCBuffer[3].buffer[ctlADCBuffer[3].i] = ADC_readResult(ADCCRESULT_BASE, (ADC_SOCNumber)0);
        ctlADCBuffer[3].i++;
    }

    if( ctlADCBuffer[4].i < ctlADCBuffer[4].size ){
        ctlADCBuffer[4].buffer[ctlADCBuffer[4].i] = ADC_readResult(ADCARESULT_BASE, (ADC_SOCNumber)2);
        ctlADCBuffer[4].i++;
    }

    if( ctlADCBuffer[5].i < ctlADCBuffer[5].size ){
        ctlADCBuffer[5].buffer[ctlADCBuffer[5].i] = ADC_readResult(ADCBRESULT_BASE, (ADC_SOCNumber)1);
        ctlADCBuffer[5].i++;
    }
}
//-----------------------------------------------------------------------------
//=============================================================================
