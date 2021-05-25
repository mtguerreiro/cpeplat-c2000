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

/* Libs */
#include "clibs/serial.h"
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
static void ctlInitializeGPIOs(void);

static uint32_t ctlCommandRelay1(serialDataExchange_t *data);
static uint32_t ctlCommandRelay2(serialDataExchange_t *data);
static uint32_t ctlCommandStartAcq(serialDataExchange_t *data);
static uint32_t ctlCommandReadADCData(serialDataExchange_t *data);
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

    ctlInitializeGPIOs();

    adcInitialize();

    /* Register commands */
    serialRegisterHandle(CTL_CMD_RELAY1, ctlCommandRelay1);
    serialRegisterHandle(CTL_CMD_RELAY2, ctlCommandRelay2);
    serialRegisterHandle(CTL_CMD_OPMODE_1, ctlCommandStartAcq);
    serialRegisterHandle(CTL_CMD_GET_ADC_DATA, ctlCommandReadADCData);
}
//-----------------------------------------------------------------------------
static void ctlInitializeGPIOs(void){

    /* Initialize GPIO and configure the GPIO pin as a push-pull output */
    GPIO_setPadConfig(CTL_CONFIG_RELAY_1, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(CTL_CONFIG_RELAY_1, GPIO_DIR_MODE_OUT);

    GPIO_setPadConfig(CTL_CONFIG_RELAY_2, GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(CTL_CONFIG_RELAY_2, GPIO_DIR_MODE_OUT);
}
//-----------------------------------------------------------------------------
static uint32_t ctlCommandRelay1(serialDataExchange_t *data){

    uint32_t val = 1;

    if( *data->buffer == 0 ) val = 0;

    GPIO_writePin(CTL_CONFIG_RELAY_1, val);

    return 0;
}
//-----------------------------------------------------------------------------
static uint32_t ctlCommandRelay2(serialDataExchange_t *data){

    uint32_t val = 1;

    if( *data->buffer == 0 ) val = 0;

    GPIO_writePin(CTL_CONFIG_RELAY_2, val);

    return 0;
}
//-----------------------------------------------------------------------------
static uint32_t ctlCommandStartAcq(serialDataExchange_t *data){

    adcStartAcq();

    return 0;
}
//-----------------------------------------------------------------------------
static uint32_t ctlCommandReadADCData(serialDataExchange_t *data){

    uint8_t *pADC;

    pADC = adcGetBuffer();
    data->buffer = pADC;
    data->size = CTL_CONFIG_ADC_SAMPLES;

    return 1;
}
//-----------------------------------------------------------------------------
//=============================================================================
