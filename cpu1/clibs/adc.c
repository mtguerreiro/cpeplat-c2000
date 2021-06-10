/*
 * adc.c
 *
 *  Created on: 20.05.2021
 *      Author: mguerreiro
 */

/*
 * TODO: adc buffer should come from somewhere else?
 * TODO: initialize ADC via code not appcfg
 */

//=============================================================================
/*-------------------------------- Includes ---------------------------------*/
//=============================================================================
#include "adc.h"

/* Kernel */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Mailbox.h>

/* XDC */
#include <xdc/runtime/Error.h>

/* Device and drivers */
#include "device.h"
#include "driverlib.h"
//=============================================================================

//=============================================================================
/*-------------------------------- Prototypes -------------------------------*/
//=============================================================================
static void adcInitializeHW(void);
static void adcInitializeSW(void);

void adcISR(UArg arg);
//=============================================================================

//=============================================================================
/*--------------------------------- Globals ---------------------------------*/
//=============================================================================
typedef struct{
    uint8_t buffer[ADC_CONFIG_BUFFER_SIZE];
    uint8_t *p;
    uint8_t *bufferEnd;
}adcControl_t;

adcControl_t adcCtl;
//=============================================================================

//=============================================================================
/*-------------------------------- Functions --------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
void adcInitialize(void){

    adcInitializeHW();
    adcInitializeSW();
}
//-----------------------------------------------------------------------------
void adcStartAcq(void){

    adcCtl.p = adcCtl.buffer;
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP);
}
//-----------------------------------------------------------------------------
uint16_t* adcGetBuffer(void){

    return adcCtl.buffer;
}
//-----------------------------------------------------------------------------
int32_t adcRead(uint16_t *buffer, uint32_t nbytes){

    uint16_t *p;

    if( nbytes > sizeof(adcCtl.buffer) ) return 1;

    p = adcCtl.buffer;
    while( nbytes-- ){
        *buffer++ = *p++;
    }

    return 0;
}
//-----------------------------------------------------------------------------
//=============================================================================

//=============================================================================
/*---------------------------- Static functions -----------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
static void adcInitializeHW(void){

    /* Initializes ADC */
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    ADC_enableConverter(ADCA_BASE);
    DEVICE_DELAY_US(1000);

    /* Initializes ePWM */
    EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_U_CMPA);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 0x0800);
    EPWM_setTimeBasePeriod(EPWM1_BASE, 0x1000);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);

    /* Sets ePWM to trigger ADC */
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN1, 150);
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}
//-----------------------------------------------------------------------------
static void adcInitializeSW(void){

    adcCtl.p = adcCtl.buffer;
    adcCtl.bufferEnd = adcCtl.p + sizeof(adcCtl.buffer);
}
//-----------------------------------------------------------------------------
//=============================================================================

//=============================================================================
/*---------------------------------- IRQs -----------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
void adcISR(UArg arg){

    uint16_t data;
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    data = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
    *adcCtl.p++ = (uint8_t)(data >> 8);
    *adcCtl.p++ = (uint8_t)(data & 0xFF);

    if( adcCtl.p == adcCtl.bufferEnd ){
        adcCtl.p = adcCtl.buffer;
        EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
        EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);
    }
}
//-----------------------------------------------------------------------------
//=============================================================================
