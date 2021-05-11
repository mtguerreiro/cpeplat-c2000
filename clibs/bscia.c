/*
 * bscia.c
 *
 *  Created on: 06.05.2021
 *      Author: local_marco
 */

/*
 * TODO: initialize interrupts here instead of app thing
 * TODO: configurable RX and TX queue sizes
 */
//=============================================================================
/*-------------------------------- Includes ---------------------------------*/
//=============================================================================
#include "bscia.h"

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
static void bsciaInitializeHW(void);
static void bsciaInitializeSW(void);

void sciaTXFIFOISR(UArg arg);
void sciaRXFIFOISR(UArg arg);
//=============================================================================

//=============================================================================
/*--------------------------------- Globals ---------------------------------*/
//=============================================================================
Mailbox_Handle bsciaRXMailBox;
Mailbox_Handle bsciaTXMailBox;
//=============================================================================

//=============================================================================
/*-------------------------------- Functions --------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
void bsciaInitialize(void){

    bsciaInitializeHW();
    bsciaInitializeSW();
}
//-----------------------------------------------------------------------------
int32_t bsciaRead(uint8_t *buffer, uint32_t nbytes, uint32_t to){

    uint8_t *p;
    int32_t bytesRead = 0;

    /* Removes data from the RX queue, one byt at a time */
    p = buffer;
    while( bytesRead < nbytes ){

        if( Mailbox_pend(bsciaRXMailBox, p, to) != TRUE ) break;

        p++;
        bytesRead++;
    }

    return bytesRead;
}
//-----------------------------------------------------------------------------
int32_t bsciaWrite(uint8_t *buffer, uint32_t nbytes, uint32_t to){

    uint32_t status;
    uint8_t *p;
    uint32_t bytesWritten = 0;

    /*
     * Sends data through SCIA. If the transmitter is ready, we write the
     * data directly to the TX registers through the writeChar function. If
     * the TX is busy, we enqueue the data, which will then be automatically
     * removed from the queue on the interrupt mechanism.
     */
    p = buffer;
    while( bytesWritten < nbytes ){

        status = SCI_getInterruptStatus(SCIA_BASE);

        if( status & SCI_INT_TXRDY ){
            SCI_enableInterrupt(SCIA_BASE, SCI_INT_TXRDY);
            SCI_writeCharNonBlocking(SCIA_BASE, *p);
        }
        else{
            if( Mailbox_post(bsciaTXMailBox, p, to) != TRUE ) break;
        }

        p++;
        bytesWritten++;
    }

    return bytesWritten;
}
//-----------------------------------------------------------------------------
//=============================================================================

//=============================================================================
/*---------------------------- Static functions -----------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
static void bsciaInitializeHW(void){

    /* Configuration for the SCI Rx pin */
    GPIO_setMasterCore(DEVICE_GPIO_PIN_SCIRXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_QUAL_ASYNC);

    /* Configuration for the SCI Tx pin */
    GPIO_setMasterCore(DEVICE_GPIO_PIN_SCITXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_QUAL_ASYNC);


    SCI_performSoftwareReset(SCIA_BASE);

    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, 115200, (SCI_CONFIG_WLEN_8 |
                                                        SCI_CONFIG_STOP_ONE |
                                                        SCI_CONFIG_PAR_NONE));
    SCI_resetChannels(SCIA_BASE);
    //SCI_enableFIFO(SCIA_BASE);
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
    //SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXRDY | SCI_INT_RXRDY_BRKDT);
    //SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX2, SCI_FIFO_RX2);
    SCI_enableModule(SCIA_BASE);
    SCI_performSoftwareReset(SCIA_BASE);

    SCI_enableInterrupt(SCIA_BASE, SCI_INT_RXRDY_BRKDT);
}
//-----------------------------------------------------------------------------
static void bsciaInitializeSW(void){

    bsciaRXMailBox = Mailbox_create(sizeof(uint8_t), 50, NULL, NULL);
    bsciaTXMailBox = Mailbox_create(sizeof(uint8_t), 50, NULL, NULL);
}
//-----------------------------------------------------------------------------
//=============================================================================

//=============================================================================
/*---------------------------------- IRQs -----------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
void sciaTXISR(UArg arg){

    uint8_t data;

    if( Mailbox_pend(bsciaTXMailBox, &data, BIOS_NO_WAIT) == TRUE ){
        SCI_writeCharNonBlocking(SCIA_BASE, data);
    }
    else{
        SCI_disableInterrupt(SCIA_BASE, SCI_INT_TXRDY);
    }
}
//-----------------------------------------------------------------------------
void sciaRXISR(UArg arg){

    uint8_t data;

    data = SCI_readCharNonBlocking(SCIA_BASE);

    Mailbox_post(bsciaRXMailBox, &data, BIOS_NO_WAIT);
}
//-----------------------------------------------------------------------------
//=============================================================================

