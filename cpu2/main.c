/*
 * main.c
 *
 *  Created on: 01.12.2020
 *      Author: mguerreiro
 */

/*
 * TODO: IPC0 flag is hard coded in CPU2. Change that.
 */

//TESTCOMMIT

//=============================================================================
/*-------------------------------- Includes ---------------------------------*/
//=============================================================================
/* Device */
#include "driverlib.h"
#include "device.h"
#include "inc/hw_ipc.h"

#include "plat_defs.h"
//=============================================================================

//===========================================================================
/*------------------------------- Data types ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------,
typedef void (*mainCommandHandle)(uint32_t data);
//---------------------------------------------------------------------------
typedef struct{
    uint32_t *p;
    uint32_t size;
    uint32_t space;
}mainRAMControl_t;
//---------------------------------------------------------------------------
typedef struct{
    uint32_t blink;
    mainCommandHandle handle[PLAT_CMD_CPU2_END];
    mainRAMControl_t ram;
}mainControl_t;
//---------------------------------------------------------------------------
//===========================================================================

//=============================================================================
/*--------------------------------- Defines ---------------------------------*/
//=============================================================================

//=============================================================================

//=============================================================================
/*--------------------------------- Globals ---------------------------------*/
//=============================================================================
mainControl_t mainControl = {.blink = 1000};
//=============================================================================

//=============================================================================
/*------------------------------- Prototypes --------------------------------*/
//=============================================================================
static void mainInitialize(void);

static void mainInitializeIPC(void);

static void mainInitializeRAM(void);

static void mainCommandInitializeHandlers(void);
static void mainCommandBlink(uint32_t data);
static void mainCommandGPIO(uint32_t data);

static void mainIPC0ISR(void);
//=============================================================================

//=============================================================================
/*----------------------------------- Main ----------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
void main(void){

    mainInitialize();

    while(1){
        GPIO_togglePin(DEVICE_GPIO_PIN_LED2);
        DEVICE_DELAY_US(1000 * mainControl.blink);
    }
}
testetsets

//-----------------------------------------------------------------------------
//=============================================================================

//=============================================================================
/*---------------------------- Static functions -----------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
static void mainInitialize(void){

    /*
     * Initialize device clock and peripherals. We don't initialize GPIOs
     * because this will be done by CPU1.
     */
    Device_init();

    /* Initializes handlers for CPU1 commands */
    mainCommandInitializeHandlers();

    /* Sets IPC that is used for commands */
    mainInitializeIPC();

    /*
     * Initializes the section of RAM that is used for CPU2->CPU1 data
     * exchange.
     */
    mainInitializeRAM();
}
//-----------------------------------------------------------------------------
static void mainInitializeIPC(void){

    /* Initialize PIE and clear PIE registers. Disables CPU interrupts */
    Interrupt_initModule();

    /*
     * Initialize the PIE vector table with pointers to the shell Interrupt
     * Service Routines (ISR).
     */
    Interrupt_initVectorTable();

    /* Enable Global Interrupt (INTM) and realtime interrupt (DBGM) */
    EINT;
    ERTM;

    /* Sets up IPC interrupt */
    Interrupt_register(INT_IPC_0, mainIPC0ISR);
    Interrupt_enable(INT_IPC_0);

    /* Signals CPU2 initialization to CPU1 */
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_INIT;
}
//-----------------------------------------------------------------------------
static void mainInitializeRAM(void){

    uint32_t k;
    uint32_t *p;

    /* Wait until CPU01 gives us ownership of RAM sections GS14 and GS15 */
    while( !(HWREG(IPC_BASE + IPC_O_STS) & (1UL << PLAT_IPC_FLAG_MEM_OWN)) );

    /* Acks the IPC flag */
    HWREG(IPC_BASE + IPC_O_ACK) = 1UL << PLAT_IPC_FLAG_MEM_OWN;
    HWREG(IPC_BASE + IPC_O_CLR) = 1UL << PLAT_IPC_FLAG_MEM_OWN;

    /* Initializes the control structure and fills the memory */
    mainControl.ram.p = (uint32_t *)PLAT_CPU2_CPU1_RAM_ADD;
    mainControl.ram.size = PLAT_CPU2_CPU1_RAM_SIZE;
    mainControl.ram.space = PLAT_CPU2_CPU1_RAM_SIZE;

    k = PLAT_CPU2_CPU1_RAM_SIZE;
    p = (uint32_t *)PLAT_CPU2_CPU1_RAM_ADD;
    while(k--){
        *p++ = 0xAA995500;
    }
}
//-----------------------------------------------------------------------------
static void mainCommandInitializeHandlers(void){

    mainControl.handle[PLAT_CMD_CPU2_BLINK] = mainCommandBlink;
    mainControl.handle[PLAT_CMD_CPU2_GPIO] = mainCommandGPIO;
}
//-----------------------------------------------------------------------------
static void mainCommandBlink(uint32_t data){

    mainControl.blink = data;
}
//-----------------------------------------------------------------------------
static void mainCommandGPIO(uint32_t data){

    uint32_t gpio, state;

    gpio = data & 0xFF;
    state = (data >> 16) & 0xFF;

    GPIO_writePin(gpio, state);
}
//-----------------------------------------------------------------------------
//=============================================================================

//=============================================================================
/*---------------------------------- IRQs -----------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
__interrupt void mainIPC0ISR(void){

    uint32_t cmd, data;

    HWREG(IPC_BASE + IPC_O_ACK) = 1UL << 0;
    HWREG(IPC_BASE + IPC_O_CLR) = 1UL << 0;

    cmd = HWREG(IPC_BASE + IPC_O_RECVCOM);

    if( cmd < PLAT_CMD_CPU2_END ){
        data = HWREG(IPC_BASE + IPC_O_RECVDATA);
        mainControl.handle[cmd](data);
    }

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
//-----------------------------------------------------------------------------
//=============================================================================
