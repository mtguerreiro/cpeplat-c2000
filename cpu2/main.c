/*
 * main.c
 *
 *  Created on: 01.12.2020
 *      Author: LRS
 */

/*
 * TODO: IPC0 flag is hard coded in CPU2. Change that.
 * TODO: Register ADCB_EVT
 * TODO: add checking condition for adc limits in isr
 */

//=============================================================================
/*-------------------------------- Includes ---------------------------------*/
//=============================================================================
/* Device */
#include "driverlib.h"
#include "device.h"
#include "inc/hw_ipc.h"

#include "plat_defs.h"

#include "F2837xD_GlobalVariableDefs.c"
//=============================================================================

//===========================================================================
/*------------------------------- Data types ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------,
typedef void (*mainCommandHandle)(uint32_t data);
//---------------------------------------------------------------------------
typedef struct{
    uint16_t *buffer;
    uint16_t i;
    uint16_t size;
}mainBufferControl_t;
//---------------------------------------------------------------------------
typedef struct{
    uint32_t status;
    uint32_t blink;
    mainCommandHandle handle[PLAT_CMD_CPU2_END];
    mainBufferControl_t buffer;
    uint16_t u;
}mainControl_t;
//---------------------------------------------------------------------------
//===========================================================================

//=============================================================================
/*--------------------------------- Defines ---------------------------------*/
//=============================================================================
#define MAIN_CONFIG_EPWM2_PERIOD        (0x03E7>>1)
#define MAIN_CONFIG_EPWM4_PERIOD        (0x03E7>>1)


#define MAIN_STATUS_ADCA_PPB1_TRIP      (1 << 0)
#define MAIN_STATUS_ADCA_PPB2_TRIP      (1 << 1)
#define MAIN_STATUS_ADCB_PPB1_TRIP      (1 << 2)


//#define PWM_PERIOD  (0x03E7>>1)          // PWM1 frequency 999 counts = 200kHz
//#define PWM_CMPR25  PWM_PERIOD>>2   // PWM1 initial duty cycle = 25%
//#define SWITCHINGFREQUENCY 200000
//#define DUTYCYCLE 0.25
//=============================================================================
//=============================================================================

//=============================================================================
/*--------------------------------- Globals ---------------------------------*/
//=============================================================================
mainControl_t mainControl;
//=============================================================================
/*------------------------------- Prototypes --------------------------------*/
//=============================================================================
static void mainInitialize(void);

static void mainInitializeIPC(void);

static void mainInitializeBuffer(void);

static void mainInitializeADC(void);
static void mainInitializeADCLimits(void);
static void mainInitializeADCISR(void);

static void mainInitializeEPWM(void);
static void mainInitializeEPWM2(void);
static void mainInitializeEPWM4(void);

static void mainCommandInitializeHandlers(void);
static void mainCommandStatus(uint32_t data);
static void mainCommandStatusClear(uint32_t data);
static void mainCommandBlink(uint32_t data);
static void mainCommandGPIO(uint32_t data);
static void mainCommandPWMEnable(uint32_t data);
static void mainCommandPWMDisable(uint32_t data);

static __interrupt void mainIPC0ISR(void);

static __interrupt void mainADCAISR(void);
static __interrupt void mainADCPPBISR(void);
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

    /* Initialize PIE and clear PIE registers. Disables CPU interrupts */
    Interrupt_initModule();

    /*
     * Initialize the PIE vector table with pointers to the shell Interrupt
     * Service Routines (ISR).
     */
    Interrupt_initVectorTable();

    /* Initializes control structure */
    mainControl.blink = 1000;
    mainControl.status = 0;

    /* Initializes handlers for CPU1 commands */
    mainCommandInitializeHandlers();

    /* Sets IPC that is used for commands */
    mainInitializeIPC();

    /* Waits until CPU1 has initialized */
    while( !(HWREG(IPC_BASE + IPC_O_STS) & (1UL << PLAT_IPC_FLAG_CPU1_INIT)) );

    /* Acks the IPC flag */
    HWREG(IPC_BASE + IPC_O_ACK) = 1UL << PLAT_IPC_FLAG_CPU1_INIT;
    HWREG(IPC_BASE + IPC_O_CLR) = 1UL << PLAT_IPC_FLAG_CPU1_INIT;

    /* Now we initialize the peripherals owned by CPU2 */
    mainInitializeADC();
    mainInitializeADCLimits();
    mainInitializeADCISR();

    mainInitializeEPWM();

    /* Initializes CPU2 buffer */
    mainInitializeBuffer();

    /* Enable Global Interrupt (INTM) and realtime interrupt (DBGM) */
    EINT;
    ERTM;

    /* Signals CPU1 that we have initialized CPU2 */
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_INIT;
}
//-----------------------------------------------------------------------------
static void mainInitializeIPC(void){

//    /* Initialize PIE and clear PIE registers. Disables CPU interrupts */
//    Interrupt_initModule();
//
//    /*
//     * Initialize the PIE vector table with pointers to the shell Interrupt
//     * Service Routines (ISR).
//     */
//    Interrupt_initVectorTable();

//    /* Enable Global Interrupt (INTM) and realtime interrupt (DBGM) */
//    EINT;
//    ERTM;

    /* Sets up IPC interrupt */
    Interrupt_register(INT_IPC_0, mainIPC0ISR);
    Interrupt_enable(INT_IPC_0);
}
//-----------------------------------------------------------------------------
static void mainInitializeADC(void){

    EALLOW;

    // ADC-A
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;          // Set ADCCLK divider to /4
    AdcaRegs.ADCCTL2.bit.RESOLUTION =  0;       // 12-bit resolution
    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0;        // Single-ended channel conversions (12-bit mode only)
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // Power up the ADC

    // ADC_B
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;          // Set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.RESOLUTION =  0;       // 12-bit resolution
    AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0;        // Single-ended channel conversions (12-bit mode only)
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Set pulse positions to late
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // Power up the ADC

    // ADC-C
    AdccRegs.ADCCTL2.bit.PRESCALE = 6;          // Set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.RESOLUTION =  0;       // 12-bit resolution RESOLUTION_12BIT;
    AdccRegs.ADCCTL2.bit.SIGNALMODE = 0;        // Single-ended channel conversions (12-bit mode only)
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Set pulse positions to late
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // Power up the ADC

    EDIS;

    DEVICE_DELAY_US(1000);                             // Delay for 1ms to allow ADC time to power up

    // Select the channels to convert and end of conversion flag
    EALLOW;

    //ADC-A
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 1;          // SOC0 will convert pin A1
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;         // Sample window is 100 SYSCLK cycles  //i think 15 SYSCLK -> 75ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 7;        // Trigger on ePWM2 SOCA/C

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 4;          // SOC1 will convert pin A4
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14;         // Sample window is 100 SYSCLK cycles  //i think 15 SYSCLK -> 75ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 7;        // Trigger on ePWM2 SOCA/C

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 5;          // SOC2 will convert pin A5
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 14;         // Sample window is 100 SYSCLK cycles  //i think 15 SYSCLK -> 75ns
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 7;        // Trigger on ePWM2 SOCA/C

    //Flag ADC-A End of Conversion
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;      // End of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;        // Enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Make sure INT1 flag is cleared

    //Flag ADC-A Limit
    AdcaRegs.ADCINTSEL1N2.bit.INT2SEL = 2;      // End of SOC2 will set INT2 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT2E = 1;        // Enable INT2 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;      // Make sure INT2 flag is cleared

    //ADC-B
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4;          // SOC0 will convert pin B4
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14;         // Sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 7;        // Trigger on ePWM2 SOCA/C

    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 5;          // SOC1 will convert pin B5
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 14;         // Sample window is 100 SYSCLK cycles
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 7;        // Trigger on ePWM2 SOCA/C

    //Flag ADC-B Limit
    AdcbRegs.ADCINTSEL1N2.bit.INT2SEL = 0;      // End of SOC0 will set INT2 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT2E = 1;        // Enable INT2 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;      // Make sure INT2 flag is cleared

    //ADC-C
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 4;          // SOC0 will convert pin C4
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 14;         // Sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 7;        // Trigger on ePWM2 SOCA/C


    EDIS;
}
//-----------------------------------------------------------------------------
static void mainInitializeADCLimits(void){

    EALLOW;

    //Vin limit
    AdcaRegs.ADCPPB1CONFIG.bit.CONFIG = 0;  //PPB1A is associated with soc0 -> Vin

    //
    //set high and low limits
    //
    AdcaRegs.ADCPPB1TRIPHI.bit.LIMITHI = 4000;
    AdcaRegs.ADCPPB1TRIPLO.bit.LIMITLO = 0;

    //
    //enable high events to generate interrupt
    //
    AdcaRegs.ADCEVTINTSEL.bit.PPB1TRIPHI = 1;
    AdcaRegs.ADCEVTINTSEL.bit.PPB1TRIPLO = 0;



    //IL limit
    AdcaRegs.ADCPPB2CONFIG.bit.CONFIG = 2;  //PPB2A is associated with soc2 -> IL

    //
    //set high and low limits
    //
    AdcaRegs.ADCPPB2TRIPHI.bit.LIMITHI = 3150;
    AdcaRegs.ADCPPB2TRIPLO.bit.LIMITLO = 0;

    //
    //enable high events to generate interrupt
    //
    AdcaRegs.ADCEVTINTSEL.bit.PPB2TRIPHI = 1;
    AdcaRegs.ADCEVTINTSEL.bit.PPB2TRIPLO = 0;



    //Vout
    AdcbRegs.ADCPPB1CONFIG.bit.CONFIG = 0;  //PPB1B is associated with soc0 -> Vout

    //
    //set high and low limits
    //
    AdcbRegs.ADCPPB1TRIPHI.bit.LIMITHI = 4000;
    AdcbRegs.ADCPPB1TRIPLO.bit.LIMITLO = 0;
    //
    //enable high events to generate interrupt
    //
    AdcbRegs.ADCEVTINTSEL.bit.PPB1TRIPHI = 1;
    AdcbRegs.ADCEVTINTSEL.bit.PPB1TRIPLO = 0;

    EDIS;
}
//-----------------------------------------------------------------------------
static void mainInitializeADCISR(void){

    /* Sets up ADC EOC interrupt */
    Interrupt_register(INT_ADCA1, mainADCAISR);
    Interrupt_enable(INT_ADCA1);

    /* Sets up ADC compare interrupt */
    Interrupt_register(INT_ADCA_EVT, mainADCPPBISR);
    Interrupt_enable(INT_ADCA_EVT);

    Interrupt_register(INT_ADCB_EVT, mainADCPPBISR);
    Interrupt_enable(INT_ADCB_EVT);

    //    // Map ISR functions
//    EALLOW;
//    PieVectTable.ADCA1_INT = &adca1_isr;            // Function for ADCA interrupt 1
//    PieVectTable.ADCA_EVT_INT = &adca_ppb_isr;
//    PieVectTable.ADCB_EVT_INT = &adca_ppb_isr;
//    EDIS;
}
//-----------------------------------------------------------------------------
static void mainInitializeEPWM(void){

    // Initialize System Control
    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;
    EDIS;

    mainInitializeEPWM2();
    mainInitializeEPWM4();

    // Sync ePWM
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

//    // Start ePWM
//    EALLOW;
//    EPwm2Regs.ETSEL.bit.SOCAEN = 1;             // Enable SOCA
//    EPwm2Regs.TBCTL.bit.CTRMODE = 0;            // Un-freeze and enter up-count mode
//    EDIS;

}
//-----------------------------------------------------------------------------
static void mainInitializeEPWM2(void){

    EALLOW;
    // Assumes ePWM clock is already enabled
    EPwm2Regs.TBCTL.bit.CTRMODE = 3;            // Freeze counter
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;          // TBCLK pre-scaler = /1
    EPwm2Regs.TBPRD = MAIN_CONFIG_EPWM2_PERIOD; // Set period to 500 counts (200kHz) PWM1_PERIOD
    EPwm2Regs.ETSEL.bit.SOCAEN  = 0;            // Disable SOC on A group
    EPwm2Regs.ETSEL.bit.SOCASEL = 2;            // Select SOCA on period match
    EPwm2Regs.ETSEL.bit.SOCAEN = 1;             // Enable SOCA
    EPwm2Regs.ETPS.bit.SOCAPRD = 1;             // Generate pulse on 1st event
    EDIS;
}
//-----------------------------------------------------------------------------
static void mainInitializeEPWM4(void){

    // Setup TBCLK
     EALLOW;

     EPwm4Regs.TBCTL.bit.CTRMODE = 3;             // Freeze counter
     EPwm4Regs.TBPRD = MAIN_CONFIG_EPWM4_PERIOD;
     EPwm4Regs.TBCTL.bit.PHSEN = 0;               // disable phase loading
     EPwm4Regs.TBPHS.bit.TBPHS = 0x0000;          // Phase
     EPwm4Regs.TBCTR = 0x0000;                    // Clear counter
     EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0;           // Clock ratio to SYSCLKOUT
     EPwm4Regs.TBCTL.bit.CLKDIV = 0;
     EPwm4Regs.TBCTL.bit.SYNCOSEL = 1;

     // Setup shadow register load on ZERO
     EPwm4Regs.CMPCTL.bit.SHDWAMODE = 0;
     EPwm4Regs.CMPCTL.bit.SHDWBMODE = 0;
     EPwm4Regs.CMPCTL.bit.LOADAMODE = 0;
     EPwm4Regs.CMPCTL.bit.LOADBMODE = 0;

     // Set Compare values
     //EPwm4Regs.CMPA.bit.CMPA = dutyCycle_count;        // Set compare A value
     EPwm4Regs.CMPA.bit.CMPA = 0;        // Set compare A value

     // Set actions - Active Low
     EPwm4Regs.AQCTLA.bit.ZRO = 2;                // Clear PWM4A on Zero
     EPwm4Regs.AQCTLA.bit.CAU = 1;                // Set PWM4A on event A, up count

     EPwm4Regs.AQCTLB.bit.ZRO = 1;                // Set PWM4B on Zero
     EPwm4Regs.AQCTLB.bit.CAU = 2;                // Clear PWM4B on event A, up count


     // Active Low complementary PWMs - setup the deadband - spruhm8i - page 1994
     EPwm4Regs.DBCTL.bit.OUT_MODE = 3;
     EPwm4Regs.DBCTL.bit.POLSEL = 1;
     EPwm4Regs.DBCTL.bit.IN_MODE = 0;
     EPwm4Regs.DBRED.bit.DBRED = 5;
     EPwm4Regs.DBFED.bit.DBFED = 5;

     /* Enables counter */
     EPwm4Regs.TBCTL.bit.CTRMODE = 0;             // Freeze counter

     EDIS;
}
//-----------------------------------------------------------------------------
static void mainInitializeBuffer(void){

    /* Initializes the buffer control structure */
    mainControl.buffer.buffer = (uint16_t *)PLAT_CPU2_CPU1_RAM_ADD;
    mainControl.buffer.i = 0;
    mainControl.buffer.size = PLAT_CPU2_CPU1_RAM_SIZE;
}
//-----------------------------------------------------------------------------
static void mainCommandInitializeHandlers(void){

    mainControl.handle[PLAT_CMD_CPU2_STATUS] = mainCommandStatus;
    mainControl.handle[PLAT_CMD_CPU2_STATUS_CLEAR] = mainCommandStatusClear;
    mainControl.handle[PLAT_CMD_CPU2_BLINK] = mainCommandBlink;
    mainControl.handle[PLAT_CMD_CPU2_GPIO] = mainCommandGPIO;
    mainControl.handle[PLAT_CMD_CPU2_PWM_ENABLE] = mainCommandPWMEnable;
    mainControl.handle[PLAT_CMD_CPU2_PWM_DISABLE] = mainCommandPWMDisable;
}
//-----------------------------------------------------------------------------
static void mainCommandStatus(uint32_t data){

    HWREG(IPC_BASE + IPC_O_SENDDATA) = mainControl.status;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
}
//-----------------------------------------------------------------------------
static void mainCommandStatusClear(uint32_t data){

    mainControl.status = 0;

    HWREG(IPC_BASE + IPC_O_SENDDATA) = mainControl.status;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
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

    HWREG(IPC_BASE + IPC_O_SENDDATA) = mainControl.status;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
}
//-----------------------------------------------------------------------------
static void mainCommandPWMEnable(uint32_t data){

    /* Doesn't enable PWM if any status flag is set */
    if( mainControl.status != 0 ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = PLAT_CMD_CPU2_PWM_ENABLE_ERR_STATUS;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    /* Does not change the duty cycle if received an invalid one  */
    if( data > MAIN_CONFIG_EPWM4_PERIOD ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = PLAT_CMD_CPU2_PWM_ENABLE_ERR_INVALID_DC;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;

    mainControl.buffer.i = 0;

    // Start ePWM
    EALLOW;
    EPwm2Regs.ETSEL.bit.SOCAEN = 1;             // Enable SOCA
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;            // Un-freeze and enter up-count mode

    EPwm4Regs.CMPA.bit.CMPA = data;        // Set compare A value
    mainControl.u = data;
    //EPwm4Regs.TBCTL.bit.CTRMODE = 0;             // Count up
    EDIS;

}
//-----------------------------------------------------------------------------
static void mainCommandPWMDisable(uint32_t data){

    HWREG(IPC_BASE + IPC_O_SENDDATA) = mainControl.status;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;

    // Stop ePWM
    EALLOW;
    EPwm2Regs.ETSEL.bit.SOCAEN = 0;             // Disable SOCA
    EPwm2Regs.TBCTL.bit.CTRMODE = 3;            // Freezes counter

    EPwm4Regs.CMPA.bit.CMPA = 0;        // Set compare A value
    mainControl.u = 0;
    //EPwm4Regs.TBCTL.bit.CTRMODE = 3;             //
    EDIS;

}
//-----------------------------------------------------------------------------
//=============================================================================

//=============================================================================
/*---------------------------------- IRQs -----------------------------------*/
//=============================================================================
//-----------------------------------------------------------------------------
static __interrupt void  mainIPC0ISR(void){

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
static __interrupt void mainADCAISR(void){

    EPwm4Regs.CMPA.bit.CMPA = mainControl.u;

    if( mainControl.buffer.i < mainControl.buffer.size ){
        mainControl.buffer.buffer[mainControl.buffer.i] = mainControl.u;
        mainControl.buffer.i++;
    }

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Clear ADC INT1 flag
    PieCtrlRegs.PIEACK.all = 0x0001;     // Acknowledge PIE group 1 to enable further interrupts
}
//-----------------------------------------------------------------------------
static __interrupt void mainADCPPBISR(void){

    if( AdcaRegs.ADCEVTSTAT.bit.PPB1TRIPHI ){
        /* --- Disables PWM --- */
        mainCommandPWMDisable(0); //TODO: properly disable PWM/system (relays?)
        HWREG(IPC_BASE + IPC_O_ACK) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        HWREG(IPC_BASE + IPC_O_CLR) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        /* -------------------- */
        mainControl.status |= MAIN_STATUS_ADCA_PPB1_TRIP;
        AdcaRegs.ADCEVTCLR.bit.PPB1TRIPHI = 1;
    }

    if( AdcaRegs.ADCEVTSTAT.bit.PPB2TRIPHI ){
        /* --- Disables PWM --- */
        mainCommandPWMDisable(0); //TODO: properly disable PWM/system (relays?)
        HWREG(IPC_BASE + IPC_O_ACK) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        HWREG(IPC_BASE + IPC_O_CLR) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        /* -------------------- */
        mainControl.status |= MAIN_STATUS_ADCA_PPB2_TRIP;
        AdcaRegs.ADCEVTCLR.bit.PPB2TRIPHI = 1;
    }

    if( AdcbRegs.ADCEVTSTAT.bit.PPB1TRIPHI ){
        /* --- Disables PWM --- */
        mainCommandPWMDisable(0); //TODO: properly disable PWM/system (relays?)
        HWREG(IPC_BASE + IPC_O_ACK) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        HWREG(IPC_BASE + IPC_O_CLR) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        /* -------------------- */
        mainControl.status |= MAIN_STATUS_ADCB_PPB1_TRIP;
        AdcbRegs.ADCEVTCLR.bit.PPB1TRIPHI = 1;
    }

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;      // Clear ADCA INT2 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;      // Clear ADCB INT2 flag
    PieCtrlRegs.PIEACK.all = 0x0200;
}
//-----------------------------------------------------------------------------
//=============================================================================

//////
////// adca_ppb_isr - ISR for ADCA and ADCB
//////
//interrupt void adca_ppb_isr(void)
//{
//    //
//    //warning, you are outside of PPB limits
//    //
//
//    //Routine Vin
//    if(AdcaRegs.ADCEVTSTAT.bit.PPB1TRIPHI)
//    {
//        //
//        //voltage exceeded high limit
//        //
//        //asm("   ESTOP0");
//
//        //
//        //clear the trip flag and continue
//        //
//        AdcaRegs.ADCEVTCLR.bit.PPB1TRIPHI = 1;
//    }
//
//    //Routine IL
//    if(AdcaRegs.ADCEVTSTAT.bit.PPB2TRIPHI)
//        {
//            //
//            //voltage exceeded high limit
//            //
//            //asm("   ESTOP0");
//
//            //
//            //clear the trip flag and continue
//            //
//            AdcaRegs.ADCEVTCLR.bit.PPB2TRIPHI = 1;
//        }
//
//
//    //Routine Vout
//     if(AdcbRegs.ADCEVTSTAT.bit.PPB1TRIPHI)
//       {
//
//
//            //
//            //voltage exceeded high limit
//            //
//            //asm("   ESTOP0");
//
//            //
//            //clear the trip flag and continue
//            //
//            AdcbRegs.ADCEVTCLR.bit.PPB1TRIPHI = 1;
//      }
//
//
////    if(AdcaRegs.ADCEVTSTAT.bit.PPB1TRIPLO)
////    {
////        //
////        //voltage exceeded low limit
////        //
////        asm("   ESTOP0");
////
////        //
////        //clear the trip flag and continue
////        //
////        AdcaRegs.ADCEVTCLR.bit.PPB1TRIPLO = 1;
////    }
//
//
//    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;      // Clear ADCA INT2 flag
//    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;      // Clear ADCB INT2 flag
//    PieCtrlRegs.PIEACK.all = 0x0200;
//}

