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

#include "control.h"
#include "dmpc.h"
#include "observer.h"
//=============================================================================

//=============================================================================
/*--------------------------------- Defines ---------------------------------*/
//=============================================================================
#define MAIN_CONFIG_EPWM2_PERIOD        PLAT_CONFIG_EPWM2_PERIOD
#define MAIN_CONFIG_EPWM4_PERIOD        PLAT_CONFIG_EPWM4_PERIOD

#define PLAT_CPU2_BUFFER_MAX            3

#define MAIN_STATUS_ADCA_PPB1_TRIP      (1 << 0)
#define MAIN_STATUS_ADCA_PPB2_TRIP      (1 << 1)
#define MAIN_STATUS_ADCA_PPB3_TRIP      (1 << 2)
#define MAIN_STATUS_ADCB_PPB1_TRIP      (1 << 3)
#define MAIN_STATUS_ADCB_PPB2_TRIP      (1 << 4)
#define MAIN_STATUS_ADCC_PPB1_TRIP      (1 << 5)
//=============================================================================

//===========================================================================
/*------------------------------- Data types ------------------------------*/
//===========================================================================
//---------------------------------------------------------------------------,
typedef void (*mainCommandHandle)(uint32_t data);
//---------------------------------------------------------------------------
typedef struct{
    uint16_t *p;
    uint16_t *pInit;
    uint16_t *pEnd;
    uint32_t size;
}mainBuffer_t;
//---------------------------------------------------------------------------
typedef struct{
    uint32_t i;
    uint32_t start;
    uint32_t end;
    uint32_t gpio;
}mainEvent_t;
//---------------------------------------------------------------------------
typedef struct{

    uint32_t blink;
    uint32_t status;

    mainCommandHandle handle[PLAT_CMD_CPU2_END];

    mainBuffer_t buffer[PLAT_CPU2_BUFFER_MAX];

    uint16_t u;
    uint16_t ref;

    uint32_t controlMode;
    platCPU2ControlData_t controlData;

    uint32_t observerMode;
    platCPU2ObserverData_t observerData;

    uint32_t eventMode;
    mainEvent_t event;

}mainControl_t;
//---------------------------------------------------------------------------
//===========================================================================

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

static void mainPWMEnable(void);
static void mainPWMDisable(void);

static void mainInitializeControlStructure(void);

static uint32_t mainBufferUpdate(void);

static void mainCommandInitializeHandlers(void);

static void mainCommandStatus(uint32_t data);
static void mainCommandStatusClear(uint32_t data);

static void mainCommandBlink(uint32_t data);

static void mainCommandGPIO(uint32_t data);

static void mainCommandPWMEnable(uint32_t data);
static void mainCommandPWMDisable(uint32_t data);

static void mainCommandCPU2BufferSet(uint32_t data);
static void mainCommandCPU2BufferSamples(uint32_t data);
static void mainCommandCPU2BufferAddress(uint32_t data);

static void mainCommandCPU2ControlModeSet(uint32_t data);
static void mainCommandCPU2ControlModeRead(uint32_t data);

static void mainCommandCPU2ObserverModeSet(uint32_t data);
static void mainCommandCPU2ObserverModeRead(uint32_t data);

static void mainCommandCPU2RefSet(uint32_t data);
static void mainCommandCPU2RefRead(uint32_t data);

static void mainCommandCPU2TripSet(uint32_t data);
static void mainCommandCPU2TripEnable(uint32_t data);
static void mainCommandCPU2TripDisable(uint32_t data);
static void mainCommandCPU2TripRead(uint32_t data);

static void mainCommandCPU2EventSet(uint32_t data);

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

    /* Initializes CPU2's control structure */
    mainInitializeControlStructure();

    /* Enable Global Interrupt (INTM) and realtime interrupt (DBGM) */
    EINT;
    ERTM;

    /* Signals CPU1 that we have initialized CPU2 */
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_INIT;
}
//-----------------------------------------------------------------------------
static void mainInitializeIPC(void){

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
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 2;      // End of SOC2 will set INT1 flag
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
    AdcbRegs.ADCINTSEL1N2.bit.INT2SEL = 1;      // End of SOC1 will set INT2 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT2E = 1;        // Enable INT2 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;      // Make sure INT2 flag is cleared

    //ADC-C
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 4;          // SOC0 will convert pin C4
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 14;         // Sample window is 100 SYSCLK cycles
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 7;        // Trigger on ePWM2 SOCA/C

    //Flag ADC-C Limit
    AdccRegs.ADCINTSEL1N2.bit.INT2SEL = 0;      // End of SOC0 will set INT2 flag
    AdccRegs.ADCINTSEL1N2.bit.INT2E = 1;        // Enable INT2 flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;      // Make sure INT2 flag is cleared
    EDIS;
}
//-----------------------------------------------------------------------------
static void mainInitializeADCLimits(void){

    EALLOW;

    /*
     * Sets ADCA SOC 0 limit. In the buck platform, ADCA SOC0 (ADCIN_A1)
     * corresponds to Vin.
     */
    AdcaRegs.ADCPPB1CONFIG.bit.CONFIG = 0;  //PPB1A is associated with soc0 -> Vin

    /* Clears upper and lower limits */
    AdcaRegs.ADCPPB1TRIPHI.bit.LIMITHI = 0;
    AdcaRegs.ADCPPB1TRIPLO.bit.LIMITLO = 0;

    /* Sets upper events to generate interrupts */
    AdcaRegs.ADCEVTINTSEL.bit.PPB1TRIPHI = 1;
    AdcaRegs.ADCEVTINTSEL.bit.PPB1TRIPLO = 0;

    /*
     * Sets ADCA SOC 1 limit. In the buck platform, ADCA SOC1 (ADCIN_A4)
     * corresponds to Vin_buck.
     */
    AdcaRegs.ADCPPB2CONFIG.bit.CONFIG = 1;  //PPB2A is associated with soc1 -> Vin_buck

    /* Clears upper and lower limits */
    AdcaRegs.ADCPPB2TRIPHI.bit.LIMITHI = 0;
    AdcaRegs.ADCPPB2TRIPLO.bit.LIMITLO = 0;

    /* Sets upper events to generate interrupts */
    AdcaRegs.ADCEVTINTSEL.bit.PPB2TRIPHI = 1;
    AdcaRegs.ADCEVTINTSEL.bit.PPB2TRIPLO = 0;

    /*
     * Sets ADCA SOC 2 limit. In the buck platform, ADCA SOC2 (ADCIN_A5)
     * corresponds to IL.
     */
    AdcaRegs.ADCPPB3CONFIG.bit.CONFIG = 2;  //PPB3A is associated with soc2 -> IL

    /* Clears upper and lower limits */
    AdcaRegs.ADCPPB3TRIPHI.bit.LIMITHI = 0;
    AdcaRegs.ADCPPB3TRIPLO.bit.LIMITLO = 0;

    /* Sets upper events to generate interrupts */
    AdcaRegs.ADCEVTINTSEL.bit.PPB3TRIPHI = 1;
    AdcaRegs.ADCEVTINTSEL.bit.PPB3TRIPLO = 0;

    /*
     * Sets ADCB SOC 0 limit. In the buck platform, ADCB SOC0 (ADCIN_B4)
     * corresponds to Vout.
     */
    AdcbRegs.ADCPPB1CONFIG.bit.CONFIG = 0;  //PPB1B is associated with soc0 -> Vout

    /* Clears upper and lower limits */
    AdcbRegs.ADCPPB1TRIPHI.bit.LIMITHI = 0;
    AdcbRegs.ADCPPB1TRIPLO.bit.LIMITLO = 0;

    /* Sets upper events to generate interrupts */
    AdcbRegs.ADCEVTINTSEL.bit.PPB1TRIPHI = 1;
    AdcbRegs.ADCEVTINTSEL.bit.PPB1TRIPLO = 0;

    /*
     * Sets ADCB SOC 1 limit. In the buck platform, ADCB SOC1 (ADCIN_B5)
     * corresponds to IL_avg.
     */
    AdcbRegs.ADCPPB2CONFIG.bit.CONFIG = 1;  //PPB2B is associated with soc1 -> IL_avg

    /* Clears upper and lower limits */
    AdcbRegs.ADCPPB2TRIPHI.bit.LIMITHI = 0;
    AdcbRegs.ADCPPB2TRIPLO.bit.LIMITLO = 0;

    /* Sets upper events to generate interrupts */
    AdcbRegs.ADCEVTINTSEL.bit.PPB2TRIPHI = 1;
    AdcbRegs.ADCEVTINTSEL.bit.PPB2TRIPLO = 0;

    /*
     * Sets ADCC SOC 0 limit. In the buck platform, ADCC SOC0 (ADCIN_C4)
     * corresponds to Vout_buck.
     */
    AdccRegs.ADCPPB1CONFIG.bit.CONFIG = 0;  //PPB1C is associated with soc0 -> Vout_buck

    /* Clears upper and lower limits */
    AdccRegs.ADCPPB1TRIPHI.bit.LIMITHI = 0;
    AdccRegs.ADCPPB1TRIPLO.bit.LIMITLO = 0;

    /* Sets upper events to generate interrupts */
    AdccRegs.ADCEVTINTSEL.bit.PPB1TRIPHI = 1;
    AdccRegs.ADCEVTINTSEL.bit.PPB1TRIPLO = 0;

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

    Interrupt_register(INT_ADCC_EVT, mainADCPPBISR);
    Interrupt_enable(INT_ADCC_EVT);
}
//-----------------------------------------------------------------------------
static void mainInitializeEPWM(void){

    // Initialize System Control
    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC= 0;                // Turn off all clocks at the same time
    EDIS;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    mainInitializeEPWM2();
    mainInitializeEPWM4();

    // Sync ePWM
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;               // Turn on all clocks at the same time
    EDIS;
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

    //Sets output to high for incoming trip
    EPwm2Regs.TZCTL.bit.TZA = 2;                // EPWM2A forces to low
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
     EPwm4Regs.CMPA.bit.CMPA = 0;                 // Set compare A value

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

     //Sets output to high for incoming trip
     EPwm4Regs.TZCTL.bit.TZA = 1;                 // EPWM4A forces to high
     EPwm4Regs.TZCTL.bit.TZB = 2;                 // EPWM4B forces to low

     /* Enables counter */
     EPwm4Regs.TBCTL.bit.CTRMODE = 0;             // Enable Up-count mode

     EDIS;
}
//-----------------------------------------------------------------------------
static void mainPWMEnable(void){

    // Start ePWM
    EALLOW;
    EPwm4Regs.TZCLR.bit.OST = 1;                    // clear trip zone flags
    EPwm4Regs.CMPA.bit.CMPA = 0;                    // Set compare A value

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC= 1;        // Turn on all clocks at the same time
    EPwm2Regs.ETSEL.bit.SOCAEN = 1;             // Enable SOCA
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;            // Un-freeze and enter up-count mode

    EPwm4Regs.TBCTL.bit.CTRMODE = 0;            // Un-freeze and enter up-count mode
    EDIS;
}
//-----------------------------------------------------------------------------
static void mainPWMDisable(void){

    // Stop ePWM
    EALLOW;

    EPwm4Regs.TZFRC.bit.OST = 1;                //Trigger Safety Status

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC= 0;        // Turn off all clocks
    EPwm2Regs.ETSEL.bit.SOCAEN = 0;             // Disable SOCA
    EPwm2Regs.TBCTL.bit.CTRMODE = 3;            // Freezes counter
    EPwm2Regs.TBCTR = 0x0000;                   // Clear Counter

    EPwm4Regs.TBCTL.bit.CTRMODE = 3;            // Freezes counter
    EPwm4Regs.TBCTR = 0x0000;                   // Clear Counter
    EDIS;
}
//-----------------------------------------------------------------------------
static void mainInitializeBuffer(void){

    uint32_t k;

    /* Initializes the buffer control structure */
    for(k = 0; k < PLAT_CPU2_BUFFER_MAX; k++){
        mainControl.buffer[k].p = (uint16_t *)PLAT_CPU2_BUFFER_RAM_SEC;
        mainControl.buffer[k].pInit = (uint16_t *)PLAT_CPU2_BUFFER_RAM_SEC;
        mainControl.buffer[k].pEnd = (uint16_t *)PLAT_CPU2_BUFFER_RAM_SEC;
        mainControl.buffer[k].size = 0;
    }
}
//-----------------------------------------------------------------------------
static uint32_t mainBufferUpdate(void){

    uint32_t size, k;
    uint16_t *p;

    size = 0;
    for(k = 0; k < PLAT_CPU2_BUFFER_MAX; k++){
        size += mainControl.buffer[k].size;
    }

    if( size > PLAT_CPU2_BUFFER_RAM_SIZE ){
        return 1;
    }

    p = (uint16_t *)PLAT_CPU2_BUFFER_RAM_ADD;
    for(k = 0; k < PLAT_CPU2_BUFFER_MAX; k++){
        mainControl.buffer[k].p = p;
        mainControl.buffer[k].pInit = p;
        p += mainControl.buffer[k].size;
        mainControl.buffer[k].pEnd = p;
    }

    return 0;
}
//-----------------------------------------------------------------------------
static void mainInitializeControlStructure(void){

    /* Initializes controllers and observers */
    controlInitialize();
    observerInitialize();

    /* Clears status and initializes u and ref to zero */
    mainControl.status = 0;
    mainControl.ref = 0;
    mainControl.u = 0;

    /* Initializes control data */
    mainControl.controlMode = 0;

    mainControl.controlData.adc[0] = (uint16_t *)(ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + 0);
    mainControl.controlData.adc[1] = (uint16_t *)(ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + 1);
    mainControl.controlData.adc[2] = (uint16_t *)(ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + 2);
    mainControl.controlData.adc[3] = (uint16_t *)(ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + 0);
    mainControl.controlData.adc[4] = (uint16_t *)(ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + 1);
    mainControl.controlData.adc[5] = (uint16_t *)(ADCCRESULT_BASE + ADC_RESULTx_OFFSET_BASE + 0);

    mainControl.controlData.u = &mainControl.u;

    /* Initializes observer data */
    mainControl.controlData.observer = &mainControl.observerData;
        
    mainControl.observerData.adc[0] = (uint16_t *)(ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + 0);
    mainControl.observerData.adc[1] = (uint16_t *)(ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + 1);
    mainControl.observerData.adc[2] = (uint16_t *)(ADCARESULT_BASE + ADC_RESULTx_OFFSET_BASE + 2);
    mainControl.observerData.adc[3] = (uint16_t *)(ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + 0);
    mainControl.observerData.adc[4] = (uint16_t *)(ADCBRESULT_BASE + ADC_RESULTx_OFFSET_BASE + 1);
    mainControl.observerData.adc[5] = (uint16_t *)(ADCCRESULT_BASE + ADC_RESULTx_OFFSET_BASE + 0);

    mainControl.observerData.u = &mainControl.u;

    /* Initializes event data */
    mainControl.eventMode = 0;
    mainControl.event.i = 0;
    mainControl.event.end = 0;
    mainControl.event.start = 0;
    mainControl.event.gpio = 0xFFFFFFFF;
}
//-----------------------------------------------------------------------------
static void mainCommandInitializeHandlers(void){

    mainControl.handle[PLAT_CMD_CPU2_STATUS] = mainCommandStatus;
    mainControl.handle[PLAT_CMD_CPU2_STATUS_CLEAR] = mainCommandStatusClear;

    mainControl.handle[PLAT_CMD_CPU2_BLINK] = mainCommandBlink;

    mainControl.handle[PLAT_CMD_CPU2_GPIO] = mainCommandGPIO;

    mainControl.handle[PLAT_CMD_CPU2_PWM_ENABLE] = mainCommandPWMEnable;
    mainControl.handle[PLAT_CMD_CPU2_PWM_DISABLE] = mainCommandPWMDisable;

    mainControl.handle[PLAT_CMD_CPU2_BUFFER_SET] = mainCommandCPU2BufferSet;
    mainControl.handle[PLAT_CMD_CPU2_BUFFER_SAMPLES] = mainCommandCPU2BufferSamples;
    mainControl.handle[PLAT_CMD_CPU2_BUFFER_ADDRESS] = mainCommandCPU2BufferAddress;

    mainControl.handle[PLAT_CMD_CPU2_CONTROL_MODE_SET] = mainCommandCPU2ControlModeSet;
    mainControl.handle[PLAT_CMD_CPU2_CONTROL_MODE_READ] = mainCommandCPU2ControlModeRead;

    mainControl.handle[PLAT_CMD_CPU2_OBSERVER_MODE_SET] = mainCommandCPU2ObserverModeSet;
    mainControl.handle[PLAT_CMD_CPU2_OBSERVER_MODE_READ] = mainCommandCPU2ObserverModeRead;

    mainControl.handle[PLAT_CMD_CPU2_REF_SET] = mainCommandCPU2RefSet;
    mainControl.handle[PLAT_CMD_CPU2_REF_READ] = mainCommandCPU2RefRead;

    mainControl.handle[PLAT_CMD_CPU2_TRIP_SET] = mainCommandCPU2TripSet;
    mainControl.handle[PLAT_CMD_CPU2_TRIP_ENABLE] = mainCommandCPU2TripEnable;
    mainControl.handle[PLAT_CMD_CPU2_TRIP_DISABLE] = mainCommandCPU2TripDisable;
    mainControl.handle[PLAT_CMD_CPU2_TRIP_READ] = mainCommandCPU2TripRead;

    mainControl.handle[PLAT_CMD_CPU2_EVENT_SET] = mainCommandCPU2EventSet;
}
//-----------------------------------------------------------------------------
static void mainCommandStatus(uint32_t data){

    HWREG(IPC_BASE + IPC_O_SENDDATA) = mainControl.status;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
}
//-----------------------------------------------------------------------------
static void mainCommandStatusClear(uint32_t data){

    mainControl.status = 0;

    HWREG(IPC_BASE + IPC_O_SENDDATA) = mainControl.status;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
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

    HWREG(IPC_BASE + IPC_O_SENDDATA) = data;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
}
//-----------------------------------------------------------------------------
static void mainCommandPWMEnable(uint32_t data){

    /* Doesn't enable PWM if any status flag is set */
    if( mainControl.status != 0 ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
        HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_PWM_ENABLE_ERR_STATUS;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;

    mainBufferUpdate();
    
    mainPWMEnable();
}
//-----------------------------------------------------------------------------
static void mainCommandPWMDisable(uint32_t data){

    HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
    
    mainPWMDisable();
}
//-----------------------------------------------------------------------------
static void mainCommandCPU2BufferSet(uint32_t data){

    uint32_t buffer, size, sizeOriginal;

    buffer = data >> 16;
    size = data & 0xFFFF;

    if( (buffer + 1) > PLAT_CPU2_BUFFER_MAX ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
        HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_BUFFER_SET_INVALID_BUFFER;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    sizeOriginal = mainControl.buffer[buffer].size;

    /* Tries to update buffer. If it fails, restores the original size. */
    mainControl.buffer[buffer].size = size;
    if( mainBufferUpdate() != 0 ){
        mainControl.buffer[buffer].size = sizeOriginal;
        mainBufferUpdate();
        HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
        HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_BUFFER_SET_INVALID_SIZE;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    HWREG(IPC_BASE + IPC_O_SENDDATA) = data;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
}
//-----------------------------------------------------------------------------
static void mainCommandCPU2BufferSamples(uint32_t data){

    uint32_t buffer, size;

    buffer = data;

    /* Checks if buffer is valid */
    if( (buffer + 1) > PLAT_CPU2_BUFFER_MAX ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
        HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_BUFFER_SET_INVALID_BUFFER;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    size = (uint32_t)(mainControl.buffer[buffer].p - mainControl.buffer[buffer].pInit);

    HWREG(IPC_BASE + IPC_O_SENDDATA) = size;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
}
//-----------------------------------------------------------------------------
static void mainCommandCPU2BufferAddress(uint32_t data){

    uint32_t buffer, address;

    buffer = data;

    /* Checks if buffer is valid */
    if( (buffer + 1) > PLAT_CPU2_BUFFER_MAX ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
        HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_BUFFER_SET_INVALID_BUFFER;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    address = (uint32_t)(mainControl.buffer[buffer].pInit);

    HWREG(IPC_BASE + IPC_O_SENDDATA) = address;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
}
//-----------------------------------------------------------------------------
static void mainCommandCPU2ControlModeSet(uint32_t data){

    uint32_t *p;

    /* Checks if control mode is valid */
    if( data > PLAT_CPU2_CONTROL_MODE_END ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
        HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_CONTROL_MODE_SET_ERR_INVALID;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    /* Sets the controller */
    if( data != PLAT_CPU2_CONTROL_MODE_NONE ){
        p = (uint32_t *)PLAT_CPU1_CPU2_DATA_RAM_ADD;
        if( controlSet((controlModeEnum_t)data, p) != 0 ){
            HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
            HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_CONTROL_MODE_SET_ERR_INVALID;
            HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
            return;
        }
    }

    HWREG(IPC_BASE + IPC_O_SENDDATA) = data;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;

    mainControl.controlMode = data;
    mainControl.u = 0;
}
//-----------------------------------------------------------------------------
static void mainCommandCPU2ControlModeRead(uint32_t data){

    /* Checks if control mode is valid */
    if( data > PLAT_CPU2_CONTROL_MODE_END ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
        HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_CONTROL_MODE_SET_ERR_INVALID;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    HWREG(IPC_BASE + IPC_O_SENDDATA) = mainControl.controlMode;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
}
//-----------------------------------------------------------------------------
static void mainCommandCPU2ObserverModeSet(uint32_t data){

    uint32_t *p;

    /* Checks if observer mode is valid */
    if( data > PLAT_CPU2_OBSERVER_MODE_END ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
        HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_OBSERVER_MODE_SET_ERR_INVALID;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    /* Sets the observer */
    p = (uint32_t *)PLAT_CPU1_CPU2_DATA_RAM_ADD;
    if( observerSet((observerModeEnum_t)data, p) != 0 ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
        HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_OBSERVER_MODE_SET_ERR_INVALID;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    HWREG(IPC_BASE + IPC_O_SENDDATA) = data;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;

    mainControl.observerMode = data;
}
//-----------------------------------------------------------------------------
static void mainCommandCPU2ObserverModeRead(uint32_t data){

    /* Checks if observer mode is valid */
    if( data > PLAT_CPU2_OBSERVER_MODE_END ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
        HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_OBSERVER_MODE_SET_ERR_INVALID;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    HWREG(IPC_BASE + IPC_O_SENDDATA) = mainControl.observerMode;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
}
//-----------------------------------------------------------------------------
static void mainCommandCPU2RefSet(uint32_t data){

    /* checks the reference */
    if( data > 4095 ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
        HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_REF_SET_ERR_INVALID;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    mainControl.ref = data;

    HWREG(IPC_BASE + IPC_O_SENDDATA) = mainControl.ref;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
}
//-----------------------------------------------------------------------------
static void mainCommandCPU2RefRead(uint32_t data){

    HWREG(IPC_BASE + IPC_O_SENDDATA) = mainControl.ref;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
}
//-----------------------------------------------------------------------------
static void mainCommandCPU2TripSet(uint32_t data){

    uint32_t adc, ref;

    adc = data & 0xFFFF;

    if( (adc + 1) > 6 ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
        HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_TRIP_SET_ERR_INVALID_ADC;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    ref = data >> 16;

    if( ref > 4095 ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
        HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_TRIP_SET_ERR_INVALID_REF;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    EALLOW;
    if( adc == 0 ){
        AdcaRegs.ADCPPB1TRIPHI.bit.LIMITHI = (uint16_t)ref;
    }
    else if( adc == 1){
        AdcaRegs.ADCPPB2TRIPHI.bit.LIMITHI = (uint16_t)ref;
    }
    else if( adc == 2){
        AdcaRegs.ADCPPB3TRIPHI.bit.LIMITHI = (uint16_t)ref;
    }
    else if( adc == 3){
        AdcbRegs.ADCPPB1TRIPHI.bit.LIMITHI = (uint16_t)ref;
    }
    else if( adc == 4){
        AdcbRegs.ADCPPB2TRIPHI.bit.LIMITHI = (uint16_t)ref;
    }
    else if( adc == 5){
        AdccRegs.ADCPPB1TRIPHI.bit.LIMITHI = (uint16_t)ref;
    }
    EDIS;

    HWREG(IPC_BASE + IPC_O_SENDDATA) = data;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
}
//-----------------------------------------------------------------------------
static void mainCommandCPU2TripEnable(uint32_t data){

    uint32_t adc;

    adc = data;

    if( (adc + 1) > 6 ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
        HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_TRIP_SET_ERR_INVALID_ADC;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    EALLOW;
    if( adc == 0 ){
        AdcaRegs.ADCEVTINTSEL.bit.PPB1TRIPHI = 1;
    }
    else if( adc == 1){
        AdcaRegs.ADCEVTINTSEL.bit.PPB2TRIPHI = 1;
    }
    else if( adc == 2){
        AdcaRegs.ADCEVTINTSEL.bit.PPB3TRIPHI = 1;
    }
    else if( adc == 3){
        AdcbRegs.ADCEVTINTSEL.bit.PPB1TRIPHI = 1;
    }
    else if( adc == 4){
        AdcbRegs.ADCEVTINTSEL.bit.PPB2TRIPHI = 1;
    }
    else if( adc == 5){
        AdccRegs.ADCEVTINTSEL.bit.PPB1TRIPHI = 1;
    }
    EDIS;

    HWREG(IPC_BASE + IPC_O_SENDDATA) = data;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
}
//-----------------------------------------------------------------------------
static void mainCommandCPU2TripDisable(uint32_t data){

    uint32_t adc;

    adc = data;

    if( (adc + 1) > 6 ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
        HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_TRIP_SET_ERR_INVALID_ADC;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    EALLOW;
    if( adc == 0 ){
        AdcaRegs.ADCEVTINTSEL.bit.PPB1TRIPHI = 0;
    }
    else if( adc == 1){
        AdcaRegs.ADCEVTINTSEL.bit.PPB2TRIPHI = 0;
    }
    else if( adc == 2){
        AdcaRegs.ADCEVTINTSEL.bit.PPB3TRIPHI = 0;
    }
    else if( adc == 3){
        AdcbRegs.ADCEVTINTSEL.bit.PPB1TRIPHI = 0;
    }
    else if( adc == 4){
        AdcbRegs.ADCEVTINTSEL.bit.PPB2TRIPHI = 0;
    }
    else if( adc == 5){
        AdccRegs.ADCEVTINTSEL.bit.PPB1TRIPHI = 0;
    }
    EDIS;

    HWREG(IPC_BASE + IPC_O_SENDDATA) = data;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
}
//-----------------------------------------------------------------------------
static void mainCommandCPU2TripRead(uint32_t data){

    uint32_t adc, ref;

    adc = data & 0xFFFF;

    if( (adc + 1) > 6 ){
        HWREG(IPC_BASE + IPC_O_SENDDATA) = 0;
        HWREG(IPC_BASE + IPC_O_SENDCOM) = PLAT_CMD_CPU2_TRIP_SET_ERR_INVALID_ADC;
        HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
        return;
    }

    EALLOW;
    if( adc == 0 ){
        ref = AdcaRegs.ADCPPB1TRIPHI.bit.LIMITHI;
    }
    else if( adc == 1){
        ref = AdcaRegs.ADCPPB2TRIPHI.bit.LIMITHI;
    }
    else if( adc == 2){
        ref = AdcaRegs.ADCPPB3TRIPHI.bit.LIMITHI;
    }
    else if( adc == 3){
        ref = AdcbRegs.ADCPPB1TRIPHI.bit.LIMITHI;
    }
    else if( adc == 4){
        ref = AdcbRegs.ADCPPB2TRIPHI.bit.LIMITHI;
    }
    else if( adc == 5){
        ref = AdccRegs.ADCPPB1TRIPHI.bit.LIMITHI;
    }
    EDIS;

    data = (ref << 16) | adc;
    HWREG(IPC_BASE + IPC_O_SENDDATA) = data;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
}
//-----------------------------------------------------------------------------
static void mainCommandCPU2EventSet(uint32_t data){

    uint32_t *p;

    p = (uint32_t *)PLAT_CPU1_CPU2_DATA_RAM_ADD;

    mainControl.event.gpio = *p++;
    mainControl.event.start = *p++;
    mainControl.event.end = *p++;

    mainControl.event.i = 0;
    mainControl.eventMode = 1;

    HWREG(IPC_BASE + IPC_O_SENDDATA) = data;
    HWREG(IPC_BASE + IPC_O_SENDCOM) = 0;
    HWREG(IPC_BASE + IPC_O_SET) = 1UL << PLAT_IPC_FLAG_CPU2_CPU1_DATA;
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

    float u;

    GPIO_writePin(PLAT_CPU2_GPIO_2, 1);

    /* Clears ADC INT1 flags and acks PIE group 1 for further interrupts */
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = 0x0001;

    /* Checks for event generation */
    if( mainControl.eventMode != 0 ){
        if( (mainControl.event.i > mainControl.event.start) && (mainControl.event.i < mainControl.event.end) ){
            GPIO_writePin(mainControl.event.gpio, 1);
        }
        if( mainControl.event.i > mainControl.event.end ){
            GPIO_writePin(mainControl.event.gpio, 0);
            mainControl.eventMode = 0;
        }
        mainControl.event.i++;
    }

    /* Executes observer before controller, if enabled */
    if( mainControl.observerMode != PLAT_CPU2_OBSERVER_MODE_NONE ){
        observerObserve((observerModeEnum_t)mainControl.observerMode, \
                        &mainControl.observerData);
    }

    /* Executes controller, if enabled */
    if( mainControl.controlMode != PLAT_CPU2_CONTROL_MODE_NONE ){
        u = controlControl((controlModeEnum_t)mainControl.controlMode,\
                           mainControl.ref, &mainControl.controlData);
        u = u * ((float)MAIN_CONFIG_EPWM2_PERIOD);
        mainControl.u = (uint16_t)u;
    }

    /* Updates PWM */
    EPwm4Regs.CMPA.bit.CMPA = mainControl.u;

    /* Saves relevant data to buffers */
    if( mainControl.buffer[0].p != mainControl.buffer[0].pEnd ){
        *mainControl.buffer[0].p++ = mainControl.u;
    }
    if( mainControl.buffer[1].p != mainControl.buffer[1].pEnd ){
        *mainControl.buffer[1].p++ = controlDMPCIters();
    }
    if( mainControl.buffer[2].p != mainControl.buffer[2].pEnd ){
        uint32_t data32;

        data32 = *((uint32_t *)(&mainControl.observerData.states[0]));

        *mainControl.buffer[2].p++ = (uint16_t)(data32 & 0xFF);
        *mainControl.buffer[2].p++ = (uint16_t)(data32 >> 16);
    }

//    if( mainControl.buffer[1].p != mainControl.buffer[1].pEnd ){
//        uint32_t data32;
//
//        data32 = *((uint32_t *)(&mainControl.observerData.states[0]));
//
//        *mainControl.buffer[1].p++ = (uint16_t)(data32 & 0xFF);
//        *mainControl.buffer[1].p++ = (uint16_t)(data32 >> 16);
//    }
//    if( mainControl.buffer[2].p != mainControl.buffer[2].pEnd ){
//        uint32_t data32;
//
//        data32 = *((uint32_t *)(&mainControl.observerData.states[1]));
//
//        *mainControl.buffer[2].p++ = (uint16_t)(data32 & 0xFF);
//        *mainControl.buffer[2].p++ = (uint16_t)(data32 >> 16);
//    }

    GPIO_writePin(PLAT_CPU2_GPIO_2, 0);
}
//-----------------------------------------------------------------------------
static __interrupt void mainADCPPBISR(void){

    if( AdcaRegs.ADCEVTSTAT.bit.PPB1TRIPHI ){
        mainPWMDisable();
        mainControl.status |= MAIN_STATUS_ADCA_PPB1_TRIP;
        AdcaRegs.ADCEVTCLR.bit.PPB1TRIPHI = 1;
    }

    if( AdcaRegs.ADCEVTSTAT.bit.PPB2TRIPHI ){
        mainPWMDisable();
        mainControl.status |= MAIN_STATUS_ADCA_PPB2_TRIP;
        AdcaRegs.ADCEVTCLR.bit.PPB1TRIPHI = 1;
    }

    if( AdcaRegs.ADCEVTSTAT.bit.PPB3TRIPHI ){
        mainPWMDisable();
        mainControl.status |= MAIN_STATUS_ADCA_PPB3_TRIP;
        AdcaRegs.ADCEVTCLR.bit.PPB3TRIPHI = 1;
    }

    if( AdcbRegs.ADCEVTSTAT.bit.PPB1TRIPHI ){
        mainPWMDisable();
        mainControl.status |= MAIN_STATUS_ADCB_PPB1_TRIP;
        AdcbRegs.ADCEVTCLR.bit.PPB1TRIPHI = 1;
    }

    if( AdcbRegs.ADCEVTSTAT.bit.PPB2TRIPHI ){
        mainPWMDisable();
        mainControl.status |= MAIN_STATUS_ADCB_PPB2_TRIP;
        AdcbRegs.ADCEVTCLR.bit.PPB2TRIPHI = 1;
    }

    if( AdccRegs.ADCEVTSTAT.bit.PPB1TRIPHI ){
        mainPWMDisable();
        mainControl.status |= MAIN_STATUS_ADCC_PPB1_TRIP;
        AdccRegs.ADCEVTCLR.bit.PPB1TRIPHI = 1;
    }

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;      // Clear ADCA INT2 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;      // Clear ADCB INT2 flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;      // Clear ADCC INT2 flag
    PieCtrlRegs.PIEACK.all = 0x0200;
}
//-----------------------------------------------------------------------------
//=============================================================================
