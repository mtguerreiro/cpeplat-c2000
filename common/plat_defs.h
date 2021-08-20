/*
 * plat_defs.h
 *
 *  Created on: 10.06.2021
 *      Author: mguerreiro
 */

#ifndef PLAT_DEFS_H_
#define PLAT_DEFS_H_

#include "device.h"

//=============================================================================
/*--------------------------------- Defines ---------------------------------*/
//=============================================================================

/* CPU2 control modes */
typedef enum{
    PLAT_CPU2_CONTROL_MODE_NONE = 0,
    PLAT_CPU2_CONTROL_MODE_OL,
    PLAT_CPU2_CONTROL_MODE_PID,
    PLAT_CPU2_CONTROL_MODE_SFB,
    PLAT_CPU2_CONTROL_MODE_MATLAB,
    PLAT_CPU2_CONTROL_MODE_DMPC,
    PLAT_CPU2_CONTROL_MODE_END
}platCPU2ControlModeEnum_t;

/* CPU2 observer data */
typedef struct{
    uint16_t *adc[6];
    uint16_t *u;
    float states[2];
}platCPU2ObserverData_t;

/* CPU2 control data */
typedef struct{
    uint16_t *adc[6];
    uint16_t *u;
    platCPU2ObserverData_t *observer;
}platCPU2ControlData_t;

/* CPU2 observer modes */
typedef enum{
    PLAT_CPU2_OBSERVER_MODE_NONE = 0,
    PLAT_CPU2_OBSERVER_MODE_LUENBERGER,
    PLAT_CPU2_OBSERVER_MODE_CIMINI,
    PLAT_CPU2_OBSERVER_MODE_END,
}platCPU2ObserverModeEnum_t;

/* CPU1 commands */
typedef enum{
    PLAT_CMD_CPU1_BLINK = 1,
    PLAT_CMD_CPU1_CPU2_STATUS,
    PLAT_CMD_CPU1_CPU2_STATUS_CLEAR,
    PLAT_CMD_CPU1_CPU2_BLINK,
    PLAT_CMD_CPU1_CPU2_GPIO,
    PLAT_CMD_CPU1_CPU2_PWM_ENABLE,
    PLAT_CMD_CPU1_CPU2_PWM_DISABLE,
    PLAT_CMD_CPU1_ADC_BUFFER_SET,
    PLAT_CMD_CPU1_ADC_BUFFER_READ,
    PLAT_CMD_CPU1_CPU2_BUFFER_SET,
    PLAT_CMD_CPU1_CPU2_BUFFER_READ,
    PLAT_CMD_CPU1_CPU2_CONTROL_MODE_SET,
    PLAT_CMD_CPU1_CPU2_CONTROL_MODE_READ,
    PLAT_CMD_CPU1_CPU2_REF_SET,
    PLAT_CMD_CPU1_CPU2_REF_READ,
    PLAT_CMD_CPU1_CPU2_TRIP_SET,
    PLAT_CMD_CPU1_CPU2_TRIP_ENABLE,
    PLAT_CMD_CPU1_CPU2_TRIP_DISABLE,
    PLAT_CMD_CPU1_CPU2_TRIP_READ,
    PLAT_CMD_CPU1_CPU2_OBSERVER_MODE_SET,
    PLAT_CMD_CPU1_CPU2_OBSERVER_MODE_READ,
    PLAT_CMD_CPU1_CPU2_OBSERVER_ENABLE,
    PLAT_CMD_CPU1_CPU2_OBSERVER_DISABLE,
    PLAT_CMD_CPU1_CPU2_EVENT_SET,
    PLAT_CMD_CPU1_END
}platCPU1CommandsEnum_t;

/* CPU2 commands */
typedef enum{
    PLAT_CMD_CPU2_STATUS = 0,
    PLAT_CMD_CPU2_STATUS_CLEAR,
    PLAT_CMD_CPU2_BLINK,
    PLAT_CMD_CPU2_GPIO,
    PLAT_CMD_CPU2_PWM_ENABLE,
    PLAT_CMD_CPU2_PWM_DISABLE,
    PLAT_CMD_CPU2_BUFFER_SET,
    PLAT_CMD_CPU2_BUFFER_SAMPLES,
    PLAT_CMD_CPU2_BUFFER_ADDRESS,
    PLAT_CMD_CPU2_CONTROL_MODE_SET,
    PLAT_CMD_CPU2_CONTROL_MODE_READ,
    PLAT_CMD_CPU2_OBSERVER_MODE_SET,
    PLAT_CMD_CPU2_OBSERVER_MODE_READ,
    PLAT_CMD_CPU2_OBSERVER_ENABLE,
    PLAT_CMD_CPU2_OBSERVER_DISABLE,
    PLAT_CMD_CPU2_REF_SET,
    PLAT_CMD_CPU2_REF_READ,
    PLAT_CMD_CPU2_TRIP_SET,
    PLAT_CMD_CPU2_TRIP_ENABLE,
    PLAT_CMD_CPU2_TRIP_DISABLE,
    PLAT_CMD_CPU2_TRIP_READ,
    PLAT_CMD_CPU2_EVENT_SET,
    PLAT_CMD_CPU2_END
}platCPU2CommandsEnum_t;

/* CPU1 command errors */
#define PLAT_CMD_CPU1_ERR_CPU2_UNRESPONSIVE             1
#define PLAT_CMD_CPU1_ADC_BUFFER_SET_ERR_ADC            1
#define PLAT_CMD_CPU1_ADC_BUFFER_SET_ERR_SIZE           2
#define PLAT_CMD_CPU1_PWM_ENABLE_ERR_BUFFER             1
#define PLAT_CMD_CPU1_PWM_ENABLE_ERR_RAM_BUFFER         2
//#define PLAT_CMD_CPU1_CONTROL_MODE_SET_ERR_INVALID      6
#define PLAT_CMD_CPU1_CONTROL_MODE_ERR_RAM_BUFFER       1
#define PLAT_CMD_CPU1_REF_SET_ERR_INVALID               1
#define PLAT_CMD_CPU1_EVENT_SET_ERR_RAM_BUFFER          1

/* CPU2 commands errors */
#define PLAT_CMD_CPU2_PWM_ENABLE_ERR_STATUS             1
#define PLAT_CMD_CPU2_CONTROL_MODE_SET_ERR_INVALID      1
#define PLAT_CMD_CPU2_OBSERVER_MODE_SET_ERR_INVALID     1
#define PLAT_CMD_CPU2_OBSERVER_ENABLE_ERR_NO_OBS_SET    1
#define PLAT_CMD_CPU2_REF_SET_ERR_INVALID               1
#define PLAT_CMD_CPU2_BUFFER_SET_INVALID_BUFFER         1
#define PLAT_CMD_CPU2_BUFFER_SET_INVALID_SIZE           2
#define PLAT_CMD_CPU2_TRIP_SET_ERR_INVALID_ADC          1
#define PLAT_CMD_CPU2_TRIP_SET_ERR_INVALID_REF          2

/* GPIOs for CPU1 */
#define PLAT_CPU1_LED                   31U

/* GPIOs for CPU2 */
#define PLAT_CPU2_LED                   34U
#define PLAT_CPU2_GPIO_0                8U
#define PLAT_CPU2_GPIO_1                9U
#define PLAT_CPU2_GPIO_2                25U

/* IPC flag for commands */
#define PLAT_IPC_FLAG_CMD               0U
#define PLAT_IPC_FLAG_CPU2_CPU1_DATA    10U
#define PLAT_IPC_FLAG_CPU1_INIT         16U
#define PLAT_IPC_FLAG_CPU2_INIT         17U
#define PLAT_IPC_FLAG_MEM_OWN           18U

/* RAM section for ADC data (saved by CPU1) */
#define PLAT_CPU1_ADC_BUFFER_MAX        6
#define PLAT_CPU1_ADC_RAM_SEC           (MEMCFG_SECT_GS6 | MEMCFG_SECT_GS7 | MEMCFG_SECT_GS8 | MEMCFG_SECT_GS9)
#define PLAT_CPU1_ADC_RAM_ADD           0x00012000
#define PLAT_CPU1_ADC_RAM_SIZE          (0x001000 * 4)

/* RAM section for CPU1->CPU2 data exchange */
#define PLAT_CPU1_CPU2_DATA_RAM_SEC     (MEMCFG_SECT_GS10)
#define PLAT_CPU1_CPU2_DATA_RAM_ADD     0x00016000
#define PLAT_CPU1_CPU2_DATA_RAM_SIZE    (0x001000)

/* RAM section for CPU2 buffer */
#define PLAT_CPU2_BUFFER_MAX            4
#define PLAT_CPU2_BUFFER_RAM_SEC        (MEMCFG_SECT_GS11 | MEMCFG_SECT_GS12 | MEMCFG_SECT_GS13 | MEMCFG_SECT_GS14 | MEMCFG_SECT_GS15)
#define PLAT_CPU2_BUFFER_RAM_ADD        0x00017000
#define PLAT_CPU2_BUFFER_RAM_SIZE       (0x001000 * 5)

/* (0x03E7 >> 1) -> 200 kHz*/
#define PLAT_CONFIG_EPWM2_PERIOD        (0x03E7 << 1)
#define PLAT_CONFIG_EPWM4_PERIOD        (0x03E7 << 1)

/* Defines control gain */
#define PLAT_GAIN_CTL                   ( 1.0 / PLAT_CONFIG_EPWM4_PERIOD )

/* Defines gains for ADCs */
#define PLAT_GAIN_ADC_0                 ( 30.0 / 4095.0 )
#define PLAT_GAIN_ADC_1                 ( 30.0 / 4095.0 )
#define PLAT_GAIN_ADC_3                 ( 30.0 / 4095.0 )
#define PLAT_GAIN_ADC_5                 ( 30.0 / 4095.0 )

/* ADCs */
/*
 * ADCA - SOC0: ADCIN_A1 (Vin)
 * ADCA - SOC1: ADCIN_A4 (Vin_buck)
 * ADCA - SOC2: ADCIN_A5 (IL)
 * ADCB - SOC0: ADCIN_B4 (Vout)
 * ADCB - SOC1: ADCIN_B5 (IL_avg)
 * ADCC - SOC0: ADCIN_C4 (Vout_buck)
 */

//=============================================================================

#endif /* PLAT_DEFS_H_ */
