/*
 * plat_defs.h
 *
 *  Created on: 10.06.2021
 *      Author: LRS
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
    uint16_t *adc[7];
    uint16_t *u;
    float states[2];
}platCPU2ObserverData_t;

/* CPU2 control data */
typedef struct{
    uint16_t *adc[7];
    uint16_t *u;
    platCPU2ObserverData_t *observer;
}platCPU2ControlData_t;

/* CPU2 observer modes */
typedef enum{
    PLAT_CPU2_OBSERVER_MODE_NONE = 0,
    PLAT_CPU2_OBSERVER_MODE_LUENBERGER,
    PLAT_CPU2_OBSERVER_MODE_CIMINI,
    PLAT_CPU2_OBSERVER_PREDICTIVE,
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
    PLAT_CMD_CPU1_CPU2_ADC_TRIM,
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
    PLAT_CMD_CPU2_ADC_TRIM,
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
#define PLAT_CPU2_GPIO_3                10U

/* IPC flag for commands */
#define PLAT_IPC_FLAG_CMD               0U
#define PLAT_IPC_FLAG_CPU2_CPU1_DATA    10U
#define PLAT_IPC_FLAG_CPU1_INIT         16U
#define PLAT_IPC_FLAG_CPU2_INIT         17U
#define PLAT_IPC_FLAG_MEM_OWN           18U

/* RAM section for ADC data (saved by CPU1) */
#define PLAT_CPU1_ADC_BUFFER_MAX        7
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

/* (500 - 1) -> 200 kHz */
#define PLAT_CONFIG_EPWM2_PERIOD        (2000 - 1)
#define PLAT_CONFIG_EPWM4_PERIOD        (2000 - 1)

/* Defines control gain */
#define PLAT_CONFIG_GAIN_CTL                   ( 1.0 / PLAT_CONFIG_EPWM4_PERIOD )

/* Gain for reference */
#define PLAT_CONFIG_GAIN_REF            (30.0 / 4095.0)

/* ADCs mapping (don't change this) */
#define PLAT_CONFIG_ADC_A1              1
#define PLAT_CONFIG_ADC_A4              4
#define PLAT_CONFIG_ADC_A5              5
#define PLAT_CONFIG_ADC_B4              4
#define PLAT_CONFIG_ADC_B5              5
#define PLAT_CONFIG_ADC_C4              4
#define PLAT_CONFIG_ADC_C5              5

/* SOC settings */
#define PLAT_CONFIG_ADC_A_SOC0_SEL      PLAT_CONFIG_ADC_A5
#define PLAT_CONFIG_ADC_A_SOC1_SEL      PLAT_CONFIG_ADC_A4
#define PLAT_CONFIG_ADC_A_SOC2_SEL      PLAT_CONFIG_ADC_A1
#define PLAT_CONFIG_ADC_B_SOC0_SEL      PLAT_CONFIG_ADC_B4
#define PLAT_CONFIG_ADC_B_SOC1_SEL      PLAT_CONFIG_ADC_B5
#define PLAT_CONFIG_ADC_C_SOC0_SEL      PLAT_CONFIG_ADC_C4
#define PLAT_CONFIG_ADC_C_SOC1_SEL      PLAT_CONFIG_ADC_C5

//=============================================================================

//=============================================================================
/*---------------------------- Buck definitions -----------------------------*/
//=============================================================================
/*
 * Buck measurements
 *
 * +----------+-------------+-----------+--------+
 * |    ADC   | Measurement |    SOC    | Buffer |
 * +----------+-------------+-----------+--------+
 * | ADCIN_A1 |     V_in    | ADCA_SOC2 |    2   |
 * +----------+-------------+-----------+--------+
 * | ADCIN_A4 |  V_in_buck  | ADCA_SOC1 |    1   |
 * +----------+-------------+-----------+--------+
 * | ADCIN_B4 |    V_out    | ADCB_SOC0 |    3   |
 * +----------+-------------+-----------+--------+
 * | ADCIN_C4 |  V_out_buck | ADCC_SOC0 |    5   |
 * +----------+-------------+-----------+--------+
 * | ADCIN_A5 |      IL     | ADCA_SOC0 |    0   |
 * +----------+-------------+-----------+--------+
 * | ADCIN_B5 |    IL_avg   | ADCB_SOC1 |    4   |
 * +----------+-------------+-----------+--------+
 */

/*
 * Index of stored buck measurements.
 *
 * If any SOC is changed, the buffer index must be changed to reflect where
 * the measurement is being stored. All control algorithms for the buck
 * converter rely in these indexes to get the measurements from the buffers.
 */
#define PLAT_CONFIG_BUCK_V_IN_BUFFER            2
#define PLAT_CONFIG_BUCK_V_IN_BUCK_BUFFER       1
#define PLAT_CONFIG_BUCK_V_OUT_BUFFER           3
#define PLAT_CONFIG_BUCK_V_OUT_BUCK_BUFFER      5
#define PLAT_CONFIG_BUCK_IL_BUFFER              0
#define PLAT_CONFIG_BUCK_IL_AVG_BUCK_BUFFER     4

/* Gains for buck measurements */
#define PLAT_CONFIG_BUCK_V_IN_GAIN              (3 * (16 / 1.6588) / 4095)
#define PLAT_CONFIG_BUCK_V_IN_OFFS              (0.0)
#define PLAT_CONFIG_BUCK_V_IN_BUCK_GAIN         (3 * (16 / 1.6416) / 4095)
#define PLAT_CONFIG_BUCK_V_IN_BUCK_OFFS         (0.0)
#define PLAT_CONFIG_BUCK_V_OUT_GAIN             (3 * (5.83 / 0.5927) / 4095.0)
#define PLAT_CONFIG_BUCK_V_OUT_OFFS             (0.0)
#define PLAT_CONFIG_BUCK_V_OUT_BUCK_GAIN        (3 * (5.83 / 0.6) / 4095.0)
#define PLAT_CONFIG_BUCK_V_OUT_BUCK_OFFS        (0.0)
#define PLAT_CONFIG_BUCK_IL_GAIN                (3 * (5.9 / 3.9) / 4095 / 50e-3)
#define PLAT_CONFIG_BUCK_IL_OFFS                (-(2.49 / 50e-3 + 1.4165))
#define PLAT_CONFIG_BUCK_IL_AVG_GAIN            (3 * (5.9 / 3.9) / 4095 / 50e-3)
#define PLAT_CONFIG_BUCK_IL_AVG_OFFS            (-(2.49 / 50e-3 + 0.8603))
#define PLAT_CONFIG_BUCK_IO_GAIN                (3 * (3.9 / 2.0) / 4095 / 50e-3)
#define PLAT_CONFIG_BUCK_IO_OFFS                (-(2.50 / 50e-3 + 2.999657404380524))
//=============================================================================

#endif /* PLAT_DEFS_H_ */
