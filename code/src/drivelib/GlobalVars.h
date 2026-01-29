#ifndef DRIVE_LIB_GLOBAL_VARS_H
#define DRIVE_LIB_GLOBAL_VARS_H
#include "AppBuild.h"
#define PWM_PRESCALER_M 1U
#define PWM_PRESCALER_N 0U
#define PWM_PRESCALER (((PWM_PRESCALER_M - 1U) << 4) + PWM_PRESCALER_N)
#define PWM_TIMER_CLK (PLL_FREQ / ((PWM_PRESCALER_M * 1U) << PWM_PRESCALER_N))
#define PWM_REG_PERIOD (PWM_TIMER_CLK / PWM_FREQ)
#define PWM_SCALE_OFFSET ((uint16_t)((PWM_REG_PERIOD + 1U) / 2U))
#define PWM_OFFSET_25 (uint16_t)((PWM_REG_PERIOD + 2U) / 4U)
#define PWM_OFFSET_75 (uint16_t)(PWM_REG_PERIOD - PWM_OFFSET_25)
#define TIMER_PRESCALER 16U
#define TIMER_CLOCK ((uint32_t)(PLL_FREQ / TIMER_PRESCALER))
#define C_TMRx_CTRL_MODE0 (C_CTIMER0_DIV_CPU_DIV_16 | C_CTIMER0_MODE_TIMER | B_CTIMER0_ENCMP)
#define C_MIN_PWM_PERIOD (PWM_REG_PERIOD / TIMER_PRESCALER)
typedef enum __attribute__((packed))
{
	C_MOTOR_REQUEST_NONE = 0,
	C_MOTOR_REQUEST_STOP,
	C_MOTOR_REQUEST_INIT,
	C_MOTOR_REQUEST_START,
	C_MOTOR_REQUEST_START_wINIT,
	C_MOTOR_REQUEST_SLEEP,
	C_MOTOR_REQUEST_EMRUN,
	C_MOTOR_REQUEST_SPEED_CHANGE,
	C_MOTOR_REQUEST_RESTART,
	C_MOTOR_REQUEST_RESET,
	C_MOTOR_REQUEST_CALIB_FACTORY,
}

MOTOR_REQUEST;
#if ((LIN_COMM != FALSE) && ((CAN_COMM != FALSE) || (I2C_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))) || ((I2C_COMM != FALSE) && (PWM_COMM != FALSE))
typedef enum __attribute__((packed))
{
	C_COMM_DISCONNECTED = 0,
	C_COMM_CONNECTED,
	C_COMM_ACTIVE
}

COMM_MODE;
#endif
#define C_STALLDET_NONE 0x00U
#define C_STALLDET_A 0x01U
#define C_STALLDET_O 0x00U
#define C_STALLDET_P 0x00U
#define C_STALLDET_LA 0x08U
#define C_STALLDET_FLUX 0x00U
#define C_STALLDET_S 0x00U
#define C_STALLDET_B 0x00U
#define C_STALLDET_CDI 0x00U
#define C_STALLDET_H 0x00U
#define C_STALLDET_ALL (C_STALLDET_A | C_STALLDET_O | C_STALLDET_P | C_STALLDET_LA | C_STALLDET_FLUX | C_STALLDET_S | C_STALLDET_B | C_STALLDET_H)
#define C_STALLDET_CALIB 0x40U
#define C_MOTOR_SPEED_STOP 0U
#define C_MOTOR_SPEED_1 1U
#define C_MOTOR_SPEED_2 2U
#define C_MOTOR_SPEED_3 3U
#define C_MOTOR_SPEED_4 4U
#define C_MOTOR_SPEED_AUTO 5U
#define C_CALIB_MOTOR_SPEED C_MOTOR_SPEED_1
#define C_DEFAULT_MOTOR_SPEED C_MOTOR_SPEED_1
#define C_BUTTON_MOTOR_SPEED C_MOTOR_SPEED_2
#define C_MOTOR_CTRL_STOP 0x00U
#define C_MOTOR_CTRL_NORMAL 0x01U
#define C_MOTOR_CTRL_CALIBRATION 0x03U
typedef enum __attribute__((packed))
{
	C_MOTOR_STATUS_STOP = ((uint8_t) 0x00U),
	C_MOTOR_STATUS_HOLD = ((uint8_t) 0x01U),
	C_MOTOR_STATUS_WINDMILLING = ((uint8_t) 0x04U),
	C_MOTOR_STATUS_SRP = ((uint8_t) 0x05U),
	C_MOTOR_STATUS_RUNNING = ((uint8_t) 0x08U),
	C_MOTOR_STATUS_STOPPING = ((uint8_t) 0x0AU),
	C_MOTOR_STATUS_STOP_MASK = ((uint8_t) 0x0CU),
	C_MOTOR_STATUS_MASK = ((uint8_t) 0x0FU),
	C_MOTOR_STATUS_INIT = ((uint8_t) 0x10U),
	C_MOTOR_STATUS_SELFTEST = ((uint8_t) 0x20U),
	C_MOTOR_STATUS_APPL_STOP = ((uint8_t) 0x40U),
}

MOTOR_STATUS;
typedef enum __attribute__((packed))
{
	C_ERR_OTEMP_NO = ((uint8_t) 0x00U),
	C_ERR_OTEMP_WARN = ((uint8_t) 0x01U),
	C_ERR_OTEMP_SHUTDOWN = ((uint8_t) 0x02U),
	C_ERR_OTEMP_ERROR = ((uint8_t) 0x80U)
}

TEMPSENSOR_STATUS;
#define C_TEMPERATURE_HYS 3
#define C_TEMPERATURE_JUMP 25
#define C_CHIP_OVERTEMP_LEVEL 160
#define C_ERR_NONE 0x00U
#define C_ERR_PERMANENT 0x80U
#define C_ERR_SEMI_PERMANENT 0x40U
#define C_ERR_SUPPLY 0x20U
#define C_ERR_SUP_VDDA C_ERR_SUPPLY
#define C_ERR_SUP_VDDD (C_ERR_PERMANENT | C_ERR_SUPPLY)
#define C_ERR_SUP_VBGD C_ERR_SUPPLY
#define C_ERR_SUP_VIO C_ERR_SUPPLY
#define C_ERR_SUP_VDDAF C_ERR_SUPPLY
#define C_ERR_SUP_VBOOST C_ERR_SUPPLY
#define C_ERR_MEMORY 0x10U
#define C_ERR_MEM_NVM_CALIB (C_ERR_PERMANENT | C_ERR_MEMORY)
#define C_ERR_MEM_FLASH (C_ERR_PERMANENT | C_ERR_MEMORY)
#define C_ERR_MEM_COLIN_ROM (C_ERR_PERMANENT | C_ERR_MEMORY)
#define C_ERR_MEM_SYS_ROM (C_ERR_PERMANENT | C_ERR_MEMORY)
#define C_ERR_MEM_RAM (C_ERR_PERMANENT | C_ERR_MEMORY)
#define C_ERR_SENSOR (C_ERR_PERMANENT | 0x08U)
#define C_ERR_RPT_OVER_TEMPERATURE 0x04U
#define C_ERR_MOTOR_ZERO_CURRENT 0x02U
#define C_ERR_MOTOR_OVER_CURRENT 0x01U
#define C_ERR_MOTOR (C_ERR_MOTOR_ZERO_CURRENT | C_ERR_MOTOR_OVER_CURRENT)
#define C_ERR_VDS C_ERR_MOTOR_OVER_CURRENT
#define C_ERR_MOTOR_TEST_VPH (C_ERR_PERMANENT | C_ERR_MOTOR_OVER_CURRENT)
#define C_ERR_VOLTAGE_IN_RANGE 0U
#define C_ERR_VOLTAGE_UNDER 1U
#define C_ERR_VOLTAGE_OVER 2U
#define C_VOLTAGE_HYS ((uint16_t)50U)
#define C_SAFETY_RUN_NO 0U
#define C_SAFETY_RUN_ACTIVE 1U
#define C_SAFETY_RUN_DONE 2U
#define C_MOTOR_DIR_OPENING 0U
#define C_MOTOR_DIR_CW 0U
#define C_MOTOR_DIR_CLOSING 1U
#define C_MOTOR_DIR_CCW 1U
#define C_MOTOR_DIR_UNKNOWN 2U
#define C_DEBFLT_ERR_NONE 0x00U
#define C_DEBFLT_ERR_PHASE_SHORT 0x01U
#define C_DEBFLT_ERR_OVT 0x02U
#define C_DEBFLT_ERR_UV 0x04U
#define C_DEBFLT_ERR_OV 0x08U
#define C_DEBFLT_ERR_TEMP_PROFILE 0x10U
#define C_DEBFLT_ERR_TEMP_SENSOR 0x20U
#define C_DEBFLT_ERR_HALL_LATCH 0x40U
#define C_DEBFLT_ERR_GEARBOX 0x80U
#define C_SLEEP_TIMER_CONFIG (C_CTIMER0_DIV_CPU_DIV_256 | C_CTIMER0_MODE_TIMER | B_CTIMER0_ENCMP)
#define C_SLEEP_TIMER_PERIOD ((uint16_t)((PLL_FREQ / 256U) * 0.030))
#define C_DELAY_mPWM (uint16_t)(((1000000UL / C_DELAY_CONST) * FPLL) / PWM_FREQ)
#define C_REWIND_DIRECTION_CCW 0x01U
#define C_REWIND_STALL_DETECT 0x02U
#define C_REWIND_DIRECTION_ANY 0x04U
#define C_REWIND_ACTIVE 0x08U
#define C_REWIND_FULLAUTO 0x10U
#define C_REWIND_REWIND 0x20U
#define C_LIN_SLEEP 0U
#define C_LIN_AWAKE 1U
#define C_ERR_APP_OK 0U
#define C_ERR_APP_RST 1U
#define C_TEMP_STABIL_TIMEOUT C_PI_TICKS_250MS
#define C_TEMP_STABIL_INT_FILTER_COEF 8U
#define C_TEMP_STABIL_THRESHOLD 20U
#define C_ROTOR_TOLERANCE 0
#pragma space dp
extern uint16_t g_u16MinSpeedRPM;
extern uint16_t g_u16StartupSpeedRPM;
extern uint16_t g_u16LowSpeedRPM;
extern uint16_t g_u16MaxSpeedRPM;
extern uint16_t g_u16MotorPolePairs;
extern uint8_t g_u8StallTypeComm;
extern uint8_t g_u8MotorStatusSpeed;
extern uint8_t g_e8MotorRequest;
extern volatile uint8_t g_e8ErrorElectric;
extern volatile uint8_t g_e8ErrorVoltage;
extern volatile uint8_t g_u8ChipResetOcc;
extern volatile uint8_t g_u8StallOcc;
extern volatile uint8_t g_e8EmergencyRunOcc;
extern volatile uint8_t g_e8ErrorOverTemperature;
extern uint8_t g_e8DegradedMotorRequest;
#pragma space none
extern uint16_t g_u16ActualPosition __attribute__ ((dp, section(".dp.noinit")));
extern uint16_t g_u16TargetPosition __attribute__ ((dp, section(".dp.noinit")));
extern uint16_t g_u16ActualMotorSpeedRPM __attribute__ ((dp, section(".dp.noinit")));
extern uint16_t g_u16TargetMotorSpeedRPM __attribute__ ((dp, section(".dp.noinit")));
extern volatile uint8_t g_e8MotorStatus __attribute__ ((dp, section(".dp.noinit")));
extern volatile uint8_t g_e8DegradeStatus __attribute__ ((dp, section(".dp.noinit")));
extern uint8_t g_e8StallDetectorEna __attribute__ ((dp, section(".dp.noinit")));
extern uint8_t g_u8MotorHoldingCurrEna __attribute__ ((dp, section(".dp.noinit")));
extern uint8_t g_u8MotorCtrlSpeed __attribute__ ((dp, section(".dp.noinit")));
extern uint8_t g_e8MotorDirectionCCW __attribute__ ((dp, section(".dp.noinit")));
extern uint8_t g_e8MotorCtrlMode __attribute__ ((dp, section(".dp.noinit")));
extern volatile uint32_t l_u32ActualPosition __attribute__ ((dp, section(".dp.noinit")));
extern uint32_t g_u32TargetPosition __attribute__ ((dp, section(".dp.noinit")));
#pragma space nodp
extern uint16_t g_u16MotorStartDelay;
#if ((GPIO_COMM != FALSE) && ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_TERMINAL)))
extern uint8_t g_u8GpioConnected;
#endif
extern uint8_t g_u8OverTemperatureCount;
extern uint8_t g_e8ErrorVoltageComm;
extern uint8_t g_u8NewMotorDirectionCCW;
extern uint8_t g_u8RamTest;
extern volatile uint8_t g_u8RewindFlags;
#if (I2C_COMM != FALSE) && (((LIN_COMM != FALSE) && (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE) && (C_I2C_SLAVE_SCK_IO == PIN_FUNC_LIN)) || (PWM_COMM != FALSE))
extern uint8_t g_u8I2cConnected;
#endif
#if (LIN_COMM != FALSE) && ((CAN_COMM != FALSE) || ((I2C_COMM != FALSE) && (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE) && (C_I2C_SLAVE_SCK_IO == PIN_FUNC_LIN)) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
extern uint8_t g_u8LinConnected;
#endif
extern uint16_t g_u16MLX4_RAM_Dynamic_CRC1;
extern uint16_t g_u16MLX4_RAM_Dynamic_CRC2;
#pragma space none
#endif
