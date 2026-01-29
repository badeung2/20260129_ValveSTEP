#include "AppBuild.h"
#include "drivelib/GlobalVars.h"
#include "drivelib/MotorStall.h"

#pragma space dp
uint16_t g_u16MinSpeedRPM;
uint16_t g_u16StartupSpeedRPM;
uint16_t g_u16LowSpeedRPM;
uint16_t g_u16MaxSpeedRPM;
uint16_t g_u16MotorPolePairs;
uint8_t g_u8StallTypeComm = C_STALL_NOT_FOUND;
uint8_t g_u8MotorStatusSpeed = (uint8_t)C_MOTOR_SPEED_STOP;
uint8_t g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
volatile uint8_t g_e8ErrorElectric = (uint8_t)C_ERR_NONE;
volatile uint8_t g_e8ErrorVoltage = (uint8_t)C_ERR_VOLTAGE_IN_RANGE;
volatile uint8_t g_u8ChipResetOcc = TRUE;
volatile uint8_t g_u8StallOcc = FALSE;
volatile uint8_t g_e8EmergencyRunOcc = (uint8_t)C_SAFETY_RUN_NO;
volatile uint8_t g_e8ErrorOverTemperature = (uint8_t)C_ERR_OTEMP_NO;
uint8_t g_e8DegradedMotorRequest = (uint8_t)C_MOTOR_REQUEST_NONE;
#pragma space none
uint16_t g_u16ActualPosition;
uint16_t g_u16TargetPosition;
uint16_t g_u16ActualMotorSpeedRPM;
uint16_t g_u16TargetMotorSpeedRPM;
volatile uint8_t g_e8MotorStatus;
volatile uint8_t g_e8DegradeStatus;
uint8_t g_e8StallDetectorEna;
uint8_t g_e8MotorDirectionCCW;
uint8_t g_u8MotorHoldingCurrEna;
uint8_t g_u8MotorCtrlSpeed;
uint8_t g_e8MotorCtrlMode;
volatile uint32_t l_u32ActualPosition;
uint32_t g_u32TargetPosition;
#pragma space nodp
uint16_t g_u16MotorStartDelay = 0U;
uint16_t g_u16PorPollCounter = 0U;
uint8_t g_u8OverTemperatureCount = 0U;
uint8_t g_e8ErrorVoltageComm = (uint8_t)C_ERR_VOLTAGE_IN_RANGE;
uint8_t g_u8NewMotorDirectionCCW;
volatile uint8_t g_u8RewindFlags = 0U;
#if (I2C_COMM != FALSE) && (((LIN_COMM != FALSE) && (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE) && (C_I2C_SLAVE_SCK_IO == PIN_FUNC_LIN)) ||(PWM_COMM != FALSE))
uint8_t g_u8I2cConnected = C_COMM_DISCONNECTED;
#endif
#if (LIN_COMM != FALSE) && ((CAN_COMM != FALSE) || ((I2C_COMM != FALSE) && (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE) && (C_I2C_SLAVE_SCK_IO == PIN_FUNC_LIN)) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
uint8_t g_u8LinConnected = C_COMM_DISCONNECTED;
#endif
uint16_t g_u16MLX4_RAM_Dynamic_CRC1 = 0x0000U;
uint16_t g_u16MLX4_RAM_Dynamic_CRC2 = 0x0000U;
#pragma space none
