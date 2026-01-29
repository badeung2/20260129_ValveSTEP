#ifndef DRIVE_LIB_NV_USER_PAGE_H
#define DRIVE_LIB_NV_USER_PAGE_H
#include "AppBuild.h"
#define C_NV_USER_REV 0x01U
typedef struct
{
	uint16_t u8CRC8 : 8;
	uint16_t u8Revision : 8;
	uint16_t u16ConfigurationID;
	uint16_t u12StructIDs : 14;
#define C_HEADER_PARAMS (1U << 0)
#define C_STD_LIN_PARAMS (1U << 1)
#define C_ENH_LIN_PARAMS (0U << 2)
#define C_UDS_LIN_PARAMS (0U << 3)
#define C_APP_EOL (1U << 4)
#define C_APP_STORE (1U << 5)
#define C_ACT_PARAMS (1U << 6)
#define C_SENSOR_PARAMS (0U << 7)
#define C_ACT_STALL (1U << 8)
#define C_APP_ERROR_LOG (0U << 9)
#define C_I2C_PARAMS (0U << 10)
#define C_CAN_PARAMS (0U << 11)
#define C_RES1_PARAMS (0U << 12)
#define C_RES2_PARAMS (0U << 13)
	uint16_t u2WriteCycleCountMSW : 2;
	uint16_t u16WriteCycleCountLSW;
}

HEADER_t;
typedef struct
{
	uint16_t u8CRC8 : 8;
	uint16_t u3LinUV : 3;
	uint16_t u1BusTimeOutSleep : 1;
	uint16_t u4Reserved : 4;
	uint16_t u8NAD : 8;
	uint16_t u8ControlFrameID : 8;
	uint16_t u8StatusFrameID : 8;
	uint16_t u8Variant : 8;
	uint16_t u8HardwareID : 8;
	uint16_t u8SoftwareID : 8;
}

STD_LIN_PARAMS_t;
typedef struct
{
	uint16_t u8CRC8 : 8;
	uint16_t u7EmergencyRunPos : 7;
	uint16_t u1EmergencyRunPosEna : 1;
	uint16_t u1MotorDirectionCCW : 1;
#define C_MOTOR_ROTATION_CW 0U
#define C_MOTOR_ROTATION_CCW 1U
	uint16_t u1StallDetectorEna : 1;
#define C_STALLDET_DIS 0U
#define C_STALLDET_ENA 1U
	uint16_t u1RealTravelSaved : 1;
	uint16_t u1PorCalibration : 1;
	uint16_t u4Reserved_2 : 4;
	uint16_t u8EndStopTime : 8;
	uint16_t u16RealTravel;
	uint16_t u8TravelToleranceLo : 8;
	uint16_t u8TravelToleranceUp : 8;
}

APP_EOL_t;
typedef struct
{
	uint16_t u8CRC8 : 8;
	uint16_t u4PageID : 4;
	uint16_t u4WrtCntH : 4;
	uint16_t u16WrtCntL;
	uint16_t u16ParamLSW;
	uint16_t u16ParamMSW;
}

APP_PARAMS_t;
typedef struct
{
	uint16_t u8CRC8 : 8;
	uint16_t u8VsupRef : 8;
	uint16_t u12GearBoxRatio : 12;
	uint16_t u4PolePairs : 4;
	uint16_t u8MotorConstant : 8;
	uint16_t u8MotorCoilRtot : 8;
	uint16_t u13MinSpeed : 13;
	uint16_t u3MicroSteps : 3;
	uint16_t u16Speed_1;
	uint16_t u16Speed_2;
	uint16_t u16Speed_3;
	uint16_t u16Speed_4;
	uint16_t u16AccelerationConst;
	uint16_t u3AccelerationSteps : 3;
	uint16_t u3DecelerationSteps : 3;
	uint16_t u2MotorCurrentMultiplier : 2;
	uint16_t u8HoldingTorqueCurrent : 8;
	uint16_t u8RunningTorqueCurrent : 8;
	uint16_t u4TorqueBoost1 : 4;
	uint16_t u4TorqueBoost2 : 4;
	uint16_t u8PidCoefP : 8;
	uint16_t u8PidCoefI : 8;
	uint16_t u8PidCoefD : 8;
	uint16_t u8PidStartupOrLowerHoldingLimit : 8;
	uint16_t u8PidLowerLimit : 8;
	uint16_t u8PidUpperLimit : 8;
	uint16_t u7PidCtrlPeriod : 7;
	uint16_t u1PidPeriodTimeOrSpeed : 1;
	uint16_t u8AppOT : 8;
	uint16_t u8AppUV : 8;
	uint16_t u8AppOV : 8;
}

ACT_PARAMS_t;
typedef struct
{
	uint16_t u8CRC8 : 8;
	uint16_t u7StallA_Threshold : 7;
	uint16_t u1StallA_Ena : 1;
	uint16_t u4StallA_Width : 4;
	uint16_t u4StallO_Width : 4;
	uint16_t u7StallO_Threshold : 7;
	uint16_t u1StallO_Ena : 1;
	uint16_t u7StallS_Threshold : 7;
	uint16_t u1StallS_Ena : 1;
	uint16_t u4StallS_Width : 4;
	uint16_t u1RestallPor : 1;
	uint16_t u1StallSpeedDepended : 1;
	uint16_t u2Reserved : 2;
	uint16_t u8RewindSteps : 8;
	uint16_t u8StallDetectorDelay : 8;
}

ACT_STALL_t;
#define C_STALL_THRESHOLD_SDIV 7U
#define C_STALL_THRESHOLD_DIV (1 << C_STALL_THRESHOLD_SDIV)
typedef struct
{
	uint16_t u16ReservedA;
	uint16_t u16ReservedB;
	uint16_t u16ReservedC;
	uint16_t u8Address : 8;
	uint16_t u8CRC : 8;
}

BOOTLOADER_PARAMS_t;
typedef struct
{
	HEADER_t hdr;
	STD_LIN_PARAMS_t stdlin[2];
	APP_EOL_t eol;
	APP_PARAMS_t app;
	ACT_PARAMS_t act;
	ACT_STALL_t stall;
}

NV_USER_MAP_t;
extern volatile NV_USER_MAP_t UserParams __attribute__((nodp, addr(ADDR_NV_USER)));
#define SZ_NV_HDR sizeof(HEADER_t)
#define SZ_NV_STD_LIN sizeof(STD_LIN_PARAMS_t)
#define SZ_NV_EOL sizeof(APP_EOL_t)
#define SZ_NV_APP_PARAMS sizeof(APP_PARAMS_t)
#define SZ_NV_ACT_PARAMS sizeof(ACT_PARAMS_t)
#define SZ_NV_ACT_STALL sizeof(ACT_STALL_t)
#define ADDR_NV_USER_1 ((uint16_t)ADDR_NV_USER)
#define ADDR_NV_HDR ((uint16_t)ADDR_NV_USER_1)
#define ADDR_NV_USER_2 ((uint16_t)(ADDR_NV_HDR + SZ_NV_HDR))
#define ADDR_NV_STD_LIN_1 ((uint16_t)ADDR_NV_USER_2)
#define ADDR_NV_STD_LIN_2 ((uint16_t)(ADDR_NV_STD_LIN_1 + SZ_NV_STD_LIN))
#define ADDR_NV_USER_3 ((uint16_t)(ADDR_NV_STD_LIN_2 + SZ_NV_STD_LIN))
#define ADDR_NV_USER_4 ((uint16_t)ADDR_NV_USER_3)
#define ADDR_NV_USER_5 ((uint16_t)ADDR_NV_USER_4)
#define ADDR_NV_EOL ((uint16_t)ADDR_NV_USER_5)
#define ADDR_NV_USER_6 ((uint16_t)(ADDR_NV_EOL + SZ_NV_EOL))
#define ADDR_NV_APP_PARAMS ((uint16_t)ADDR_NV_USER_6)
#define ADDR_NV_USER_7 ((uint16_t)(ADDR_NV_APP_PARAMS + SZ_NV_APP_PARAMS))
#define ADDR_NV_ACT_PARAMS ((uint16_t)ADDR_NV_USER_7)
#define ADDR_NV_USER_8 ((uint16_t)(ADDR_NV_ACT_PARAMS + SZ_NV_ACT_PARAMS))
#define ADDR_NV_USER_9 ((uint16_t)ADDR_NV_USER_8)
#define ADDR_NV_ACT_STALL ((uint16_t)ADDR_NV_USER_9)
#define ADDR_NV_USER_10 ((uint16_t)(ADDR_NV_ACT_STALL + SZ_NV_ACT_STALL))
#define ADDR_NV_USER_11 ADDR_NV_USER_10
#define ADDR_NV_USER_12 ((uint16_t)ADDR_NV_USER_11)
#define ADDR_NV_USER_13 ((uint16_t)ADDR_NV_USER_12)
#define ADDR_NV_END ((uint16_t)ADDR_NV_USER_13)
#define ADDR_NV_APP_PARAMS_2 ((uint16_t)ADDR_NV_END)
#define ADDR_NV_BL ((uint16_t)0x09A8U)
#define C_MAX_NV_PROGRAM_COUNT 65000U
#define C_NV_STORE_OKAY 0x00U
#define C_NV_STORE_MAX_WRITE_CYCLE 0x01U
#define C_NV_STORE_INVALID_COUNTER 0x02U
#define C_NV_STORE_WRITE_FAILED 0x03U
#define NV_IC_UV_LEVEL ((uint16_t)C_IC_UV_LEVEL)
#define NV_IC_OV_LEVEL ((uint16_t)C_IC_OV_LEVEL)
#define NV_PWM_COMM_IN_ACTIVE_LEVEL 0U
#define NV_PWM_COMM_IN_FILTER_TYPE 1U
#define NV_PWM_COMM_IN_STABLE_WIDTH 5U
#define NV_PWM_COMM_IN_DC_STABLE ((uint16_t)(0.005 * 65536U))
#define NV_PWM_COMM_IN_DC_MIN ((uint16_t)(0.100 * 65536U))
#define NV_PWM_COMM_IN_DC_MAX ((uint16_t)(0.900 * 65536U))
#define NV_PWM_COMM_IN_DC_NV_WRT ((uint16_t)(0.050 * 65536U))
#define NV_PWM_COMM_IN_TIMEOUT (((10UL * C_PWM_COMM_IN_SAMPLE_FREQ) + 32768) / 65536U)
#define NV_PWM_COMM_IN_TO_ACTION 1U
#define NV_PID_HOLDINGCTRL_PER ((uint16_t)(C_PID_HOLDINGCTRL_PERIOD << 1))
#define NV_PID_THRSHLDCTRL_PER ((uint16_t)(C_PID_THRSHLDCTRL_PERIOD << 6))
#define NV_MOTOR_COIL_LTOT ((uint16_t)C_TOT_COILS_L)
#define NV_PID_RAMP_UP ((uint16_t)C_PID_RAMP_UP)
#define NV_PID_RAMP_DOWN ((uint16_t)C_PID_RAMP_DOWN)
#define NV_APPL_OTEMP ( (int16_t)UserParams.act.u8AppOT - 60)
#define NV_VSUP_REF ((uint16_t)(UserParams.act.u8VsupRef * 25U) >> 1)
#define NV_APPL_UVOLT ((uint16_t)(UserParams.act.u8AppUV * 25U) >> 1)
#define NV_APPL_OVOLT ((uint16_t)((UserParams.act.u8AppOV * 25U) >> 1) + C_APP_OV_OFF)
#define NV_GEARBOX_RATIO ((uint16_t)UserParams.act.u12GearBoxRatio)
#define NV_POLE_PAIRS ((uint16_t)UserParams.act.u4PolePairs + 1U)
#define NV_MICRO_STEPS ((uint16_t)(1U << UserParams.act.u3MicroSteps))
#define NV_ACT_SPEED1 ((uint16_t)UserParams.act.u16Speed_1)
#define NV_ACT_SPEED2 ((uint16_t)UserParams.act.u16Speed_2)
#define NV_ACT_SPEED3 ((uint16_t)UserParams.act.u16Speed_3)
#define NV_ACT_SPEED4 ((uint16_t)UserParams.act.u16Speed_4)
#define NV_MIN_SPEED ((uint16_t)UserParams.act.u13MinSpeed)
#define NV_ACCELERATION_CONST ((uint16_t)UserParams.act.u16AccelerationConst)
#define NV_ACCELERATION_PWR ((uint16_t)UserParams.act.u3AccelerationSteps)
#define NV_ACCELERATION_STEPS ((uint16_t)(1U << UserParams.act.u3AccelerationSteps))
#define NV_DECELERATION_PWR ((uint16_t)UserParams.act.u3DecelerationSteps)
#define NV_DECELERATION_STEPS ((uint16_t)(1U << UserParams.act.u3DecelerationSteps))
#define NV_MOTOR_CONSTANT ((uint16_t)UserParams.act.u8MotorConstant)
#define NV_MOTOR_COIL_RTOT ((uint16_t)UserParams.act.u8MotorCoilRtot)
#define NV_HOLDING_CURR_LEVEL ((uint16_t)UserParams.act.u8HoldingTorqueCurrent)
#define NV_MIN_HOLDCORR_RATIO ((uint16_t)(p_MulU32_U16byU16(UserParams.act.u8PidStartupOrLowerHoldingLimit, PWM_REG_PERIOD) >> (8U - C_PID_FACTOR)))
#define NV_STARTUP_CORR_RATIO ((uint16_t)(p_MulU32_U16byU16(UserParams.act.u8PidStartupOrLowerHoldingLimit, PWM_REG_PERIOD) >> (8U - C_PID_FACTOR)))
#define NV_RUNNING_CURR_LEVEL ((uint16_t)(UserParams.act.u8RunningTorqueCurrent << UserParams.act.u2MotorCurrentMultiplier))
#define NV_RUNNING_CURR_MAX ((uint16_t)(UserParams.act.u8RunningTorqueCurrent << UserParams.act.u2MotorCurrentMultiplier))
#define NV_PID_RUNNINGCTRL_PER ((uint16_t)UserParams.act.u7PidCtrlPeriod)
#define NV_PID_RUNNINGCTRL_PER_UNIT ((uint16_t)UserParams.act.u1PidPeriodTimeOrSpeed)
#define NV_PID_COEF_P ((uint16_t)UserParams.act.u8PidCoefP)
#define NV_PID_COEF_I ((uint16_t)UserParams.act.u8PidCoefI)
#define NV_PID_COEF_D ((uint16_t)UserParams.act.u8PidCoefD)
#define NV_MIN_CORR_RATIO ((uint16_t)(p_MulU32_U16byU16(UserParams.act.u8PidLowerLimit, PWM_REG_PERIOD) >> (8U - C_PID_FACTOR)))
#define NV_MAX_CORR_RATIO_PID ((uint16_t)(p_MulU32_U16byU16( (UserParams.act.u8PidUpperLimit + 1), PWM_REG_PERIOD) >> (8U - C_PID_FACTOR)))
#define NV_MAX_CORR_RATIO_PWM ((uint16_t)(p_MulU32_U16byU16( (UserParams.act.u8PidUpperLimit + 1), (uint16_t)(PWM_REG_PERIOD * 1.291)) >> (8U - C_PID_FACTOR)))
#define NV_BUSTIMEOUT_SLEEP ((uint16_t)UserParams.stdlin[0].u1BusTimeOutSleep)
#define NV_LIN_UV ((uint16_t)UserParams.stdlin[0].u3LinUV)
#define NV_AUTO_RECALIBRATE ((uint16_t)UserParams.eol.u1PorCalibration)
#define NV_STALL_DETECTOR_DELAY (((uint16_t)UserParams.stall.u8StallDetectorDelay) * C_MICROSTEP_PER_FULLSTEP)
#define NV_STALL_A ((uint16_t)UserParams.stall.u1StallA_Ena)
#define NV_STALL_A_WIDTH ((uint16_t)UserParams.stall.u4StallA_Width)
#define NV_STALL_A_THRSHLD ((uint16_t)UserParams.stall.u7StallA_Threshold)
#define NV_STALL_O ((uint16_t)UserParams.stall.u1StallO_Ena)
#define NV_STALL_O_THRSHLD ((uint16_t)UserParams.stall.u7StallO_Threshold)
#define NV_STALL_O_WIDTH ((uint16_t)UserParams.stall.u4StallO_Width)
#define NV_STALL_P ((uint16_t)UserParams.stall.u1StallO_Ena)
#define NV_STALL_P_POS_REVERSE ((uint16_t)UserParams.stall.u4StallO_Width)
#define NV_STALL_P_THRSHLD ((uint16_t)(UserParams.stall.u7StallO_Threshold << 3))
#define NV_STALL_S ((uint16_t)UserParams.stall.u1StallS_Ena)
#define NV_STALL_S_WIDTH ((uint16_t)UserParams.stall.u4StallS_Width)
#define NV_STALL_S_THRSHLD ((uint16_t)UserParams.stall.u7StallS_Threshold)
#define NV_STALL_SPEED_DEPENDED ((uint16_t)UserParams.stall.u1StallSpeedDepended)
#define NV_REWIND_STEPS ((uint16_t)UserParams.stall.u8RewindSteps)
#define NV_ROTATION_DIRECTION ((uint16_t)UserParams.eol.u1MotorDirectionCCW)
#define NV_REAL_TRAVEL ((uint16_t)UserParams.eol.u16RealTravel)
#define NV_CALIB_TRAVEL ((uint16_t)UserParams.eol.u16RealTravel)
#define NV_CALIB_ENDSTOP_TIME ((uint16_t)UserParams.eol.u8EndStopTime)
#define NV_CALIB_TOLERANCE_LOW ((uint16_t)UserParams.eol.u8TravelToleranceLo)
#define NV_CALIB_TOLERANCE_HIGH ((uint16_t)UserParams.eol.u8TravelToleranceUp)
#define NV_SAFETY_POSITION ((uint16_t)UserParams.eol.u7EmergencyRunPos)
#define NV_STALL_LA_THRSHLD ((uint16_t)C_STALL_LA_THRESHOLD_FOC)
#define NV_STALL_LA_WIDTH ((uint16_t)C_STALL_LA_WIDTH_FOC)
#define NV_STALL_LA_THRSHLD_OL ((uint16_t)C_STALL_LA_THRESHOLD_OL)
#define NV_STALL_LA_WIDTH_OL ((uint16_t)C_STALL_LA_WIDTH_OL)
#define NV_APP_DATA_MSW ((uint16_t)UserParams.app.u16ParamMSW)
#define NV_APP_DATA_LSW ((uint16_t)UserParams.app.u16ParamLSW)
#pragma space nodp
#pragma space none
#endif
