#ifndef LIN_2X_HVAC_H
#define LIN_2X_HVAC_H
#include "AppBuild.h"
#include <mls_api.h>
#define C_DEFAULT_NAD 0x7FU
#define C_BROADCAST_NAD 0x7FU
#define C_LINAA_MODULE_NAD 0x5AU
#define C_DEFAULT_SNIFFER_NAD 0x5FU
#define C_WILDCARD_SUPPLIER_ID 0x7FFFU
#define C_MLX_SUPPLIER_ID ((('M' - '@') << 10) | (('L' - '@') << 5) | ('X' - '@'))
#define C_SUPPLIER_ID C_MLX_SUPPLIER_ID
#define C_WILDCARD_FUNCTION_ID 0xFFFFU
#define C_FUNCTION_ID ((('A' - '@') << 10) | (('C' - '@') << 5) | ('T' - '@'))
#define C_VARIANT_ID 0x01U
#define C_SW_REF 0x10U
#define C_HW_REF 0x10U
#define C_PROJECT_ID 0x71U
#define MSG_CONTROL 0x0001U
#define MSG_STATUS 0x0002U
#define C_MIN_POS 0x0000U
#define C_MAX_POS 0xFFFEU
#define C_INI_POS 0x7FFFU
#define C_INV_POS 0xFFFFU
#define C_LINAA_TIMEOUT 40U
#define mlxCONTROL 0xC1U
#define mlxSTATUS 0x42U
#define QR_INVALID 0xFFU
#define C_POSTYPE_NONE 0U
#define C_POSTYPE_INIT 1U
#define C_POSTYPE_TARGET 2U
typedef struct _HVAC_CTRL
{
	uint16_t u8NAD : 8;
	uint16_t u2Program : 2;
#define C_CTRL_PROGRAM_DIS 0U
#define C_CTRL_PROGRAM_ENA 1U
#define C_CTRL_PROGRAM_RES 2U
#define C_CTRL_PROGRAM_INV 3U
	uint16_t u2StallDetector : 2;
#define C_CTRL_STALLDET_DIS 0U
#define C_CTRL_STALLDET_ENA 1U
#define C_CTRL_STALLDET_RES 2U
#define C_CTRL_STALLDET_INV 3U
	uint16_t u4ClearEventFlags : 4;
#define C_CTRL_CLREVENT_NONE 0x0U
#define C_CTRL_CLREVENT_RESET 0x8U
#define C_CTRL_CLREVENT_STALL 0x4U
#define C_CTRL_CLREVENT_EMRUN 0x2U
#define C_CTRL_CLREVENT_RES 0x1U
#define C_CTRL_CLREVENT_INV 0xFU
	uint16_t u2HoldingCurrent : 2;
#define C_CTRL_MHOLDCUR_DIS 0U
#define C_CTRL_MHOLDCUR_ENA 1U
#define C_CTRL_MHOLDCUR_RES 2U
#define C_CTRL_MHOLDCUR_INV 3U
	uint16_t u2PositionType : 2;
#define C_CTRL_POSITION_TARGET 0U
#define C_CTRL_POSITION_INITIAL 1U
#define C_CTRL_POSITION_NONE 2U
#define C_CTRL_POSITION_INV 3U
	uint16_t u4Speed : 4;
#define C_CTRL_SPEED_RES 0U
#define C_CTRL_SPEED_1 1U
#define C_CTRL_SPEED_2 2U
#define C_CTRL_SPEED_3 3U
#define C_CTRL_SPEED_4 4U
#define C_CTRL_SPEED_AUTO 5U
#define C_CTRL_SPEED_INV 15U
	uint16_t u8TargetPositionLSB : 8;
	uint16_t u8TargetPositionMSB : 8;
	uint16_t u8StartPositionLSB : 8;
	uint16_t u8StartPositionMSB : 8;
	uint16_t u2EmergencyRun : 2;
#define C_CTRL_EMRUN_DIS 0U
#define C_CTRL_EMRUN_ENA 1U
#define C_CTRL_EMRUN_RES 2U
#define C_CTRL_EMRUN_INV 3U
	uint16_t u2EmergencyEndStop : 2;
#define C_CTRL_ENRUN_ENDSTOP_LO 0U
#define C_CTRL_ENRUN_ENDSTOP_HI 1U
#define C_CTRL_ENRUN_ENDSTOP_RES 2U
#define C_CTRL_ENRUN_ENDSTOP_INV 3U
	uint16_t u2RotationDirection : 2;
#define C_CTRL_DIR_CW 0U
#define C_CTRL_DIR_CCW 1U
#define C_CTRL_DIR_RES 2U
#define C_CTRL_DIR_INV 3U
	uint16_t u2StopMode : 2;
#define C_CTRL_STOPMODE_NORMAL 0U
#define C_CTRL_STOPMODE_STOP 1U
#define C_CTRL_STOPMODE_RES 2U
#define C_CTRL_STOPMODE_INV 3U
}

HVAC_CTRL;
typedef struct _HVAC_STATUS
{
	uint16_t u1ResponseError : 1;
#define C_STATUS_ERROR_NONE 0U
#define C_STATUS_ERROR 1U
	uint16_t u1Reserved1 : 1;
	uint16_t u2OverTemperature : 2;
#define C_STATUS_OTEMP_NO 0U
#define C_STATUS_OTEMP_YES 1U
#define C_STATUS_OTEMP_RES 2U
#define C_STATUS_OTEMP_INV 3U
	uint16_t u2ElectricDefect : 2;
#define C_STATUS_ELECDEFECT_NO 0U
#define C_STATUS_ELECDEFECT_YES 1U
#define C_STATUS_ELECDEFECT_PERM 2U
#define C_STATUS_ELECDEFECT_INV 3U
	uint16_t u2VoltageError : 2;
#define C_STATUS_VOLTAGE_OK 0U
#define C_STATUS_VOLTAGE_UNDER 1U
#define C_STATUS_VOLTAGE_OVER 2U
#define C_STATUS_VOLTAGE_INV 3U
	uint16_t u2EmergencyOccurred : 2;
#define C_STATUS_EMRUNOCC_NO 0U
#define C_STATUS_EMRUNOCC_YES 1U
#define C_STATUS_EMRUNOCC_RES 2U
#define C_STATUS_EMRUNOCC_INV 3U
	uint16_t u2StallDetector : 2;
#define C_STATUS_STALLDET_DIS 0U
#define C_STATUS_STALLDET_ENA 1U
#define C_STATUS_STALLDET_RES 2U
#define C_STATUS_STALLDET_INV 3U
	uint16_t u2StallOccurred : 2;
#define C_STATUS_STALLOCC_NO 0U
#define C_STATUS_STALLOCC_YES 1U
#define C_STATUS_STALLOCC_RES 2U
#define C_STATUS_STALLOCC_INV 3U
	uint16_t u2Reset : 2;
#define C_STATUS_RESETOCC_NO 0U
#define C_STATUS_RESETOCC_YES 1U
#define C_STATUS_RESETOCC_RES 2U
#define C_STATUS_RESETOCC_INV 3U
	uint16_t u2HoldingCurrent : 2;
#define C_STATUS_MHOLDCUR_DIS 0U
#define C_STATUS_MHOLDCUR_ENA 1U
#define C_STATUS_MHOLDCUR_RES 2U
#define C_STATUS_MHOLDCUR_INV 3U
	uint16_t u2PositionTypeStatus : 2;
#define C_STATUS_POSITION_ACTUAL 0U
#define C_STATUS_POSITION_INIT 1U
#define C_STATUS_POSITION_NONE 2U
#define C_STATUS_POSITION_INV 3U
	uint16_t u4SpeedStatus : 4;
#define C_STATUS_SPEED_STOP 0U
#define C_STATUS_SPEED_1 1U
#define C_STATUS_SPEED_2 2U
#define C_STATUS_SPEED_3 3U
#define C_STATUS_SPEED_4 4U
#define C_STATUS_SPEED_AUTO 5U
#define C_STATUS_SPEED_INV 15U
	uint16_t u8ActualPositionLSB : 8;
	uint16_t u8ActualPositionMSB : 8;
	uint16_t u2ActualRotationalDir : 2;
#define C_STATUS_ACT_DIR_CLOSING 0U
#define C_STATUS_ACT_DIR_OPENING 1U
#define C_STATUS_ACT_DIR_UNKNOWN 2U
#define C_STATUS_ACT_DIR_INV 3U
	uint16_t u2SelfHoldingTorque : 2;
#define C_STATUS_HOLDING_TORQUE_DIS 0U
#define C_STATUS_HOLDING_TORQUE_ENA 1U
#define C_STATUS_HOLDING_TORQUE_RES 2U
#define C_STATUS_HOLDING_TORQUE_INV 3U
	uint16_t u2SpecialFunctionActive : 2;
#define C_STATUS_SFUNC_ACTIVE_NO 0U
#define C_STATUS_SFUNC1_ACTIVE_YES 1U
#define C_STATUS_SFUNC_ACTIVE_RES 2U
#define C_STATUS_SFUNC_ACTIVE_INV 3U
	uint16_t u2Reserved : 2;
	uint16_t u8NAD : 8;
	uint16_t u2EmergencyRun : 2;
#define C_STATUS_EMRUN_DIS 0U
#define C_STATUS_EMRUN_ENA 1U
#define C_STATUS_EMRUN_RES 2U
#define C_STATUS_EMRUN_INV 3U
	uint16_t u2EmergencyRunEndStop : 2;
#define C_STATUS_EMRUN_ENDPOS_LO 0U
#define C_STATUS_EMRUN_ENDPOS_HI 1U
#define C_STATUS_EMRUN_ENDPOS_RES 2U
#define C_STATUS_EMRUN_ENDPOS_INV 3U
	uint16_t u2RotationDirection : 2;
#define C_STATUS_DIRECTION_CW 0U
#define C_STATUS_DIRECTION_CCW 1U
#define C_STATUS_DIRECTION_RES 2U
#define C_STATUS_DIRECTION_INV 3U
	uint16_t u2StopMode : 2;
#define C_STATUS_STOPMODE_NORMAL 0U
#define C_STATUS_STOPMODE_STOP 1U
#define C_STATUS_STOPMODE_RES 2U
#define C_STATUS_STOPMODE_DEGRADED 2U
#define C_STATUS_STOPMODE_INV 3U
}

HVAC_STATUS;
#pragma space dp
extern uint8_t g_u8NAD;
extern uint8_t g_u8CtrlPID;
extern uint8_t g_u8StsPID;
#pragma space none
#pragma space nodp
#pragma space none
extern void LIN_2x_Init(void);
extern void HandleActCtrl(void);
extern void HandleActStatus(void);
extern void HandleBusTimeout(void);
extern void HandleDataTransmitted(ml_MessageID_t Index);
extern void HandleLinError(ml_LinError_t Error);
#endif
