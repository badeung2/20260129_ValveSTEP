#include "AppBuild.h"
#include "../AppVersion.h"
#include "../ActADC.h"
#include "drivelib/ADC.h"
#include "drivelib/ErrorCodes.h"
#include "drivelib/NV_Functions.h"
#include "drivelib/NV_UserPage.h"
#include "drivelib/MotorDriverTables.h"
#include "camculib/private_mathlib.h"
#include "commlib/LIN_Communication.h"
#include "commlib/LIN_Diagnostics.h"
#include <bl_bist.h>
#include <builtin_mlx16.h>
#include <eeprom_drv.h>
#include <mem_checks.h>
#include <plib.h>
#include <atomic.h>
#define C_MAX_TEMP_WRITE_CNT_INCR_1 60
#define C_MAX_TEMP_WRITE_CNT_INCR_2 92
#define C_MAX_TEMP_WRITE_CNT_INCR_3 111
#define C_MAX_TEMP_WRITE_CNT_INCR_4 123
#define C_MAX_TEMP_WRITE_CNT_INCR_5 131
#define C_MAX_TEMP_WRITE_CNT_INCR_6 138
#define C_MAX_TEMP_WRITE_CNT_INCR_7 142
#define C_MAX_TEMP_WRITE_CNT_INCR_8 146
#define C_MAX_TEMP_WRITE_CNT_INCR_9 149
const int16_t ai16TemperatureScale[] =
{
	C_MAX_TEMP_WRITE_CNT_INCR_1,
	C_MAX_TEMP_WRITE_CNT_INCR_2,
	C_MAX_TEMP_WRITE_CNT_INCR_3,
	C_MAX_TEMP_WRITE_CNT_INCR_4,
	C_MAX_TEMP_WRITE_CNT_INCR_5,
	C_MAX_TEMP_WRITE_CNT_INCR_6,
	C_MAX_TEMP_WRITE_CNT_INCR_7,
	C_MAX_TEMP_WRITE_CNT_INCR_8,
	C_MAX_TEMP_WRITE_CNT_INCR_9
};
#define C_NV_REV 0x02U
#define C_HDR_STRUCT_ID (C_HEADER_PARAMS | C_STD_LIN_PARAMS | C_ENH_LIN_PARAMS | C_UDS_LIN_PARAMS | C_APP_EOL | C_APP_STORE | C_ACT_PARAMS | C_SENSOR_PARAMS | C_ACT_STALL | C_I2C_PARAMS | C_CAN_PARAMS )
NV_CRC eeCrc[] =
{
	{
		ADDR_NV_STD_LIN_1, (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)), C_ERR_NV_LIN_STD_1
	},
	{
		ADDR_NV_STD_LIN_2, (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)), C_ERR_NV_LIN_STD_2
	},
	{
		ADDR_NV_EOL, (sizeof(APP_EOL_t) / sizeof(uint16_t)), C_ERR_NV_EOL
	},
	{
		ADDR_NV_APP_PARAMS, (sizeof(APP_PARAMS_t) / sizeof(uint16_t)), C_ERR_NV_APP_PARAMS
	},
	{
		ADDR_NV_ACT_PARAMS, (sizeof(ACT_PARAMS_t) / sizeof(uint16_t)),C_ERR_NV_ACT_PARAMS
	},
	{
		ADDR_NV_ACT_STALL, (sizeof(ACT_STALL_t) / sizeof(uint16_t)), C_ERR_NV_ACT_STALL
	},
};
#define C_NV_DEF_HEADER { .u8CRC8 = 0x00U, .u8Revision = C_NV_REV, .u16ConfigurationID = CONFIGURATION_ID, .u12StructIDs = C_HDR_STRUCT_ID, .u2WriteCycleCountMSW = 0x0U, .u16WriteCycleCountLSW = 0x0001U }
#define C_NV_DEF_STD_LIN_PARAMS { .u8CRC8 = 0x00U, .u3LinUV = 0U, .u1BusTimeOutSleep = 1U, .u4Reserved = 0U, .u8NAD = 0x7FU, .u8ControlFrameID = mlxCONTROL, .u8StatusFrameID = mlxSTATUS, .u8Variant = C_VARIANT_ID, .u8HardwareID = 0xFFU, .u8SoftwareID = 0xFFU }
#define C_NV_DEF_EOL_PARAMS { .u8CRC8 = 0x00U, .u7EmergencyRunPos = 100U, .u1EmergencyRunPosEna = FALSE, .u1MotorDirectionCCW = C_MOTOR_ROTATION_CW, .u1StallDetectorEna = C_STALLDET_ENA, .u1RealTravelSaved = FALSE, .u1PorCalibration = FALSE, .u4Reserved_2 = 0x0U, .u8EndStopTime = C_ENDSTOP_TIME, .u16RealTravel = C_DEFAULT_TRAVEL, .u8TravelToleranceLo = C_TRAVEL_TOLERANCE_LO, .u8TravelToleranceUp = C_TRAVEL_TOLERANCE_UP }
#define C_NV_DEF_APP_PARAMS { .u8CRC8 = 0x00U, .u4PageID = 0x0U, .u4WrtCntH = 0x0U, .u16WrtCntL = 0x0000U, .u16ParamLSW = 0x0000U, .u16ParamMSW = 0x0000U }
#define C_NV_DEF_ACT_STALL { .u8CRC8 = 0x00U, .u7StallA_Threshold = C_STALL_A_THRESHOLD, .u1StallA_Ena = C_STALL_A_DET, .u4StallA_Width = C_STALL_A_WIDTH, .u4StallO_Width = C_STALL_O_WIDTH, .u7StallO_Threshold = C_STALL_O_THRESHOLD, .u1StallO_Ena = C_STALL_O_DET, .u7StallS_Threshold = C_STALL_S_THRESHOLD, .u1StallS_Ena = C_STALL_S_DET, .u4StallS_Width = C_STALL_S_WIDTH, .u1RestallPor = C_RESTALL_POR, .u1StallSpeedDepended = C_STALL_SPEED_DEPENDED, .u2Reserved = 0U, .u8RewindSteps = C_REWIND_STEPS, .u8StallDetectorDelay = (C_DETECTOR_DELAY / C_MICROSTEP_PER_FULLSTEP) }
static const HEADER_t DefHeader = C_NV_DEF_HEADER;
static const STD_LIN_PARAMS_t DefStdLinParams = C_NV_DEF_STD_LIN_PARAMS;
const APP_EOL_t DefEolParams = C_NV_DEF_EOL_PARAMS;
const ACT_PARAMS_t DefActParams =
{
	.u8CRC8 = 0x00U,
	.u8VsupRef = C_VSUP_REF,
	.u12GearBoxRatio = C_MOTOR_GEAR_BOX_RATIO,
	.u4PolePairs = (C_MOTOR_POLE_PAIRS - 1U),
	.u8MotorConstant = C_MOTOR_CONST_MV_PER_RPS,
	.u8MotorCoilRtot = ((C_TOT_COILS_R + 50U) / 100U),
	.u13MinSpeed = C_SPEED_MIN,
	.u3MicroSteps = C_MOTOR_MICROSTEPS,
	.u16Speed_1 = C_SPEED_0,
	.u16Speed_2 = C_SPEED_1,
	.u16Speed_3 = C_SPEED_2,
	.u16Speed_4 = C_SPEED_3,
	.u16AccelerationConst = C_ACCELERATION_CONST,
	.u3AccelerationSteps = C_ACCELERATION_STEPS,
	.u3DecelerationSteps = C_DECELERATION_STEPS,
	.u2MotorCurrentMultiplier = C_NV_CURR_DIV,
	.u8HoldingTorqueCurrent = C_PID_HOLDING_CURR_LEVEL,
	.u8RunningTorqueCurrent = (C_PID_RUNNING_CURR_LEVEL  >>  C_NV_CURR_DIV),
	.u4TorqueBoost1 = 0U,
	.u4TorqueBoost2 = 5U,
	.u8PidCoefP = C_PID_COEF_P,
	.u8PidCoefI = C_PID_COEF_I,
	.u8PidCoefD = C_PID_COEF_D,
	.u8PidStartupOrLowerHoldingLimit = C_MIN_HOLDCORR_RATIO,
	.u8PidLowerLimit = C_MIN_CORR_RATIO,
	.u8PidUpperLimit = C_MAX_CORR_RATIO,
	.u7PidCtrlPeriod = C_PID_RUNNINGCTRL_PERIOD,
	.u1PidPeriodTimeOrSpeed = C_PID_RUNNINGCTRL_PERIOD_UNIT,
	.u8AppOT = C_APP_OT_LEVEL + 60U,
	.u8AppUV = C_APP_UV_LEVEL,
	.u8AppOV = (C_APP_OV_LEVEL - (uint8_t)(C_APP_OV_OFF / 12.5)),
};
CONST ACT_STALL_t DefActStall = C_NV_DEF_ACT_STALL;
const APP_PARAMS_t DefAppParams = C_NV_DEF_APP_PARAMS;
#pragma space dp
#pragma space none
#pragma space nodp
uint16_t g_u16CP_FreqTrim_RT;
#pragma space none
static uint16_t NV_CalcCRC(const uint16_t *pu16BeginAddress, const uint16_t u16LengthW)
{
	uint16_t u16Result = p_CalcCRC_U8(pu16BeginAddress, u16LengthW);
	return(u16Result);
}

void p_CopyU16(const uint16_t u16SizeW, uint16_t *pu16Dest, const uint16_t *pu16Src)
{
	uint16_t *pu16Dest_Waste;
	uint16_t *pu16Src_Waste;
	uint16_t u16Size_Waste;
	__asm__ __volatile__
	(
	"mov c, ml.7\n\t"
	"movsw [X++], [Y++]\n\t"
	"add A, #0xFFFF\n\t"
	"jnz .-4"
	: "=x" (pu16Dest_Waste), "=y" (pu16Src_Waste), "=a" (u16Size_Waste)
	: "x" (pu16Dest), "y" (pu16Src), "a" (u16SizeW)
	:
	);
}

static void p_CopyU64(uint16_t *pu16Dest, const uint16_t *pu16Src)
{
	uint16_t *pu16Dest_Waste;
	uint16_t *pu16Src_Waste;
	__asm__ __volatile__
	(
	"mov c, ml.7\n\t"
	"movsw [X++], [Y++]\n\t"
	"movsw [X++], [Y++]\n\t"
	"movsw [X++], [Y++]\n\t"
	"movsw [X++], [Y++]"
	: "=x" (pu16Dest_Waste), "=y" (pu16Src_Waste)
	: "x" (pu16Dest), "y" (pu16Src)
	:
	);
}

static uint16_t p_CompareU64(const uint16_t *pu16Dest, const uint16_t *pu16Src)
{
	uint16_t u16Result = FALSE;
	if((*pu16Dest++  ==  *pu16Src++)  &&  (*pu16Dest++  ==  *pu16Src++)  &&  (*pu16Dest++  ==  *pu16Src++)  &&  (*pu16Dest++  ==  *pu16Src++)){u16Result = TRUE;}
	return(u16Result);
}

static void p_NV_BusyChecks(void)
{
	if((IO_EEPROM_FLASH_EE_CTRL_S & B_EEPROM_FLASH_EE_BUSY_STBY)  !=  0u){IO_EEPROM_FLASH_EE_CTRL_S  |=  B_EEPROM_FLASH_EE_ACTIVE;}
	else if((IO_EEPROM_FLASH_EE_CTRL_S & B_EEPROM_FLASH_EE_BUSY_BUF_NOT_EMPTY)  !=  0u){IO_EEPROM_FLASH_EE_CTRL_S = (IO_EEPROM_FLASH_EE_CTRL_S & ~M_EEPROM_FLASH_EE_WE_KEY) | 0U;}
	else{}
}

static void p_NV_WriteWord64_blocking(const uint16_t u16Address, uint16_t *pu64Data, const uint16_t u16WriteEnaKey)
{
	while(IO_GET(EEPROM_FLASH, EE_BUSY)  !=  0u){p_NV_BusyChecks();}
	IO_EEPROM_FLASH_EE_DATA_ERROR = (IO_EEPROM_FLASH_EE_DATA_ERROR |
	(B_EEPROM_FLASH_EE_DATA_CORRUPTED_2 |
	B_EEPROM_FLASH_EE_SBE_2 |
	B_EEPROM_FLASH_EE_DATA_CORRUPTED_1 |
	B_EEPROM_FLASH_EE_SBE_1));
	IO_EEPROM_FLASH_EE_CTRL_S = (IO_EEPROM_FLASH_EE_CTRL_S & ~(M_EEPROM_FLASH_EE_W_MODE | M_EEPROM_FLASH_EE_WE_KEY)) |
	(u16WriteEnaKey  <<  4);
	p_CopyU64( (uint16_t *)u16Address, (const uint16_t *)pu64Data);
	while(IO_GET(EEPROM_FLASH, EE_BUSY_WR)  !=  0u){}
}

uint16_t NV_WriteBlock(const uint16_t u16Address64, uint16_t *pu16Data, uint16_t u16SizeW, uint16_t u16Force)
{
	uint16_t u16Result = C_ERR_NONE;
	uint16_t *pBlock = (uint16_t *)pu16Data;
	uint16_t u16Address = u16Address64;
	do
	{
		if((u16Force  !=  FALSE)  ||  (p_CompareU64((uint16_t *)u16Address, (uint16_t *)pBlock)  ==  FALSE))
		{
			ENTER_SECTION(ATOMIC_SYSTEM_MODE);
			if(ADC_GetNewSampleVsupply() > C_MIN_VS_EEWRT){p_NV_WriteWord64_blocking(u16Address, pBlock, C_NV_WRT_KEY);}
			p_AwdAck();
			EXIT_SECTION();
			if(p_CompareU64((uint16_t *)u16Address, (uint16_t *)pBlock)  ==  FALSE){u16Result = C_ERR_NVWRITE;}
		}
		u16Address  +=  SZ_NV_BLOCK;
		pBlock  +=  (SZ_NV_BLOCK / sizeof(uint16_t));
	}
	while((pBlock < (uint16_t *)pu16Data + u16SizeW)  &&  (u16Result  ==  C_ERR_NONE));
	return(u16Result);
}

static uint16_t NV_BlockCheckCRC(void)
{
	uint16_t idx;
	uint16_t u16Result = 0U;
	for(idx = 0U; idx < (sizeof(eeCrc) / sizeof(eeCrc[0])); idx++)
	{
		IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_EE_SH_ECC;
		if((NV_CalcCRC((const uint16_t *)eeCrc[idx].u16Address, eeCrc[idx].u16Size)  !=  0xFFU)  ||  ((IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_EE_SH_ECC)  !=  0U)){u16Result  |=  eeCrc[idx].u16ErrorCode;}
	}
	return(u16Result);
}

static uint16_t NV_CheckStdLin(uint16_t u16Result)
{
	if((u16Result & (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2))  ==  0U)
	{
		if(p_CompareU64((const uint16_t *)ADDR_NV_STD_LIN_1, (const uint16_t *)ADDR_NV_STD_LIN_2)  ==  FALSE)
		{
			STD_LIN_PARAMS_t StdLin;
			SetLastError(C_ERR_INV_USERPAGE_2 | C_ERR_EXT | 0x0300U);
			p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
			(uint16_t *)&StdLin,
			(const uint16_t *)ADDR_NV_STD_LIN_1);
			if(NV_WriteLIN_STD(&StdLin, (C_NV_WRT_FORCE | C_NV_WRT_LIN_ID_2))  !=  C_ERR_NONE){u16Result  |=  C_ERR_NV_LIN_STD_2;}
		}
	}
	else if((u16Result & (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2))  ==  C_ERR_NV_LIN_STD_1)
	{
		STD_LIN_PARAMS_t StdLin;
		SetLastError(C_ERR_INV_USERPAGE_2 | C_ERR_EXT | 0x0100U);
		p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
		(uint16_t *)&StdLin,
		(const uint16_t *)ADDR_NV_STD_LIN_2);
		if(NV_WriteLIN_STD(&StdLin, (C_NV_WRT_FORCE | C_NV_WRT_LIN_ID_1))  ==  C_ERR_NONE){u16Result  &=  ~C_ERR_NV_LIN_STD_1;}
	}
	else if((u16Result & (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2))  ==  C_ERR_NV_LIN_STD_2)
	{
		STD_LIN_PARAMS_t StdLin;
		SetLastError(C_ERR_INV_USERPAGE_2 | C_ERR_EXT | 0x0200U);
		p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
		(uint16_t *)&StdLin,
		(const uint16_t *)ADDR_NV_STD_LIN_1);
		if(NV_WriteLIN_STD(&StdLin, (C_NV_WRT_FORCE | C_NV_WRT_LIN_ID_2))  ==  C_ERR_NONE){u16Result  &=  ~C_ERR_NV_LIN_STD_2;}
	}
	return(u16Result);
}

uint16_t NV_CheckCRC(void)
{
	uint16_t u16Result = C_ERR_NONE;
	NV_USER_MAP_t *pNV_User = (NV_USER_MAP_t *)ADDR_NV_USER;
	ENTER_SECTION(SYSTEM_MODE);
	IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_EE_SH_ECC;
	if((NV_CalcCRC((const uint16_t *)ADDR_NV_HDR, (sizeof(HEADER_t) / sizeof(uint16_t)))  !=  0xFFU)  ||  ((IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_EE_SH_ECC)  !=  0U)  ||  (pNV_User  ->  hdr.u8Revision  !=  C_NV_REV))
	{
		u16Result = (C_ERR_NV_HDR_RST_COUNT
		| (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2)
		| C_ERR_NV_EOL
		| C_ERR_NV_APP_PARAMS
		| C_ERR_NV_ACT_PARAMS
		| C_ERR_NV_ACT_STALL
		);
	}
	else if((pNV_User  ->  hdr.u16ConfigurationID  !=  CONFIGURATION_ID)  ||  (pNV_User  ->  hdr.u12StructIDs  !=  C_HDR_STRUCT_ID))
	{
		u16Result = (C_ERR_NV_HDR_KEEP_COUNT
		| (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2)
		| C_ERR_NV_EOL
		| C_ERR_NV_APP_PARAMS
		| C_ERR_NV_ACT_PARAMS
		| C_ERR_NV_ACT_STALL
		);
	}
	else
	{
		u16Result = NV_BlockCheckCRC();
		u16Result = NV_CheckStdLin(u16Result);
	}
	EXIT_SECTION();
	return(u16Result);
}

uint16_t NV_WriteLIN_STD(volatile STD_LIN_PARAMS_t *pLinData, uint16_t u16BlockID)
{
	STD_LIN_PARAMS_t StdLinDef;
	uint16_t u16Force = FALSE;
	uint16_t u16Result = C_ERR_NONE;
	if((u16BlockID & C_NV_WRT_FORCE)  !=  0x0000U){u16Force = TRUE;}
	if(pLinData  ==  NULL)
	{
		p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
		(uint16_t *)&StdLinDef,
		(const uint16_t *)&DefStdLinParams);
		pLinData = &StdLinDef;
	}
	pLinData  ->  u8CRC8 = 0x00U;
	pLinData  ->  u8CRC8 = 0xFFU -
	(uint8_t)NV_CalcCRC((const uint16_t *)pLinData,
	(sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)));
	if((u16BlockID & C_NV_WRT_LIN_ID_1)  !=  0x0000U)
	{
		u16Result = NV_WriteBlock(ADDR_NV_STD_LIN_1, (uint16_t *)pLinData,
		(sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
		u16Force);
	}
	if(((u16BlockID & C_NV_WRT_LIN_ID_2)  !=  0x0000U)  &&  (u16Result  ==  C_ERR_NONE))
	{
		u16Result = NV_WriteBlock(ADDR_NV_STD_LIN_2, (uint16_t *)pLinData,
		(sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
		u16Force);
	}
	return(u16Result);
}

uint16_t NV_WriteEOL(volatile APP_EOL_t* pEolData)
{
	APP_EOL_t EolDef;
	uint16_t u16Result = C_ERR_NONE;
	if(pEolData  ==  NULL)
	{
		p_CopyU16( (sizeof(APP_EOL_t) / sizeof(uint16_t)),
		(uint16_t *)&EolDef,
		(const uint16_t *)&DefEolParams);
		pEolData = &EolDef;
	}
	pEolData  ->  u8CRC8 = 0x00U;
	pEolData  ->  u8CRC8 = 0xFFU - (uint8_t)NV_CalcCRC((const uint16_t *)pEolData,
	(sizeof(APP_EOL_t) / sizeof(uint16_t)));
	u16Result = NV_WriteBlock(ADDR_NV_EOL, (uint16_t *)pEolData,
	(sizeof(APP_EOL_t) / sizeof(uint16_t)),
	FALSE);
	return(u16Result);
}

uint16_t NV_WriteAPP(volatile APP_PARAMS_t* pAppData)
{
	APP_PARAMS_t AppDef;
	uint16_t u16Result = C_ERR_NONE;
	if(pAppData  ==  NULL)
	{
		p_CopyU16( (sizeof(APP_PARAMS_t) / sizeof(uint16_t)),
		(uint16_t *)&AppDef,
		(const uint16_t *)&DefAppParams);
		pAppData = &AppDef;
	}
	pAppData  ->  u8CRC8 = 0x00U;
	pAppData  ->  u8CRC8 = 0xFFU - (uint8_t)NV_CalcCRC( (const uint16_t *)pAppData,
	(sizeof(APP_PARAMS_t) / sizeof(uint16_t)));
	u16Result = NV_WriteBlock(ADDR_NV_APP_PARAMS, (uint16_t *)pAppData,
	(sizeof(APP_PARAMS_t) / sizeof(uint16_t)), FALSE);
	return(u16Result);
}

uint16_t NV_WriteActParams(volatile ACT_PARAMS_t* pActData)
{
	ACT_PARAMS_t ActDef;
	uint16_t u16Result = C_ERR_NONE;
	if(pActData  ==  NULL)
	{
		p_CopyU16( (sizeof(ACT_PARAMS_t) / sizeof(uint16_t)),
		(uint16_t *)&ActDef,
		(const uint16_t *)&DefActParams);
		pActData = &ActDef;
	}
	pActData  ->  u8CRC8 = 0x00U;
	pActData  ->  u8CRC8 = 0xFFU - (uint8_t)NV_CalcCRC( (const uint16_t *)pActData,
	(sizeof(ACT_PARAMS_t) / sizeof(uint16_t)));
	u16Result = NV_WriteBlock(ADDR_NV_ACT_PARAMS, (uint16_t *)pActData,
	(sizeof(ACT_PARAMS_t) / sizeof(uint16_t)), FALSE);
	return(u16Result);
}

uint16_t NV_WriteActStall(volatile ACT_STALL_t* pActStall)
{
	ACT_STALL_t ActStallDef;
	uint16_t u16Result = C_ERR_NONE;
	if(pActStall  ==  NULL)
	{
		p_CopyU16( (sizeof(ACT_STALL_t) / sizeof(uint16_t)),
		(uint16_t *)&ActStallDef,
		(const uint16_t *)&DefActStall);
		pActStall = &ActStallDef;
	}
	pActStall  ->  u8CRC8 = 0x00U;
	pActStall  ->  u8CRC8 = 0xFFU -
	(uint8_t)NV_CalcCRC( (const uint16_t *)pActStall,
	(sizeof(ACT_STALL_t) / sizeof(uint16_t)));
	u16Result = NV_WriteBlock(ADDR_NV_ACT_STALL, (uint16_t *)pActStall,
	(sizeof(ACT_STALL_t) / sizeof(uint16_t)), FALSE);
	return(u16Result);
}

uint16_t NV_WritePatch(volatile PATCH_HDR_t* pPatch)
{
	uint16_t u16Result = C_ERR_NONE;
	uint16_t u16LengthW = ((pPatch  ->  u8Length + 3U) & 0x1CU);
	if(pPatch  ->  u8Length  !=  0x00U)
	{
		pPatch  ->  u8CRC = 0x00U;
		pPatch  ->  u8CRC = 0xFFU - (uint8_t)NV_CalcCRC( (const uint16_t *)pPatch,
		(uint16_t)pPatch  ->  u8Length);
	}
	else{u16LengthW = 0x0004U;}
	u16Result = NV_WriteBlock(ADDR_NV_START, (uint16_t *)pPatch,
	u16LengthW, FALSE);
	return(u16Result);
}

uint16_t NV_WriteUserDefaults(uint16_t u16ErrorIdMask)
{
	uint16_t u16Result = C_ERR_NONE;
	if((u16ErrorIdMask & (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2))  !=  0x0000U)
	{
		if((u16ErrorIdMask & (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2))  ==  (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2)){u16Result  |=  NV_WriteLIN_STD(NULL, C_NV_WRT_LIN_ID_ALL);}
		else{(void)NV_CheckStdLin(u16ErrorIdMask);}
	}
	if((u16ErrorIdMask & C_ERR_NV_EOL)  !=  0x0000U){u16Result  |=  NV_WriteEOL(NULL);}
	if((u16ErrorIdMask & C_ERR_NV_APP_PARAMS)  !=  0x0000U){u16Result  |=  NV_WriteAPP(NULL);}
	if((u16ErrorIdMask & C_ERR_NV_ACT_PARAMS)  !=  0x0000U){u16Result  |=  NV_WriteActParams(NULL);}
	if((u16ErrorIdMask & C_ERR_NV_ACT_STALL)  !=  0x0000U){u16Result  |=  NV_WriteActStall(NULL);}
	if((u16ErrorIdMask & (C_ERR_NV_HDR_KEEP_COUNT | C_ERR_NV_HDR_RST_COUNT))  !=  0x0000U)
	{
		HEADER_t hdr;
		p_CopyU16( (sizeof(HEADER_t) / sizeof(uint16_t)),
		(uint16_t *)&hdr,
		(const uint16_t *)&DefHeader);
		hdr.u12StructIDs = C_HDR_STRUCT_ID;
		if((u16ErrorIdMask & C_ERR_NV_HDR_KEEP_COUNT)  !=  0x0000U)
		{
			hdr.u16WriteCycleCountLSW = UserParams.hdr.u16WriteCycleCountLSW + 1U;
			if(hdr.u16WriteCycleCountLSW  !=  0U){hdr.u2WriteCycleCountMSW = UserParams.hdr.u2WriteCycleCountMSW;}
			else{hdr.u2WriteCycleCountMSW = UserParams.hdr.u2WriteCycleCountMSW + 1U;}
		}
		hdr.u8CRC8 = 0xFFU - (uint8_t)NV_CalcCRC( (const uint16_t *)&hdr,
		(sizeof(HEADER_t) / sizeof(uint16_t)));
		u16Result  |=  NV_WriteBlock(ADDR_NV_HDR, (uint16_t *)&hdr,
		(sizeof(HEADER_t) / sizeof(uint16_t)), TRUE);
	}
	return(u16Result);
}

uint16_t NV_MlxCalib(void)
{
	uint16_t u16ErrorResult;
	ENTER_SECTION(SYSTEM_MODE);
	IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_EE_SH_ECC;
	EXIT_SECTION();
	if((NV_CalcCRC((const uint16_t *)&CalibrationParams, (sizeof(MLX_CALIB_t) / sizeof(uint16_t)))  !=  0xFFU)  ||  ((IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_EE_SH_ECC)  !=  0U)){u16ErrorResult = TRUE;}
	else if(CalibrationParams.u8APP_TRIM03_CalibVersion < C_NV_MLX_VER_3){u16ErrorResult = TRUE;}
	else
	{
		uint16_t u16Trim;
		IO_TRIM1_DRV = CalibrationParams.u16APP_TRIM30_TRIM1_DRV;
		g_u16CP_FreqTrim_RT = (IO_TRIM1_DRV & M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK)  >>  2;
		IO_TRIM2_DRV = CalibrationParams.u16APP_TRIM31_TRIM2_DRV;
		u16Trim = CalibrationParams.u8APP_TRIM32_TRIM3_DRV_LOW;
		if((u16Trim & M_TRIM3_DRV_TRIM_CSA_CL)  ==  0x00U){u16Trim = 0x002EU;}
		IO_TRIM3_DRV = u16Trim;
		IO_TRIM_MISC = CalibrationParams.u16APP_TRIM33_TRIM_MISC;
		IO_PORT_SSCM2_CONF = B_PORT_SSCM2_CONF_SSCM2_CENTERED;
		IO_PORT_STEP2_CONF = (138U  <<  8) |
		(5U  <<  4) |
		(1U);
		u16ErrorResult = FALSE;
	}
	return(u16ErrorResult);
}

uint16_t NV_TemperatureBasedWriteCountIncrease(void)
{
	uint16_t u16Result;
	int16_t i16ChipTemperature = ADC_Conv_TempJ(FALSE);
	for(u16Result = 0; u16Result < (sizeof(ai16TemperatureScale) / sizeof(ai16TemperatureScale[0])); u16Result++)
	{
		if(i16ChipTemperature < ai16TemperatureScale[u16Result]){break;}
	}
	return(u16Result + 1U);
}

void NV_AppStore(uint16_t u16ParamLSW, uint16_t u16ParamMSW)
{
	uint16_t *pNV_AppData_Rd = (uint16_t *)ADDR_NV_APP_PARAMS;
	uint16_t *pNV_AppData_Wrt = (uint16_t *)ADDR_NV_APP_PARAMS;
	uint16_t *pNV_AppData_1 = (uint16_t *)ADDR_NV_APP_PARAMS;
	uint16_t *pNV_AppData_2 = (uint16_t *)ADDR_NV_APP_PARAMS_2;
	APP_PARAMS_t AppData;
	ENTER_SECTION(SYSTEM_MODE);
	IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_EE_SH_ECC;
	if((NV_CalcCRC((const uint16_t *)pNV_AppData_1, (sizeof(APP_PARAMS_t) / sizeof(uint16_t)))  ==  0xFFU)  &&  ((IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_EE_SH_ECC)  ==  0U))
	{
		if((NV_CalcCRC((const uint16_t *)pNV_AppData_2, (sizeof(APP_PARAMS_t) / sizeof(uint16_t)))  ==  0xFFU)  &&  ((IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_EE_SH_ECC)  ==  0U))
		{
			if((((APP_PARAMS_t *)pNV_AppData_1)  ->  u4PageID ^ ((APP_PARAMS_t *)pNV_AppData_2)  ->  u4PageID)  ==  0U){pNV_AppData_Rd = pNV_AppData_1;}
			else{pNV_AppData_Rd = pNV_AppData_2;}
			p_CopyU16( (sizeof(APP_PARAMS_t) / sizeof(uint16_t)),
			(uint16_t *)&AppData,
			(const uint16_t *)pNV_AppData_Rd);
			if((((APP_PARAMS_t *)pNV_AppData_1)  ->  u4WrtCntH < ((APP_PARAMS_t *)pNV_AppData_2)  ->  u4WrtCntH)  ||  ((((APP_PARAMS_t *)pNV_AppData_1)  ->  u4WrtCntH  ==  ((APP_PARAMS_t *)pNV_AppData_2)  ->  u4WrtCntH)  &&  (((APP_PARAMS_t *)pNV_AppData_1)  ->  u16WrtCntL  <=  ((APP_PARAMS_t *)pNV_AppData_2)  ->  u16WrtCntL)))
			{
				pNV_AppData_Wrt = pNV_AppData_1;
				AppData.u4PageID = ((APP_PARAMS_t *)pNV_AppData_2)  ->  u4PageID;
			}
			else
			{
				pNV_AppData_Wrt = pNV_AppData_2;
				AppData.u4PageID = ((APP_PARAMS_t *)pNV_AppData_1)  ->  u4PageID ^ 1U;
			}
			AppData.u16WrtCntL = ((APP_PARAMS_t *)pNV_AppData_Wrt)  ->  u16WrtCntL;
			AppData.u4WrtCntH = ((APP_PARAMS_t *)pNV_AppData_Wrt)  ->  u4WrtCntH;
		}
		else
		{
			pNV_AppData_Rd = pNV_AppData_1;
			pNV_AppData_Wrt = pNV_AppData_2;
			p_CopyU16( (sizeof(APP_PARAMS_t) / sizeof(uint16_t)),
			(uint16_t *)&AppData,
			(const uint16_t *)pNV_AppData_Rd);
			AppData.u4PageID = ((APP_PARAMS_t *)pNV_AppData_Rd)  ->  u4PageID ^ 1U;
		}
	}
	else
	{
		IO_MLX16_ITC_PEND0_S = B_MLX16_ITC_PEND0_EE_SH_ECC;
		if((NV_CalcCRC((const uint16_t *)pNV_AppData_2, (sizeof(APP_PARAMS_t) / sizeof(uint16_t)))  ==  0xFFU)  &&  ((IO_MLX16_ITC_PEND0_S & B_MLX16_ITC_PEND0_EE_SH_ECC)  ==  0U))
		{
			pNV_AppData_Rd = pNV_AppData_2;
			pNV_AppData_Wrt = pNV_AppData_1;
			p_CopyU16( (sizeof(APP_PARAMS_t) / sizeof(uint16_t)),
			(uint16_t *)&AppData,
			(const uint16_t *)pNV_AppData_Rd);
			AppData.u4PageID = ((APP_PARAMS_t *)pNV_AppData_Rd)  ->  u4PageID;
		}
		else
		{
			pNV_AppData_Rd = (uint16_t *)&DefAppParams;
			pNV_AppData_Wrt = pNV_AppData_1;
			p_CopyU16( (sizeof(APP_PARAMS_t) / sizeof(uint16_t)),
			(uint16_t *)&AppData,
			(const uint16_t *)pNV_AppData_Rd);
		}
	}
	EXIT_SECTION();
	{
		uint16_t u16WrtCntL = AppData.u16WrtCntL;
		AppData.u16WrtCntL = AppData.u16WrtCntL + NV_TemperatureBasedWriteCountIncrease();
		if(AppData.u16WrtCntL < u16WrtCntL){AppData.u4WrtCntH = AppData.u4WrtCntH + 1U;}
	}
	AppData.u16ParamLSW = u16ParamLSW;
	AppData.u16ParamMSW = u16ParamMSW;
	AppData.u8CRC8 = 0x00U;
	AppData.u8CRC8 = 0xFFU -
	(uint8_t)NV_CalcCRC( (const uint16_t *)&AppData,
	(sizeof(APP_PARAMS_t) / sizeof(uint16_t)));
	(void)NV_WriteBlock( (const uint16_t)pNV_AppData_Wrt, (uint16_t *)&AppData,
	(sizeof(APP_PARAMS_t) / sizeof(uint16_t)), FALSE);
}

__attribute__((interrupt)) void NV_COMPLETE_ISR(void)
{
	NOP();
}
