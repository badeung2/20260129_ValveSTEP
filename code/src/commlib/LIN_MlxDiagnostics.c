#include "AppBuild.h"
#include "../AppVersion.h"
#include "../ActADC.h"
#include "drivelib/AppFunctions.h"
#include "drivelib/NV_Functions.h"
#include "drivelib/ErrorCodes.h"
#include "drivelib/GlobalVars.h"
#include "drivelib/MotorDriver.h"
#include "drivelib/MotorStall.h"
#include "drivelib/PID_Control.h"
#include "camculib/private_mathlib.h"
#include "commlib/LIN_Communication.h"
#include "commlib/LIN_AutoAddressing.h"
#include <bist_inline_impl.h>
#include <bl_bist.h>
#include <bl_tools.h>
#include <fwversion.h>
#include <string.h>

#pragma space dp
#pragma space none
#pragma space nodp
static uint16_t l_u16IoValue = 0x0000U;
#pragma space none
const uint16_t au16AnaOutRegs[8] =
{
	0x201CU, 0x201EU, 0x2020U, 0x204AU, 0x204CU, 0x204EU, 0x28CCU, 0x28CEU
};
const uint16_t tMlxDbgSupport[16] =
{
	C_DBG_SUBFUNC_SUPPORT_0, C_DBG_SUBFUNC_SUPPORT_1, C_DBG_SUBFUNC_SUPPORT_2, C_DBG_SUBFUNC_SUPPORT_3,
	C_DBG_SUBFUNC_SUPPORT_4, C_DBG_SUBFUNC_SUPPORT_5, 0x0000U, 0x0000U,
	0x0000U, 0x0000U, C_DBG_SUBFUNC_SUPPORT_A, C_DBG_SUBFUNC_SUPPORT_B,
	C_DBG_SUBFUNC_SUPPORT_C, C_DBG_SUBFUNC_SUPPORT_D, C_DBG_SUBFUNC_SUPPORT_E, C_DBG_SUBFUNC_SUPPORT_F
};
void DfrDiagMlxDebug(DFR_DIAG *pDiag, uint16_t u16PCI_SID)
{
	(void)u16PCI_SID;
	{
		uint8_t u8FunctionID;
		uint16_t u16SupplierID;
		u16SupplierID = (((uint16_t)pDiag  ->  u.SF.byD2)  <<  8) | ((uint16_t)pDiag  ->  u.SF.byD1);
		u8FunctionID = (uint8_t)pDiag  ->  u.SF.byD5;
		if((u8FunctionID  >=  C_DBG_SUBFUNC_NV_WR_IDX0)  &&  (u8FunctionID  <=  C_DBG_SUBFUNC_NV_WR_IDX_MAX))
		{
			static uint16_t au16NvBlock[4];
			uint16_t u16Index = (pDiag  ->  u.SF.byD5 - C_DBG_SUBFUNC_NV_WR_IDX0);
			if((u16Index & 0x0002U)  ==  0x0000U)
			{
				au16NvBlock[0] = ((uint16_t)pDiag  ->  u.SF.byD1) | (((uint16_t)pDiag  ->  u.SF.byD2)  <<  8);
				au16NvBlock[1] = ((uint16_t)pDiag  ->  u.SF.byD3) | (((uint16_t)pDiag  ->  u.SF.byD4)  <<  8);
			}
			else
			{
				const uint16_t u16Address = (uint16_t)(ADDR_NV_USER + ((u16Index & 0xFFFCU)  <<  1));
				au16NvBlock[2] = ((uint16_t)pDiag  ->  u.SF.byD1) | (((uint16_t)pDiag  ->  u.SF.byD2)  <<  8);
				au16NvBlock[3] = ((uint16_t)pDiag  ->  u.SF.byD3) | (((uint16_t)pDiag  ->  u.SF.byD4)  <<  8);
				(void)NV_WriteBlock(u16Address, (uint16_t *)&au16NvBlock[0], 4U, FALSE);
			}
		}
		else if(u16SupplierID  ==  C_SUPPLIER_ID)
		{
			g_DiagResponse.byNAD = g_u8NAD;
			g_DiagResponse.byPCI = 0x06U;
			g_DiagResponse.u.SF.byRSID = (uint8_t)C_RSID_MLX_DEBUG;
			if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_SUPPORT)
			{
				uint16_t u16Index = (uint16_t)(pDiag  ->  u.SF.byD3 & 0x0FU);
				StoreD1to2(tMlxDbgSupport[u16Index]);
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_STALLDET)
			{
				LinDiag_MotorStall();
				g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_APPLSTATE)
			{
				g_DiagResponse.u.SF.byD1 = g_e8MotorStatus;
				{
					uint16_t u16CopyValue = g_u16ActualPosition;
					g_DiagResponse.u.SF.byD2 = (uint8_t)(u16CopyValue & 0xFFU);
					g_DiagResponse.u.SF.byD3 = (uint8_t)(u16CopyValue  >>  8);
				}
				g_DiagResponse.u.SF.byD4 = (g_e8MotorRequest & 0x0FU);
				{
					uint8_t u8D5 = ((g_e8ErrorVoltage & 0x03U)  <<  2);
					if((g_e8ErrorElectric & (uint8_t)C_ERR_PERMANENT)  !=  0U){u8D5  |=  0x02U;}
					else if((g_e8ErrorElectric & (uint8_t) ~C_ERR_PERMANENT)  !=  0U){u8D5  |=  0x01U;}
					if(g_e8ErrorOverTemperature  !=  (uint8_t)C_ERR_OTEMP_NO){u8D5  |=  0x10U;}
					if(g_e8EmergencyRunOcc  !=  (uint8_t)C_SAFETY_RUN_NO){u8D5  |=  0x20U;}
					if(g_u8StallOcc  !=  FALSE){u8D5  |=  0x40U;}
					if(g_u8ChipResetOcc  !=  FALSE){u8D5  |=  0x80U;}
					g_DiagResponse.u.SF.byD5 = u8D5;
				}
				g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_LIN_BAUDRATE)
			{
				g_DiagResponse.u.SF.byD1 = (uint8_t)(PLL_FREQ / 250000UL);
				StoreD2to5(0xFFFFU, p_ml_GetBaudRate(MLX4_FPLL));
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_RESTART_AUTO_BAUDRATE)
			{
				(void)ml_SetAutoBaudRateMode(ML_ABR_ON_FIRST_FRAME);
				Set_Mlx4ErrorState(C_MLX4_STATE_IMMEDIATE_RST);
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_SPEED)
			{
				{
					StoreD2to5(g_u16TargetMotorSpeedRPM, g_u16ActualMotorSpeedRPM);
				}
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_AMBJENV)
			{
				uint16_t u16Value = (uint16_t)(Get_AmbientTemperature() + C_TEMPOFF);
				g_DiagResponse.u.SF.byD1 = (uint8_t)(u16Value & 0xFFU);
				StoreD2to5(Get_MotorVoltage(), 0xFFFFU);
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_FOC_IV){StoreD2to5(l_i16StallThresholdLA, l_i16ActLoadAngleLPF);}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_SET_DRV_SR)
			{
				if((pDiag  ->  u.SF.byD3  !=  0xFFU)  ||  (pDiag  ->  u.SF.byD4  !=  0xFFU))
				{
					IO_TRIM2_DRV = (IO_TRIM2_DRV & ~M_TRIM2_DRV_TRIM_SLWRT) | (pDiag  ->  u.SF.byD3 & 0x0FU);
					IO_TRIM1_DRV = (IO_TRIM1_DRV & ~(M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK | M_TRIM1_DRV_TRIM_DRVSUP)) |
					(((uint16_t)pDiag  ->  u.SF.byD4)  <<  4) | ((pDiag  ->  u.SF.byD3 & 0x30U)  >>  4);
					IO_PORT_DRV_OUT = (IO_PORT_DRV_OUT & ~M_PORT_DRV_OUT_DRVMOD_OPTION) |
					(((uint16_t)(pDiag  ->  u.SF.byD3 & 0xC0U))  <<  (9 - 6));
				}
				g_DiagResponse.u.SF.byD1 = (uint8_t)(IO_PORT_DRV_OUT  >>  8);
				StoreD2to5(IO_TRIM1_DRV, IO_TRIM2_DRV);
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_SET_CPU_FREQ)
			{
				IO_TRIM_RCO32M = (IO_TRIM_RCO32M & ~M_TRIM_RCO32M_TR_RCO32M_IN) | (((uint16_t)pDiag  ->  u.SF.byD3)  <<  2);
				StoreD2to5(IO_TRIM_RCO32M, 0xFFFFU);
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_SET_CPU_SSCM)
			{
				if(pDiag  ->  u.SF.byD4  ==  0x00U){IO_PORT_SSCM_CONF = pDiag  ->  u.SF.byD3;}
				else{IO_PORT_STEP_CONF = (((uint16_t)pDiag  ->  u.SF.byD4)  <<  8) | pDiag  ->  u.SF.byD3;}
				StoreD2to5(IO_PORT_SSCM_CONF, IO_PORT_STEP_CONF);
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_SET_LIN_SR)
			{
				uint8_t u8LinSR = pDiag  ->  u.SF.byD3 & 0x07U;
				IO_TRIM_RCO1M = (IO_TRIM_RCO1M & ~M_TRIM_RCO1M_PRE_TR_LIN_SLEWRATE) | (((uint16_t)u8LinSR)  <<  8);
				StoreD2to5(IO_TRIM_RCO1M, 0xFFFFU);
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_SET_DRV_SSCM)
			{
				if(pDiag  ->  u.SF.byD4  ==  0x00U){IO_PORT_SSCM2_CONF = pDiag  ->  u.SF.byD3;}
				else{IO_PORT_STEP2_CONF = (((uint16_t)pDiag  ->  u.SF.byD4)  <<  8) | pDiag  ->  u.SF.byD3;}
				StoreD2to5(IO_PORT_SSCM2_CONF, IO_PORT_STEP2_CONF);
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_SET_LIN_TERM)
			{
				uint8_t u8LinTERM = pDiag  ->  u.SF.byD3 & 0x07U;
				IO_TRIM_RCO1M = (IO_TRIM_RCO1M & ~M_TRIM_RCO1M_PRE_TR_LIN_SLVTERM) | (((uint16_t)u8LinTERM)  <<  11);
				StoreD2to5(IO_TRIM_RCO1M, 0xFFFFU);
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_MLX16_CLK)
			{
				int8_t i8OffsetClock = CalibrationParams.i8APP_TRIM09_OffClock28;
				uint8_t u8GainClockHigh = CalibrationParams.u8APP_TRIM09_GainClock28HighT;
				uint8_t u8GainClockLow = CalibrationParams.u8APP_TRIM08_GainClock28LowT;
				uint16_t u16RC_Clock = 28000U + (int16_t)(p_MulI32_I16byI16(i8OffsetClock, 28000)  >>  11);
				int16_t i16ADC_Temp = (int16_t)(Get_RawTemperature() - Get_TempMidADC());
				int16_t i16Coef;
				if(i16ADC_Temp  <=  0){i16Coef = u8GainClockHigh;}
				else{i16Coef = u8GainClockLow;}
				i16Coef = (125 * i16Coef);
				u16RC_Clock  +=  (int16_t)(p_MulI32_I16byI16(i16ADC_Temp, i16Coef)  >>  14);
				StoreD1to2(u16RC_Clock);
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_CHIPID)
			{
				uint16_t u16Index = (uint16_t)(pDiag  ->  u.SF.byD3 & 0x02U);
				g_DiagResponse.u.SF.byD1 = (uint8_t)u16Index;
				{
					const uint16_t *pu16NvramData = ((uint16_t *)ADDR_NV_CHIPID) + u16Index;
					StoreD2to5(pu16NvramData[0], pu16NvramData[1]);
				}
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_SUPPORT_OPTIONS)
			{
				g_DiagResponse.u.SF.byD1 = (uint8_t)(C_DIAG_RES
				& ~(1U  <<  6)
				);
				g_DiagResponse.u.SF.byD2 = (uint8_t)(C_DIAG_RES
				& ~(1U  <<  0)
				);
				g_DiagResponse.u.SF.byD3 = (uint8_t)(C_DIAG_RES
				& ~(1U  <<  1)
				& ~(1U  <<  3)
				& ~(1U  <<  4)
				);
				g_DiagResponse.u.SF.byD4 = (uint8_t)(C_DIAG_RES
				& ~(1U  <<  0)
				);
				g_DiagResponse.u.SF.byD5 = (uint8_t)(C_DIAG_RES
				& ~(1U  <<  4)
				& ~(1U  <<  6)
				);
				g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_MLX4_VERSION)
			{
				uint16_t u16ValueR = p_GetExMem(C_ADDR_MLX4_FW_VERSION);
				uint16_t u16ValueF = p_GetExMem(C_ADDR_MLX4_LOADER_VERSION);
				StoreD1to4(u16ValueR, u16ValueF);
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_PLTF_VERSION)
			{
				uint32_t version = VERSION_getFwPltfVersion();
				StoreD1to4( (((version  >>  24) & 0xFF) | (((version  >>  16) & 0xFF)  <<  8)),
				(((version  >>  8) & 0xFF) | (((version  >>  0) & 0xFF)  <<  8)));
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_APP_VERSION)
			{
				if(pDiag  ->  u.SF.byD3  ==  0U){StoreD1to4( (__APP_VERSION_MAJOR__ | (__APP_VERSION_MINOR__  <<  8)), __APP_VERSION_REVISION__);}
				else if(pDiag  ->  u.SF.byD3  ==  1U){StoreD1to4(p_GetExMem(C_ADDR_APP_STRING), p_GetExMem(C_ADDR_APP_STRING + 2U));}
				else if(pDiag  ->  u.SF.byD3  ==  2U){StoreD1to4(p_GetExMem(C_ADDR_APP_STRING + 4U), p_GetExMem(C_ADDR_APP_STRING + 6U));}
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_MLXPAGE)
			{
				uint16_t u16Index = (uint16_t)(pDiag  ->  u.SF.byD3 & 0x3EU);
				g_DiagResponse.u.SF.byD1 = (uint8_t)u16Index;
				{
					const uint16_t *pu16NvramData = ((uint16_t *)ADDR_NV_MLX_CALIB) + u16Index;
					StoreD2to5(pu16NvramData[0], pu16NvramData[1]);
				}
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_MLXPID)
			{
				StorePID_Info();
				g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_CHIPENV)
			{
				uint16_t u16Value = (uint16_t)(Get_ChipTemperature() + C_TEMPOFF);
				g_DiagResponse.u.SF.byD1 = (uint8_t)(u16Value & 0xFFU);
				if(pDiag  ->  u.SF.byD3  ==  1U){StoreD2to5(Get_MotorCurrentMovAvgxN_mA(), Get_MotorVoltage());}
				else{StoreD2to5(Get_MotorCurrent_mA(), Get_SupplyVoltage());}
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_FUNC)
			{
				uint16_t u16FunctionID = (((uint16_t)pDiag  ->  u.SF.byD4)  <<  8) | ((uint16_t)pDiag  ->  u.SF.byD3);
				if(u16FunctionID  ==  C_DBG_DBGFUNC_RESET){AppReset();}
				else if(u16FunctionID  ==  C_DBG_DBGFUNC_ENTER_EPM)
				{
					g_u8LinInFrameBufState = (uint8_t)C_LIN_IN_FREE;
					MLX16_RESET_SIGNED(C_CHIP_STATE_PPM_CMD_EPM);
				}
				else if(u16FunctionID  ==  C_DBG_DBGFUNC_LOCK_PPM)
				{
					PATCH_HDR_t Patch;
					SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
					(uint8_t)C_ERRCODE_PENDING);
					Patch.u8CRC = 0x89U;
					Patch.u8Length = 0x04U;
					Patch.u16PatchProjID = PROJECT_ID;
					Patch.u16PatchAddress0 = 0x2C2BU;
					Patch.u16PatchInstruction0 = 0x0005U;
					if(NV_WritePatch(&Patch)  ==  C_ERR_NONE)
					{
						SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
						(uint8_t)C_ERRCODE_POSITIVE_RESPONSE);
					}
					else
					{
						SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
						(uint8_t)C_ERRCODE_COND_SEQ);
					}
				}
				else if(u16FunctionID  ==  C_DBG_DBGFUNC_UNLOCK_PPM)
				{
					PATCH_HDR_t Patch;
					SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
					(uint8_t)C_ERRCODE_PENDING);
					Patch.u8CRC = 0xFFU;
					Patch.u8Length = 0x00U;
					Patch.u16PatchProjID = 0x0000U;
					Patch.u16PatchAddress0 = 0x0000U;
					Patch.u16PatchInstruction0 = 0x0000U;
					if(NV_WritePatch(&Patch)  ==  C_ERR_NONE)
					{
						SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
						(uint8_t)C_ERRCODE_POSITIVE_RESPONSE);
					}
					else
					{
						SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
						(uint8_t)C_ERRCODE_COND_SEQ);
					}
				}
				else{}
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_VDDA_VDDD){LinDiag_VddaVddd();}
#if (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0)
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_VIO0HV)
			{
				uint16_t u16Value = Get_IO0HV();
				u16Value =
				(uint16_t)((p_MulU32_U16byU16(u16Value,
				Get_HighVoltGain()) + (C_VOLTGAIN_DIV / 2)) / C_VOLTGAIN_DIV);
				StoreD1to2(u16Value);
			}
#endif
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_ERRORCODES)
			{
				uint16_t u16Error1 = GetFirstError();
				uint16_t u16Error2 = 0U;
				if(((u16Error1 & C_ERR_EXTW)  ==  C_ERR_EXTW)  ||  ((PeakFirstError() & C_ERR_EXTW)  !=  C_ERR_EXTW)){u16Error2 = GetFirstError();}
				g_DiagResponse.u.SF.byD5 = 0x00U;
				StoreD1to4(u16Error1, u16Error2);
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_NV_READ)
			{
				if(pDiag  ->  u.SF.byD3  ==  0xFFU)
				{
					g_DiagResponse.u.SF.byD1 = (ADDR_NV_END - ADDR_NV_USER)  >>  1;
					g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;
				}
				else
				{
					uint16_t u16Index = pDiag  ->  u.SF.byD3;
					g_DiagResponse.u.SF.byD1 = (uint8_t)u16Index;
					{
						const uint16_t *pu16NvmData = ((uint16_t *)ADDR_NV_USER) + u16Index;
						StoreD2to5(pu16NvmData[0], pu16NvmData[1]);
					}
				}
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_FILLNVRAM)
			{
				uint8_t u8NvramID = pDiag  ->  u.SF.byD3;
				uint16_t u16Pattern = (((uint16_t)pDiag  ->  u.SF.byD4)  <<  8) | ((uint16_t)pDiag  ->  u.SF.byD4);
#define C_BUF_SZ 4
				uint16_t u16Buffer[C_BUF_SZ];
				u16Buffer[0] = u16Pattern;
				u16Buffer[1] = u16Pattern;
				u16Buffer[2] = u16Pattern;
				u16Buffer[3] = u16Pattern;
				if((u8NvramID & 0x80U)  !=  0x00)
				{
					(void)NV_WriteUserDefaults( (C_ERR_NV_LIN_STD_1 | C_ERR_NV_LIN_STD_2)
					| C_ERR_NV_EOL
					| C_ERR_NV_APP_PARAMS
					| C_ERR_NV_ACT_PARAMS
					| C_ERR_NV_ACT_STALL
					| C_ERR_NV_HDR_KEEP_COUNT);
				}
				else
				{
					if((u8NvramID & ID_NV_HDR)  !=  (uint8_t)0x00U){(void)NV_WriteBlock(ADDR_NV_HDR, (uint16_t *)u16Buffer, C_BUF_SZ, FALSE);}
					if((u8NvramID & ID_NV_STD_LIN)  !=  (uint8_t)0x00U)
					{
						(void)NV_WriteBlock(ADDR_NV_STD_LIN_1, (uint16_t *)u16Buffer, C_BUF_SZ, FALSE);
						(void)NV_WriteBlock(ADDR_NV_STD_LIN_2, (uint16_t *)u16Buffer, C_BUF_SZ, FALSE);
					}
					if((u8NvramID & ID_NV_EOL)  !=  (uint8_t)0x00U)
					{
						uint16_t u16Address = ADDR_NV_EOL;
						do
						{
							(void)NV_WriteBlock(u16Address, (uint16_t *)u16Buffer, C_BUF_SZ, FALSE);
							u16Address  +=  SZ_NV_BLOCK;
						}
						while(u16Address < (ADDR_NV_EOL + SZ_NV_EOL));
					}
					if((u8NvramID & ID_NV_ACT_PARAMS)  !=  (uint8_t)0x00U)
					{
						uint16_t u16Address = ADDR_NV_ACT_PARAMS;
						do
						{
							(void)NV_WriteBlock(u16Address, (uint16_t *)u16Buffer, C_BUF_SZ, FALSE);
							u16Address  +=  SZ_NV_BLOCK;
						}
						while(u16Address < (ADDR_NV_ACT_PARAMS + SZ_NV_ACT_PARAMS));
					}
					if((u8NvramID & ID_NV_ACT_STALL)  !=  (uint8_t)0x00U)
					{
						uint16_t u16Address = ADDR_NV_ACT_STALL;
						do
						{
							(void)NV_WriteBlock(u16Address, (uint16_t *)u16Buffer, C_BUF_SZ, FALSE);
							u16Address  +=  SZ_NV_BLOCK;
						}
						while(u16Address < (ADDR_NV_ACT_STALL + SZ_NV_ACT_STALL));
					}
				}
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_SET_IO_VALUE){l_u16IoValue = (((uint16_t)pDiag  ->  u.SF.byD4)  <<  8) | ((uint16_t)pDiag  ->  u.SF.byD3);}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_SET_IO_REG)
			{
				uint16_t u16IoAddress = (((uint16_t)pDiag  ->  u.SF.byD4)  <<  8) | ((uint16_t)(pDiag  ->  u.SF.byD3 & 0xFEU));
				if(u16IoAddress  <=  C_ADDR_IO_END){*((uint16_t *)u16IoAddress) = l_u16IoValue;}
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_GET_IO_REG)
			{
				uint16_t u16IoAddress = (((uint16_t)pDiag  ->  u.SF.byD4)  <<  8) | ((uint16_t)pDiag  ->  u.SF.byD3);
				if((u16IoAddress  <=  C_ADDR_IO_END)  ||  ((u16IoAddress  >=  C_ADDR_NV_BGN)  &&  (u16IoAddress  <=  C_ADDR_NV_END))  ||  ((u16IoAddress  >=  C_ADDR_RAM_BGN)  &&  (u16IoAddress  <=  C_ADDR_RAM_END))  ||  ((u16IoAddress  >=  C_ADDR_ROM_BGN)  &&  (u16IoAddress  <=  C_ADDR_ROM_END))  ||  ((u16IoAddress  >=  C_ADDR_FLASH_BGN)  &&  (u16IoAddress  <=  C_ADDR_FLASH_END))){StoreD1to4(u16IoAddress, *((uint16_t *)u16IoAddress));}
			}
			else if(u8FunctionID  ==  (uint8_t)C_DBG_SUBFUNC_FATAL_ERRORCODES){StoreD2to5(bistError, bistErrorInfo);}
			else{}
		}
		else
		{
			SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
			(uint8_t)C_ERRCODE_SFUNC_NOSUP);
		}
	}
}
