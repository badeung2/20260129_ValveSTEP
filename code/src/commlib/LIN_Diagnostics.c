#include "AppBuild.h"
#include "drivelib/AppFunctions.h"
#include "drivelib/NV_Functions.h"
#include "drivelib/ErrorCodes.h"
#include "drivelib/GlobalVars.h"
#include "drivelib/MotorDriver.h"
#include "drivelib/Timer.h"
#include "camculib/private_mathlib.h"
#include "commlib/LIN_Communication.h"
#include <version.h>

extern void DfrDiagMlxDebug(DFR_DIAG *pDiag, uint16_t u16PCI_SID);
#pragma space dp
#pragma space none
#pragma space nodp
static uint16_t l_u16DiagResponseTimeoutCount = 0U;
uint8_t g_e8LoaderReset = (uint8_t)C_LOADER_CMD_NONE;
#pragma space none
static uint8_t CalcProtectionBits(uint8_t byFrameID)
{
	uint8_t u8FrameID = byFrameID;
	u8FrameID  |=
	(((u8FrameID & 0x01U) ^ ((u8FrameID & 0x02U)  >>  1) ^ ((u8FrameID & 0x04U)  >>  2) ^ ((u8FrameID & 0x10U)  >>  4))  !=
	0U) ? 0x40U : 0x00U;
	u8FrameID  |=
	((((u8FrameID & 0x02U)  >>
	1) ^ ((u8FrameID & 0x08U)  >>  3) ^ ((u8FrameID & 0x10U)  >>  4) ^ ((u8FrameID & 0x20U)  >>  5))  !=
	0U) ? 0x00U : 0x80U;
	return(u8FrameID);
}

void SetupDiagResponse(uint8_t u8NAD, uint8_t u8SID, uint8_t u8ResponseCode)
{
	g_DiagResponse.byNAD = u8NAD;
	if(u8ResponseCode  ==  (uint8_t)C_ERRCODE_POSITIVE_RESPONSE)
	{
		g_DiagResponse.byPCI = (uint8_t)C_RPCI_REASSIGN_NAD;
		g_DiagResponse.u.SF.byRSID = (uint8_t)(u8SID + (uint8_t)C_RSID_OK);
		g_DiagResponse.u.SF.byD1 = (uint8_t)C_DIAG_RES;
		g_DiagResponse.u.SF.byD2 = (uint8_t)C_DIAG_RES;
	}
	else
	{
		g_DiagResponse.byPCI = (uint8_t)C_RPCI_NOK;
		g_DiagResponse.u.SF.byRSID = (uint8_t)C_RSID_NOK;
		g_DiagResponse.u.SF.byD1 = u8SID;
		g_DiagResponse.u.SF.byD2 = u8ResponseCode;
	}
	g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;
}

static uint16_t CheckSupplier(uint16_t const u16SupplierID)
{
	uint16_t u16Result = FALSE;
	if((u16SupplierID  ==  (uint16_t)C_WILDCARD_SUPPLIER_ID)  ||  (u16SupplierID  ==  (uint16_t)C_SUPPLIER_ID)){u16Result = TRUE;}
	return(u16Result);
}

static uint16_t ValidSupplierFunctionID(uint16_t const u16SupplierID, uint16_t const u16FunctionID)
{
	uint16_t u16Result = FALSE;
	if(((u16SupplierID  ==  C_SUPPLIER_ID)  ||  (u16SupplierID  ==  C_WILDCARD_SUPPLIER_ID))  &&  ((u16FunctionID  ==  C_FUNCTION_ID)  ||  (u16FunctionID  ==  C_WILDCARD_FUNCTION_ID))){u16Result = TRUE;}
	return(u16Result);
}

static void DfrDiagReassignNAD(DFR_DIAG *pDiag)
{
	if(ValidSupplierFunctionID((pDiag  ->  u.SF.byD1) | ((uint16_t)(pDiag  ->  u.SF.byD2)  <<  8), (pDiag  ->  u.SF.byD3) | ((uint16_t)(pDiag  ->  u.SF.byD4)  <<  8))  !=  FALSE)
	{
		STD_LIN_PARAMS_t *pStdLin = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
		uint8_t byInitialNAD = pStdLin  ->  u8NAD;
		{
			STD_LIN_PARAMS_t StdLinRam;
			SetupDiagResponse(byInitialNAD, pDiag  ->  u.SF.bySID,
			(uint8_t)C_ERRCODE_PENDING);
			p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
			(uint16_t *)&StdLinRam,
			(const uint16_t *)ADDR_NV_STD_LIN_1);
			g_u8NAD = pDiag  ->  u.SF.byD5;
			StdLinRam.u8NAD = g_u8NAD;
			if((NV_WriteLIN_STD(&StdLinRam, C_NV_WRT_LIN_ID_ALL)  ==  C_ERR_NONE)  &&  (pStdLin  ->  u8NAD  ==  pDiag  ->  u.SF.byD5))
			{
				SetupDiagResponse(byInitialNAD, pDiag  ->  u.SF.bySID,
				(uint8_t)C_ERRCODE_POSITIVE_RESPONSE);
			}
			else
			{
				SetupDiagResponse(byInitialNAD, pDiag  ->  u.SF.bySID,
				(uint8_t)C_ERRCODE_SFUNC_NOSUP);
			}
		}
	}
}

static void DfrDiagAssignMessageToFrameID(DFR_DIAG *pDiag)
{
	if(CheckSupplier((pDiag  ->  u.SF.byD1) | ((uint16_t)(pDiag  ->  u.SF.byD2)  <<  8)))
	{
		uint16_t wMessageID = (((uint16_t)pDiag  ->  u.SF.byD4)  <<  8) | ((uint16_t)pDiag  ->  u.SF.byD3);
		STD_LIN_PARAMS_t *pStdLin = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
		STD_LIN_PARAMS_t StdLinRam;
		if(wMessageID  ==  MSG_CONTROL)
		{
			g_u8CtrlPID = pDiag  ->  u.SF.byD5;
			SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
			(uint8_t)C_ERRCODE_PENDING);
			p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
			(uint16_t *)&StdLinRam,
			(const uint16_t *)ADDR_NV_STD_LIN_1);
			StdLinRam.u8ControlFrameID = g_u8CtrlPID;
			(void)ml_Disconnect();
			(void)ml_AssignFrameToMessageID(MSG_CONTROL, g_u8CtrlPID);
#if (MSG_CONTROL <= 7)
			g_u16MLX4_RAM_Dynamic_CRC1 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR1,
			(C_RAM_MLX4_DYNAMIC_END_ADR1 - C_RAM_MLX4_DYNAMIC_BGN_ADR1)/sizeof(uint16_t));
#else
			g_u16MLX4_RAM_Dynamic_CRC2 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR2,
			(C_RAM_MLX4_DYNAMIC_END_ADR2 - C_RAM_MLX4_DYNAMIC_BGN_ADR2)/sizeof(uint16_t));
#endif
			(void)ml_Connect();
			if((NV_WriteLIN_STD(&StdLinRam, C_NV_WRT_LIN_ID_ALL)  ==  C_ERR_NONE)  &&  (pStdLin  ->  u8ControlFrameID  ==  pDiag  ->  u.SF.byD5))
			{
				SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
				(uint8_t)C_ERRCODE_POSITIVE_RESPONSE);
			}
			else
			{
				SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
				(uint8_t)C_ERRCODE_SFUNC_NOSUP);
			}
		}
		else if(wMessageID  ==  MSG_STATUS)
		{
			g_u8StsPID = pDiag  ->  u.SF.byD5;
			SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
			(uint8_t)C_ERRCODE_PENDING);
			p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
			(uint16_t *)&StdLinRam,
			(const uint16_t *)ADDR_NV_STD_LIN_1);
			StdLinRam.u8StatusFrameID = g_u8StsPID;
			(void)ml_Disconnect();
			(void)ml_AssignFrameToMessageID(MSG_STATUS, g_u8StsPID);
#if (MSG_STATUS <= 7)
			g_u16MLX4_RAM_Dynamic_CRC1 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR1,
			(C_RAM_MLX4_DYNAMIC_END_ADR1 - C_RAM_MLX4_DYNAMIC_BGN_ADR1)/sizeof(uint16_t));
#else
			g_u16MLX4_RAM_Dynamic_CRC2 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR2,
			(C_RAM_MLX4_DYNAMIC_END_ADR2 - C_RAM_MLX4_DYNAMIC_BGN_ADR2)/sizeof(uint16_t));
#endif
			(void)ml_Connect();
			if((NV_WriteLIN_STD(&StdLinRam, C_NV_WRT_LIN_ID_ALL)  ==  C_ERR_NONE)  &&  (pStdLin  ->  u8StatusFrameID  ==  pDiag  ->  u.SF.byD5))
			{
				SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
				(uint8_t)C_ERRCODE_POSITIVE_RESPONSE);
			}
			else
			{
				SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
				(uint8_t)C_ERRCODE_SFUNC_NOSUP);
			}
		}
		else
		{
			SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
			(uint8_t)C_ERRCODE_SFUNC_NOSUP);
		}
	}
	else
	{
		SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
		(uint8_t)C_ERRCODE_SFUNC_NOSUP);
	}
}

static void DfrDiagReadByIdentifier(DFR_DIAG *pDiag)
{
	if((pDiag  ->  u.SF.byD1  ==  0x33U)  &&  (pDiag  ->  u.SF.byD2  ==  0x13U)  &&  (pDiag  ->  u.SF.byD3  ==  0x00U)  &&  (pDiag  ->  u.SF.byD5  ==  0xCAU))
	{
		if((pDiag  ->  u.SF.byD4  ==  0xBCU)  ||  (pDiag  ->  u.SF.byD4  ==  0xFCU))
		{
			g_DiagResponse.byNAD = g_u8NAD;
			g_DiagResponse.byPCI = C_RPCI_READ_BY_ID_01;
			g_DiagResponse.u.SF.byRSID = C_RSID_READ_BY_ID;
			StoreD1to4( (uint16_t)PRODUCT_VERSION_32, (uint16_t)(PRODUCT_VERSION_32  >>  16));
		}
		else if((pDiag  ->  u.SF.byD4  ==  0xBDU)  ||  (pDiag  ->  u.SF.byD4  ==  0xFDU))
		{
			g_DiagResponse.byNAD = g_u8NAD;
			g_DiagResponse.byPCI = C_RPCI_READ_BY_ID_01;
			g_DiagResponse.u.SF.byRSID = C_RSID_READ_BY_ID;
			StoreD1to4( (uint16_t)PRODUCT_VERSION_32, (uint16_t)(PRODUCT_VERSION_32  >>  16));
			g_e8LoaderReset = (uint8_t)C_LOADER_CMD_EPM;
		}
		else{}
	}
	else if(ValidSupplierFunctionID((pDiag  ->  u.SF.byD2) | ((uint16_t)(pDiag  ->  u.SF.byD3)  <<  8), (pDiag  ->  u.SF.byD4) | ((uint16_t)(pDiag  ->  u.SF.byD5)  <<  8))  !=  FALSE)
	{
		if(pDiag  ->  u.SF.byD1  ==  C_LIN_PROD_ID)
		{
			g_DiagResponse.byNAD = g_u8NAD;
			g_DiagResponse.byPCI = C_RPCI_READ_BY_ID_00;
			g_DiagResponse.u.SF.byRSID = C_RSID_READ_BY_ID;
			g_DiagResponse.u.SF.byD5 = C_VARIANT_ID;
			StoreD1to4(C_SUPPLIER_ID, C_FUNCTION_ID);
		}
		else if(pDiag  ->  u.SF.byD1  ==  0x07U)
		{
			uint16_t u16CurrentBaudrate = p_ml_GetBaudRate(MLX4_FPLL);
			g_DiagResponse.u.SF.byD5 = 0x00U;
			StoreD1to4(0x4DBA, u16CurrentBaudrate);
		}
		else if(pDiag  ->  u.SF.byD1  ==  C_MSG_ID_1)
		{
			g_DiagResponse.byNAD = g_u8NAD;
			g_DiagResponse.byPCI = C_RPCI_READ_BY_ID_1X;
			g_DiagResponse.u.SF.byRSID = C_RSID_READ_BY_ID;
			g_DiagResponse.u.SF.byD3 = CalcProtectionBits(g_u8CtrlPID);
			StoreD1to2(MSG_CONTROL);
		}
		else if(pDiag  ->  u.SF.byD1  ==  C_MSG_ID_2)
		{
			g_DiagResponse.byNAD = g_u8NAD;
			g_DiagResponse.byPCI = C_RPCI_READ_BY_ID_1X;
			g_DiagResponse.u.SF.byRSID = C_RSID_READ_BY_ID;
			g_DiagResponse.u.SF.byD3 = CalcProtectionBits(g_u8StsPID);
			StoreD1to2(MSG_STATUS);
		}
		else if((pDiag  ->  u.SF.byD1  >=  C_MSG_ID_3)  &&  (pDiag  ->  u.SF.byD1  <=  C_MSG_ID_16))
		{
			SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
			(uint8_t)C_ERRCODE_SFUNC_NOSUP);
		}
		else if(pDiag  ->  u.SF.byD1  ==  (uint8_t)C_HVAC4x_VERIFY_NAD)
		{
			STD_LIN_PARAMS_t *pStdLin = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
			g_DiagResponse.byNAD = g_u8NAD;
			g_DiagResponse.byPCI = (uint8_t)C_RPCI_READ_BY_ID_21;
			g_DiagResponse.u.SF.byRSID = (uint8_t)C_RSID_READ_BY_ID;
			g_DiagResponse.u.SF.byD1 = (uint8_t)pStdLin  ->  u8NAD;
			g_DiagResponse.u.SF.byD2 = (uint8_t)(pStdLin  ->  u8ControlFrameID & 0x3FU);
			g_DiagResponse.u.SF.byD3 = (uint8_t)(pStdLin  ->  u8StatusFrameID & 0x3FU);
			g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;
		}
		else if(pDiag  ->  u.SF.byD1  ==  (uint8_t)C_HVAC4x_SW_HW_REF)
		{
			STD_LIN_PARAMS_t *pStdLin = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
			g_DiagResponse.byNAD = g_u8NAD;
			g_DiagResponse.byPCI = (uint8_t)C_RPCI_READ_BY_ID_2A;
			g_DiagResponse.u.SF.byRSID = (uint8_t)C_RSID_READ_BY_ID;
			g_DiagResponse.u.SF.byD1 = (uint8_t)C_SW_REF;
			g_DiagResponse.u.SF.byD2 = (uint8_t)pStdLin  ->  u8HardwareID;
			g_u8BufferOutID = (uint8_t)QR_RFR_DIAG;
		}
		else
		{
			SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
			(uint8_t)C_ERRCODE_SFUNC_NOSUP);
		}
	}
}

static void DfrDiagAssignVariantID(DFR_DIAG *pDiag)
{
	if(CheckSupplier((pDiag  ->  u.SF.byD1) | ((uint16_t)(pDiag  ->  u.SF.byD2)  <<  8))  !=  FALSE)
	{
		STD_LIN_PARAMS_t StdLinRam;
		uint16_t u16NvramStoreResult = FALSE;
		SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
		(uint8_t)C_ERRCODE_PENDING);
		p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
		(uint16_t *)&StdLinRam,
		(const uint16_t *)ADDR_NV_STD_LIN_1);
		if(pDiag  ->  u.SF.byD3  !=  0xFFU)
		{
			StdLinRam.u8Variant = pDiag  ->  u.SF.byD3;
			u16NvramStoreResult = TRUE;
		}
		if(pDiag  ->  u.SF.byD4  !=  0xFFU)
		{
			StdLinRam.u8HardwareID = pDiag  ->  u.SF.byD4;
			u16NvramStoreResult = TRUE;
		}
		if(pDiag  ->  u.SF.byD5  !=  0xFFU)
		{
			StdLinRam.u8SoftwareID = pDiag  ->  u.SF.byD5;
			u16NvramStoreResult = TRUE;
		}
		if(u16NvramStoreResult  !=  FALSE){u16NvramStoreResult = NV_WriteLIN_STD(&StdLinRam, C_NV_WRT_LIN_ID_ALL);}
		if(u16NvramStoreResult  ==  C_ERR_NONE)
		{
			SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
			(uint8_t)C_ERRCODE_POSITIVE_RESPONSE);
		}
		else
		{
			SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
			(uint8_t)C_ERRCODE_SFUNC_NOSUP);
		}
	}
}

static void DfrDiagAssignNAD(DFR_DIAG *pDiag)
{
	if((CheckSupplier((pDiag  ->  u.SF.byD1) | ((uint16_t)(pDiag  ->  u.SF.byD2)  <<  8))  !=  FALSE)  &&  (pDiag  ->  u.SF.byD4  ==  (uint8_t)C_SNPD_METHOD_BSM2))
	{
		if((pDiag  ->  u.SF.byD3  ==  (uint8_t)C_SNPD_SUBFUNC_START)  &&  (g_u8LinAAMode  ==  (uint8_t)C_SNPD_SUBFUNC_INACTIVE))
		{
			if((g_e8MotorStatus & C_MOTOR_STATUS_STOP_MASK)  !=  C_MOTOR_STATUS_STOP){MotorDriverStop( (uint16_t)C_STOP_WO_HOLDING);}
			g_u8ChipResetOcc = FALSE;
			g_u8StallOcc = FALSE;
			g_e8MotorDirectionCCW = (uint8_t)C_MOTOR_DIR_UNKNOWN;
			ml_SetSlaveNotAddressed();
			g_u8NAD = C_INVALID_NAD;
			g_u16LinAATicker = PI_TICKS_PER_SECOND;
			g_u8LinAATimeout = (uint8_t)C_LINAA_TIMEOUT;
			g_u8LinAAMode = (uint8_t)C_SNPD_SUBFUNC_START;
			ml_InitAutoAddressing();
		}
		else if((pDiag  ->  u.SF.byD3  ==  (uint8_t)C_SNPD_SUBFUNC_ADDR)  &&  (g_u8LinAAMode  ==  (uint8_t)C_SNPD_SUBFUNC_START))
		{
			if(ml_GetAutoaddressingStatus()  !=  FALSE)
			{
				{
					g_u8NAD = (pDiag  ->  u.SF.byD5);
					ml_SetSlaveAddressed();
				}
			}
		}
		else if((pDiag  ->  u.SF.byD3  ==  (uint8_t)C_SNPD_SUBFUNC_STORE)  &&  (g_u8LinAAMode  ==  (uint8_t)C_SNPD_SUBFUNC_START))
		{
			if(g_u8NAD  !=  C_INVALID_NAD)
			{
				STD_LIN_PARAMS_t StdLinRam;
				p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
				(uint16_t *)&StdLinRam,
				(const uint16_t *)ADDR_NV_STD_LIN_1);
				StdLinRam.u8NAD = g_u8NAD;
				(void)NV_WriteLIN_STD(&StdLinRam, C_NV_WRT_LIN_ID_ALL);
				g_u8LinAAMode = (uint8_t)C_SNPD_SUBFUNC_STORE;
			}
		}
		else if((pDiag  ->  u.SF.byD3  ==  (uint8_t)C_SNPD_SUBFUNC_FINISH)  &&  (g_u8LinAAMode  !=  (uint8_t)C_SNPD_SUBFUNC_INACTIVE))
		{
			ml_StopAutoAddressing();
			if(g_u8NAD  ==  C_INVALID_NAD)
			{
				STD_LIN_PARAMS_t *pStdLin = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
				g_u8NAD = pStdLin  ->  u8NAD;
			}
			g_u16LinAATicker = 0U;
			g_u8LinAAMode = (uint8_t)C_SNPD_SUBFUNC_INACTIVE;
		}
		else{}
	}
}

static void DfrDiagAssignFrameIdRange(DFR_DIAG *pDiag)
{
	STD_LIN_PARAMS_t *pStdLin = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
	SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
	(uint8_t)C_ERRCODE_PENDING);
	{
		uint16_t u16NvramStoreResult = FALSE;
		if(pDiag  ->  u.SF.byD1  ==  0U)
		{
			if(pDiag  ->  u.SF.byD2  !=  0xFFU)
			{
				g_u8CtrlPID = pDiag  ->  u.SF.byD2;
				(void)ml_Disconnect();
				if(g_u8CtrlPID  !=  0x00U){(void)ml_AssignFrameToMessageID(MSG_CONTROL, g_u8CtrlPID);}
				else{(void)ml_DisableMessage(MSG_CONTROL);}
				u16NvramStoreResult = TRUE;
			}
			if(pDiag  ->  u.SF.byD3  !=  0xFFU)
			{
				g_u8StsPID = pDiag  ->  u.SF.byD3;
				(void)ml_Disconnect();
				if(g_u8StsPID  !=  0x00U){(void)ml_AssignFrameToMessageID(MSG_STATUS, g_u8StsPID);}
				else{(void)ml_DisableMessage(MSG_STATUS);}
				u16NvramStoreResult = TRUE;
			}
		}
		else if(pDiag  ->  u.SF.byD1  ==  1)
		{
			if(pDiag  ->  u.SF.byD2  !=  0xFF)
			{
				g_u8StsPID = pDiag  ->  u.SF.byD2;
				(void)ml_Disconnect();
				if(g_u8StsPID  !=  0x00U){(void)ml_AssignFrameToMessageID(MSG_STATUS, g_u8StsPID);}
				else{(void)ml_DisableMessage(MSG_STATUS);}
				u16NvramStoreResult = TRUE;
			}
		}
		else{}
		if(u16NvramStoreResult  !=  FALSE)
		{
			STD_LIN_PARAMS_t StdLinRam;
#if (MSG_CONTROL <= 7) || (MSG_STATUS <= 7)
			g_u16MLX4_RAM_Dynamic_CRC1 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR1,
			(C_RAM_MLX4_DYNAMIC_END_ADR1 - C_RAM_MLX4_DYNAMIC_BGN_ADR1)/sizeof(uint16_t));
#endif
#if (MSG_CONTROL >= 8) || (MSG_STATUS >= 8)
			g_u16MLX4_RAM_Dynamic_CRC2 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR2,
			(C_RAM_MLX4_DYNAMIC_END_ADR2 - C_RAM_MLX4_DYNAMIC_BGN_ADR2)/sizeof(uint16_t));
#endif
			(void)ml_Connect();
			p_CopyU16( (sizeof(STD_LIN_PARAMS_t) / sizeof(uint16_t)),
			(uint16_t *)&StdLinRam,
			(const uint16_t *)ADDR_NV_STD_LIN_1);
			StdLinRam.u8ControlFrameID = g_u8CtrlPID;
			StdLinRam.u8StatusFrameID = g_u8StsPID;
			u16NvramStoreResult = NV_WriteLIN_STD(&StdLinRam, C_NV_WRT_LIN_ID_ALL);
		}
		if((u16NvramStoreResult  ==  C_ERR_NONE)  &&  (((pDiag  ->  u.SF.byD1  ==  0U)  &&  ((pDiag  ->  u.SF.byD2  ==  0xFFU)  ||  (pStdLin  ->  u8ControlFrameID  ==  pDiag  ->  u.SF.byD2))  &&  ((pDiag  ->  u.SF.byD3  ==  0xFFU)  ||  (pStdLin  ->  u8StatusFrameID  ==  pDiag  ->  u.SF.byD3)))  ||  ((pDiag  ->  u.SF.byD1  ==  1U)  &&  ((pDiag  ->  u.SF.byD2  ==  0xFFU)  ||  (pStdLin  ->  u8StatusFrameID  ==  pDiag  ->  u.SF.byD2)))))
		{
			SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
			(uint8_t)C_ERRCODE_POSITIVE_RESPONSE);
		}
		else
		{
			SetupDiagResponse(g_u8NAD, pDiag  ->  u.SF.bySID,
			(uint8_t)C_ERRCODE_SFUNC_NOSUP);
		}
	}
}

static void HandleStandardLIN(DFR_DIAG *pDiag, uint16_t u16PCI_SID)
{
	switch(u16PCI_SID)
	{
		case C_PCI_SID_REASSIGN_NAD:
		{
			DfrDiagReassignNAD(pDiag);
			break;
		}
		case C_PCI_SID_ASSIGN_FRAME_ID:
		{
			DfrDiagAssignMessageToFrameID(pDiag);
			break;
		}
		case C_PCI_SID_READ_BY_ID:
		{
			if(pDiag  ->  byNAD  ==  g_u8NAD){DfrDiagReadByIdentifier(pDiag);}
			break;
		}
		case C_PCI_SID_DATA_DUMP:
		{
			DfrDiagAssignVariantID(pDiag);
			break;
		}
		case C_PCI_SID_ASSIGN_NAD:
		{
			DfrDiagAssignNAD(pDiag);
			break;
		}
		case C_PCI_SID_ASSIGN_FRAME_ID_RNG:
		{
			DfrDiagAssignFrameIdRange(pDiag);
			break;
		}
		default:
		{
			break;
		}
	}
}

void HandleDfrDiag(void)
{
	DFR_DIAG *pDiag = &g_LinCmdFrameBuffer.Diag;
	l_u16DiagResponseTimeoutCount = PI_TICKS_PER_SECOND;
	if(pDiag  ->  byNAD  ==  0x00U)
	{
		g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_SLEEP;
		return;
	}
	if(pDiag  ->  byNAD  !=  0x7EU){g_u8BufferOutID = (uint8_t)QR_INVALID;}
	if((pDiag  ->  byNAD  ==  g_u8NAD)  ||  (pDiag  ->  byNAD  ==  C_BROADCAST_NAD))
	{
		uint16_t u16PCI_SID = (((uint16_t)pDiag  ->  byPCI)  <<  8) | ((uint16_t)pDiag  ->  u.SF.bySID);
		g_DiagResponse.u.SF.byD1 = (uint8_t)C_DIAG_RES;
		g_DiagResponse.u.SF.byD2 = (uint8_t)C_DIAG_RES;
		g_DiagResponse.u.SF.byD3 = (uint8_t)C_DIAG_RES;
		g_DiagResponse.u.SF.byD4 = (uint8_t)C_DIAG_RES;
		g_DiagResponse.u.SF.byD5 = (uint8_t)C_DIAG_RES;
		if(((u16PCI_SID & 0x00FF)  >=  C_SID_REASSIGN_NAD)  &&  ((u16PCI_SID & 0x00FF)  <=  C_SID_ASSIGN_FRAME_ID_RNG)){HandleStandardLIN(pDiag, u16PCI_SID);}
		if((u16PCI_SID & 0x00FF)  ==  (uint8_t)C_SID_MLX_DEBUG){DfrDiagMlxDebug(pDiag, u16PCI_SID);}
	}
}

void LinDiagResponseTimeoutCount(uint16_t u16Period)
{
	if(l_u16DiagResponseTimeoutCount  !=  0U)
	{
		if(l_u16DiagResponseTimeoutCount > u16Period){l_u16DiagResponseTimeoutCount  -=  u16Period;}
		else
		{
			l_u16DiagResponseTimeoutCount = 0U;
			if(g_u8BufferOutID  ==  QR_RFR_DIAG){g_u8BufferOutID = (uint8_t)QR_INVALID;}
		}
	}
}

void RfrDiagReset(void)
{
	if(g_u8NAD  !=  (uint8_t)C_BROADCAST_NAD)
	{
		g_DiagResponse.byNAD = g_u8NAD;
		g_DiagResponse.byPCI = 0x06U;
		g_DiagResponse.u.SF.byRSID = (uint8_t)C_RSID_MLX_DEBUG;
		g_DiagResponse.u.SF.byD5 = (uint8_t) C_VARIANT_ID;
		StoreD1to4(C_SUPPLIER_ID, C_FUNCTION_ID);
	}
}
