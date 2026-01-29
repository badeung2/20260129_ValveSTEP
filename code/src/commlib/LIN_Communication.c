#include "AppBuild.h"
#include "commlib/LIN_Communication.h"
#if ((_SUPPORT_SPI != FALSE) || (_SUPPORT_3WIRE_SPI != FALSE)) && (_SUPPORT_SPI_DMA != FALSE)
#include "commlib/SPI.h"
#endif
#include "drivelib/ADC.h"
#include "drivelib/AppFunctions.h"
#include "drivelib/NV_Functions.h"
#include "drivelib/ErrorCodes.h"
#include "drivelib/GlobalVars.h"
#include "drivelib/MotorDriver.h"
#include <itc_helper.h>
#include <bl_tools.h>
#include <bl_bist.h>

#pragma space dp
uint8_t g_u8BufferOutID = QR_INVALID;
uint8_t l_u8LinInFrameMsgID = 0xFFU;
LININBUF g_LinCmdFrameBuffer __attribute__((aligned(2)));
RFR_DIAG g_DiagResponse __attribute__((aligned(2)));
#pragma space none
#pragma space nodp
volatile uint8_t g_u8LinInFrameBufState = (uint8_t)C_LIN_IN_FREE;
volatile uint8_t g_u8ErrorCommunication = FALSE;
volatile uint8_t g_u8ErrorCommBusTimeout = FALSE;
uint8_t g_byCommEvent = C_COMM_EVENT_NONE;
#if ML_HAS_LIN_EVENT_TABLE_IN_RAM != 1
LINEventTable_t lin_table;
#endif
#pragma space none
extern void mlu_AutoAddressingStep(uint8_t StepNumber);
void __attribute__((interrupt)) COLIN_IT(void);
ml_Status_t mlu_ApplicationStop(void)
{
	AppStop();
	return(ML_SUCCESS);
}

void mlu_DataRequest(ml_MessageID_t MessageID)
{
	g_u8ErrorCommBusTimeout = FALSE;
	switch(MessageID)
	{
		case mlxRFR_DIAG:
		if((g_u8BufferOutID  ==  (uint8_t)QR_RFR_DIAG)  &&  (COLIN_LINstatus.buffer_used  ==  0U))
		{
			const RFR_DIAG *pDiag = &g_DiagResponse;
			const uint16_t *src = (uint16_t *)((void *)pDiag);
			uint16_t *dst = (uint16_t *)((void *)ML_DATA_LIN_FRAME_DATA_BUFFER);
			dst[0] = src[0];
			dst[1] = src[1];
			dst[2] = src[2];
			dst[3] = src[3];
			if(g_e8LoaderReset  !=  (uint8_t)C_LOADER_CMD_NONE)
			{
				if(ml_DataReady(ML_END_OF_TX_ENABLED)  !=  ML_SUCCESS)
				{
					g_u8ErrorCommunication = TRUE;
					SetLastError(C_ERR_LIN_API);
				}
			}
			else
			{
				if(ml_DataReady(ML_END_OF_TX_DISABLED)  !=  ML_SUCCESS)
				{
					g_u8ErrorCommunication = TRUE;
					SetLastError(C_ERR_LIN_API);
				}
			}
#if ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
			g_u8LinConnected = C_COMM_ACTIVE;
#endif
			if((g_DiagResponse.byPCI  ==  0x03U)  &&  (g_DiagResponse.u.SF.byRSID  ==  0x7FU)  &&  (g_DiagResponse.u.SF.byD2  ==  0x78U)){}
			else{g_u8BufferOutID = (uint8_t)QR_INVALID;}
		}
		else{(void)ml_DiscardFrame();}
		break;
		case MSG_STATUS:
		HandleActStatus();
#if ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
		g_u8LinConnected = C_COMM_ACTIVE;
#endif
		break;
		default:
		(void)ml_DiscardFrame();
	}
	ClearLinFrameTimeOut();
	ClearMlx4CheckPeriodCount();
}

void mlu_DataTransmitted(ml_MessageID_t Index)
{
	if(g_e8LoaderReset  !=  (uint8_t)C_LOADER_CMD_NONE)
	{
		uint16_t u16MiscOut = IO_PORT_MISC_OUT;
		IO_PORT_MISC_OUT = (u16MiscOut & ~B_PORT_MISC_OUT_SET_RSTAT) | B_PORT_MISC_OUT_CLEAR_RSTAT;
		IO_PORT_MISC_OUT = u16MiscOut;
		if(g_e8LoaderReset  ==  (uint8_t)C_LOADER_CMD_EPM)
		{
			g_e8LoaderReset = (uint8_t)C_LOADER_CMD_NONE;
			MLX16_RESET_SIGNED( (BistResetInfo_t)C_CHIP_STATE_CMD_EPM);
		}
		else
		{
			g_e8LoaderReset = (uint8_t)C_LOADER_CMD_NONE;
			MLX16_RESET_SIGNED( (BistResetInfo_t)C_CHIP_STATE_CMD_RESET);
		}
	}
	HandleDataTransmitted(Index);
}

void mlu_ErrorDetected(ml_LinError_t Error)
{
	g_e8LoaderReset = (uint8_t)C_LOADER_CMD_NONE;
	if(Error  ==  ml_erLinModuleReset)
	{
		uint8_t u8SubCode = (uint8_t)((ML_DATA_LIN_MESSAGE  >>  8U) & 0x000FU);
		if(u8SubCode  ==  (uint8_t)erCRASHTX)
		{
			(void)ml_Disconnect();
			(void)ml_Connect();
			Error = ml_erBit;
		}
	}
	HandleLinError(Error);
	ClearLinFrameTimeOut();
}

void mlu_LinSleepMode(ml_SleepReason_t Reason)
{
	if((Reason  ==  ml_reasonMaster)  ||  (Reason  ==  ml_reasonCommand)){g_e8MotorRequest = (uint8_t)C_MOTOR_REQUEST_SLEEP;}
	if((Reason  ==  ml_reasonTimeOut)  ||  (Reason  ==  ml_reasonTimeOutDominant)){HandleBusTimeout();}
}

void mlu_MessageReceived(ml_MessageID_t byMessageID)
{
	if(g_u8LinInFrameBufState  !=  (uint8_t)C_LIN_IN_FULL)
	{
		l_u8LinInFrameMsgID = byMessageID;
		{
			const uint16_t *pu16Source = (uint16_t *)((void *)ML_DATA_LIN_FRAME_DATA_BUFFER);
			uint16_t *pu16Target = (uint16_t *)&g_LinCmdFrameBuffer;
			pu16Target[0] = pu16Source[0];
			pu16Target[1] = pu16Source[1];
			pu16Target[2] = pu16Source[2];
			pu16Target[3] = pu16Source[3];
		}
		g_u8LinInFrameBufState = (uint8_t)C_LIN_IN_FULL;
		g_u8ErrorCommBusTimeout = FALSE;
#if ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
		g_u8LinConnected = C_COMM_ACTIVE;
#endif
		LinFrame[0] = 0x00U;
	}
	ClearLinFrameTimeOut();
	ClearMlx4CheckPeriodCount();
}

void HandleLinInMsg(void)
{
	if(g_u8LinInFrameBufState  ==  (uint8_t)C_LIN_IN_POSTPONE){g_u8LinInFrameBufState = (uint8_t)C_LIN_IN_FULL;}
	if(l_u8LinInFrameMsgID  ==  (uint8_t)mlxDFR_DIAG){HandleDfrDiag();}
	else if(l_u8LinInFrameMsgID  ==  (uint8_t)MSG_CONTROL){HandleActCtrl();}
	else{}
	if(g_u8LinInFrameBufState  !=  (uint8_t)C_LIN_IN_POSTPONE){g_u8LinInFrameBufState = (uint8_t)C_LIN_IN_FREE;}
}

void LIN_Init(void)
{
	plinEventTable  ->  mlu_MessageReceived = mlu_MessageReceived;
	plinEventTable  ->  mlu_DataRequest = mlu_DataRequest;
	plinEventTable  ->  mlu_ErrorDetected = mlu_ErrorDetected;
	plinEventTable  ->  mlu_DataTransmitted = mlu_DataTransmitted;
	plinEventTable  ->  mlu_LinSleepMode = mlu_LinSleepMode;
	plinEventTable  ->  mlu_AutoAddressingStep = mlu_AutoAddressingStep;
	p_MemSet((void *)&ml_Data, 0x00U, sizeof(ml_Data));
	*((uint8_t *) 0x0E01U) = 0x00U;
	*((uint8_t *) 0x0E02U) = 0x00U;
	*((uint8_t *) 0x0E03U) = 0x00U;
	ENTER_SECTION(SYSTEM_MODE);
	(void)ml_InitLinModule();
	EXIT_SECTION();
	(void)ml_SetAutoBaudRateMode(ML_ABR_ON_FIRST_FRAME);
	(void)ml_SetOptions(0U,
	0U,
	ML_ENABLED,
	ML_LIGHTSLEEP,
	ML_VER_2_X);
	(void)ml_SetSleepTo(1U, 13U, 7U);
#if ((LIN_BR < 12000) && (_SUPPORT_AUTO_BAUDRATE == FALSE))
	(void)ml_SetSlewRate(ML_SLEWLOW);
#else
	(void)ml_SetSlewRate(ML_SLEWHIGH);
#endif
	LIN_2x_Init();
	ENTER_SECTION(SYSTEM_MODE);
	IO_MLX16_ITC_PRIO2_S = (IO_MLX16_ITC_PRIO2_S & ~M_MLX16_ITC_PRIO2_COLIN_LIN) | C_MLX16_ITC_PRIO2_COLIN_LIN_PRIO5;
	IO_MLX16_ITC_MASK2_S  |=  B_MLX16_ITC_MASK2_COLIN_LIN;
	EXIT_SECTION();
	g_u16MLX4_RAM_Dynamic_CRC1 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR1,
	(C_RAM_MLX4_DYNAMIC_END_ADR1 - C_RAM_MLX4_DYNAMIC_BGN_ADR1)/sizeof(uint16_t));
	g_u16MLX4_RAM_Dynamic_CRC2 = p_CalcCRC_U16((const uint16_t *)C_RAM_MLX4_DYNAMIC_BGN_ADR2,
	(C_RAM_MLX4_DYNAMIC_END_ADR2 - C_RAM_MLX4_DYNAMIC_BGN_ADR2)/sizeof(uint16_t));
	(void)ml_Connect();
	g_u8ErrorCommBusTimeout = FALSE;
#if ((CAN_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
	g_u8LinConnected = C_COMM_CONNECTED;
#endif
}

#if ((CAN_COMM != FALSE) || ((I2C_COMM != FALSE) && (I2C_SLAVE_PROT != I2C_SLAVE_PROT_NONE) && (C_I2C_SLAVE_SCK_IO == PIN_FUNC_LIN)) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
void LIN_Stop(void)
{
	(void)ml_Disconnect();
	g_u8LinConnected = C_COMM_DISCONNECTED;
#if (_SUPPORT_APP_USER_MODE != FALSE)
	ENTER_SECTION(SYSTEM_MODE);
#endif
	ml_ResetDrv();
#if (_SUPPORT_APP_USER_MODE != FALSE)
	EXIT_SECTION();
#endif
}

#endif
void __attribute__((interrupt)) COLIN_IT(void);
void COLIN_IT(void)
{
	ml_GetLinEventData();
	ml_ProcessLinEvent();
}

uint16_t p_ml_GetBaudRate(uint16_t u16Mlx4Clk)
{
	uint16_t u16Result;
	uint16_t u16Rubbish;
	__asm__ __volatile__ (
	"cmp Y, #50000 \n\t"
	"jug p_GBR_Error_%= \n\t"
	"lod AL, dp:0x0C \n\t"
	"cmp AL, #99 \n\t"
	"juge p_GBR_00_%= \n\t"
	"p_GBR_Error_%=: \n\t"
	"mov A, #0 \n\t"
	"jmp p_GBR_Exit_%= \n\t"
	"p_GBR_00_%=: \n\t"
	"usex A \n\t"
	"mov X, A \n\t"
	"lod AL, dp:0x0D \n\t"
	"cmp AL, #8 \n\t"
	"jug p_GBR_Error_%= \n\t"
	"mov Cx, AL \n\t"
	"p_GBR_10_%=: \n\t"
	"asl X \n\t"
	"djnz Cx, p_GBR_10_%= \n\t"
	"p_GBR_20_%=: \n\t"
	"mov A, Y \n\t"
	"mulu YA, A, #500 \n\t"
	"cmp Y, X \n\t"
	"juge p_GBR_Error_%= \n\t"
	"divu YA, X \n\t"
	"divu YA, X \n\t"
	"p_GBR_Exit_%=:"
	: "=a" (u16Result), "=x" (u16Rubbish), "=y" (u16Rubbish)
	: "y" (u16Mlx4Clk)
	: "D"
	);
	return(u16Result);
}

uint16_t p_ml_GetLastBaudRate(uint16_t u16Mlx4Clk)
{
	uint16_t u16Result;
	uint16_t u16Rubbish;
	__asm__ __volatile__ (
	"cmp Y, #50000 \n\t"
	"jug p_GLBR_Error_%= \n\t"
	"lod AL, 0x0E01 \n\t"
	"cmp AL, #99 \n\t"
	"juge p_GLBR_00_%= \n\t"
	"p_GLBR_Error_%=: \n\t"
	"mov A, #0 \n\t"
	"jmp p_GLBR_Exit_%= \n\t"
	"p_GLBR_00_%=: \n\t"
	"usex A \n\t"
	"mov X, A \n\t"
	"lod AL, 0x0E03 \n\t"
	"and AL, #0x0F \n\t"
	"cmp AL, #8 \n\t"
	"jug p_GLBR_Error_%= \n\t"
	"mov Cx, AL \n\t"
	"p_GLBR_10_%=: \n\t"
	"asl X \n\t"
	"djnz Cx, p_GLBR_10_%= \n\t"
	"p_GLBR_20_%=: \n\t"
	"mov A, Y \n\t"
	"mulu YA, A, #500 \n\t"
	"cmp Y, X \n\t"
	"juge p_GLBR_Error_%= \n\t"
	"divu YA, X \n\t"
	"divu YA, X \n\t"
	"p_GLBR_Exit_%=:"
	: "=a" (u16Result), "=x" (u16Rubbish), "=y" (u16Rubbish)
	: "y" (u16Mlx4Clk)
	: "D"
	);
	return(u16Result);
}

uint16_t p_ml_GetAutoBaudRate(uint16_t u16Mlx4Clk)
{
	uint16_t u16Result;
	uint16_t u16Rubbish;
	__asm__ __volatile__ (
	"cmp Y, #50000 \n\t"
	"jug p_GABR_Error_%= \n\t"
	"lod AL, 0x0E02 \n\t"
	"cmp AL, #99 \n\t"
	"juge p_GABR_00_%= \n\t"
	"p_GABR_Error_%=: \n\t"
	"mov A, #0 \n\t"
	"jmp p_GABR_Exit_%= \n\t"
	"p_GABR_00_%=: \n\t"
	"usex A \n\t"
	"mov X, A \n\t"
	"lod AL, 0x0E03 \n\t"
	"lsr AL, #2 \n\t"
	"lsr AL, #2 \n\t"
	"cmp AL, #8 \n\t"
	"jug p_GABR_Error_%= \n\t"
	"mov Cx, AL \n\t"
	"p_GABR_10_%=: \n\t"
	"asl X \n\t"
	"djnz Cx, p_GABR_10_%= \n\t"
	"p_GABR_20_%=: \n\t"
	"mov A, Y \n\t"
	"mulu YA, A, #500 \n\t"
	"cmp Y, X \n\t"
	"juge p_GABR_Error_%= \n\t"
	"divu YA, X \n\t"
	"divu YA, X \n\t"
	"p_GABR_Exit_%=:"
	: "=a" (u16Result), "=x" (u16Rubbish), "=y" (u16Rubbish)
	: "y" (u16Mlx4Clk)
	: "D"
	);
	return(u16Result);
}
