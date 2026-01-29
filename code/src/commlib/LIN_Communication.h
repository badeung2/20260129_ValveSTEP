#ifndef LIN_COMMUNICATION_H
#define LIN_COMMUNICATION_H
#include "AppBuild.h"
#include "commlib/LIN_Diagnostics.h"
#include "commlib/LIN_2x_HVAC.h"
#include "commlib/LIN_AutoAddressing.h"
#include <fwversion.h>
#include <lib_miscio.h>
#include <mls_api.h>
#include <mls_support.h>

extern ml_Data_t ml_Data __attribute__ ((dp, section(".lin_ram")));
#define LIN_BR 19200U
#define LIN_BR_PRESCALER 2
#define LIN_BR_DIV ((((1000UL * MLX4_FPLL) / (1U << (1U + LIN_BR_PRESCALER))) + (LIN_BR >> 1)) / LIN_BR)
#if (LIN_BR_DIV < 99) || (LIN_BR_DIV > 200)
#error "ERROR: Wrong LinBaudrate pre-scaler; Please adapt pre-scaler value so the LinBaudrate is between 99 and 200."
#endif
#define C_DEF_DEVICE_ID 0x3FU
#define C_LIN_IN_FREE 0x00U
#define C_LIN_IN_FULL 0x01U
#define C_LIN_IN_POSTPONE 0x02U
typedef union _LININBUF
{
	DFR_DIAG Diag;
	HVAC_CTRL Ctrl;
}

LININBUF;
typedef union _LINOUTBUF
{
	RFR_DIAG DiagResponse;
	HVAC_STATUS Status;
}

LINOUTBUF;
#define PLINOUTBUF (LINOUTBUF *)
#define INVALID 0
#define VALID 1
#define C_COMM_EVENT_NONE ((uint8_t)0x00U)
#define C_COMM_EVENT_CHIPRESET ((uint8_t)(1U << 0))
#define C_COMM_EVENT_STALL ((uint8_t)(1U << 1))
#define C_COMM_EVENT_EMRUN ((uint8_t)(1U << 2))
#define C_COMM_EVENT_LINERROR ((uint8_t)(1U << 3))
#pragma space dp
extern uint8_t g_u8BufferOutID;
extern LININBUF g_LinCmdFrameBuffer;
extern RFR_DIAG g_DiagResponse;
#pragma space none
#pragma space nodp
extern volatile uint8_t g_u8LinInFrameBufState;
extern volatile uint8_t g_u8ErrorCommunication;
extern volatile uint8_t g_u8ErrorCommBusTimeout;
extern uint8_t g_byCommEvent;
#pragma space none
extern volatile uint8_t LinProtectedID __attribute((nodp, addr(0x0E47)));
extern volatile uint8_t LinFrame[8] __attribute((nodp, addr(0x0E48)));
extern void LIN_Init(void);
#if ((CAN_COMM != FALSE) || (I2C_COMM != FALSE) || (PWM_COMM != FALSE) || (SPI_COMM != FALSE) || ((_SUPPORT_UART != FALSE) && (_SUPPORT_UART_IF_APP == C_UART_IF_QUAD_WHEEL_CAR)))
extern void LIN_Stop(void);
#endif
extern void HandleLinInMsg(void);
extern ml_Status_t mlu_ApplicationStop(void);
extern uint16_t p_ml_GetBaudRate(uint16_t u16Mlx4Clk);
extern uint16_t p_ml_GetLastBaudRate(uint16_t u16Mlx4Clk);
extern uint16_t p_ml_GetAutoBaudRate(uint16_t u16Mlx4Clk);
#endif
