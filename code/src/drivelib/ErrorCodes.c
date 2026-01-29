#include "AppBuild.h"
#include "drivelib/AppFunctions.h"
#include "drivelib/ErrorCodes.h"
#include <atomic.h>
#include <bl_bist.h>

#pragma space nodp
#define C_ERR_LOG_SZ 16U
uint16_t l_au16FiFoErrorLog[C_ERR_LOG_SZ];
uint8_t l_u8ErrorLogIdx = 0U;
#pragma space none
void ErrorLogInit(void)
{
	if(((IO_RST_CTRL_S & (B_RST_CTRL_SOFT_WBOOT | B_RST_CTRL_AWD_WBOOT))  !=  0U)  &&  ((bistResetInfo  !=  C_CHIP_STATE_FATAL_CRASH_RECOVERY)  &&  (bistResetInfo  !=  C_CHIP_STATE_FATAL_RECOVER_ENA)))
	{
		uint16_t i;
		for(i = 0U; i < C_ERR_LOG_SZ; i++){l_au16FiFoErrorLog[i] = C_ERR_NONE;}
		l_u8ErrorLogIdx = 0U;
	}
	if((bistHeader  ==  (BistHeader_t)C_CHIP_HEADER)  &&  (bistResetInfo  !=  (uint16_t)C_CHIP_STATE_CMD_RESET))
	{
		uint16_t u16RstCtrl = IO_RST_CTRL_S & (B_RST_CTRL_IWD_WBOOT | B_RST_CTRL_SOFT_WBOOT | B_RST_CTRL_AWD_WBOOT);
		if(u16RstCtrl  !=  0U){SetLastError(C_ERR_WD_RST | C_ERR_EXT | (u16RstCtrl  <<  8));}
	}
	{
		uint16_t u16Wakeup = (IO_PORT_MISC_IN & (B_PORT_MISC_IN_LOCAL_WU | B_PORT_MISC_IN_LIN_WU | B_PORT_MISC_IN_INTERNAL_WU))  >>  2;
		if(u16Wakeup  !=  0U){SetLastError(C_INF_WU | C_ERR_EXT | u16Wakeup);}
	}
	ENTER_SECTION(SYSTEM_MODE);
	IO_RST_CTRL_S = (B_RST_CTRL_IWD_WBOOT |
	B_RST_CTRL_DBG_WBOOT |
	B_RST_CTRL_HVDIG_WBOOT |
	B_RST_CTRL_SOFT_WBOOT |
	B_RST_CTRL_AWD_WBOOT);
	EXIT_SECTION();
}

void SetLastError(uint16_t u16ErrorCode)
{
	if((l_u8ErrorLogIdx  ==  0U)  ||  (l_au16FiFoErrorLog[l_u8ErrorLogIdx - 1U]  !=  u16ErrorCode))
	{
		l_au16FiFoErrorLog[l_u8ErrorLogIdx] = u16ErrorCode;
		if(l_u8ErrorLogIdx < (C_ERR_LOG_SZ - 1U)){l_u8ErrorLogIdx++;}
	}
}

uint16_t GetFirstError(void)
{
	uint16_t u16OldestErrorCode = l_au16FiFoErrorLog[0];
	if(l_u8ErrorLogIdx  !=  0U)
	{
		uint16_t *pu16Dest = &l_au16FiFoErrorLog[0];
		uint16_t *pu16Src = &l_au16FiFoErrorLog[1];
		ENTER_SECTION(ATOMIC_KEEP_MODE);
		{
			uint8_t i = l_u8ErrorLogIdx;
			while(--i  !=  0U){*pu16Dest++ = *pu16Src++;}
			*pu16Dest = C_ERR_NONE;
			l_u8ErrorLogIdx--;
		}
		EXIT_SECTION();
	}
	return(u16OldestErrorCode);
}

uint16_t PeakFirstError(void)
{
	return(l_au16FiFoErrorLog[0]);
}
