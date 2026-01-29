#include "AppBuild.h"
#include <mls_api.h>
#include "drivelib/ADC.h"
#include "drivelib/NV_Functions.h"
#include "drivelib/Timer.h"
#include "camculib/private_mathlib.h"
#include "commlib/LIN_AutoAddressing.h"
#include "commlib/LIN_Communication.h"
#include <plib.h>
#include <string.h>
#define C_LINAA_RBOND 60U
#define C_LINAA_ESHUNT 0U
#define C_LINAA_ISHUNT ((MLX_CALIB_t *)ADDR_NV_MLX_CALIB)->u16APP_TRIM12_LINAA_ISHUNT
#define C_LINAA_GDM ((int16_t)(0 - (((MLX_CALIB_t *)ADDR_NV_MLX_CALIB)->u16APP_TRIM15_SAADM)))
#define C_LINAA_SAADMCM ((MLX_CALIB_t *)ADDR_NV_MLX_CALIB)->i16APP_TRIM14_AASDMCM
#define C_LIN_AA_CURR_DIVISOR (int16_t)((4096L * 50 * 8) / 100)
#define C_LIN_AA_CURR_SDIV 14
#define C_LIN_AA_SAADMCM_DIVISOR (int16_t)(32768L / 8)
#define C_LIN_AA_SAADMCM_SDIV 12
#define C_CURRENT_SOURCE 240U
#define C_SLOW_BAUDRATE_DELAY (uint16_t)((40UL * FPLL) / C_DELAY_CONST)
#define C_T_FRAME_MAX (uint32_t)((1000000UL / CT_PERIODIC_RATE) * 1.4 * (34 + 10 * (8 + 1)))
#define C_LIN_KEY 0xB2A3U
#define _LINAA_ASM TRUE
#pragma space dp
static volatile uint8_t l_u8AutoAddressingFlags = (uint8_t)0x00U;
volatile uint8_t g_u8LinAAMode = 0U;
#pragma space none
#pragma space nodp
uint8_t g_u8LinAATimeout = 0U;
uint16_t g_u16LinAATicker = 0U;
uint16_t l_u16LinFrameMaxTime = 0U;
uint16_t l_u16LinFrameTimeOut = 0U;
uint16_t l_u16SlowBaudrateAdjustment = 0U;
uint16_t l_u16LinAATimerPeriod;
static uint16_t l_u16AutoAddressingCM_0 = 0U;
static uint16_t l_u16AutoAddressingDM_0 = 0;
static volatile uint16_t l_u16AutoAddressingCM = 0U;
static volatile uint16_t l_u16AutoAddressingDM = 0U;
static int16_t l_i16LINAA_PreSelCurr;
static int16_t l_i16LINAA_FinalSelCurr;
volatile ADC_LINAA LinAutoAddressing;
static uint16_t l_u16TrimBG;
static uint16_t l_u16TrimVdd;
#pragma space none
#define C_ADC_LINAA_CM_CT1 { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_LINAA_CM, .u3AdcVref = C_ADC_VREF_2_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3, .u1AdcReserved = 0U } }
#define C_ADC_LINAA_DM_CT1 { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_LINAA_DM, .u3AdcVref = C_ADC_VREF_2_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_CTIMER1_INT3, .u1AdcReserved = 0U } }
static ADC_SDATA_t const SBASE_LIN[] =
{
	C_ADC_LINAA_CM_CT1,
	C_ADC_LINAA_CM_CT1,
	C_ADC_LINAA_DM_CT1,
	C_ADC_LINAA_DM_CT1,
	C_ADC_LINAA_DM_CT1,
	C_ADC_LINAA_DM_CT1,
	C_ADC_LINAA_DM_CT1,
	C_ADC_LINAA_DM_CT1,
	C_ADC_LINAA_DM_CT1,
	C_ADC_LINAA_DM_CT1,
	C_ADC_LINAA_DM_CT1,
	C_ADC_LINAA_DM_CT1,
	C_ADC_LINAA_DM_CT1,
	C_ADC_LINAA_DM_CT1,
	C_ADC_LINAA_DM_CT1,
	C_ADC_LINAA_DM_CT1,
	C_ADC_LINAA_DM_CT1,
	C_ADC_LINAA_DM_CT1,
	{
		.u16 = C_ADC_EOS
	}
};
#define LINAA_TIMER_RESET() {IO_CTIMER1_CTRL = B_CTIMER1_STOP; }
#define LINAA_TIMER_SETUP() {IO_CTIMER1_CTRL = C_CTIMER1_DIV_CPU | C_CTIMER1_MODE_TIMER; }
#define LINAA_TIMER_START() {IO_CTIMER1_CTRL = B_CTIMER1_START; }
#define LINAA_TIMER_STOP() {IO_CTIMER1_CTRL = B_CTIMER1_STOP; }
#define LINAA_TIMER_PERIOD(p) {IO_CTIMER1_TREGB = p; }
#define LINAA_TIMER_IRQ_DIS() {IO_MLX16_ITC_MASK1_S &= ~B_MLX16_ITC_MASK1_CTIMER1_3; }
#define LIN_SET_XKEY() {IO_PORT_LIN_XKEY_S = C_LIN_KEY; }
#define LIN_CLR_XKEY() {IO_PORT_LIN_XKEY_S = 0x0000U; }
static inline void LINAA_RESET(void)
{
	IO_PORT_LINAA1 = B_PORT_LINAA1_LINAA_EN;
	IO_PORT_LINAA2 = B_PORT_LINAA2_LCD_DIS_LINAA;
}

static inline void LINAA_STOP(void)
{
	IO_PORT_LINAA1 = 0U;
	IO_PORT_LINAA2 = B_PORT_LINAA2_LCD_DIS_LINAA;
}

static inline void LINAA_InitMeasurement(uint16_t u16CMSuppGain)
{
	IO_PORT_LINAA1 = (B_PORT_LINAA1_LINAA_EN |
	B_PORT_LINAA1_LINAA_RST1 |
	B_PORT_LINAA1_LINAA_RST2 |
	(u16CMSuppGain & (M_PORT_LINAA1_LINAA_DIV | M_PORT_LINAA1_LINAA_GAIN)));
}

static INLINE void LINAA_SetCurrentSource(uint16_t u16CurrentLevel)
{
	uint16_t u16LinAA_Selection = ((u16CurrentLevel  >>  6) & M_PORT_LINAA2_LCD_SEL_LINAA);
	uint16_t u16LinAA_SelTrim = (u16CurrentLevel & M_TRIM_MISC_TRIM_LCD_LINAA);
	IO_PORT_LINAA2 = (IO_PORT_LINAA2 & ~(B_PORT_LINAA2_LCD_DIS_LINAA | M_PORT_LINAA2_LCD_SEL_LINAA)) |
	(u16LinAA_Selection |
	B_PORT_LINAA2_LCD_ON_LINAA);
	IO_TRIM_MISC = (IO_TRIM_MISC & ~M_TRIM_MISC_TRIM_LCD_LINAA) | u16LinAA_SelTrim;
	IO_PORT_LINAA1  |=  B_PORT_LINAA1_LINAA_CDOUTEN;
}

static void AutoAddressingReadADCResult(void)
{
	LINAA_TIMER_STOP();
	asm ("mov x, #_LinAutoAddressing");
	asm ("mov a, [x++]");
	asm ("add a, [x++]");
	asm ("lsr a, #1");
	asm ("mov _l_u16AutoAddressingCM, a");
	asm ("mov a, [x++]");
	asm ("add a, [x++]");
	asm ("add a, [x++]");
	asm ("add a, [x++]");
	asm ("add a, [x++]");
	asm ("add a, [x++]");
	asm ("add a, [x++]");
	asm ("add a, [x++]");
	asm ("add a, [x++]");
	asm ("add a, [x++]");
	asm ("add a, [x++]");
	asm ("add a, [x++]");
	asm ("add a, [x++]");
	asm ("add a, [x++]");
	asm ("add a, [x++]");
	asm ("add a, [x++]");
	asm ("lsr a, #1");
	asm ("mov _l_u16AutoAddressingDM, a");
}

void mlu_AutoAddressingStep(uint8_t StepNumber)
{
	if(StepNumber  ==  1U)
	{
		uint8_t u8AutoAddressingFlags;
		ENTER_SECTION(SYSTEM_MODE);
		IO_PORT_LIN_XCFG_S  |=  B_PORT_LIN_XCFG_DISTERM;
		EXIT_SECTION();
		u8AutoAddressingFlags = l_u8AutoAddressingFlags;
		u8AutoAddressingFlags  &=  (uint8_t) ~(SLAVEFINALSTEP |
		LASTSLAVE |
		WAITINGFORBREAK);
		l_u8AutoAddressingFlags = u8AutoAddressingFlags;
		if((u8AutoAddressingFlags & SLAVEADDRESSED)  ==  0U)
		{
			LINAA_InitMeasurement( ((CalibrationParams.u8APP_TRIM05_LINAA_TRIM & 0x1FU)  <<  4) |
			(CalibrationParams.u8APP_TRIM17_SC_GAIN & 0x0FU));
			LINAA_TIMER_SETUP();
			LINAA_TIMER_PERIOD(l_u16LinAATimerPeriod);
			IO_ADC_SBASE = (uint16_t)&l_au16AdcSource[0];
		}
		l_u16LinFrameTimeOut = l_u16LinFrameMaxTime;
	}
	else if((l_u8AutoAddressingFlags & (SLAVEADDRESSED | WAITINGFORBREAK))  ==  0U)
	{
		if(StepNumber  ==  2U)
		{
			if(l_u16SlowBaudrateAdjustment  !=  0U){DELAY(l_u16SlowBaudrateAdjustment);}
			DELAY(C_DELAY_5US);
			IO_PORT_LINAA1  &=  ~B_PORT_LINAA1_LINAA_RST1;
			DELAY(C_DELAY_10US);
			IO_PORT_LINAA1  &=  ~B_PORT_LINAA1_LINAA_RST2;
			HAL_ADC_Start();
			LINAA_TIMER_START();
			l_u16AutoAddressingCM = 0U;
			l_u16AutoAddressingDM = 0U;
			while((IO_ADC_CTRL & B_ADC_STOP)  ==  0U){}
			AutoAddressingReadADCResult();
		}
		else if(StepNumber  ==  3U)
		{
			if(l_u16SlowBaudrateAdjustment  !=  0U){DELAY(l_u16SlowBaudrateAdjustment);}
			if(CalibrationParams.u8APP_TRIM03_CalibVersion  >=  C_NV_MLX_VER_3){LINAA_SetCurrentSource(CalibrationParams.u8APP_TRIM16_IAA_Trim045mA);}
			else
			{
				ENTER_SECTION(SYSTEM_MODE);
				IO_PORT_LIN_XCFG_S  &=  ~B_PORT_LIN_XCFG_DISTERM;
				EXIT_SECTION();
			}
			IO_ADC_SBASE = (uint16_t)&l_au16AdcSource[0];
		}
		else if(StepNumber  ==  4U)
		{
			HAL_ADC_Start();
			LINAA_TIMER_START();
			l_u16AutoAddressingCM_0 = l_u16AutoAddressingCM;
			l_u16AutoAddressingDM_0 = l_u16AutoAddressingDM;
			l_u16AutoAddressingCM = 0U;
			l_u16AutoAddressingDM = 0U;
			while((IO_ADC_CTRL & B_ADC_STOP)  ==  0U){}
			AutoAddressingReadADCResult();
			{
				int16_t i16DM = ((int16_t)(l_u16AutoAddressingDM_0 - l_u16AutoAddressingDM)) +
				(int16_t) (p_MulI32_I16byI16( (int16_t)(l_u16AutoAddressingCM - l_u16AutoAddressingCM_0),
				C_LINAA_SAADMCM)  >>  C_LIN_AA_SAADMCM_SDIV);
				l_i16LINAA_PreSelCurr =
				(int16_t) (p_MulI32_I16byI16(i16DM, C_LINAA_GDM)  >>  C_LIN_AA_CURR_SDIV);
			}
		}
		else if(StepNumber  ==  5U)
		{
			if(l_u16SlowBaudrateAdjustment  !=  0U){DELAY(l_u16SlowBaudrateAdjustment);}
			if(l_i16LINAA_PreSelCurr > C_LIN13AA_dI_1)
			{
				LINAA_STOP();
				ENTER_SECTION(SYSTEM_MODE);
				IO_PORT_LIN_XCFG_S  |=  B_PORT_LIN_XCFG_DISTERM;
				EXIT_SECTION();
			}
			else
			{
				if(CalibrationParams.u8APP_TRIM03_CalibVersion  >=  C_NV_MLX_VER_3){LINAA_SetCurrentSource(0x100U | CalibrationParams.u8APP_TRIM17_IAA_Trim240mA);}
				else{LINAA_SetCurrentSource(CalibrationParams.u8APP_TRIM16_IAA_Trim205mA);}
				IO_ADC_SBASE = (uint16_t)&l_au16AdcSource[0];
				l_u8AutoAddressingFlags  |=  SLAVEFINALSTEP;
			}
		}
		else if((StepNumber  ==  6U)  &&  ((l_u8AutoAddressingFlags & SLAVEFINALSTEP)  !=  0U))
		{
			if(l_u16SlowBaudrateAdjustment  !=  0U){DELAY(l_u16SlowBaudrateAdjustment);}
			HAL_ADC_Start();
			LINAA_TIMER_START();
			l_u16AutoAddressingCM = 0U;
			l_u16AutoAddressingDM = 0U;
			while((IO_ADC_CTRL & B_ADC_STOP)  ==  0U){}
			AutoAddressingReadADCResult();
			{
				int16_t i16DM = ((int16_t)(l_u16AutoAddressingDM_0 - l_u16AutoAddressingDM)) +
				(int16_t) (p_MulI32_I16byI16( (int16_t)(l_u16AutoAddressingCM - l_u16AutoAddressingCM_0),
				C_LINAA_SAADMCM)  >>  C_LIN_AA_SAADMCM_SDIV);
				l_i16LINAA_FinalSelCurr =
				(int16_t) (p_MulI32_I16byI16(i16DM, C_LINAA_GDM)  >>  C_LIN_AA_CURR_SDIV);
			}
		}
	}
	if(COLIN_LINstatus.event_overflow  !=  0U)
	{
		StepNumber = 8U;
		l_u8AutoAddressingFlags  &=  (uint8_t) ~(SLAVEFINALSTEP | LASTSLAVE);
		(void)ml_GetState(ML_CLR_LIN_CMD_OVERFLOW);
	}
	if((StepNumber  ==  0U)  ||  (StepNumber  >=  7U))
	{
		LINAA_TIMER_RESET();
		l_u8AutoAddressingFlags  |=  (uint8_t)WAITINGFORBREAK;
		if((StepNumber  ==  7U)  &&  ((l_u8AutoAddressingFlags & SLAVEFINALSTEP)  !=  0U))
		{
			if(l_i16LINAA_FinalSelCurr < C_LIN13AA_dI_2)
			{
				l_u8AutoAddressingFlags = (l_u8AutoAddressingFlags & (uint8_t) ~WAITINGFORBREAK) |
				(uint8_t)LASTSLAVE;
			}
		}
		if(l_u16SlowBaudrateAdjustment  !=  0U){DELAY(l_u16SlowBaudrateAdjustment);}
		LINAA_STOP();
		ENTER_SECTION(SYSTEM_MODE);
		IO_PORT_LIN_XCFG_S  &=  ~B_PORT_LIN_XCFG_DISTERM;
		EXIT_SECTION();
	}
}

void ml_SetSlaveNotAddressed(void)
{
	l_u8AutoAddressingFlags  &=  ~SLAVEADDRESSED;
}

void ml_SetSlaveAddressed(void)
{
	l_u8AutoAddressingFlags  |=  SLAVEADDRESSED;
}

void ml_InitAutoAddressing(void)
{
	uint16_t u16LinBaudrate = p_ml_GetBaudRate(MLX4_FPLL);
	if((u16LinBaudrate  !=  0U)  &&  (u16LinBaudrate < 12000U)){l_u16SlowBaudrateAdjustment = C_SLOW_BAUDRATE_DELAY;}
	l_u16LinFrameMaxTime = p_DivU16_U32byU16(C_T_FRAME_MAX, u16LinBaudrate);
	l_u16LinFrameTimeOut = 0U;
	HAL_ADC_StopSafe();
	l_au16AdcSource[0] = (uint16_t)&LinAutoAddressing;
	memcpy( (void *)&l_au16AdcSource[1], (const void *)&SBASE_LIN[0], sizeof(SBASE_LIN));
	HAL_ADC_Setup(
	C_ADC_ASB_NEVER |
	C_ADC_INT_SCHEME_NOINT |
	B_ADC_SATURATE |
	B_ADC_NO_INTERLEAVE |
	C_ADC_SOC_SOURCE_HARD_CTRIG |
	C_ADC_SOS_SOURCE_2ND_HARD_CTRIG);
	HAL_ADC_ClearErrors();
	l_u8AdcMode = (uint8_t)C_ADC_MODE_LINAA;
	l_u16TrimBG = IO_TRIM_BG_BIAS;
	IO_TRIM_BG_BIAS = (l_u16TrimBG & ~(M_TRIM_BG_BIAS_PRE_TR_BIAS | M_TRIM_BG_BIAS_PRE_TR_BGA)) | ((0x20U  <<  8) | 8U);
	l_u16TrimVdd = IO_TRIM_VDD;
	IO_TRIM_VDD = l_u16TrimVdd & ~M_TRIM_VDD_PRE_TR_VDDA;
	LINAA_TIMER_RESET();
	ENTER_SECTION(SYSTEM_MODE);
	LINAA_TIMER_IRQ_DIS();
	LIN_SET_XKEY();
	EXIT_SECTION();
	LINAA_RESET();
	l_u8AutoAddressingFlags  |=  (uint8_t)WAITINGFORBREAK;
	if(l_u16SlowBaudrateAdjustment  ==  C_SLOW_BAUDRATE_DELAY){l_u16LinAATimerPeriod = (PLL_FREQ / 83333U);}
	else{l_u16LinAATimerPeriod = (PLL_FREQ / 166667U);}
}

void ml_StopAutoAddressing(void)
{
	IO_TRIM_BG_BIAS = l_u16TrimBG;
	IO_TRIM_VDD = l_u16TrimVdd;
	ENTER_SECTION(SYSTEM_MODE);
	LIN_CLR_XKEY();
	EXIT_SECTION();
	l_u8AdcMode = (uint8_t)C_ADC_MODE_IDLE;
}

uint16_t ml_GetAutoaddressingStatus(void)
{
	uint16_t u16Result = FALSE;
	if((l_u8AutoAddressingFlags & LASTSLAVE)  !=  0U){u16Result = TRUE;}
	return(u16Result);
}

void LinAATimeoutControl(void)
{
	STD_LIN_PARAMS_t *pStdLin = (STD_LIN_PARAMS_t *)ADDR_NV_STD_LIN_1;
	ml_StopAutoAddressing();
	g_u8NAD = pStdLin  ->  u8NAD;
	g_u16LinAATicker = 0U;
	g_u8LinAAMode = 0U;
	l_u16LinFrameTimeOut = 0U;
}

void ClearLinFrameTimeOut(void)
{
	l_u16LinFrameTimeOut = 0U;
}

void LinAutoAddressingTimer(void)
{
	if(g_u16LinAATicker  !=  0U)
	{
		--g_u16LinAATicker;
		if(g_u16LinAATicker  ==  0U)
		{
			--g_u8LinAATimeout;
			if(g_u8LinAATimeout  ==  0U){LinAATimeoutControl();}
			else{g_u16LinAATicker = PI_TICKS_PER_SECOND;}
		}
	}
	{
		uint16_t u16Mlx4Reconnect = FALSE;
		ENTER_SECTION(ATOMIC_KEEP_MODE);
		if(l_u16LinFrameTimeOut  !=  0U)
		{
			l_u16LinFrameTimeOut--;
			if(l_u16LinFrameTimeOut  ==  0U){u16Mlx4Reconnect = TRUE;}
		}
		EXIT_SECTION();
		if(u16Mlx4Reconnect  !=  FALSE)
		{
			(void)ml_Disconnect();
			(void)ml_Connect();
		}
	}
	if((l_u8AutoAddressingFlags & WAITINGFORBREAK)  !=  0U){p_AwdAck();}
}
