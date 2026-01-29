#ifndef ACT_PLTFRM_LIN_AUTOADDRESSING_H
#define ACT_PLTFRM_LIN_AUTOADDRESSING_H
#include "AppBuild.h"
#define C_LINAA_TIMEOUT 40U
#define C_DELAY_5US (uint16_t)((5UL * FPLL) / C_DELAY_CONST)
#define C_DELAY_10US (uint16_t)((10UL * FPLL) / C_DELAY_CONST)
#define C_DELAY_250US (uint16_t)((250UL * FPLL) / C_DELAY_CONST)
#define SLAVEADDRESSED 0x01
#define SLAVEFINALSTEP 0x02
#define WAITINGFORBREAK 0x04
#define LASTSLAVE 0x80
typedef struct _ADC_LINAA
{
	uint16_t au16Result_LinAA[18];
	uint16_t u16ADC_CRC;
}

ADC_LINAA;
#define PADC_LINAA (ADC_LINAA *)
#define C_LIN13AA_dI_1 120
#define C_LIN13AA_dI_2 120
#define C_LIN2xAA_dI_1_BSM2 120
#define C_LIN2xAA_dI_2_BSM2 120
typedef enum __attribute__((packed))
{
	AUTOADDRESSING_IDLE = 0,
	AUTOADDRESSING_STEP0,
	AUTOADDRESSING_STEP1,
	AUTOADDRESSING_STEP2,
	AUTOADDRESSING_STEP3,
	AUTOADDRESSING_STEP4,
	AUTOADDRESSING_STEP5,
	AUTOADDRESSING_STEP6,
	AUTOADDRESSING_WAIT,
	AUTOADDRESSING_DONE
}

T_AUTOADDRESSING_STEP;
#pragma space dp
extern volatile uint8_t g_u8LinAAMode;
#pragma space none
#pragma space nodp
extern uint8_t g_u8LinAATimeout;
extern uint16_t g_u16LinAATicker;
#pragma space none
extern void ml_SetSlaveNotAddressed(void);
extern void ml_SetSlaveAddressed(void);
extern uint16_t ml_GetAutoaddressingStatus(void);
extern void ml_InitAutoAddressing(void);
extern void ml_StopAutoAddressing(void);
extern void LinAATimeoutControl(void);
extern void ClearLinFrameTimeOut(void);
extern void LinAutoAddressingTimer(void);
#endif
