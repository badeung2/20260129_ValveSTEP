#ifndef DRIVE_LIB_NV_FUNCTIONS_H_
#define DRIVE_LIB_NV_FUNCTIONS_H_
#include "AppBuild.h"
#include "drivelib/NV_UserPage.h"
#define C_NV_WRT_KEY 0x07U
#define SZ_NV_BLOCK 0x0008U
#define ID_NV_HDR 0x0001U
#define ID_NV_STD_LIN 0x0002U
#define ID_NV_EOL 0x0010U
#define ID_NV_ACT_PARAMS 0x0020U
#define ID_NV_SENSOR_PARAMS 0x0040U
#define ID_NV_ACT_STALL 0x0080U
#define ID_NV_PERIODIC 0x0020U
#define C_ERR_NV_LIN_STD_1 0x0001U
#define C_ERR_NV_LIN_STD_2 0x0002U
#define C_ERR_NV_HDR_RST_COUNT 0x0010U
#define C_ERR_NV_EOL 0x0040U
#define C_ERR_NV_APP_PARAMS 0x0080U
#define C_ERR_NV_ACT_PARAMS 0x0100U
#define C_ERR_NV_SENSOR 0x0200U
#define C_ERR_NV_ACT_STALL 0x0400U
#define C_ERR_NV_I2C_PARAMS 0x0800U
#define C_ERR_NV_CAN_PARAMS 0x1000U
#define C_ERR_NV_HDR_KEEP_COUNT 0x8000U
#define C_NV_WRT_LIN_ID_1 0x0001U
#define C_NV_WRT_LIN_ID_2 0x0002U
#define C_NV_WRT_LIN_ID_ALL (C_NV_WRT_LIN_ID_1 | C_NV_WRT_LIN_ID_2)
#define C_NV_WRT_FORCE 0x0004U
typedef struct
{
	uint16_t u16Address;
	uint16_t u16Size;
	uint16_t u16ErrorCode;
}

NV_CRC;
#pragma space dp
#pragma space none
#pragma space nodp
extern uint16_t g_u16CP_FreqTrim_RT;
#pragma space none
extern uint16_t NV_CheckCRC(void);
extern uint16_t NV_WriteBlock(const uint16_t u16Address, uint16_t *pu16Data, uint16_t u16SizeW, uint16_t u16Force);
extern uint16_t NV_WriteLIN_STD(volatile STD_LIN_PARAMS_t *pLinData, uint16_t u16BlockID);
extern uint16_t NV_WriteAPP(volatile APP_PARAMS_t* pAppData);
extern uint16_t NV_WriteEOL(volatile APP_EOL_t* pEolData);
extern uint16_t NV_WriteActParams(volatile ACT_PARAMS_t* pActData);
extern uint16_t NV_WriteActStall(volatile ACT_STALL_t* pActStall);
extern uint16_t NV_WritePatch(volatile PATCH_HDR_t* pPatch);
extern uint16_t NV_WriteUserDefaults(uint16_t u16BlockIdMask);
extern void p_CopyU16(const uint16_t u16Size, uint16_t *pu16Dest, const uint16_t *pu16Src);
extern uint16_t NV_MlxCalib(void);
extern void NV_AppStore(uint16_t u16ParamLSW, uint16_t u16ParamMSW);
#endif
