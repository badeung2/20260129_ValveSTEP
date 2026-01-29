#ifndef ACT_ADC_H_
#define ACT_ADC_H_
#include "AppBuild.h"
#include "drivelib/ADC.h"
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE)
#include "drivelib/MotorDriver.h"
#endif
typedef enum __attribute__((packed))
{
C_ADC_MODE_ON_STEPPER = 3U,
C_ADC_MODE_ON_BEMF
} ADC_MODE;
#if (_SUPPORT_POTI != FALSE)
#define C_POS_MOVAVG_SSZ 5U
#define C_POS_MOVAVG_SZ (1 << C_POS_MOVAVG_SSZ)
#endif
#if ((_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)) || (ANA_COMM != FALSE)
#define C_DELAY_1MS (uint16_t)((1000UL * FPLL) / C_DELAY_CONST)
#endif
#if (_SUPPORT_APP_TYPE == C_APP_SOLENOID)
typedef struct _ADC_RESULTS
{
uint16_t u16AdcVs;
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
uint16_t u16AdcCurr;
uint16_t u16AdcVsmF;
uint16_t u16AdcTj;
uint16_t u16CRC;
} ADC_RESULTS;
#else
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM)
typedef struct _ADC_RESULTS
{
uint16_t u16AdcVs_1;
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
uint16_t u16AdcCurrA_1;
uint16_t u16AdcCurrB_1;
uint16_t u16AdcVsmF_1;
uint16_t u16AdcTj_1;
#if (_SUPPORT_POTI != FALSE)
uint16_t u16AdcPotiPos;
#elif (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0)
uint16_t u16AdcIO0HV;
#endif
uint16_t u16AdcVs_2;
#if (_SUPPORT_IO_CMD_SELECT_LVIO != FALSE)
uint16_t u16AdcIoSelectA;
#if (_SUPPORT_NR_OF_IO_SELECT >= 2U)
uint16_t u16AdcIoSelectB;
#if (_SUPPORT_NR_OF_IO_SELECT >= 3U)
uint16_t u16AdcIoSelectC;
#endif
#endif
#endif
uint16_t u16AdcCurrA_2;
uint16_t u16AdcCurrB_2;
uint16_t u16AdcVsmF_2;
uint16_t u16AdcTj_2;
#if (ANA_COMM != FALSE)
uint16_t u16AdcRefPos;
#endif
uint16_t u16CRC;
} ADC_RESULTS;
#elif (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM_BIPOLAR)
typedef struct _ADC_RESULTS
{
#if (_SUPPORT_ADC_BGD != FALSE)
uint16_t u16AdcVs;
uint16_t u16AdcVsmF;
uint16_t u16AdcTj;
uint16_t u16AdcCurr;
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
#endif
uint16_t u16AdcVaux;
uint16_t u16AdcVbgd;
#else
uint16_t u16AdcVs;
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
#endif
uint16_t u16AdcCurr;
uint16_t u16AdcVsmF;
uint16_t u16AdcTj;
#endif
uint16_t u16CRC;
} ADC_RESULTS;
#elif (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF)
typedef struct _ADC_RESULTS
{
uint16_t u16AdcVs;
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
#if (_SUPPORT_STALLDET_BRI != FALSE)
uint16_t u16AdcVbemfA;
#endif
uint16_t u16AdcCurrA;
#if (_SUPPORT_STALLDET_BRI != FALSE)
uint16_t u16AdcVbemfB;
#endif
uint16_t u16AdcVsmF;
uint16_t u16AdcTj;
uint16_t u16CRC;
} ADC_RESULTS;
#elif (_SUPPORT_PWM_MODE == BIPOLAR_PWM_SINGLE_INDEPENDENT_GND)
typedef struct _ADC_RESULTS
{
#if (_SUPPORT_ADC_BGD != FALSE)
uint16_t u16AdcVs;
uint16_t u16AdcVsmF;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint16_t u16Resolver_X;
uint16_t u16Resolver_Y;
#else
uint16_t u16AdcTj;
#endif
uint16_t u16AdcCurrA;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint16_t u16AdcTj;
uint16_t u16AdcVdda;
#else
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
#endif
uint16_t u16AdcVaux;
uint16_t u16AdcVbgd;
uint16_t u16AdcCurrB;
#else
uint16_t u16AdcVs;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint16_t u16Resolver_X;
uint16_t u16Resolver_Y;
#else
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
#endif
uint16_t u16AdcCurrA;
uint16_t u16AdcVsmF;
uint16_t u16AdcTj;
uint16_t u16AdcCurrB;
#endif
uint16_t u16CRC;
} ADC_RESULTS;
#elif (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_ALLPWM_MIRROR)
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
typedef struct _ADC_RESULTS
{
uint16_t u16AdcVs;
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
#endif
uint16_t u16AdcCurrA;
#else
uint16_t u16AdcCurrA;
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
uint16_t u16AdcVdda;
#endif
#endif
uint16_t u16AdcVsmF;
uint16_t u16AdcTj;
uint16_t u16AdcCurrB;
uint16_t u16CRC;
} ADC_RESULTS;
#else
typedef struct _ADC_RESULTS
{
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
uint16_t u16AdcVs;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint16_t u16Resolver_X;
uint16_t u16Resolver_Y;
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
#endif
uint16_t u16AdcCurrA;
#else
uint16_t u16AdcVs;
uint16_t u16AdcCurrA;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint16_t u16Resolver_X;
uint16_t u16Resolver_Y;
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
uint16_t u16AdcVdda;
#endif
#endif
uint16_t u16AdcVsmF;
uint16_t u16AdcTj;
uint16_t u16AdcCurrB;
uint16_t u16CRC;
} ADC_RESULTS;
#endif
#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR)
#if (_SUPPORT_STALLDET_BZC != FALSE)
typedef struct _ADC_RESULTS
{
uint16_t u16AdcVs;
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
uint16_t u16AdcVsmF;
uint16_t u16AdcTj;
uint16_t u16AdcCurr;
uint16_t u16CRC;
} ADC_RESULTS;
#elif (_SUPPORT_FOC_MODE != FOC_MODE_NONE)
typedef struct _ADC_RESULTS
{
uint16_t u16AdcVs;
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
#endif
uint16_t u16AdcCurrA;
#else
uint16_t u16AdcCurrA;
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
uint16_t u16AdcVdda;
#endif
#endif
uint16_t u16AdcVsmF;
uint16_t u16AdcTj;
uint16_t u16AdcCurrB;
uint16_t u16CRC;
} ADC_RESULTS;
#else
typedef struct _ADC_RESULTS
{
#if (_SUPPORT_ADC_SOS_TRIGGER == ADC_SOS_TRIGGER_SW)
uint16_t u16AdcVs;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint16_t u16Resolver_X;
uint16_t u16Resolver_Y;
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
#endif
uint16_t u16AdcCurrA;
#else
uint16_t u16AdcVs;
uint16_t u16AdcCurrA;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint16_t u16Resolver_X;
uint16_t u16Resolver_Y;
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE) && (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
uint16_t u16AdcVdda;
#endif
#endif
uint16_t u16AdcVsmF;
uint16_t u16AdcTj;
uint16_t u16AdcCurrB;
uint16_t u16CRC;
} ADC_RESULTS;
#endif
#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_SUP)
typedef struct _ADC_RESULTS
{
#if (_SUPPORT_ADC_BGD != FALSE)
uint16_t u16AdcVs;
uint16_t u16AdcVsmF;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint16_t u16Resolver_X;
uint16_t u16Resolver_Y;
uint16_t u16AdcTj;
uint16_t u16AdcVdda;
#else
uint16_t u16AdcTj;
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
#endif
uint16_t u16AdcVaux;
uint16_t u16AdcVbgd;
#else
uint16_t u16AdcVs;
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
uint16_t u16AdcVsmF;
uint16_t u16AdcTj;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint16_t u16Resolver_X;
uint16_t u16Resolver_Y;
#endif
uint16_t u16AdcCurrA;
#endif
uint16_t u16CRC;
} ADC_RESULTS;
#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_MIRROR_GND)
typedef struct _ADC_RESULTS
{
#if (_SUPPORT_ADC_BGD != FALSE)
uint16_t u16AdcVs;
uint16_t u16AdcVsmF;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint16_t u16Resolver_X;
uint16_t u16Resolver_Y;
#else
uint16_t u16AdcTj;
#endif
uint16_t u16AdcCurr;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint16_t u16AdcTj;
uint16_t u16AdcVdda;
#else
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
#endif
uint16_t u16AdcVaux;
uint16_t u16AdcVbgd;
#else
uint16_t u16AdcVs;
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
uint16_t u16AdcCurr;
uint16_t u16AdcVsmF;
uint16_t u16AdcTj;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint16_t u16Resolver_X;
uint16_t u16Resolver_Y;
#endif
#endif
uint16_t u16CRC;
} ADC_RESULTS;
#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) || \
(_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND)
typedef struct _ADC_RESULTS
{
#if (_SUPPORT_ADC_BGD != FALSE)
uint16_t u16AdcVs;
uint16_t u16AdcVsmF;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint16_t u16Resolver_X;
uint16_t u16Resolver_Y;
#else
uint16_t u16AdcTj;
#if (ANA_COMM != FALSE)
uint16_t u16AdcVana;
#elif (_SUPPORT_IO_DUT_SELECT_HVIO_LR != FALSE) && (IO_DUT_SELECT == PIN_FUNC_IO_0)
uint16_t u16AdcIO0HV;
#endif
#endif
uint16_t u16AdcCurrA;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint16_t u16AdcTj;
#if (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE)
uint16_t u16Resolver2_X;
uint16_t u16Resolver2_Y;
#else
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
uint16_t u16AdcVdda;
uint16_t u16AdcVaux;
uint16_t u16AdcVbgd;
#endif
#endif
#else
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
uint16_t u16AdcVaux;
uint16_t u16AdcVbgd;
#endif
#endif
uint16_t u16AdcCurrB;
#else
uint16_t u16AdcVs;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
uint16_t u16Resolver_X;
uint16_t u16Resolver_Y;
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE)
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
#endif
uint16_t u16AdcCurrA;
uint16_t u16AdcVsmF;
uint16_t u16AdcTj;
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
#if (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && (_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE)
uint16_t u16Resolver2_X;
uint16_t u16Resolver2_Y;
#elif (_SUPPORT_ADC_VDDA_VDDD != FALSE)
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
#endif
#elif (ANA_COMM != FALSE)
uint16_t u16AdcVana;
#elif (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0)
uint16_t u16AdcIO0HV;
#elif (_SUPPORT_NTC != FALSE)
uint16_t u16AdcNTC;
#elif (_SUPPORT_ADC_BGD != FALSE)
uint16_t u16AdcVbgd;
#elif (_SUPPORT_POTI != FALSE)
uint16_t u16AdcPotiPos;
#endif
uint16_t u16AdcCurrB;
#endif
uint16_t u16CRC;
} ADC_RESULTS;
#elif (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF)
typedef struct _ADC_RESULTS
{
uint16_t u16AdcVs;
uint16_t u16AdcVdda;
uint16_t u16AdcVddd;
uint16_t u16AdcCurrA;
#if (_SUPPORT_STALLDET_BRI != FALSE)
uint16_t u16AdcVbemf;
#endif
uint16_t u16AdcVsmF;
uint16_t u16AdcTj;
uint16_t u16CRC;
} ADC_RESULTS;
#endif
#endif
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE)
typedef struct _ADC_RESULTS_3
{
uint16_t u16AdcVs;
uint16_t u16AdcCurrA;
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
uint16_t u16AdcVdda;
#if (_SUPPORT_ADC_BGD != FALSE)
uint16_t u16AdcVbgd;
#endif
uint16_t u16AdcVddd;
#endif
uint16_t u16AdcCurrB;
uint16_t u16AdcVsmF;
uint16_t u16AdcTj;
#if (_SUPPORT_ADC_VBOOST != FALSE)
uint16_t u16AdcVboost;
#endif
uint16_t u16AdcCurrC;
uint16_t u16CRC;
} ADC_RESULTS_3;
#endif
#if (_SUPPORT_STALLDET_BZC != FALSE)
typedef struct _ADC_MOTORRUN_BEMF
{
uint16_t u16AdcVs;
uint16_t u16AdcVsmF;
#if (_SUPPORT_BEMF_STARPOINT != FALSE)
uint16_t u16AdcVio0HV;
#endif
uint16_t u16AdcVphA;
uint16_t u16AdcCurr;
uint16_t u16AdcTj;
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
uint16_t u16AdcVdda;
#endif
uint16_t u16AdcVphB;
uint16_t u16CRC;
} ADC_MOTORRUN_BEMF;
#endif
#if ((_SUPPORT_POTI != FALSE) && (_SUPPORT_POTI_MODE == C_POTI_MODE_CLOSED_LOOP)) || (ANA_COMM != FALSE)
typedef struct _ADC_POS
{
uint16_t u16AdcVs;
#if (_SUPPORT_POTI != FALSE)
uint16_t u16AdcPotiPos;
#endif
#if (ANA_COMM != FALSE)
uint16_t u16AdcRefPos;
#endif
uint16_t u16CRC;
} ADC_POS;
#endif
#pragma space dp
#if (C_MOTOR_PHASES != 1)
extern int16_t g_i16MotorCurrentCoilA;
#if (_SUPPORT_PWM_MODE != BIPOLAR_FULL_STEP_BEMF) && (_SUPPORT_PWM_MODE != BIPOLAR_HALF_STEP_BEMF)
extern int16_t g_i16MotorCurrentCoilB;
#if (C_MOTOR_PHASES == 3)
extern int16_t g_i16MotorCurrentCoilC;
extern int16_t g_i16MotorCurrentCoilY;
#elif (C_MOTOR_PHASES == 4)
#define g_i16MotorCurrentCoilY g_i16MotorCurrentCoilB
#endif
#endif
#endif
extern uint16_t g_u16MotorCurrentPeak;
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_LA != FALSE)
extern int16_t g_i16MotorCurrentAngle;
#endif
#if (_SUPPORT_STALLDET_BRI != FALSE)
extern uint16_t g_u16MotorBemfVoltage;
#endif
#if (ANA_COMM != FALSE)
extern uint16_t g_u16ReferencePotiPos;
#endif
#pragma space none
#pragma space nodp
extern volatile ADC_RESULTS l_AdcResult;
#pragma space none
#if (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF) || (_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF)
extern void ADC_Start(uint16_t u16FullStep, uint16_t u16IrqEna);
#else
extern void ADC_Start(uint16_t u16IrqEna);
#endif
#if (_SUPPORT_MOTION_DET == C_MOTION_DET_BEMF) || (_SUPPORT_MOTION_DET == C_MOTION_DET_SENSOR)
extern void ADC_MovementDetection(void);
#endif
#if (_SUPPORT_STALLDET_BZC != FALSE)
extern void ADC_BEMF_Start(uint16_t u16Idx);
#endif
#if (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
extern uint16_t ADC_GetRawMotorDriverCurrent(uint16_t u16MicroStepIdx);
#else
extern uint16_t ADC_GetRawMotorDriverCurrent(void);
#endif
extern void ADC_MeasureVsupplyAndTemperature(void);
extern uint16_t ADC_GetNewSampleVsupply(void);
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE)
extern void MeasureResolverPos(void);
extern uint16_t GetResolverPosition(void);
#endif
#if (_SUPPORT_POTI != FALSE)
extern uint16_t ADC_PotentiometerPosition(void);
#endif
#if (ANA_COMM != FALSE)
extern uint16_t ADC_ReferencePosition(void);
#endif
#if (_SUPPORT_POTI != FALSE) || (ANA_COMM != FALSE)
extern void ADC_ActRefPositionInit(void);
#endif
static inline uint16_t Get_RawVsupplyChip(void)
{
#if (_SUPPORT_STALLDET_BZC != FALSE)
extern uint8_t l_u8AdcMode;
extern volatile ADC_MOTORRUN_BEMF l_AdcMotorRunBemf;
#endif
extern uint16_t l_u16HighVoltOffset;
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
return ( ((l_AdcResult.u16AdcVs_1 + l_AdcResult.u16AdcVs_2) >> 1) - l_u16HighVoltOffset);
#else
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE)
if (g_u8TriplePWM != FALSE)
{
extern volatile ADC_RESULTS_3 l_AdcResult3;
return (l_AdcResult3.u16AdcVs - l_u16HighVoltOffset);
}
else
#elif (_SUPPORT_STALLDET_BZC != FALSE)
if (l_u8AdcMode == (uint8_t)C_ADC_MODE_ON_BEMF)
{
return (l_AdcMotorRunBemf.u16AdcVs - l_u16HighVoltOffset);
}
else
#endif
{
return (l_AdcResult.u16AdcVs - l_u16HighVoltOffset);
}
#endif
}
static inline void Set_RawVsupplyChip(uint16_t u16Value)
{
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
l_AdcResult.u16AdcVs_1 = u16Value;
l_AdcResult.u16AdcVs_2 = u16Value;
#else
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_VARIABLE_PWM
if (g_u8TriplePWM != FALSE)
{
extern volatile ADC_RESULTS_3 l_AdcResult3;
l_AdcResult3.u16AdcVs = u16Value;
}
else
#endif
{
l_AdcResult.u16AdcVs = u16Value;
}
#endif
}
static inline uint16_t Get_RawVmotorF(void)
{
#if (_SUPPORT_STALLDET_BZC != FALSE)
extern uint8_t l_u8AdcMode;
extern volatile ADC_MOTORRUN_BEMF l_AdcMotorRunBemf;
#endif
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
return ( ((l_AdcResult.u16AdcVsmF_1 + l_AdcResult.u16AdcVsmF_2) >> 1) - Get_VsmOffset() );
#else
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE)
if (g_u8TriplePWM != FALSE)
{
extern volatile ADC_RESULTS_3 l_AdcResult3;
return (l_AdcResult3.u16AdcVsmF - Get_VsmOffset() );
}
else
#elif (_SUPPORT_STALLDET_BZC != FALSE)
if (l_u8AdcMode == (uint8_t)C_ADC_MODE_ON_BEMF)
{
return (l_AdcMotorRunBemf.u16AdcVsmF - Get_VsmOffset() );
}
else
#endif
{
uint16_t u16RawVSMF = 0U;
if ((int16_t)l_AdcResult.u16AdcVsmF > Get_VsmOffset())
{
u16RawVSMF = (uint16_t)(l_AdcResult.u16AdcVsmF - Get_VsmOffset());
}
return (u16RawVSMF);
}
#endif
}
static inline uint16_t Get_RawTemperature(void)
{
#if (_SUPPORT_STALLDET_BZC != FALSE)
extern uint8_t l_u8AdcMode;
extern volatile ADC_MOTORRUN_BEMF l_AdcMotorRunBemf;
#endif
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
return ( (l_AdcResult.u16AdcTj_1 + l_AdcResult.u16AdcTj_2) >> 1);
#else
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE)
if (g_u8TriplePWM != FALSE)
{
extern volatile ADC_RESULTS_3 l_AdcResult3;
return (l_AdcResult3.u16AdcTj);
}
else
#elif (_SUPPORT_STALLDET_BZC != FALSE)
if (l_u8AdcMode == (uint8_t)C_ADC_MODE_ON_BEMF)
{
return (l_AdcMotorRunBemf.u16AdcTj);
}
else
#endif
{
#if (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
return (l_AdcResult.u16AdcTj);
#else
return (600);
#endif
}
#endif
}
static inline void Set_RawTemperature(uint16_t u16Value)
{
#if (_SUPPORT_STALLDET_BZC != FALSE)
extern uint8_t l_u8AdcMode;
extern volatile ADC_MOTORRUN_BEMF l_AdcMotorRunBemf;
#endif
#if (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) && (_SUPPORT_APP_TYPE != C_APP_SOLENOID)
l_AdcResult.u16AdcTj_1 = u16Value;
l_AdcResult.u16AdcTj_2 = u16Value;
#else
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && (_SUPPORT_VARIABLE_PWM != FALSE)
if (g_u8TriplePWM != FALSE)
{
extern volatile ADC_RESULTS_3 l_AdcResult3;
l_AdcResult3.u16AdcTj = u16Value;
}
else
#elif (_SUPPORT_STALLDET_BZC != FALSE)
if (l_u8AdcMode == (uint8_t)C_ADC_MODE_ON_BEMF)
{
l_AdcMotorRunBemf.u16AdcTj = u16Value;
}
else
#endif
{
#if (_SUPPORT_PWM_ADC_TRIGGER_MODE == ADC_TRIGGER_PWM_MIN_MAX)
l_AdcResult.u16AdcTj = u16Value;
#else
(void)u16Value;
#endif
}
#endif
}
#if (_SUPPORT_ADC_VDDA_VDDD != FALSE)
static inline uint16_t Get_AdcVdda(void)
{
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_VARIABLE_PWM
if (g_u8TriplePWM != FALSE)
{
extern volatile ADC_RESULTS_3 l_AdcResult3;
return (l_AdcResult3.u16AdcVdda - C_OADC);
}
else
#endif
{
return (l_AdcResult.u16AdcVdda - C_OADC);
}
}
static inline uint16_t Get_AdcVddd(void)
{
#if (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR) || (_SUPPORT_PWM_MODE == BIPOLAR_TRIPHASE_ALLPWM_MIRROR) || \
(_SUPPORT_TRIAXIS_MLX9038x != FALSE)
return ( (C_MIN_VDDD + C_MAX_VDDD) / 2U);
#else
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_VARIABLE_PWM
if (g_u8TriplePWM != FALSE)
{
extern volatile ADC_RESULTS_3 l_AdcResult3;
return (l_AdcResult3.u16AdcVddd - C_OADC);
}
else
#endif
{
return (l_AdcResult.u16AdcVddd - C_OADC);
}
#endif
}
#endif
#if (_SUPPORT_ADC_BGD != FALSE)
static inline uint16_t Get_AdcVaux(void)
{
return (l_AdcResult.u16AdcVaux - C_OADC);
}
static inline uint16_t Get_AdcVbgd(void)
{
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) && _SUPPORT_VARIABLE_PWM
if (g_u8TriplePWM != FALSE)
{
extern volatile ADC_RESULTS_3 l_AdcResult3;
return (l_AdcResult3.u16AdcVbgd - C_OADC);
}
else
#endif
{
return (l_AdcResult.u16AdcVbgd - C_OADC);
}
}
#endif
#if (_SUPPORT_MOTION_DET == C_MOTION_DET_BEMF)
static inline int16_t Get_RawVoltagePhaseU(void)
{
extern uint16_t l_u16HighVoltOffset;
extern volatile ADC_RESULTS l_AdcResult;
return ( (int16_t)(l_AdcResult.u16AdcCurrA - l_u16HighVoltOffset));
}
static inline int16_t Get_RawVoltagePhaseV(void)
{
extern uint16_t l_u16HighVoltOffset;
extern volatile ADC_RESULTS l_AdcResult;
return ( (int16_t)(l_AdcResult.u16AdcVboost - l_u16HighVoltOffset));
}
static inline int16_t Get_RawVoltagePhaseW(void)
{
extern uint16_t l_u16HighVoltOffset;
extern volatile ADC_RESULTS l_AdcResult;
return ( (int16_t)(l_AdcResult.u16AdcCurrB - l_u16HighVoltOffset));
}
#endif
#if (_SUPPORT_STALLDET_BZC != FALSE)
static inline void Set_ZcTime(uint16_t u16Value)
{
extern uint16_t l_u16ZcTime;
l_u16ZcTime = u16Value;
}
#endif
#if (_SUPPORT_TRIAXIS_MLX9038x != FALSE) && FALSE
static inline uint16_t Get_ResolverPosX(void)
{
return (l_AdcResult.u16Resolver_X);
}
static inline uint16_t Get_ResolverPosY(void)
{
return (l_AdcResult.u16Resolver_Y);
}
#if (_DEBUG_MLX90381 == FALSE) && (_SUPPORT_RESOLVER2_X_IO != PIN_FUNC_IO_NONE) && \
(_SUPPORT_RESOLVER2_Y_IO != PIN_FUNC_IO_NONE)
static inline uint16_t Get_Resolver2PosX(void)
{
return (l_AdcResult.u16Resolver2_X);
}
static inline uint16_t Get_Resolver2PosY(void)
{
return (l_AdcResult.u16Resolver2_Y);
}
#endif
#endif
#if (_SUPPORT_IO_DUT_SELECT_HVIO != FALSE) && (HVIO_DUT_SELECT == PIN_FUNC_IO_0)
static inline uint16_t Get_IO0HV(void)
{
return (l_AdcResult.u16AdcIO0HV);
}
#endif
#if (_SUPPORT_NTC != FALSE)
static inline uint16_t Get_AdcNTC(void)
{
return (l_AdcResult.u16AdcNTC - C_OADC);
}
#endif
#endif