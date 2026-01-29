#ifndef HAL_LIB_ADC_CHANNELS_H
#define HAL_LIB_ADC_CHANNELS_H
#include "../AppBuild.h"
#define C_ADC_VREF_HV C_ADC_VREF_2_50_V
#define C_ADC_VREF_MCUR C_ADC_VREF_2_50_V
#define C_ADC_IO0_HV_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_IO0_HV, .u3AdcVref = C_ADC_VREF_HV, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_IO0_LV_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_IO0_LV, .u3AdcVref = C_ADC_VREF_2_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_IO1_LV_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_IO1_LV, .u3AdcVref = C_ADC_VREF_2_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_IO2_LV_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_IO2_LV, .u3AdcVref = C_ADC_VREF_2_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_IO3_LV_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_IO3_LV, .u3AdcVref = C_ADC_VREF_2_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_IO4_LV_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_IO4_LV, .u3AdcVref = C_ADC_VREF_2_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_IO5_LV_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_IO5_LV, .u3AdcVref = C_ADC_VREF_2_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_IO6_LV_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_IO6_LV, .u3AdcVref = C_ADC_VREF_2_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_IO7_LV_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_IO7_LV, .u3AdcVref = C_ADC_VREF_2_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_POTI_IOX_EOC C_ADC_IO3_LV_EOC
#define C_ADC_RESOLVER_IOX_LV_EOC C_ADC_IO2_LV_EOC
#define C_ADC_RESOLVER_IOY_LV_EOC C_ADC_IO3_LV_EOC
#define C_ADC_NTC_IO_LV_EOC C_ADC_IO3_LV_EOC
#define C_ADC_MCUR_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_MCUR, .u3AdcVref = C_ADC_VREF_MCUR, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_MCUR_SLV1 { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_MCUR, .u3AdcVref = C_ADC_VREF_MCUR, .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV1_CMP, .u1AdcReserved = 0U } }
#define C_ADC_MCUR_SLV2 { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_MCUR, .u3AdcVref = C_ADC_VREF_MCUR, .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV2_CMP, .u1AdcReserved = 0U } }
#define C_ADC_MCUR_SLV3 { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_MCUR, .u3AdcVref = C_ADC_VREF_MCUR, .u5AdcTrigger = C_ADC_HW_TRIGGER_SLV3_CMP, .u1AdcReserved = 0U } }
#define C_ADC_MCUR_MSTR2 { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_MCUR, .u3AdcVref = C_ADC_VREF_MCUR, .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR2_CMP, .u1AdcReserved = 0U } }
#define C_ADC_MCURF_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_MCURF, .u3AdcVref = C_ADC_VREF_MCUR, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_TEMP_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_TEMP, .u3AdcVref = C_ADC_VREF_2_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_VAUX_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_VAUX, .u3AdcVref = C_ADC_VREF_2_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_VBGD_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_VBGD, .u3AdcVref = C_ADC_VREF_2_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCpls16xADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_VBGD_EOC_1V5 { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_VBGD, .u3AdcVref = C_ADC_VREF_1_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCpls16xADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_VDDA_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_VDDA, .u3AdcVref = C_ADC_VREF_2_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_VDDD_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_VDDD, .u3AdcVref = C_ADC_VREF_2_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_VS_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_VS_HV, .u3AdcVref = C_ADC_VREF_HV, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_VS_MSTR1_CMP { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_VS_HV, .u3AdcVref = C_ADC_VREF_HV, .u5AdcTrigger = C_ADC_HW_TRIGGER_MSTR1_CMP, .u1AdcReserved = 0U } }
#define C_ADC_VSMF_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_VSMF_HV, .u3AdcVref = C_ADC_VREF_HV, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_VPHU_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_PH_U_HV, .u3AdcVref = C_ADC_VREF_HV, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_VPHV_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_PH_V_HV, .u3AdcVref = C_ADC_VREF_HV, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_VPHW_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_PH_W_HV, .u3AdcVref = C_ADC_VREF_HV, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_VPHT_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_PH_T_HV, .u3AdcVref = C_ADC_VREF_HV, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_LIN_EOC { { .u2AdcMarker = C_ADC_NO_SIGN, .u5AdcChannel = C_ADC_LIN, .u3AdcVref = C_ADC_VREF_2_50_V, .u5AdcTrigger = C_ADC_HW_TRIGGER_EOCplsNxADC_CLOCK, .u1AdcReserved = 0U } }
#define C_ADC_MIN 0x000U
#define C_ADC_MAX 0x3FFU
#define C_ADC_MID ((C_ADC_MAX - C_ADC_MIN) / 2U)
#define C_ADC_HV_DIV 21U
#endif
