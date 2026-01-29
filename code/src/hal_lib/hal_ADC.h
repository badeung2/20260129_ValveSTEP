/*!************************************************************************** *
 * \file        hal_ADC.h
 * \brief       Hardware Abstraction Layer for ADC handling
 *
 * \note        project MLX81160/33x/34x/35x
 *
 * \author      Marcel Braat
 *
 * \date        2024-03-22
 *
 * \version     2.0
 *
 *
 * MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2024-2024 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * ************************************************************************** */

#ifndef HAL_LIB_ADC_H
#define HAL_LIB_ADC_H

/*!************************************************************************** */
/*                          INCLUDES                                          */
/* ************************************************************************** */
#include "../AppBuild.h"                                                        /* Application Build */

#include "hal_ADC_channels.h"                                                   /* ADC channels support */
#include "hal_ADC_inline.h"                                                     /* ADC inline function support */

/*!************************************************************************** */
/*                          DEFINITIONS                                       */
/* ************************************************************************** */
#define C_ADC_SBASE_LEN     22U                                                 /*!< ADC SBASE Array length */

/*! ADC Modes */
#define C_ADC_MODE_OFF      0U                                                  /*!< ADC-mode: OFF */
#define C_ADC_MODE_IDLE     1U                                                  /*!< ADC-mode: IDLE */
#define C_ADC_MODE_RUN_HW   2U                                                  /*!< ADC-mode: ACTIVE HW-Trigger */
#define C_ADC_MODE_LINAA    3U                                                  /*!< ADC-mode: LIN-AA */
#define C_ADC_MODE_MOV_DET  4U                                                  /*!< ADC-mode: Movement Detector */
#define C_ADC_MODE_WINDMILL 5U                                                  /*!< ADC-mode: Wind-milling Detector */

#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
#define C_ADC_SETTLING_TIME     5U                                              /*!< ADC Settling time [us] */
#elif defined (__MLX81339__) || defined (__MLX81350__)
#define C_ADC_SETTLING_TIME     2U                                              /*!< ADC Settling time [us] */  /* TODO[MMP39/MMP50] */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_ADC_SETTLING_TIME     2U                                              /*!< ADC Settling time [us] */
#endif

/* Copy from ErrorCodes.h */
#define C_HAL_ERR_NONE          0U
#define C_HAL_ERR_ADC           1U
#define C_HAL_ERR_TO_ADC_STOP   2U

/* ADC Conversion */
/* IC Temperature Sensor Gain-divider */
#if defined (__MLX81339__) || defined (__MLX81350__)
#define C_GTEMP_SDIV        10U                                                 /*!< Temperature gain shift-divider: (1U << 10) */
#else  /* defined (__MLX81339__) || defined (__MLX81350__) */
#define C_GTEMP_SDIV        7U                                                  /*!< Temperature gain shift-divider: (1U << 7) (MMP221101-1) */
#endif /* defined (__MLX81339__) || defined (__MLX81350__) */
#define C_GTEMP_DIV         (1U << C_GTEMP_SDIV)                                /*!< Temperature gain divider */

/* High and Low Voltage Gain-divider */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__)
#define C_VOLTGAIN_SDIV     5U                                                  /*!< Supply gain (filter) divider [10mV]: (1U << 5) */
#define C_LVOLTGAIN_SDIV    6U                                                 /*!< LV gain divider [1mV]: (1U << 6) */
#elif defined (__MLX81339__) || defined (__MLX81350__)
#define C_VOLTGAIN_SDIV     8U                                                  /*!< Supply gain (filter) divider [10mV]: (1U << 8) */
#define C_LVOLTGAIN_SDIV    8U                                                  /*!< LV gain (filter) divider [1mV]: (1U << 8) */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#define C_VOLTGAIN_SDIV     9U                                                  /*!< Supply gain (filter) divider [10mV]: (1U << 9) */
#define C_LVOLTGAIN_SDIV    9U                                                  /*!< LV gain divider [1mV]: (1U << 9) */
#endif
#define C_VOLTGAIN_DIV      (1U << C_VOLTGAIN_SDIV)                             /*!< Supply gain (filter) divider [10mV] */
#define C_LVOLTGAIN_DIV     (1U << C_LVOLTGAIN_SDIV)                            /*!< LV gain (filter) divider [1mV] */

/* Motor Current Gain-divider */
#if defined (__MLX81330__)
#define C_GMCURR_SDIV       6U                                                  /*!< Motor Current gain (filter) shift-divider [mA]: (1U << 6) (MMP221101-1) */
#define C_GMCURR_DIV        (1U << C_GMCURR_SDIV)                               /*!< Motor Current gain (filter) divider [mA] */
#elif defined (__MLX81332__) || defined (__MLX81334__)
#define C_GMCURR_SDIV       5U                                                  /*!< Motor Current gain (filter) shift-divider [mA]: (1U << 5) (MMP221101-1) */
#define C_GMCURR_DIV        (1U << C_GMCURR_SDIV)                               /*!< Motor Current gain (filter) divider [mA] */
#elif defined (__MLX81339__) || defined (__MLX81350__)
#define C_GMCURR_SDIV       8U                                                  /*!< Motor Current gain (filter) shift-divider [mA]: (1U << 8) (MMP221101-1) */
#define C_GMCURR_DIV        (1U << C_GMCURR_SDIV)                               /*!< Motor Current gain (filter) divider [mA] */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
#if (C_ONE_DIV_RSHUNT < (65536 / 148))
#define C_GMCURR_DIV        512U                                                /*!< Motor-current gain divider [mA] */
#define C_GMCURR_DIV_HG     1024U                                               /*!< Motor-current gain divider (High Gain) [mA] */
#elif (C_ONE_DIV_RSHUNT < (65536 / 148) * 2U)
#define C_GMCURR_DIV        (512U / 2U)                                         /*!< Motor-current gain divider [mA] */
#elif (C_ONE_DIV_RSHUNT < (65536 / 148) * 4U)
#define C_GMCURR_DIV        (512U / 4U)                                         /*!< Motor-current gain divider [mA] */
#else
#error "Invalid Shunt resistance (too low)"
#endif
#endif

/*!*************************************************************************** */
/*                           GLOBAL VARIABLES                                  */
/* *************************************************************************** */
#pragma space dp
#pragma space none

#pragma space nodp                                                              /* __NEAR_SECTION__ */
extern uint8_t l_u8AdcMode;                                                     /*!< ADC Mode */
extern uint16_t l_au16AdcSource[C_ADC_SBASE_LEN];                               /*!< Generic ADC HW-Trigger buffer */
#pragma space none                                                              /* __NEAR_SECTION__ */

/*!*************************************************************************** */
/*                           GLOBAL FUNCTIONS                                  */
/* *************************************************************************** */
extern void HAL_ADC_StopSafe(void);                                             /*!< Stop ADC (Safe) */
extern void HAL_ADC_PowerOff(void);                                             /*!< Power off ADC */
extern void HAL_ADC_PowerOn(void);                                              /*!< Power on ADC */
extern uint16_t HAL_ADC_StartSoftTrig(uint16_t u16NextState);                   /*!< Start ADC by using Software Trigger */

extern void HAL_ADC_Conv_Init(void);                                            /*!< ADC Conversion initialisation */
#if (_SUPPORT_ADC_REF_HV_CALIB != FALSE)
extern void HAL_ADC_ConvHvGain( uint16_t u16Mul, uint16_t u16Div);              /*!< ADC Conversion HV-channel */
#endif /* (_SUPPORT_ADC_REF_HV_CALIB != FALSE) */
extern uint16_t HAL_ADC_Conv_Vsupply(uint16_t u16TempJ, uint16_t u16SupplyVoltage);  /*!< Convert Supply Voltage [ADC-LSB] in [10mV] */
extern uint16_t HAL_ADC_Conv_Vmotor(uint16_t u16TempJ, uint16_t u16MotorVoltage);  /*!< Convert Motor Voltage [ADC-LSB] in [10mV] */
extern int16_t HAL_ADC_Conv_TempJ(uint16_t u16TempJ, uint16_t u16Init);         /*!< Convert IC Junction Temperature [ADC-LSB] in [C] */
extern uint16_t HAL_ADC_Conv_Cmotor(uint16_t u16TempJ, uint16_t u16MotorCurrent);  /*!< Convert Motor Current [ADC-LSB] in [mA] */

/*!*************************************************************************** *
 * Get_SupplyVoltage
 * \brief   Get variable l_u16SupplyVoltage
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16SupplyVoltage
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_SupplyVoltage(void)
{
    extern volatile uint16_t l_u16SupplyVoltage;
    return (l_u16SupplyVoltage);
} /* End of Get_SupplyVoltage() */

/*!*************************************************************************** *
 * Get_MotorVoltage
 * \brief   Get variable l_u16MotorVoltage
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16MotorVoltage
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_MotorVoltage(void)
{
    extern volatile uint16_t l_u16MotorVoltage;
    return (l_u16MotorVoltage);
} /* End of Get_MotorVoltage() */

/*!*************************************************************************** *
 * Get_HighVoltGain
 * \brief   Get variable l_u16HighVoltGain
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16VoltGain
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_HighVoltGain(void)
{
    extern uint16_t l_u16HighVoltGain;
    return (l_u16HighVoltGain);
} /* End of Get_HighVoltGain() */

/*!*************************************************************************** *
 * Get_HighVoltOffset
 * \brief   Get variable l_u16HighVoltOffset
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16HighVoltOffset
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_HighVoltOffset(void)
{
    extern uint16_t l_u16HighVoltOffset;
    return (l_u16HighVoltOffset);
} /* End of Get_HighVoltOffset() */

/*!*************************************************************************** *
 * Set_HighVoltOffset
 * \brief   Set variable l_u16HighVoltOffset
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Value
 * \return  -
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline void Set_HighVoltOffset(uint16_t u16Value)
{
    extern uint16_t l_u16HighVoltOffset;
    l_u16HighVoltOffset = u16Value;
} /* End of Set_HighVoltOffset() */

/*!*************************************************************************** *
 * Get_LowVoltOffset
 * \brief   Get variable l_u16LowVoltOffset
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16LowVoltOffset
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_LowVoltOffset(void)
{
    extern uint16_t l_u16LowVoltOffset;
    return (l_u16LowVoltOffset);
} /* End of Get_LowVoltOffset() */

/*!*************************************************************************** *
 * Get_MotorVoltGainF
 * \brief   Get variable l_u16VoltGainFiltered
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16VoltGainFiltered
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_MotorVoltGainF(void)
{
    extern uint16_t l_u16VsmGain;
    return (l_u16VsmGain);
} /* End of Get_MotorVoltGainF() */

#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
/*!*************************************************************************** *
 * Get_VsmOffset
 * \brief   Get variable l_i16VsmOffset
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  int16_t l_i16VsmOffset
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline int16_t Get_VsmOffset(void)
{
    extern int16_t l_i16VsmOffset;
    return (l_i16VsmOffset);
} /* End of Get_VsmOffset() */

/*!*************************************************************************** *
 * Set_VsmOffset
 * \brief   Set variable l_u16VsmOffset
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Value: VSM-Offset [ADC_LSB]
 * \return  -
 * *************************************************************************** *
 * \details in-line function to set 'local' variable
 * *************************************************************************** */
static inline void Set_VsmOffset(int16_t i16Value)
{
    extern int16_t l_i16VsmOffset;
    l_i16VsmOffset = i16Value;
} /* End of Set_VsmOffset() */
#elif defined (__MLX81160__) || defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)
/*!*************************************************************************** *
 * Get_VsmOffset
 * \brief   Get variable l_u16VsmOffset
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16VsmOffset
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_VsmOffset(void)
{
    extern uint16_t l_u16VsmOffset;
    return (l_u16VsmOffset);
} /* End of Get_VsmOffset() */

/*!*************************************************************************** *
 * Set_VsmOffset
 * \brief   Set variable l_u16VsmOffset
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Value: VSM-Offset [ADC_LSB]
 * \return  -
 * *************************************************************************** *
 * \details in-line function to set 'local' variable
 * *************************************************************************** */
static inline void Set_VsmOffset(uint16_t u16Value)
{
    extern uint16_t l_u16VsmOffset;
    l_u16VsmOffset = u16Value;
} /* End of Set_VsmOffset() */
#endif

/*!*************************************************************************** *
 * Get_ChipTemperature
 * \brief   Get variable l_i16ChipTemperature
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_i16ChipTemperature
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline int16_t Get_ChipTemperature(void)
{
    extern volatile int16_t l_i16ChipTemperature;
    return (l_i16ChipTemperature);
} /* End of Get_ChipTemperature() */

/*!*************************************************************************** *
 * Get_TempMidADC
 * \brief   Get variable l_u16TempMidADC
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16TempMidADC
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_TempMidADC(void)
{
    extern uint16_t l_u16TempMidADC;
    return (l_u16TempMidADC);
} /* End of Get_TempMidADC() */

/*!*************************************************************************** *
 * Get_MotorCurrent_mA
 * \brief   Get variable l_u16MotorCurrent_mA
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16MotorCurrent_mA
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_MotorCurrent_mA(void)
{
    extern volatile uint16_t l_u16MotorCurrent_mA;                              /* Motor Current */
    return (l_u16MotorCurrent_mA);
} /* End of Get_MotorCurrent_mA() */

/*!*************************************************************************** *
 * Get_MCurrGain
 * \brief   Get variable l_u16MCurrGain
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16MCurrGain
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_MCurrGain(void)
{
#if defined (__MLX81160__)
    extern uint16_t l_u16MCurrGain1;
    return (l_u16MCurrGain1);
#else  /* defined (__MLX81160__) */
    extern uint16_t l_u16MCurrGain;
    return (l_u16MCurrGain);
#endif /* defined (__MLX81160__) */
} /* End of Get_MCurrGain() */

#if (_SUPPORT_ADC_VBOOST != FALSE) && (defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__))
/*!*************************************************************************** *
 * Get_VboostGain
 * \brief   Get variable l_u16VboostGain
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16VboostGain
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_VboostGain(void)
{
    extern uint16_t l_u16VboostGain;
    return (l_u16VboostGain);
} /* End of Get_VboostGain() */

/*!*************************************************************************** *
 * Get_VboostOffset
 * \brief   Get variable l_u16VboostOffset
 * \author  mmp
 * *************************************************************************** *
 * \param   -
 * \return  uint16_t l_u16VboostOffset
 * *************************************************************************** *
 * \details in-line function to get 'local' variable
 * *************************************************************************** */
static inline uint16_t Get_VboostOffset(void)
{
    extern uint16_t l_u16VboostOffset;
    return (l_u16VboostOffset);
} /* End of Get_VboostOffset() */
#endif /* (_SUPPORT_ADC_VBOOST != FALSE) && (defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)) */

#endif /* HAL_LIB_ADC_H */

/* EOF */
