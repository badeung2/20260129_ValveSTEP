/*!*************************************************************************** *
 * \file        AppDefines.h
 * \brief       MLX813xx Project Application Defines file (Used by C and S files)
 *
 * \note        project MLX813xx
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-14
 *
 * \version     2.0
 *
 *
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2017-2022 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 *
 * *************************************************************************** *
 * Note: Only use "#define" statements, no "typedefs"
 * *************************************************************************** */

#ifndef _APP_DEFINES_H_

#define _APP_DEFINES_H_

/*!*************************************************************************** */
/*                               DEFINITIONS                                   */
/* *************************************************************************** */
#define FALSE                               0U                                  /*!< Definition for FALSE */
#define TRUE                                1U                                  /*!< Definition for TRUE; Should be not-FALSE */
#define UNKNOWN                             -2U                                 /*!< Definition for UNKNOWN (Special cases) */

#if !defined (NULL)
#define NULL                                0U                                  /*!< Definition for NULL */
#endif /* !defined (NULL) */

/*! ADC Start-of-Sequence (SOS) Trigger Source */
#define ADC_SOS_TRIGGER_HW                  1                                   /*!< ADC SOS Trigger by Hardware, e.g. PWM or CTimer */
#define ADC_SOS_TRIGGER_SW                  2                                   /*!< ADC SOS Trigger by Software */

/*! Application Supply Range */
#define C_APP_SUPPLY_RANGE_12V              1                                   /*!< Application Supply Range: 12V (7V ... 18V) */
#define C_APP_SUPPLY_RANGE_24V              2                                   /*!< Application Supply Range: 24V (32V) */
#define C_APP_SUPPLY_RANGE_48V              4                                   /*!< Application Supply Range: 48V (54V) */

/*! Application (_SUPPORT_APP_TYPE) */
#define C_APP_POSITIONING_ACT               0xA1U                               /*!< Application: Positioning Actuator */
#define C_APP_CONTINUOUS_ACT                0xA2U                               /*!< Application: Continuous Actuator */
#define C_APP_RELAY                         0xA3U                               /*!< Application: Relay */
#define C_APP_SOLENOID                      0xA4U                               /*!< Application: Solenoid */

/*! Bootloader */
#define BOOTLOADER_NONE                     0                                   /*!< No Boot-loader present */
#define BOOTLOADER_CAN                      1                                   /*!< CAN Boot-loader present */
#define BOOTLOADER_I2C                      2                                   /*!< I2C Boot-loader present */
#define BOOTLOADER_LIN                      3                                   /*!< LIN Boot-loader present */
#define BOOTLOADER_UART                     4                                   /*!< UART Boot-loader present */

/*! Calibration */
#define C_APP_OV_OFF                        800U                                /*!< Minimum/Offset Over-voltage level [10mV] (MMP190709-1) */
#define C_DELAY_CONST                       4084U                               /*!< Depend on Flash WS (=2) */
#define C_TEMPOFF                           60                                  /*!< Temperature range: -60 to +195 (-C_TEMPOFF to +(255-C_TEMPOFF) */

/*! Chip-package */
#define SO8                                 8                                   /*!< IC Package: SOIC 8 */
#define QFN24                               24                                  /*!< IC Package: QFN 24 */
#define QFN32                               32                                  /*!< IC Package: QFN 32 */
#define TQFP48                              48                                  /*!< IC Package: TQFP 48 */

/*! MLX81xxx EVB Section */
#define MLX8133xEVB                         0                                   /*!< EVB MLX8133x/4x */
#define MLX813xxEVB_APP                     0                                   /*!< EVB Application */
#define MLX81160EVB_RELAY                   1                                   /*!< EVB with relay's */
#define MLX81160EVB_RELAY_SINGLE_DC         1                                   /*!< Single DC-actuator */
#define MLX81160EVB_3P_3N                   2                                   /*!< EVB with each driver a single FET (3x P-FET & 3x N-FET), 2 or 3 phase */
#define MLX81160EVB_3P_3N_SINGLE_DC         1                                   /*!< Single DC-actuator */
#define MLX81160EVB_3P_3N_SINGLE_BLDC       3                                   /*!< Single (3-Phase) BLDC-actuator */
#define MLX81160EVB_6PN                     3                                   /*!< EVB with each driver a PN-FET, 2, 3, 4 or 6 phases */
#define MLX81160EVB_6PN_SINGLE_DC           1                                   /*!< Single DC-actuator */
#define MLX81160EVB_6PN_DUAL_DC             2                                   /*!< Dual DC-actuator */
#define MLX81160EVB_6PN_SINGLE_BLDC         3                                   /*!< Single (3-phase) BLDC-actuator */
#define MLX81160EVB_6PN_BIPOLAR             4                                   /*!< Single (4-Phase) bipolar-actuator */
#define MLX81160EVB_6PN_DUAL_BLDC           5                                   /*!< Dual (3-phase) BLDC-actuator */
#define MLX81160EVB_6PN_6PH_BLDC            6                                   /*!< Single (6-phase) BLDC-actuator */
#define MLX8134xEVB_100W                    10                                  /*!< BLDC (3-phase) 100W EVB, FET */
#define MLX8134xEVB_400W                    11                                  /*!< BLDC (3-phase) 400W EVB */
#define MLX81346EVB_200W                    12                                  /*!< BLDC (3-phase) 48V/200W EVB, FET */
#define MLX81346EVB_600W                    13                                  /*!< BLDC (3-phase) 48V/600W EVB, FET */
#define MLX81346EVB_ITFAN                   14                                  /*!< BLDC (3-phase) 48V/50W EVB, FET */
#define MLX81160EVB_2P_4N                   4                                   /*!< EVB with each driver a single FET (2x P-FET & 4x N-FET) */
#define MLX81160EVB_2P_4N_DUAL_RELAY        2                                   /*!< Dual Relay with P/N-FET + Safety N-FET */

/*! Non-volatile Storage */
#define C_NV_NONE                           0                                   /*!< No Non-volatile storage */
#define C_NV_EEPROM                         1                                   /*!< EEPROM Non-volatile storage */
#define C_NV_FLASH                          2                                   /*!< FLASH Non-volatile storage */
#define C_NV_EXTERNAL                       3                                   /*!< External Non-volatile storage */

/*! Usage of NVRAM or Code Constants */
#define MP_CONST                            0                                   /*!< Use fixed constants for UniROM code (Shortest code size) */
#define MP_NVM                              1                                   /*!< Use Non Volatile Memory parameters for UniROM code (Most flexible) */

/*! I/O Pin Functions */
#define PIN_FUNC_IO_NONE                    0                                   /*!< Pin-Function assigned to: None */
#define PIN_FUNC_IO_0                       1                                   /*!< Pin-Function assigned to: IO[0] */
#define PIN_FUNC_IO_1                       2                                   /*!< Pin-Function assigned to: IO[1] */
#define PIN_FUNC_IO_2                       3                                   /*!< Pin-Function assigned to: IO[2] */
#define PIN_FUNC_IO_3                       4                                   /*!< Pin-Function assigned to: IO[3] */
#if !defined (__MLX81330__) && !defined (__MLX81350__)
#define PIN_FUNC_IO_4                       5                                   /*!< Pin-Function assigned to: IO[4] */
#define PIN_FUNC_IO_5                       6                                   /*!< Pin-Function assigned to: IO[5] */
#define PIN_FUNC_IO_6                       7                                   /*!< Pin-Function assigned to: IO[6] */
#define PIN_FUNC_IO_7                       8                                   /*!< Pin-Function assigned to: IO[7] */
#if !defined (__MLX81332__) && !defined (__MLX81334__) && !defined (__MLX81339__)
#define PIN_FUNC_IO_8                       9                                   /*!< Pin-Function assigned to: IO[8] */
#define PIN_FUNC_IO_9                       10                                  /*!< Pin-Function assigned to: IO[9] */
#define PIN_FUNC_IO_10                      11                                  /*!< Pin-Function assigned to: IO[10] */
#define PIN_FUNC_IO_11                      12                                  /*!< Pin-Function assigned to: IO[11] */
#endif /* !defined (__MLX81332__) && !defined (__MLX81334__) && !defined (__MLX81339__) */
#endif /* !defined (__MLX81330__) && !defined (__MLX81350__) */
#if defined (__MLX81160__)
#define PIN_FUNC_LIN                        13                                  /*!< Pin-Function assigned to: LIN */
#define PIN_FUNC_R                          14                                  /*!< Pin-Function assigned to: R-Phase */
#define PIN_FUNC_S                          15                                  /*!< Pin-Function assigned to: S-Phase */
#elif defined (__MLX81339__)
#define PIN_FUNC_COMM                       15                                  /*!< Pin-Function assigned to: COMM */
#else  /* defined (__MLX81160__) */
#define PIN_FUNC_LIN                        15                                  /*!< Pin-Function assigned to: LIN */
#endif /* defined (__MLX81160__) */
#if defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#define PIN_FUNC_T                          16                                  /*!< Pin-Function assigned to: T-Phase */
#endif /* defined (__MLX81160__) || defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__) */
#define PIN_FUNC_U                          17                                  /*!< Pin-Function assigned to: U-Phase */
#define PIN_FUNC_V                          18                                  /*!< Pin-Function assigned to: V-Phase */
#define PIN_FUNC_W                          19                                  /*!< Pin-Function assigned to: W-Phase */
#define PIN_FUNC_IO_O_HV                    20                                  /*!< Pin-Function assigned to: HV_IO[0] */

/*! I2C Protocols (MMP211104-2) */
#define I2C_MASTER_PROT_NONE                0x00U                               /*!< I2C Master No I2C Protocol */
#define I2C_MASTER_PROT_GENERIC             0x81U                               /*!< I2C Master Generic Protocol */
#define I2C_MASTER_PROT_MEMORY              0x82U                               /*!< I2C Master Memory Device (e.g. Non Volatile Memory) (MMP211104-1) */
#define I2C_MASTER_PROT_TRIAXIS_90381       0x83U                               /*!< I2C Master Triaxis 90381 */
#define I2C_SLAVE_PROT_NONE                 0x00U                               /*!< I2C Slave No I2C Protocol */
#define I2C_SLAVE_PROT_GENERIC              0x01U                               /*!< I2C Slave Generic Protocol */
#define I2C_SLAVE_PROT_POSITIONING          0x02U                               /*!< I2C Slave Positioning Protocol (e.g. Flap & Valve's) */
#define I2C_SLAVE_PROT_CONTINUOUS           0x03U                               /*!< I2C Slave Continuous Protocol (e.g. Fans & Pumps) */
#define I2C_SLAVE_PROT_MLX_ACT              0x10U                               /*!< I2C Melexis Actuator Protocol (Positioning & Continuous) */
#define I2C_SLAVE_PROT_AS_LIN               0x20U                               /*!< I2C as LIN Protocol */

/*! I2C Master HW support */
#define C_I2C_MASTER_NONE                   0U                                  /*!< I2C Master SCK Resource based on IO & CTimer #1 (Enable _SUPPORT_I2C_ISR) (max. 100kBaud, non-atomic) */
#define C_I2C_MASTER_CTIMER1                1U                                  /*!< I2C Master SCK Resource based on CTimer #1 (max. 400kBaud, atomic) */
#define C_I2C_MASTER_PWM_MASTER2            2U                                  /*!< I2C Master SCK Resource based on PWM Master #2 (max. 400kBaud, atomic) */
#define C_I2C_MASTER_IC                     3U                                  /*!< I2C Master SCK/SDA based on IC I2C Master implementation */

/*! LIN protocols */
#define NOLIN                               (0x00U << 8)                        /*!< No LIN Protocol */
#define LIN13                               (0x13U << 8)                        /*!< LIN protocol according LIN 1.3 */
#define LIN20                               (0x20U << 8)                        /*!< LIN protocol according LIN 2.0 */
#define LIN21                               (0x21U << 8)                        /*!< LIN protocol according LIN 2.1 */
#define LIN22                               (0x22U << 8)                        /*!< LIN protocol according LIN 2.2 */
#define LIN2J                               (0x2AU << 8)                        /*!< LIN protocol according LIN 2.x/SAE-J2602 */
#define LIN2X                               (0x2FU << 8)                        /*!< LIN protocol according LIN 2.x */
#define LIN2                                (0x20U << 8)                        /*!< Any LIN 2.x protocol, including SAE-J2602 */
#define LINX                                (0xF0U << 8)                        /*!< Any LIN n.x protocol, including SAE-J2602 */
#define LINXX                               (0xFFU << 8)                        /*!< Any LIN x.x protocol */
/* Applications */
#define APP_SIMPLE_PCT                      0x10U                               /*!< 10 = Simple Percentage LIN protocol */
#define APP_SOLENOID                        0x20U                               /*!< 20 = Solenoid, e.g. valve */
#define APP_RELAY                           0x30U                               /*!< 30 = Relay, e.g. Window Lift */
#define APP_ACT_AGS                         0x4DU                               /*!< 4D = AGS (Demo) */
#define APP_HVAC49                          0x49U                               /*!< 49 = LIN LH HVAC V4.9 */
#define APP_HVAC52                          0x52U                               /*!< 52 = LIN LH HVAC V5.2 */
#define APP_HVACTB                          0x4FU                               /*!< 4F = LIN Toshiba TB905xFNG */
#define APP_AIRVENT12                       0xA1U                               /*!< A1 = MGU_STELLMOTOR_LUFTDUESE_V12 */
#define APP_ACT_AAB                         0xAAU                               /*!< AA = VW AAB LIN protocol */
#define APP_ACT_KVA                         0xABU                               /*!< AB = VW KVA (Kuehlerverblendungsaktor) LIN protocol */
#define APP_ACT_VALVE                       0xE3U                               /*!< E3 = VALVE */
#define APP_ACT_FAN_01                      0xF0U                               /*!< F0 = Fan (Melexis) V0.1 */
/*! LIN Application protocols */
#define LIN2X_AGS                           (LIN2X | APP_ACT_AGS)               /*!< LIN protocol according LIN 2.x/AGS (Demo) */
#define LIN2X_FAN01                         (LIN2X | APP_ACT_FAN_01)            /*!< LIN protocol according LIN 2.x/Fan 0.1 */
#define LIN2X_HVAC49                        (LIN2X | APP_HVAC49)                /*!< LIN protocol according HVAC Flap standard V4.9 */
#define LIN2X_HVAC52                        (LIN2X | APP_HVAC52)                /*!< LIN protocol according HVAC Flap standard V5.2 */
#define LIN2X_AIRVENT12                     (LIN2X | APP_AIRVENT12)             /*!< LIN protocol according MGU_STELLMOTOR_LUFTDUESE_V12 */
#define LIN13_HVACTB                        (LIN13 | APP_HVACTB)                /*!< LIN Protocol according HVAC Flap Toshiba TB905xFNG */
#define LIN2X_RELAY                         (LIN2X | APP_RELAY)                 /*!< LIN protocol according LIN 2.x/Relay (Demo) */
#define LIN22_SIMPLE_PCT                    (LIN22 | APP_SIMPLE_PCT)            /*!< LIN protocol according Melexis Simple Percentage */
#define LIN2X_SOLENOID                      (LIN2X | APP_SOLENOID)              /*!< LIN protocol according LIN 2.x/Solenoid (Demo) */
#define LIN2X_VALVE                         (LIN22 | APP_ACT_VALVE)             /*!< LIN protocol according LIN 2.x/Valve */
#define LIN2X_AAB                           (LIN2X | APP_ACT_AAB)               /*!< LIN protocol according LIN 2.x/VW AAB */
#define LIN2X_KVA                           (LIN2X | APP_ACT_KVA)               /*!< LIN protocol according LIN 2.2/VW KVA */
/*! LIN-AA stuff */
#define LIN_AA_RESOURCE_CTIMER0             0                                   /*!< LIN-AA ADC HW-Trigger Resource CTimer 0 ID */
#define LIN_AA_RESOURCE_CTIMER1             1                                   /*!< LIN-AA ADC HW-Trigger Resource CTimer 1 ID */
#define LIN_AA_RESOURCE_PWM_MASTER1         2                                   /*!< LIN-AA ADC HW-Trigger Resource PWM Master 1 ID */
/*! LIN Universal Diagnostics services */
#define _SUPPORT_UDS_DK1                    1                                   /*!< LIN UDS DK1 Method ID */
#define _SUPPORT_UDS_DK4                    4                                   /*!< LIN UDS DK4 Method ID */

/*! Motor */
#define SINGLE_COIL_PWM                     0                                   /*!< DC Motor PWM */
#define SINGLE_COIL_PWM_BIPOLAR             1                                   /*!< Single Coil Motor PWM */
#define BIPOLAR_FULL_STEP_BEMF              2                                   /*!< Dual Coil, first PWM and second BEMF */
#define BIPOLAR_TRIPHASE_TWOPWM_INDEPENDENT_GND  3                              /*!< Dual coil, three-phase */
#define BIPOLAR_PWM_SINGLE_INDEPENDENT_GND  4                                   /*!< Each coil single PWM mirror/inverse mirror, other HIGH *
                                                                                 * Phase 1  |     +---------------+     |  Phase 3: |----+                 +----|
                                                                                 *          |     |               |     |           |    |                 |    |
                                                                                 *          |-----+               +-----|           |    +-----------------+    |
                                                                                 * Phase 2: |                           |           |                           |
                                                                                 *          |                           |           |                           |
                                                                                 *          |---------------------------|  Phase 4:	|---------------------------|
                                                                                 *                        ^                                                     ^
                                                                                 * ADC:                  Im                                                     Im
                                                                                 *                       50%           100%                      50%           100%
                                                                                 * Note: VDS-Monitor is not possible!
                                                                                 */
#define BIPOLAR_TRIPHASE_ALLPWM_MIRROR      5                                   /*!< Dual coil, three-phase */
#define TRIPLEPHASE_ALLPWM_MIRROR           6                                   /*!< Each phase PWM mirror *
                                                                                 * Phase 1  |          +-----+          |
                                                                                 *          |          |     |          |
                                                                                 *          |----------+     +----------|
                                                                                 * Phase 2  |        +---------+        |
                                                                                 *          |        |         |        |
                                                                                 *          |--------+         +--------|
                                                                                 * Phase 3  |  +---------------------+  |
                                                                                 *          |  |                     |  |
                                                                                 *          |--+                     +--|
                                                                                 *                  ^     ^
                                                                                 * ADC:             Im   Vds
                                                                                 *                 25%   50%
                                                                                 * or                  ^             ^
                                                                                 *                    Im1           Im2
                                                                                 *               at Raising narrow  at Falling widest
                                                                                 */
#define ADC_TRIGGER_PWM_MIN_MAX             1                                   /*!< ADC Current Trigger at Minimum PWM-DC (25%-50%) & Maximum PWM-DC (75%-100%) */
#define ADC_TRIGGER_PWM_MID                 2                                   /*!< ADC Current Trigger at Medium PWM-DC (12.5%-37.5% & (62.5%-87.5%) */
#define _SUPPORT_PWM_ADC_TRIGGER_MODE       ADC_TRIGGER_PWM_MIN_MAX             /*!< Choice of ADC Current Trigger points */
#define VOLTAGE_SHAPE_NONE                  0                                   /*!< No shape (Block-mode, BEMF-mode, DC-motor) */
#define VOLTAGE_SHAPE_SINE                  1                                   /*!< Sine-shape (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR) */
#define VOLTAGE_SHAPE_SVM                   2                                   /*!< Space Vector Modulation (SVM or SVM-M) (_SUPPORT_PWM_MODE == TRIPLEPHASE_ALLPWM_MIRROR) */
#define VOLTAGE_SHAPE_DSVM_GND              3                                   /*!< Discontinue Space Vector Modulation to GND (SVM-L); GND recirculation (_SUPPORT_PWM_MODE == TRIPLEPHASE_TWOPWM_INDEPENDENT_GND) */
#define VOLTAGE_SHAPE_DSVM_SUP              4                                   /*!< Discontinue Space Vector Modulation to SUP (SVM_H); SUP recirculation */
#define VOLTAGE_SHAPE_DSVM_ALT              5                                   /*!< Discontinue Space Vector Modulation alternating to GND/SUP (SVM-LH); GND & SUP recirculation */
#define TRIPLEPHASE_TWOPWM_MIRROR_SUP       7                                   /*!< Two phase PWM, one phase steady SUPPLY */
#define TRIPLEPHASE_TWOPWM_MIRROR_GND       8                                   /*!< Two phase PWM, one phase steady LOW *
                                                                                 * Phase 1  |           +---+           |
                                                                                 *          |           |   |           |
                                                                                 *          |-----------+   +-----------|
                                                                                 * Phase 2  |         +-------+         |
                                                                                 *          |         |       |         |
                                                                                 *          |---------+       +---------|
                                                                                 * Phase 3  |                           |
                                                                                 *          |                           |
                                                                                 *          |---------------------------|
                                                                                 *                        ^  ^
                                                                                 * ADC:                 Im1  Im2 (Optional)
                                                                                 *                       50%           100%
                                                                                 * Note: VDS-Monitor is not possible!
                                                                                 */
#define TRIPLEPHASE_TWOPWM_INDEPENDENT_GND  11                                  /*!< Two phase PWM mirror/inverse mirror, one phase steady LOW *
                                                                                 * Phase 1  |           +---+           |
                                                                                 *          |           |   |           |
                                                                                 *          |-----------+   +-----------|
                                                                                 * Phase 2  |----+                 +----|
                                                                                 *          |    |                 |    |
                                                                                 *          |    +-----------------+    |
                                                                                 * Phase 3  |                           |
                                                                                 *          |                           |
                                                                                 *          |---------------------------|
                                                                                 *                        ^             ^
                                                                                 * ADC:                   Im            Im
                                                                                 *                       50%           100%
                                                                                 */
#define TRIPLEPHASE_FULL_STEP_BEMF          12                                  /*!< Triple Coil, with BEMF-sense */
#define TRIPLEPHASE_FULL_STEP               13                                  /*!< Triple Coil, Full-Step (MMP230715-2) */
#define TRIPLEPHASE_HALF_STEP_BEMF          14                                  /*!< Triple Coil, Half-Step with BEMF-sense */
#define BIPOLAR_HALF_STEP_BEMF              15                                  /*!< Dual Coil, Half-Step with BEMF-sense (MMP231017-1) */
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#define SINGLE_COIL_DUAL                    1                                   /*!< Two/Dual actuator with each a single coil (4-phase) */
#define SINGLE_COIL_PARALLEL                2                                   /*!< Single actuator with single coil, driver in parallel-mode (MLX8133x) */
#elif defined (__MLX81160__)
#define TRIPLE_COIL_DUAL                    1                                   /*!< Two/Dual actuator with each triple coil (6-phase) */
#define TRIPLE_COIL_PARALLEL                3                                   /*!< Single actuator with triple coil, driver in parallel-mode (MLX81160) */
#endif
#if defined (__MLX81330__) || defined (__MLX81332__) || defined (__MLX81334__) || defined (__MLX81339__) || defined (__MLX81350__)
#define BIPOLAR_MODE_UV_WT                  0                                   /*!< Coil between U (pin 2) & V (pin 3), and second coil between W (pin 6) & T (pin 7) */
#define BIPOLAR_MODE_UT_VW                  1                                   /*!< Coil between U (pin 2) & T (pin 7), and second coil between V (pin 3) & W (pin 6) */
#define BIPOLAR_MODE_UW_VT                  2                                   /*!< Coil between U (pin 2) & W (pin 6), and second coil between V (pin 3) & T (pin 7) */
#define BIPOLAR_MODE_UV_TW                  4                                   /*!< Coil between U (pin 2) & V (pin 3), and second coil between T (pin 7) & W (pin 6) */
#elif defined (__MLX81160__)
#define BIPOLAR_MODE_RS_TU                  0                                   /*!< Coil between R (pin 1) & S (pin 2), and second coil between T (pin 3) & U (pin 4) */
#define BIPOLAR_MODE_RU_ST                  1                                   /*!< Coil between R (pin 1) & U (pin 4), and second coil between S (pin 2) & T (pin 3) */
#define BIPOLAR_MODE_RT_SU                  2                                   /*!< Coil between R (pin 1) & T (pin 3), and second coil between S (pin 2) & U (pin 4) */
#endif
#define FOC_MODE_NONE                       0x00                                /*!< No FOC based motor operation */
#define FOC_MODE_IV                         0x01                                /*!< FOC based on IV (Current-Voltage) */
#define FOC_MODE_IB                         0x02                                /*!< FOC based on IB (Current-BEMF) */
#define FOC_MODE_IB_IPK                     0x03                                /*!< FOC based on IB with Ipk */
#define FOC_MODE_ID_IQ                      0x04                                /*!< FOC based on ID/IQ (Cartesian/Polar) */
#define FOC_MODE_SA                         0x05                                /*!< FOC based on Slope/Area */
#define FOC_MODE_MASK                       0x07                                /*!< FOC mode mask bit 2:0 */
#define FOC_OBSERVER_SENSORLESS             0x00                                /*!< FOC Observer based on Sensorless */
#define FOC_OBSERVER_SENSORED               0x80                                /*!< FOC Observer based on Sensor (e.g. 9038x Resolver) */
#define FOC_OBSERVER_MASK                   0x80                                /*!< FOC sensor mask bit 7 */
#define FOC_ID_IQ_iCLARKE                   1                                   /*!< FOC ID/IQ using inverse Clarke transformation */
#define FOC_ID_IQ_LUT                       2                                   /*!< FOC ID/IQ using look-up waveform-table (LUT) */
#define PWM_LIMIT_BY_PID                    0                                   /*!< Motor PWM Limited by PID (MMP181114-2) */
#define PWM_LIMIT_BY_PWM                    1                                   /*!< Motor PWM limited by MotorDriver (MMP181114-2) */
#define BOOST_MODE_CLIPPING                 0                                   /*!< Clip the PWM-DC at Max (Max: 108%) */
#define BOOST_MODE_WAVEFORM                 1                                   /*!< Change Waveform: Sine to Block (Max: 126%) */
#define C_PID_FACTOR                        4U                                  /*!< PID-Gain factor: (1 << 4) = 16 */
#define C_MIN_MOTORCURRENT                  10U                                 /*!< Minimum current [ADC_LSB] */

#define C_ACCELERATION_CONSTANT             0                                   /*!< Constant acceleration/deceleration */
#define C_ACCELERATION_COSINE_CURVE         1                                   /*!< Cosine (smooth) acceleration/deceleration */

/* Motor family */
#define MF_TYPE                             0xF0U                               /*!< Upper 4 bits is Family Type */
#define MF_SUBTYPE                          0x0FU                               /*!< Lower 4 bits is Sub-type */
#define MF_DC                               0x10U                               /*!< DC */
#define MF_DC_T0                            (MF_DC | 0x00U)                     /*!< DC, Pos: None */
#define MF_DC_T1                            (MF_DC | 0x01U)                     /*!< DC, Pos: 3-wire (Phase1, P, Phase2) */
#define MF_DC_T2                            (MF_DC | 0x02U)                     /*!< DC, Pos: 3-wires (3V3, P, GND) */
#define MF_DC_T3                            (MF_DC | 0x03U)                     /*!< DC, Triaxis/XY-Resolver */
#define MF_DC_T4                            (MF_DC | 0x04U)                     /*!< DC, with Hall-Latch */
#define MF_BLDC                             0x20U                               /*!< BEMF, with startup in Stepper mode */
#define MF_STEPPER                          0x30U                               /*!< Stepper */
#define MF_RELUCTANCE                       0x40U                               /*!< Reluctance motor */
#define MF_FOC                              0x50U                               /*!< FOC/IV */

#if defined (__MLX81160__)
#define ADC_CUR_SENSE_CH1                   1                                   /*!< ADC Current Sense Channel #1 */
#define ADC_CUR_SENSE_CH2                   2                                   /*!< ADC Current Sense Channel #2 */
#define ADC_CUR_SENSE_CHX                   3                                   /*!< ADC Current Sense Channel #1 or/and #2 */
#endif /* defined (__MLX81160__) */

/*! Motor Position (MMP230406-1) */
#define C_MOTOR_POS_NONE                    0U                                  /*!< No Motor Position: Continuous */
#define C_MOTOR_POS_ROTOR                   1U                                  /*!< Motor Position based on Rotor */
#define C_MOTOR_POS_SHAFT                   2U                                  /*!< Motor Position based on (outer) shaft */

/*! Motion Detector support */
#define C_MOTION_DET_NONE                   0U                                  /*!< No Motion-detector */
#define C_MOTION_DET_BEMF                   1U                                  /*!< Motion-detector based on BEMF */
#define C_MOTION_DET_SENSOR                 2U                                  /*!< Motion-detector based on sensor */

/*! Motor Rotational Direction (MMP230710-1) */
#define C_MOTOR_DIRECTION_CW                0U                                  /*!< Motor Rotational Direction: ClockWise */
#define C_MOTOR_DIRECTION_CCW               1U                                  /*!< Motor Rotational Direction: Counter-ClockWise */
#define C_MOTOR_DIRECTION_CMD               2U                                  /*!< Motor Rotational Direction: By Command */

/*! PWM Communication */
#define C_PWM_COMM_NONE                     0U                                  /*!< PWM Communication None */
#define C_PWM_COMM_CTIMER0                  1U                                  /*!< PWM Communication based on CTimer #0 */
#define C_PWM_COMM_CTIMER1                  2U                                  /*!< PWM Communication based on CTimer #1 */
#define C_PWM_COMM_IO_ISR                   3U                                  /*!< PWM Communication based on IO ISR */
#define C_PWM_COMM_PPM                      4U                                  /*!< PWM Communication based on PPM */
#define C_PWM_HW_RES_NONE                   0U                                  /*!< PWM (Sensor) based on None */
#define C_PWM_HW_RES_CTIMER1                1U                                  /*!< PWM (Sensor) based on CTimer #1 */

/*! SENT support */
#define C_SENT_NONE                         0U                                  /*!< SENT Resource based on IO (not supported) */
#define C_SENT_CTIMER1                      1U                                  /*!< SENT Resource based on CTimer #1 */
#define C_SENT_PPM                          2U                                  /*!< SENT Resource based on PPM */

/*! External Shunt [mR]; max-current ADC is 100mV/R-shunt (+45%) */
#define C_RSHUNT_R001                         1                                 /*!<   1 [mR] (minimum); Up to 100A */
#define C_RSHUNT_R002                         2                                 /*!<   2 [mR]; Up to 50A */
#define C_RSHUNT_R003                         3                                 /*!<   3 [mR]; Up to 33A */
#define C_RSHUNT_R004                         4                                 /*!<   4 [mR]; Up to 25A */
#define C_RSHUNT_R005                         5                                 /*!<   5 [mR]; Up to 20A */
#define C_RSHUNT_R010                        10                                 /*!<  10 [mR]; Up to 10A */
#define C_RSHUNT_R020                        20                                 /*!<  20 [mR]; Up to 5A */
#define C_RSHUNT_R022                        22                                 /*!<  22 [mR]; Up to 4.5A */
#define C_RSHUNT_R025                        25                                 /*!<  25 [mR]; Up to 4A */
#define C_RSHUNT_R033                        33                                 /*!<  33 [mR]; Up to 3A */
#define C_RSHUNT_R040                        40                                 /*!<  40 [mR]; Up to 2.5A */
#define C_RSHUNT_R050                        50                                 /*!<  50 [mR]; Up to 2A */
#define C_RSHUNT_R100                       100                                 /*!< 100 [mR]; Up to 1A */
#define C_RSHUNT_R200                       200                                 /*!< 200 [mR]; Up to 0.5A */
#define C_RSHUNT_R250                       250                                 /*!< 250 [mR]; Up to 0.4A */
#define C_RSHUNT_R330                       330                                 /*!< 330 [mR]; Up to 0.3A */
#define C_RSHUNT_R500                       500                                 /*!< 500 [mR]; Up to 0.2A */

/*! SPI */
#define SPI_2WIRE_IO0_1                     0                                   /*!< 2-Wire SPI by IO[0] for MOSI/MISO and IO[1] for CLOCK */
#define SPI_2WIRE_IO2_1                     1                                   /*!< 2-Wire SPI by IO[2] for MOSI/MISO and IO[1] for CLOCK (MLX81330-only) */
#define SPI_2WIRE_IO6_7                     2                                   /*!< 2-Wire SPI by IO[7] for MOSI/MISO and IO[6] for CLOCK (Not MLX81330) */
#define SPI_WORD_8BITS                      0                                   /*!< SPI Word length: 8-bits */
#define SPI_WORD_16BITS                     1                                   /*!< SPI Word length: 16-bits */
#define C_SPI_HALF_DUPLEX                   0                                   /*!< SPI Half-duplex */
#define C_SPI_FULL_DUPLEX                   1                                   /*!< SPI Full-duplex */

/*! UART Interface applications */
#define C_UART_IF_NONE                      0                                   /*!< UART-Interface not used */
#define C_UART_IF_ACTUATOR                  1                                   /*!< UART-Interface for actuator control (similar as LIN) */
#define C_UART_IF_BLUETOOTH                 2                                   /*!< UART-Interface for Bluetooth module */
#define C_UART_IF_QUAD_WHEEL_CAR            3                                   /*!< UART-Interface for Quad-Wheel-Car inter-module communication */
#define C_UART_IF_SCOPE                     4                                   /*!< UART-Interface for scope */
#define C_UART_IF_TERMINAL                  5                                   /*!< UART-Interface for Terminal */
#define C_UART_IF_MLX_ACT                   6                                   /*!< UART-Interface for Melexis Actuator (similar as I2C) */
#define C_UART_IF_DEBUG                     15                                  /*!< UART-Interface for IC debug/testing */
/*! UART Scope modes */
#define NONE_SCOPE                          0                                   /*!< No scope */
#define TINY_SCOPE                          1                                   /*!< Tiny Scope: Single 16-bit channel */
#define DUAL_SCOPE                          2                                   /*!< Dual Scope: Dual 16-bit channels */
#define QUAD_SCOPE                          4                                   /*!< Quad Scope: Quad 16-bit channels */
#define CH12_SCOPE                          12                                  /*!< 12-Channel Scope: 12x 8-bit channels */

/*! STimer Clock Source Selection */
#define C_STIMER_CLKSRC_DEFAULT             0                                   /*!< STimer Clock Source: Default */
#define C_STIMER_CLKSRC_CPU                 1                                   /*!< STimer Clock Source: CPU RCO32MHz */
#define C_STIMER_CLKSRC_1MHz                2                                   /*!< STimer Clock Source: RCO1MHz */
#define C_STIMER_CLKSRC_10kHz               3                                   /*!< STimer Clock Source: RCO10kHz */

/*! PID Controller Period units */
#define C_PID_PERIOD_UNIT_TIME              0                                   /*!< PID Period based on time (constant) */
#define C_PID_PERIOD_UNIT_SPEED             1                                   /*!< PID Period based on speed (variable) */

/*! Support various triple hall-Latch switching schemes (MMP230705-2) */
#define C_3HL_SWITCH_MODE_SAME_EDGE         0                                   /*!< Hall-Latch Switch-mode: "3-same" edge */
#define C_3HL_SWITCH_MODE_ALT1_EDGE         1                                   /*!< Hall-Latch Switch-mode: Alternating edge */
#define C_3HL_SWITCH_MODE_ALT2_EDGE         2                                   /*!< Hall-Latch Switch-mode: Alternating edge */

/* NOTE: Don't use below marco's with local stack-variables; The C-compiler doesn't "see" the psup/pop
 * It's better to use ATOMIC_CODE() */
#define BEGIN_CRITICAL_SECTION()            __asm__ ("psup #0")                 /*!< Set to priority 0 to block all interrupts */
#define END_CRITICAL_SECTION()              __asm__ ("pop M")                   /*!< Restore priority */

/*! coPRO Interface */
#define C_COPRO_IF_NONE                     0                                   /*!< coPRO Interface: None */
#define C_COPRO_IF_API                      1                                   /*!< coPRO Interface: API */
#define C_COPRO_IF_MAILBOX                  2                                   /*!< coPRO Interface: MailBox */

#endif /* _APP_DEFINES_H_ */

/* EOF */

