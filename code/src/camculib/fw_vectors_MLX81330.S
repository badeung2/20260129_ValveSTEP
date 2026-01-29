;*!*************************************************************************** *
;* \file        fw_vectors_MLX81330.S
;* \brief       MLX81330 Interrupt vector table
;*
;* \note        project MLX81330
;*
;* \author      Marcel Braat
;*
;* \date        2017-08-15
;*
;* \version     2.0
;*
;*
;* \copyright   MELEXIS Microelectronic Integrated Systems
;*
;* Copyright (C) 2018-2022 Melexis N.V.
;* The Software is being delivered 'AS IS' and Melexis, whether explicitly or
;* implicitly, makes no warranty as to its Use or performance.
;* The user accepts the Melexis Firmware License Agreement.
;*
;* Melexis confidential & proprietary
;*
;* ************************************************************************** */
;*
;* Helper macros to place vector to vector table
;*
;*  The format is:
;*
;*     JMPVECTOR (offset, function)
;*     -or-
;*     CALLVECTOR (offset, function, priority)
;*
;*
;*  where <offset> is an offset of the vector, the <function> is the
;*  interrupt processing function and <priority> is the absolute
;*  priority at which the interrupt is to be processed.
;*
;* NOTE : To reduce possible errors this version of JMPVECTOR and CALLVECTOR macros
;*        doesn't use compiler's include <vectors.h> and VECTORS_BASE.
;*        Thus address of the vector should be specified as offset from the start
;*        of the .vectors section
;* ************************************************************************** *
;
#include "AppSwitches.h"                                                        /* Application Switches for C and S files */

#define USER_MODE                           (1 << 3)                            /*!< User-mode mask */

/* Define check; GNU S-assembler doesn't check; None defined defines are handled as defined '0' (==FALSE) (MMP240306-1) */
#if !defined(CAN_COMM) || \
    ((CAN_COMM != FALSE) && !defined(_SUPPORT_CAN_IRQ_IO)) || \
    !defined(_DEBUG_IO_7_0_ISR) || \
    !defined(LIN_COMM) || \
    !defined(PWM_COMM) || \
    ((PWM_COMM != FALSE) && !defined(_SUPPORT_PWM_COMM_ISR)) || \
    ((PWM_COMM != FALSE) && !defined(_SUPPORT_PWM_HW_RESOURCE)) || \
    !defined(_SUPPORT_DIAG_DRV_PROT) || \
    !defined(_SUPPORT_DIAG_OC) || \
    !defined(_SUPPORT_DIAG_OT) || \
    !defined(_SUPPORT_DIAG_VDDA) || \
    !defined(_SUPPORT_DIAG_VDDAF) || \
    !defined(_SUPPORT_DIAG_VDS) || \
    !defined(_SUPPORT_DUAL_HALL_LATCH_MLX92251) || \
    !defined(_SUPPORT_DUAL_HALL_LATCH_MLX92255) || \
    !defined(_SUPPORT_NV_EMERGENCY_STORE) || \
    !defined(_SUPPORT_FOC_MODE) || \
    !defined(_SUPPORT_HALL_LATCH) || \
    ((_SUPPORT_HALL_LATCH != FALSE) && !defined(_SUPPORT_HLA_IO)) || \
    ((_SUPPORT_HALL_LATCH != FALSE) && !defined(_SUPPORT_NR_OF_HL)) || \
    !defined(_SUPPORT_HALL_LATCH_MLX9227x) || \
    !defined(_SUPPORT_I2C) || \
    ((_SUPPORT_I2C != FALSE) && !defined(_SUPPORT_I2C_ERR_ISR)) || \
    ((_SUPPORT_I2C != FALSE) && !defined(_SUPPORT_I2C_RESET_ISR)) || \
    !defined(_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513) || \
    !defined(_SUPPORT_PRESSURE_MLX90829) || \
    !defined(_SUPPORT_PWM_MODE) || \
    !defined(_SUPPORT_PWM_SYNC) || \
    !defined(_SUPPORT_RINNEN) || \
    !defined(_SUPPORT_SENSOR_PWM_IO) || \
    !defined(_SUPPORT_SENSOR_SENT_IO) || \
    !defined(_SUPPORT_SENT_HW_RESOURCE) || \
    !defined(_SUPPORT_SPI_ISR) || \
    !defined(_SUPPORT_STALLDET_BRI) || \
    !defined(_SUPPORT_STALLDET_BZC) || \
    !defined(_SUPPORT_START_TO_RUN) || \
    !defined(_SUPPORT_TRIAXIS_MLX90367) || \
    !defined(_SUPPORT_TRIAXIS_MLX90372) || \
    !defined(_SUPPORT_TRIAXIS_MLX90377) || \
    !defined(_SUPPORT_TRIAXIS_MLX90421) || \
    !defined(_SUPPORT_TRIAXIS_MLX90422) || \
    !defined(_SUPPORT_TRIAXIS_MLX90425) || \
    !defined(_SUPPORT_TRIAXIS_MLX90426)
#error "ERROR: One of the used defines is NOT defined"
#endif

#define JMPVECTOR(addr,lbl)     JVECTOR (addr), _##lbl
#define JMPUVECTOR(addr,lbl)    JUVECTOR (addr), _##lbl
#define CALLVECTOR(addr,lbl,pr) CVECTOR (addr), _##lbl, (pr)
#define CALLFATAL(addr)         CFATAL  (addr)
#define JMPUFATAL(addr)         JUFATAL  (addr)

    .macro JVECTOR name, lbl
    .org    \name, 0x0
    mov UPR, #0x0
    .word   0x76DA                          ; Opcode of JMPF to evade linker relaxation
    .word   \lbl(addr17)                    ; addr17 to get word address [16:1] of the label
    .endm

    .macro JUVECTOR name, lbl               ; MMP200120-1
    .org    \name, 0x0
    mov UPR, #USER_MODE                     ; USER mode with priority 0
    .word   0x76DA                          ; Opcode of JMPF to evade linker relaxation
    .word   \lbl(addr17)                    ; addr17 to get word address [16:1] of the label
    .endm

    .macro CVECTOR name, lbl, priority
    .org    \name, 0x0
#ifdef REMAP_FP0_2_FLASH
    psup    #\priority
#else
    mov     UPR, #\priority
#endif
    .word   0x76DA                          ; JMPF but used as word to evade relaxation
    .word   \lbl(addr17)
    .endm

    .macro CFATAL name
    .org    \name, 0x0
    mov UPR, #0x0
    mov     YL, #\name/8
    .word   0x76DA                          ; JMPF but used as word to evade relaxation
    .word   _fw_fatal(addr17)
    .endm

    .macro JUFATAL name
    .org    \name, 0x0
    mov UPR, #USER_MODE                     ; USER mode with priority 0
    mov     YL, #\name/8
    .word   0x76DA                          ; JMPF but used as word to evade relaxation
    .word   _fw_fatal(addr17)
    .endm

;-------------------------------------------------------------
; Interrupt vector table
    .section .fw_vectors,"xa",@progbits
    .global __fw_vectors

;   Vector     ISR                      CPU prio    Allowed  Vector
;   offset     Handler                  within ISR  SW prio  name
; ---------------------------------------------------------------------------------------------------------------------------------------
__fw_vectors:
    JMPVECTOR  (0x0000, fw_start)           ; 00 [0, 0] MLX16_RESET Reset
    .org 0x0006
    .word 0xA55A ^ PROJECT_ID               ; (FW_FLASH_KEY)
    JMPUVECTOR (0x0008, fw_stack_it)        ; 01 [0, 0] MLX16_STACKERR Stack error (MMP200120-1)
    JMPUFATAL  (0x0010)                     ; 02 [0, 0] MLX16_PROTERR Protection error (MMP200120-1)
    JMPUFATAL  (0x0018)                     ; 03 [0, 0] MLX16_MEMERR Memory error (MMP200120-1)
    JMPUVECTOR (0x0020, fw_mlx16_operr)     ; 04 [0, 0] MLX16_OPERR Operation error (MMP200120-1)
    .org 0x0026
    .word BIST_PAGE_COUNT                   ;          (FW_FLASH_BIST_PAGES_COUNT)
    JMPUFATAL  (0x0028)                     ; 05 [0, 0] unused_5 (MMP200120-1)
    CALLFATAL  (0x0030)                     ; 06 [1, 1] MLX16_EXCHG Exchange (PTC and Debugger_fatal)
    CALLFATAL  (0x0038)                     ; 07 [1, 1] MLX16_DMAERR DMA error
#if (_SUPPORT_RINNEN != FALSE)
    CALLFATAL  (0x0040)                     ; 07 [1, 1] AWD_ATT Absolute watchdog attention
#else  /* (_SUPPORT_RINNEN != FALSE) */
    CALLVECTOR (0x0040, ISR_AWD_ATT, 0)     ; 08 [1, 1] AWD_ATT Absolute watchdog attention
#endif /* (_SUPPORT_RINNEN != FALSE) */
    CALLFATAL  (0x0048)                     ; 09 [1, 1] IWD_ATT Intelligent watchdog attention
#if defined (__MLX81330A01__) || defined (__MLX81330A02__)
    CALLFATAL  (0x0050)                     ; 0A [1, 1] EE_SH_ECC
    CALLFATAL  (0x0058)                     ; 0B [1, 1] FL_SH_ECC
#else
    CALLFATAL  (0x0050)                     ; 0A [1, 1] FL_SH_ECC
    CALLFATAL  (0x0058)                     ; 0B [1, 1] EE_SH_ECC
#endif

#if (_SUPPORT_DIAG_VDDA != FALSE) && (_SUPPORT_RINNEN == FALSE)
    CALLVECTOR (0x0060, ISR_UV_VDDA, 0)     ; 0C [1, 1] UV_VDDA
#else  /* (_SUPPORT_DIAG_VDDA != FALSE) */
    CALLFATAL  (0x0060)                     ; 0C [1, 1] UV_VDDA
#endif /* (_SUPPORT_DIAG_VDDA != FALSE) */

#if (_SUPPORT_RINNEN != FALSE)
    CALLFATAL  (0x0068)                     ; 0D [1, 1] UV_VS
#else  /* (_SUPPORT_RINNEN != FALSE) */
    CALLVECTOR (0x0068, ISR_UV_VS, 0)       ; 0D [1, 1] UV_VS
#endif /* (_SUPPORT_RINNEN != FALSE) */

#if (_SUPPORT_DIAG_VDDAF != FALSE) && (_SUPPORT_RINNEN == FALSE)
    CALLVECTOR (0x0070, ISR_UV_VDDAF, 0)    ; 0E [1, 1] UV_VDDAF
#else  /* (_SUPPORT_DIAG_VDDAF != FALSE) */
    CALLFATAL  (0x0070)                     ; 0E [1, 1] UV_VDDAF
#endif /* (_SUPPORT_DIAG_VDDAF != FALSE) */

    CALLFATAL  (0x0078)                     ; 0F [2, 1] ANA_PLL_ERR

#if (_SUPPORT_DIAG_OT != FALSE) && (_SUPPORT_RINNEN == FALSE)
    CALLVECTOR (0x0080, ISR_OT, 1)          ; 10 [2, 1] OVT
#else  /* (_SUPPORT_DIAG_OT != FALSE) */
    CALLFATAL  (0x0080)                     ; 10 [2, 1] OVT
#endif /* (_SUPPORT_DIAG_OT != FALSE) */

#if (_SUPPORT_DIAG_OC != FALSE) && (_SUPPORT_RINNEN == FALSE)
    CALLVECTOR (0x0088, ISR_OC, 1)          ; 11 [2, 1] OVC
#else  /* (_SUPPORT_DIAG_OC != FALSE) */
    CALLFATAL  (0x0088)                     ; 11 [2, 1] OVC
#endif /* (_SUPPORT_DIAG_OC != FALSE) */

#if (_SUPPORT_DIAG_VDS != FALSE)
    CALLVECTOR (0x0090, ISR_VDS, 1)         ; 12 [2, 1] OV_HS_VDS0
    CALLVECTOR (0x0098, ISR_VDS, 1)         ; 13 [2, 1] OV_HS_VDS1
    CALLVECTOR (0x00A0, ISR_VDS, 1)         ; 14 [2, 1] OV_HS_VDS2
    CALLVECTOR (0x00A8, ISR_VDS, 1)         ; 15 [2, 1] OV_HS_VDS3
    CALLVECTOR (0x00B0, ISR_VDS, 1)         ; 16 [2, 1] OV_LS_VDS0
    CALLVECTOR (0x00B8, ISR_VDS, 1)         ; 17 [2, 1] OV_LS_VDS1
    CALLVECTOR (0x00C0, ISR_VDS, 1)         ; 18 [2, 1] OV_LS_VDS2
    CALLVECTOR (0x00C8, ISR_VDS, 1)         ; 19 [2, 1] OV_LS_VDS3
#else  /* (_SUPPORT_DIAG_VDS != FALSE) */
    CALLFATAL  (0x0090)                     ; 12 [2, 1] OV_HS_VDS0
    CALLFATAL  (0x0098)                     ; 13 [2, 1] OV_HS_VDS1
    CALLFATAL  (0x00A0)                     ; 14 [2, 1] OV_HS_VDS2
    CALLFATAL  (0x00A8)                     ; 15 [2, 1] OV_HS_VDS3
    CALLFATAL  (0x00B0)                     ; 16 [2, 1] OV_LS_VDS0
    CALLFATAL  (0x00B8)                     ; 17 [2, 1] OV_LS_VDS1
    CALLFATAL  (0x00C0)                     ; 18 [2, 1] OV_LS_VDS2
    CALLFATAL  (0x00C8)                     ; 19 [2, 1] OV_LS_VDS3
#endif /* (_SUPPORT_DIAG_VDS != FALSE) */

    CALLVECTOR (0x00D0, ISR_STIMER, 5)      ; 1A [3, 6] STIMER Simple Timer interrupt
    CALLFATAL  (0x00D8)                     ; 1B [3, 6] CTIMER0_1 Complex Timer interrupt 1
    CALLFATAL  (0x00E0)                     ; 1C [3, 6] CTIMER0_2 Complex Timer interrupt 2
    CALLVECTOR (0x00E8, ISR_CTIMER0_3, 3)   ; 1D [3, 6] CTIMER0_3 Complex Timer interrupt 3
#if (_SUPPORT_START_TO_RUN != FALSE)
    CALLFATAL  (0x00F0)                     ; 1E [3, 6] CTIMER1_1 Complex Timer interrupt 1
    CALLFATAL  (0x00F8)                     ; 1F [3, 6] CTIMER1_2 Complex Timer interrupt 2
    CALLVECTOR (0x0100, ISR_CTIMER1_3, 3)   ; 20 [3, 6] CTIMER1_3 Complex Timer interrupt 3
#elif (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || \
      (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
      (_SUPPORT_HALL_LATCH_MLX9227x != FALSE) || \
      ((_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL >= 2))
    CALLVECTOR (0x00F0, ISR_CTIMER1_1, 3)   ; 1E [3, 6] CTIMER1_1 Complex Timer interrupt 1
    CALLVECTOR (0x00F8, ISR_CTIMER1_2, 3)   ; 1F [3, 6] CTIMER1_2 Complex Timer interrupt 2
    CALLVECTOR (0x0100, ISR_CTIMER1_3, 3)   ; 20 [3, 6] CTIMER1_3 Complex Timer interrupt 3
#elif (((_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
        (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        (_SUPPORT_PRESSURE_MLX90829 != FALSE)) && (_SUPPORT_SENT_HW_RESOURCE == C_SENT_CTIMER1)) || \
      ((_SUPPORT_TRIAXIS_MLX90377 != FALSE) && (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_NONE) && (_SUPPORT_SENSOR_PWM_IO != PIN_FUNC_IO_NONE)) || \
      (((_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90425 != FALSE)) && (_SUPPORT_PWM_HW_RESOURCE == C_PWM_HW_RES_CTIMER1))
    CALLVECTOR (0x00F0, ISR_CTIMER1_1, 3)   ; 1E [3, 6] CTIMER1_1 Complex Timer interrupt 1
    CALLFATAL  (0x00F8)                     ; 1F [3, 6] CTIMER1_2 Complex Timer interrupt 2
    CALLFATAL  (0x0100)                     ; 20 [3, 6] CTIMER1_3 Complex Timer interrupt 3
#elif (PWM_COMM != FALSE) && (_SUPPORT_PWM_COMM_ISR == C_PWM_COMM_CTIMER1)
    CALLVECTOR (0x00F0, ISR_CTIMER1_1, 4)   ; 1E [3, 6] CTIMER1_1 Complex Timer interrupt 1
    CALLVECTOR (0x00F8, ISR_CTIMER1_2, 4)   ; 1F [3, 6] CTIMER1_2 Complex Timer interrupt 2
    CALLVECTOR (0x0100, ISR_CTIMER1_3, 4)   ; 20 [3, 6] CTIMER1_3 Complex Timer interrupt 3
#else
    CALLFATAL  (0x00F0)                     ; 1E [3, 6] CTIMER1_1 Complex Timer interrupt 1
    CALLFATAL  (0x00F8)                     ; 1F [3, 6] CTIMER1_2 Complex Timer interrupt 2
    CALLFATAL  (0x0100)                     ; 20 [3, 6] CTIMER1_3 Complex Timer interrupt 3
#endif
#if (_SUPPORT_SPI_ISR != FALSE)
    CALLVECTOR (0x0108, ISR_SPI_TE, 5)      ; 21 [3, 6] SPI_TE SPI transmit register is empty
    CALLVECTOR (0x0110, ISR_SPI_RF, 5)      ; 22 [3, 6] SPI_RF SPI receive register full
    CALLVECTOR (0x0118, ISR_SPI_ER, 5)      ; 23 [3, 6] SPI_ER SPI transmission error
#else  /* (_SUPPORT_SPI_ISR != FALSE) */
    CALLFATAL  (0x0108)                     ; 21 [3, 6] SPI_TE SPI transmit register is empty
    CALLFATAL  (0x0110)                     ; 22 [3, 6] SPI_RF SPI receive register full
    CALLFATAL  (0x0118)                     ; 23 [3, 6] SPI_ER SPI transmission error
#endif /* (_SUPPORT_SPI_ISR != FALSE) */
    CALLFATAL  (0x0120)                     ; 24 [3, 6] PWM_MASTER1_CMP Custom interrupt during the PWM period
#if (_SUPPORT_PWM_SYNC != FALSE)
    CALLFATAL  (0x0128)                     ; 25 [3, 6] PWM_MASTER1_END Interrupt at the end of the PWM period
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
    CALLVECTOR (0x0128, ISR_PWM_MASTER1_END, 3)  ; 25 [3, 6] PWM_MASTER1_END Interrupt at the end of the PWM period
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
    CALLFATAL  (0x0130)                     ; 26 [3, 6] PWM_SLAVE1_CMP Custom interrupt during the PWM period
    CALLFATAL  (0x0138)                     ; 27 [3, 6] PWM_SLAVE2_CMP Custom interrupt during the PWM period
    CALLFATAL  (0x0140)                     ; 28 [3, 6] PWM_SLAVE3_CMP Custom interrupt during the PWM period
    CALLFATAL  (0x0148)                     ; 29 [3, 6] PWM_MASTER2_CMP Custom interrupt during the PWM period
    CALLFATAL  (0x0150)                     ; 2A [3, 6] PWM_MASTER2_END Interrupt at the end of the PWM period
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_BZC != FALSE) || (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) || \
    (((_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_HALF_STEP_BEMF)) && (_SUPPORT_STALLDET_BRI != FALSE))
    CALLVECTOR (0x0158, ADC_ISR, 3)         ; 2B [3, 6] ADC_BLOCK ADC interrupt (PWM Motor Drive)
#elif _DEBUG_IO_ADC
    CALLVECTOR (0x0158, ADC_ISR, 4)         ; 2B [3, 6] ADC_BLOCK ADC interrupt (Debug purpose)
#else
    CALLFATAL  (0x0158)                     ; 2B [3, 6] ADC_SAR ADC interrupt
#endif /* (_SUPPORT_RINNEN != FALSE) */
#if ( _SUPPORT_NV_EMERGENCY_STORE == FALSE)
    CALLFATAL  (0x0160)                     ; 2C [3, 6] EE_COMPLETE
#else  /* ( _SUPPORT_NV_EMERGENCY_STORE == FALSE) */
    CALLVECTOR (0x0160, EE_COMPLETE_ISR, 0) ; 2C [3, 6] EE_COMPLETE
#endif /* ( _SUPPORT_NV_EMERGENCY_STORE == FALSE) */
    CALLFATAL  (0x0168)                     ; 2D [3, 6] FL_COMPLETE
    CALLFATAL  (0x0170)                     ; 2E [3, 6] COLIN_OWNMTX mutex interrupt
#if (LIN_COMM != FALSE)
#if defined (__MLX81330A01__)
    CALLVECTOR (0x0178, _COLIN_LIN_INT, 4)  ; 2F [3, 6] COLIN_LIN LIN interrupt
#else  /* defined (__MLX81330A01__) */
    CALLVECTOR (0x0178, COLIN_IT, 4)        ; 2F [3, 6] COLIN_LIN LIN interrupt
#endif /* defined (__MLX81330A01__) */
#else  /* (LIN_COMM != FALSE) */
    CALLFATAL  (0x0178)                     ; 2F [3, 6] COLIN_LIN LIN interrupt
#endif /* (LIN_COMM != FALSE) */
#if (_SUPPORT_RINNEN != FALSE)
    CALLFATAL  (0x0180)                     ; 30 [3, 6] OV_VS
#else  /* (_SUPPORT_RINNEN != FALSE) */
    CALLVECTOR (0x0180, ISR_OV_VS, 2)       ; 30 [3, 6] OV_VS
#endif /* (_SUPPORT_RINNEN != FALSE) */

#if (_SUPPORT_DIAG_DRV_PROT != FALSE) && (_SUPPORT_RINNEN == FALSE)
    CALLVECTOR (0x0188, ISR_DIAG, 2)        ; 31 [3, 6] DIAG
#else  /* (_SUPPORT_DIAG_DRV_PROT != FALSE) */
    CALLFATAL  (0x0188)                     ; 31 [3, 6] DIAG
#endif /* (_SUPPORT_DIAG_DRV_PROT != FALSE) */

#if defined (__MLX81330A01__) || defined (__MLX81330A02__)
    /* 81330A */
    CALLFATAL  (0x0190)                     ; 32 [3, 6] IO_IN0
    CALLFATAL  (0x0198)                     ; 33 [3, 6] IO_IN1
    CALLFATAL  (0x01A0)                     ; 34 [3, 6] IO_IN2
    CALLFATAL  (0x01A8)                     ; 35 [3, 6] IO_IN3
#if (_SUPPORT_I2C_RESET_ISR != FALSE)
    CALLVECTOR (0x01B0, ISR_I2C_GLOBAL_RESET, 2)  ; 36 [3, 6] I2C_GLOBAL_RESET
#else  /* (_SUPPORT_I2C_RESET_ISR != FALSE) */
    CALLFATAL  (0x01B0)                     ; 36 [3, 6] I2C_GLOBAL_RESET
#endif /* (_SUPPORT_I2C_RESET_ISR != FALSE) */
#if ((PWM_COMM != FALSE) && (_SUPPORT_PWM_COMM_ISR == C_PWM_COMM_PPM))
    CALLVECTOR (0x01B8, ISR_PPM_RX, 4)      ; 37 [3, 6] PPM_RX
#else  /* ((PWM_COMM != FALSE) && (_SUPPORT_PWM_COMM_ISR == C_PWM_COMM_PPM)) */
    CALLFATAL  (0x01B8)                     ; 37 [3, 6] PPM_RX
#endif /* ((PWM_COMM != FALSE) && (_SUPPORT_PWM_COMM_ISR == C_PWM_COMM_PPM)) */
    CALLFATAL  (0x01C0)                     ; 38 [3, 6] PPM_TX
#if ((_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
     (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
     (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
     (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
     (_SUPPORT_PRESSURE_MLX90829 != FALSE)) && (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM)
    CALLVECTOR (0x01C8, ISR_PPM_ERR, 3)     ; 39 [3, 6] PPM_ERR
#elif ((PWM_COMM != FALSE) && (_SUPPORT_PWM_COMM_ISR == C_PWM_COMM_PPM))
    CALLVECTOR (0x01C8, ISR_PPM_ERR, 4)     ; 39 [3, 6] PPM_ERR
#else  /* (_SUPPORT_TRIAXIS_MLX90367 || _SUPPORT_TRIAXIS_MLX90372 || _SUPPORT_TRIAXIS_MLX90377 || _SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 || _SUPPORT_PRESSURE_MLX90829) && (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM) */
    CALLFATAL  (0x01C8)                     ; 39 [3, 6] PPM_ERR
#endif /* (_SUPPORT_TRIAXIS_MLX90367 || _SUPPORT_TRIAXIS_MLX90372 || _SUPPORT_TRIAXIS_MLX90377 || _SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 || _SUPPORT_PRESSURE_MLX90829) && (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM) */
    CALLFATAL  (0x01D0)                     ; 3A [7, 7] MLX16_SOFT Software Interrupt request
#else
    /* 81330B */
#if (_SUPPORT_I2C_RESET_ISR != FALSE)
    CALLVECTOR (0x0190, ISR_I2C_GLOBAL_RESET, 2)  ; 32 [3, 6] I2C_GLOBAL_RESET
#else  /* (_SUPPORT_I2C_RESET_ISR != FALSE) */
    CALLFATAL  (0x0190)                     ; 32 [3, 6] I2C_GLOBAL_RESET
#endif /* (_SUPPORT_I2C_RESET_ISR != FALSE) */
#if ((PWM_COMM != FALSE) && (_SUPPORT_PWM_COMM_ISR == C_PWM_COMM_PPM))
    CALLVECTOR (0x0198, ISR_PPM_RX, 4)      ; 33 [3, 6] PPM_RX
#else  /* ((PWM_COMM != FALSE) && (_SUPPORT_PWM_COMM_ISR == C_PWM_COMM_PPM)) */
    CALLFATAL  (0x0198)                     ; 33 [3, 6] PPM_RX
#endif /* ((PWM_COMM != FALSE) && (_SUPPORT_PWM_COMM_ISR == C_PWM_COMM_PPM)) */
    CALLFATAL  (0x01A0)                     ; 34 [3, 6] PPM_TX
#if ((_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
     (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
     (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
     (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
     (_SUPPORT_PRESSURE_MLX90829 != FALSE)) && (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM)
    CALLVECTOR (0x01A8, ISR_PPM_ERR, 3)     ; 35 [3, 6] PPM_ERR
#elif ((PWM_COMM != FALSE) && (_SUPPORT_PWM_COMM_ISR == C_PWM_COMM_PPM))
    CALLVECTOR (0x01A8, ISR_PPM_ERR, 4)     ; 35 [3, 6] PPM_ERR
#else  /* (_SUPPORT_TRIAXIS_MLX90367 || _SUPPORT_TRIAXIS_MLX90372 || _SUPPORT_TRIAXIS_MLX90377 || _SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 || _SUPPORT_PRESSURE_MLX90829) && (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM) */
    CALLFATAL  (0x01A8)                     ; 35 [3, 6] PPM_ERR
#endif /* (_SUPPORT_TRIAXIS_MLX90367 || _SUPPORT_TRIAXIS_MLX90372 || _SUPPORT_TRIAXIS_MLX90377 || _SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 || _SUPPORT_PRESSURE_MLX90829) && (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM) */
#if ((_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U) && (_SUPPORT_HLA_IO == PIN_FUNC_IO_0)) || \
    ((PWM_COMM != FALSE) && (C_PWM_COMM_IN == PIN_FUNC_IO_0) && (_SUPPORT_PWM_COMM_ISR == C_PWM_COMM_IO_ISR))
    CALLVECTOR (0x01B0, ISR_IO_IN0, 3)      ; 36 [3, 6] IO_IN0
#elif (CAN_COMM && (_SUPPORT_CAN_IRQ_IO == PIN_FUNC_IO_0))
    CALLVECTOR (0x01B0, ISR_CAN_IRQ, 4)     ; 36 [3, 6] IO_IN0
#else  /* _SUPPORT_HALL_LATCH */
    CALLFATAL  (0x01B0)                     ; 36 [3, 6] IO_IN0
#endif /* _SUPPORT_HALL_LATCH */
#if ((_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U) && (_SUPPORT_HLA_IO == PIN_FUNC_IO_1))
    CALLVECTOR (0x01B8, ISR_IO_IN1, 3)      ; 37 [3, 6] IO_IN1
#elif ((CAN_COMM != FALSE) && (_SUPPORT_CAN_IRQ_IO == PIN_FUNC_IO_1))
    CALLVECTOR (0x01B8, ISR_CAN_IRQ, 4)     ; 37 [3, 6] IO_IN1
#else  /* _SUPPORT_HALL_LATCH */
    CALLFATAL  (0x01B8)                     ; 37 [3, 6] IO_IN1
#endif /* _SUPPORT_HALL_LATCH */
#if ((_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL == 1U) && (_SUPPORT_HLA_IO == PIN_FUNC_IO_2))
    CALLVECTOR (0x01C0, ISR_IO_IN2, 3)      ; 38 [3, 6] IO_IN2
#elif ((CAN_COMM != FALSE) && (_SUPPORT_CAN_IRQ_IO == PIN_FUNC_IO_2))
    CALLVECTOR (0x01C0, ISR_CAN_IRQ, 4)     ; 38 [3, 6] IO_IN2
#else  /* _SUPPORT_HALL_LATCH */
    CALLFATAL  (0x01C0)                     ; 38 [3, 6] IO_IN2
#endif /* _SUPPORT_HALL_LATCH */
#if (_SUPPORT_HALL_LATCH && (_SUPPORT_NR_OF_HL == 1U) && (_SUPPORT_HLA_IO == PIN_FUNC_IO_3))
    CALLVECTOR (0x01C8, ISR_IO_IN3, 3)      ; 39 [3, 6] IO_IN3
#elif ((CAN_COMM != FALSE) && (_SUPPORT_CAN_IRQ_IO == PIN_FUNC_IO_3))
    CALLVECTOR (0x01C8, ISR_CAN_IRQ, 4)     ; 39 [3, 6] IO_IN3
#elif ((PWM_COMM != FALSE) && (_SUPPORT_PWM_COMM_ISR == C_PWM_COMM_IO_ISR) && (C_PWM_COMM_IN == PIN_FUNC_IO_3))
    CALLVECTOR (0x01C8, ISR_IO_IN3, 4)      ; 39 [3, 6] IO_IN3
#else  /* _SUPPORT_HALL_LATCH */
    CALLFATAL  (0x01C8)                     ; 39 [3, 6] IO_IN3
#endif /* _SUPPORT_HALL_LATCH */
    CALLFATAL  (0x01D0)                     ; 3A [3, 6] TX_TIMEOUT
    CALLFATAL  (0x01D8)                     ; 3B [7, 7] MLX16_SOFT Software Interrupt request
#endif

; EOF
