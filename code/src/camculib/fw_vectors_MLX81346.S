;*!*************************************************************************** *
;* \file        fw_vectors_MLX81346.S
;* \brief       MLX81346 Interrupt vector table
;*
;* \note        project MLX81346
;*
;* \author      Marcel Braat
;*
;* \date        2020-08-11
;*
;* \version     2.0
;*
;*
;* \copyright   MELEXIS Microelectronic Integrated Systems
;*
;* Copyright (C) 2020-2022 Melexis N.V.
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

#define _DEBUG_IO_11_0_ISR                  FALSE /*TRUE*/                      /*!< FALSE: No IO-IRQ's; TRUE: IO-IRQ's */

/* Define check; GNU S-assembler doesn't check; None defined defines are handled as defined '0' (==FALSE) (MMP240306-1) */
#if !defined(CAN_COMM) || \
    ((CAN_COMM != FALSE) && !defined(_SUPPORT_CAN_IRQ_IO)) || \
    !defined(_DEBUG_IO_11_0_ISR) || \
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
    !defined(_SUPPORT_TRIAXIS_MLX90426) || \
    !defined(_SUPPORT_UART_0_ISR) || \
    !defined(_SUPPORT_UART_1_ISR) || \
    !defined(_SUPPORT_UART) || \
    ((_SUPPORT_UART != FALSE) && !defined(_SUPPORT_UART_DMA)) || \
    ((_SUPPORT_I2C != FALSE) && !defined(_SUPPORT_I2C_RW_ISR))
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
    CALLFATAL  (0x0050)                     ; 0A [1, 1] FL_SH_ECC
    CALLFATAL  (0x0058)                     ; 0B [1, 1] EE_SH_ECC

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

#if (_SUPPORT_DIAG_BOOST != FALSE) && (_SUPPORT_RINNEN == FALSE)
    CALLVECTOR (0x0070, ISR_UV_VBOOST, 0)   ; 0E [1, 1] UV_BOOST
#else  /* (_SUPPORT_DIAG_VDDAF != FALSE) */
    CALLFATAL  (0x0070)                     ; 0E [1, 1] UV_BOOST
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

#if (_SUPPORT_DIAG_VDDA != FALSE) && (_SUPPORT_RINNEN == FALSE)
    CALLVECTOR (0x0090, ISR_OC_VDDA, 1)     ; 12 [2, 1] OC_VDDA
#else  /* (_SUPPORT_DIAG_VDDA != FALSE) */
    CALLFATAL  (0x0090)                     ; 12 [2, 1] OC_VDDA
#endif /* (_SUPPORT_DIAG_VDDA != FALSE) */
    CALLFATAL  (0x0098)                     ; 13 [2, 1] OV_HS_IO3
    CALLFATAL  (0x00A0)                     ; 14 [2, 1] OV_HS_IO4

#if (_SUPPORT_DIAG_VDS != FALSE)
    CALLVECTOR (0x00A8, ISR_VDS, 1)         ; 15 [2, 1] OV_HS_VDS0
    CALLVECTOR (0x00B0, ISR_VDS, 1)         ; 16 [2, 1] OV_HS_VDS1
    CALLVECTOR (0x00B8, ISR_VDS, 1)         ; 17 [2, 1] OV_HS_VDS2
    CALLVECTOR (0x00C0, ISR_VDS, 1)         ; 18 [2, 1] OV_LS_VDS0
    CALLVECTOR (0x00C8, ISR_VDS, 1)         ; 19 [2, 1] OV_LS_VDS1
    CALLVECTOR (0x00D0, ISR_VDS, 1)         ; 1A [2, 1] OV_LS_VDS2
#else  /* (_SUPPORT_DIAG_VDS != FALSE) */
    CALLFATAL  (0x00A8)                     ; 15 [2, 1] OV_HS_VDS0
    CALLFATAL  (0x00B0)                     ; 16 [2, 1] OV_HS_VDS1
    CALLFATAL  (0x00B8)                     ; 17 [2, 1] OV_HS_VDS2
    CALLFATAL  (0x00C0)                     ; 18 [2, 1] OV_LS_VDS0
    CALLFATAL  (0x00C8)                     ; 19 [2, 1] OV_LS_VDS1
    CALLFATAL  (0x00D0)                     ; 1A [2, 1] OV_LS_VDS2
#endif /* (_SUPPORT_DIAG_VDS != FALSE) */

    CALLVECTOR (0x00D8, ISR_STIMER, 5)      ; 1B [3, 6] STIMER Simple Timer interrupt
    CALLFATAL  (0x00E0)                     ; 1C [3, 6] CTIMER0_1 Complex Timer interrupt 1
    CALLFATAL  (0x00E8)                     ; 1D [3, 6] CTIMER0_2 Complex Timer interrupt 2
    CALLVECTOR (0x00F0, ISR_CTIMER0_3, 3)   ; 1E [3, 6] CTIMER0_3 Complex Timer interrupt 3
#if (_SUPPORT_START_TO_RUN != FALSE)
    CALLFATAL  (0x00F8)                     ; 1F [3, 6] CTIMER1_1 Complex Timer interrupt 1
    CALLFATAL  (0x0100)                     ; 20 [3, 6] CTIMER1_2 Complex Timer interrupt 2
    CALLVECTOR (0x0108, ISR_CTIMER1_3, 3)   ; 21 [3, 6] CTIMER1_3 Complex Timer interrupt 3
#elif (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || \
      (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) || \
      (_SUPPORT_HALL_LATCH_MLX9227x != FALSE) || \
      ((_SUPPORT_HALL_LATCH != FALSE) && (_SUPPORT_NR_OF_HL >= 2) && (_SUPPORT_MICRO_STEP_COMMUTATION != FALSE))
    CALLVECTOR (0x00F8, ISR_CTIMER1_1, 3)   ; 1F [3, 6] CTIMER1_1 Complex Timer interrupt 1
    CALLVECTOR (0x0100, ISR_CTIMER1_2, 3)   ; 20 [3, 6] CTIMER1_2 Complex Timer interrupt 2
    CALLVECTOR (0x0108, ISR_CTIMER1_3, 3)   ; 21 [3, 6] CTIMER1_3 Complex Timer interrupt 3
#elif (((_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
        (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || \
        (_SUPPORT_TRIAXIS_MLX90422 != FALSE) || (_SUPPORT_TRIAXIS_MLX90426 != FALSE) || \
        (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
        (_SUPPORT_PRESSURE_MLX90829 != FALSE)) && (_SUPPORT_SENT_HW_RESOURCE == C_SENT_CTIMER1)) || \
      ((_SUPPORT_TRIAXIS_MLX90377 != FALSE) && (_SUPPORT_SENSOR_SENT_IO == PIN_FUNC_IO_NONE) && (_SUPPORT_SENSOR_PWM_IO != PIN_FUNC_IO_NONE)) || \
      (((_SUPPORT_TRIAXIS_MLX90421 != FALSE) || (_SUPPORT_TRIAXIS_MLX90425 != FALSE)) && (_SUPPORT_PWM_HW_RESOURCE == C_PWM_HW_RES_CTIMER1))
    CALLVECTOR (0x00F8, ISR_CTIMER1_1, 3)   ; 1F [3, 6] CTIMER1_1 Complex Timer interrupt 1
    CALLFATAL  (0x0100)                     ; 20 [3, 6] CTIMER1_2 Complex Timer interrupt 2
    CALLFATAL  (0x0108)                     ; 21 [3, 6] CTIMER1_3 Complex Timer interrupt 3
#elif (_SUPPORT_I2C_MSTR_ISR != FALSE) && (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
    CALLFATAL  (0x00F8)                     ; 1F [3, 6] CTIMER1_1 Complex Timer interrupt 1
    CALLFATAL  (0x0100)                     ; 20 [3, 6] CTIMER1_2 Complex Timer interrupt 2
    CALLVECTOR (0x0108, ISR_CTIMER1_3, 3)   ; 21 [3, 6] CTIMER1_3 Complex Timer interrupt 3
#elif (PWM_COMM != FALSE) && (_SUPPORT_PWM_COMM_ISR == C_PWM_COMM_CTIMER1)
    CALLVECTOR (0x00F8, ISR_CTIMER1_1, 4)   ; 1F [3, 6] CTIMER1_1 Complex Timer interrupt 1
    CALLVECTOR (0x0100, ISR_CTIMER1_2, 4)   ; 20 [3, 6] CTIMER1_2 Complex Timer interrupt 2
    CALLVECTOR (0x0108, ISR_CTIMER1_3, 4)   ; 21 [3, 6] CTIMER1_3 Complex Timer interrupt 3
#else
    CALLFATAL  (0x00F8)                     ; 1F [3, 6] CTIMER1_1 Complex Timer interrupt 1
    CALLFATAL  (0x0100)                     ; 20 [3, 6] CTIMER1_2 Complex Timer interrupt 2
    CALLFATAL  (0x0108)                     ; 21 [3, 6] CTIMER1_3 Complex Timer interrupt 3
#endif /* (_SUPPORT_DUAL_HALL_LATCH_MLX92251 != FALSE) || (_SUPPORT_DUAL_HALL_LATCH_MLX92255 != FALSE) */
#if (_SUPPORT_SPI_ISR != FALSE) || (_SUPPORT_ZWICKAU != FALSE)
    CALLVECTOR (0x0110, ISR_SPI_TE, 5)      ; 22 [3, 6] SPI_TE SPI transmit register is empty
    CALLVECTOR (0x0118, ISR_SPI_RF, 5)      ; 23 [3, 6] SPI_RF SPI receive register full
    CALLVECTOR (0x0120, ISR_SPI_ER, 5)      ; 24 [3, 6] SPI_ER SPI transmission error
#else  /* (_SUPPORT_SPI_ISR != FALSE) || (_SUPPORT_ZWICKAU != FALSE) */
    CALLFATAL  (0x0110)                     ; 22 [3, 6] SPI_TE SPI transmit register is empty
    CALLFATAL  (0x0118)                     ; 23 [3, 6] SPI_RF SPI receive register full
    CALLFATAL  (0x0120)                     ; 24 [3, 6] SPI_ER SPI transmission error
#endif /* (_SUPPORT_SPI_ISR != FALSE) || (_SUPPORT_ZWICKAU != FALSE) */
    CALLFATAL  (0x0128)                     ; 25 [3, 6] PWM_MASTER1_CMP Custom interrupt during the PWM period
#if (_SUPPORT_PWM_SYNC != FALSE)
    CALLFATAL  (0x0130)                     ; 26 [3, 6] PWM_MASTER1_END Interrupt at the end of the PWM period
#else  /* (_SUPPORT_PWM_SYNC != FALSE) */
    CALLVECTOR (0x0130, ISR_PWM_MASTER1_END, 3)  ; 26 [3, 6] PWM_MASTER1_END Interrupt at the end of the PWM period
#endif /* (_SUPPORT_PWM_SYNC != FALSE) */
    CALLFATAL  (0x0138)                     ; 27 [3, 6] PWM_SLAVE1_CMP Custom interrupt during the PWM period
    CALLFATAL  (0x0140)                     ; 28 [3, 6] PWM_SLAVE2_CMP Custom interrupt during the PWM period
    CALLFATAL  (0x0148)                     ; 29 [3, 6] PWM_SLAVE3_CMP Custom interrupt during the PWM period
    CALLFATAL  (0x0150)                     ; 2A [3, 6] PWM_MASTER2_CMP Custom interrupt during the PWM period
    CALLFATAL  (0x0158)                     ; 2B [3, 6] PWM_MASTER2_END Interrupt at the end of the PWM period
#if (_SUPPORT_FOC_MODE != FOC_MODE_NONE) || (_SUPPORT_STALLDET_BZC != FALSE) || (_SUPPORT_PWM_MODE == SINGLE_COIL_PWM) || \
    (((_SUPPORT_PWM_MODE == TRIPLEPHASE_FULL_STEP_BEMF) || (_SUPPORT_PWM_MODE == BIPOLAR_FULL_STEP_BEMF)) && (_SUPPORT_STALLDET_BRI != FALSE)) || \
    (_SUPPORT_MOTION_DET == C_MOTION_DET_BEMF) || (_SUPPORT_MOTION_DET == C_MOTION_DET_SENSOR)
    CALLVECTOR (0x0160, ADC_ISR, 3)         ; 2C [3, 6] ADC_BLOCK ADC interrupt (PWM Motor Drive)
#elif (_DEBUG_IO_ADC != FALSE)
    CALLVECTOR (0x0160, ADC_ISR, 4)         ; 2C [3, 6] ADC_BLOCK ADC interrupt (Debug purpose)
#else  /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
    CALLFATAL  (0x0160)                     ; 2C [3, 6] ADC_BLOCK ADC
#endif /* (_SUPPORT_FOC_MODE != FOC_MODE_NONE) */
#if ( _SUPPORT_NV_EMERGENCY_STORE == FALSE)
    CALLFATAL  (0x0168)                     ; 2D [3, 6] EE_COMPLETE
#else  /* ( _SUPPORT_NV_EMERGENCY_STORE == FALSE) */
    CALLVECTOR (0x0168, EE_COMPLETE_ISR, 0) ; 2D [3, 6] EE_COMPLETE
#endif /* ( _SUPPORT_NV_EMERGENCY_STORE == FALSE) */
    CALLFATAL  (0x0170)                     ; 2E [3, 6] FL_COMPLETE
    CALLFATAL  (0x0178)                     ; 2F [3, 6] COLIN_OWNMTX mutex interrupt
#if (LIN_COMM != FALSE)
    CALLVECTOR (0x0180, COLIN_IT, 4)        ; 30 [3, 6] COLIN_LIN LIN interrupt
#else  /* (LIN_COMM != FALSE) */
    CALLFATAL  (0x0180)                     ; 30 [3, 6] COLIN_LIN LIN interrupt
#endif /* (LIN_COMM != FALSE) */

#if (_SUPPORT_DIAG_DRV_PROT != FALSE) && (_SUPPORT_RINNEN == FALSE)
    CALLVECTOR (0x0188, ISR_DIAG, 2)        ; 31 [3, 6] DIAG
#else  /* (_SUPPORT_DIAG_DRV_PROT != FALSE) */
    CALLFATAL  (0x0188)                     ; 31 [3, 6] DIAG
#endif /* (_SUPPORT_DIAG_DRV_PROT != FALSE) */

#if (_SUPPORT_I2C != FALSE) && (_SUPPORT_I2C_RESET_ISR != FALSE)
    CALLVECTOR (0x0190, ISR_I2C_GlobalReset, 5) ; 32 [3, 6] I2C_GLOBAL_RESET
#else  /* (_SUPPORT_I2C != FALSE) && (_SUPPORT_I2C_RESET_ISR != FALSE) */
    CALLFATAL  (0x0190)                     ; 32 [3, 6] I2C_GLOBAL_RESET
#endif /* (_SUPPORT_I2C != FALSE) && (_SUPPORT_I2C_RESET_ISR != FALSE) */

    CALLFATAL  (0x0198)                     ; 33 [3, 6] PPM_RX
    CALLFATAL  (0x01A0)                     ; 34 [3, 6] PPM_TX
#if ((_SUPPORT_TRIAXIS_MLX90367 != FALSE) || (_SUPPORT_TRIAXIS_MLX90372 != FALSE) || \
     (_SUPPORT_TRIAXIS_MLX90377 != FALSE) || (_SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 != FALSE) || \
     (_SUPPORT_PRESSURE_MLX90829 != FALSE)) && (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM)
    CALLVECTOR (0x01A8, ISR_PPM_ERR, 3)     ; 35 [3, 6] PPM_ERR
#else  /* (_SUPPORT_TRIAXIS_MLX90367 || _SUPPORT_TRIAXIS_MLX90372 || _SUPPORT_TRIAXIS_MLX90377 || _SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 || _SUPPORT_PRESSURE_MLX90829) && (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM) */
    CALLFATAL  (0x01A8)                     ; 35 [3, 6] PPM_ERR
#endif /* (_SUPPORT_TRIAXIS_MLX90367 || _SUPPORT_TRIAXIS_MLX90372 || _SUPPORT_TRIAXIS_MLX90377 || _SUPPORT_INDUCTIVE_POS_SENSOR_MLX90513 || _SUPPORT_PRESSURE_MLX90829) && (_SUPPORT_SENT_HW_RESOURCE == C_SENT_PPM) */

#if (_SUPPORT_RINNEN != FALSE)
    CALLFATAL  (0x01B0)                     ; 36 [3, 6] OV_VS
#else  /* (_SUPPORT_RINNEN != FALSE) */
    CALLVECTOR (0x01B0, ISR_OV_VS, 2)       ; 36 [3, 6] OV_VS
#endif /* (_SUPPORT_RINNEN != FALSE) */
#if (_SUPPORT_DIAG_VDDA != FALSE) && (_SUPPORT_RINNEN == FALSE)
    CALLVECTOR (0x01B8, ISR_OV_VDDA, 2)     ; 37 [3, 6] OV_VDDA
#else  /* (_SUPPORT_DIAG_VDDA != FALSE) */
    CALLFATAL  (0x01B8)                     ; 37 [3, 6] OV_VDDA
#endif /* (_SUPPORT_DIAG_VDDA != FALSE) */
#if (_SUPPORT_DIAG_BOOST != FALSE) && (_SUPPORT_RINNEN == FALSE)
    CALLVECTOR (0x01C0, ISR_OV_VBOOST, 2)   ; 38 [3, 6] OV_BOOST
#else  /* (_SUPPORT_DIAG_BOOST != FALSE) */
    CALLFATAL  (0x01C0)                     ; 38 [3, 6] OV_BOOST
#endif /* (_SUPPORT_DIAG_BOOST != FALSE) */

#if (_DEBUG_IO_11_0_ISR != FALSE)
    CALLVECTOR (0x01C8, ISR_IO_IN0, 5)      ; 39 [3, 6] IO_IN0
    CALLVECTOR (0x01D0, ISR_IO_IN1, 5)      ; 3A [3, 6] IO_IN1
    CALLVECTOR (0x01D8, ISR_IO_IN2, 5)      ; 3B [3, 6] IO_IN2
    CALLVECTOR (0x01E0, ISR_IO_IN3, 5)      ; 3C [3, 6] IO_IN3
    CALLVECTOR (0x01E8, ISR_IO_IN4, 5)      ; 3D [3, 6] IO_IN4
    CALLVECTOR (0x01F0, ISR_IO_IN5, 5)      ; 3E [3, 6] IO_IN5
    CALLVECTOR (0x01F8, ISR_IO_IN6, 5)      ; 3F [3, 6] IO_IN6
    CALLVECTOR (0x0200, ISR_IO_IN7, 5)      ; 40 [3, 6] IO_IN7
    CALLVECTOR (0x0208, ISR_IO_IN8, 5)      ; 41 [3, 6] IO_IN8
    CALLVECTOR (0x0210, ISR_IO_IN9, 5)      ; 42 [3, 6] IO_IN9
    CALLVECTOR (0x0218, ISR_IO_IN10, 5)     ; 43 [3, 6] IO_IN10
    CALLVECTOR (0x0220, ISR_IO_IN11, 5)     ; 44 [3, 6] IO_IN11
#else  /* (_DEBUG_IO_11_0_ISR != FALSE) */
#if ((_SUPPORT_HALL_LATCH != FALSE) && (((_SUPPORT_NR_OF_HL == 1U) && (_SUPPORT_HLA_IO == PIN_FUNC_IO_0)) || \
                                        ((_SUPPORT_NR_OF_HL >= 2U) && (_SUPPORT_MICRO_STEP_COMMUTATION == FALSE) && (_SUPPORT_HLA_IO == PIN_FUNC_IO_0))))
    CALLVECTOR (0x01C8, ISR_IO_IN0, 3)      ; 39 [3, 6] IO_IN0
#elif ((CAN_COMM != FALSE) && (_SUPPORT_CAN_IRQ_IO == PIN_FUNC_IO_0))
    CALLVECTOR (0x01C8, ISR_CAN_IRQ, 4)     ; 39 [3, 6] IO_IN0
#else  /* (_SUPPORT_HALL_LATCH != FALSE) */
    CALLFATAL  (0x01C8)                     ; 39 [3, 6] IO_IN0
#endif /* (_SUPPORT_HALL_LATCH != FALSE) */
#if ((_SUPPORT_HALL_LATCH != FALSE) && (((_SUPPORT_NR_OF_HL == 1U) && (_SUPPORT_HLA_IO == PIN_FUNC_IO_1)) || \
                                        ((_SUPPORT_NR_OF_HL >= 2U) && (_SUPPORT_MICRO_STEP_COMMUTATION == FALSE) && ((_SUPPORT_HLA_IO == PIN_FUNC_IO_1) || (_SUPPORT_HLB_IO == PIN_FUNC_IO_1)))))
    CALLVECTOR (0x01D0, ISR_IO_IN1, 3)      ; 3A [3, 6] IO_IN1
#elif ((CAN_COMM != FALSE) && (_SUPPORT_CAN_IRQ_IO == PIN_FUNC_IO_1))
    CALLVECTOR (0x01D0, ISR_CAN_IRQ, 4)     ; 3A [3, 6] IO_IN1
#else  /* (_SUPPORT_HALL_LATCH != FALSE) */
    CALLFATAL  (0x01D0)                     ; 3A [3, 6] IO_IN1
#endif /* (_SUPPORT_HALL_LATCH != FALSE) */
#if ((_SUPPORT_HALL_LATCH != FALSE) && (((_SUPPORT_NR_OF_HL == 1U) && (_SUPPORT_HLA_IO == PIN_FUNC_IO_2)) || \
                                        ((_SUPPORT_NR_OF_HL >= 2U) && (_SUPPORT_MICRO_STEP_COMMUTATION == FALSE) && ((_SUPPORT_HLA_IO == PIN_FUNC_IO_2) || (_SUPPORT_HLB_IO == PIN_FUNC_IO_2) || (_SUPPORT_HLC_IO == PIN_FUNC_IO_2)))))
    CALLVECTOR (0x01D8, ISR_IO_IN2, 3)      ; 3B [3, 6] IO_IN2
#elif ((CAN_COMM != FALSE) && (_SUPPORT_CAN_IRQ_IO == PIN_FUNC_IO_2))
    CALLVECTOR (0x01D8, ISR_CAN_IRQ, 4)     ; 3B [3, 6] IO_IN2
#else  /* (_SUPPORT_HALL_LATCH != FALSE) */
    CALLFATAL  (0x01D8)                     ; 3B [3, 6] IO_IN2
#endif /* (_SUPPORT_HALL_LATCH != FALSE) */
#if ((_SUPPORT_HALL_LATCH != FALSE) && (((_SUPPORT_NR_OF_HL == 1U) && (_SUPPORT_HLA_IO == PIN_FUNC_IO_3)) || \
                                        ((_SUPPORT_NR_OF_HL >= 2U) && (_SUPPORT_MICRO_STEP_COMMUTATION == FALSE) && ((_SUPPORT_HLA_IO == PIN_FUNC_IO_3) || (_SUPPORT_HLB_IO == PIN_FUNC_IO_3) || (_SUPPORT_HLC_IO == PIN_FUNC_IO_3)))))
    CALLVECTOR (0x01E0, ISR_IO_IN3, 3)      ; 3C [3, 6] IO_IN3
#elif ((CAN_COMM != FALSE) && (_SUPPORT_CAN_IRQ_IO == PIN_FUNC_IO_3))
    CALLVECTOR (0x01E0, ISR_CAN_IRQ, 4)     ; 3C [3, 6] IO_IN3
#else  /* (_SUPPORT_HALL_LATCH != FALSE) */
    CALLFATAL  (0x01E0)                     ; 3C [3, 6] IO_IN3
#endif /* (_SUPPORT_HALL_LATCH != FALSE) */
#if ((_SUPPORT_HALL_LATCH != FALSE) && (((_SUPPORT_NR_OF_HL == 1U) && (_SUPPORT_HLA_IO == PIN_FUNC_IO_4)) || \
                                        ((_SUPPORT_NR_OF_HL >= 2U) && (_SUPPORT_MICRO_STEP_COMMUTATION == FALSE) && ((_SUPPORT_HLA_IO == PIN_FUNC_IO_4) || (_SUPPORT_HLB_IO == PIN_FUNC_IO_4) || (_SUPPORT_HLC_IO == PIN_FUNC_IO_4)))))
    CALLVECTOR (0x01E8, ISR_IO_IN4, 3)      ; 3D [3, 6] IO_IN4
#elif ((CAN_COMM != FALSE) && (_SUPPORT_CAN_IRQ_IO == PIN_FUNC_IO_4))
    CALLVECTOR (0x01E8, ISR_CAN_IRQ, 4)     ; 3D [3, 6] IO_IN4
#else  /* (_SUPPORT_HALL_LATCH != FALSE) */
    CALLFATAL  (0x01E8)                     ; 3D [3, 6] IO_IN4
#endif /* (_SUPPORT_HALL_LATCH != FALSE) */
#if ((_SUPPORT_HALL_LATCH != FALSE) && (((_SUPPORT_NR_OF_HL == 1U) && (_SUPPORT_HLA_IO == PIN_FUNC_IO_5)) || \
                                        ((_SUPPORT_NR_OF_HL >= 2U) && (_SUPPORT_MICRO_STEP_COMMUTATION == FALSE) && ((_SUPPORT_HLA_IO == PIN_FUNC_IO_5) || (_SUPPORT_HLB_IO == PIN_FUNC_IO_5) || (_SUPPORT_HLC_IO == PIN_FUNC_IO_5)))))
    CALLVECTOR (0x01F0, ISR_IO_IN5, 3)      ; 3E [3, 6] IO_IN5
#elif ((CAN_COMM != FALSE) && (_SUPPORT_CAN_IRQ_IO == PIN_FUNC_IO_5))
    CALLVECTOR (0x01F0, ISR_CAN_IRQ, 4)     ; 3E [3, 6] IO_IN5
#elif ((PWM_COMM != FALSE) && (_SUPPORT_PWM_COMM_ISR == C_PWM_COMM_IO_ISR) && (C_PWM_COMM_IN == PIN_FUNC_IO_5))
    CALLVECTOR (0x01F0, ISR_IO_IN5, 4)      ; 3E [3, 6] IO_IN5
#else  /* (_SUPPORT_HALL_LATCH != FALSE) */
    CALLFATAL  (0x01F0)                     ; 3E [3, 6] IO_IN5
#endif /* (_SUPPORT_HALL_LATCH != FALSE) */
#if ((_SUPPORT_HALL_LATCH != FALSE) && (((_SUPPORT_NR_OF_HL == 1U) && (_SUPPORT_HLA_IO == PIN_FUNC_IO_6)) || \
                                        ((_SUPPORT_NR_OF_HL >= 2U) && (_SUPPORT_MICRO_STEP_COMMUTATION == FALSE) && ((_SUPPORT_HLB_IO == PIN_FUNC_IO_6) || (_SUPPORT_HLC_IO == PIN_FUNC_IO_6)))))
    CALLVECTOR (0x01F8, ISR_IO_IN6, 3)      ; 3F [3, 6] IO_IN6
#elif ((CAN_COMM != FALSE) && (_SUPPORT_CAN_IRQ_IO == PIN_FUNC_IO_6))
    CALLVECTOR (0x01F8, ISR_CAN_IRQ, 4)     ; 3F [3, 6] IO_IN6
#else  /* (_SUPPORT_HALL_LATCH != FALSE) */
    CALLFATAL  (0x01F8)                     ; 3F [3, 6] IO_IN6
#endif /* (_SUPPORT_HALL_LATCH != FALSE) */
#if ((_SUPPORT_HALL_LATCH != FALSE) && (((_SUPPORT_NR_OF_HL == 1U) && (_SUPPORT_HLA_IO == PIN_FUNC_IO_7)) || \
                                        ((_SUPPORT_NR_OF_HL >= 2U) && (_SUPPORT_MICRO_STEP_COMMUTATION == FALSE) && (_SUPPORT_HLC_IO == PIN_FUNC_IO_7))))
    CALLVECTOR (0x0200, ISR_IO_IN6, 3)      ; 40 [3, 6] IO_IN7
#elif ((CAN_COMM != FALSE) && (_SUPPORT_CAN_IRQ_IO == PIN_FUNC_IO_7))
    CALLVECTOR (0x0200, ISR_CAN_IRQ, 4)     ; 40 [3, 6] IO_IN7
#else  /* (_SUPPORT_HALL_LATCH != FALSE) */
    CALLFATAL  (0x0200)                     ; 40 [3, 6] IO_IN7
#endif /* (_SUPPORT_HALL_LATCH != FALSE) */
    CALLFATAL  (0x0208)                     ; 41 [3, 6] IO_IN8
    CALLFATAL  (0x0210)                     ; 42 [3, 6] IO_IN9
    CALLFATAL  (0x0218)                     ; 43 [3, 6] IO_IN10
    CALLFATAL  (0x0220)                     ; 44 [3, 6] IO_IN11
#endif /* (_DEBUG_IO_11_0_ISR != FALSE) */

    CALLFATAL  (0x0228)                     ; 45 [3, 6] TX_TIMEOUT
#if (_SUPPORT_UART != FALSE) && (_SUPPORT_UART_0_ISR != FALSE)
    CALLVECTOR (0x0230, ISR_UART0_SB, 4);   ; 46 [3, 6] UART0_SB
    CALLVECTOR (0x0238, ISR_UART0_RS, 4);   ; 47 [3, 6] UART0_RS
#if (_SUPPORT_UART_DMA == FALSE)
    CALLVECTOR (0x0240, ISR_UART0_RR, 4);   ; 48 [3, 6] UART0_RR
    CALLVECTOR (0x0248, ISR_UART0_TS, 4);   ; 49 [3, 6] UART0_TS
    CALLVECTOR (0x0250, ISR_UART0_TR, 4);   ; 4A [3, 6] UART0_TR
#elif (_SUPPORT_UART_IF_APP == C_UART_IF_SCOPE)
    CALLVECTOR (0x0240, ISR_UART0_RR, 4);   ; 48 [3, 6] UART0_RR
    CALLFATAL  (0x0248)                     ; 49 [3, 6] UART0_TS
    CALLFATAL  (0x0250)                     ; 4A [3, 6] UART0_TR
#else  /* (_SUPPORT_UART_DMA == FALSE) */
    CALLFATAL  (0x0240)                     ; 48 [3, 6] UART0_RR
    CALLFATAL  (0x0248)                     ; 49 [3, 6] UART0_TS
    CALLFATAL  (0x0250)                     ; 4A [3, 6] UART0_TR
#endif /* (_SUPPORT_UART_DMA == FALSE) */
    CALLVECTOR (0x0258, ISR_UART0_TE, 4);   ; 4B [3, 6] UART0_TE
#if (_SUPPORT_UART_DMA != FALSE)
    CALLVECTOR (0x0260, ISR_UDFR0, 4);      ; 4C [3, 6] UDFR0
    CALLVECTOR (0x0268, ISR_UDTF0, 4);      ; 4D [3, 6] UDTF0
#else  /* (_SUPPORT_UART_DMA != FALSE) */
    CALLFATAL  (0x0260)                     ; 4C [3, 6] UDFR0
    CALLFATAL  (0x0268)                     ; 4D [3, 6] UDTF0
#endif /* (_SUPPORT_UART_DMA != FALSE) */
#else  /* (_SUPPORT_UART != FALSE) && (_SUPPORT_UART_0_ISR != FALSE) */
    CALLFATAL  (0x0230)                     ; 46 [3, 6] UART0_SB
    CALLFATAL  (0x0238)                     ; 47 [3, 6] UART0_RS
    CALLFATAL  (0x0240)                     ; 48 [3, 6] UART0_RR
    CALLFATAL  (0x0248)                     ; 49 [3, 6] UART0_TS
    CALLFATAL  (0x0250)                     ; 4A [3, 6] UART0_TR
    CALLFATAL  (0x0258)                     ; 4B [3, 6] UART0_TE
    CALLFATAL  (0x0260)                     ; 4C [3, 6] UDFR0
    CALLFATAL  (0x0268)                     ; 4D [3, 6] UDTF0
#endif /* (_SUPPORT_UART != FALSE) && (_SUPPORT_UART_0_ISR != FALSE) */
#if (_SUPPORT_UART != FALSE) && ((_SUPPORT_UART_1_ISR != FALSE) || (_SUPPORT_UART2_ISR != FALSE))
    CALLVECTOR (0x0270, ISR_UART1_SB, 4);   ; 4E [3, 6] UART1_SB
    CALLVECTOR (0x0278, ISR_UART1_RS, 4);   ; 4F [3, 6] UART1_RS
#if (_SUPPORT_UART2_DMA == FALSE)
    CALLVECTOR (0x0280, ISR_UART1_RR, 4);   ; 50 [3, 6] UART1_RR
    CALLVECTOR (0x0288, ISR_UART1_TS, 4);   ; 51 [3, 6] UART1_TS
    CALLVECTOR (0x0290, ISR_UART1_TR, 4);   ; 52 [3, 6] UART1_TR
#elif (_SUPPORT_UART2_IF_APP == C_UART_IF_SCOPE)
    CALLVECTOR (0x0280, ISR_UART1_RR, 4);   ; 50 [3, 6] UART0_RR
    CALLFATAL  (0x0288)                     ; 51 [3, 6] UART1_TS
    CALLFATAL  (0x0290)                     ; 52 [3, 6] UART1_TR
#else /* (_SUPPORT_UART2_DMA == FALSE) */
    CALLFATAL  (0x0280)                     ; 50 [3, 6] UART1_RR
    CALLFATAL  (0x0288)                     ; 51 [3, 6] UART1_TS
    CALLFATAL  (0x0290)                     ; 52 [3, 6] UART1_TR
#endif /* (_SUPPORT_UART2_DMA == FALSE) */
    CALLVECTOR (0x0298, ISR_UART1_TE, 4);   ; 53 [3, 6] UART1_TE
#if (_SUPPORT_UART2_DMA != FALSE)
    CALLVECTOR (0x02A0, ISR_UDFR1, 4);      ; 54 [3, 6] UDFR1
    CALLVECTOR (0x02A8, ISR_UDTF1, 4);      ; 55 [3, 6] UDTF1
#else  /* (_SUPPORT_UART2_DMA != FALSE) */
    CALLFATAL  (0x02A0)                     ; 54 [3, 6] UDFR1
    CALLFATAL  (0x02A8)                     ; 55 [3, 6] UDTF1
#endif /* (_SUPPORT_UART2_DMA != FALSE) */
#else  /* (_SUPPORT_UART != FALSE) && ((_SUPPORT_UART_1_ISR != FALSE) || (_SUPPORT_UART2_ISR != FALSE)) */
    CALLFATAL  (0x0270)                     ; 4E [3, 6] UART1_SB
    CALLFATAL  (0x0278)                     ; 4F [3, 6] UART1_RS
    CALLFATAL  (0x0280)                     ; 50 [3, 6] UART1_RR
    CALLFATAL  (0x0288)                     ; 51 [3, 6] UART1_TS
    CALLFATAL  (0x0290)                     ; 52 [3, 6] UART1_TR
    CALLFATAL  (0x0298)                     ; 53 [3, 6] UART1_TE
    CALLFATAL  (0x02A0)                     ; 54 [3, 6] UDFR1
    CALLFATAL  (0x02A8)                     ; 55 [3, 6] UDTF1
#endif /* (_SUPPORT_UART != FALSE) && ((_SUPPORT_UART_1_ISR != FALSE) || (_SUPPORT_UART2_ISR != FALSE)) */

#if (_SUPPORT_CDI != FALSE)
    CALLVECTOR (0x02B0, ISR_CDI, 3)         ; 56 [3, 6] ACTIVE_CDI
#else  /* (_SUPPORT_CDI != FALSE) */
    CALLFATAL  (0x02B0)                     ; 56 [3, 6] ACTIVE_CDI
#endif /* (_SUPPORT_CDI != FALSE) */
#if (_SUPPORT_I2C != FALSE) && (_SUPPORT_I2C_RW_ISR != FALSE)
    CALLVECTOR (0x02B8, ISR_I2C_EOW, 5)     ; 57 [3, 6] I2C_EOW
    CALLVECTOR (0x02C0, ISR_I2C_EOR, 5)     ; 58 [3, 6] I2C_EOR
#else  /* (_SUPPORT_I2C != FALSE) && (_SUPPORT_I2C_RW_ISR != FALSE) */
    CALLFATAL  (0x02B8)                     ; 57 [3, 6] I2C_EOW
    CALLFATAL  (0x02C0)                     ; 58 [3, 6] I2C_EOR
#endif /* (_SUPPORT_I2C != FALSE) && (_SUPPORT_I2C_RW_ISR != FALSE) */
#if defined (__MLX81346A01__)
/* MLX81346A */
    CALLFATAL  (0x02C8)                     ; 59 [7, 7] MLX16_SOFT Software Interrupt request
#else  /* defined (__MLX81346A01__) */
#if (_SUPPORT_I2C != FALSE) && (_SUPPORT_I2C_ERR_ISR != FALSE)
    CALLVECTOR (0x02C8, ISR_I2C_ERR, 5)     ; 59 [3, 6] I2C_ERR
#else  /* (_SUPPORT_I2C != FALSE) && (_SUPPORT_I2C_ERR_ISR != FALSE) */
    CALLFATAL  (0x02C8)                     ; 59 [3, 6] I2C_ERR
#endif /* (_SUPPORT_I2C != FALSE) && (_SUPPORT_I2C_ERR_ISR != FALSE) */
    CALLFATAL  (0x02D0)                     ; 60 [7, 7] MLX16_SOFT Software Interrupt request
#endif /* defined (__MLX81346A01__) */

; EOF
