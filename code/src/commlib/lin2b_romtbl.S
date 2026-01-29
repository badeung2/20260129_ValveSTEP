;
;
; Copyright (C) 2018-2022 Melexis N.V.
;
; Software Platform
;
;--------------------------------------------------
;-------- LIN Configuration ROM tables ------------
;--------------------------------------------------

#include "AppDefines.h"                                                         /* Application Defines for C and S files */
#include "AppSwitches.h"                                                        /* Application Switches for C and S files */

    .global _INDXtbl
    .global _PARAMtbl
    .global _AUTOADDtbl

    .sect .mlx4_flash_table,"xa",@progbits

;Index table (ROM) : 32 bytes (32 x 8 x 2 = 64 ID)
;Access : Rom:INDXtbl[X,A] with ID in [X,A] shifted once to the left
;Index : Frame ID
;(order : nibble1, nibble0, nibble3, nibble2)
;         arg_b    arg_a    arg_d    arg_c
;
;         xx  xxxxxx
;           \       \
;            \       local index
;             00 (0) : Message Index defined in ROM - index 10h to 3Fh
;             01 (4) : Message Index defined during initialization (in RAM) or part of a group - index 00h to 0Fh <------------------
;             10 (8) : Message Index defined in Non Volatile Memory - need to be loaded in RAM during initialization - not available yet
;             11 (C) : other / not used (default)

_INDXtbl:   ;
    .byte 0x40  ;FrameID 00h : unused
    .byte 0x40  ;FrameID 01h : unused
    .byte 0x40  ;FrameID 02h : unused
    .byte 0x40  ;FrameID 03h : unused
    .byte 0x40  ;FrameID 04h : unused
    .byte 0x40  ;FrameID 05h : unused
    .byte 0x40  ;FrameID 06h : unused
    .byte 0x40  ;FrameID 07h : unused
    ;
    .byte 0x40  ;FrameID 08h : unused
    .byte 0x40  ;FrameID 09h : unused
    .byte 0x40  ;FrameID 0Ah : unused
    .byte 0x40  ;FrameID 0Bh : unused
    .byte 0x40  ;FrameID 0Ch : unused
    .byte 0x40  ;FrameID 0Dh : unused
    .byte 0x40  ;FrameID 0Eh : unused
    .byte 0x40  ;FrameID 0Fh : unused
    ;
    .byte 0x40  ;FrameID 10h : unused
    .byte 0x40  ;FrameID 11h : unused
    .byte 0x40  ;FrameID 12h : unused
    .byte 0x40  ;FrameID 13h : unused
    .byte 0x40  ;FrameID 14h : unused
    .byte 0x40  ;FrameID 15h : unused
    .byte 0x40  ;FrameID 16h : unused
    .byte 0x40  ;FrameID 17h : unused
    ;
    .byte 0x40  ;FrameID 18h : unused
    .byte 0x40  ;FrameID 19h : unused
    .byte 0x40  ;FrameID 1Ah : unused
    .byte 0x40  ;FrameID 1Bh : unused
    .byte 0x40  ;FrameID 1Ch : unused
    .byte 0x40  ;FrameID 1Dh : unused
    .byte 0x40  ;FrameID 1Eh : unused
    .byte 0x40  ;FrameID 1Fh : unused
    ;
    .byte 0x40  ;FrameID 20h : unused
    .byte 0x40  ;FrameID 21h : unused
    .byte 0x40  ;FrameID 22h : unused
    .byte 0x40  ;FrameID 23h : unused
    .byte 0x40  ;FrameID 24h : unused
    .byte 0x40  ;FrameID 25h : unused
    .byte 0x40  ;FrameID 26h : unused
    .byte 0x40  ;FrameID 27h : unused
    ;
    .byte 0x40  ;FrameID 28h : unused
    .byte 0x40  ;FrameID 29h : unused
    .byte 0x40  ;FrameID 2Ah : unused
    .byte 0x40  ;FrameID 2Bh : unused
    .byte 0x40  ;FrameID 2Ch : unused
    .byte 0x40  ;FrameID 2Dh : unused
    .byte 0x40  ;FrameID 2Eh : unused
    .byte 0x40  ;FrameID 2Fh : unused
    ;
    .byte 0x40  ;FrameID 30h : unused
    .byte 0x40  ;FrameID 31h : unused
    .byte 0x40  ;FrameID 32h : unused
    .byte 0x40  ;FrameID 33h : unused
    .byte 0x40  ;FrameID 34h : unused
    .byte 0x40  ;FrameID 35h : unused
    .byte 0x40  ;FrameID 36h : unused
    .byte 0x40  ;FrameID 37h : unused
    ;
    .byte 0x40  ;FrameID 38h : unused
    .byte 0x40  ;FrameID 39h : unused
    .byte 0x40  ;FrameID 3Ah : unused
    .byte 0x40  ;FrameID 3Bh : unused
    .byte 0x10  ;FrameID 3Ch: MRF (Diagnostic Master Request Frame) - THIS INDEX SHOULD NOT BE CHANGED !!!
    .byte 0x11  ;FrameID 3Dh: SRF (Diagnostic Slave Response Frame) - THIS INDEX SHOULD NOT BE CHANGED !!!
    .byte 0xC0  ;FrameID 3Eh: Reserved
    .byte 0xC0  ;FrameID 3Fh: Reserved

;--------------------------------------------------

;Frame configuration table
;Access : Rom:PARAMtbl[X,A] with index in [X,A] shifted once to the left

;The 16 first Index are reserved for Configurable Message Indices (in RAM)

;Content for Transmit and Receive frames
;   byte 0 : xxx x   x xxx
;               \ \   \   \
;                \ \   \   \
;                 \ \   \   \
;                  \ \   \   length : 0 = 1 bytes... 7 = 8 bytes)
;                   \ \   checksum : 0 = chk13, 1 = chk20
;                    \ not used yet
;                     000 : receive
;                     001 : transmit
;                     010 : reserved
;                     011 : reserved (error)
;                     100 : reserved (ask the application)
;                     101 : discard
;                     110 : reserved (hook)
;                     111 : reserved (error)
;

_PARAMtbl:
#if (LINPROT == LIN22_SIMPLE_PCT)
    .byte 0xA0  ;Message-index 00h : discard
    .byte 0x09  ;Message-index 01h : Control - RX - 2 bytes - chk20
    .byte 0x2B  ;Message-index 02h : Status - TX - 4 bytes - chk20
    .byte 0x2F  ;Message-index 03h : SENSOR - TX - 8 bytes - chk20
#if (_SUPPORT_SPECIAL_COMM_FIELD != FALSE)
    .byte 0x0F  ;Message-index 04h : Control (Special) - RX - 8 bytes - chk20
#else  /* (_SUPPORT_SPECIAL_COMM_FIELD != FALSE) */
    .byte 0xA0  ;Message-index 04h : discard
#endif /* (_SUPPORT_SPECIAL_COMM_FIELD != FALSE) */
#elif (LINPROT == LIN2X_HVAC52)
    .byte 0xA0  ;Message-index 00h : discard
    .byte 0x0F  ;Message-index 01h : CONTROL_ACTx - RX - 8 bytes - chk20 (Message-ID: 0x0001)
    .byte 0x2F  ;Message-index 02h : STATUS_ACTx - TX - 8 bytes - chk20 (Message-ID: 0x0002)
    .byte 0x0F  ;Message-index 03h : GROUP_CONTROL_ACTx - RX - 8 bytes - chk20 (Message-ID: 0x0003)
    .byte 0x2F  ;Message-index 04h : SENSOR - TX - 8 bytes - chk20
#elif (LINPROT == LIN2X_AGS) || (LINPROT == LIN2X_FAN01) || (LINPROT == LIN2X_HVAC49)
    .byte 0xA0  ;Message-index 00h : discard
    .byte 0x0F  ;Message-index 01h : CONTROL_FANx - RX - 8 bytes - chk20 (Message-ID: 0x0001)
    .byte 0x2F  ;Message-index 02h : STATUS_FANx - TX - 8 bytes - chk20 (Message-ID: 0x0002)
    .byte 0x2F  ;Message-index 03h : SENSOR - TX - 8 bytes - chk20
    .byte 0xA0  ;Message-index 04h : discard
#elif (LINPROT == LIN2X_AIRVENT12)
    .byte 0xA0  ;Message-index 00h : discard
    .byte 0x0F  ;Message-index 01h : CONTROL_ACTx - RX - 8 bytes - chk20 (Message-ID: 0x0001)
    .byte 0x2F  ;Message-index 02h : STATUS_ACTx - TX - 8 bytes - chk20 (Message-ID: 0x0002)
    .byte 0x2F  ;Message-index 03h : STATUS1_ACTx - TX - 8 bytes - chk20 (Message-ID: 0x0003)
    .byte 0xA0  ;Message-index 04h : discard
#elif (LINPROT == LIN2X_RELAY) || (LINPROT == LIN2X_SOLENOID)
    .byte 0xA0  ;Message-index 00h : discard
    .byte 0x09  ;Message-index 01h : Control - RX - 2 bytes - chk20
    .byte 0x29  ;Message-index 02h : Status - TX - 2 bytes - chk20
    .byte 0xA0  ;Message-index 03h : discard
    .byte 0xA0  ;Message-index 04h : discard
#elif (LINPROT == LIN13_HVACTB)
    .byte 0xA0  ;Message-index 00h : discard
    .byte 0x01  ;Message-index 01h : CONTROL_FANx - RX - 2 bytes - chk13 (Message-ID: 0x0001)
    .byte 0x21  ;Message-index 02h : STATUS_FANx - TX - 2 bytes - chk13 (Message-ID: 0x0002)
    .byte 0xA0  ;Message-index 03h : discard
    .byte 0xA0  ;Message-index 04h : discard
#elif (LIN_COMM != FALSE)
#error "WARNING: Unsupported LIN Messages"
#else
    .byte 0xA0  ;Message-index 00h : discard
    .byte 0xA0  ;Message-index 01h : discard
    .byte 0xA0  ;Message-index 02h : discard
    .byte 0xA0  ;Message-index 03h : discard
    .byte 0xA0  ;Message-index 04h : discard
#endif
    ;
    .byte 0xA0  ;Message-index 05h : discard
    .byte 0xA0  ;Message-index 06h : discard
    .byte 0xA0  ;Message-index 07h : discard
    ;
    .byte 0xA0  ;Message-index 08h : discard
    .byte 0xA0  ;Message-index 09h : discard
    .byte 0xA0  ;Message-index 0Ah : discard
    .byte 0xA0  ;Message-index 0Bh : discard
    ;
#if (_SUPPORT_MLX_CHIP_STATUS != FALSE)
    .byte 0x2F  ;Message-index 0Ch : MLX Test Status (MMT)
    .byte 0x0F  ;Message-index 0Dh : MLX Test Control (MMT)
#else
    .byte 0xA0  ;Message-index 0Ch : discard
    .byte 0xA0  ;Message-index 0Dh : discard
#endif
    .byte 0xA0  ;Message-index 0Eh : discard
    .byte 0xA0  ;Message-index 0Fh : discard
    ;
    .byte 0x07  ;Message-index 10h : MRF (Diagnostic Master Request Frame) - RX - 8 bytes - chk13 - THIS VALUE SHOULD NOT BE CHANGED !!!
    .byte 0x27  ;Message-index 11h : SRF (Diagnostic Slave Response Frame) - TX - 8 bytes - chk13 - THIS VALUE SHOULD NOT BE CHANGED !!!
    .byte 0xA0  ;Message-index 12h : discard
    .byte 0xA0  ;Message-index 13h : discard
    ;
    .byte 0xA0  ;Message-index 14h : discard
    .byte 0xA0  ;Message-index 15h : discard
    .byte 0xA0  ;Message-index 16h : discard
    .byte 0xA0  ;Message-index 17h : discard
    ;
    .byte 0xA0  ;Message-index 18h : discard
    .byte 0xA0  ;Message-index 19h : discard
    .byte 0xA0  ;Message-index 1Ah : discard
    .byte 0xA0  ;Message-index 1Bh : discard
    ;
    .byte 0xA0  ;Message-index 1Ch : discard
    .byte 0xA0  ;Message-index 1Dh : discard
    .byte 0xA0  ;Message-index 1Eh : discard
    .byte 0xA0  ;Message-index 1Fh : discard
    ;
    .byte 0xA0  ;Message-index 20h : discard
    .byte 0xA0  ;Message-index 21h : discard
    .byte 0xA0  ;Message-index 22h : discard
    .byte 0xA0  ;Message-index 23h : discard
    ;
    .byte 0xA0  ;Message-index 24h : discard
    .byte 0xA0  ;Message-index 25h : discard
    .byte 0xA0  ;Message-index 26h : discard
    .byte 0xA0  ;Message-index 27h : discard
    ;
    .byte 0xA0  ;Message-index 28h : discard
    .byte 0xA0  ;Message-index 29h : discard
    .byte 0xA0  ;Message-index 2Ah : discard
    .byte 0xA0  ;Message-index 2Bh : discard
    ;
    .byte 0xA0  ;Message-index 2Ch : discard
    .byte 0xA0  ;Message-index 2Dh : discard
    .byte 0xA0  ;Message-index 2Eh : discard
    .byte 0xA0  ;Message-index 2Fh : discard
    ;
    .byte 0xA0  ;Message-index 30h : discard
    .byte 0xA0  ;Message-index 31h : discard
    .byte 0xA0  ;Message-index 32h : discard
    .byte 0xA0  ;Message-index 33h : discard
    ;
    .byte 0xA0  ;Message-index 34h : discard
    .byte 0xA0  ;Message-index 35h : discard
    .byte 0xA0  ;Message-index 36h : discard
    .byte 0xA0  ;Message-index 37h : discard
    ;
    .byte 0xA0  ;Message-index 38h : discard
    .byte 0xA0  ;Message-index 39h : discard
    .byte 0xA0  ;Message-index 3Ah : discard
    .byte 0xA0  ;Message-index 3Bh : discard
    ;
    .byte 0xA0  ;Message-index 3Ch : discard
    .byte 0xA0  ;Message-index 3Dh : discard
    .byte 0xA0  ;Message-index 3Eh : discard
    .byte 0xA0  ;Message-index 3Fh : discard

; This part is not used in BL-algorithm. So, it is not included into the ROM.
;--------------------------------------------------

;Cooling Auto Addressing Pulse Table
;Store the values in half bit times between two pulses
;Access : Rom:AUTOADDtbl[X,A] with X = 8

_AUTOADDtbl:    ;   ; Pulse Number
;* 13T AA-LIN           ; Step 0, 2 & 4: Setup Measurement, Step 1, 3 & 5: Perform Measurement
    .byte 0x10  ; 2,  1 ; Step 0 directly (code delay: 0.5T), Step 1 after 0.5T
    .byte 0x26  ; 4,  3 ; Step 2 after 3.0T, Step 3 after 1.0T
    .byte 0x26  ; 6,  5 ; Step 4 after 3.0T, Step 5 after 1.0T
    .byte 0x16  ; 8,  7 ; Step 6 after 3.0T, Step 7 after 0.5T
    .byte 0x00  ;10,  9 ; Total: 0.5(delay) + 0.5 + 3.0 + 1.0 + 3.0 + 1.0 + 3.0 + 0.5 = 12.5T
    .byte 0x00  ;12, 11
    .byte 0x00  ;14, 13
    .byte 0x00  ;16, 15

;--------------------------------------------------
    end
