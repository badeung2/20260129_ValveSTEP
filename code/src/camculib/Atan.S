/*
 * Copyright (C) 2016-2022 Melexis N.V.
 *
 * Pack and Unpack Library
 *
 * Revision $Name:  $
 *
 * File $RCSfile: $
 *
 * ROM: 81340B01-rom-defs.ld
 * _p_atan2I16 = 0x00004924; /* Function */

 */

#include "AppDefines.h"                                                         /* Application defines for C and S files */
#include "AppSwitches.h"                                                        /* Application Switches for C and S files */

/* Validate MLX16-GCC version */
#if ((__MLX16_GCC_MAJOR__ == 1) && (__MLX16_GCC_MINOR__ >= 8)) || (__MLX16_GCC_MAJOR__ > 1)
/* ok */
#else
#warning "Math library requires MLX16-GCC release 1.8 or later"
#endif

    .text
#if (_SUPPORT_ROM_FAST_MATH == FALSE) && _SUPPORT_FAST_ATAN2
    .global _p_atan2I16

#if _SUPPORT_FAST_ATAN2_LOOKUP
;
; _p_atan2I16()
;
; Description:
;    ATAN2 by Look-up table only (w/o CALL/Sub-functions; speed-optimized)
;    Performance difference: 1.175us @ 32MHz = 37-38Cy
; Input:
;    A     Y
;    [S-4] X
;
; Output:
;    A      atan
;
#if _SUPPORT_OPTIMIZE_FOR_SPEED
    .align 3
#endif /* _SUPPORT_OPTIMIZE_FOR_SPEED */
_p_atan2I16:
    cmp  A, #0                      ; [1]           if (y < 0)
    jsge _L_atan_20                 ; [1/2+FWS+CPU] { /* jump: y >= 0; 0 - 180 degrees */
    neg  A                          ; [1]             Y = -Y; /* 180 - 360 degrees */
    lod  X, [S-4]                   ; [1]             if (x < 0)
    jnn  _L_atan_10                 ; [1/2+FWS+CPU]   { /* jump: x >= 0 */
    neg  X                          ; [1]               X = -X /* 180-270 degrees */
    cmp  A, X                       ; [1]           if ( y > x )
    jule _L_atan2_lookup_110        ; [1/2+FWS+CPU]
    mov  Y, X                       ; [1]           { /* Swap X & Y */
    mov  X, A                       ; [1]
    lod  A, #0                      ; [1]
    divu YA, X                      ; [8]           X/Y
    lod  YL, AL                     ; [1]
    lod  YH, #0                     ; [1]           uint16_t u16Index  = (uint16_t)(u16Ratio / LUT_FACTOR);
    asl  Y                          ; [1]           Convert index to Word-table index
    add  Y, #_p_ATAN_LUT            ; [1]           Y = &p_ATAN_LUT[u16Index]
    mov  A, [Y]                     ; [1+2*FWS]     A = p_ATAN_LUT[u16Index]
;    lsr  A, #2                      ; [1]           u16Atan /= 4u /* 0 - 45 degrees */
    neg  A                          ; [1]           /* 0 - -45 degrees */
    add  AH, #0xC0                  ; [1]           } } (Performance improved; MMP200306-2) /* 270 - 225 degrees */
    ret                             ; [5+FWS+CPU]   return ( result )
_L_atan2_lookup_110:                ;               else { /* Y <= X */
;    cmp  X, #0                      ; [1]             if ( x == 0u ) /* X can't be zero */
;    je   _L_atan2_lookup_120        ; [1/2+FWS+CPU]   {
    cmp  A, X                       ; [1]               if ( y == x )
    je   _L_atan2_lookup_130        ; [1/2+FWS+CPU]     {
    mov  Y, A                       ; [1]                 result = p_atan2_helper (y, x);
    lod  A, #0                      ; [1]
    divu YA, X                      ; [8]           Y/X
    lod  YL, AL                     ; [1]
    lod  YH, #0                     ; [1]           uint16_t u16Index  = (uint16_t)(u16Ratio / LUT_FACTOR);
    asl  Y                          ; [1]           Convert index to Word-table index
    add  Y, #_p_ATAN_LUT            ; [1]           Y = &p_ATAN_LUT[u16Index]
    mov  A, [Y]                     ; [1+2*FWS]     A = p_ATAN_LUT[u16Index]
;    lsr  A, #2                      ; [1]           u16Atan /= 4u /* 0 - 45 degrees */
    add  AH, #0x80                  ; [1]           } } (Performance improved; MMP200306-2) /* 180 - 225 degrees */
    ret                             ; [5+FWS+CPU]   return ( result )
;_L_atan2_lookup_120:                ; /* Y == 0 */
;    lod  A, #0x8000                 ; [1]           result = 0U;
;    ret                             ; [5+FWS+CPU]   return ( result )
_L_atan2_lookup_130:                ; /* Y = X */
    lod  A, #0xA000                 ; [1]           result = 0x2000U; /* 225 degrees */
    ret                             ; [5+FWS+CPU]   return ( result )

_L_atan_10:                         ; /* X >= 0; 270-360 degrees */
    cmp  A, X                       ; [1]           if ( y > x )
    jule _L_atan2_lookup_210        ; [1/2+FWS+CPU]
    mov  Y, X                       ; [1]           { /* Swap X & Y */
    mov  X, A                       ; [1]
    lod  A, #0                      ; [1]
    divu YA, X                      ; [8]           X/Y
    lod  YL, AL                     ; [1]
    lod  YH, #0                     ; [1]           uint16_t u16Index  = (uint16_t)(u16Ratio / LUT_FACTOR);
    asl  Y                          ; [1]           Convert index to Word-table index
    add  Y, #_p_ATAN_LUT            ; [1]           Y = &p_ATAN_LUT[u16Index]
    mov  A, [Y]                     ; [1+2*FWS]     A = p_ATAN_LUT[u16Index]
;    lsr  A, #2                      ; [1]           u16Atan /= 4u /* 0 - 45 degrees */
    sub  AH, #0x40                  ; [1]           } (Performance improved; MMP200306-2) /* 270 - 315 degrees */
    ret                             ; [5+FWS+CPU]   return ( result )
_L_atan2_lookup_210:                ;               else { /* Y <= X & Y > 0 */
    cmp  X, #0                      ; [1]             if ( x == 0u )
    je   _L_atan2_lookup_220        ; [1/2+FWS+CPU]   { /* X = Y = 0 */
    cmp  A, X                       ; [1]               if ( y == x )
    je   _L_atan2_lookup_230        ; [1/2+FWS+CPU]     {
    mov  Y, A                       ; [1]                 result = p_atan2_helper (y, x);
    lod  A, #0                      ; [1]
    divu YA, X                      ; [8]           Y/X
    lod  YL, AL                     ; [1]
    lod  YH, #0                     ; [1]           uint16_t u16Index  = (uint16_t)(u16Ratio / LUT_FACTOR);
    asl  Y                          ; [1]           Convert index to Word-table index
    add  Y, #_p_ATAN_LUT            ; [1]           Y = &p_ATAN_LUT[u16Index]
    mov  A, [Y]                     ; [1+2*FWS]     A = p_ATAN_LUT[u16Index]
;    lsr  A, #2                      ; [1]           u16Atan /= 4u /* 0 - 45 degrees */
    neg  A                          ; [1]              result = -result /* 0 - -45 degrees --> 360 - 315 degrees */
    ret                             ; [5+FWS+CPU]   return ( result )
_L_atan2_lookup_220:                ; /* Y == 0 */
    lod  A, #0                      ; [1]           result = 0U;
    ret                             ; [5+FWS+CPU]   return ( result )
_L_atan2_lookup_230:                ; /* Y = X */
    lod  A, #0xE000                 ; [1]           result = -0x2000U; /* 315 degrees */
    ret                             ; [5+FWS+CPU]   return ( result )

_L_atan_20:                         ; else /* A(Y) >= 0; 0 - 180 degrees */
    lod  X, [S-4]                   ; [1]           { if (x < 0)
    jnn  _L_atan_30                 ; [1/2+FWS+CPU]   { /* jump: x >= 0 */
    neg  X                          ; [1]               X = -X /* 90 - 180 degrees */
    cmp  A, X                       ; [1]           if ( y > x )
    jule _L_atan2_lookup_310        ; [1/2+FWS+CPU]
    mov  Y, X                       ; [1]           { /* Swap X & Y */
    mov  X, A                       ; [1]
    lod  A, #0                      ; [1]
    divu YA, X                      ; [8]           X/Y
    lod  YL, AL                     ; [1]
    lod  YH, #0                     ; [1]           uint16_t u16Index  = (uint16_t)(u16Ratio / LUT_FACTOR);
    asl  Y                          ; [1]           Convert index to Word-table index
    add  Y, #_p_ATAN_LUT            ; [1]           Y = &p_ATAN_LUT[u16Index]
    mov  A, [Y]                     ; [1+2*FWS]     A = p_ATAN_LUT[u16Index]
;    lsr  A, #2                      ; [1]           u16Atan /= 4u /* 0 - 45 degrees */
    add  AH, #0x40                  ; [1]           } (Performance improved; MMP200306-2) /* 90 - 135 degrees */
    ret                             ; [5+FWS+CPU]   return ( result )
_L_atan2_lookup_310:                ;               else { /* Y <= X */
;    cmp  X, #0                      ; [1]             if ( x == 0u ) /* X can't be zero */
;    je   _L_atan2_lookup_320        ; [1/2+FWS+CPU]   {
    cmp  A, X                       ; [1]               if ( y == x )
    je   _L_atan2_lookup_330        ; [1/2+FWS+CPU]     {
    mov  Y, A                       ; [1]                 result = p_atan2_helper (y, x);
    lod  A, #0                      ; [1]
    divu YA, X                      ; [8]           Y/X
    lod  YL, AL                     ; [1]
    lod  YH, #0                     ; [1]           uint16_t u16Index  = (uint16_t)(u16Ratio / LUT_FACTOR);
    asl  Y                          ; [1]           Convert index to Word-table index
    add  Y, #_p_ATAN_LUT            ; [1]           Y = &p_ATAN_LUT[u16Index]
    mov  A, [Y]                     ; [1+2*FWS]     A = p_ATAN_LUT[u16Index]
;    lsr  A, #2                      ; [1]           u16Atan /= 4u /* 0 - 45 degrees */
    neg  A                          ; [1]           /* 0 - -45 degrees */
    add  AH, #0x80                  ; [1]           } } (Performance improved; MMP200306-2) /* 180 - 135 degrees */
    ret                             ; [5+FWS+CPU]   return ( result )
;_L_atan2_lookup_320:                ; /* Y == 0 */
;    lod  A, #0x8000                 ; [1]           result = 0U;
;    ret                             ; [5+FWS+CPU]   return ( result )
_L_atan2_lookup_330:                ; /* Y = X */
    lod  A, #0x6000                 ; [1]           result = 0x6000U; /* 135 degrees */
    ret                             ; [5+FWS+CPU]   return ( result )

_L_atan_30:                         ; else /* (A(Y) >= 0) & (X >= 0); 0 - 90 degrees */
    cmp  A, X                       ; [1]           if ( y > x )
    jule _L_atan2_lookup_410        ; [1/2+FWS+CPU]
    mov  Y, X                       ; [1]           { /* Swap X & Y */
    mov  X, A                       ; [1]
    lod  A, #0                      ; [1]
    divu YA, X                      ; [8]           X/Y
    lod  YL, AL                     ; [1]
    lod  YH, #0                     ; [1]           uint16_t u16Index  = (uint16_t)(u16Ratio / LUT_FACTOR);
    asl  Y                          ; [1]           Convert index to Word-table index
    add  Y, #_p_ATAN_LUT            ; [1]           Y = &p_ATAN_LUT[u16Index]
    mov  A, [Y]                     ; [1+2*FWS]     A = p_ATAN_LUT[u16Index]
;    lsr  A, #2                      ; [1]           u16Atan /= 4u /* 0 - 45 degrees */
    neg  A                          ; [1]           /* 0 - -45 degrees */
    add  AH, #0x40                  ; [1]           } (Performance improved; MMP200306-2) /* 90 - 45 degrees */
    ret                             ; [5+FWS+CPU]   return ( result )
_L_atan2_lookup_410:                ;               else { /* Y <= X */
    cmp  X, #0                      ; [1]             if ( x == 0u )
    je   _L_atan2_lookup_420        ; [1/2+FWS+CPU]   { /* X = Y = 0 */
    cmp  A, X                       ; [1]               if ( y == x )
    je   _L_atan2_lookup_430        ; [1/2+FWS+CPU]     {
    mov  Y, A                       ; [1]                 result = p_atan2_helper (y, x);
    lod  A, #0                      ; [1]
    divu YA, X                      ; [8]           Y/X
    lod  YL, AL                     ; [1]
    lod  YH, #0                     ; [1]           uint16_t u16Index  = (uint16_t)(u16Ratio / LUT_FACTOR);
    asl  Y                          ; [1]           Convert index to Word-table index
    add  Y, #_p_ATAN_LUT            ; [1]           Y = &p_ATAN_LUT[u16Index]
    mov  A, [Y]                     ; [1+2*FWS]     A = p_ATAN_LUT[u16Index]
;    lsr  A, #2                      ; [1]           u16Atan /= 4u /* 0 - 45 degrees */
    ret                             ; [5+FWS+CPU]   return ( u16Atan)
_L_atan2_lookup_420:                ; /* Y == 0 */
    lod  A, #0                      ; [1]           result = 0U;
    ret                             ; [5+FWS+CPU]   return ( result )
_L_atan2_lookup_430:                ; /* Y = X */
    lod  A, #0x2000                 ; [1]           result = 0x2000U; /* 45 degrees */
    ret                             ; [5+FWS+CPU]   return ( result )

#else  /* _SUPPORT_FAST_ATAN2_LOOKUP */
;
; _p_atan2I16()
;
; Description:
;    ATAN2 by Look-up table with interpolation (code size optimized)
;
; Input:
;    A     Y
;    [S-4] X
;
; Output:
;    A      atan
_p_atan2I16:
    cmp  A, #0                      ; [1]           if (y < 0)
    jsge _L_atan_20                 ; [1/2+FWS+CPU] {
    neg  A                          ; [1]             Y = -Y;
    lod  X, [S-4]                   ; [1]             if (x < 0)
    jnn  _L_atan_10                 ; [1/2+FWS+CPU]   {
    neg  X                          ; [1]               X = -X
    call _p_atan2_lookup            ; [2+FWS+CPU]       result = (uint16_t)p_atan2_lookup ((uint16_t)y, (uint16_t)x) - 0x8000u
;   add  A, #32768                  ;               } }
    add  AH, #128                   ; [1]           } } (Performance improved; MMP200306-2)
    ret                             ; [5+FWS+CPU]   return ( result )
_L_atan_10:                         ;
    call _p_atan2_lookup            ; [2+FWS+CPU]      else result = p_atan2_lookup ((uint16_t)y, (uint16_t)x)
    neg  A                          ; [1]              result = -result
    ret                             ; [5+FWS+CPU]   return ( result )
_L_atan_20:                         ; else
    lod  X, [S-4]                   ; [1]           { if (x < 0)
    jnn  _L_atan_30                 ; [1/2+FWS+CPU]   {
    neg  X                          ; [1]               X = -X
    call _p_atan2_lookup            ; [2+FWS+CPU]       result = 0x8000u - (uint16_t)p_atan2_lookup ((uint16_t)y, (uint16_t)x); /* pi - x */
    neg  A                          ; [1]
;   add  A, #32768                  ; } }
    add  AH, #128                   ; [1]           } } (Performance improved; MMP200306-2)
    ret                             ; [5+FWS+CPU]   return ( result )
_L_atan_30:                         ;   else
;    call _p_atan2_lookup           ;               result = p_atan2_lookup ((uint16_t)y, (uint16_t)x);
;    ret                            ;               return ( result )
;
; _p_atan2_lookup()
;
; Input:
;    A     Y
;    X     X
;
; Output:
;    A      atan
_p_atan2_lookup:
    cmp  A, X                       ; [1]           if ( y > x )
    jule _L_atan2_lookup_10         ; [1/2+FWS+CPU]
    mov  Y, X                       ; [1]           { /* Swap X & Y */
    mov  X, A                       ; [1]
    call _p_atan2_helper            ; [2+FWS+CPU]     result = (0x4000U - p_atan2_helper (x, y));
    neg  A                          ; [1]
;   add  A, #16384                  ;               }
    add  AH, #64                    ; [1]           } (Performance improved; MMP200306-2)
    ret                             ; [5+FWS+CPU]   return ( result )
_L_atan2_lookup_20:                 ; /* Y == 0 */
    lod  A, #0                      ; [1]           result = 0U;
    ret                             ; [5+FWS+CPU]   return ( result )
_L_atan2_lookup_30:                 ; /* Y = X */
    lod  A, #8192                   ; [1]           result = 0x2000U;
    ret                             ; [5+FWS+CPU]   return ( result )
_L_atan2_lookup_10:                 ;               else { /* Y <= X */
    cmp  X, #0                      ; [1]             if ( x == 0u )
    je   _L_atan2_lookup_20         ; [1/2+FWS+CPU]   {
    cmp  A, X                       ; [1]               if ( y == x )
    je   _L_atan2_lookup_30         ; [1/2+FWS+CPU]     {
    mov  Y, A                       ; [1]                 result = p_atan2_helper (y, x);
;    call _p_atan2_helper            ;              } } }
;    ret                             ;              return ( result )
;
; _p_atan2_helper()
;
; Input:
;    Y     Y
;    X     X
;
; Output:
;    A      atan
; Note: LUT_FACTOR = 256!
; Performance: 30-49 Cy (excl. "ret")
_p_atan2_helper:
    lod  A, #0                      ; [1]
#if 0
; [Calc:22, Meas:?]
    divu YA, X                      ; [8]
    lod  YL, AL                     ; [1]
    lod  YH, #0                     ; [1]           uint16_t u16Index  = (uint16_t)(u16Ratio / LUT_FACTOR);
    asl  Y                          ; [1]           Convert index to Word-table index
    add  Y, #_p_ATAN_LUT            ; [1]           Y = &p_ATAN_LUT[u16Index]
    mov  A, [Y]                     ; [1+2*FWS]     A = p_ATAN_LUT[u16Index]
#elif 0
; [Calc:30, Meas:?]
    divu YA, X                      ; [8]
    divu YA, X                      ; [8]           uint16_t u16Ratio = p_DivU16hi_U16byU16( y, x);
    lod  YL, AH                     ; [1]
    lod  YH, #0                     ; [1]           uint16_t u16Index  = (uint16_t)(u16Ratio / LUT_FACTOR);
    asl  Y                          ; [1]           Convert index to Word-table index
    add  Y, #_p_ATAN_LUT            ; [1]           Y = &p_ATAN_LUT[u16Index]
    mov  A, [Y]                     ; [1+2*FWS]     A = p_ATAN_LUT[u16Index]
#elif 0
; [Calc:42, Meas:43]
    divu YA, X                      ; [8]
    divu YA, X                      ; [8]           uint16_t u16Ratio = p_DivU16hi_U16byU16( y, x);
    lod  YL, AH                     ; [1]
    lod  YH, #0                     ; [1]           uint16_t u16Index  = (uint16_t)(u16Ratio / LUT_FACTOR);
    asl  Y                          ; [1]           Convert index to Word-table index
    add  Y, #_p_ATAN_LUT            ; [1]           Y = &p_ATAN_LUT[u16Index]
;   usex A                          ; [1]           uint16_t u16Interp = (uint16_t)(u16Ratio % LUT_FACTOR);
    mov  X, A                       ; [1]           uint16_t u16Interp = (uint16_t)(u16Ratio % LUT_FACTOR);
    mov  YA, [Y]                    ; [1+3*FWS]     Y = p_ATAN_LUT[u16Index + 1], A = p_ATAN_LUT[u16Index]
    swap YA                         ; [1]           A = p_ATAN_LUT[u16Index + 1], Y = p_ATAN_LUT[u16Index]
    sub  A, Y                       ; [1]           A = p_ATAN_LUT[u16Index + 1] - p_ATAN_LUT[u16Index]
;   mulu A, A, X                    ; [4]           u16Atan = u16Interp * X
    mulu A, AL, XL                  ; [4]           u16Atan = u16Interp * X (Performance improved; MMP200306-2)
    lod  AL, AH                     ; [1]
    mov  AH, #0                     ; [1]           u16Atan /= ((uint16_t) LUT_FACTOR)
    add  A, Y                       ; [1]           u16Atan += p_ATAN_LUT[u16Index]
#elif 1
; [Calc:41, Meas:42]
    divu YA, X                      ; [8]
    divu YA, X                      ; [8]           uint16_t u16Ratio = p_DivU16hi_U16byU16( y, x);
    lod  YL, AH                     ; [1]
    lod  YH, #0                     ; [1]           uint16_t u16Index  = (uint16_t)(u16Ratio / LUT_FACTOR);
    asl  Y                          ; [1]           Convert index to Word-table index
    add  Y, #_p_ATAN_LUT            ; [1]           Y = &p_ATAN_LUT[u16Index]
    mov  X, [Y]                     ; [1+2*FWS]     X = p_ATAN_LUT[u16Index]
    lod  Y, [Y+2]                   ; [1+1*FWS]     Y = p_ATAN_LUT[u16Index + 1]
    sub  Y, X                       ; [1]           Y = p_ATAN_LUT[u16Index + 1] - p_ATAN_LUT[u16Index]
;   usex A                          ; [1]           uint16_t u16Interp = (uint16_t)(u16Ratio % LUT_FACTOR);
;   mulu A, A, Y                    ; [4]           u16Atan = u16Interp * Y
    mulu A, AL, YL                  ; [4]           u16Atan = u16Interp * Y (Performance improved; MMP200306-2)
    lod  AL, AH                     ; [1]
    mov  AH, #0                     ; [1]           u16Atan /= ((uint16_t) LUT_FACTOR)
    add  A, X                       ; [1]           u16Atan += p_ATAN_LUT[u16Index]
#elif 0
; [Calc:49, Meas:49]
    divu YA, X                      ; [8]
    divu YA, X                      ; [8]           uint16_t u16Ratio = p_DivU16hi_U16byU16( y, x);
    lod  YL, AH                     ; [1]
    lod  YH, #0                     ; [1]           uint16_t u16Index  = (uint16_t)(u16Ratio / LUT_FACTOR);
    asl  Y                          ; [1]           Convert index to Word-table index
    add  Y, #_p_ATAN_LUT            ; [1]           Y = &p_ATAN_LUT[u16Index]
    lod  X, [Y+2]                   ; [1+2*FWS]     X = p_ATAN_LUT[u16Index + 1]
    sub  X, [Y]                     ; [1+2*FWS]     X = p_ATAN_LUT[u16Index + 1] - p_ATAN_LUT[u16Index]
;   usex A                          ; [1]           uint16_t u16Interp = (uint16_t)(u16Ratio % LUT_FACTOR);
;   mulu A, A, X                    ; [4]           u16Atan = u16Interp * X
    mulu A, AL, XL                  ; [4]           u16Atan = u16Interp * X (Performance improved; MMP200306-2)
    lod  AL, AH                     ; [1]
    mov  AH, #0                     ; [1]           u16Atan /= ((uint16_t) LUT_FACTOR)
    add  A, [Y]                     ; [1+2*FWS]     u16Atan += p_ATAN_LUT[u16Index]
#endif
;    lsr  A, #2                      ; [1]           u16Atan /= 4u (MMP230830-1)
    ret                             ; [5+FWS+CPU]   return ( u16Atan)

#endif /* _SUPPORT_FAST_ATAN2_LOOKUP */
#endif /* (_SUPPORT_ROM_FAST_MATH == FALSE) && _SUPPORT_FAST_ATAN2 */

; EOF
