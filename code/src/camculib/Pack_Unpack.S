/*
 * Copyright (C) 2016-2022 Melexis N.V.
 *
 * Pack and Unpack Library
 *
 * Revision $Name:  $
 *
 * File $RCSfile: $
 *
 *
 */


/* Validate MLX16-GCC version */
#if ((__MLX16_GCC_MAJOR__ == 1) && (__MLX16_GCC_MINOR__ >= 8)) || (__MLX16_GCC_MAJOR__ > 1)
/* ok */
#else
#warning "Math library requires MLX16-GCC release 1.8 or later"
#endif

    .text
    .global _p_Pack
    .global _p_Unpack
;
; _p_Unpack()
;
; Input:
;    A     u16PtrDest
;    [S-6] u16Len
;    [S-4] u16PtrSrc
;
; Output:
;    Nothing
_p_Pack:
    mov  X, A                   ; X = Destination-pointer (packed 3-byte, 4 character array)
    mov  Y, [S-4]               ; Y = Source-pointer
    mov  A, [S-6]               ; Number of characters
    push Y
    add  Y, A
_L_Pack_Loop:
    lod  AL, [--Y]              ; AL = 1st character
    add  AL, #-' '              ; Pack (0x20..0x5F --> 0x00..0x3F)
    cmp  Y, [S-2]               ; Check against begin of array
    je   _L_Pack_Done           ; Last character
    lod  AH, [--Y]              ; AH = 2nd character
    add  AH, #-' '              ; Pack (0x20..0x5F --> 0x00..0x3F)
    rr   AH, #2                 ; AH[1:0] --> AH[7:6] & AH[5:2] --> AH[4:0]
    push A
    and  A, #0xC03F             ; Isolate 2nd character[1:0]
    or   AL, AH
    mov  [X++], AL              ; Save first packed character[5:0] + 2nd character [1:0]
    pop  A
    mov  AL, AH
    and  AL, #0x0F              ; Isolate 2nd (AL) character[5:2]
    cmp  Y, [S-2]               ; Check against begin of array
    je   _L_Pack_Done           ; Last character
    lod  AH, [--Y]              ; AL = 3rd character
    add  AH, #-' '              ; Pack (0x20..0x5F --> 0x00..0x3F)
    rr   AH, #2                 ; AH[5:4] --> AH[1:0]
    rr   AH, #2                 ; AH[3:0] --> AH[7:4]
    push A
    and  AH, #0xF0              ; Isolate 3rd (AH) character[3:0]
    or   AL, AH
    mov  [X++], AL              ; Save 2nd character [5:2] + 3rd character [3:0]
    pop  A
    mov  AL, AH
    and  AL, #0x03              ; Isolate 3rd character[5:4]
    cmp  Y, [S-2]               ; Check against begin of array
    je   _L_Pack_Done           ; Last character
    lod  AH, [--Y]              ; AH = 4th character
    add  AH, #-' '              ; Pack (0x20..0x5F --> 0x00..0x3F)
    lsl  AH, #2                 ; AH[5:0] --> AH[7:2]
    or   AL, AH
    mov  [X++], AL              ; Save 3rd character [5:4] + 4th character [5:0]
    cmp  Y, [S-2]               ; Check against begin of array
    jne  _L_Pack_Loop           ; Still more characters to pack
    pop  A
    ret
_L_Pack_Done:
    mov  [X++], AL
    pop  A
    ret
;
; _p_Unpack()
;
; Input:
;    A     u16PtrDest
;    [S-6] u16Len
;    [S-4] u16PtrSrc
;
; Output:
;    A      u16PtrDest
_p_Unpack:
    mov  Y, A                   ; Y = Destination-pointer
    mov  X, [S-4]               ; X = Source-pointer (packed 3-byte, 4 character array)
    mov  A, [S-6]               ; A = number of characters
    add  A, #3                  ; Convert X to 'end' source-pointer
    lsr  A, #2
    add  A, #-1
    add  X, A
    add  A, A
    add  X, A
    mov  A, [S-6]               ; A = number of characters
    and  A, #3                  ; packed 4 characters in three bytes
    jz   _L_Unpack_10           ; Use all (packed) characters
    add  A, #-1
    jz   _L_Unpack_40           ; Use of last packed-structure only 1 character
    add  A, #-1
    jz   _L_Unpack_30           ; Use of last packed-structure only 2 characters
    jmp  _L_Unpack_20           ; Use of last packed-structure only 3 characters
_L_Unpack_10:
    lod  AL, [X+2]              ; Take last character of packed-structure
    lsr  AL, #2                 ; Isolate character
    add  AL, #' '               ; Convert to ASCII ' ' to '_'
    mov  [Y++], AL              ; Store ASCII character in buffer
_L_Unpack_20:
    lod  AH, [X+2]              ; Take 3rd character 2 upper bits
    and  AH, #0x03
    lod  AL, [X+1]              ; Take 3rd character 4 lower bits
    and  AL, #0xF0
    or   AL, AH                 ; Combine together
    rl   AL, #2                 ; Rotate to position
    rl   AL, #2
    add  AL, #' '               ; Convert to ASCII ' ' to '_'
    mov  [Y++], AL              ; Store ASCII character in buffer
_L_Unpack_30:
    lod  AH, [X+1]              ; Take 2nd character 4 upper bits
    and  AH, #0x0F
    lod  AL, [X]                ; Take 2nd character 2 lower bits
    and  AL, #0xC0
    or   AL, AH                 ; Combine together
    rl   AL, #2                 ; Rotate to position
    add  AL, #' '               ; Convert to ASCII ' ' to '_'
    mov  [Y++], AL              ; Store ASCII character in buffer
_L_Unpack_40:
    lod  AL, [X]                ; Take 1st character
    and  AL, #0x3F              ; Isolate 1st character
    add  AL, #' '               ; Convert to ASCII ' ' to '_'
    mov  [Y++], AL              ; Store ASCII character in buffer
    add  X, #-3                 ; Update source packed-structure pointer to previous array element
    cmp  X, [S-4]               ; Check against begin of array
    jnc  _L_Unpack_10           ; Still more characters in array
    mov  A, Y                   ; Return updated destination pointer
    ret
;
; EOF
