/*
 * Copyright (C) 2016-2022 Melexis N.V.
 *
 * Private Math Library
 *
 * Revision $Name:  20200310$
 *
 * File $RCSfile: $
 *
 * Functions:   p_CpyU32_U32()
 *              p_CpyU32p_U8p()
 *              p_CalcCRC_U8()
 *              p_CalcCRC_U16()
 *              p_AddU32_U32byI16()
 *              p_AddU32_U32byU16()
 *              p_SubU32_U32byI16()
 *              p_SubU32_U32byU16()
 *              p_AddU32byU16()
 *              p_SubU32byU16()
 *              p_MulI16hi_I16byI16()
 *              p_MulI16_I16byQ15()
 *              p_MulI16_I16bypQ15()
 *              p_MulU16hi_U16byU16()
 *              p_MulI32_I16byI16()
 *              p_MulU32_U16byU16()
 *              p_MulU32_U16byU16lsr8()
 *              p_MulU32_U16byU16lsr10()
 *              p_MulU16lo_U16byU16()
 *              p_MulDivI16_I16byI16byI16()
 *              p_MulDivU16_U16byU16byU16()
 *              p_MulU16lo_U16byU16lsr1()
 *              p_MulI16hi_I16byI16asr3()
 *              p_MulU16hi_U16byU16lsr3()
 *              p_MulI16hi_I16byI16asr4()
 *              p_MulU16hi_U16byU16lsr4()
 *              p_MulU16hi_pU16byU16lsr4()
 *              p_MulI16hi_I16byI16asr5()
 *              p_MulU16hi_U16byU16lsr5()
 *              p_MulI16lo_I16byI16asr6Rnd()
 *              p_MulI16lo_I16byI16asr8Rnd()
 *              p_DivI16_I32byI16()
 *              p_DivU16_U32byU16()
 *              p_ModU16_U32byU16()
 *              p_DivU32_U32byU16()
 *              p_DivI16_I16hibyI16()
 *              p_DivU16_U16hibyU16()
 *              p_LpfU16_I16byI16()
 *              p_LpfI16_I16byI16()
 *              p_Abs16()
 *              p_Abs32()
 *              p_BitRev8()
 *              p_MemSet()
 *              p_MemCpyU16()
 *              p_MemSwapU8()
 *              p_Access()
 *              p_PID_Control()
 *              p_iPID_Control()
 *              p_ClipMinMaxI16()
 *              p_ClipMinMaxU16()
 *              p_IdxToAngle()
 *              p_AproxSqrtU16_I16byI16()
 *              p_GetExMem()
 *              p_DecNzU8()
 *              p_DecNzU16()
 *              GET_STACK()
 *              p_AwdAck()
 *              p_I2C_MasterSendData()
 *              p_I2C_MasterReceiveData()
 */

#ifndef DRIVE_LIB_PRIVATE_MATHLIB_H_
#define DRIVE_LIB_PRIVATE_MATHLIB_H_

#include <mlx16_cfg.h>

/* Validate MLX16-GCC version */
#if ((__MLX16_GCC_MAJOR__ == 1) && (__MLX16_GCC_MINOR__ >= 8)) || (__MLX16_GCC_MAJOR__ > 1)
/* ok */
#else
#warning "Math library requires MLX16-GCC release 1.8 or later"
#endif

extern void p_Pack(uint16_t u16Dest, uint16_t u16Src, uint16_t u16Len);
extern uint16_t p_Unpack(uint16_t u16Dest, uint16_t u16Src, uint16_t u16Len);
extern int16_t p_atan2I16(uint16_t u16Y, uint16_t u16X);
extern uint16_t ConvBase8Exp3toU16(uint8_t u8Base, uint8_t u3Exp);
extern uint16_t ConvU16toBase8Exp3(uint16_t u16Value);
extern uint32_t ConvBase14Exp2toU32(uint16_t u14Base, uint8_t u2Exp);
extern uint16_t ConvU32toBase14Exp2(uint32_t u32Value);

/* ************************************************************************** *
 * Function : p_CpyU32_U32()                                                  *
 * Purpose  : Copy a (16-bit aligned) u32-variable (atomically) and save in   *
 *            another (16-bit aligned) unsigned 32-bit variable               *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : const uint32_t *u32PtrSrc: Pointer to unsigned 32-bit variable  *
 *            uint32_t *u32PtrDest: Pointer to unsigned 32-bit storage        *
 *            variable.                                                       *
 * Return   : Nothing                                                         *
 * ************************************************************************** *
 * Comments : (uint32_t)[u32PtrDest] = (uint32_t)[u32PtrSrc]                  *
 * ************************************************************************** */
static __inline__ void p_CpyU32_U32(const uint32_t *u32PtrSrc, uint32_t *u32PtrDest) __attribute__ ((always_inline));
static __inline__ void p_CpyU32_U32(const uint32_t *u32PtrSrc, uint32_t *u32PtrDest)
{
    __asm__ __volatile__ (
        "mov D, [X] \n\t"
        "mov [Y], D"
        :
        : "x" (u32PtrSrc), "y" (u32PtrDest)
        : "D"
        );
} /* End of p_CpyU32_U32() */

/* ************************************************************************** *
 * Function : p_CpyU32p_U8p()                                                 *
 * Purpose  : Copy a 16-bit aligned u32-variable (atomically) to a 8-bit      *
 *            aligned data-buffer                                             *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : const uint32_t *u32PtrSrc: Pointer to unsigned 32-bit variable  *
 *            uint8_t *u8PtrDest: Pointer to unsigned 8-bit storage buffer    *
 * Return   : (uint8_t) Updated storage buffer pointer (end-of-buffer)        *
 * ************************************************************************** *
 * Comments : (uint8_t*)[u8PtrDest] = (uint32_t)[u32PtrSrc]                   *
 * ************************************************************************** */
static __inline__ uint8_t* p_CpyU32p_U8p(const uint32_t *u32PtrSrc, uint8_t *u8PtrDest) __attribute__ ((always_inline));
static __inline__ uint8_t* p_CpyU32p_U8p(const uint32_t *u32PtrSrc, uint8_t *u8PtrDest)
{
    uint16_t u16Rubbish; /*lint -e529 */                                        /* Clobbering of the register */

    __asm__ __volatile__ (
        "mov YA, [Y] \n\t"
        "mov [X++], AL \n\t"
        "mov [X++], AH \n\t"
        "mov [X++], YL \n\t"
        "mov [X++], YH \n\t"
        : "+x" (u8PtrDest), "=y" (u16Rubbish), "=a" (u16Rubbish)
        : "y" (u32PtrSrc)
        );
    return (u8PtrDest);
} /* End of p_CpyU32_U32() */

/* ************************************************************************** *
 * Function : p_CalcCRC_U8()                                                  *
 * Purpose  : Calculate an 8-bit CRC based on sum with overflow (CY)          *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : const uint16_t *pu16BeginAddress: Begin-address of area         *
 *            const uint16_t u16Length: Length of area (in 16-bits words)     *
 * Return   : (uint16_t) CRC8 result                                          *
 * ************************************************************************** *
 * Comments :                                                                 *
 * ************************************************************************** */
static __inline__ uint16_t p_CalcCRC_U8(const uint16_t *pu16BeginAddress,
                                        uint16_t u16LengthW) __attribute__ ((always_inline));
static __inline__ uint16_t p_CalcCRC_U8(const uint16_t *pu16BeginAddress, uint16_t u16LengthW)
{
    uint16_t u16Result = 0U;

    __asm__ /*__volatile__*/ (
        "clrb ML.7 \n\t"                                                        /* Clear CY */
        "p_CRC_U8_10_%=: \n\t"
        "adc %[csum], [%[addr]++] \n\t"                                         /* Add value to CRC */
        "djnz %[len], p_CRC_U8_10_%= \n\t"
        "adc %b[csum], %h[csum] \n\t"                                           /* Add CRC-H to CRC-L, including CY */
        "adc %b[csum], #0 \n\t"                                                 /* Make sure no more CY */
        "usex %[csum]"                                                          /* Unsigned extend */
        : [csum] "+a" (u16Result), [addr] "+y" (pu16BeginAddress), [len] "+x" (u16LengthW)
        );
    return (u16Result);
} /* End of p_CalcCRC_U8() */

/* ************************************************************************** *
 * Function : p_CalcCRC_U16()                                                 *
 * Purpose  : Calculate an 16-bit CRC based on sum with overflow (CY)         *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : const uint16_t *pu16BeginAddress: Begin-address of area         *
 *            const uint16_t u16Length: Length of area                        *
 * Return   : (uint16_t) CRC16 result                                         *
 * ************************************************************************** *
 * Comments :                                                                 *
 * ************************************************************************** */
static __inline__ uint16_t p_CalcCRC_U16(const uint16_t *pu16BeginAddress,
                                         uint16_t u16Length) __attribute__ ((always_inline));
static __inline__ uint16_t p_CalcCRC_U16(const uint16_t *pu16BeginAddress, uint16_t u16Length)
{
    uint16_t u16Result = 0U;

    __asm__ __volatile__ (
        "clrb ML.7 \n\t"                                                        /* Clear CY */
        "p_CRC_U16_10_%=: \n\t"
        "adc %[csum], [%[addr]++]  \n\t"                                        /* Add value to CRC */
        "djnz %[len], p_CRC_U16_10_%= \n\t"
        "adc %[csum], #0  \n\t"                                                 /* Add CY to CSUM (add no more CY) */
        : [csum] "+a" (u16Result), [addr] "+y" (pu16BeginAddress), [len] "+x" (u16Length)
        );
    return (u16Result);
} /* End of p_CalcCRC_U16() */

/* ************************************************************************** *
 * Function : p_AddU32_U32byI16()                                             *
 * Purpose  : Add signed extended 16-bit value to unsigned 32-bit variable    *
 *            and save in another unsigned 32-bit variable                    *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint32_t *u32PtrDest: Pointer to unsigned 32-bit storage        *
 *            variable.                                                       *
 *            const uint32_t *u32PtrSrc: Pointer to unsigned 32-bit variable  *
 *            to which the unsigned extended 16-bit value should be added     *
 *            int16_t i16Value: 16-bit signed value                           *
 * Return   : Nothing                                                         *
 * ************************************************************************** *
 * Comments : (uint32_t)[u32PtrDest] = (uint32_t)[u32PtrSrc] +                *
 *                                     (int32_t) ((int16_t) i16Value)         *
 * ************************************************************************** */
static __inline__ void p_AddU32_U32byI16(uint32_t *u32PtrDest, const uint32_t *u32PtrSrc,
                                         int16_t i16Value) __attribute__ ((always_inline));
static __inline__ void p_AddU32_U32byI16(uint32_t *u32PtrDest, const uint32_t *u32PtrSrc, int16_t i16Value)
{
    __asm__ __volatile__ (
        "mov D, [X] \n\t"
        "adds D, %[value] \n\t"
        "mov [Y], D"
        :
        : [value] "na" (i16Value), "x" (u32PtrSrc), "y" (u32PtrDest)
        : "D"
        );
} /* End of p_AddU32_U32byI16() */

/* ************************************************************************** *
 * Function : p_AddU32_U32byU16()                                             *
 * Purpose  : Add unsigned extended 16-bit value to unsigned 32-bit variable  *
 *            and save in another unsigned 32-bit variable                    *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint32_t *u32PtrDest: Pointer to unsigned 32-bit storage        *
 *            variable.                                                       *
 *            const uint32_t *u32PtrSrc: Pointer to unsigned 32-bit variable  *
 *            to which the unsigned extended 16-bit value should be added     *
 *            uint16_t u16Value: 16-bit unsigned value                        *
 * Return   : Nothing                                                         *
 * ************************************************************************** *
 * Comments : (uint32_t)[u32PtrDest] = (uint32_t)[u32PtrSrc] +                *
 *                                     (uint32_t) ((uint16_t) u16Value)       *
 * ************************************************************************** */
static __inline__ void p_AddU32_U32byU16(uint32_t *u32PtrDest, const uint32_t *u32PtrSrc,
                                         uint16_t u16Value) __attribute__ ((always_inline));
static __inline__ void p_AddU32_U32byU16(uint32_t *u32PtrDest, const uint32_t *u32PtrSrc, uint16_t u16Value)
{
    __asm__ __volatile__ (
        "mov D, [X] \n\t"
        "addu D, %[value] \n\t"
        "mov [Y], D"
        :
        : [value] "na" (u16Value), "x" (u32PtrSrc), "y" (u32PtrDest)
        : "D"
        );
} /* End of p_AddU32_U32byU16() */

/* ************************************************************************** *
 * Function : p_SubU32_U32byI16()                                             *
 * Purpose  : Subtract signed extended 16-bit value to unsigned 32-bit        *
 *            variable and save in another unsigned 32-bit variable           *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint32_t *u32PtrDest: Pointer to unsigned 32-bit storage        *
 *            variable.                                                       *
 *            const uint32_t *u32PtrSrc: Pointer to unsigned 32-bit variable  *
 *            to which the unsigned extended 16-bit value should be subtracted*
 *            int16_t i16Value: 16-bit signed value                           *
 * Return   : Nothing                                                         *
 * ************************************************************************** *
 * Comments : (uint32_t)[u32PtrDest] = (uint32_t)[u32PtrSrc] -                *
 *                                     (int32_t) ((int16_t) i16Value)         *
 * ************************************************************************** */
static __inline__ void p_SubU32_U32byI16(uint32_t *u32PtrDest, const uint32_t *u32PtrSrc,
                                         int16_t i16Value) __attribute__ ((always_inline));
static __inline__ void p_SubU32_U32byI16(uint32_t *u32PtrDest, const uint32_t *u32PtrSrc, int16_t i16Value)
{
    __asm__ __volatile__ (
        "mov D, [X] \n\t"
        "subs D, %[value] \n\t"
        "mov [Y], D"
        :
        : [value] "na" (i16Value), "x" (u32PtrSrc), "y" (u32PtrDest)
        : "D"
        );
} /* End of p_SubU32_U32byI16() */

/* ************************************************************************** *
 * Function : p_SubU32_U32byU16()                                             *
 * Purpose  : Subtract unsigned extended 16-bit value to unsigned 32-bit      *
 *            variable and save in another unsigned 32-bit variable           *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint32_t *u32PtrDest: Pointer to unsigned 32-bit storage        *
 *            variable.                                                       *
 *            const uint32_t *u32PtrSrc: Pointer to unsigned 32-bit variable  *
 *            to which the unsigned extended 16-bit value should be subtracted*
 *            uint16_t u16Value: 16-bit unsigned value                        *
 * Return   : Nothing                                                         *
 * ************************************************************************** *
 * Comments : (uint32_t)[u32PtrDest] = (uint32_t)[u32PtrSrc] -                *
 *                                     (uint32_t) ((uint16_t) u16Value)       *
 * ************************************************************************** */
static __inline__ void p_SubU32_U32byU16(uint32_t *u32PtrDest, const uint32_t *u32PtrSrc,
                                         uint16_t u16Value) __attribute__ ((always_inline));
static __inline__ void p_SubU32_U32byU16(uint32_t *u32PtrDest, const uint32_t *u32PtrSrc, uint16_t u16Value)
{
    __asm__ __volatile__ (
        "mov D, [X] \n\t"
        "subu D, %[value] \n\t"
        "mov [Y], D"
        :
        : [value] "na" (u16Value), "x" (u32PtrSrc), "y" (u32PtrDest)
        : "D"
        );
} /* End of p_SubU32_U32byU16() */

/* ************************************************************************** *
 * Function : p_AddU32byI16()                                                 *
 * Purpose  : Add signed extended 16-bit value to unsigned 32-bit variable    *
 *            and save in same unsigned 32-bit variable                       *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint32_t *u32Ptr: Pointer to unsigned 32-bit variable to which  *
 *            the unsigned extended 16-bit value should be added (and stored) *
 *            int16_t i16Value: 16-bit signed value                           *
 * Return   : Nothing                                                         *
 * ************************************************************************** *
 * Comments : (uint32_t)[u32Ptr] = (uint32_t)[u32Ptr] +                       *
 *                                 (int32_t) ((int16_t) i16Value)             *
 * ************************************************************************** */
static __inline__ void p_AddU32byI16(uint32_t *u32Ptr, int16_t i16Value) __attribute__ ((always_inline));
static __inline__ void p_AddU32byI16(uint32_t *u32Ptr, int16_t i16Value)
{
    __asm__ __volatile__ (
        "mov D, [X] \n\t"
        "adds D, %[value] \n\t"
        "mov [X], D"
        :
        : [value] "na" (i16Value), "x" (u32Ptr)
        : "D"
        );
} /* End of p_AddU32byI16() */

/* ************************************************************************** *
 * Function : p_AddU32byU16()                                                 *
 * Purpose  : Add unsigned extended 16-bit value to unsigned 32-bit variable  *
 *            and save in same unsigned 32-bit variable                       *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint32_t *u32Ptr: Pointer to unsigned 32-bit variable to which  *
 *            the unsigned extended 16-bit value should be added (and stored) *
 *            uint16_t u16Value: 16-bit unsigned value                        *
 * Return   : Nothing                                                         *
 * ************************************************************************** *
 * Comments : (uint32_t)[u32Ptr] = (uint32_t)[u32Ptr] +                       *
 *                                 (uint32_t) ((uint16_t) u16Value)           *
 * ************************************************************************** */
static __inline__ void p_AddU32byU16(uint32_t *u32Ptr, uint16_t u16Value) __attribute__ ((always_inline));
static __inline__ void p_AddU32byU16(uint32_t *u32Ptr, uint16_t u16Value)
{
    __asm__ __volatile__ (
        "mov D, [X] \n\t"
        "addu D, %[value] \n\t"
        "mov [X], D"
        :
        : [value] "na" (u16Value), "x" (u32Ptr)
        : "D"
        );
} /* End of p_AddU32byU16() */

/* ************************************************************************** *
 * Function : p_SubU32byI16()                                                 *
 * Purpose  : Subtract signed extended 16-bit value to unsigned 32-bit        *
 *            variable and save in same unsigned 32-bit variable              *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint32_t *u32Ptr: Pointer to unsigned 32-bit variable to which  *
 *            the unsigned extended 16-bit value should be subtracted (and    *
 *            stored)                                                         *
 *            uint16_t u16Value: 16-bit unsigned value                        *
 * Return   : Nothing                                                         *
 * ************************************************************************** *
 * Comments : (uint32_t)[u32Ptr] = (uint32_t)[u32Ptr] -                       *
 *                                 (int32_t) ((int16_t) i16Value)             *
 * ************************************************************************** */
static __inline__ void p_SubU32byI16(uint32_t *u32Ptr, int16_t i16Value) __attribute__ ((always_inline));
static __inline__ void p_SubU32byI16(uint32_t *u32Ptr, int16_t i16Value)
{
    __asm__ __volatile__ (
        "mov D, [X] \n\t"
        "subs D, %[value] \n\t"
        "mov [X], D"
        :
        : [value] "ny" (i16Value), "x" (u32Ptr)
        : "D"
        );
} /* End of p_SubU32byI16() */

/* ************************************************************************** *
 * Function : p_SubU32byU16()                                                 *
 * Purpose  : Subtract unsigned extended 16-bit value to unsigned 32-bit      *
 *            variable and save in same unsigned 32-bit variable              *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint32_t *u32Ptr: Pointer to unsigned 32-bit variable to which  *
 *            the unsigned extended 16-bit value should be subtracted (and    *
 *            stored)                                                         *
 *            uint16_t u16Value: 16-bit unsigned value                        *
 * Return   : Nothing                                                         *
 * ************************************************************************** *
 * Comments : (uint32_t)[u32Ptr] = (uint32_t)[u32Ptr] -                       *
 *                                 (uint32_t) ((uint16_t) u16Value)           *
 * ************************************************************************** */
static __inline__ void p_SubU32byU16(uint32_t *u32Ptr, uint16_t u16Value) __attribute__ ((always_inline));
static __inline__ void p_SubU32byU16(uint32_t *u32Ptr, uint16_t u16Value)
{
    __asm__ __volatile__ (
        "mov D, [X] \n\t"
        "subu D, %[value] \n\t"
        "mov [X], D"
        :
        : [value] "ny" (u16Value), "x" (u32Ptr)
        : "D"
        );
} /* End of p_SubU32byU16() */

/* ************************************************************************** *
 * Function : p_MulU32byU16()                                                 *
 * Purpose  : Multiply unsigned 32-bit variable with a unsigned 16-bit        *
 *            variable and return it in the same unsigned 32-bit variable     *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint32_t a: unsigned 32-bit variable                            *
 *            uint16_t b: 16-bit unsigned value                               *
 * Return   : uint32_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint32_t) = (uint32_t)a * (uint16_t) b                         *
 * ************************************************************************** */
static __inline__ uint32_t p_MulU32byU16(uint32_t a, uint16_t b) __attribute__ ((always_inline));
static __inline__ uint32_t p_MulU32byU16(uint32_t a, uint16_t b)
{
    uint32_t u32Result;  /*lint -e530 */

    __asm__ __volatile__ (
        "mulu D, A, %[b] \n\t"
        "lsr D, #16 \n\t"
        "swap YA \n\t"
        "macu D, A, %[b] \n\t"
        "mov YA, D"
        : "=b" (u32Result)
        : "b" (a), [b] "nx" (b)
        : "D"
        );
    return (u32Result);
} /* End of p_MulU32byU16() */

/* ************************************************************************** *
 * Function : p_MulI16hi_I16byI16()                                           *
 * Purpose  : Multiply signed 16-bit variable with a signed 16-bit            *
 *            variable and return the upper 15-bit                            *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t a: signed 16-bit variable 'a'                           *
 *            int16_t b: 16-bit signed value 'b'                              *
 * Return   : int16_t: Return value                                           *
 * ************************************************************************** *
 * Comments : (int16_t) = (int16_t) ((int32_t) (a * b)) >> 16)                *
 *            Private mulI16_I16byI16 is about 0.4 us faster then ROM code,   *
 *            and one 16-bit word shorter                                     *
 * ************************************************************************** */
static __inline__ int16_t p_MulI16hi_I16byI16(int16_t a, int16_t b) __attribute__ ((always_inline));
static __inline__ int16_t p_MulI16hi_I16byI16(int16_t a, int16_t b)
{
    int16_t i16Result;  /*lint -e530 */
    int16_t i16Rubbish; /*lint -e529 */                                           /* Clobbering of the register */

    __asm__ __volatile__ (
        "muls YA, A, %[b]"
        : "=y" (i16Result), "=a" (i16Rubbish)
        : "%a" (a), [b] "nr" (b)
        );

    return (i16Result);
} /* End of p_MulI16hi_I16byI16() */

/* ************************************************************************** *
 * Function : p_MulI16_I16byQ15()                                             *
 * Purpose  : Multiply signed 16-bit variable with a signed 16-bit            *
 *            variable and return the upper 15-bit                            *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t a: signed 16-bit variable 'a'                           *
 *            int16_t b: 16-bit signed value 'b'                              *
 * Return   : int16_t: Return value                                           *
 * ************************************************************************** *
 * Comments : (int16_t) = (int16_t) ((int32_t) (a * b)) >> 15)                *
 *            Private mulI16_I16byI16 is about 0.4 us faster then ROM code,   *
 *            and one 16-bit word shorter                                     *
 *            Multiplier could be invalid (rubbish)                           *
 * ************************************************************************** */
static __inline__ int16_t p_MulI16_I16byQ15(int16_t a, int16_t b) __attribute__ ((always_inline));
static __inline__ int16_t p_MulI16_I16byQ15(int16_t a, int16_t b)
{
    int16_t i16Result;  /*lint -e530 */
    int16_t i16Rubbish; /*lint -e529 */                                           /* Clobbering of the register */

    __asm__ __volatile__ (
        "muls YA, A, %[b] \n\t"
        "asl YA, #1"
        : "=y" (i16Result), "=a" (i16Rubbish)
        : "%a" (a), [b] "nr" (b)
        );

    return (i16Result);
} /* End of p_MulI16_I16byQ15() */

/* ************************************************************************** *
 * Function : p_MulI16_I16byQ15x()                                            *
 * Purpose  : Multiply signed 16-bit variable with a signed 16-bit            *
 *            variable and return the upper 15-bit                            *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t a: signed 16-bit variable 'a'                           *
 *            int16_t b: 16-bit signed value 'b' (in reg x)                   *
 * Return   : int16_t: Return value                                           *
 * ************************************************************************** *
 * Comments : (int16_t) = (int16_t) ((int32_t) (a * b)) >> 15)                *
 *            Private mulI16_I16byI16 is about 0.4 us faster then ROM code,   *
 *            and one 16-bit word shorter                                     *
 *            Multiplier remains valid                                        *
 * ************************************************************************** */
static __inline__ int16_t p_MulI16_I16byQ15x(int16_t a, int16_t b) __attribute__ ((always_inline));
static __inline__ int16_t p_MulI16_I16byQ15x(int16_t a, int16_t b)
{
    int16_t i16Result;  /*lint -e530 */
    int16_t i16Rubbish; /*lint -e529 */                                           /* Clobbering of the register */

    __asm__ __volatile__ (
        "muls YA, A, %[b] \n\t"
        "asl YA, #1"
        : "=y" (i16Result), "=a" (i16Rubbish)
        : "%a" (a), [b] "nx" (b)
        );

    return (i16Result);
} /* End of p_MulI16_I16byQ15x() */

/* ************************************************************************** *
 * Function : p_MulI16_I16bypQ15()                                            *
 * Purpose  : Multiply signed 16-bit variable with a pointed signed 16-bit    *
 *            variable and return the upper 15-bit                            *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t a: signed 16-bit variable 'a'                           *
 *            int16_t *b: pointer to 16-bit signed value 'b'                  *
 * Return   : int16_t: Return value                                           *
 * ************************************************************************** *
 * Comments : (int16_t) = (int16_t) ((int32_t) (a * b)) >> 15)                *
 *            Private mulI16_I16byI16 is about 0.4 us faster then ROM code,   *
 *            and one 16-bit word shorter                                     *
 * ************************************************************************** */
static __inline__ int16_t p_MulI16_I16bypQ15(int16_t a, const int16_t *b) __attribute__ ((always_inline));
static __inline__ int16_t p_MulI16_I16bypQ15(int16_t a, const int16_t *b)
{
    int16_t i16Result;  /*lint -e530 */
    int16_t i16Rubbish; /*lint -e529 */                                           /* Clobbering of the register */

    __asm__ __volatile__ (
        "muls YA, A, [%[b]] \n\t"
        "asl YA, #1"
        : "=y" (i16Result), "=a" (i16Rubbish)
        : "%a" (a), [b] "r" (b)
        );

    return (i16Result);
} /* End of p_MulI16_I16bypQ15() */

/* ************************************************************************** *
 * Function : p_GetSine()                                                     *
 * Purpose  : Get Sine/Cosine from table                                      *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t p: Pointer to Sine/Cosine in table                      *
 * Return   : int16_t: Return Sine/Cosine (in X)                              *
 * ************************************************************************** *
 * Comments :                                                                 *
 * ************************************************************************** */
static __inline__ int16_t p_GetSine(int16_t p) __attribute__ ((always_inline));
static __inline__ int16_t p_GetSine(int16_t p)
{
    int16_t i16Result;  /*lint -e530 */

    __asm__ __volatile__ (
        "lod X, [%[p]] \n\t"
        : "=x" (i16Result)
        : [p] "xy" (p)
        );

    return (i16Result);
} /* p_NewX */

/* ************************************************************************** *
 * Function : p_MulU16hi_U16byU16()                                           *
 * Purpose  : Multiply unsigned 16-bit variable with a unsigned 16-bit        *
 *            variable and return the upper 16-bit                            *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t a: unsigned 16-bit variable 'a'                        *
 *            uint16_t b: 16-bit unsigned value 'b'                           *
 * Return   : uint16_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint16_t) = (uint16_t) ((uint32_t) (a * b)) >> 16)             *
 *            Private mulU16_U16byU16 is about 0.4 us faster then ROM code,   *
 *            and one 16-bit word shorter                                     *
 * ************************************************************************** */
static __inline__ uint16_t p_MulU16hi_U16byU16(uint16_t a, uint16_t b) __attribute__ ((always_inline));
static __inline__ uint16_t p_MulU16hi_U16byU16(uint16_t a, uint16_t b)
{
    uint16_t u16Result;  /*lint -e530 */
    uint16_t u16Rubbish; /*lint -e529 */                                          /* Clobbering of the register */

    __asm__ __volatile__ (
        "mulu YA, A, %[b]"
        : "=y" (u16Result), "=a" (u16Rubbish)
        : "%a" (a), [b] "nr" (b)
        );

    return (u16Result);
} /* End of p_MulU16hi_U16byU16() */

static __inline__ uint32_t p_MulU32lo_U32byU16(uint32_t a, uint16_t b) __attribute__ ((always_inline));
static __inline__ uint32_t p_MulU32lo_U32byU16(uint32_t a, uint16_t b)
{
    uint32_t u32Result;  /*lint -e530 */

    __asm__ __volatile__ (
        "swap YA \n\t"
        "mulu D, A, %[b] \n\t"
        "asl D, #16 \n\t"
        "swap YA \n\t"
        "macu D, A, %[b] \n\t"
        "mov YA, D"
        : "=b" (u32Result)
        : "b" (a), [b] "nx" (b)
        : "D"
        );

    return (u32Result);
}

/* ************************************************************************** *
 * Function : p_MulI32_I16byI16()                                             *
 * Purpose  : Multiply signed 16-bit variable with a signed 16-bit            *
 *            variable and return (32-bits)                                   *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t a: signed 16-bit variable 'a'                           *
 *            int16_t b: 16-bit signed value 'b'                              *
 * Return   : int32_t: Return value                                           *
 * ************************************************************************** *
 * Comments : (int32_t) = ((int32_t) (a * b))                                 *
 *            Private mulI32_I16byI16 is about 0.4 us faster then ROM code,   *
 *            and one 16-bit word shorter                                     *
 * ************************************************************************** */
static __inline__ int32_t p_MulI32_I16byI16(int16_t a, int16_t b) __attribute__ ((always_inline));
static __inline__ int32_t p_MulI32_I16byI16(int16_t a, int16_t b)
{
    int32_t i32Result;

    __asm__ __volatile__ (
        "muls %Q0, A, %[b]"
        : "=b" (i32Result)
        : "a" (a), [b] "nr" (b)
        );

    return (i32Result);
} /* End of p_MulI32_I16byI16() */

static __inline__ int16_t p_MulI16lo_I16byI16(int16_t a, int16_t b) __attribute__ ((always_inline));
static __inline__ int16_t p_MulI16lo_I16byI16(int16_t a, int16_t b)
{
    int16_t i16Result;

    __asm__ __volatile__ (
        "muls A, A, %[b]"
        : "=a" (i16Result)
        : "a" (a), [b] "nr" (b)
        );

    return (i16Result);
} /* End of p_MulI16lo_I16byI16() */

/* ************************************************************************** *
 * Function : p_MulU32_U16byU16()                                             *
 * Purpose  : Multiply unsigned 16-bit variable with a unsigned 16-bit        *
 *            variable and return (32-bits)                                   *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t a: unsigned 16-bit variable 'a'                        *
 *            uint16_t b: 16-bit unsigned value 'b'                           *
 * Return   : uint32_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint32_t) = ((uint32_t) (a * b))                               *
 *            Private mulU32_U16byU16 is about 0.4 us faster then ROM code,   *
 *            and one 16-bit word shorter                                     *
 * ************************************************************************** */
static __inline__ uint32_t p_MulU32_U16byU16(uint16_t a, uint16_t b) __attribute__ ((always_inline));
static __inline__ uint32_t p_MulU32_U16byU16(uint16_t a, uint16_t b)
{
    uint32_t u32Result;

    __asm__ __volatile__ (
        "mulu %Q0, A, %[b]"
        : "=b" (u32Result)
        : "%a" (a), [b] "nr" (b)
        );

    return (u32Result);
} /* End of p_MulU32_U16byU16() */

/* ************************************************************************** *
 * Function : p_MulU32_U16byU16lsr8()                                         *
 * Purpose  : Multiply unsigned 16-bit variable with a unsigned 16-bit        *
 *            variable and return (32-bits)                                   *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t a: unsigned 16-bit variable 'a'                        *
 *            uint16_t b: 16-bit unsigned value 'b'                           *
 * Return   : uint32_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint32_t) = ((uint32_t) (a * b) >> 8)                          *
 *            Private mulU32_U16byU16 is about 0.4 us faster then ROM code,   *
 *            and one 16-bit word shorter                                     *
 * ************************************************************************** */
static __inline__ uint32_t p_MulU32_U16byU16lsr8(uint16_t a, uint16_t b) __attribute__ ((always_inline));
static __inline__ uint32_t p_MulU32_U16byU16lsr8(uint16_t a, uint16_t b)
{
    uint32_t u32Result;

    __asm__ __volatile__ (
        "mulu %Q0, A, %[b] \n\t"
        "lsr %Q0, #8"
        : "=b" (u32Result)
        : "%a" (a), [b] "nr" (b)
        );

    return (u32Result);
} /* End of p_MulU32_U16byU16lsr8() */

/* ************************************************************************** *
 * Function : p_MulI32_I16byI16asr8()                                         *
 * Purpose  : Multiply signed 16-bit variable with a signed 16-bit            *
 *            variable and return (32-bits)                                   *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t a: unsigned 16-bit variable 'a'                         *
 *            int16_t b: 16-bit unsigned value 'b'                            *
 * Return   : int32_t: Return value                                           *
 * ************************************************************************** *
 * Comments : (int32_t) = ((int32_t) (a * b) >> 8)                            *
 *            Private mulU32_U16byU16 is about 0.4 us faster then ROM code,   *
 *            and one 16-bit word shorter                                     *
 * ************************************************************************** */
static __inline__ int32_t p_MulI32_I16byI16asr8(int16_t a, int16_t b) __attribute__ ((always_inline));
static __inline__ int32_t p_MulI32_I16byI16asr8(int16_t a, int16_t b)
{
    int32_t i32Result;

    __asm__ __volatile__ (
        "muls %Q0, A, %[b] \n\t"
        "asr %Q0, #8"
        : "=b" (i32Result)
        : "%a" (a), [b] "nr" (b)
        );

    return (i32Result);
} /* End of p_MulI32_I16byI16lsr8() */

/* ************************************************************************** *
 * Function : p_MulU32_U16byU16lsr10()                                        *
 * Purpose  : Multiply unsigned 16-bit variable with a unsigned 16-bit        *
 *            variable and return (32-bits)                                   *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t a: unsigned 16-bit variable 'a'                        *
 *            uint16_t b: 16-bit unsigned value 'b'                           *
 * Return   : uint32_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint16_t) = (uint16_t) (((uint32_t) (a * b)) >> 10)            *
 *            Private mulU32_U16byU16 is about 0.4 us faster then ROM code,   *
 *            and one 16-bit word shorter                                     *
 * ************************************************************************** */
static __inline__ uint16_t p_MulU16_U16byU16lsr10(uint16_t a, uint16_t b) __attribute__ ((always_inline));
static __inline__ uint16_t p_MulU16_U16byU16lsr10(uint16_t a, uint16_t b)
{
    uint16_t u16Result;
    uint16_t u16Rubbish; /*lint -e529 */                                          /* Clobbering of the register */

    __asm__ __volatile__ (
        "mulu YA, A, %[b] \n\t"
        "lsr YA, #10"
        : "=a" (u16Result), "=y" (u16Rubbish)
        : "%a" (a), [b] "nr" (b)
        );

    return (u16Result);
} /* End of p_MulU16_U16byU16lsr10() */

/* ************************************************************************** *
 * Function : p_MulI32_I16byI16asr10()                                        *
 * Purpose  : Multiply signed 16-bit variable with a signed 16-bit            *
 *            variable and return (32-bits)                                   *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t a: signed 16-bit variable 'a'                           *
 *            int16_t b: 16-bit signed value 'b'                              *
 * Return   : int32_t: Return value                                           *
 * ************************************************************************** *
 * Comments : (int16_t) = (int16_t) (((int32_t) (a * b)) >> 10)               *
 *            Private mulI32_I16byI16 is about 0.4 us faster then ROM code,   *
 *            and one 16-bit word shorter                                     *
 * ************************************************************************** */
static __inline__ int16_t p_MulI16_I16byI16asr10(int16_t a, int16_t b) __attribute__ ((always_inline));
static __inline__ int16_t p_MulI16_I16byI16asr10(int16_t a, int16_t b)
{
    int16_t i16Result;
    int16_t i16Rubbish; /*lint -e529 */                                          /* Clobbering of the register */

    __asm__ __volatile__ (
        "muls YA, A, %[b] \n\t"
        "asr YA, #10"
        : "=a" (i16Result), "=y" (i16Rubbish)
        : "%a" (a), [b] "nr" (b)
        );

    return (i16Result);
} /* End of p_MulI16_I16byI16asr10() */

static __inline__ uint16_t p_MulU16lo_U16byU16(uint16_t a, uint16_t b) __attribute__ ((always_inline));
static __inline__ uint16_t p_MulU16lo_U16byU16(uint16_t a, uint16_t b)
{
    uint16_t u16Result;

    __asm__ __volatile__ (
        "mulu A, A, %[b]"
        : "=a" (u16Result)
        : "%a" (a), [b] "nr" (b)
        );

    return (u16Result);
} /* End of p_MulU16lo_U16byU16() */

/* ************************************************************************** *
 * Function : p_MulDivI16_I16byI16byI16()                                     *
 * Purpose  : Multiply signed 16-bit variable with a signed 16-bit            *
 *            variable and divide by signed 16-bit variable and return        *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t a: signed 16-bit variable 'a'                           *
 *            int16_t b: 16-bit signed value 'b'                              *
 *            int16_t c: 16-bit signed divisor 'c'                            *
 * Return   : int16_t: Return value                                           *
 * ************************************************************************** *
 * Comments : (int16_t) = (int16_t) (((int32_t) (a * b)) / c)                 *
 * ************************************************************************** */
static __inline__ int16_t  p_MulDivI16_I16byI16byI16(int16_t a, int16_t b, int16_t c) __attribute__ ((always_inline));
static __inline__ int16_t  p_MulDivI16_I16byI16byI16(int16_t a, int16_t b, int16_t c)
{
    int16_t i16Result;  /*lint -e530 */
    int16_t i16Rubbish; /*lint -e529 */                                         /* Clobbering of the register */

    __asm__ __volatile__ (
        "muls YA, A, %[b] \n\t"
        "divs YA, %[c] \n\t"
        "divs YA, %[c] \n\t"
        "dadjs YA, %[c]"
        : "=a" (i16Result), "=y" (i16Rubbish)
        : "%a" (a), [b] "ny" (b), [c] "x" (c)
        );

    return (i16Result);
} /* End of p_MulDivI16_I16byI16byI16() */

/* ************************************************************************** *
 * Function : p_MulDivU16_U16byU16byU16()                                     *
 * Purpose  : Multiply unsigned 16-bit variable with a unsigned 16-bit        *
 *            variable and divide by unsigned 16-bit variable and return      *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t a: unsigned 16-bit variable 'a'                        *
 *            uint16_t b: 16-bit unsigned value 'b'                           *
 *            uint16_t c: 16-bit unsigned divisor 'c'                         *
 * Return   : uint16_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint16_t) = (uint16_t) (((uint32_t) (a * b)) / c)              *
 * ************************************************************************** */
static __inline__ uint16_t p_MulDivU16_U16byU16byU16(uint16_t a, uint16_t b, uint16_t c) __attribute__ (
    (always_inline));
static __inline__ uint16_t p_MulDivU16_U16byU16byU16(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;                                                          /* Clobbering of the register */

    __asm__ __volatile__ (
        "mulu YA, A, %[b] \n\t"
        "divu YA, %[c] \n\t"
        "divu YA, %[c]"
        : "=a" (u16Result), "=y" (u16Rubbish)
        : "%a" (a), [b] "ny" (b), [c] "nx" (c)
        );

    return (u16Result);
} /* End of p_MulDivU16_U16byU16byU16() */

/* ************************************************************************** *
 * Function : p_MulU16lo_U16byU16lsr1()                                       *
 * Purpose  : Multiply unsigned 16-bit variable with a unsigned 16-bit        *
 *            variable, divided by 2 and return (16-bits)                     *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t a: unsigned 16-bit variable 'a'                        *
 *            uint16_t b: 16-bit unsigned value 'b'                           *
 * Return   : uint16_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint16_t) = (uint16_t) (((uint32_t) (a * b)) >> 1)             *
 * ************************************************************************** */
static __inline__ uint16_t p_MulU16lo_U16byU16lsr1(uint16_t a, uint16_t b) __attribute__ ((always_inline));
static __inline__ uint16_t p_MulU16lo_U16byU16lsr1(uint16_t a, uint16_t b)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;                                                        /* Clobbering of the register */

    __asm__ __volatile__ (
        "mulu YA, A, %[b] \n\t"
        "lsr YA, #1"
        : "=a" (u16Result), "=y" (u16Rubbish)
        : "%a" (a), [b] "ny" (b)
        );

    return (u16Result);
} /* End of p_MulU16lo_U16byU16lsr1() */

/* ************************************************************************** *
 * Function : p_MulI16hi_I16byI16asr3()                                       *
 * Purpose  : Multiply signed 16-bit variable with a signed 16-bit            *
 *            variable and divide by 8 and return                             *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t a: signed 16-bit variable 'a'                           *
 *            int16_t b: 16-bit signed value 'b'                              *
 * Return   : int16_t: Return value                                           *
 * ************************************************************************** *
 * Comments : (int16_t) = (int16_t) (((int32_t) (a * b)) >> 3)                *
 * ************************************************************************** */
static __inline__ int16_t p_MulI16hi_I16byI16asr3(int16_t a, int16_t b) __attribute__ ((always_inline));
static __inline__ int16_t p_MulI16hi_I16byI16asr3(int16_t a, int16_t b)
{
    int16_t i16Result;
    int16_t i16Rubbish;                                                         /* Clobbering of the register */

    __asm__ __volatile__ (
        "muls YA, A, %[b] \n\t"
        "asr YA, #3"
        : "=y" (i16Result), "=a" (i16Rubbish)
        : "%a" (a), [b] "nr" (b)
        );

    return (i16Result);
} /* End of p_MulI16hi_I16byI16asr3() */

/* ************************************************************************** *
 * Function : p_MulU16hi_U16byU16lsr3()                                       *
 * Purpose  : Multiply unsigned 16-bit variable with a unsigned 16-bit        *
 *            variable and divide by 8 and return                             *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t a: unsigned 16-bit variable 'a'                        *
 *            uint16_t b: 16-bit unsigned value 'b'                           *
 * Return   : uint16_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint16_t) = (uint16_t) (((uint32_t) (a * b)) >> 3)             *
 * ************************************************************************** */
static __inline__ uint16_t p_MulU16hi_U16byU16lsr3(uint16_t a, uint16_t b) __attribute__ ((always_inline));
static __inline__ uint16_t p_MulU16hi_U16byU16lsr3(uint16_t a, uint16_t b)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;                                                        /* Clobbering of the register */

    __asm__ __volatile__ (
        "mulu YA, A, %[b] \n\t"
        "lsr YA, #3"
        : "=y" (u16Result), "=a" (u16Rubbish)
        : "%a" (a), [b] "nr" (b)
        );

    return (u16Result);
} /* End of p_MulU16hi_U16byU16lsr3() */

/* ************************************************************************** *
 * Function : p_MulI16hi_pI16byI16asr3()                                      *
 * Purpose  : Multiply signed 16-bit pointer-variable with a signed 16-bit    *
 *            variable and divide by 8 and return                             *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t ptr: pointer to a signed 16-bit                        *
 *            uint16_t a: 16-bit signed value 'a'                             *
 * Return   : uint16_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (int16_t) = (int16_t) (((int32_t) (p[] * a)) >> 3)              *
 * ************************************************************************** */
static __inline__ int16_t p_MulI16hi_pI16byI16asr3(int16_t *ptr, int16_t a) __attribute__ ((always_inline));
static __inline__ int16_t p_MulI16hi_pI16byI16asr3(int16_t *ptr, int16_t a)
{
    int16_t i16Result;
    int16_t i16Rubbish;                                                         /* Clobbering of the register */

    __asm__ __volatile__ (
        "muls YA, A, [X] \n\t"
        "asr YA, #3"
        : "=y" (i16Result), "=a" (i16Rubbish)
        : "a" (a), "x" (ptr)
        );

    return (i16Result);
} /* End of p_MulI16hi_pI16byI16lsr3() */

/* ************************************************************************** *
 * Function : p_MulU16hi_pU16byU16lsr3()                                      *
 * Purpose  : Multiply unsigned 16-bit pointer-variable with a unsigned 16-bit*
 *            variable and divide by 8 and return                             *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t ptr: pointer to a unsigned 16-bit                      *
 *            uint16_t a: 16-bit unsigned value 'a'                           *
 * Return   : uint16_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint16_t) = (uint16_t) (((uint32_t) (p[] * a)) >> 3)           *
 * ************************************************************************** */
static __inline__ uint16_t p_MulU16hi_pU16byU16lsr3(uint16_t *ptr, uint16_t a) __attribute__ ((always_inline));
static __inline__ uint16_t p_MulU16hi_pU16byU16lsr3(uint16_t *ptr, uint16_t a)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;                                                        /* Clobbering of the register */

    __asm__ __volatile__ (
        "mulu YA, A, [X] \n\t"
        "lsr YA, #3"
        : "=y" (u16Result), "=a" (u16Rubbish)
        : "a" (a), "x" (ptr)
        );

    return (u16Result);
} /* End of p_MulU16hi_pU16byU16lsr3() */

/* ************************************************************************** *
 * Function : p_MulI16hi_I16byI16asr4()                                       *
 * Purpose  : Multiply signed 16-bit variable with a signed 16-bit            *
 *            variable and divide by 16 and return                            *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t a: signed 16-bit variable 'a'                           *
 *            int16_t b: 16-bit signed value 'b'                              *
 * Return   : int16_t: Return value                                           *
 * ************************************************************************** *
 * Comments : (int16_t) = (int16_t) (((int32_t) (a * b)) >> 4)                *
 * ************************************************************************** */
static __inline__ int16_t p_MulI16hi_I16byI16asr4(int16_t a, int16_t b) __attribute__ ((always_inline));
static __inline__ int16_t p_MulI16hi_I16byI16asr4(int16_t a, int16_t b)
{
    int16_t i16Result;
    int16_t i16Rubbish;                                                         /* Clobbering of the register */

    __asm__ __volatile__ (
        "muls YA, A, %[b] \n\t"
        "asr YA, #4"
        : "=y" (i16Result), "=a" (i16Rubbish)
        : "%a" (a), [b] "nr" (b)
        );

    return (i16Result);
} /* End of p_MulI16hi_I16byI16asr4() */

/* ************************************************************************** *
 * Function : p_MulU16hi_U16byU16lsr4()                                       *
 * Purpose  : Multiply unsigned 16-bit variable with a unsigned 16-bit        *
 *            variable and divide by 16 and return                            *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t a: unsigned 16-bit variable 'a'                        *
 *            uint16_t b: 16-bit unsigned value 'b'                           *
 * Return   : uint16_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint16_t) = (uint16_t) (((uint32_t) (a * b)) >> 4)             *
 * ************************************************************************** */
static __inline__ uint16_t p_MulU16hi_U16byU16lsr4(uint16_t a, uint16_t b) __attribute__ ((always_inline));
static __inline__ uint16_t p_MulU16hi_U16byU16lsr4(uint16_t a, uint16_t b)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;                                                        /* Clobbering of the register */

    __asm__ __volatile__ (
        "mulu YA, A, %[b] \n\t"
        "lsr YA, #4"
        : "=y" (u16Result), "=a" (u16Rubbish)
        : "%a" (a), [b] "nr" (b)
        );

    return (u16Result);
} /* End of p_MulU16hi_U16byU16lsr4() */

/* ************************************************************************** *
 * Function : p_MulI16hi_pI16byI16asr4()                                      *
 * Purpose  : Multiply signed 16-bit pointer-variable with a signed 16-bit    *
 *            variable and divide by 16 and return                            *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t ptr: pointer to a signed 16-bit                        *
 *            uint16_t a: 16-bit signed value 'a'                             *
 * Return   : uint16_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint16_t) = (uint16_t) (((uint32_t) (p[] * a)) >> 4)           *
 * ************************************************************************** */
static __inline__ int16_t p_MulI16hi_pI16byI16asr4(int16_t *ptr, int16_t a) __attribute__ ((always_inline));
static __inline__ int16_t p_MulI16hi_pI16byI16asr4(int16_t *ptr, int16_t a)
{
    int16_t i16Result;
    int16_t i16Rubbish;                                                         /* Clobbering of the register */

    __asm__ __volatile__ (
        "muls YA, A, [X] \n\t"
        "asr YA, #4"
        : "=y" (i16Result), "=a" (i16Rubbish)
        : "a" (a), "x" (ptr)
        );

    return (i16Result);
} /* End of p_MulI16hi_pI16byI16asr4() */

/* ************************************************************************** *
 * Function : p_MulU16hi_pU16byU16lsr4()                                      *
 * Purpose  : Multiply unsigned 16-bit pointer-variable with a unsigned 16-bit*
 *            variable and divide by 16 and return                            *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t ptr: pointer to a unsigned 16-bit                      *
 *            uint16_t a: 16-bit unsigned value 'a'                           *
 * Return   : uint16_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint16_t) = (uint16_t) (((uint32_t) (p[] * a)) >> 4)           *
 * ************************************************************************** */
static __inline__ uint16_t p_MulU16hi_pU16byU16lsr4(uint16_t *ptr, uint16_t a) __attribute__ ((always_inline));
static __inline__ uint16_t p_MulU16hi_pU16byU16lsr4(uint16_t *ptr, uint16_t a)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;                                                        /* Clobbering of the register */

    __asm__ __volatile__ (
        "mulu YA, A, [X] \n\t"
        "lsr YA, #4"
        : "=y" (u16Result), "=a" (u16Rubbish)
        : "a" (a), "x" (ptr)
        );

    return (u16Result);
} /* End of p_MulU16hi_pU16byU16lsr4() */

/* ************************************************************************** *
 * Function : p_MulI16hi_I16byI16asr5()                                       *
 * Purpose  : Multiply signed 16-bit variable with a signed 16-bit            *
 *            variable and divide by 32 and return                            *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t a: signed 16-bit variable 'a'                           *
 *            int16_t b: 16-bit signed value 'b'                              *
 * Return   : int16_t: Return value                                           *
 * ************************************************************************** *
 * Comments : (int16_t) = (int16_t) (((int32_t) (a * b)) >> 5)                *
 * ************************************************************************** */
static __inline__ int16_t p_MulI16hi_I16byI16asr5(int16_t a, int16_t b) __attribute__ ((always_inline));
static __inline__ int16_t p_MulI16hi_I16byI16asr5(int16_t a, int16_t b)
{
    int16_t i16Result;
    int16_t i16Rubbish;                                                         /* Clobbering of the register */

    __asm__ __volatile__ (
        "muls YA, A, %[b] \n\t"
        "asr YA, #5"
        : "=y" (i16Result), "=a" (i16Rubbish)
        : "a" (a), [b] "nr" (b)
        );

    return (i16Result);
} /* End of p_MulI16hi_I16byI16asr5() */

/* ************************************************************************** *
 * Function : p_MulI16hi_U16byU16lsr5()                                       *
 * Purpose  : Multiply unsigned 16-bit variable with a unsigned 16-bit        *
 *            variable and divide by 32 and return                            *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t a: unsigned 16-bit variable 'a'                        *
 *            uint16_t b: 16-bit unsigned value 'b'                           *
 * Return   : uint16_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint16_t) = (uint16_t) (((uint32_t) (a * b)) >> 5)             *
 * ************************************************************************** */
static __inline__ uint16_t p_MulU16hi_U16byU16lsr5(uint16_t a, uint16_t b) __attribute__ ((always_inline));
static __inline__ uint16_t p_MulU16hi_U16byU16lsr5(uint16_t a, uint16_t b)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;                                                        /* Clobbering of the register */

    __asm__ __volatile__ (
        "mulu YA, A, %[b] \n\t"
        "lsr YA, #5"
        : "=y" (u16Result), "=a" (u16Rubbish)
        : "a" (a), [b] "nr" (b)
        );

    return (u16Result);
} /* End of p_MulU16hi_U16byU16lsr5() */

/* ************************************************************************** *
 * Function : p_MulU16hi_pU16byU16lsr5()                                      *
 * Purpose  : Multiply unsigned 16-bit pointer-variable with a unsigned 16-bit*
 *            variable and divide by 16 and return                            *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t ptr: pointer to a unsigned 16-bit                      *
 *            uint16_t a: 16-bit unsigned value 'a'                           *
 * Return   : uint16_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint16_t) = (uint16_t) (((uint32_t) (p[] * a)) >> 5)           *
 * ************************************************************************** */
static __inline__ uint16_t p_MulU16hi_pU16byU16lsr5(uint16_t *ptr, uint16_t a) __attribute__ ((always_inline));
static __inline__ uint16_t p_MulU16hi_pU16byU16lsr5(uint16_t *ptr, uint16_t a)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;                                                        /* Clobbering of the register */

    __asm__ __volatile__ (
        "mulu YA, A, [X] \n\t"
        "lsr YA, #5"
        : "=y" (u16Result), "=a" (u16Rubbish)
        : "a" (a), "x" (ptr)
        );

    return (u16Result);
} /* End of p_MulU16hi_pU16byU16lsr5() */

/* ************************************************************************** *
 * Function : p_MulI16lo_I16byI16asr6Rnd()                                    *
 * Purpose  : Multiply signed 16-bit variable with a signed 16-bit            *
 *            variable and divide by 64 include rounding and return           *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t a: signed 16-bit variable 'a'                           *
 *            int16_t b: 16-bit signed value 'b'                              *
 * Return   : int16_t: Return value                                           *
 * ************************************************************************** *
 * Comments : (int16_t) = (int16_t) (((int32_t) (a * b)) >> 6)                *
 * ************************************************************************** */
static __inline__ int16_t p_MulI16lo_I16byI16asr6Rnd(int16_t a, int16_t b) __attribute__ ((always_inline));
static __inline__ int16_t p_MulI16lo_I16byI16asr6Rnd(int16_t a, int16_t b)
{
    int16_t result;
    int16_t result2;    /* clobbering of the register */

    __asm__ __volatile__ (
        "muls YA, A, %[b] \n\t"
        "asr YA, #6 \n\t"
        "adc A, #0"
        : "=a" (result), "=y" (result2)
        : "%a" (a), [b] "r" (b)
        );

    return result;
} /* End of p_MulI16lo_I16byI16asr6Rnd() */

/* ************************************************************************** *
 * Function : p_MulI16lo_I16byI16asr8Rnd()                                    *
 * Purpose  : Multiply signed 16-bit variable with a signed 16-bit            *
 *            variable and divide by 32 and return                            *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t a: signed 16-bit variable 'a'                           *
 *            int16_t b: 16-bit signed value 'b'                              *
 * Return   : int16_t: Return value                                           *
 * ************************************************************************** *
 * Comments : (int16_t) = (int16_t) (((int32_t) (a * b)) >> 5)                *
 * ************************************************************************** */
static __inline__ int16_t p_MulI16lo_I16byI16asr8Rnd(int16_t a, int16_t b) __attribute__ ((always_inline));
static __inline__ int16_t p_MulI16lo_I16byI16asr8Rnd(int16_t a, int16_t b)
{
    int16_t result;
    int16_t result2;    /* clobbering of the register */

    __asm__ __volatile__ (
        "muls YA, A, %[b] \n\t"
        "asr YA, #8 \n\t"
        "adc A, #0"
        : "=a" (result), "=y" (result2)
        : "%a" (a), [b] "nr" (b)
        );

    return result;
} /* End of p_MulI16lo_I16byI16asr8Rnd() */

/* ************************************************************************** *
 * Function : p_DivI16_I32byI16()                                             *
 * Purpose  : Signed division of 32-bit variable with a signed 16-bit         *
 *            variable and return                                             *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int32_t a: signed 32-bit variable 'a'                           *
 *            int16_t b: 16-bit signed value 'b'                              *
 * Return   : int16_t: Return value                                           *
 * ************************************************************************** *
 * Comments : (int16_t) = (int16_t) (((int32_t) a) / b)                       *
 * ************************************************************************** */
static __inline__ int16_t p_DivI16_I32byI16(int32_t a, int16_t b) __attribute__ ((always_inline));
static __inline__ int16_t p_DivI16_I32byI16(int32_t a, int16_t b)
{
    int16_t i16Result;
    int16_t i16Rubbish;                                  /* Clobbering of the register */

    __asm__ __volatile__ (
        "divs YA, %[b] \n\t"
        "divs YA, %[b] \n\t"
        "dadjs YA,%[b]"
        : "=a" (i16Result), "=y" (i16Rubbish)
        : "b" (a), [b] "nx" (b)
        );

    return (i16Result);
} /* End of p_DivI16_I32byI16() */

/* ************************************************************************** *
 * Function : p_DivU16_U32byU16()                                             *
 * Purpose  : Unsigned division of 32-bit variable with a unsigned 16-bit     *
 *            variable and return                                             *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint32_t a: unsigned 16-bit variable 'a'                        *
 *            uint16_t b: 16-bit unsigned value 'b'                           *
 * Return   : uint16_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint16_t) = (uint16_t) (((uint32_t) a) / b)                    *
 * ************************************************************************** */
static __inline__ uint16_t p_DivU16_U32byU16(uint32_t a, uint16_t b) __attribute__ ((always_inline));
static __inline__ uint16_t p_DivU16_U32byU16(uint32_t a, uint16_t b)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;                                  /* Clobbering of the register */

    __asm__ __volatile__ (
        "divu YA, %[b] \n\t"
        "divu YA, %[b] \n\t"
        : "=a" (u16Result), "=y" (u16Rubbish)
        : "b" (a), [b] "nx" (b)
        );

    return (u16Result);
} /* End of p_DivU16_U32byU16() */

/* ************************************************************************** *
 * Function : p_ModU16_U32byU16()                                             *
 * Purpose  : Unsigned Modulo of 32-bit variable with a unsigned 16-bit     *
 *            variable and return                                             *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint32_t a: unsigned 16-bit variable 'a'                        *
 *            uint16_t b: 16-bit unsigned value 'b'                           *
 * Return   : uint16_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint16_t) = (uint16_t) (((uint32_t) a) / b)                    *
 * ************************************************************************** */
static __inline__ uint16_t p_ModU16_U32byU16(uint32_t a, uint16_t b) __attribute__ ((always_inline));
static __inline__ uint16_t p_ModU16_U32byU16(uint32_t a, uint16_t b)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;                                  /* Clobbering of the register */

    __asm__ __volatile__ (
        "divu YA, %[b] \n\t"
        "divu YA, %[b] \n\t"
        : "=y" (u16Result), "=a" (u16Rubbish)
        : "b" (a), [b] "nx" (b)
        );

    return (u16Result);
} /* End of p_ModU16_U32byU16() */

/* ************************************************************************** *
 * Function : p_DivU32_U32byU16()                                             *
 * Purpose  : Unsigned division of 32-bit variable with a unsigned 16-bit     *
 *            variable and return                                             *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint32_t a: unsigned 16-bit variable 'a'                        *
 *            uint16_t b: 16-bit unsigned value 'b'                           *
 * Return   : uint32_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint32_t) = (uint32_t) (((uint32_t) a) / b)                    *
 * ************************************************************************** */
static __inline__ uint32_t p_DivU32_U32byU16(uint32_t a, uint16_t b) __attribute__ ((always_inline));
static __inline__ uint32_t p_DivU32_U32byU16(uint32_t a, uint16_t b)
{
    uint32_t u32Result;

    __asm__ __volatile__ (
        "push A \n\t"
        "lod  A, Y \n\t"
        "lod  Y, #0 \n\t"
        "divu YA, %[b] \n\t"
        "divu YA, %[b] \n\t"
        "push A \n\t"
        "lod  A, [S-4] \n\t"
        "divu YA, %[b] \n\t"
        "divu YA, %[b] \n\t"
        "pop  Y \n\t"
        "dec  S, #2"
        : "=b" (u32Result)
        : "b" (a), [b] "nx" (b)
        );

    return (u32Result);
} /* End of p_DivU32_U32byU16() */

static __inline__ int16_t p_DivI16_I16hibyI16(int16_t a, int16_t b) __attribute__ ((always_inline));
static __inline__ int16_t p_DivI16_I16hibyI16(int16_t a, int16_t b)
{
    int16_t i16Result;
    int16_t i16Rubbish;                                                        /* Clobbering of the register */

    __asm__ __volatile__ (
        "lod A, #0 \n\t"
        "asr YA, #1 \n\t"
        "divs YA, %[b] \n\t"
        "divs YA, %[b] \n\t"
        "dadjs YA, %[b]"
        : "=a" (i16Result), "=y" (i16Rubbish)
        : "y" (a), [b] "nx" (b)
        );

    return (i16Result);
} /* End of p_DivI16_I16hibyI16() */

static __inline__ uint16_t p_DivU16_U16hibyU16(uint16_t a, uint16_t b) __attribute__ ((always_inline));
static __inline__ uint16_t p_DivU16_U16hibyU16(uint16_t a, uint16_t b)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;                                                        /* Clobbering of the register */

    __asm__ __volatile__ (
        "lod A, #0 \n\t"
        "divu YA, %[b] \n\t"
        "divu YA, %[b]"
        : "=a" (u16Result), "=y" (u16Rubbish)
        : "y" (a), [b] "nx" (b)
        );

    return (u16Result);
} /* End of p_DivU16hi_U16byU16() */

/* ************************************************************************** *
 * Function : p_LpfU16_I16byI16()                                             *
 * Purpose  : First order Low Pass Filter (IIR-1)                             *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint32_t *pu32LPF: Internal variable                            *
 *            int16_t i16Coef: Filter coefficient                             *
 *            int16_t i16Value: Input (16-bits signed)                        *
 * Return   : uint16_t: Return value                                          *
 * ************************************************************************** *
 * Comments : *u16Ptr32 += (i16Value * i16Coef);                              *
 *            u16Result = (*u16Ptr32 / 65536)                                 *
 * ************************************************************************** */
static __inline__ uint16_t p_LpfU16_I16byI16(uint32_t *pu32LPF, int16_t i16Coef,
                                             int16_t i16Value) __attribute__ ((always_inline));
static __inline__ uint16_t p_LpfU16_I16byI16(uint32_t *pu32LPF, int16_t i16Coef, int16_t i16Value)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;

    __asm__ __volatile__ (
        "muls YA, A, %[coef] \n\t"
        "add YA, [X] \n\t"
        "mov [X], YA \n\t"
        : "=y" (u16Result), "=a" (u16Rubbish)
        : "a" (i16Value), "x" (pu32LPF), [coef] "ny" (i16Coef)
        );
    return (u16Result);
} /* End of p_LpfU16_I16byI16() */

/* ************************************************************************** *
 * Function : p_LpfI16_I16byI16()                                             *
 * Purpose  : First order Low Pass Filter (IIR-1)                             *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int32_t *pi32LPF: Internal variable                             *
 *            int16_t i16Coef: Filter coefficient                             *
 *            int16_t i16Value: Input (16-bits signed)                        *
 * Return   : int16_t: Return value                                           *
 * ************************************************************************** *
 * Comments : *pi32LPF += (i16Value * i16Coef);                               *
 *            i16Result = (*pi32LPF / 65536)                                  *
 * ************************************************************************** */
static __inline__ int16_t p_LpfI16_I16byI16(int32_t *pi32LPF, int16_t i16Coef,
                                            int16_t i16Value) __attribute__ ((always_inline));
static __inline__ int16_t p_LpfI16_I16byI16(int32_t *pi32LPF, int16_t i16Coef, int16_t i16Value)
{
    int16_t i16Result;
    uint16_t i16Rubbish;

    __asm__ __volatile__ (
        "muls YA, A, %[coef] \n\t"
        "add YA, [X] \n\t"
        "mov [X], YA \n\t"
        : "=y" (i16Result), "=a" (i16Rubbish)
        : "a" (i16Value), "x" (pi32LPF), [coef] "ny" (i16Coef)
        );
    return (i16Result);
} /* End of p_LpfU16_I16byI16() */

/* ************************************************************************** *
 * Function : p_Abs16()                                                       *
 * Purpose  : Absolute value of signed 16-bit integer                         *
 *            variable and return                                             *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t a: signed 16-bit variable 'a'                           *
 * Return   : uint16_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint16_t) = abs( a)                                            *
 * ************************************************************************** */
static __inline__ uint16_t p_Abs16(int16_t a) __attribute__ ((always_inline));
static __inline__ uint16_t p_Abs16(int16_t a)
{
    uint16_t u16Result;

    __asm__ __volatile__ (
        "cmp A, #0 \n\t"
        "jsge p_Abs16_Exit_%= \n\t"
        "neg A \n\t"
        "p_Abs16_Exit_%=: \n\t"
        : "=a" (u16Result)
        : "a" (a)
        );
    return (u16Result);
} /* End of p_Abs16() */

/* ************************************************************************** *
 * Function : p_Abs32()                                                       *
 * Purpose  : Absolute value of signed 32-bit integer                         *
 *            variable and return                                             *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int32_t a: signed 32-bit variable 'a'                           *
 * Return   : uint32_t: Return value                                          *
 * ************************************************************************** *
 * Comments : (uint32_t) = abs( a)                                            *
 * ************************************************************************** */
static __inline__ uint32_t p_Abs32(int32_t a) __attribute__ ((always_inline));
static __inline__ uint32_t p_Abs32(int32_t a)
{
    uint32_t u32Result;

    __asm__ __volatile__ (
        "cmp Y, #0 \n\t"
        "jsge p_Abs32_Exit_%= \n\t"
        "neg YA \n\t"
        "p_Abs32_Exit_%=: \n\t"
        : "=b" (u32Result)
        : "b" (a)
        );
    return (u32Result);
} /* End of p_Abs32() */

#if FALSE /* Check performance */
/* ************************************************************************** *
 * Function : p_BitRev8()                                                     *
 * Purpose  : reverse bit-order of 8-bit variable                             *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint8_t a: 8-bit variable 'a'                                   *
 * Return   : uint8_t: Return value                                           *
 * ************************************************************************** *
 * Comments : (uint8_t) = bitrev( a)                                          *
 * Performance: 20Cy; Size: 14+8 = 22 Words                                   *
 * ************************************************************************** */
static __inline__ uint8_t p_BitRev8(int8_t u8Data) __attribute__ ((always_inline));
static __inline__ uint8_t p_BitRev8(int8_t u8Data)
{
    uint8_t u8Result;

    __asm__ __volatile__
    (
        "usex A \n\t"                       /* 1 Cy */
        "mov Y, A \n\t"                     /* 1 Cy */
        "and Y, #15 \n\t"                   /* 1 Cy */
        "add Y, #_tNibbleRev \n\t"          /* 1 Cy */
        "lod YL, [Y] \n\t"                  /* 1+4 Cy */
        "asl YL, #2 \n\t"                   /* 1 Cy */
        "asl YL, #2 \n\t"                   /* 1 Cy */
        "swap YA \n\t"                      /* 2 Cy */
        "lsr Y, #2 \n\t"                    /* 1 Cy */
        "lsr Y, #2 \n\t"                    /* 1 Cy */
        "add Y, #_tNibbleRev \n\t"          /* 1 Cy */
        "or AL, [Y] \n\t"                   /* 1+4 Cy */
        : "=al" (u8Result)
        : "al" (u8Data)
        : "Y"
    );
    return (u8Result);
} /* End of p_BitRev8() */
#endif

/* ************************************************************************** *
 * Function : p_BitRev8()                                                     *
 * Purpose  : reverse bit-order of 8-bit variable                             *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint8_t a: 8-bit variable 'a'                                   *
 * Return   : uint8_t: Return value                                           *
 * ************************************************************************** *
 * Comments : (uint8_t) = bitrev( a)                                          *
 * Performance: 14Cy (excluding CALL/RET); Size: 14 Words                     *
 * ************************************************************************** */
static __inline__ uint8_t p_BitRev8(int8_t u8Data) __attribute__ ((always_inline));
static __inline__ uint8_t p_BitRev8(int8_t u8Data)
{
    uint8_t u8Result;

    __asm__ __volatile__
    (
        "rr AL, #2 \n\t"                   /* 1 Cy */
        "rr AL, #2 \n\t"                   /* 1 Cy */
        "mov AH, AL \n\t"                  /* 1 Cy */
        "and AL, #0x33 \n\t"               /* 1 Cy */
        "lsl AL, #2 \n\t"                  /* 1 Cy */
        "lsr AH, #2 \n\t"                  /* 1 Cy */
        "and AH, #0x33 \n\t"               /* 1 Cy */
        "or AL, AH \n\t"                   /* 1 Cy */
        "mov AH, AL \n\t"                  /* 1 Cy */
        "and AL, #0x55 \n\t"               /* 1 Cy */
        "lsl AL \n\t"                      /* 1 Cy */
        "lsr AH \n\t"                      /* 1 Cy */
        "and AH, #0x55 \n\t"               /* 1 Cy */
        "or AL, AH \n\t"                   /* 1 Cy */
        : "=al" (u8Result)
        : "al" (u8Data)
        :
    );
#if FALSE
    __asm__ __volatile__
    (
        "mov AH, AL \n\t"                   /* 1 Cy */
        "rrc AH \n\t"                       /* 1 Cy */
        "rlc AL \n\t"                       /* 1 Cy */
        "rrc AH \n\t"                       /* 1 Cy */
        "rlc AL \n\t"                       /* 1 Cy */
        "rrc AH \n\t"                       /* 1 Cy */
        "rlc AL \n\t"                       /* 1 Cy */
        "rrc AH \n\t"                       /* 1 Cy */
        "rlc AL \n\t"                       /* 1 Cy */
        "rrc AH \n\t"                       /* 1 Cy */
        "rlc AL \n\t"                       /* 1 Cy */
        "rrc AH \n\t"                       /* 1 Cy */
        "rlc AL \n\t"                       /* 1 Cy */
        "rrc AH \n\t"                       /* 1 Cy */
        "rlc AL \n\t"                       /* 1 Cy */
        "rrc AH \n\t"                       /* 1 Cy */
        "rlc AL"                            /* 1 Cy */
        : "=al" (u8Result)
        : "al" (u8Data)
        :
    );
#endif
    return (u8Result);
} /* End of p_BitRev8() */

/* ************************************************************************** *
 * Function : p_MemSet()                                                      *
 * Purpose  : Fill buffer with (8-bit) pattern                                *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : void *pBuffer: Pointer to buffer                                *
 *            uint8_t u8Pattern: Data-Pattern                                 *
 *            uint16_t u16Size: Buffer size to be filled                      *
 * Return   : Nothing                                                         *
 * ************************************************************************** *
 * Comments :                                                                 *
 * ************************************************************************** */
static __inline__ void p_MemSet(void *pBuffer, uint8_t u8Pattern, uint16_t u16Size) __attribute__ ((always_inline));
static __inline__ void p_MemSet(void *pBuffer, uint8_t u8Pattern, uint16_t u16Size)
{
    uint16_t u16Rubbish;

    __asm__ __volatile__ (
        "p_MemSet_10_%=: \n\t"
        "mov [Y++], AL \n\t"
        "djnz X, p_MemSet_10_%="
        : "=x" (u16Rubbish), "=y" (u16Rubbish)
        : "%a" (u8Pattern), "x" (u16Size), "y" (pBuffer)
        );
    (void)u16Rubbish;
} /* End of p_MemSet() */

/* ************************************************************************** *
 * Function : p_MemCpyU16()                                                   *
 * Purpose  : Copy buffer "Src" to buffer "Dest", for length "Size"           *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t *pDest: Pointer to destination-buffer                  *
 *            uint16_t *pSrc: Pointer to source-buffer                        *
 *            uint16_t u16Size: Size to be copied as "uint16_t"               *
 * Return   : uint16_t *pDest: Updated destination-buffer pointer             *
 * ************************************************************************** *
 * Comments : Both "Src" and "Dest" must be on even-address; Size is max. 16  *
 * ************************************************************************** */
static __inline__ uint16_t* p_MemCpyU16(uint16_t *pDest, uint16_t *pSrc, uint16_t u16Size) __attribute__ ((always_inline));
static __inline__ uint16_t* p_MemCpyU16(uint16_t *pDest, uint16_t *pSrc, uint16_t u16Size)
{
    uint16_t *pNewDest;
    uint16_t u16Rubbish;

    __asm__ __volatile__ (
        "mov Cx, %[size] \n\t"
        "p_MemCpy_10_%=: \n\t"
        "movsw [X++], [Y++] \n\t"
        "djnz Cx, p_MemCpy_10_%="
        : "=x" (pNewDest), "=y" (u16Rubbish)
        : [size] "rn" (u16Size), "x" (pDest), "y" (pSrc)
        );
    return (pNewDest);
} /* End of p_MemCpyU16() */

/* ************************************************************************** *
 * Function : p_MemCpyU8()                                                    *
 * Purpose  : Copy buffer "Src" to buffer "Dest", for length "Size"           *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint8_t *pDest: Pointer to destination-buffer                   *
 *            uint8_t *pSrc: Pointer to source-buffer                         *
 *            uint16_t u16Size: Size to be copied as "uint16_t"               *
 * Return   : uint8_t *pDest: Updated destination-buffer pointer              *
 * ************************************************************************** *
 * Comments : Size is max. 16 bytes                                           *
 * ************************************************************************** */
static __inline__ uint8_t* p_MemCpyU8(uint8_t *pDest, uint8_t *pSrc, uint16_t u16Size) __attribute__ ((always_inline));
static __inline__ uint8_t* p_MemCpyU8(uint8_t *pDest, uint8_t *pSrc, uint16_t u16Size)
{
    uint8_t *pNewDest;
    uint16_t u16Rubbish;

    __asm__ __volatile__ (
        "mov Cx, %[size] \n\t"
        "p_MemCpyU8_10_%=: \n\t"
        "movsb [X++], [Y++] \n\t"
        "djnz Cx, p_MemCpyU8_10_%="
        : "=x" (pNewDest), "=y" (u16Rubbish)
        : [size] "rn" (u16Size), "x" (pDest), "y" (pSrc)
        );
    return (pNewDest);
} /* End of p_MemCpyU8() */

static __inline__ uint16_t* p_MemCpyExtU16(uint16_t *pDest, uint16_t *pSrc, uint16_t u16Size) __attribute__ ((always_inline));
static __inline__ uint16_t* p_MemCpyExtU16(uint16_t *pDest, uint16_t *pSrc, uint16_t u16Size)
{
    uint16_t *pNewDest;
    uint16_t u16Rubbish;

    __asm__ __volatile__ (
        "mov A, %[size] \n\t"
        "p_MemCpyExt_Loop_%=: \n\t"
        "movsw [X++], [Y++] \n\t"
        "sub a, #1 \n\t"
        "jnz p_MemCpyExt_Loop_%="
        : "=x" (pNewDest), "=y" (u16Rubbish), "=a" (u16Rubbish)
        : [size] "rn" (u16Size), "x" (pDest), "y" (pSrc)
        );
    return (pNewDest);
} /* End of p_MemCpyU16() */

/* ************************************************************************** *
 * Function : p_MemSwapU8()                                                   *
 * Purpose  : Copy buffer "Src" to buffer "Dest", for length "Size"           *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t *pBuf: Pointer to buffer                               *
 *            uint16_t u16Size: Size to be swapped as "uint16_t"              *
 * Return   :                                                                 *
 * ************************************************************************** *
 * Comments : Both "Src" and "Dest" must be on even-address; Size is max. 16  *
 * ************************************************************************** */
static __inline__ void p_MemSwapU8(uint16_t *pBuf, uint16_t u16Size) __attribute__ ((always_inline));
static __inline__ void p_MemSwapU8(uint16_t *pBuf, uint16_t u16Size)
{
    uint16_t u16Rubbish;

    __asm__ __volatile__ (
        "mov Cx, %[size] \n\t"
        "p_MemSwap_10_%=: \n\t"
        "lod A, [X] \n\t"
        "swap A \n\t"
        "mov [X++], A \n\t"
        "djnz Cx, p_MemSwap_10_%="
        : "=x" (u16Rubbish)
        : [size] "rn" (u16Size), "x" (pBuf)
        : "A"
        );
} /* End of p_MemSwapU8() */

/* ************************************************************************** *
 * Function : p_Access32()                                                    *
 * Purpose  : Access buffer (e.g. Flash) as 32-bits                           *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : void *pBuffer: Pointer to buffer                                *
 *            uint16_t u16Size32: Buffer size to be checked in 32-bit words   *
 * Return   : Nothing                                                         *
 * ************************************************************************** *
 * Comments :                                                                 *
 * ************************************************************************** */
static __inline__ void p_Access32(void *pBuffer, uint16_t u16Size32) __attribute__ ((always_inline));
static __inline__ void p_Access32(void *pBuffer, uint16_t u16Size32)
{
    uint16_t u16Rubbish;

    __asm__ __volatile__ (
        "p_Access32_10_%=: \n\t"
        "mov D, [Y++] \n\t"
        "djnz X, p_Access32_10_%="
        : "=x" (u16Rubbish), "=y" (u16Rubbish)
        : "x" (u16Size32), "y" (pBuffer)
        : "D"
        );
    (void)u16Rubbish;
} /* End of p_Access() */

/* ************************************************************************** *
 * Function : p_PID_Control()                                                 *
 * Purpose  : PID Controller (unsigned output, with clipping)                 *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : (A) int16_t i16Error: PI(D) input (error-signal)                *
 *            (X) void *psPID: Pointer to PID-structure                       *
 *                     [0]:  (int16_t) i16CoefI: I-Coefficient PID            *
 *                     [2]: (uint32_t) u32SumError: Integrator PID            *
 *                     [6]: (uint32_t) u32SumErrorMax: Integrator Max PID     *
 *                    [10]:  (int16_t) i16CoefP: P-Coefficient PID            *
 *                    [12]:  (int16_t) i16PrevError: Previous Error           *
 *                    [14]:  (int16_t) i16CoefD: D-Coefficient PID            *
 *                    [16]: (uint16_t) u16MinOutput: Minimum output           *
 *                    [18]: (uint32_t) u32MaxOutput: Maximum output           *
 * Return   : uint32_t: PI(D) output                                          *
 * ************************************************************************** *
 * Comments : This PI(D) is intentional for angle(error) to actual-speed      *
 *            The speed is limit to 65000 RPM-e                               *
 *            Performance: 61T @ 28MHz --> 2.18us                             *
 *                             @ 32MHz --> 1.91us (2.13us + 14T)              *
 * (MMP201104-2) Improve PID_Controller, use of SumError Maximum (not fixed), *
 *               and 32-bit output-support                                    *
 * ************************************************************************** */
static __inline__ uint32_t p_PID_Control(int16_t i16Error, void *psPID) __attribute__ ((always_inline));
static __inline__ uint32_t p_PID_Control(int16_t i16Error, void *psPID)
{
    uint32_t u32Result;
    uint16_t u16Rubbish;

    __asm__ __volatile__ (
        /* Integrator */
        "push A \n\t"                        /* 1T:                                              [ASM] A = i16Error, X = psPID */
        "muls YA, A, [X++] \n\t"             /* 4T: [C] i32Result = i16Error * psPID->i16CoefI;  [ASM] YA = i16Error * i16CoefI; X = Pointer to SumError */
        "movu D, #0 \n\t"                    /* 1T: [C] u32Temp = 0UL;                           [ASM] D = minimum threshold Integrator (0) */
        "add YA, [X++] \n\t"                 /* 2T: [C] i32Result += psPID->u32SumError;         [ASM] Integrator;  X = Pointer to u32SumErrorMax */
        "jn p_PID_Ctrl_10_%= \n\t"           /* 1T: [C] if ( i32Result >= 0 ) */
        "mov D, [X] \n\t"                    /* 2T: [C] {  u32Temp = psPID->u32SumErrorMax;      [ASM] D = maximum threshold Integrator */
        "cmp D, YA \n\t"                     /* 2T: [C]    if ( u32Temp > i32Result ) */
        "jn p_PID_Ctrl_10_%= \n\t"           /* 1T: */
        "mov D, YA \n\t"                     /* 2T: [C]    { u32Temp = i32Result; }              [ASM] D = Integrator of i16Error * i16CoefI */
        "p_PID_Ctrl_10_%=: \n\t"             /*     [C] } */
        "add X, #-4 \n\t"                    /* 1T:                                              [ASM] X = Pointer to SumError */
        "mov [X], D \n\t"                    /* 2T: [C] psPID->u32SumError = u32Temp;            [ASM] Save SumError */
        "add X, #8 \n\t"                     /* 1T:                                              [ASM] X = Pointer to i16CoefP */
        "pop A \n\t"                         /* 1T:                                              [ASM] Restore i16Error */
        /* Proportional */
        "macs D, A, [X++] \n\t"              /* 6T: [C] u32Temp += i16Error * psPID->i16CoefP;   [ASM] i16Error x CoefP; X = Pointer to i16PrevError */
#if (_SUPPORT_PID_D_COEF != FALSE)
        /* Differential */
        "mov Y, A \n\t"                      /* 1T:                                              [ASM] i16Error */
        "sub A, [X] \n\t"                    /* 1T: [C] i16DifFErr = i16Error - psPID->i16PrevError; [ASM] (i16Error - i16PrevError) */
        "mov [X++], Y \n\t"                  /* 1T: [C] psPID->16PrevErr = i16Error;             [ASM] Store i16Error; X = Pointer to i16CoefD */
        "macs D, A, [X++] \n\t"              /* 6T: [C] u32Temp += i16DifFErr * psPID->i16CoefD; [ASM] (i16Error - i16Error[t-1]) x CoefD; X = Pointer to u16MinOutput */
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
        "asr D, #12 \n\t"                    /* 2T: [C] u32Temp = u32Temp / 2^12;                [ASM] GN_PID */
        /* Minimum Output Limit */
        "movu YA, [X++] \n\t"                /* 1T: [C] u32Result = (uint32_t)psPID->u16MinOutput; [ASM] YA = Minimum Output Limit; X = Pointer to u16MaxOutput */
        "cmp D, YA \n\t"                     /* 2T  [C] if (u32Temp >= u32Result) */
        "jn p_PID_Ctrl_Exit_%= \n\t"         /* 1T */
        /* Maximum Output Limit */
        "mov YA, [X] \n\t"                   /* 2T: [C] { u32Result = psPID->u32MaxOutput;       [ASM] YA = Maximum Output Limit */
        "cmp D, YA \n\t"                     /* 2T: [C]   if (u32Temp < u32Result) */
        "jsg p_PID_Ctrl_Exit_%= \n\t"        /* 1T */
        "mov YA, D \n\t"                     /* 2T:       { u32Result = u32Temp; }               [ASM] YA = Output */
        "p_PID_Ctrl_Exit_%=:"                /*         } */
        : "=b" (u32Result), "=x" (u16Rubbish)
        : "a" (i16Error), "x" (psPID)
        : "D"
        );

    return (u32Result);
} /* End of p_PID_Control() */

/* ************************************************************************** *
 * Function : p_iPID_Control()                                                *
 * Purpose  : PID Controller (signed output, without clipping)                *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t i16Error: PI(D) input (error-signal)                    *
 *            void *psPID: Pointer to PID-structure                           *
 *                     [0]:  (int16_t) i16CoefI: I-Coefficient PID            *
 *                     [2]: (uint32_t) u32SumError: Integrator PID            *
 *                     [6]: (uint32_t) u32SumErrorMax: Integrator Max (Skipped)*
 *                    [10]:  (int16_t) i16CoefP: P-Coefficient PID            *
 *                    [12]:  (int16_t) i16PrevError: Previous Error (optional)*
 *                    [14]:  (int16_t) i16CoefD: D-Coefficient PID (optional) *
 *                    [16]: (uint16_t) u16MinOutput: Minimum output (Skipped) *
 *                    [18]: (uint32_t) u32MaxOutput: Maximum output (Skipped) *
 * Return   : int16_t: PI(D) output                                           *
 * ************************************************************************** *
 * Comments : This PI(D) is intentional for ID (which can be negative)        *
 * ************************************************************************** */
static __inline__ uint16_t p_iPID_Control(int16_t i16Error, void *psPID) __attribute__ ((always_inline));
static __inline__ uint16_t p_iPID_Control(int16_t i16Error, void *psPID)
{
    int16_t i16Result;
    uint16_t u16Rubbish;

    __asm__ __volatile__ (
        /* Integrator */
        "muls D, A, [X++] \n\t"              /* 4T: [C] i32Temp = i16Error * psPID->i16CoefI;    [ASM] YA = i16Error * i16CoefI; X = Pointer to SumError */
        "add D, [X] \n\t"                    /* 2T: [C] i32Temp += psPID->u32SumError;           [ASM] Integrator;  X = Pointer to u32SumErrorMax */
        "mov [X++], D \n\t"                  /* 2T: [C] psPID->u32SumError = i32Temp;            [ASM] Save SumError */
        "add X, #4 \n\t"                     /* 1T:                                              [ASM] X = Pointer to i16CoefP (Skip "Integrator Max PID") */
        /* Proportional */
        "macs D, A, [X++] \n\t"              /* 6T: [C] i32Temp += i16Error * psPID->i16CoefP;   [ASM] i16Error x CoefP; X = Pointer to i16PrevError */
#if (_SUPPORT_PID_D_COEF != FALSE)
        /* Differential */
        "mov Y, A \n\t"                      /* 1T:                                              [ASM] i16Error */
        "sub A, [X] \n\t"                    /* 1T: [C] i16DifFErr = i16Error - psPID->i16PrevError; [ASM] (i16Error - i16PrevError) */
        "mov [X++], Y \n\t"                  /* 1T: [C] psPID->16PrevErr = i16Error;             [ASM] Store i16Error; X = Pointer to i16CoefD */
        "macs D, A, [X] \n\t"                /* 6T: [C] i32Temp += i16DifFErr * psPID->i16CoefD; [ASM] (i16Error - i16Error[t-1]) x CoefD */
#endif /* (_SUPPORT_PID_D_COEF != FALSE) */
        "asr D, #12 \n\t"                    /* 2T: [C] i32Temp = i32Temp / 2^12;                [ASM] GN_PID */
        "mov YA, D"                          /* 2T: [C] i16Result = (int16_t)i32Temp;            [ASM] YA = Output */
        : "=a" (i16Result), "=x" (u16Rubbish), "=y" (u16Rubbish)
        : "a" (i16Error), "x" (psPID)
        : "D"
        );

    return (i16Result);
} /* End of p_iPID_Control() */

static __inline__ uint16_t* p_X(uint16_t u16Index) __attribute__ ((always_inline));
static __inline__ uint16_t* p_X(uint16_t u16Index)
{
    uint16_t *u16Result;

    __asm__ __volatile__
    (
        "mov X, %[index] \n\t"
        "asl X \n\t"
        "add X, #_c_au16MicroStepVector3PH"
        : "=x" (u16Result)
        : [index] "r" (u16Index)
    );

    return (u16Result);
} /* End of p_X() */

/* ************************************************************************** *
 * Function : p_ClipMinMaxI16()                                               *
 * Purpose  : Clip/Limit input value between minimum and maximum              *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t u16Value: input value                                  *
 *            uint16_t u16Min: Minimum value                                  *
 *            uint16_t u16Max: Maximum value                                  *
 * Return   : uint16_t: value between min and max.                            *
 * ************************************************************************** *
 * Comments :                                                                 *
 * ************************************************************************** */
static __inline__ int16_t p_ClipMinMaxI16(int16_t i16Value, int16_t i16Min,
                                          int16_t i16Max) __attribute__ ((always_inline));
static __inline__ int16_t p_ClipMinMaxI16(int16_t i16Value, int16_t i16Min, int16_t i16Max)
{
    int16_t i16Rubbish;

    __asm__ __volatile__
    (
        "cmp %[val], %[max] \n\t"     /* u16Max check */
        "jsg p_ClipMinMaxI16_10_%= \n\t"
        "mov %[max], %[val] \n\t"     /* Use u16Value */
    "p_ClipMinMaxI16_10_%=: \n\t"
        "lod %[val], %[min] \n\t"
        "cmp %[max], %[val] \n\t"     /* u16Min check */
        "jsl p_ClipMinMaxI16_Exit_%= \n\t"
        "mov %[val], %[max] \n\t"     /* Use u16Value */
    "p_ClipMinMaxI16_Exit_%=: \n\t"
        : [val] "+r" (i16Value), "=r" (i16Rubbish)
        : [min] "" (i16Min), [max] "1" (i16Max)
    );

    return (i16Value);
} /* End of p_ClipMinMaxI16() */

/* ************************************************************************** *
 * Function : p_ClipMinMaxU16()                                               *
 * Purpose  : Clip/Limit input value between minimum and maximum              *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t u16Value: input value                                  *
 *            uint16_t u16Min: Minimum value                                  *
 *            uint16_t u16Max: Maximum value                                  *
 * Return   : uint16_t: value between min and max.                            *
 * ************************************************************************** *
 * Comments :                                                                 *
 * ************************************************************************** */
static __inline__ uint16_t p_ClipMinMaxU16(uint16_t u16Value, uint16_t u16Min,
                                           uint16_t u16Max) __attribute__ ((always_inline));
static __inline__ uint16_t p_ClipMinMaxU16(uint16_t u16Value, uint16_t u16Min, uint16_t u16Max)
{
    uint16_t u16Rubbish;

    __asm__ __volatile__
    (
        "cmp %[val], %[max] \n\t"     /* u16Max check */
        "jug p_ClipMinMaxU16_10_%= \n\t"
        "mov %[max], %[val] \n\t"     /* Use u16Value */
    "p_ClipMinMaxU16_10_%=: \n\t"
        "lod %[val], %[min] \n\t"
        "cmp %[max], %[val] \n\t"     /* u16Min check */
        "jul p_ClipMinMaxU16_Exit_%= \n\t"
        "mov %[val], %[max] \n\t"     /* Use u16Value */
    "p_ClipMinMaxU16_Exit_%=: \n\t"
        : [val] "+r" (u16Value), "=r" (u16Rubbish)
        : [min] "" (u16Min), [max] "1" (u16Max)
    );

    return (u16Value);
} /* End of p_ClipMinMaxU16() */

static __inline__ uint16_t p_IdxToAngle(uint16_t idx, uint16_t max) __attribute__ ((always_inline));
static __inline__ uint16_t p_IdxToAngle(uint16_t idx, uint16_t max)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;

    __asm__ __volatile__
    (
        "asl Y \n\t"
        "add Y, #1 \n\t"
        "mov A, #0 \n\t"
        "asl X \n\t"
        "divu YA, X \n\t"
        "divu YA, X"
        : "=a" (u16Result), "=y" (u16Rubbish), "=x" (u16Rubbish)
        : "y" (idx), "x" (max)
    );

    return (u16Result);
} /* End of p_IdxToAngle() */

#if TRUE
#define _HIGH_ACCURACY       FALSE
/* ************************************************************************** *
 * Function : p_AproxSqrtU16_I16byI16()                                       *
 * Purpose  : Square-root of two square-integer values                        *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t i16X: First integer value                               *
 *            int16_t i16Y: Second integer value                              *
 * Return   : uint16_t: Square-root result (approximate)                      *
 * ************************************************************************** *
 * Details  : Based on Heron's method                                         *
 * ************************************************************************** *
 * Comments : result = SQRT( X*X + Y*Y)                                       *
 * Special case: |X|+|Y| = 0 --> 0, and |X|+|Y| = 1 --> 1                     *
 * Note     : |X|+|Y| < 65536                                                 *
 * Performance: 69Cy (@ 28MHz: 2.46us) (High_Accuracy: 89Cy (@ 28MHz: 3.18us) *
 * ************************************************************************** */
static __inline__ uint16_t p_AproxSqrtU16_I16byI16(int16_t i16X, int16_t i16Y) __attribute__ ((always_inline));
static __inline__ uint16_t p_AproxSqrtU16_I16byI16(int16_t i16X, int16_t i16Y)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;
    __asm__ __volatile__ (
        "mov A, X \n\t"                                                         /* [1] */
        "muls D, A, A \n\t"                                                     /* [4] D = X^2 */
        "asl A \n\t"                                                            /* [1] Sign into Carry */
        "subc A, A \n\t"                                                        /* [1] A = X >> 15; A - A - Cy = 0 (Pos) or -1 (Neg) */
        "add X, A \n\t"                                                         /* [1] */
        "xor X, A \n\t"                                                         /* [1] X = |X| */
        "mov A, Y \n\t"                                                         /* [1] */
        "macs D, A, A \n\t"                                                     /* [6] D = X^2 + Y^2*/
        "asl A \n\t"                                                            /* [1] */
        "subc A, A \n\t"                                                        /* [1] A = Y >> 15 */
        "add Y, A \n\t"                                                         /* [1] */
        "xor A, Y \n\t"                                                         /* [1] A = |Y| */
        "add A, X \n\t"                                                         /* [1] A = |Y| + |X| */
        "cmp A, #1 \n\t"                                                        /* [1] */
        "jule p_AproxSqrtU16_End_%= \n\t"                                       /* [1] */
        "mulu YA, A, #55100 \n\t"                                               /* [5] Y = (|Y| + |X|) * 0.84076 */
        "mov X, Y \n\t"                                                         /* [1] Sqrt = X = (|Y| + |X|) * 0.84076 */
        "mov YA, D \n\t"                                                        /* [2] YA = X^2 + Y^2 */
        "divu YA, X \n\t"                                                       /* [8] */
        "divu YA, X \n\t"                                                       /* [8] A = (X^2 + Y^2) / Sqrt */
        "add X, A \n\t"                                                         /* [1] */
        "rrc X \n\t"                                                            /* [1] Sqrt = (Sqrt + A)/2 */
#if (_HIGH_ACCURACY != FALSE)
        /* Higher accuracy */
        "mov YA, D \n\t"                                                        /* [2] YA = X^2 + Y^2 */
        "divu YA, X \n\t"                                                       /* [8] */
        "divu YA, X \n\t"                                                       /* [8] A = (X^2 + Y^2) / Sqrt */
        "add X, A \n\t"                                                         /* [1] */
        "rrc X \n\t"                                                            /* [1] Sqrt = (Sqrt + A)/2 */
#endif /* (_HIGH_ACCURACY != FALSE) */
        "mov YA, D \n\t"                                                        /* [2] YA = X^2 + Y^2 */
        "divu YA, X \n\t"                                                       /* [8] */
        "divu YA, X \n\t"                                                       /* [8] A = (X^2 + Y^2) / Sqrt */
        "add A, X \n\t"                                                         /* [1] */
        "rrc A \n\t"                                                            /* [1] Sqrt = (Sqrt + A)/2 */
    "p_AproxSqrtU16_End_%=:\n\t"
        : "=a" (u16Result), "=y" (u16Rubbish), "=x" (u16Rubbish)
        : "x" (i16X), "y" (i16Y)
        : "D"
        );
    return (u16Result);
} /* End of p_AproxSqrtU16_I16byI16() */

#else /* FALSE */

/* ************************************************************************** *
 * Function : p_AproxSqrtU16_I16byI16()                                       *
 * Purpose  : Square-root of two square-integer values                        *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : int16_t i16X: First integer value                               *
 *            int16_t i16Y: Second integer value                              *
 * Return   : uint16_t: Square-root result (approximate)                      *
 * ************************************************************************** *
 * Comments : result = SQRT( X*X + Y*Y)                                       *
 * Note     : Alpha max plus beta min algorithm                               *
 * Performance: 62Cy worst-case (@ 28MHz: 2.21us)                             *
 * ************************************************************************** */
static __inline__ uint16_t p_AproxSqrtU16_I16byI16(int16_t i16X, int16_t i16Y) __attribute__ ((always_inline));
static __inline__ uint16_t p_AproxSqrtU16_I16byI16(int16_t i16X, int16_t i16Y)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;
    __asm__ __volatile__ (
        "mov A, Y \n\t"                                                         /* [1] A = Y */
        "asl A \n\t"                                                            /* [1] CY = Sign */
        "subc A, A \n\t"                                                        /* [1] A = Y >> 15 */
        "add Y, A \n\t"                                                         /* [1] */
        "xor Y, A \n\t"                                                         /* [1] Y = |Y| */
        "mov A, X \n\t"                                                         /* [1] A = X */
        "asl A \n\t"                                                            /* [1] CY = Sign */
        "subc A, A \n\t"                                                        /* [1] A = X >> 15 */
        "add X, A \n\t"                                                         /* [1] */
        "xor A, X \n\t"                                                         /* [1] A = |X| */
        "cmp A, Y \n\t"                                                         /* [1] */
        "jug p_AproxSqrtU16_10_%= \n\t"                                         /* [1/7] */
        "swap YA \n\t"                                                          /* [1] */
    "p_AproxSqrtU16_10_%=:\n\t"
        "push A \n\t"                                                           /* [1] */
        "mulu D, A, #64970 \n\t"                                                /* [5] Z0 = Max * ~127/128 */
        "mov A, #12425 \n\t"                                                    /* [2] */
        "macu D, A, Y \n\t"                                                     /* [6] Z0 += Min * ~3/16 */
        "pop A \n\t"                                                            /* [1] */
        "push D \n\t"                                                           /* [1] */
        "mulu D, A, #55335 \n\t"                                                /* [5] Z1 = Max * ~27/32 */
        "mov A, #36380 \n\t"                                                    /* [2] */
        "macu D, A, Y \n\t"                                                     /* [6] Z1 += Min * ~71/128 */
        "pop YA \n\t"                                                           /* [1] */
        "cmp D, YA \n\t"                                                        /* [2] */
        "jul p_AproxSqrtU16_Exit_%= \n\t"                                       /* [1/7] */
        "mov YA, D \n\t"                                                        /* [2] */
    "p_AproxSqrtU16_Exit_%=:\n\t"
        : "=y" (u16Result), "=a" (u16Rubbish), "=x" (u16Rubbish)
        : "x" (i16X), "y" (i16Y)
        : "D"
        );
    return (u16Result);
} /* End of p_AproxSqrtU16_I16byI16() */
#endif

/* ************************************************************************** *
 * Function : p_GetExMem()                                                    *
 * Purpose  : Get (read) Extended Memory (>64k) Word                          *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint32_t u20Addr: Address                                       *
 * Return   : uint16_t: value at address                                      *
 * ************************************************************************** *
 * Comments : MMP190716-1                                                     *
 * ************************************************************************** */
static __inline__ uint16_t p_GetExMem(uint32_t u20Addr) __attribute__ ((always_inline));
static __inline__ uint16_t p_GetExMem(uint32_t u20Addr)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;

    __asm__ __volatile__
    (
        "swap YA \n\t"
        "lsl AL, #2 \n\t"
        "lsl AL, #2 \n\t"
        "mov R, AL \n\t"
        "mov A, [Y] \n\t"
        : "=a" (u16Result), "=y" (u16Rubbish)
        : "b" (u20Addr)
        : "M"
    );

    return (u16Result);
} /* End of p_GetExMem() */

/* ************************************************************************** *
 * Function : p_DecNzPU8()                                                    *
 * Purpose  : Decrement in case non-zero pointed 8-bit variable               *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t *u8Addr: Address                                       *
 * Return   : -                                                               *
 * ************************************************************************** *
 * Comments : As (conditional) jumps are costly in CAMCU, this implementation *
 *            doesn't use any jump.                                           *
 * ************************************************************************** */
static __inline__ void p_DecNzPU8(uint8_t *u16Addr) __attribute__ ((always_inline));
static __inline__ void p_DecNzPU8(uint8_t *u16Addr)
{
    uint16_t u16Rubbish;

    __asm__ __volatile__
    (
        "mov AL, [X] \n\t"
        "snz Y \n\t"
        "sub AL, YL \n\t"
        "mov [X], AL"
        : "=a" (u16Rubbish), "=y" (u16Rubbish)
        : "x" (u16Addr)
    );
} /* End of p_DecNzPU8() */

/* ************************************************************************** *
 * Function : p_DecNzU8()                                                     *
 * Purpose  : Decrement 8-bit value in case non-zero                          *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint8_t u8Data: Value                                           *
 * Return   : (uint8_t) 8-bit value updated                                   *
 * ************************************************************************** *
 * Comments : As (conditional) jumps are costly in CAMCU, this implementation *
 *            doesn't use any jump.                                           *
 * ************************************************************************** */
static __inline__ uint8_t p_DecNzU8(uint8_t u8Data) __attribute__ ((always_inline));
static __inline__ uint8_t p_DecNzU8(uint8_t u8Data)
{
    uint8_t u8Result;
    uint16_t u16Rubbish;

    __asm__ __volatile__
    (
        "snz Y\n\t"
        "sub AL, YL\n\t"
        : "=al" (u8Result), "=y" (u16Rubbish)
        : "al" (u8Data)
    );
    return (u8Result);
} /* End of p_DecNzU8() */

/* ************************************************************************** *
 * Function : p_DecNzPU16()                                                   *
 * Purpose  : Decrement in case non-zero pointed 16-bit variable              *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t *u16Addr: Address                                      *
 * Return   : -                                                               *
 * ************************************************************************** *
 * Comments : As (conditional) jumps are costly in CAMCU, this implementation *
 *            doesn't use any jump.                                           *
 * ************************************************************************** */
static __inline__ void p_DecNzPU16(uint16_t *u16Addr) __attribute__ ((always_inline));
static __inline__ void p_DecNzPU16(uint16_t *u16Addr)
{
    uint16_t u16Rubbish;

    __asm__ __volatile__
    (
        "mov A, [X]\n\t"
        "snz Y\n\t"
        "sub A, Y\n\t"
        "mov [X], A"
        : "=a" (u16Rubbish), "=y" (u16Rubbish)
        : "x" (u16Addr)
    );
} /* End of p_DecNzPU16() */

/* ************************************************************************** *
 * Function : p_DecNzU16()                                                    *
 * Purpose  : Decrement value in case non-zero                                *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t u16Data: Value                                         *
 * Return   : (uint16_t) 16-bit Value                                         *
 * ************************************************************************** *
 * Comments : As (conditional) jumps are costly in CAMCU, this implementation *
 *            doesn't use any jump.                                           *
 * ************************************************************************** */
static __inline__ uint16_t p_DecNzU16(uint16_t u16Data) __attribute__ ((always_inline));
static __inline__ uint16_t p_DecNzU16(uint16_t u16Data)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;

    __asm__ __volatile__
    (
        "snz Y\n\t"
        "sub A, Y\n\t"
        : "=a" (u16Result), "=y" (u16Rubbish)
        : "a" (u16Data)
    );
    return (u16Result);
} /* End of p_DecNzU16() */

/* ************************************************************************** *
 * Function : p_GET_STACK()                                                   *
 * Purpose  : Get stack-pointer value                                         *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : -                                                               *
 * Return   : (uint16_t) Stack-pointer value                                  *
 * ************************************************************************** *
 * Comments : (MMP200506-1)                                                   *
 * ************************************************************************** */
static __inline__ uint16_t p_GET_STACK(void) __attribute__ ((always_inline));
static __inline__ uint16_t p_GET_STACK(void)
{
    uint16_t u16StackPointer;

    __asm__ __volatile__
    (
        "mov %[pointer], S"
        : [pointer] "=r" (u16StackPointer)
        :
    );
    return (u16StackPointer);
} /* End of p_GET_STACK() */

/* ************************************************************************** *
 * Function : p_MemU32_U16nU16()                                              *
 * Purpose  : Convert MSW + LSW to DWORD (uint32_t)                           *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : uint16_t u16MSW: Most Significant Word                          *
 *            uint16_t u16LSW: Least Significant Word                         *
 * Return   : uint32_t: 32-bit value                                          *
 * ************************************************************************** *
 * Comments :                                                                 *
 * ************************************************************************** */
static __inline__ uint32_t p_MemU32_U16nU16(uint16_t u16MSW, uint16_t u16LSW) __attribute__ ((always_inline));
static __inline__ uint32_t p_MemU32_U16nU16(uint16_t u16MSW, uint16_t u16LSW)
{
    uint32_t u32Result;
    __asm__ __volatile__
    (
        ""
        : "=b" (u32Result)
        : "y" (u16MSW), "a" (u16LSW)
    );

    return (u32Result);
} /* End of p_MemU32_U16nU16() */

/* ************************************************************************** *
 * Function : p_AwdAck()                                                      *
 * Purpose  : Acknowledge AWD when WIN_OPEN                                   *
 * Author   : mmp                                                             *
 * ************************************************************************** *
 * Input    : Nothing                                                         *
 * Return   : Nothing                                                         *
 * ************************************************************************** *
 * Comments : (MMP200625-2)                                                   *
 * ************************************************************************** */
static __inline__ void p_AwdAck(void) __attribute__ ((always_inline));
static __inline__ void p_AwdAck(void)
{
    __asm__ __volatile__ (
        "lod C, io:0x0F.7 \n\t"                                                 /* WIN_OPEN --> C */
        "mov io:0x0E.6, C"                                                      /* C --> ACK */
        :
        :
        );
} /* End of p_AwdAck() */

#if (_SUPPORT_I2C != FALSE) && ((_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2) || \
    (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1))
#if defined (__MLX81160__)
static __inline__ uint16_t p_I2C_MasterSendData(uint8_t u8Data) __attribute__ ((always_inline));
static __inline__ uint16_t p_I2C_MasterSendData(uint8_t u8Data)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;

    __asm__ __volatile__
    (
        "mov Cx, #8 \n\t"                   /* Data bits: 8 */
        "lod X, 0x020E \n\t"                /* IO_PORT_IO_OUT_SOFT */
        "p_I2C_MSD_Loop_%=:\n\t"
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_1)
        "and X, #0xFFFD \n\t"               /* SDA - IO[1] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "and X, #0xFFFB \n\t"               /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "and X, #0xFFF7 \n\t"               /* SDA - IO[3] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "and X, #0xFF7F \n\t"               /* SDA - IO[7] */
#endif
        "lod YH, #0x00 \n\t"
        "p_I2C_MSD_10_%=: \n\t"
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "lod A, 0x0052 \n\t"                /* IO_MLX16_ITC_PEND1 */
        "and A, #0x1000 \n\t"               /* B_MLX16_ITC_PEND1_PWM_MASTER2_CMP */
        "je  p_I2C_MSD_10_%= \n\t"
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "lod A,0x0052 \n\t"                 /* IO_MLX16_ITC_PEND1 */
        "and A, #0x0004 \n\t"               /* B_MLX16_ITC_PEND1_CTIMER1_1 */
        "je  p_I2C_MSD_10_%= \n\t"
#endif
        "asl YL \n\t"
        "jnc p_I2C_MSD_20_%= \n\t"
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_1)
        "or X, #0x0002 \n\t"                /* SDA - IO[1] */
        "lod YH, #0x02 \n\t"                /* SDA - IO[1] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "or X, #0x0004 \n\t"                /* SDA - IO[2] */
        "lod YH, #0x04 \n\t"                /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "or X, #0x0008 \n\t"                /* SDA - IO[3] */
        "lod YH, #0x08 \n\t"                /* SDA - IO[3] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "or X, #0x0080 \n\t"                /* SDA - IO[7] */
        "lod YH, #0x80 \n\t"                /* SDA - IO[7] */
#endif
        "p_I2C_MSD_20_%=: \n\t"
        "mov 0x020E, X \n\t"                /* IO_PORT_IO_OUT_SOFT, <800ns between falling edge clock and TX-bit @ 32MHz */
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "mov 0x0052, A \n\t"                /* IO_MLX16_ITC_PEND1 */
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "mov 0x0052, A \n\t"
#endif
        "jmp p_I2C_MSD_30_%= \n\t"
        "p_I2C_MSD_Error_%=: \n\t"
        "mov AL, #0x03 \n\t"                /* Error */
        "jmp p_I2C_MSD_Exit_%= \n\t"
        "p_I2C_MSD_30_%=: \n\t"
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "lod A, 0x0052 \n\t"                /* IO_MLX16_ITC_PEND1 */
        "and A, #0x2000 \n\t"               /* B_MLX16_ITC_PEND1_PWM_MASTER2_END */
        "je  p_I2C_MSD_30_%= \n\t"
        "mov 0x0052, A \n\t"                /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_END */
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "lod A, 0x0052 \n\t"                /* IO_MLX16_ITC_PEND1 */
        "and A, #0x0010 \n\t"               /* B_MLX16_ITC_PEND1_CTIMER1_3 */
        "je  p_I2C_MSD_30_%= \n\t"
        "mov 0x0052, A \n\t"                /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_3 */
#endif
        "lod A, 0x0204 \n\t"                /* IO_PORT_IO_IN */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_1)
        "and AL, #0x02 \n\t"                /* SDA - IO[1] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "and AL, #0x04 \n\t"                /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "and AL, #0x08 \n\t"                /* SDA - IO[3] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "and AL, #0x80 \n\t"                /* SDA - IO[7] */
#endif
        "cmp AL, YH \n\t"
        "jne p_I2C_MSD_Error_%= \n\t"
        "djnz Cx, p_I2C_MSD_Loop_%= \n\t"
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_1)
        "or X, #0x0002 \n\t"                /* SDA - IO[1] = High */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "or X, #0x0004 \n\t"                /* SDA - IO[2] = High */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "or X, #0x0008 \n\t"                /* SDA - IO[3] = High */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "or X, #0x0080 \n\t"                /* SDA - IO[7] = High */
#endif
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "lod Y, #0x0052 \n\t"               /* IO_MLX16_ITC_PEND1 */
        "p_I2C_MSD_40_%=: \n\t"
        "lod A, [Y] \n\t"
        "and A, #0x1000 \n\t"               /* B_MLX16_ITC_PEND1_PWM_MASTER2_CMP */
        "je  p_I2C_MSD_40_%= \n\t"
        "mov 0x020E, X \n\t"                /* IO_PORT_IO_OUT_SOFT */
        "mov [Y], A \n\t"                   /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_PWM_MASTER2_CMP */
        "p_I2C_MSD_50_%=: \n\t"
        "lod A, [Y] \n\t"                   /* IO_MLX16_ITC_PEND1 */
        "and A, #0x2000 \n\t"               /* B_MLX16_ITC_PEND1_PWM_MASTER2_END */
        "je  p_I2C_MSD_50_%= \n\t"
        "mov [Y], A \n\t"                   /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_PWM_MASTER2_END */
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "lod Y, #0x0052 \n\t"               /* IO_MLX16_ITC_PEND1 */
        "p_I2C_MSD_40_%=: \n\t"
        "lod A, [Y] \n\t"
        "and A, #0x0004 \n\t"               /* B_MLX16_ITC_PEND1_CTIMER1_1 */
        "je  p_I2C_MSD_40_%= \n\t"
        "mov 0x020E, X \n\t"                /* IO_PORT_IO_OUT_SOFT */
        "mov [Y], A \n\t"                   /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_1 */
        "p_I2C_MSD_50_%=: \n\t"
        "lod A, [Y] \n\t"                   /* IO_MLX16_ITC_PEND1 */
        "and A, #0x0010 \n\t"               /* B_MLX16_ITC_PEND1_CTIMER1_3 */
        "je  p_I2C_MSD_50_%= \n\t"
        "mov [Y], A \n\t"                   /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_3 */
#endif
        "lod A, 0x0204 \n\t"                /* IO_PORT_IO_IN */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_1)
        "and A, #0x0002 \n\t"               /* SDA - IO[1] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "and A, #0x0004 \n\t"               /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "and A, #0x08 \n\t"                 /* SDA - IO[3] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "and A, #0x0080 \n\t"               /* SDA - IO[7] */
#endif
        "snz A \n\t"           /* IO[11:0] --> bit 0 */
        "p_I2C_MSD_Exit_%=:\n"
        : "=a" (u16Result), "=x" (u16Rubbish), "=y" (u16Rubbish)
        : "y" (u8Data)
    );

    return (u16Result);
} /* End of p_I2C_MasterSendData() */

static __inline__ uint16_t p_I2C_MasterReceiveData(uint8_t u8Ack) __attribute__ ((always_inline));
static __inline__ uint16_t p_I2C_MasterReceiveData(uint8_t u8Ack)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;

    __asm__ __volatile__
    (
        "mov Cx, #8 \n\t"
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "mov Y, #0x0052 \n\t"               /* IO_MLX16_ITC_PEND1 */
        "p_I2C_MRD_Loop_%=: \n\t"
        "p_I2C_MRD_10_%=: \n\t"
        "lod X, [Y] \n\t"
        "and X, #0x1000 \n\t"               /* B_MLX16_ITC_PEND1_PWM_MASTER2_CMP */
        "je  p_I2C_MRD_10_%= \n\t"
        "mov [Y], X \n\t"                   /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "asl AH \n\t"
        "p_I2C_MRD_20_%=: \n\t"
        "lod X, [Y] \n\t"
        "and X, #0x2000 \n\t"               /* B_MLX16_ITC_PEND1_PWM_MASTER2_END */
        "je  p_I2C_MRD_20_%= \n\t"
        "mov [Y], X \n\t"                   /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_END */
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "mov Y, #0x0052 \n\t"               /* IO_MLX16_ITC_PEND1 */
        "p_I2C_MRD_Loop_%=: \n\t"
        "p_I2C_MRD_10_%=: \n\t"
        "lod X, [Y] \n\t"
        "and X, #0x0004 \n\t"               /* B_MLX16_ITC_PEND1_CTIMER1_1 */
        "je  p_I2C_MRD_10_%= \n\t"
        "mov [Y], X \n\t"                   /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_1 */
        "asl AH \n\t"
        "p_I2C_MRD_20_%=: \n\t"
        "lod X, [Y] \n\t"
        "and X, #0x0010 \n\t"               /* B_MLX16_ITC_PEND1_CTIMER1_3 */
        "je  p_I2C_MRD_20_%= \n\t"
        "mov [Y], X \n\t"                   /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_3 */
#endif
        "lod X, 0x0204 \n\t"                /* IO_PORT_IO_IN */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_1)
        "and X, #0x0002 \n\t"               /* SDA - IO[1] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "and X, #0x0004 \n\t"               /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "and X, #0x08 \n\t"                 /* SDA - IO[3] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "and X, #0x80 \n\t"                 /* SDA - IO[7] */
#endif
        "je p_I2C_MRD_30_%= \n\t"
        "or AH, #1 \n\t"
        "p_I2C_MRD_30_%=:"
        "djnz Cx, p_I2C_MRD_Loop_%= \n\t"
        "lod Y, 0x020E \n\t"                /* IO_PORT_IO_OUT_SOFT */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_1)
        "and Y, #0xFFFD \n\t"               /* SDA - IO[1] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "and Y, #0xFFFB \n\t"               /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "and Y, #0xFFF7 \n\t"               /* SDA - IO[3] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "and Y, #0xFF7F \n\t"               /* SDA - IO[7] */
#endif
        "or AL, AL \n\t"
        "je p_I2C_MRD_40_%= \n\t"
        "or Y, #0x0002 \n\t"                /* SDA - IO[1] */
        "p_I2C_MRD_40_%=: \n\t"
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "lod X, 0x0052 \n\t"                /* IO_MLX16_ITC_PEND1 */
        "and X, #0x1000 \n\t"               /* B_MLX16_ITC_PEND1_PWM_MASTER2_CMP */
        "je  p_I2C_MRD_40_%= \n\t"
        "mov 0x020E, Y \n\t"                /* IO_PORT_IO_OUT_SOFT */
        "mov 0x0052, X \n\t"                /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "p_I2C_MRD_50_%=: \n\t"
        "lod X, 0x0052 \n\t"                /* IO_MLX16_ITC_PEND1 */
        "and X, #0x2000 \n\t"               /* B_MLX16_ITC_PEND1_PWM_MASTER2_END */
        "je  p_I2C_MRD_50_%= \n\t"
        "mov 0x0052, X \n\t"                /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_END */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "or Y, #0x0002 \n\t"                /* SDA - IO[1] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "or Y, #0x0004 \n\t"                /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "or Y, #0x0008 \n\t"                /* SDA - IO[3] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "or Y, #0x0080 \n\t"                /* SDA - IO[7] */
#endif
        "p_I2C_MRD_60_%=: \n\t"
        "lod X, 0x0052 \n\t"                /* IO_MLX16_ITC_PEND1 */
        "and X, #0x1000 \n\t"               /* B_MLX16_ITC_PEND1_PWM_MASTER2_CMP */
        "je  p_I2C_MRD_60_%= \n\t"
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "lod X, 0x0052 \n\t"                /* IO_MLX16_ITC_PEND1 */
        "and X, #0x0004 \n\t"               /* B_MLX16_ITC_PEND1_CTIMER1_1 */
        "je  p_I2C_MRD_40_%= \n\t"
        "mov 0x020E, Y \n\t"                /* IO_PORT_IO_OUT_SOFT */
        "mov 0x0052, X \n\t"                /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_1 */
        "p_I2C_MRD_50_%=: \n\t"
        "lod X, 0x0052 \n\t"                /* IO_MLX16_ITC_PEND1 */
        "and X, #0x0010 \n\t"               /* B_MLX16_ITC_PEND1_CTIMER1_3 */
        "je  p_I2C_MRD_50_%= \n\t"
        "mov 0x0052, X \n\t"                /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_3 */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "or Y, #0x0002 \n\t"                /* SDA - IO[1] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "or Y, #0x0004 \n\t"                /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "or Y, #0x0008 \n\t"                /* SDA - IO[3] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "or Y, #0x0080 \n\t"                /* SDA - IO[7] */
#endif
        "p_I2C_MRD_60_%=: \n\t"
        "lod X, 0x0052 \n\t"                /* IO_MLX16_ITC_PEND1 */
        "and X, #0x0004 \n\t"               /* B_MLX16_ITC_PEND1_CTIMER1_1 */
        "je  p_I2C_MRD_60_%= \n\t"
#endif
        "mov 0x020E, Y \n\t"                /* IO_PORT_IO_OUT_SOFT */
        "swap A"
        : "=a" (u16Result), "=x" (u16Rubbish), "=y" (u16Rubbish)
        : "a" (u8Ack)
    );

    return (u16Result);
} /* End of p_I2C_MasterReceiveData() */

#elif defined (__MLX81332__) || defined (__MLX81334__)
static __inline__ uint16_t p_I2C_MasterSendData(uint8_t u8Data) __attribute__ ((always_inline));
static __inline__ uint16_t p_I2C_MasterSendData(uint8_t u8Data)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;

    __asm__ __volatile__
    (
        "mov Cx, #8 \n\t"     /* Data bits: 8 */
        "lod X, 0x01E6 \n\t"  /* IO_PORT_IO_OUT_SOFT */
        "p_I2C_MSD_Loop_%=:\n\t"
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "and X, #0xFFFB \n\t" /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "and X, #0xFFF7 \n\t" /* SDA - IO[3] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "and X, #0xFF7F \n\t" /* SDA - IO[7] */
#endif
        "lod YH, #0x00 \n\t"
        "p_I2C_MSD_10_%=: \n\t"
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "lod A, 0x0054 \n\t"  /* IO_MLX16_ITC_PEND2 */
        "and A, #0x0008 \n\t" /* B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "je  p_I2C_MSD_10_%= \n\t"
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "lod A, 0x0052 \n\t"  /* IO_MLX16_ITC_PEND1 */
        "and A, #0x0100 \n\t" /* B_MLX16_ITC_PEND1_CTIMER1_1 */
        "je  p_I2C_MSD_10_%= \n\t"
#endif
        "asl YL \n\t"
        "jnc p_I2C_MSD_20_%= \n\t"
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "or X, #0x0004 \n\t" /* SDA - IO[2] */
        "lod YH, #0x04 \n\t" /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "or X, #0x0008 \n\t" /* SDA - IO[3] */
        "lod YH, #0x08 \n\t" /* SDA - IO[3] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "or X, #0x0080 \n\t" /* SDA - IO[7] */
        "lod YH, #0x80 \n\t" /* SDA - IO[7] */
#endif
        "p_I2C_MSD_20_%=: \n\t"
        "mov 0x01E6, X \n\t" /* IO_PORT_IO_OUT_SOFT; <800ns between falling edge clock and TX-bit @ 32MHz */
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "mov 0x0054, A \n\t" /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "mov 0x0052, A \n\t" /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_1 */
#endif
        "jmp p_I2C_MSD_30_%= \n\t"
        "p_I2C_MSD_Error_%=: \n\t"
        "mov AL, #0x03 \n\t" /* Error */
        "jmp p_I2C_MSD_Exit_%= \n\t"
        "p_I2C_MSD_30_%=: \n\t"
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "lod A, 0x0054 \n\t"  /* IO_MLX16_ITC_PEND2 */
        "and A, #0x0010 \n\t" /* B_MLX16_ITC_PEND2_PWM_MASTER2_END */
        "je  p_I2C_MSD_30_%= \n\t"
        "mov 0x0054, A \n\t"  /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_END */
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "lod A, 0x0052 \n\t"  /* IO_MLX16_ITC_PEND1 */
        "and A, #0x0400 \n\t" /* B_MLX16_ITC_PEND1_CTIMER1_3 */
        "je  p_I2C_MSD_30_%= \n\t"
        "mov 0x0052, A \n\t"  /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_3 */
#endif
        "lod A, 0x01DE \n\t"  /* IO_PORT_IO_IN */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "and AL, #0x04 \n\t"  /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "and AL, #0x08 \n\t"  /* SDA - IO[3] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "and AL, #0x80 \n\t"  /* SDA - IO[7] */
#endif
        "cmp AL, YH \n\t"
        "jne p_I2C_MSD_Error_%= \n\t"
        "djnz Cx, p_I2C_MSD_Loop_%= \n\t"
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "or X, #0x0004 \n\t"  /* SDA - IO[2] = High */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "or X, #0x0008 \n\t"  /* SDA - IO[3] = High */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "or X, #0x0080 \n\t"  /* SDA - IO[7] = High */
#endif
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "lod Y, #0x0054 \n\t"  /* IO_MLX16_ITC_PEND2 */
        "p_I2C_MSD_40_%=: \n\t"
        "lod A, [Y] \n\t"
        "and A, #0x0008 \n\t" /* B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "je  p_I2C_MSD_40_%= \n\t"
        "mov 0x01E6, X \n\t"   /* IO_PORT_IO_OUT_SOFT */
        "mov [Y], A \n\t"      /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "p_I2C_MSD_50_%=: \n\t"
        "lod A, [Y] \n\t"      /* IO_MLX16_ITC_PEND2 */
        "and A, #0x0010 \n\t"  /* B_MLX16_ITC_PEND2_PWM_MASTER2_END */
        "je  p_I2C_MSD_50_%= \n\t"
        "mov [Y], A \n\t"      /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_END */
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "lod Y, #0x0052 \n\t"  /* IO_MLX16_ITC_PEND1 */
        "p_I2C_MSD_40_%=: \n\t"
        "lod A, [Y] \n\t"
        "and A, #0x0100 \n\t"  /* B_MLX16_ITC_PEND1_CTIMER1_1 */
        "je  p_I2C_MSD_40_%= \n\t"
        "mov 0x01E6, X \n\t"   /* IO_PORT_IO_OUT_SOFT */
        "mov [Y], A \n\t"      /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_1 */
        "p_I2C_MSD_50_%=: \n\t"
        "lod A, [Y] \n\t"      /* IO_MLX16_ITC_PEND1 */
        "and A, #0x0400 \n\t"  /* B_MLX16_ITC_PEND1_CTIMER1_3 */
        "je  p_I2C_MSD_50_%= \n\t"
        "mov [Y], A \n\t"      /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_3 */
#endif
        "lod A, 0x01DE \n\t"   /* IO_PORT_IO_IN */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "and A, #0x04 \n\t"    /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "and A, #0x08 \n\t"    /* SDA - IO[3] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "and A, #0x80 \n\t"    /* SDA - IO[7] */
#endif
        "snz A \n\t"           /* IO[11:0] --> bit 0 */
        "p_I2C_MSD_Exit_%=: \n"
        : "=a" (u16Result), "=x" (u16Rubbish), "=y" (u16Rubbish)
        : "y" (u8Data)
    );

    return (u16Result);
} /* End of p_I2C_MasterSendData() */

static __inline__ uint16_t p_I2C_MasterReceiveData(uint8_t u8Ack) __attribute__ ((always_inline));
static __inline__ uint16_t p_I2C_MasterReceiveData(uint8_t u8Ack)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;

    __asm__ __volatile__
    (
        "mov Cx, #8 \n\t"
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "mov Y, #0x0054 \n\t"  /* IO_MLX16_ITC_PEND2 */
        "p_I2C_MRD_Loop_%=: \n\t"
        "p_I2C_MRD_10_%=: \n\t"
        "lod X, [Y] \n\t"
        "and X, #0x0008 \n\t"  /* B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "je  p_I2C_MRD_10_%= \n\t"
        "mov [Y], X \n\t"      /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "asl AH \n\t"
        "p_I2C_MRD_20_%=: \n\t"
        "lod X, [Y] \n\t"
        "and X, #0x0010 \n\t"  /* B_MLX16_ITC_PEND2_PWM_MASTER2_END */
        "je  p_I2C_MRD_20_%= \n\t"
        "mov [Y], X \n\t"      /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_END */
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "mov Y, #0x0052 \n\t"  /* IO_MLX16_ITC_PEND1 */
        "p_I2C_MRD_Loop_%=: \n\t"
        "p_I2C_MRD_10_%=: \n\t"
        "lod X, [Y] \n\t"
        "and X, #0x0100 \n\t"  /* B_MLX16_ITC_PEND1_CTIMER1_1 */
        "je  p_I2C_MRD_10_%= \n\t"
        "mov [Y], X \n\t"      /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_1 */
        "asl AH \n\t"
        "p_I2C_MRD_20_%=: \n\t"
        "lod X, [Y] \n\t"
        "and X, #0x0400 \n\t"  /* B_MLX16_ITC_PEND1_CTIMER1_3 */
        "je  p_I2C_MRD_20_%= \n\t"
        "mov [Y], X \n\t"      /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_3 */
#endif
        "lod X, 0x01DE \n\t"   /* IO_PORT_IO_IN */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "and X, #0x0004 \n\t"  /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "and X, #0x08 \n\t"    /* SDA - IO[3] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "and X, #0x80 \n\t"    /* SDA - IO[7] */
#endif
        "je p_I2C_MRD_30_%= \n\t"
        "or AH, #1 \n\t"
        "p_I2C_MRD_30_%=: \n\t"
        "djnz Cx, p_I2C_MRD_Loop_%= \n\t"
        "lod Y, 0x01E6 \n\t"  /* IO_PORT_IO_OUT_SOFT */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "and Y, #0xFFFB \n\t" /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "and Y, #0xFFF7 \n\t" /* SDA - IO[3] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "and Y, #0xFF7F \n\t" /* SDA - IO[7] */
#endif
        "or AL, AL \n\t"
        "je p_I2C_MRD_40_%= \n\t"
        "or Y, #0x0080 \n\t"  /* SDA - IO[7] */
        "p_I2C_MRD_40_%=: \n\t"
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "lod X, 0x0054 \n\t"  /* IO_MLX16_ITC_PEND2 */
        "and X, #0x0008 \n\t" /* B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "je  p_I2C_MRD_40_%= \n\t"
        "mov 0x01E6, Y \n\t"  /* IO_PORT_IO_OUT_SOFT */
        "mov 0x0054, X \n\t"  /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "p_I2C_MRD_50_%=: \n\t"
        "lod X, 0x0054 \n\t"  /* IO_MLX16_ITC_PEND2 */
        "and X, #0x0010 \n\t" /* B_MLX16_ITC_PEND2_PWM_MASTER2_END */
        "je  p_I2C_MRD_50_%= \n\t"
        "mov 0x0054, X \n\t"  /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_END */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "or Y, #0x0004 \n\t"  /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "or Y, #0x0008 \n\t"  /* SDA - IO[3] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "or Y, #0x0080 \n\t"  /* SDA - IO[7] */
#endif
        "p_I2C_MRD_60_%=: \n\t"
        "lod X, 0x0054 \n\t"  /* IO_MLX16_ITC_PEND2 */
        "and X, #0x0008 \n\t" /* B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "je  p_I2C_MRD_60_%= \n\t"
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "lod X, 0x0052 \n\t"  /* IO_MLX16_ITC_PEND1 */
        "and X, #0x0100 \n\t" /* B_MLX16_ITC_PEND1_CTIMER1_1 */
        "je  p_I2C_MRD_40_%= \n\t"
        "mov 0x01E6, Y \n\t"  /* IO_PORT_IO_OUT_SOFT */
        "mov 0x0052, X \n\t"  /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_1 */
        "p_I2C_MRD_50_%=: \n\t"
        "lod X, 0x0052 \n\t"  /* IO_MLX16_ITC_PEND1 */
        "and X, #0x0400 \n\t" /* B_MLX16_ITC_PEND1_CTIMER1_3 */
        "je  p_I2C_MRD_50_%= \n\t"
        "mov 0x0052, X \n\t"  /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_3 */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "or Y, #0x0004 \n\t"  /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_3)
        "or Y, #0x0008 \n\t"  /* SDA - IO[3] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_7)
        "or Y, #0x0080 \n\t"  /* SDA - IO[7] */
#endif
        "p_I2C_MRD_60_%=: \n\t"
        "lod X, 0x0052 \n\t"  /* IO_MLX16_ITC_PEND1 */
        "and X, #0x0100 \n\t" /* B_MLX16_ITC_PEND1_CTIMER1_1 */
        "je  p_I2C_MRD_60_%= \n\t"
#endif
        "mov 0x01E6, Y \n\t"  /* IO_PORT_IO_OUT_SOFT */
        "swap A"
        : "=a" (u16Result), "=x" (u16Rubbish), "=y" (u16Rubbish)
        : "a" (u8Ack)
    );

    return (u16Result);
} /* End of p_I2C_MasterReceiveData() */

#elif defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__)

static __inline__ uint16_t p_I2C_MasterSendData(uint8_t u8Data) __attribute__ ((always_inline));
static __inline__ uint16_t p_I2C_MasterSendData(uint8_t u8Data)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;

    __asm__ __volatile__
    (
        "mov Cx, #8 \n\t"     /* Data bits: 8 */
        "lod X, 0x0206 \n\t"  /* IO_PORT_IO_OUT_SOFT */
        "p_I2C_MSD_Loop_%=: \n\t"
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "and X, #0xFFFB \n\t" /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_9)
        "and X, #0xFDFF \n\t" /* SDA - IO[9] */
#endif
        "lod YH, #0x00 \n\t"
        "p_I2C_MSD_10_%=: \n\t"
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "lod A, 0x0054 \n\t"  /* IO_MLX16_ITC_PEND2 */
        "and A, #0x0010 \n\t" /* B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "je  p_I2C_MSD_10_%= \n\t"
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "lod A, 0x0052 \n\t"  /* IO_MLX16_ITC_PEND1 */
        "and A, #0x0200 \n\t" /* B_MLX16_ITC_PEND1_CTIMER1_1 */
        "je  p_I2C_MSD_10_%= \n\t"
#endif
        "asl YL \n\t"
        "jnc p_I2C_MSD_20_%= \n\t"
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "or X, #0x0004 \n\t" /* SDA - IO[2] */
        "lod YH, #0x04 \n\t" /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_9)
        "or X, #0x0200 \n\t" /* SDA - IO[9] */
        "lod YH, #0x02 \n\t" /* SDA - IO[9] */
#endif
        "p_I2C_MSD_20_%=: \n\t"
#if defined (__MLX81340__)
        "mov 0x0206, X \n\t" /* IO_PORT_IO_OUT_SOFT, <800ns between falling edge clock and TX-bit @ 32MHz */
#elif defined (__MLX81344__) || defined (__MLX81346__)
        "mov 0x0208, X \n\t" /* IO_PORT_IO_OUT_SOFT, <800ns between falling edge clock and TX-bit @ 32MHz */
#endif
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "mov 0x0054, A \n\t" /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "mov 0x0052, A \n\t" /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_1 */
#endif
        "jmp p_I2C_MSD_30_%= \n\t"
        "p_I2C_MSD_Error_%=: \n\t"
        "mov AL, #0x03 \n\t" /* Error */
        "jmp p_I2C_MSD_Exit_%= \n\t"
        "p_I2C_MSD_30_%=: \n\t"
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "lod A, 0x0054 \n\t"  /* IO_MLX16_ITC_PEND2 */
        "and A, #0x0020 \n\t" /* B_MLX16_ITC_PEND2_PWM_MASTER2_END */
        "je  p_I2C_MSD_30_%= \n\t"
        "mov 0x0054, A \n\t"  /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_END */
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "lod A, 0x0052 \n\t"  /* IO_MLX16_ITC_PEND1 */
        "and A, #0x0800 \n\t" /* B_MLX16_ITC_PEND1_CTIMER1_3 */
        "je  p_I2C_MSD_30_%= \n\t"
        "mov 0x0052, A \n\t"  /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND1_CTIMER1_3 */
#endif
        "lod A, 0x01FA \n\t"  /* IO_PORT_IO_IN */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "and AL, #0x04 \n\t"  /* SDA - IO[2] */
        "cmp AL, YH \n\t"
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_9)
        "and AH, #0x02 \n\t"  /* SDA - IO[9] */
        "cmp AH, YH \n\t"
#endif
        "jne p_I2C_MSD_Error_%= \n\t"
        "djnz Cx, p_I2C_MSD_Loop_%= \n\t"
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "or X, #0x0004 \n\t"   /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_9)
        "or X, #0x0200 \n\t"   /* SDA - IO[9] */
#endif
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "lod Y, #0x0054 \n\t"  /* IO_MLX16_ITC_PEND2 */
        "p_I2C_MSD_40_%=:\n\t"
        "lod A, [Y] \n\t"
        "and A, #0x0010 \n\t"  /* B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "je  p_I2C_MSD_40_%= \n\t"
#if defined (__MLX81340__)
        "mov 0x0206, X \n\t"   /* IO_PORT_IO_OUT_SOFT */
#elif defined (__MLX81344__) || defined (__MLX81346__)
        "mov 0x0208, X \n\t"   /* IO_PORT_IO_OUT_SOFT */
#endif
        "mov [Y], A \n\t"      /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "p_I2C_MSD_50_%=: \n\t"
        "lod A, [Y] \n\t"      /* IO_MLX16_ITC_PEND2 */
        "and A, #0x0020 \n\t"  /* B_MLX16_ITC_PEND2_PWM_MASTER2_END */
        "je  p_I2C_MSD_50_%= \n\t"
        "mov [Y], A \n\t"      /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_END */
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "lod Y, #0x0052 \n\t"  /* IO_MLX16_ITC_PEND1 */
        "p_I2C_MSD_40_%=: \n\t"
        "lod A, [Y] \n\t"
        "and A, #0x0200 \n\t"  /* B_MLX16_ITC_PEND1_CTIMER1_1 */
        "je  p_I2C_MSD_40_%= \n\t"
#if defined (__MLX81340__)
        "mov 0x0206, X \n\t"   /* IO_PORT_IO_OUT_SOFT */
#elif defined (__MLX81344__) || defined (__MLX81346__)
        "mov 0x0208, X \n\t"   /* IO_PORT_IO_OUT_SOFT */
#endif
        "mov [Y], A \n\t"      /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_1 */
        "p_I2C_MSD_50_%=:\n\t"
        "lod A, [Y]\n\t"
        "and A, #0x0800 \n\t"  /* B_MLX16_ITC_PEND1_CTIMER1_3 */
        "je  p_I2C_MSD_50_%=\n\t"
        "mov [Y], A \n\t"      /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_3 */
#endif
        "lod A, 0x01FA \n\t"   /* IO_PORT_IO_IN */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "and A, #0x04 \n\t"    /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_9)
        "and A, #0x0200 \n\t"  /* SDA - IO[9] */
#endif
        "snz A \n\t"           /* IO[11:0] --> bit 0 */
        "p_I2C_MSD_Exit_%=: \n"
        : "=a" (u16Result), "=x" (u16Rubbish), "=y" (u16Rubbish)
        : "y" (u8Data)
    );

    return (u16Result);
} /* End of p_I2C_MasterSendData() */

static __inline__ uint16_t p_I2C_MasterReceiveData(uint8_t u8Ack) __attribute__ ((always_inline));
static __inline__ uint16_t p_I2C_MasterReceiveData(uint8_t u8Ack)
{
    uint16_t u16Result;
    uint16_t u16Rubbish;

    __asm__ __volatile__
    (
        "mov Cx, #8 \n\t"
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "mov Y, #0x0054 \n\t"  /* IO_MLX16_ITC_PEND2 */
        "p_I2C_MRD_Loop_%=: \n\t"
        "p_I2C_MRD_10_%=: \n\t"
        "lod X, [Y] \n\t"
        "and X, #0x0010 \n\t"  /* B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "je  p_I2C_MRD_10_%=\n\t"
        "mov [Y], X \n\t"      /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "asl AH\ n\t"
        "p_I2C_MRD_20_%=: \n\t"
        "lod X, [Y] \n\t"
        "and X, #0x0020 \n\t"  /* B_MLX16_ITC_PEND2_PWM_MASTER2_END */
        "je  p_I2C_MRD_20_%= \n\t"
        "mov [Y], X \n\t"      /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_END */
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "mov Y, #0x0052 \n\t"  /* IO_MLX16_ITC_PEND1 */
        "p_I2C_MRD_Loop_%=: \n\t"
        "p_I2C_MRD_10_%=: \n\t"
        "lod X, [Y] \n\t"
        "and X, #0x0200 \n\t"  /* B_MLX16_ITC_PEND1_CTIMER1_1 */
        "je  p_I2C_MRD_10_%= \n\t"
        "mov [Y], X \n\t"      /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_1 */
        "asl AH \n\t"
        "p_I2C_MRD_20_%=: \n\t"
        "lod X, [Y] \n\t"
        "and X, #0x0800 \n\t"  /* B_MLX16_ITC_PEND1_CTIMER1_3 */
        "je  p_I2C_MRD_20_%= \n\t"
        "mov [Y], X \n\t"      /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_3 */
#endif
        "lod X, 0x01FA \n\t"   /* IO_PORT_IO_IN */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "and X, #0x0004 \n\t"  /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_9)
        "and X, #0x0200 \n\t"  /* SDA - IO[9] */
#endif
        "je p_I2C_MRD_30_%= \n\t"
        "or AH, #1 \n\t"
        "p_I2C_MRD_30_%=: \n\t"
        "djnz Cx, p_I2C_MRD_Loop_%= \n\t"
#if defined (__MLX81340__)
        "lod Y, 0x0206 \n\t"  /* IO_PORT_IO_OUT_SOFT */
#elif defined (__MLX81344__) || defined (__MLX81346__)
        "lod Y, 0x0208 \n\t"  /* IO_PORT_IO_OUT_SOFT */
#endif
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "and Y, #0xFFFB \n\t" /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_9)
        "and Y, #0xFDFF \n\t" /* SDA - IO[9] */
#endif
        "or AL, AL \n\t"
        "je p_I2C_MRD_40_%= \n\t"
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "or Y, #0x0004 \n\t"  /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_9)
        "or Y, #0x0200 \n\t"  /* SDA - IO[9] */
#endif
        "p_I2C_MRD_40_%=: \n\t"
#if (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2)
        "lod X, 0x0054 \n\t"  /* IO_MLX16_ITC_PEND2 */
        "and X, #0x0010 \n\t" /* B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "je  p_I2C_MRD_40_%= \n\t"
#if defined (__MLX81340__)
        "mov 0x0206, Y \n\t"  /* IO_PORT_IO_OUT_SOFT */
#elif defined (__MLX81344__) || defined (__MLX81346__)
        "mov 0x0208, Y \n\t"  /* IO_PORT_IO_OUT_SOFT */
#endif
        "mov 0x0054, X \n\t"  /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "p_I2C_MRD_50_%=: \n\t"
        "lod X, 0x0054 \n\t"  /* IO_MLX16_ITC_PEND2 */
        "and X, #0x0020 \n\t" /* B_MLX16_ITC_PEND2_PWM_MASTER2_END */
        "je  p_I2C_MRD_50_%= \n\t"
        "mov 0x0054, X \n\t"  /* IO_MLX16_ITC_PEND2, Clear B_MLX16_ITC_PEND2_PWM_MASTER2_END */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "or Y, #0x0004 \n\t"  /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_9)
        "or Y, #0x0200 \n\t"  /* SDA - IO[9] */
#endif
        "p_I2C_MRD_60_%=: \n\t"
        "lod X, 0x0054 \n\t"  /* IO_MLX16_ITC_PEND2 */
        "and X, #0x0010 \n\t" /* B_MLX16_ITC_PEND2_PWM_MASTER2_CMP */
        "je  p_I2C_MRD_60_%= \n\t"
#elif (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)
        "lod X, 0x0052 \n\t"  /* IO_MLX16_ITC_PEND1 */
        "and X, #0x0200 \n\t" /* B_MLX16_ITC_PEND1_CTIMER1_1 */
        "je  p_I2C_MRD_40_%= \n\t"
#if defined (__MLX81340__)
        "mov 0x0206, Y \n\t"  /* IO_PORT_IO_OUT_SOFT */
#elif defined (__MLX81344__) || defined (__MLX81346__)
        "mov 0x0208, Y \n\t"  /* IO_PORT_IO_OUT_SOFT */
#endif
        "mov 0x0052, X \n\t"  /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_1 */
        "p_I2C_MRD_50_%=: \n\t"
        "lod X, 0x0052 \n\t"  /* IO_MLX16_ITC_PEND1 */
        "and X, #0x0800 \n\t" /* B_MLX16_ITC_PEND1_CTIMER1_3 */
        "je  p_I2C_MRD_50_%= \n\t"
        "mov 0x0052, X \n\t"  /* IO_MLX16_ITC_PEND1, Clear B_MLX16_ITC_PEND1_CTIMER1_3 */
#if (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_2)
        "or Y, #0x0004 \n\t"  /* SDA - IO[2] */
#elif (C_I2C_MASTER_SDA_IO == PIN_FUNC_IO_9)
        "or Y, #0x0200 \n\t"  /* SDA - IO[9] */
#endif
        "p_I2C_MRD_60_%=: \n\t"
        "lod X, 0x0052 \n\t"  /* IO_MLX16_ITC_PEND2 */
        "and X, #0x0200 \n\t" /* B_MLX16_ITC_PEND1_CTIMER1_1 */
        "je  p_I2C_MRD_60_%= \n\t"
#endif
#if defined (__MLX81340__)
        "mov 0x0206, Y \n\t"  /* IO_PORT_IO_OUT_SOFT */
#elif defined (__MLX81344__) || defined (__MLX81346__)
        "mov 0x0208, Y \n\t"  /* IO_PORT_IO_OUT_SOFT */
#endif
        "swap A"
        : "=a" (u16Result), "=x" (u16Rubbish), "=y" (u16Rubbish)
        : "a" (u8Ack)
    );

    return (u16Result);
} /* End of p_I2C_MasterReceiveData() */
#endif /* defined (__MLX81340__) || defined (__MLX81344__) || defined (__MLX81346__) */
#endif /* _SUPPORT_I2C && ((_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_PWM_MASTER2) || (_SUPPORT_I2C_MASTER_HW_RESOURCE == C_I2C_MASTER_CTIMER1)) */

#endif /* DRIVE_LIB_PRIVATE_MATHLIB_H_ */
