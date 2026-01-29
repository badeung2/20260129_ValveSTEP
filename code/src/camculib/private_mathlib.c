/*!*************************************************************************** *
 * \file        private_mathlib.c
 * \brief       private math library tables
 *
 * \note        project MLX813xx
 *
 * \author      Marcel Braat
 *
 * \date        2022-01-07
 *
 * \version     2.0
 *
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2022-2022 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 * *************************************************************************** */

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Build */

#if (_SUPPORT_ROM_FAST_MATH == FALSE) && _SUPPORT_FAST_ATAN2
#ifndef LOOKUPTABLESIZE
#define LOOKUPTABLESIZE 256u                                                    /*!< ArcTanget look-up table size (precision) */
#endif /* ! LOOKUPTABLESIZE */

#if LOOKUPTABLESIZE == 256
#if (_SUPPORT_FAST_ATAN2_LOOKUP != FALSE)
#define LUT_VAL8(v1, v2, v3, v4, v5, v6, v7, v8) (v1 + 2) / 4, (v2 + 2) / 4, (v3 + 2) / 4, \
    (v4 + 2) / 4, (v5 + 2) / 4, (v6 + 2) / 4, (v7 + 2) / 4, (v8 + 2) / 4        /*!< ArcTanget look-up table macro */
#define LUT_FINAL_VAL8(v1, v2, v3, v4, v5, v6, v7, v8) (v1 + 2) / 4             /*!< ArcTanget table macro */
#else  /* (_SUPPORT_FAST_ATAN2_LOOKUP != FALSE) */
#define LUT_VAL8(v1, v2, v3, v4, v5, v6, v7, v8) v1 / 4, v2 / 4, v3 / 4, v4 / 4, v5 / 4, v6 / 4, v7 / 4, v8 / 4 /*!< ArcTanget look-up table macro */
#define LUT_FINAL_VAL8(v1, v2, v3, v4, v5, v6, v7, v8) v1                       /*!< ArcTanget table macro */
#endif /* (_SUPPORT_FAST_ATAN2_LOOKUP != FALSE) */
#elif LOOKUPTABLESIZE == 128
#define LUT_VAL8(v1, v2, v3, v4, v5, v6, v7, v8) v1,     v3,     v5,     v7
#define LUT_FINAL_VAL8(v1, v2, v3, v4, v5, v6, v7, v8) v2
#elif LOOKUPTABLESIZE == 64
#define LUT_VAL8(v1, v2, v3, v4, v5, v6, v7, v8) v1,             v5
#define LUT_FINAL_VAL8(v1, v2, v3, v4, v5, v6, v7, v8) v4
#elif LOOKUPTABLESIZE == 32
#define LUT_VAL8(v1, v2, v3, v4, v5, v6, v7, v8) v1
#define LUT_FINAL_VAL8(v1, v2, v3, v4, v5, v6, v7, v8) v8
#else
#error "Supported LOOKUPTABLESIZE values 32, 64, 128, 256"
#define LUT_VAL8(v1, v2, v3, v4, v5, v6, v7, v8) 0
#define LUT_FINAL_VAL8(v1, v2, v3, v4, v5, v6, v7, v8) 0
#endif

/*! ArcTangen look-up table */
/* one "octant", i.e. for input values 0..1 (0..65536) */
/* S16, to be updated for U16 */
const uint16_t p_ATAN_LUT[ LOOKUPTABLESIZE + 2 ] =
{
    LUT_VAL8(0,    163,    326,    489,    652,    815,    978,   1141),
    LUT_VAL8(1303,   1466,   1629,   1792,   1954,   2117,   2279,   2442),
    LUT_VAL8(2604,   2767,   2929,   3091,   3253,   3415,   3577,   3738),
    LUT_VAL8(3900,   4061,   4223,   4384,   4545,   4706,   4867,   5028),
    LUT_VAL8(5188,   5349,   5509,   5669,   5829,   5989,   6148,   6308),
    LUT_VAL8(6467,   6626,   6784,   6943,   7101,   7260,   7418,   7575),
    LUT_VAL8(7733,   7890,   8047,   8204,   8361,   8517,   8673,   8829),
    LUT_VAL8(8985,   9140,   9296,   9450,   9605,   9759,   9914,  10067),
    LUT_VAL8(10221,  10374,  10527,  10680,  10832,  10984,  11136,  11287),
    LUT_VAL8(11439,  11590,  11740,  11890,  12040,  12190,  12339,  12488),
    LUT_VAL8(12637,  12785,  12933,  13081,  13228,  13375,  13522,  13668),
    LUT_VAL8(13814,  13959,  14105,  14249,  14394,  14538,  14682,  14825),
    LUT_VAL8(14968,  15111,  15253,  15395,  15537,  15678,  15819,  15960),
    LUT_VAL8(16100,  16239,  16379,  16518,  16656,  16794,  16932,  17069),
    LUT_VAL8(17206,  17343,  17479,  17615,  17750,  17885,  18020,  18154),
    LUT_VAL8(18288,  18421,  18554,  18687,  18819,  18951,  19083,  19213),
    LUT_VAL8(19344,  19474,  19604,  19733,  19862,  19991,  20119,  20247),
    LUT_VAL8(20374,  20501,  20627,  20753,  20879,  21004,  21129,  21254),
    LUT_VAL8(21378,  21501,  21624,  21747,  21870,  21992,  22113,  22234),
    LUT_VAL8(22355,  22475,  22595,  22714,  22834,  22952,  23070,  23188),
    LUT_VAL8(23306,  23423,  23539,  23655,  23771,  23886,  24001,  24116),
    LUT_VAL8(24230,  24344,  24457,  24570,  24682,  24795,  24906,  25017),
    LUT_VAL8(25128,  25239,  25349,  25459,  25568,  25677,  25785,  25893),
    LUT_VAL8(26001,  26108,  26215,  26321,  26427,  26533,  26638,  26743),
    LUT_VAL8(26848,  26952,  27056,  27159,  27262,  27364,  27467,  27568),
    LUT_VAL8(27670,  27771,  27871,  27972,  28072,  28171,  28270,  28369),
    LUT_VAL8(28467,  28565,  28663,  28760,  28857,  28953,  29050,  29145),
    LUT_VAL8(29241,  29336,  29430,  29525,  29619,  29712,  29805,  29898),
    LUT_VAL8(29991,  30083,  30175,  30266,  30357,  30448,  30538,  30628),
    LUT_VAL8(30718,  30807,  30896,  30985,  31073,  31161,  31248,  31336),
    LUT_VAL8(31423,  31509,  31595,  31681,  31767,  31852,  31937,  32022),
    LUT_VAL8(32106,  32190,  32273,  32357,  32439,  32522,  32604,  32686),
#if (_SUPPORT_FAST_ATAN2_LOOKUP != FALSE)
    32768 / 4,
#else  /* (_SUPPORT_FAST_ATAN2_LOOKUP != FALSE) */
    32768,
#endif /* (_SUPPORT_FAST_ATAN2_LOOKUP != FALSE) */
    LUT_FINAL_VAL8(32849, 32930, 33011, 33091, 33171, 33251, 33331, 33410)
};
#endif /* (_SUPPORT_ROM_FAST_MATH == FALSE) && _SUPPORT_FAST_ATAN2 */

#if FALSE /* Check performance */
const uint8_t tNibbleRev[16] = {0x00, 0x08, 0x04, 0x0C, 0x02, 0x0A, 0x06, 0x0E,
                                0x01, 0x09, 0x05, 0x0D, 0x03, 0x0B, 0x07, 0x0F};
#endif /* FALSE */

/*!*************************************************************************** *
 * ConvBase8Exp3toU16
 * \brief   Convert 8-bit Base with 3-bit Exponent to uint16_t
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u8Base: Base (8-bit)
 * \param   [in] u3Exp: Exponent (3-bit)
 * \return  (uint16_t) u16Value = ((256 + u8Base) << u3Exp) - 256
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ConvBase8Exp3toU16(uint8_t u8Base, uint8_t u3Exp)
{
    return ( ((256U + (uint16_t)u8Base) << (u3Exp & 0x07)) - 256U);
} /* End of ConvBase8Exp3toU16() */

/*!*************************************************************************** *
 * ConvU16toBase8Exp3
 * \brief   Convert 16-bit value into a 8-bit Base with 3-bit Exponent
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u16Value: 16-bit value
 * \return  (uint16_t) u16Value = (u3Exp << 8) | u8Base
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 1+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ConvU16toBase8Exp3(uint16_t u16Value)
{
    uint16_t u16Exp = 0U;
    while (u16Value >= 256U)
    {
        u16Exp++;
        u16Value = ((u16Value - 256U) + 1U) >> 1U;
    }
    return ((u16Exp << 8) | u16Value);
} /* End of ConvU16toBase8Exp3() */

#if (_SUPPORT_PID_U32 != FALSE)
/*!*************************************************************************** *
 * ConvBase14Exp2toU32
 * \brief   Convert 14-bit Base with 2-bit Exponent to uint32_t
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u14Base: Base (14-bit)
 * \param   [in] u2Exp: Exponent (2-bit)
 * \return  (uint16_t) u16Value = ((256 + u8Base) << u3Exp) - 256
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 0+1
 * - Nesting: 0
 * - Function calling: 0
 * *************************************************************************** */
uint32_t ConvBase14Exp2toU32(uint16_t u14Base, uint8_t u2Exp)
{
    return ( ((16384UL + (uint32_t)u14Base) << (u2Exp & 0x03)) - 16384UL);
} /* End of ConvBase12Exp2toU32() */

/*!*************************************************************************** *
 * ConvU32toBase14Exp2
 * \brief   Convert 32-bit value into a 14-bit Base with 2-bit Exponent
 * \author  mmp
 * *************************************************************************** *
 * \param   [in] u32Value: 32-bit value
 * \return  (uint16_t) u16Value = (u3Exp << 8) | u8Base
 * *************************************************************************** *
 * \details -
 * *************************************************************************** *
 * - Call Hierarchy: -
 * - Cyclomatic Complexity: 2+1
 * - Nesting: 1
 * - Function calling: 0
 * *************************************************************************** */
uint16_t ConvU32toBase14Exp2(uint32_t u32Value)
{
    uint16_t u16Exp = 0U;
    while (u32Value >= 16384UL)
    {
        u16Exp++;
        u32Value = ((u32Value - 16384UL) + 1U) >> 1U;
    }
    if (u16Exp > 3U)
    {
        /* Truncate to maximum */
        u16Exp = 3U;
        u32Value = 16383UL;
    }
    return ((u16Exp << 14) | (uint16_t)u32Value);
} /* End of ConvU32toBase14Exp2() */
#endif /* (_SUPPORT_PID_U32 == FALSE) */

/* EOF */
