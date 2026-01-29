/**
 * @file
 * @brief Memory API
 * @internal
 *
 * @copyright (C) 2018 Melexis N.V.
 * git flash d0014c23
 *
 * Melexis N.V. is supplying this code for use with Melexis N.V. processor based microcontrollers only.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
 * INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.  MELEXIS N.V. SHALL NOT IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * @endinternal
 *
 * @ingroup memory
 *
 * @brief Memory API
 * @details
 * @{
 */

#ifndef MEMORY_INLINE_IMPL_H
#define MEMORY_INLINE_IMPL_H

#if defined(__COVERITY__) || defined(__POLYSPACE__)
#include <string.h>
#endif

STATIC INLINE void memcpy_word(void* destination, const void* source, size_t size)
{
    uint16_t* adestination = __builtin_assume_aligned(destination, 2);
    uint16_t* asource = __builtin_assume_aligned(source,  2);
    uint16_t asize = (uint16_t)__builtin_mlx_assume_even(size);
#if defined(__COVERITY__) || defined(__POLYSPACE__)
    (void)memcpy(adestination, asource, asize);
#else
    (void)__builtin_memcpy(adestination, asource, asize);
#endif
}

STATIC INLINE int16_t memcmp_word(const void* ptr1, const void* ptr2, size_t size)
{
    const uint16_t* u16ptr1 = (const uint16_t*)ptr1;
    const uint16_t* u16ptr2 = (const uint16_t*)ptr2;
    uint16_t words = size / 2u;
    int16_t returnValue = 0;
    while (words-- > 0u) {
        u16ptr1++;
        u16ptr2++;
        if (u16ptr1[-1] != u16ptr2[-1]) {
            returnValue = (u16ptr1[-1] < u16ptr2[-1]) ? -1 : 1;
            break;
        }
    }
    return returnValue;
}

STATIC INLINE uint8_t memoryReadByte(uint32_t dstFarPointer)
{
    uint8_t val = 0u;
    uint32_t farPointer = dstFarPointer;

    if( farPointer <= MEMORY_MAX_ADDRESS) {
#ifdef HAS_MEM_GT_64KB
#if defined(__COVERITY__) || defined(__POLYSPACE__)
        const uint8_t* pData = (uint8_t*)farPointer;
        val = *pData;
#else
        /* we need mlx16 data extension to read >64k */
        farPointer = (farPointer >> 16) | (farPointer << 16); /* swap YA. i.e. Y := addrr, A := Page */
        asm volatile ( "lsl		A, #2\n\t"
                       "lsl		A, #2\n\t" /* move Page in upper half byte*/
                       "mov		R, AL\n\t" /* set Page */
                       "mov		%[ret],[Y]" /* get the value */
                       :[ret] "=a" (val) : "b" (farPointer) : "M");
#endif /* __COVERITY__ || __POLYSPACE */
#else
        const uint8_t* pData = (uint8_t*)((uint16_t)farPointer);
        val = *pData;
#endif
    }
    return val;
}

STATIC INLINE void memoryWriteByte(uint32_t dstFarPointer, uint8_t value)
{
    uint32_t farPointer = dstFarPointer;
    if( farPointer <= MEMORY_MAX_ADDRESS) {
#ifdef HAS_MEM_GT_64KB
#if defined(__COVERITY__) || defined(__POLYSPACE__)
        uint8_t* pData = (uint8_t*)farPointer;
        *pData = value;
#else
        /* we need mlx16 data extension to write >64k */
        farPointer = (farPointer >> 16) | (farPointer << 16); /* swap YA. i.e. Y := addrr, A := Page */
        asm volatile ( "lsl		A, #2\n\t"
                       "lsl		A, #2\n\t" /* move Page in upper half byte*/
                       "mov		R, AL\n\t" /* set Page */
                       /* Now the page in AL is not needed anymore, re-use AL to read the 'value' byte from the stack.
                        * Reading relatively from the stack pointer and writing to the pointer location using indirect
                        * addressing is impossible (limitation of the 'mov' instruction). Therefore, we use AL as an
                        * intermediate register. */
                       "mov     AL, %[val]\n\t"
                       "mov     [Y], AL" /* set the value */
                       :
                       :[val] "m" (value), "b" (farPointer) : "M");
#endif /* __COVERITY__ || __POLYSPACE */
#else
        uint8_t* pData = (uint8_t*)((uint16_t)farPointer);
        *pData = value;
#endif
    }
}

STATIC INLINE uint16_t memoryReadWord(uint32_t dstFarPointer)
{
    uint16_t val = 0u;
    uint32_t farPointer = dstFarPointer;

    if( farPointer < MEMORY_MAX_ADDRESS) {
#ifdef HAS_MEM_GT_64KB
#if defined(__COVERITY__) || defined(__POLYSPACE__)
        const uint16_t* pData = (uint16_t*)farPointer;
        val = *pData;
#else
        /* we need mlx16 data extension to read >64k */
        farPointer = (farPointer >> 16) | (farPointer << 16); /* swap YA. i.e. Y := addrr, A := Page */
        asm volatile ( "lsl		A, #2\n\t"
                       "lsl		A, #2\n\t" /* move Page in upper half byte*/
                       "mov		R, AL\n\t" /* set Page */
                       "mov		%[ret],[Y]" /* get the value */
                       :[ret] "=a" (val) : "b" (farPointer) : "M");
#endif /* __COVERITY__ || __POLYSPACE */
#else
        const uint16_t* pData = (uint16_t*)((uint16_t)farPointer);
        val = *pData;
#endif
    }
    return val;
}

STATIC INLINE void memoryWriteWord(uint32_t dstFarPointer, uint16_t value)
{
    uint32_t farPointer = dstFarPointer;
    if( farPointer < MEMORY_MAX_ADDRESS) {
#ifdef HAS_MEM_GT_64KB
#if defined(__COVERITY__) || defined(__POLYSPACE__)
        uint16_t* pData = (uint16_t*)farPointer;
        *pData = value;
#else
        /* we need mlx16 data extension to write >64k */
        farPointer = (farPointer >> 16) | (farPointer << 16); /* swap YA. i.e. Y := addrr, A := Page */
        asm volatile ( "lsl		A, #2\n\t"
                       "lsl		A, #2\n\t" /* move Page in upper half byte*/
                       "mov		R, AL\n\t" /* set Page */
                       /* Moving 'value' to the pointed memory location needs 2 MOV instructions. Same comments
                        * as in memoryWriteByte. */
                       "mov     A, %[val]\n\t"
                       "mov     [Y], A" /* set the value */
                       :
                       :[val] "m" (value), "b" (farPointer) : "M");
#endif /* __COVERITY__ || __POLYSPACE */
#else
        uint16_t* pData = (uint16_t*)((uint16_t)farPointer);
        *pData = value;
#endif
    }
}

STATIC INLINE uint32_t memoryReadLong(uint32_t dstFarPointer)
{
    uint32_t val = 0u;
    uint32_t farPointer = dstFarPointer;

    if( farPointer < MEMORY_MAX_ADDRESS) {
#ifdef HAS_MEM_GT_64KB
#if defined(__COVERITY__) || defined(__POLYSPACE__)
        const uint32_t* pData = (uint32_t*)farPointer;
        val = *pData;
#else
        farPointer = (farPointer >> 16) | (farPointer << 16); /* swap YA. i.e. Y := addrr, A := Page */
        asm volatile ( "lsl     A, #2\n\t"
                       "lsl     A, #2\n\t" /* move Page in upper half byte*/
                       "mov     R, AL\n\t" /* set Page */
                       "mov     %[ret],[Y]" /* get the value */
                       :[ret] "=q" (val) : "b" (farPointer) : "M");
#endif /* __COVERITY__ || __POLYSPACE__ */
#else
        const uint32_t* pData = (uint32_t*)((uint16_t)farPointer);
        val = *pData;
#endif
    }
    return val;
}

STATIC INLINE void memoryWriteLong(uint32_t dstFarPointer, uint32_t value)
{
    uint32_t farPointer = dstFarPointer;
    if( farPointer < MEMORY_MAX_ADDRESS) {
#ifdef HAS_MEM_GT_64KB
#if defined(__COVERITY__) || defined(__POLYSPACE__)
        uint32_t* pData = (uint32_t*)farPointer;
        *pData = value;
#else
        farPointer = (farPointer >> 16) | (farPointer << 16); /* swap YA. i.e. Y := addrr, A := Page */
        asm volatile ( "lsl     A, #2\n\t"
                       "lsl     A, #2\n\t" /* move Page in upper half byte*/
                       "mov     R, AL\n\t" /* set Page */
                       "mov     [Y], D" /* set the value */
                       :
                       :[val] "q" (value), "b" (farPointer) : "M");
#endif /* __COVERITY__ || __POLYSPACE__ */
#else
        uint32_t* pData = (uint32_t*)((uint16_t)farPointer);
        *pData = value;
#endif
    }
}

#endif
/// @}

