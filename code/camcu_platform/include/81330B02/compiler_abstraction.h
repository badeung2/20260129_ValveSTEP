/**
 * @file
 * @brief Abstraction for compiler specifics
 * @internal
 *
 * @copyright (C) 2015-2016 Melexis N.V.
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
 * @addtogroup chip support package
 *
 * @details
 */

#ifndef COMPILER_ABSTRACTION_H
#define COMPILER_ABSTRACTION_H

#define STRINGIFY(a) #a

/* The macros below allow to enable/disable gcc warnings for a specific file or chunk of code
 * DISABLE_GCC_WARNING and ENABLE_GCC_WARNING enable or disable a specific warning, as shown below:
 *
 * DISABLE_GCC_WARNING("-Wbit_access")
 *
 * RESTORE_GCC_WARNING will revert the enable/disable behaviour back to the default behaviour as specified
 * when invoking gcc.
 */
#define DISABLE_GCC_WARNING(DIAG_FLAG) \
    _Pragma(STRINGIFY(GCC diagnostic push) ) \
    _Pragma(STRINGIFY(GCC diagnostic ignored DIAG_FLAG) )
#define ENABLE_GCC_WARNING() \
    _Pragma(STRINGIFY(GCC diagnostic push) ) \
    _Pragma(STRINGIFY(GCC diagnostic error DIAG_FLAG) )
#define RESTORE_GCC_WARNING() \
    _Pragma(STRINGIFY(GCC diagnostic pop) )

/* Abstraction for expressing likelyness of conditions: only has effect when optimization for speed is on (O2,O3,Ofast) */
#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)

/* Abstraction for inlining */
#ifndef INLINE
    #ifdef UNITTEST
        #define INLINE
    #else
        #define INLINE __attribute__((always_inline)) inline
    #endif
#else
    #warning "An existing definition of INLINE exists!"
#endif /* INLINE */

/* Abstraction for no-inline */
#ifndef NOINLINE
    #ifdef UNITTEST
        #define NOINLINE
    #else
        #define NOINLINE __attribute__((noinline))
    #endif
#else
    #warning "An existing definition of NOINLINE exists!"
#endif /* NOINLINE */

/* Static */
#ifndef STATIC
    #ifdef UNITTEST
        #define STATIC
    #else
        #define STATIC static
    #endif
#else
    #warning "An existing definition of STATIC exists!"
#endif

/* Const */
#ifndef CONST
    #ifdef UNITTEST
        #define CONST
    #else
        #define CONST const
    #endif
#else
    #warning "An existing definition of CONST exists!"
#endif

/* Interrupt */
#ifndef INTERRUPT
    #ifdef UNITTEST
        #define INTERRUPT
    #else
        #define INTERRUPT __attribute__((interrupt, noinline))
    #endif
#else
    #warning "An existing definition of INTERRUPT exists!"
#endif

/* Weak */
#ifndef WEAK
    #ifdef UNITTEST
        #define WEAK
    #else
        #define WEAK __attribute__((weak))
    #endif
#else
    #warning "An existing definition of WEAK exists!"
#endif

/* Weak interrupt */
#ifndef WEAK_INTERRUPT
    #ifdef UNITTEST
        #define WEAK_INTERRUPT
    #else
        #define WEAK_INTERRUPT __attribute__((weak, interrupt))
    #endif
#else
    #warning "An existing definition of WEAK_INTERRUPT exists!"
#endif

/* No return */
#ifndef NO_RETURN
    #ifdef UNITTEST
        #define NO_RETURN
    #else
        #define NO_RETURN __attribute__((noreturn))
    #endif
#else
    #warning "An existing definition of NO_RETURN exists!"
#endif

/* Packed */
#define PACKED __attribute__((packed))

/* Avoid GCC warning about parameter to function not being used */
#define UNUSED(x) ((void)(x))

/* Fallthrough */
#ifndef FALLTHROUGH
    #if defined(__GNUC__) && (__GNUC__ >= 7)
        #define FALLTHROUGH __attribute__((fallthrough));
    #else
        #define FALLTHROUGH        /* FALLTHROUGH */
    #endif
#else
    #warning "An existing definition of FALLTHROUGH exists!"
#endif


#endif /* COMPILER_ABSTRACTION_H */

