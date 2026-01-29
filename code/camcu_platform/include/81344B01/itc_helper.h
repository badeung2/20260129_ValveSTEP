/**
 * @file
 * @brief Provides a set of definitions to link the Interrupt names and Amalthea IO_ports
 * @internal
 *
 * @copyright (C) 2015-2017 Melexis N.V.
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
 * @ingroup library
 *
 * @brief Provides a set of definitions to link the Interrupt names and Amalthea IO_ports
 * @details
 * @{
 */

#ifndef ITC_HELPER_H_
#define ITC_HELPER_H_

#include <io.h>
#include <stdint.h>
#include "compiler_abstraction.h"

/** Set priority for an interrupt
 *
 * @param iname Name of the interrupt
 * @param val New priority for the interrupt
 */
#define  Itc_SetPrio(iname, val) IO_SET(MLX16, iname ## _PRIO, (val - 3U) & 3U)

/** Enable interrupt
 *
 * @param iname Name of the interrupt
 */
#define  Itc_Enable(iname) IO_SET(MLX16, iname ## _ITC, 1u)

/** Disable interrupt
 *
 * @param iname Name of the interrupt
 */
#define  Itc_Disable(iname) IO_SET(MLX16, iname ## _ITC, 0u)

/** Clear the pending flag for an interrupt
 *
 * @param iname Name of the interrupt
 */
#define  Itc_Clear(iname) IO_SET(MLX16, iname ## _PEND, 1u)

/** Check the pending flag of an interrupt
 *
 * @param iname Name of the interrupt
 * @retval True if the interrupt has pending flag set
 * @retval False otherwise
 */
#define  Itc_IsPending(iname) (IO_GET(MLX16, iname ## _PEND) != 0u)

/** Check if MASK-enabled pending IRQ(s) are present
 *
 * @param[in] pend Address of PEND-port
 * @param[in] mask Value of MASK-port
 * @retval 0 no pending IRQ
 * @retval >0 position of the pending IRQ according to the mask given
 *
 * @note
 * Function is intended to be called from the background routine to regularly check if all interrupts
 * are being serviced as required.
 *
 * @warning
 * The function might trigger false positives when the cpu interrupt service routine load is too high.
 *
   @code {.c}
        int main(void) {
            // ...
            while (1U) {
                // ...
                if (Itc_getPendingIrq(&IO_HOST(MLX16, ITC_PEND0), IO_HOST(MLX16, ITC_MASK0)) != 0U) {
                    // error: some interrupt is not being serviced while cpu is running in background
                }
                // ...
            }
            return 0U;
        }
   @endcode
 */
STATIC INLINE uint16_t Itc_getPendingIrq(volatile const uint16_t* pend, uint16_t mask);

/** Request a software interrupt */
STATIC INLINE void Itc_SwIntRequest (void);

#if !defined(UNITTEST) /* for unit test, mocks will be generated */
#include "itc_helper_inline_impl.h"
#endif

#endif /* ITC_HELPER_H_ */
/// @}
