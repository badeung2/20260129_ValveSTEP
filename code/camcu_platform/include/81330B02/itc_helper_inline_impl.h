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

#ifndef ITC_HELPER_INLINE_IMPL_H_
#define ITC_HELPER_INLINE_IMPL_H_

#include <stdint.h>
#include "io.h"
#include "compiler_abstraction.h"

#ifdef UNITTEST
#include "itc_helper.h"
#endif

STATIC INLINE uint16_t Itc_getPendingIrq(volatile const uint16_t* pend, uint16_t mask)
{
    uint16_t pendingIRQ = (*pend & mask);
    if ( pendingIRQ != 0U ) {
        // For a certain pend and mask position 1 should never be read at same time
        // As the IRQ may occur just before the actual read-out of the PEND-register,
        // the interrupt bit may be present at first read. Therefore a second check of
        // that PEND-bit is required to make sure the IRQ is not serviced in meanwhile.
        pendingIRQ = pendingIRQ & *pend;
    }
    return pendingIRQ;
}

STATIC INLINE void Itc_SwIntRequest (void)
{
    IO_SET(MLX16, SWI, 1u);
}

#endif /* ITC_HELPER_INLINE_IMPL_H_ */
/// @}
