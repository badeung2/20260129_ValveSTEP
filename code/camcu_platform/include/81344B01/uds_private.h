/**
 * @file
 * @brief Server implementation of UDS protocol
 * @internal
 *
 * @copyright (C) 2019 Melexis N.V.
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
 * @defgroup lib_uds UDS Library
 * @brief UDS Library doxygen documentations
 *
 * @details UDS protocol, based on ISO14229 specification is used to communicate between sensor (device, server) and
 * central processing unit. Central processing unit sends various messages and requests for general functionality and/or
 * diagnostic status of the sensor (device). This implementation is focused on server side (sensor) which listens for
 * requests and replies to them appropriately.
 *
 * @addtogroup lib_uds_private
 * @brief Enable unit testing of static functions inside the library
 * @ingroup lib_uds
 * @{
 *
 *
 */

#ifndef UDS_PRIVATE_H
#define UDS_PRIVATE_H

#include <stdint.h>
#include <stdbool.h>
#include "uds.h"
#include "uds_headers.h"
#include "compiler_abstraction.h"

uint16_t udsSIDGetArrayIndex(const UdsSupportedService_t *services, UdsRequest_t current_request);
void udsPositiveResponseDiagnosticSessionControl(UdsConfig_t *config);
void udsSetSession(UdsConfig_t *config, uint8_t desired_session);
void udsSetExecuteAfterStateChange(UdsConfig_t *config, void (*function_ptr)(void));
bool udsMemoryWriteableCheck(UdsConfig_t *config);

#ifdef UNITTEST
void udsPreparePositiveResponseAfterReset(UdsConfig_t *config);
#endif


#endif /* UDS_PRIVATE_H */
/** @} */
