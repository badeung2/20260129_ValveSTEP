/**
 * @file
 * @brief The UDS Loader
 * @internal
 *
 * @copyright (C) 2017 Melexis N.V.
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
 * @ingroup uds_bootloader
 * @addtogroup uds_bootloader UDS Bootloader
 *
 * @brief The UDS Loader
 * @details
 * @{
 */


#ifndef UDS_BOOTLOADER_H_
#define UDS_BOOTLOADER_H_

#include <stdint.h>
#include <stdbool.h>
#include "mls_api.h" /* needed by mls_diag_transfer.h include */
#include "mls_diag_transfer.h"
#include "uds.h"

/** Initialize UDS on LIN and flash state machine.
 *
 * Confirm that the FLASH part (services and custom functions) of the UDS bootloader is present and valid before
 * initializing the Flash state machine, subscribe to LIN library callbacks, initialize UDS library and initialization
 * of the stimer (used for generation of the random value).
 */
void BlUds_Init(void);

/** Provide tick for Flash and UDS library state machine.
 *
 * UDS bootloader uses Flash and UDS ticks to refresh state both state machines regularly. It should be called with each
 * tick, outside of the interrupts and it is not re-entrant.
 */
void BlUds_Tick(void);

/*
 * Keep call-able version of these functions
 */

/** LIN Transport layer Request handler for UDS bootloader.
 *
 * Handles the incoming LIN message by confirming the supported SId before forwarding it to UDS library for the
 * processing. Received messages signal UDS library about incoming message as well as messages that are to be sent  are
 * copied from UDS library to LIN buffers. It also resets the watchdog and.
 *
 * @param  transfer  pointer to the transport layer's receive and response buffers
 * @retval true when answer to message is expected
 * @retval false when answer to message is not expected
 */
bool BlUds_Request(LINDiagTransfer_t* transfer);

/** LIN Transport layer Response transmitted callback.
 *
 * When LIN message is successfully transmitted the callback signals UDS library that message was successfully sent.
 *
 * @param  transfer  pointer to the transport layer's receive and response buffers
 * @retval false To enable next handlers in the list to handle the message as well.
 */
bool BlUds_ResponseTransmitted(LINDiagTransfer_t* transfer);

/** Check if the UDS bootloader is enabled by the flash application
 *
 * This function will check if there is a valid flash_uds section in the flash memory.
 * @retval true The UDS bootloader is enabled by flash.
 * @retval false Otherwise
 */
bool BlUds_UdsBootloaderEnabled(void);

/** Get the LIN NAD
 *
 * Function shall return the current LIN NAD which shall be used for the communication.
 * The LIN NAD shall be stored in application specified non volatile memory so that it can be
 * recovered from there after a power cycle.
 *
 * @param[in]  NAD  value as received in the LIN message by the TL, might be usefull to check initial vs configured NAD.
 * @returns  current LIN NAD for this device.
 * @todo move the function to services section as it/should be is application specific
 */
uint8_t BlUds_GetLINNAD(uint8_t NAD);

/** Check received challenge value towards BMW specified algorithm.
 *
 * Checks if the calculated xor random value with uds_key matches the received challenge.
 *
 * @param[in] received_challenge Challenge sent by the UDS client that needs to be checked for validity
 * @param[in] random_value Random value to use in calculation
 *
 * @retval True when received challenge matches calculated challenge
 * @retval False when received challenge does not match calculated challenge
 */
UDS_BMW_CUSTOM_MEMORY_SECTION bool UDS_BMW_CheckChallenge(uint32_t received_challenge, uint32_t random_value);

/** BMW specific: UDS Memory CRC Check routine.
 *
 * @param  memory_identifier  identifier of the memory to be crc checked.
 * @param  assumed_crc  crc value received from the UDS client.
 * @retval  true  crc fitted the calculated one.
 * @retval  false  otherwise.
 */
UDS_BMW_CUSTOM_MEMORY_SECTION uint8_t UDS_BMW_MemoryCrcCheck(uint8_t memory_identifier, uint32_t assumed_crc);

/** Check received challenge value towards VW specified algorithm with the script below.
 *
 * 0x87, 0x3A, 0x45, 0x90, 0xC8, 0x68, 0x07, 0x81, 0x4A, 0x0A, 0x84, 0x3A, 0x45, 0x90, 0xC8,
 * 0x87, 0x2B, 0xAD, 0xCA, 0xFE, 0x93, 0x2B, 0xAD, 0xCA, 0xFE, 0x49, 0x4C
 *
 * @param[in] received_challenge Seed sent by the UDS client that needs to be checked for validity
 * @param[in] random_value Random value to use in calculation
 *
 * @retval True when received challenge matches calculated challenge
 * @retval False when received challenge does not match calculated challenge
 */
UDS_VW_CUSTOM_MEMORY_SECTION bool UDS_VW_CheckChallenge(uint32_t received_challenge, uint32_t random_value);

/** VW specific: UDS Memory CRC Check routine.
 *
 * @param  memory_identifier  identifier of the memory to be crc checked.
 * @param  assumed_crc  crc value received from the UDS client.
 * @retval  true  crc fitted the calculated one.
 * @retval  false  otherwise.
 */
UDS_VW_CUSTOM_MEMORY_SECTION uint8_t UDS_VW_MemoryCrcCheck(uint8_t memory_identifier, uint32_t assumed_crc);

/** Check received challenge value towards MLX specified algorithm with the script below.
 *
 * 0x87, 0x83, 0x5C, 0xA1, 0x79, 0x68, 0x0A, 0x81, 0x4A, 0x0A, 0x84, 0x83, 0x5C, 0xA1, 0x79,
 * 0x87, 0x2B, 0xAD, 0xCA, 0xFE, 0x93, 0x2B, 0xAD, 0xCA, 0xFE, 0x49, 0x4C
 *
 * @param[in] received_challenge Seed sent by the UDS client that needs to be checked for validity
 * @param[in] random_value Random value to use in calculation
 *
 * @retval True when received challenge matches calculated challenge
 * @retval False when received challenge does not match calculated challenge
 */
__attribute__((section(".uds_mlx_custom_functions"))) bool UDS_MLX_CheckChallenge(uint32_t received_challenge,
                                                                                  uint32_t random_value);

/** MLX specific: UDS Memory CRC Check routine.
 *
 * @param  memory_identifier  identifier of the memory to be crc checked.
 * @param  assumed_crc  crc value received from the UDS client.
 * @retval  true  crc fitted the calculated one.
 * @retval  false  otherwise.
 */
__attribute__((section(".uds_mlx_custom_functions"))) uint8_t UDS_MLX_MemoryCrcCheck(uint8_t memory_identifier,
                                                                                     uint32_t assumed_crc);

#endif /* UDS_BOOTLOADER_H_ */
/// @}
