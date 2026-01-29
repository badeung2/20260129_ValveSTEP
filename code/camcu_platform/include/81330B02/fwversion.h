/**
 * @file
 * @brief Version information for each project
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
 * @ingroup fw_version
 * @addtogroup fw_version FW Version
 *
 * @brief Version information for each project
 * @details
 * @{
 */

#ifndef FWVERSION_H
#define FWVERSION_H

#ifndef __ASSEMBLER__
#include <stdint.h>

/** Getter for the application version
 *
 * The function will return the actual version as a 32-bit number.
 *
 * @return 32-bit value containing the 4-byte version
 * @note: Define FW_APP_VERSION_ADDR in the memory_map.h file for the product to build.
 *        This approach was implemented because it is not possible to automatically
 *        fetch 20-bit address.
 */
uint32_t VERSION_getFwPltfVersion(void);

/** Getter for the platform version
 *
 * The function will return the actual version as a 32-bit number.
 *
 * @return 32-bit value containing the 4-byte version
 * @note: Define FW_PLTF_VERSION_ADDR in the memory_map.h file for the product to build.
 *        This approach was implemented because it is not possible to automatically
 *        fetch 20-bit address.
 */
uint32_t VERSION_getFwAppVersion(void);

#else /* __ASSEMBLER__ */

#endif /* __ASSEMBLER__ */

#endif /* FWVERSION_H */
/// @}

