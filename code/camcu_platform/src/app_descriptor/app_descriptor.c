/**
 * @file
 * @brief The application descriptor module.
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
 * @ingroup fw_app_descr
 *
 * @details This file contains the implementations of the application descriptor module.
 */

#include <stdint.h>
#include "compiler_abstraction.h"
#include "static_assert.h"

/** Product identification string */
STATIC CONST uint8_t product_id[8] __attribute__((used, section(".fw_product_id"))) = APP_PRODUCT_ID;

#ifndef APP_VERSION_LONG
/* Semantic version numbering checks */
ASSERT( APP_VERSION_MAJOR < 256 );
ASSERT( APP_VERSION_MINOR < 256 );
ASSERT( APP_VERSION_PATCH < 256 );
ASSERT( APP_VERSION_BUILD < 256 );
#endif

/** Application version number
 *
 * Application version can be set using:
 * - non semantic version numbering:
 *   - APP_VERSION_LONG = 32-bit number
 * - if previous option is not used semantic version numbering is assumed:
 *   - APP_VERSION_MAJOR = 8-bit number
 *   - APP_VERSION_MINOR = 8-bit number
 *   - APP_VERSION_PATCH = 8-bit number
 *   - APP_VERSION_BUILD = 8-bit number
 */
STATIC CONST uint32_t application_version __attribute__((used, section(".fw_app_version"))) =
#ifdef APP_VERSION_LONG
    APP_VERSION_LONG;
#else
    ( ((uint32_t)APP_VERSION_MAJOR << 0) |
      ((uint32_t)APP_VERSION_MINOR << 8) |
      ((uint32_t)APP_VERSION_PATCH << 16) |
      ((uint32_t)APP_VERSION_BUILD << 24) );
#endif

/** Flash protection key */
STATIC CONST uint64_t protection_key __attribute__((used, section(".fw_prot_key"))) = PROTECTION_KEY;

