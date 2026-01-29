/*!*************************************************************************** *
 * \file        AppVersion.c
 * \brief       MLX813xx application version
 *
 * \note        project MLX813xx
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     2.0
 *
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2017-2022 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * *************************************************************************** */

/*!*************************************************************************** */
/*                           INCLUDES                                          */
/* *************************************************************************** */
#include "AppBuild.h"                                                           /* Application Includes */
#include "AppVersion.h"

#if !defined (PROTECTION_KEY)
/** Clear Flash Key */
static const uint32_t protection_key[] __attribute__((used, section(".fw_prot_key"))) = { /*lint !e528 */ /* @suppress("Unused variable declaration in file scope") */
    0x01230123u,
    0x01230123u
};
#endif /* !defined (PROTECTION_KEY) */

#if !defined (APP_VERSION_MAJOR) && !defined (APP_VERSION_MINOR)
/** Set application version */
const uint32_t application_version __attribute__((section(".fw_app_version"))) =
    (__APP_VERSION_MAJOR__) |
    (__APP_VERSION_MINOR__ << 8) |
    (__APP_VERSION_REVISION__ << 16);
#endif /* !defined (APP_VERSION_MAJOR) && !defined (APP_VERSION_MINOR) */

#if !defined (APP_PRODUCT_ID)
/** Set application string */
const uint8_t product_id[8] __attribute__((section(".fw_product_id"))) =
    "MLX_STEP";
#endif /* !defined (APP_PRODUCT_ID) */

/* EOF */
