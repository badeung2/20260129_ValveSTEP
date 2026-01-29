/*!*************************************************************************** *
 * \file        ActDefines.h
 * \brief       MLX813xx Project Actuator defines file (Used by C and S files)
 *
 * \note        project MLX813xx
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-14
 *
 * \version     2.0
 *
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
 *
 * *************************************************************************** *
 * Note: Only use "#define" statements, no "typedefs"
 * *************************************************************************** */

#ifndef _ACT_DEFINES_H_

#define _ACT_DEFINES_H_

/*!*************************************************************************** */
/*                               DEFINITIONS                                   */
/* *************************************************************************** */
/* *** Section #3: Motor type *** */
/* 0xxx = MLX81300 : Gen2 (Mulan2), 3-phase, 500mA, F:32kB, R:2kB
 *        MLX81310 : Gen2 (Mulan2), 4-phase, 500mA, F:32kB, R:2kB
 *        MLX81315 : Gen2 (Mulan2), 4-phase, 1000mA, F:32kB, R:2kB
 *        MLX81325 : Gen2 (Mulan3), 4-phase, Pre-driver, F:32kB+16kB, R:2kB
 *        MLX81330 : Gen3 (Camcu), 4-phase, 500mA, F:32kB+14kB, R:2kB
 *        MLX81332 : Gen3 (Camcu), 4-phase, 1000mA, F:32kB+16kB, R:2kB
 *        MLX81334 : Gen3 (Camcu), 4-phase, 1000mA, F:64kB+16kB, R:4kB
 *        MLX81339 : Gen4 (Camcu), 4-phase, 2000mA, F:32kB+16kB, R:2kB+3kB
 *        MLX81340 : Gen3 (Camcu), 3-phase, Pre-driver, F:32kB+26kB, R:2kB
 *        MLX81344 : Gen3 (Camcu), 3-phase, Pre-driver, F:64kB+26kB, R:4kB
 *        MLX81346 : Gen3 (Camcu), 3-phase, Pre-driver, F:64kB+26kB, R:4kB
 *        MLX81350 : Gen4 (Camcu), 4-phase, 500mA, F:32kB+16kB, R:2kB+3kB
 */
#define MT_ITW_EVENT                        0x0461U                             /*!< ITW eVent, Bipolar stepper, 50R */

#endif /* _ACT_DEFINES_H_ */

/* EOF */

