/**
 * @file
 * @brief The RAM section initialization routines.
 * @internal
 *
 * @copyright (C) 2018-2019 Melexis N.V.
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
 * @ingroup fw_user_startup
 *
 * @details This file contains the implementations of the RAM section initialization routine.
 */

#include <stdint.h>
#include "fw_startup.h"


/* ---------------------------------------------
 * Linker symbols
 * (these objects are not created in the memory)
 * --------------------------------------------- */

extern uint16_t _bss_dp_start;  /**< Linker symbol at the beginning of non-initialized RAM in DP range */
extern uint16_t _bss_dp_end;  /**< Linker symbol at the end of non-initialized RAM in DP range */

extern uint16_t _bss_start;  /**< Linker symbol at the beginning of non-initialized RAM outside DP range */
extern uint16_t _bss_end;  /**< Linker symbol at the end of non-initialized RAM outside DP range */

extern uint16_t _data_load_start;  /**< Linker symbol at the beginning of initialization memory in Flash */

extern uint16_t _data_dp_start;  /**< Linker symbol at the beginning of initialized RAM in DP range */
extern uint16_t _data_dp_end;  /**< Linker symbol at the end of initialized RAM in DP range */

extern uint16_t _data_start;  /**< Linker symbol at the beginning of initialized RAM outside DP range */
extern uint16_t _data_end;  /**< Linker symbol at the beginning of initialized RAM outside DP range */

#if defined (HAS_MLS_API)
extern uint16_t _mlx4_cst_tables_ram_start;  /**< Linker symbol at the beginning of initialized RAM with MLX4 constant tables */
extern uint16_t _mlx4_cst_tables_ram_end;  /**< Linker symbol at the end of initialized RAM with MLX4 constant tables */
extern uint16_t _mlx4_cst_tables_ram_load_start;  /**< Linker symbol at the beginning of initialization memory in Flash with MLX4 constant tables */

extern uint16_t _mlx4_flash_table_start;  /**< Linker symbol at the beginning of initialized RAM with MLX4 configuration tables */
extern uint16_t _mlx4_flash_table_end;  /**< Linker symbol at the end of initialized RAM with MLX4 configuration tables */
extern uint16_t _mlx4_flash_table_load_start;  /**< Linker symbol at the beginning of initialization memory in Flash with MLX4 configuration tables */
#endif /* HAS_MLS_API */


/* ---------------------------------------------
 * Function Definitions
 * --------------------------------------------- */

void fw_ram_section_init(void)
{
   /*
    * @note Function should NOT be static and/or inlined otherwise space
    * on stack for local variables r and w could be allocated before calling
    * Sys_SetStack() in start().
    */

    uint16_t* w;
#if !defined(RAM_APPLICATION)
    uint16_t* r;
#endif /* !RAM_APPLICATION */

    /* clear .bss and .dp.bss sections */
    for (w = &_bss_dp_start; w < &_bss_dp_end; ) {
        *w++ = 0;
    }

    for (w = &_bss_start; w < &_bss_end; ) {
        *w++ = 0;
    }

#if !defined(RAM_APPLICATION)   /* skip ROM-to-RAM loading for RAM application */
    /* initialize .data and .dp.data sections */
    r = &_data_load_start;      /* copy data from rom */

    for (w = &_data_dp_start; w < &_data_dp_end; ) {
        *w++ = *r++;
    }

    for (w = &_data_start; w < &_data_end; ) {
        *w++ = *r++;
    }
#endif /* !RAM_APPLICATION */

#if defined (HAS_MLS_API)
    r = &_mlx4_cst_tables_ram_load_start;
    for (w = &_mlx4_cst_tables_ram_start; w < &_mlx4_cst_tables_ram_end; ) {
        *w++ = *r++;
    }

    r = &_mlx4_flash_table_load_start;
    for (w = &_mlx4_flash_table_start; w < &_mlx4_flash_table_end; ) {
        *w++ = *r++;
    }
#endif /* HAS_MLS_API */
}

