/**
 * @file
 * @brief Wrapper to include Melexis LIN Module constant tables
 * @internal
 *
 * @copyright (C) 2018 Melexis N.V.
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
 * @ingroup fw_mlx_api
 *
 * @details
 */

#include "lib_miscio.h"

    .global _lin_module_const_table_start
    .global _lin_module_const_table_end

#ifdef ML_SECTION_MLX4_CONST_TABLE
    .section ML_SECTION_MLX4_CONST_TABLE,"xa",@progbits
#else
	.section .mlx4_cst_tables_ram,"xa",@progbits
#endif

_lin_module_const_table_start:

#if (MLX4_FPLL <= 12000)   /* --- 12 MHz --- */
        .incbin "cst_tables_12mhz.bin"
#elif (MLX4_FPLL <= 14000)  /* --- 14 MHz --- */
        .incbin "cst_tables_14mhz.bin"
#elif (MLX4_FPLL <= 16000)  /* --- 16 MHz --- */
        .incbin "cst_tables_16mhz.bin"
#elif (MLX4_FPLL <= 24000)  /* --- 24 MHz --- */
        .incbin "cst_tables_24mhz.bin"
#elif (MLX4_FPLL <= 28000)  /* --- 28 MHz --- */
        .incbin "cst_tables_28mhz.bin"
#elif (MLX4_FPLL <= 30000)  /* --- 30 MHz --- */
        .incbin "cst_tables_30mhz.bin"
#elif (MLX4_FPLL <= 32000)  /* --- 32 MHz --- */
        .incbin "cst_tables_32mhz.bin"
#else
#error "This CPU frequency (defined by MLX4_FPLL) is not supported by LIN Module FW"
#endif

_lin_module_const_table_end:
    end

/* EOF */
