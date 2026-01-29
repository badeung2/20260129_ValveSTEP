/**
 * @file
 * @brief SPI support library functions
 * @internal
 *
 * @copyright (C) 2020 Melexis N.V.
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
 * @ingroup lib_spi
 *
 * @brief SPI support library functions
 * @details
 * @{
 */

#ifndef LIB_SPI_INLINE_IMPL_H
#define LIB_SPI_INLINE_IMPL_H

#include <stdint.h>
#include "compiler_abstraction.h"
#include "io.h"

#ifdef UNITTEST
#include "lib_spi.h"
#endif

STATIC INLINE void SpiSendData(uint16_t data)
{
    IO_SET(SPI, DR, data);
}

STATIC INLINE uint16_t SpiReceiveData(void)
{
    return IO_GET(SPI, DR);
}

STATIC INLINE void SpiStart(void)
{
    IO_HOST(SPI, START) = IO_MASK(SPI, START);
}

STATIC INLINE void SpiStop(void)
{
    IO_HOST(SPI, STOP) = IO_MASK(SPI, STOP);
}

STATIC INLINE void SpiInit(SpiControl_t init)
{
    /* Avoid triggering the START and STOP commands since they block the rest bits inside. */
    init.data &= ~((1u << IO_OFFSET(SPI, STOP)) | (1u << IO_OFFSET(SPI, START)));
    IO_HOST(SPI, START) = init.data;
    IO_SET( SPI,
            MODF, 1u,
            OVRF, 1u ); /* Clean the flags */
}

STATIC INLINE SpiControl_t SpiGetState(void)
{
    SpiControl_t tmp;
    tmp.data = IO_HOST(SPI, START);
    return tmp;
}

STATIC INLINE void SpiSetDividers( SpiDividers_t dividers )
{
    IO_HOST(SPI, PSCLN) = dividers.data;
}

STATIC INLINE void SpiInitDma(SpiDmaCfg_t* spi_cfg)
{
    IO_SET(SPI, SBASE, (uint16_t)spi_cfg->SBASE);
    IO_SET(SPI, RBASE, (uint16_t)spi_cfg->RBASE);
    IO_SET(SPI, MSGLEN, spi_cfg->MSGLEN);
}

STATIC INLINE void SpiTeIntEnable(void)
{
    IO_SET(MLX16, SPI_TE_PEND, 1u);
    IO_SET(MLX16, SPI_TE_ITC, 1u);
}

STATIC INLINE void SpiRxIntEnable(void)
{
    IO_SET(MLX16, SPI_RF_PEND, 1u);
    IO_SET(MLX16, SPI_RF_ITC, 1u);
}

STATIC INLINE void SpiErrIntEnable(void)
{
    IO_SET(MLX16, SPI_ER_PEND, 1u);
    IO_SET(MLX16, SPI_ER_ITC, 1u);
}

STATIC INLINE void SpiTeIntDisable(void)
{
    IO_SET(MLX16, SPI_TE_ITC, 0u);
}

STATIC INLINE void SpiRxIntDisable(void)
{
    IO_SET(MLX16, SPI_RF_ITC, 0u);
}

STATIC INLINE void SpiErrIntDisable(void)
{
    IO_SET(MLX16, SPI_ER_ITC, 0u);
}

#endif /* LIB_SPI_INLINE_IMPL_H */
/// @}
