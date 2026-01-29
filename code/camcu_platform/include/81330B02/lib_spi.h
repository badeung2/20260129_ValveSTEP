/**
 * @file
 * @brief SPI support library functions
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
 * @ingroup lib_spi
 * @addtogroup lib_spi SPI
 *
 * @brief SPI support library functions
 * @details
 * @{
 */

#ifndef LIB_SPI_H
#define LIB_SPI_H

#include <stdint.h>
#include "compiler_abstraction.h"
#include "io.h"

/** Specifies the Master/Slave SPI mode */
typedef enum {
    SPI_SLAVEMODE = 0u, /**< SPI Slave mode */
    SPI_MASTERMODE,     /**< SPI Master mode */
} SpiMaster_t;

/** Specifies the SPI SS pin whether it will be forced always to zero in master mode, or during the words-transfer only. */
typedef enum {
    SPI_SS_WORD = 0u,     /**< SS pin is triggered after each word */
    SPI_SS_LOW,         /**< SS is always low */
} SpiSS_t;

/** Specifies 16/8 bit in one SPI data block */
typedef enum {
    SPI_16Bit = 0u,       /**< 16 bit word size */
    SPI_8Bit,           /**< 8 bit word size */
} SpiByteMode_t;

/** Specifies the clock polarity */
typedef enum {
    SPI_CPOL_WAIT0 = 0u,  /**< SPI lock wait state at 0 */
    SPI_CPOL_WAIT1,     /**< SPI Clock wait state at 1  */
} SpiCPOL_t;

/** Specifies the clock signal phase */
typedef enum {
    SPI_CPHA_RISE = 0u,   /**< SPI get data on the leading edge, set data on the trailing edge */
    SPI_CPHA_FALL,      /**< SPI Set data on the leading edge, get data on the trailing edge */
} SpiCPHA_t;

/** Describes the SPI flags and controls */
typedef union {
    struct __attribute__((packed)) {
        uint8_t START           : 1;  /**< Set 1 to Start the SPI. 0: does not do anything. All other ctrl bits discarded. */
        uint8_t STOP            : 1;  /**< Set 1 to Stop the SPI. 0: does not do anything. All other ctrl bits discarded. */
        SpiMaster_t MASTER      : 1;  /**< 0: Slave mode; 1: Master mode */
        SpiSS_t SS_FORCE        : 1;  /**< SS for master-mode, 0: SS=1 between words, SS=0 while transmitting. 1: SS is always 0 */
        SpiByteMode_t BYTE_MODE : 1;  /**< Word size: 0: 16bits, 1: 8bits */
        SpiCPOL_t CPOL          : 1;  /**< Clock polarity: 0: Clock wait-state at 0; 1: Clock wait-state at 1 */
        SpiCPHA_t CPHA          : 1;  /**< Clock phase: 0: Get data on leading edge set data in the trailing edge.  */
        uint8_t DMA             : 1;  /**< Enable DMA */
        uint8_t TX_EN           : 1;  /**< Enable errors due to transmission */
        uint8_t RX_EN           : 1;  /**< Enable errors due to reception */
        uint8_t                 : 2;  /**< reserved, unused area*/
        uint8_t TE              : 1;  /**< SPI Transmit register empty */
        uint8_t RF              : 1;  /**< SPI Receive register full */
        uint8_t OVRF            : 1;  /**< Overflow Error flag */
        uint8_t MODF            : 1;  /**< Mode fault Error flag */
    } ctrl;
    uint16_t data;                    /**< The common 16-bit access value to set all parameters at once */
} SpiControl_t;


/** Describes the SPI DMA settings */
typedef struct {
    uint16_t* SBASE;  /**< Base transmit buffer's address. Should be 16bit-aligned for 16-bit mode. */
    uint16_t* RBASE;  /**< Base receive buffer's address. Should be 16bit-aligned for 16-bit mode.  */
    uint16_t MSGLEN;  /**< Number of words [0..63] to be exchanged via DMA. 0 means 64 'words' to send/receive. */
} SpiDmaCfg_t;

/** SPI CLK Deviders settings */
typedef union {
    struct __attribute__((packed)) {
        uint16_t PSCLN : 4;     /**< PSCLN devides the FPLL on 2^PSCLN. PSCLN should be [0..11] */
        uint16_t PSCLM : 4;     /**< PSCLM devides the FPLL on (PSCLM+1). PSCLM should be [0..15] */
    } divs;
    uint16_t data;              /**< common setting for the SPI CLK dividers */
} SpiDividers_t;

#ifndef IO_SPI__DR_GET
#warning The SPI is not available while the SPI library is included.
#endif /* SPI IO PORTS check */

/** SpiSendData sends the data over the SPI interface
 * @param[in] data equals to the data to send to
 */
STATIC INLINE void SpiSendData(uint16_t data);

/** SpiReceiveData gets the data received over the SPI interface
 * @return DR-register value. Which represents the data received */
STATIC INLINE uint16_t SpiReceiveData(void);

/** SpiStart starts the SPI interface */
STATIC INLINE void SpiStart(void);

/** SpiStop stops the SPI interface */
STATIC INLINE void SpiStop(void);

/** SpiInit configures the SPI interface according the mode selected.
 * @param[in] init is a configuration set of parameters
 *
 * @note START and STOP flags are not used (resetted) for initialization
 */
STATIC INLINE void SpiInit(SpiControl_t init);

/** SpiGetState returns the current SPI state register */
STATIC INLINE SpiControl_t SpiGetState(void);

/** SpiSetDeviders Sets the SPI baudrate in Master-mode. Use FPLL definition to get the MCU's default clock.
 * @param[in] dividers holds a set of divider paramters used to configure the SPI clock frequency
 */
STATIC INLINE void SpiSetDividers(SpiDividers_t dividers);

/** SpiInitDma sets the SPI DMA configuration.
 * @param[in] *spi_cfg pointer to the DMA's configuration
 *
 * @note function should be used together with the #SpiInit() function.
 */
STATIC INLINE void SpiInitDma(SpiDmaCfg_t* spi_cfg);

/** SpiTeIntEnable enables the SPI TX-empty Interrupt
 * @note When the SPI is stopped (by setting the STOP bit in the Control port), while transmit register is not empty,
 * the SPI_TE_IT will be generated even if the SPI is stopped.
 */
STATIC INLINE void SpiTeIntEnable(void);

/** SpiRxIntEnable enables the SPI Data Recevied Interrupt */
STATIC INLINE void SpiRxIntEnable(void);

/** SpiErrIntEnable enables the SPI Error (overflow) Interrupt */
STATIC INLINE void SpiErrIntEnable(void);

/** SpiTeIntDisable disables the SPI TX-empty Interrupt */
STATIC INLINE void SpiTeIntDisable(void);

/** SpiRxIntDisable disables the SPI Data Recevied Interrupt */
STATIC INLINE void SpiRxIntDisable(void);

/** SpiErrIntDisable disables the SPI Error (overflow) Interrupt */
STATIC INLINE void SpiErrIntDisable(void);

#if !defined(UNITTEST) /* for unit test, mocks will be generated */
#include "lib_spi_inline_impl.h"
#endif

#endif /* LIB_SPI_H */
/// @}
