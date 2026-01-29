/**
 * @file
 * @brief UART serial interface support library
 * @internal
 *
 * @copyright (C) 2017-2018 Melexis N.V.
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
 * @ingroup lib_uart
 * @addtogroup lib_uart UART Library
 *
 * @brief UART serial interface support library
 * @details
 * @{
 */

#ifndef LIB_UART_H
#define LIB_UART_H

#include <stdint.h>
#include "io.h"
#include "compiler_abstraction.h"

/** Bits scrambler */
typedef enum UartBscCtrl_e {
    NRZIL = 0u, /**< UART, using NRZ, LSB first [bits 15 -> 0] */
    NRZIM,      /**< UART, using NRZ, MSB first [bits 0 -> 15] */
    MPE,        /**< UART, using Manchester Phase Encoding [bits 15, 7, 14, 6 ... 8, 0] Data is written in bits
                   TRD[15:8], and TRD[7:0] must be set to the data value complemented by 1 */
    RZ,         /**< UART, using Return to Zero [bits 8, 0, 9, 1 ... 15, 7]. Data is written in bits TRD[15:8]. Unused
                   bits TRD[7:0] must be set to 00h. */
} UartBscCtrl_t;
#define UART_BSC_ctrl_t UartBscCtrl_t __attribute__ ((deprecated("Renamed to UartBscCtrl_t")))

/** Message length selector. Double buffer mechanism */
typedef enum UartMlsCtrl_e {
    UART_MLS_8Bit = 0u,/**< 8 bit of data BSD[7:0] + 1 [stop] bit as BSD8 */
    UART_MLS_9Bit,     /**< 9 bit of data BSD[8:0] + 1 [stop] bit as BSD9 */
    UART_MLS_10Bit,    /**< 10 bit of data BSD[9:0] + 1 [stop] bit as BSD10 */
    UART_MLS_11Bit,    /**< 11 bit of data BSD[10:0] + 1 [stop] bit as BSD11 */
    UART_MLS_12Bit,    /**< 12 bit of data BSD[11:0] + 1 [stop] bit as BSD12 */
    UART_MLS_13Bit,    /**< 13 bit of data BSD[12:0] + 1 [stop] bit as BSD13 */
    UART_MLS_14Bit,    /**< 14 bit of data BSD[13:0] + 1 [stop] bit as BSD14 */
    UART_MLS_15Bit,    /**< 15 bit of data BSD[14:0] + 1 [stop] bit as BSD15 */
} UartMlsCtrl_t;
#define UART_MLS_ctrl_t UartMlsCtrl_t __attribute__ ((deprecated("Renamed to UartMlsCtrl_t")))


/** Selector and Control Register type definition */
typedef union UartCtrl_u {
    struct ctrlBits {
        uint16_t reserved1 : 1;  /**< reserved bit-field */
        uint16_t TRE : 1;       /**< Transmitter enable */
        uint16_t REE : 1;       /**< Receiver enable */
        uint16_t ISB : 1;       /**< Transmit IDLE state */
        uint16_t reserved2 : 1;  /**< reserved bit-field */
        uint16_t LBE : 1;       /**< LIN bit error. Set to 0 if TRE is asserted low. Read and clear bit, set to zero for
                                   UART */
        uint16_t LSE : 1;       /**< LIN start error. Set to 0 if TRE is asserted low. Read and clear bit, set to zero
                                   for UART */
        uint16_t LTE : 1;       /**< LIN time out error. Set to 0 if TRE is asserted low. Read and clear bit, set to
                                   zero for UART */
        UartMlsCtrl_t MLS : 3;  /**< Message length selector */
        UartBscCtrl_t BSC : 2;  /**< Bit scrambler control */
        uint16_t LBC : 1;       /**< LIN bit control. Double buffer mechanism */
        uint16_t LSC : 1;       /**< LIN start bit control. Double buffer mechanism */
        uint16_t LDC : 1;       /**< LIN duration control */
    } ctrl;
    uint16_t data;
} UartCtrl_t;
#define UART_ctrl_t UartCtrl_t __attribute__ ((deprecated("Renamed to UartCtrl_t")))


/** Describes the statuses and error flags of UART */
typedef union UartStatus_u {
    struct statusBits {
        uint16_t reserved : 8;  /**< reserved [7:0] */
        uint16_t TRO : 1;       /**< Transmit register overrun. In LIN mode, bit set to 0 if read UCTRL. Set to 0 if TRE
                                   is asserted low. Read and clear bit */
        uint16_t TSB : 1;       /**< Transmit shifter busy. In LIN mode, bit set to 0 if LSE = 1 or LBE = 1. Set to 0 if
                                   TRE is asserted low */
        uint16_t RSO : 1;       /**< Receive shifter overrun. Set to 0 if REE is asserted low. Read and clear bit */
        uint16_t RSB : 1;       /**< Receive shifter busy. Set to 0 if REE is asserted low */
        uint16_t RRF : 1;       /**< Receive register full. Set to 0 if REE is asserted low */
        uint16_t NBR : 1;       /**< Noisy bit reception. Set to 0 if REE is asserted low. Read and clear bit */
        uint16_t SBE : 1;       /**< Stop bit error. Set to 0 if REE is asserted low. Read and clear bit */
    } ctrl;
    uint16_t data;
} UartStatus_t;
#define UART_status_t UartStatus_t __attribute__ ((deprecated("Renamed to UartStatus_t")))

/** UART_GetRate is a special function to calculate the UART BRRD's register value from the expected baudrate, assigned
 * in bits per second
 *
 * @param[in] freq - the baudrate expected to be set
 * Fbit = Fck / (16 * (UART_RATE+1))
 */
STATIC INLINE uint16_t UART_GetRate(uint32_t freq);

#ifndef HAS_TWO_UARTS
#if !defined(IO_UART__BRRD_GET) && !defined(UNITTEST)
#warning The UART is not available while the UART library is included.
#endif /* UART IO PORTS check */


/** UART_SetBaudrate sets the baudrate for UART communication
 * @param[in] brrd_reg_value the baudrate calculated for the UART HW. Can be set as a result of UART_GetRate function
 */
STATIC INLINE void UART_SetBaudrate(uint16_t brrd_reg_value);

#define UART_INIT UART_Init __attribute__ ((deprecated("Renamed to UART_Init")))
#define UART_RRI_ENABLE  __attribute__ ((deprecated("Renamed to UART_RriEnable")))
#define UART_TRI_DISABLE __attribute__ ((deprecated("Renamed to UART_RriDisable")))
#define UART_RSI_ENABLE __attribute__ ((deprecated("Renamed to UART_RsiEnable")))
#define UART_RSI_DISABLE __attribute__ ((deprecated("Renamed to UART_RsiDisable")))
#define UART_TSI_ENABLE __attribute__ ((deprecated("Renamed to UART_TsiEnable")))
#define UART_TSI_DISABLE __attribute__ ((deprecated("Renamed to UART_TsiDisable")))
#define UART_SBI_ENABLE __attribute__ ((deprecated("Renamed to UART_SbiEnable")))
#define UART_SBI_DISABLE __attribute__ ((deprecated("Renamed to UART_SbiDisable")))
#define UART_TEI_ENABLE __attribute__ ((deprecated("Renamed to UART_TeiEnable")))
#define UART_TEI_DISABLE __attribute__ ((deprecated("Renamed to UART_TeiDisable")))
#define UART_TX_BUSY __attribute__ ((deprecated("Renamed to UART_TxBusy")))
#define UART_RX_FULL __attribute__ ((deprecated("Renamed to UART_RxFull")))
#define UART_PUT_FRAME_7N1 __attribute__ ((deprecated("Renamed to UART_PutFrame7N1")))
#define UART_PUT_FRAME_8N1 __attribute__ ((deprecated("Renamed to UART_PutFrame8N1")))
#define UART_PUT_FRAME_8N1M __attribute__ ((deprecated("Renamed to UART_PutFrame8N1M")))
#define UART_PUT_FRAME_9N1 __attribute__ ((deprecated("Renamed to UART_PutFrame9N1")))
#define UART_PUT_FRAME_10N1 __attribute__ ((deprecated("Renamed to UART_PutFrame10N1")))
#define UART_PUT_FRAME_11N1 __attribute__ ((deprecated("Renamed to UART_PutFrame11N1")))
#define UART_PUT_FRAME_12N1 __attribute__ ((deprecated("Renamed to UART_PutFrame12N1")))
#define UART_PUT_FRAME_13N1 __attribute__ ((deprecated("Renamed to UART_PutFrame13N1")))
#define UART_PUT_FRAME_14N1 __attribute__ ((deprecated("Renamed to UART_PutFrame14N1")))
#define UART_PUT_BREAK_BLOCKING __attribute__ ((deprecated("Renamed to UART_PutBreakBlocking")))
#define UART_PUT_RAW __attribute__ ((deprecated("Renamed to UART_PutRaw")))
#define UART_GET __attribute__ ((deprecated("Renamed to UART_Get")))
#define UART_TX_ENABLE __attribute__ ((deprecated("Renamed to UART_TxEnable")))
#define UART_TX_DISABLE __attribute__ ((deprecated("Renamed to UART_TxDisable")))
#define UART_RX_ENABLE __attribute__ ((deprecated("Renamed to UART_RxEnable")))
#define UART_RX_DISABLE __attribute__ ((deprecated("Renamed to UART_RxDisable")))
#define UART_ALL_DISABLE __attribute__ ((deprecated("Renamed to UART_AllDisable")))
#define UART_LIN_LDC_ENABLE __attribute__ ((deprecated("Renamed to UART_LinLdcEnable")))
#define UART_LIN_LDC_DISABLE __attribute__ ((deprecated("Renamed to UART_LinLdcDisable")))
#define UART_LIN_LSC_ENABLE __attribute__ ((deprecated("Renamed to UART_LinLscEnable")))
#define UART_LIN_LSC_DISABLE __attribute__ ((deprecated("Renamed to UART_LinLscDisable")))
#define UART_LIN_LBC_ENABLE __attribute__ ((deprecated("Renamed to UART_LinLbcEnable")))
#define UART_LIN_LBC_DISABLE __attribute__ ((deprecated("Renamed to UART_LinLbcDisable")))
#define UART_RESET_REGISTER __attribute__ ((deprecated("Renamed to UART_ResetRegister")))
#define UART_TX_FORMAT __attribute__ ((deprecated("Renamed to UART_TxFormat")))
#define UART_FRAME_LENGTH __attribute__ ((deprecated("Renamed to UART_FrameLength")))


/** UART_Init sets the configuration for the UART and it's baudrate
 * @param[in] rate the baudrate calculated for the UART. Can be set as a result of UART_GetRate function
 * @param[in] cfg the UART's configuration. Can have UART enabled in its settings.
 *
 */
STATIC INLINE void UART_Init(uint16_t rate, UartCtrl_t cfg);

/* Interrupts support */

/** UART_RriEnable enables the Receive Register full Interrupt */
STATIC INLINE void UART_RriEnable(void);

/** UART_RriDisable disables the Receive Register full Interrupt */
STATIC INLINE void UART_RriDisable(void);

/** UART_TriEnable enables the Transmit Register empty Interrupt */
STATIC INLINE void UART_TriEnable(void);

/** UART_TriDisable disables the Transmit Register empty Interrupt */
STATIC INLINE void UART_TriDisable(void);

/** UART_RsiEnable enables the Receive Shifter overrun Interrupt */
STATIC INLINE void UART_RsiEnable(void);

/** UART_RsiDisable disables the Receive Shifter overrun Interrupt */
STATIC INLINE void UART_RsiDisable(void);

/** UART_TsiEnable enables the Transmitter Shifter empty Interrupt */
STATIC INLINE void UART_TsiEnable(void);

/** UART_TsiDisable disables the Transmitter Shifter empty Interrupt */
STATIC INLINE void UART_TsiDisable(void);

/** UART_SbiEnable enables the Stop Bit Error Interrupt */
STATIC INLINE void UART_SbiEnable(void);

/** UART_SbiDisable disables the Stop Bit Error Interrupt */
STATIC INLINE void UART_SbiDisable(void);

/** UART_TeiEnable enables the Transmit Register overrun Interrupt */
STATIC INLINE void UART_TeiEnable(void);

/** UART_TeiDisable disables the Transmit Register overrun Interrupt */
STATIC INLINE void UART_TeiDisable(void);

/* Flags support */

/** UART_TxBusy returns the current TX busy flag */
STATIC INLINE uint16_t UART_TxBusy(void);

/** UART_RxFull returns the current RX full flag */
STATIC INLINE uint16_t UART_RxFull(void);



/* Data sending */

/** Puts data frame with start and stop bits to UART
 *
 * @param[in] length sets the data bit length
 * @param[in] x sets the data to send
 *
 * Notes:
 *  1. Functions UART_PutFramexx are only applicable for UART_FORMAT_LSB
 *     format (defined during initialization, see UART_Init)
 *
 */
STATIC INLINE void UART_InternalPutFrame(uint16_t length, uint16_t x);
#define __UART_PUT_FRAME        UART_InternalPutFrame __attribute__ ((deprecated("Renamed to UART_InternalPutFrame")))
#define INTERNAL_UART_PUT_FRAME UART_InternalPutFrame __attribute__ ((deprecated("Renamed to UART_InternalPutFrame")))

/** UART_PutFrame7N1 sends the data in 7N1 format
 *
 * @param[in] data 7 bits to send
 */
STATIC INLINE void UART_PutFrame7N1(uint16_t data);

/** UART_PutFrame8N1 sends the data in 8N1 format
 *
 * @param[in] data 8 bits to send
 */
STATIC INLINE void UART_PutFrame8N1(uint16_t data);

/** UART_PutFrame8N1M sends the data in 8N1 format, assuming MSB mode
 *
 * @param[in] data 8 bits to send
 */
STATIC INLINE void UART_PutFrame8N1M(uint16_t data);

/** UART_PutFrame9N1 sends the data in 9N1 format
 *
 * @param[in] data 9 bits to send
 */
STATIC INLINE void UART_PutFrame9N1(uint16_t data);

/** UART_PutFrame10N1 sends the data in 10N1 format
 *
 * @param[in] data 10 bits to send
 */
STATIC INLINE void UART_PutFrame10N1(uint16_t data);

/** UART_PutFrame11N1 sends the data in 11N1 format
 *
 * @param[in] data 11 bits to send
 */
STATIC INLINE void UART_PutFrame11N1(uint16_t data);

/** UART_PutFrame12N1 sends the data in 12N1 format
 *
 * @param[in] data 12 bits to send
 */
STATIC INLINE void UART_PutFrame12N1(uint16_t data);

/** UART_PutFrame13N1 sends the data in 13N1 format
 *
 * @param[in] data 13 bits to send
 */
STATIC INLINE void UART_PutFrame13N1(uint16_t data);

/** UART_PutFrame14N1 sends the data in 14N1 format
 *
 * @param[in] data 14 bits to send
 */
STATIC INLINE void UART_PutFrame14N1(uint16_t data);

/** UART_WaitTx waits till tranmit register will be ready to send the next data portion
 *
 * @note: Acknowledges the AWD
 */
STATIC void UART_WaitTx(void);

/** UART_PutBreakBlocking sends the data break
 *
 * @param[in] break_length number of bits of break
 *
 * The function configures the UART to send the break field and waits for its end (and restores the UART's configuration)
 */
STATIC INLINE void UART_PutBreakBlocking(UartMlsCtrl_t break_length);

/** UART_PutRaw sends the data into the TX output register
 *
 * @param data - is raw value without start-stop
 * @note:
 *  1. Function is applicable for all data formats:
 *     UART_FORMAT_LSB, UART_FORMAT_MSB, UART_FORMAT_MPE and
 *     UART_FORMAT_RZ
 */
STATIC INLINE void UART_PutRaw(uint16_t data);



/* Read functions */

/** UART_Get returns the current RX buffer received
 *
 * @return RX register
 */
STATIC INLINE uint16_t UART_Get(void);

/*********************** Function from lm_uart, Mulan2/3 ******************************************/

/** UART_TxEnable Enable UART unit for transmit.
 *
 */
STATIC INLINE void UART_TxEnable(void);

/** UART_TxDisable Disable UART unit for transmit.
 *
 */
STATIC INLINE void UART_TxDisable(void);

/** UART_RxEnable Enable UART unit for receive.
 *
 * Enable UART unit before starting the module configuration.
 */
STATIC INLINE void UART_RxEnable(void);

/** UART_RxDisable Disable UART unit for receive.
 *
 * Disable UART unit before starting the module configuration.
 */
STATIC INLINE void UART_RxDisable(void);

/** UART_AllDisable Disable UART unit.
 *
 */
STATIC INLINE void UART_AllDisable(void);

/** UART_LinLdcEnable Enable LIN Mode - Duration Control checking.
 *
 * Enable counting the number of bit, compare UART_LFD and UART_LFC.
 */
STATIC INLINE void UART_LinLdcEnable(void);

/** UART_LinLdcDisable Disable LIN Mode
 *
 * Disable counting the number of bit, compare UART_LFD and UART_LFC.
 */
STATIC INLINE void UART_LinLdcDisable(void);

/** UART_LinLscEnable Enable LIN Mode - Start bit Control checking.
 *
 * Enable receive state verifying before sending message.
 */
STATIC INLINE void UART_LinLscEnable(void);

/** UART_LinLscDisable Disable LIN Mode - Start bit Control checking.
 *
 * Disable receive state verifying before sending message.
 */
STATIC INLINE void UART_LinLscDisable(void);

/** UART_LinLbcEnable Enable LIN Mode - Bit bit Control checking.
 *
 * Enable input and output data equality.
 */
STATIC INLINE void UART_LinLbcEnable(void);

/** UART_LinLbcDisable Disable LIN Mode - Bit Control checking.
 *
 * Disable input and output data equality.
 */
STATIC INLINE void UART_LinLbcDisable(void);

/** UART_ResetRegister Enables the UART unit and resets UART Registers to 0.
 *
 * Resets UART unit to UART mode, LIN Master mode is disabled.
 */
STATIC INLINE void UART_ResetRegister(void);

/** UART_TxFormat Select the format for data transmit.
 *
 * Configures the bit scrambler.
 * @param[in] format is one of formats specified by UartBscCtrl_t enum
 */
STATIC INLINE void UART_TxFormat(UartBscCtrl_t format);

/** UART_FrameLength Select the UART frame length including start and stop bit.
 *
 * @param[in] len - is one of lengths specified by UartMlsCtrl_t enum
 */
STATIC INLINE void UART_FrameLength(UartMlsCtrl_t len);


#else /* HAS_TWO_UARTS */


#if !defined(IO_UART0__BRRD_GET) && !defined(UNITTEST)
#warning The UART is not available while the UART library is included.
#endif /* UART IO PORTS check */


/** UART0_SetBaudrate sets the baudrate for UART communication
 * @param[in] brrd_reg_value the baudrate calculated for the UART HW. Can be set as a result of UART_GetRate function
 */
STATIC INLINE void UART0_SetBaudrate(uint16_t brrd_reg_value);

/** UART0_Init sets the configuration for the UART and it's baudrate
 * @param[in] rate the baudrate calculated for the UART. Can be set as a result of UART_GetRate function
 * @param[in] cfg the UART's configuration. Can have UART enabled in its settings.
 *
 */
STATIC INLINE void UART0_Init(uint16_t rate, UartCtrl_t cfg);

/* Interrupts support */

/** UART0_RriEnable enables the Receive Register full Interrupt */
STATIC INLINE void UART0_RriEnable(void);

/** UART0_RriDisable disables the Receive Register full Interrupt */
STATIC INLINE void UART0_RriDisable(void);

/** UART0_TriEnable enables the Transmit Register empty Interrupt */
STATIC INLINE void UART0_TriEnable(void);

/** UART0_TriDisable disables the Transmit Register empty Interrupt */
STATIC INLINE void UART0_TriDisable(void);

/** UART0_RsiEnable enables the Receive Shifter overrun Interrupt */
STATIC INLINE void UART0_RsiEnable(void);

/** UART0_RsiDisable disables the Receive Shifter overrun Interrupt */
STATIC INLINE void UART0_RsiDisable(void);

/** UART0_TsiEnable enables the Transmitter Shifter empty Interrupt */
STATIC INLINE void UART0_TsiEnable(void);

/** UART0_TsiDisable disables the Transmitter Shifter empty Interrupt */
STATIC INLINE void UART0_TsiDisable(void);

/** UART0_SbiEnable enables the Stop Bit Error Interrupt */
STATIC INLINE void UART0_SbiEnable(void);

/** UART0_SbiDisable disables the Stop Bit Error Interrupt */
STATIC INLINE void UART0_SbiDisable(void);

/** UART0_TeiEnable enables the Transmit Register overrun Interrupt */
STATIC INLINE void UART0_TeiEnable(void);

/** UART0_TeiDisable disables the Transmit Register overrun Interrupt */
STATIC INLINE void UART0_TeiDisable(void);

/* Flags support */

/** UART0_TxBusy returns the current TX busy flag */
STATIC INLINE uint16_t UART0_TxBusy(void);

/** UART0_RxFull returns the current RX full flag */
STATIC INLINE uint16_t UART0_RxFull(void);



/* Data sending */

/** Puts data frame with start and stop bits to UART
 *
 * @param[in] length sets the data bit length
 * @param[in] x sets the data to send
 *
 * Notes:
 *  1. Functions UART0_PutFramexx are only applicable for UART0_FORMAT_LSB
 *     format (defined during initialization, see UART0_Init)
 *
 */
STATIC INLINE void INTERNAL_UART0_PUT_FRAME(uint16_t length, uint16_t x);

/** UART0_PutFrame7N1 sends the data in 7N1 format
 *
 * @param[in] data 7 bits to send
 */
STATIC INLINE void UART0_PutFrame7N1(uint16_t data);

/** UART0_PutFrame8N1 sends the data in 8N1 format
 *
 * @param[in] data 8 bits to send
 */
STATIC INLINE void UART0_PutFrame8N1(uint16_t data);

/** UART0_PutFrame8N1M sends the data in 8N1 format, assuming MSB mode
 *
 * @param[in] data 8 bits to send
 */
STATIC INLINE void UART0_PutFrame8N1M(uint16_t data);

/** UART0_PutFrame9N1 sends the data in 9N1 format
 *
 * @param[in] data 9 bits to send
 */
STATIC INLINE void UART0_PutFrame9N1(uint16_t data);

/** UART0_PutFrame10N1 sends the data in 10N1 format
 *
 * @param[in] data 10 bits to send
 */
STATIC INLINE void UART0_PutFrame10N1(uint16_t data);

/** UART0_PutFrame11N1 sends the data in 11N1 format
 *
 * @param[in] data 11 bits to send
 */
STATIC INLINE void UART0_PutFrame11N1(uint16_t data);

/** UART0_PutFrame12N1 sends the data in 12N1 format
 *
 * @param[in] data 12 bits to send
 */
STATIC INLINE void UART0_PutFrame12N1(uint16_t data);

/** UART0_PutFrame13N1 sends the data in 13N1 format
 *
 * @param[in] data 13 bits to send
 */
STATIC INLINE void UART0_PutFrame13N1(uint16_t data);

/** UART0_PutFrame14N1 sends the data in 14N1 format
 *
 * @param[in] data 14 bits to send
 */
STATIC INLINE void UART0_PutFrame14N1(uint16_t data);

/** UART0_WaitTx waits till tranmit register will be ready to send the next data portion
 *
 * @note: Acknowledges the AWD
 */
STATIC void UART0_WaitTx(void);

/** UART0_PutBreakBlocking sends the data break
 *
 * @param[in] break_length number of bits of break
 *
 * The function configures the UART to send the break field and waits for its end (and restores the UART's configuration)
 */
STATIC INLINE void UART0_PutBreakBlocking(UartMlsCtrl_t break_length);

/** UART0_PutRaw sends the data into the TX output register
 *
 * @param data - is raw value without start-stop
 * @note:
 *  1. Function is applicable for all data formats:
 *     UART0_FORMAT_LSB, UART0_FORMAT_MSB, UART0_FORMAT_MPE and
 *     UART0_FORMAT_RZ
 */
STATIC INLINE void UART0_PutRaw(uint16_t data);



/* Read functions */

/** UART0_Get returns the current RX buffer received
 *
 * @return RX register
 */
STATIC INLINE uint16_t UART0_Get(void);

/*********************** Function from lm_uart, Mulan2/3 ******************************************/

/** UART0_TxEnable Enable UART unit for transmit.
 *
 */
STATIC INLINE void UART0_TxEnable(void);

/** UART0_TxDisable Disable UART unit for transmit.
 *
 */
STATIC INLINE void UART0_TxDisable(void);

/** UART0_RxEnable Enable UART unit for receive.
 *
 * Enable UART unit before starting the module configuration.
 */
STATIC INLINE void UART0_RxEnable(void);

/** UART0_RxDisable Disable UART unit for receive.
 *
 * Disable UART unit before starting the module configuration.
 */
STATIC INLINE void UART0_RxDisable(void);

/** UART0_AllDisable Disable UART unit.
 *
 */
STATIC INLINE void UART0_AllDisable(void);

/** UART0_LinLdcEnable Enable LIN Mode - Duration Control checking.
 *
 * Enable counting the number of bit, compare UART0_LFD and UART0_LFC.
 */
STATIC INLINE void UART0_LinLdcEnable(void);

/** UART0_LinLdcDisable Disable LIN Mode
 *
 * Disable counting the number of bit, compare UART0_LFD and UART0_LFC.
 */
STATIC INLINE void UART0_LinLdcDisable(void);

/** UART0_LinLscEnable Enable LIN Mode - Start bit Control checking.
 *
 * Enable receive state verifying before sending message.
 */
STATIC INLINE void UART0_LinLscEnable(void);

/** UART0_LinLscDisable Disable LIN Mode - Start bit Control checking.
 *
 * Disable receive state verifying before sending message.
 */
STATIC INLINE void UART0_LinLscDisable(void);

/** UART0_LinLbcEnable Enable LIN Mode - Bit bit Control checking.
 *
 * Enable input and output data equality.
 */
STATIC INLINE void UART0_LinLbcEnable(void);

/** UART0_LinLbcDisable Disable LIN Mode - Bit Control checking.
 *
 * Disable input and output data equality.
 */
STATIC INLINE void UART0_LinLbcDisable(void);

/** UART0_ResetRegister Enables the UART unit and resets UART Registers to 0.
 *
 * Resets UART unit to UART mode, LIN Master mode is disabled.
 */
STATIC INLINE void UART0_ResetRegister(void);

/** UART0_TxFormat Select the format for data transmit.
 *
 * Configures the bit scrambler.
 * @param[in] format is one of formats specified by UartBscCtrl_t enum
 */
STATIC INLINE void UART0_TxFormat(UartBscCtrl_t format);

/** UART0_FrameLength Select the UART frame length including start and stop bit.
 *
 * @param[in] len - is one of lengths specified by UartMlsCtrl_t enum
 */
STATIC INLINE void UART0_FrameLength(UartMlsCtrl_t len);




#if !defined(IO_UART1__BRRD_GET) && !defined(UNITTEST)
#warning The UART is not available while the UART library is included.
#endif /* UART IO PORTS check */


/** UART1_SetBaudrate sets the baudrate for UART communication
 * @param[in] brrd_reg_value the baudrate calculated for the UART HW. Can be set as a result of UART_GetRate function
 */
STATIC INLINE void UART1_SetBaudrate(uint16_t brrd_reg_value);

/** UART1_Init sets the configuration for the UART and it's baudrate
 * @param[in] rate the baudrate calculated for the UART. Can be set as a result of UART_GetRate function
 * @param[in] cfg the UART's configuration. Can have UART enabled in its settings.
 *
 */
STATIC INLINE void UART1_Init(uint16_t rate, UartCtrl_t cfg);

/* Interrupts support */

/** UART1_RriEnable enables the Receive Register full Interrupt */
STATIC INLINE void UART1_RriEnable(void);

/** UART1_RriDisable disables the Receive Register full Interrupt */
STATIC INLINE void UART1_RriDisable(void);

/** UART1_TriEnable enables the Transmit Register empty Interrupt */
STATIC INLINE void UART1_TriEnable(void);

/** UART1_TriDisable disables the Transmit Register empty Interrupt */
STATIC INLINE void UART1_TriDisable(void);

/** UART1_RsiEnable enables the Receive Shifter overrun Interrupt */
STATIC INLINE void UART1_RsiEnable(void);

/** UART1_RsiDisable disables the Receive Shifter overrun Interrupt */
STATIC INLINE void UART1_RsiDisable(void);

/** UART1_TsiEnable enables the Transmitter Shifter empty Interrupt */
STATIC INLINE void UART1_TsiEnable(void);

/** UART1_TsiDisable disables the Transmitter Shifter empty Interrupt */
STATIC INLINE void UART1_TsiDisable(void);

/** UART1_SbiEnable enables the Stop Bit Error Interrupt */
STATIC INLINE void UART1_SbiEnable(void);

/** UART1_SbiDisable disables the Stop Bit Error Interrupt */
STATIC INLINE void UART1_SbiDisable(void);

/** UART1_TeiEnable enables the Transmit Register overrun Interrupt */
STATIC INLINE void UART1_TeiEnable(void);

/** UART1_TeiDisable disables the Transmit Register overrun Interrupt */
STATIC INLINE void UART1_TeiDisable(void);

/* Flags support */

/** UART1_TxBusy returns the current TX busy flag */
STATIC INLINE uint16_t UART1_TxBusy(void);

/** UART1_RxFull returns the current RX full flag */
STATIC INLINE uint16_t UART1_RxFull(void);



/* Data sending */

/** Puts data frame with start and stop bits to UART
 *
 * @param[in] length sets the data bit length
 * @param[in] x sets the data to send
 *
 * Notes:
 *  1. Functions UART1_PutFramexx are only applicable for UART1_FORMAT_LSB
 *     format (defined during initialization, see UART1_Init)
 *
 */
STATIC INLINE void INTERNAL_UART1_PUT_FRAME(uint16_t length, uint16_t x);

/** UART1_PutFrame7N1 sends the data in 7N1 format
 *
 * @param[in] data 7 bits to send
 */
STATIC INLINE void UART1_PutFrame7N1(uint16_t data);

/** UART1_PutFrame8N1 sends the data in 8N1 format
 *
 * @param[in] data 8 bits to send
 */
STATIC INLINE void UART1_PutFrame8N1(uint16_t data);

/** UART1_PutFrame8N1M sends the data in 8N1 format, assuming MSB mode
 *
 * @param[in] data 8 bits to send
 */
STATIC INLINE void UART1_PutFrame8N1M(uint16_t data);

/** UART1_PutFrame9N1 sends the data in 9N1 format
 *
 * @param[in] data 9 bits to send
 */
STATIC INLINE void UART1_PutFrame9N1(uint16_t data);

/** UART1_PutFrame10N1 sends the data in 10N1 format
 *
 * @param[in] data 10 bits to send
 */
STATIC INLINE void UART1_PutFrame10N1(uint16_t data);

/** UART1_PutFrame11N1 sends the data in 11N1 format
 *
 * @param[in] data 11 bits to send
 */
STATIC INLINE void UART1_PutFrame11N1(uint16_t data);

/** UART1_PutFrame12N1 sends the data in 12N1 format
 *
 * @param[in] data 12 bits to send
 */
STATIC INLINE void UART1_PutFrame12N1(uint16_t data);

/** UART1_PutFrame13N1 sends the data in 13N1 format
 *
 * @param[in] data 13 bits to send
 */
STATIC INLINE void UART1_PutFrame13N1(uint16_t data);

/** UART1_PutFrame14N1 sends the data in 14N1 format
 *
 * @param[in] data 14 bits to send
 */
STATIC INLINE void UART1_PutFrame14N1(uint16_t data);

/** UART1_WaitTx waits till tranmit register will be ready to send the next data portion
 *
 * @note: Acknowledges the AWD
 */
STATIC void UART1_WaitTx(void);

/** UART1_PutBreakBlocking sends the data break
 *
 * @param[in] break_length number of bits of break
 *
 * The function configures the UART to send the break field and waits for its end (and restores the UART's configuration)
 */
STATIC INLINE void UART1_PutBreakBlocking(UartMlsCtrl_t break_length);

/** UART1_PutRaw sends the data into the TX output register
 *
 * @param data - is raw value without start-stop
 * @note:
 *  1. Function is applicable for all data formats:
 *     UART1_FORMAT_LSB, UART1_FORMAT_MSB, UART1_FORMAT_MPE and
 *     UART1_FORMAT_RZ
 */
STATIC INLINE void UART1_PutRaw(uint16_t data);



/* Read functions */

/** UART1_Get returns the current RX buffer received
 *
 * @return RX register
 */
STATIC INLINE uint16_t UART1_Get(void);

/*********************** Function from lm_uart, Mulan2/3 ******************************************/

/** UART1_TxEnable Enable UART unit for transmit.
 *
 */
STATIC INLINE void UART1_TxEnable(void);

/** UART1_TxDisable Disable UART unit for transmit.
 *
 */
STATIC INLINE void UART1_TxDisable(void);

/** UART1_RxEnable Enable UART unit for receive.
 *
 * Enable UART unit before starting the module configuration.
 */
STATIC INLINE void UART1_RxEnable(void);

/** UART1_RxDisable Disable UART unit for receive.
 *
 * Disable UART unit before starting the module configuration.
 */
STATIC INLINE void UART1_RxDisable(void);

/** UART1_AllDisable Disable UART unit.
 *
 */
STATIC INLINE void UART1_AllDisable(void);

/** UART1_LinLdcEnable Enable LIN Mode - Duration Control checking.
 *
 * Enable counting the number of bit, compare UART1_LFD and UART1_LFC.
 */
STATIC INLINE void UART1_LinLdcEnable(void);

/** UART1_LinLdcDisable Disable LIN Mode
 *
 * Disable counting the number of bit, compare UART1_LFD and UART1_LFC.
 */
STATIC INLINE void UART1_LinLdcDisable(void);

/** UART1_LinLscEnable Enable LIN Mode - Start bit Control checking.
 *
 * Enable receive state verifying before sending message.
 */
STATIC INLINE void UART1_LinLscEnable(void);

/** UART1_LinLscDisable Disable LIN Mode - Start bit Control checking.
 *
 * Disable receive state verifying before sending message.
 */
STATIC INLINE void UART1_LinLscDisable(void);

/** UART1_LinLbcEnable Enable LIN Mode - Bit bit Control checking.
 *
 * Enable input and output data equality.
 */
STATIC INLINE void UART1_LinLbcEnable(void);

/** UART1_LinLbcDisable Disable LIN Mode - Bit Control checking.
 *
 * Disable input and output data equality.
 */
STATIC INLINE void UART1_LinLbcDisable(void);

/** UART1_ResetRegister Enables the UART unit and resets UART Registers to 0.
 *
 * Resets UART unit to UART mode, LIN Master mode is disabled.
 */
STATIC INLINE void UART1_ResetRegister(void);

/** UART1_TxFormat Select the format for data transmit.
 *
 * Configures the bit scrambler.
 * @param[in] format is one of formats specified by UartBscCtrl_t enum
 */
STATIC INLINE void UART1_TxFormat(UartBscCtrl_t format);

/** UART0_FrameLength Select the UART frame length including start and stop bit.
 *
 * @param[in] len - is one of lengths specified by UartMlsCtrl_t enum
 */
STATIC INLINE void UART1_FrameLength(UartMlsCtrl_t len);


#endif /* HAS_TWO_UARTS */


#ifndef UNITTEST
#include "lib_uart_inline_impl.h"
#endif /* UNITTEST */

#endif /* LIB_UART_H */
/// @}

