/**
 * @file
 * @brief PPM bootloader functions
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
 * @ingroup ppm_phy
 * @addtogroup ppm_phy PPM Physical Layer
 *
 * @brief PPM bootloader functions
 * @details
 * The Pulse-position-modulation (PPM) is the Physical Layer library providing the simple API to maintain,
 * configure and provide the data transfer over the PPM-bus (will be based on the LIN PIN).
 * @{
 */

#ifndef BL_PPM_H
#define BL_PPM_H

#include <stdint.h>
#include <syslib.h>
#include "static_assert.h"
#include "ppm_lib.h"
#include "lib_miscio.h"
#include "flash_defines.h"
#include "eeprom_map.h"

#if !defined(HAS_SW_PPM_BL_USE_ROM_ONLY) && !defined(HAS_SW_PPM_HAS_RAM_CODE_FRAME)
#error Forbidden configuration. The one from (HAS_SW_PPM_BL_USE_ROM_ONLY, HAS_SW_PPM_HAS_RAM_CODE_FRAME) should be used!
#endif

/** clock ticks per us expected */
#define PPM_TICKS_PER_1US       ((uint16_t)FPLL / 1000u)

/** default clock ticks for Timeout
 *
 * \f{eqnarray*}{
 * T_{default\_timeout} &= T_{calib\_frame\_10kbps} * 0.150\\
 *                      &= 4500\mu s * 0.150\\
 *                      &=675\mu s
 * \f}
 */
#define PPM_DEFAULT_TIMEOUT     (PPM_TICKS_PER_1US * 675u)

/** default ON_timer PPM value: ::PPM_TICKS_PER_1US * 1.5 */
#define DEFAULT_PPM_ON_TIME     ((PPM_TICKS_PER_1US * 15u) / 10u)


/** Describes the receivers state to create the receivers' state machine for the SW type of data parsing */
typedef enum {
    ppmState_Disabled = 0,       /**< State when the RX is initiated but not in active state. */
    ppmState_Enabled,            /**< State when the RX can receive data. */
    ppmState_Header,             /**< State when the receiver reads the first byte as the header (and synchronizes its timer) */
    ppmState_Receiving,          /**< State when some data comes from the Master. */
    ppmState_Error,              /**< State when the PPM driver catches some error on the bus. Error code could be read. This state should be cleared be the Error code reading. */
    ppmState_SendACK,            /**< State when an Acknowledge sequence was initiated by the SW. Slave then will send the ACK signal. */
} ppmState_t;

typedef ppmState_t ppmReceiverState_t; /**< Lecacy to comply with the earlier versions */

/** Commong return value type for PPM functions */
typedef enum {
    ppmFAIL,        /**< PPM unsuccessful result */
    ppmSUCCESS,     /**< PPM positive result */
} ppmResult_t;

#define PPM_FT_CMDS_OFFSET    0x40u /* Sessions types' offset, added to the Session's ID to call the functions instead of set of pages */

/** PPM Frame types and commands */
typedef enum {
    ppmFT_Unknown = 0U,                                  /**< unknown frame type is being received */
    ppmFT_SessionId = 1U,                                /**< session-id info is being received */
    ppmFT_PageId = 2U,                                   /**< page-id info bytes are being received */
    ppmFT_RamData = 3U,                                  /**< Frame is to upload FLASH keys in the RAM */
    ppmFT_FlashData = 4U,                                /**< flash data bytes are being received. */
    ppmFT_Calib = 5U,                                    /**< calibration bits are being received */
    ppmFT_EepromData = 6U,                               /**< EEPROM data frame */
    ppmFT_FlashCsData = 7U,                              /**< FLASH CS data frame */
#ifdef HAS_SW_PPM_HAS_RAM_CODE_FRAME
    ppmFT_RamFunc = 8U,                                  /**< 128 bytes page data for the RAM code content */
#endif /* HAS_SW_PPM_HAS_RAM_CODE_FRAME */
    /* RAM functions features */
    ppmRAM_FlashWrite = PPM_FT_CMDS_OFFSET + 0x00,       /**< Runs the FLASH write command */
    ppmRam_FlashErase = PPM_FT_CMDS_OFFSET + 0x01,       /**< Runs the FLASH erase (supported by RAM code) */
    ppmRam_FlashEraseCs = PPM_FT_CMDS_OFFSET + 0x02,     /**< Runs the FLASH CS erase (supported by RAM code) */
    ppmFT_BistCheck = PPM_FT_CMDS_OFFSET + 0x03,         /**< Runs the FLASH BIST according the settings */
    ppmFT_EnterProgMode = PPM_FT_CMDS_OFFSET + 0x04,     /**< Enters the 'programming mode', which allows to do smth with the data */
    ppmFT_Reset = PPM_FT_CMDS_OFFSET + 0x05,             /**< Reset frame */
    ppmRam_MarginSet = PPM_FT_CMDS_OFFSET + 0x06,        /**< FLASH Margin set frame */
#if defined(HAS_PPM_EEPROM_VERIFICATION_SESSION)
    ppmFT_EepromVerification = PPM_FT_CMDS_OFFSET + 0x07,/**< EEPROM CRC read session */
#endif
    ppmRam_SessionAck = PPM_FT_CMDS_OFFSET + 0x3E,       /**< Session acknowledge request (to RAM prog) */
} ppmFrameTypes_t;


/** The data names per incoming pulse width scaled to the current calibration
 *
 * @note WARNING: max. number of enums = 256 due to typecasting to uint8_t
 */
typedef enum {
    ppm_Data_0b00 = 0U,                     /**< 0b00 two bits of data */
    ppm_Data_0b01 = 1U,                     /**< 0b01 two bits of data */
    ppm_Data_0b10 = 2U,                     /**< 0b10 two bits of data */
    ppm_Data_0b11 = 3U,                     /**< 0b11 two bits of data */
    ppm_reserved1 = 4U,                     /**< Reserved */
    ppm_SessionHeaderValue = 5U,            /**< Sessions' header pulses period */
    ppm_PageHeaderValue = 6U,               /**< Pages' header pulses period */
    ppm_reserved2 = 7U,                     /**< Reserved */
    ppm_CalibrationValue = 8U               /**< Min calibration time acceptable in programming mode */
} ppmPulseType_t;

/** The function's type declaration to parse the incoming data */
typedef void (* parser_func_t)(uint16_t index);

/** Session's header structure */
typedef struct {
    uint8_t u16P_Count;                     /**< The current page size expected */
    uint8_t u8Flash_Session_type;           /**< Session type: RAM/FLASH/ETC. **/
    uint16_t u16Flash_Page_CNT;             /**< number of Flash memory pages to receive */
    uint16_t u16Flash_CRC_Hi;               /**< high-word of 32-bit total Flash memory CRC */
    uint16_t u16Flash_CRC_Lo;               /**< low-word of 32-bit total Flash memory CRC */
} ppmSessHeader_t;

typedef union {
    ppmSessHeader_t header;
    uint16_t words[4];
} ppmSessHeaderUnion_t;

/** Brief mode description type for the modes table */
typedef struct {
    ppmFrameTypes_t frame_type;             /**< Frame type */
    uint16_t data_size;                     /**< Data_size in words allowed for this Frame type */
    parser_func_t func;                     /**< The function used to handle the Frame type */
} ppmModeDesc_t;

#if defined(HAS_SW_PPM_BL_USE_ROM_ONLY) && !defined(HAS_SW_PPM_HAS_RAM_CODE_FRAME)
/** The structure holds all the data needed for the RAM-functions */
typedef struct {
    volatile FLASH_WriteStruct_t blPpmFlashWriteCfg;    /**< FLASH-write structure used to write the FLASH */
    uint8_t u8Page_SEQ;                                 /**< byte with 1 Flash page sequence number */
    uint8_t u8Page_CRC;                                 /**< byte with 1 Flash page CRC number */
    uint16_t CurrentFlashPage;                          /**< The current FLASH Page Number offset to write */
    uint16_t CurrentEepromOffset;                       /**< The current EEPROM offset to write */
    uint16_t PendingSessionAck;                         /**< Indicated that the Session should be acknowledged after EEPROM or FLASH will be written (for last pages) */
} ppm_RamEntryInput_t;
#else
/** The structure holds all the data needed for the RAM-functions */
typedef struct {
    ppmFrameTypes_t function;                           /**< The FrameType or command */
    uint16_t* data_buf_offs;                            /**< Incoming data offset */
    volatile FLASH_WriteStruct_t blPpmFlashWriteCfg;    /**< FLASH-write structure used to write the FLASH */
    uint8_t u8Page_SEQ;                                 /**< byte with 1 Flash page sequence number */
    uint8_t u8Page_CRC;                                 /**< byte with 1 Flash page CRC number */
    uint16_t CurrentFlashPage;                          /**< The current FLASH Page Number offset to write */
    uint16_t CurrentEepromOffset;                       /**< The current EEPROM offset to write */
    uint16_t PendingSessionAck;                         /**< Indicated that the Session should be acknowledged after EEPROM or FLASH will be written (for last pages) */
    void (* ppm_ROMFillBist)(void);                      /**< Pointer to the ROM's FillBist() function to be used from the RAM */
    ppmSessHeader_t* SHeader;                           /**< Pointer to the structure to be able to change the session's answer/header */
} ppm_RamEntryInput_t;
#endif /* HAS_SW_PPM_BL_USE_ROM_ONLY */

#if !defined(BL_PPM_RAM_ENTRY_H) || defined(HAS_SW_PPM_HAS_RAM_CODE_FRAME)
/** The functions' result definition */
typedef enum {
    PPM_RERES_UNKNOWN = 0u, /**< Result: function's unknown (zero is the wrong functions answer) */
    PPM_RERES_OK,           /**< Result: function's OK */
    PPM_RERES_FAIL,         /**< Result: function's not OK */
    PPM_RERES_UNAVAILABLE,  /**< Functions unavailable*/
} ppm_RamEntryResult_t;
#endif /* !BL_PPM_RAM_ENTRY_H, Define the RAM func interface if it's not included from extern lib */

/*
 * Default library settings.
 */

/* FLASH settings */
/** Represents the first FLASH's page to write during the FLASH write sequence*/
#define FL_BL_FIRST_PAGE    1u


/* Sizes for different types of pages, in 16bit words */
/** Defines the size of RAM keys PPM-protocol page data size, in words */
#define PPM_RAM_PAGE_SIZE       8u
/** Defines the size of EEPROM PPM-protocol page data size, in words */
#define PPM_EEPROM_PAGE_SIZE    4u
/** Defines the size of FLASH PPM-protocol page data size, in words */
#define PPM_FLASH_PAGE_SIZE    ((uint16_t)MEM_FLASH_PAGE_SIZE / 2u)
/** Defines the size of RAM code PPM-protocol page data size, in words */
#define PPM_RAMCODE_PAGE_SIZE  (0x80u / 2u)
/** Defines the buffer size in RAM to store the RAM code, in words */
#define PPM_RAM_CODE_SIZE      (0x400u / 2u)
/** Defines the Header-size within the PAGE data of PPM-protocol page, in words */
#define PPM_PAGE_HEADER_SIZE    1u
/** Defines the Page Acknowledge size, in words */
#define PPM_PAGE_ACK_SIZE    (PPM_PAGE_HEADER_SIZE)
/** Defines the Session Header-size, in words */
#define PPM_SESSION_HEADER_SIZE 4u
/** Defines the Session Acknowledge size, in words */
#define PPM_SESSION_ACK_SIZE    (PPM_SESSION_HEADER_SIZE)


#define PPM_DATA_MODE       ppm_DataMode_TotalOnly
/** Sets the buffer counts in the overall DMA buffer [1..16] */
#define PPM_BUF_COUNTS      16u
/** Sets the 2^value of data amount (in 8*16bit words) which will be caught between the RX interrupts [0..3] */
#define PPM_IT_SIZE         3u
/** Minimum block size defined by the PPM HW */
#define PPM_HW_BLOCK_SIZE   8u


ASSERT(PPM_IT_SIZE < 4u);
ASSERT((PPM_BUF_COUNTS <= 16u) && (PPM_BUF_COUNTS > 0u));

/** Calculates the PPM RX buffer size */
#define PPM_DMA_SIZE            ((PPM_HW_BLOCK_SIZE * PPM_BUF_COUNTS) * (((uint8_t)PPM_DATA_MODE) + 1u))
/** The buffer should hold the TX page output data + 1 page header + 1 'OVF tick pulse data= 0 at the end '*/
#define PPM_PAGE_TX_DMA_SIZE    ((PPM_PAGE_ACK_SIZE * PPM_HW_BLOCK_SIZE) + 2u)
/** The buffer should hold the TX session output data + 1 session header + 1 'OVF tick pulse data= 0 at the end '*/
#define PPM_SESSION_TX_DMA_SIZE ((PPM_SESSION_ACK_SIZE * PPM_HW_BLOCK_SIZE) + 2u)

/** Enter programming mode sign, high 32bits */
#define EPM_CMD_SIGN_HI     ((uint16_t)0x8374u)
/** Enter programming mode sign, low 32bits */
#define EPM_CMD_SIGN_LO     ((uint16_t)0xbf12u)

#if defined(HAS_SW_PPM_HAS_RAM_CODE_FRAME) || defined(HAS_SW_PPM_BL_USE_ROM_ONLY)
/* Global variables */
extern ppm_RamEntryInput_t ppm_data;
extern uint16_t u16Page_RAM[PPM_FLASH_PAGE_SIZE + PPM_PAGE_HEADER_SIZE];
extern ppmSessHeaderUnion_t ppmSessHeader_s;
#endif /* HAS_SW_PPM_BL_USE_ROM_ONLY */

extern volatile uint16_t u16PPM_RX_ARRAY[];
extern ppm_InputPin ppm_InputPinSel;

/*
 * Library functions
 */

/** Initiates a set of parameters to start the receiving the PPM sequences
 *
 * Called when a new ppm pulse was received
 *
 */
void ppm_Init(void);

/** Sets the new state to the PPM interface
 *
 * Enables or disables the initiated PPM interface.
 *
 * @param[in] state the new PPM's state.
 */
void ppm_SetState(ppmState_t state);

/** Called from the main loop to control the state, timeouts, etc.
 *
 * Does nothing so far. All activity is performed through the interrupts.
 *
 */
void ppm_Tick(void);

/** Decode half-buffer
 *
 * Called after half-buffer interrupt
 */
void ppm_HalfBuffer_Decode(void);

/** Timeout handler
 *
 * Called after timeout interrupt (pre-defined RX_timeout +22us)
 */
void ppm_TimeOut_Decode(void);

/** Checks the sequence onto the PPM buffer is it holds the data as a PPM KEY
 *
 * @retval ppmFAIL Failed decoding
 * @retval ppmSUCCESS Succeeded decoding
 */
ppmResult_t ppm_CheckEntry(void);

/** Send the TX signal to the master
 *
 * @param[in] tx_buf the data buffer to send.
 * @retval ppmSUCCESS Succeeded to send the signal
 */
ppmResult_t ppm_SendTX(const uint16_t* tx_buf);

/** Send the Page Acknowledge signal to the master
 *
 * @retval ppmSUCCESS Succeeded to send the signal
 */
ppmResult_t ppm_TX_Page_Ack(void);

/** Send the Session Acknowledge signal to the master
 *
 * @retval ppmSUCCESS Succeeded to send the signal
 */
ppmResult_t ppm_SendSessionAck(const uint16_t* dataBuf);

#if !defined(HAS_SW_PPM_BL_USE_ROM_ONLY) || defined(HAS_SW_PPM_HAS_RAM_CODE_FRAME)
/** Call RAM process if checks are successful
 *
 * @param[in] index index of the selected process in the ::modes_table.
 */
void ppm_CallRamProcessPage(uint16_t index);
#endif /* !HAS_SW_PPM_BL_USE_ROM_ONLY or HAS_SW_PPM_HAS_RAM_CODE_FRAME */

/** Process the received message as a session ID request
 *
 * @param[in] index Not used
 */
void ppm_Process_SessionId(uint16_t index);

/** Acknowledge the reset request and execute a cold reset
 */
void ppm_Process_Reset(void);

/** Acknowledge the enter programming mode request and enter programming mode if the request is valid
 */
void ppm_Process_EPM(void);

/** Runs the FLASH BIST check according the input data and returns the result
 */
void ppm_Process_BIST(void);
#if defined(HAS_PPM_EEPROM_VERIFICATION_SESSION)
void ppm_ProcessEepromVerification(void);
#endif

/** Process the received calibration message
 *
 * @param[in] index Not used
 */
void ppm_Process_Calib(uint16_t index);

#ifdef HAS_SW_PPM_HAS_RAM_CODE_FRAME
/** Process the received RAM function message
 *
 * @param[in] index Not used
 */
void ppm_Process_RAMFunc(uint16_t index);
#endif

/** Send an acknowledgement and do a page count check
 */
void ppm_PostProcessDataPage(void);

/** Load RAM page and send an acknowledgement
 *
 * @param[in] index Not used
 */
void ppm_Process_RamPage(uint16_t index);

/** Run a full FLASH BIST and fill the FLASH BIST result into the Sessions' ACK structure
 */
void ppm_FillBist(void);

#endif /* BL_PPM_H */
/// @}
