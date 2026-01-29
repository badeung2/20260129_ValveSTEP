/**
 * @file
 * @brief Header file for SENT Hardware Abstraction Layer
 * @internal
 *
 * @copyright (C) 2015-2018 Melexis N.V.
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
 * @ingroup sent
 * @addtogroup sent SENT
 *
 * @brief SENT Hardware Abstraction Layer
 * @details
 * @{
 */
#ifndef SENT_HAL_H
#define SENT_HAL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <syslib.h>
#include "compiler_abstraction.h"
#include "lib_miscio.h"

/** The tick time is internally represented in units of 0.1 us.
 *
 * This is done to allow a stepwise configuration of the tick time in steps of 0.5 us, ranging from 0.5 us to 12 us.
 * So, internally, the tick time is represented with an integer inside the range [5 .. 120]:
 *
 * - 5 equals a tick time of 0.5 us;
 * - 120 equals a tick time of 12 us;
 *
 * The #SENT_TICK_US_TO_INTERNAL macro can be used to easily convert the tick time to the correct internal format.
 */
#define SENT_TICK_TIME_SCALE                10.0

/** Convert the SENT tick time in microseconds to the internal memory (fixed point) representation. */
#define SENT_TICK_US_TO_INTERNAL(us)        (uint8_t)((SENT_TICK_TIME_SCALE) *(us))

/** Helper macro to initialize the #SentConfiguration_t.clocksPerTick member. */
#define SENT_INIT_CLOCKS_PER_TICK(tickTimeUs, ppmClockFrequencyMHz) \
    ((long)(SENT_TICK_US_TO_INTERNAL(tickTimeUs)) * (ppmClockFrequencyMHz))

/** The number of sync pulse ticks. */
#define SENT_SYNC_TICKS                     56u

/** For SENT configurations where no callback is required. */
#define SENT_NO_CALLBACK                    (NULL)

/** The minimum number of data nibbles per SENT message. */
#define SENT_MIN_NR_OF_DATA_NIBBLES         1u

/** The maximum number of data nibbles per SENT message. */
#define SENT_MAX_NR_OF_DATA_NIBBLES         6u

/** The total number of non-data nibbles in a SENT message that must be encoded when transmitted by the PPM hardware. */
#define SENT_NR_OF_NON_DATA_NIBBLES_TO_ENCODE ( \
        1u +                            /* The calibration/synchronization pulse. */ \
        1u +                            /* The status and communication nibble. */ \
        1u +                            /* The CRC nibble. */ \
        1u                              /* The pause pulse (not optional). */ \
        )

/** The total number of non-data nibbles in a SENT message that must be decoded when received by the PPM hardware.
 *
 * @note The calibration pulse is not decoded because its value is assumed to be #SENT_SYNC_TICKS ticks in length. The
 *       PPM timer value of the calibration pulse will be used to decode all other nibbles. Also, the pause pulse is
 *       not decoded because it does not store data, it is only used to trigger the PPM time out.
 */
#define SENT_NR_OF_NON_DATA_NIBBLES_TO_DECODE ( \
        1u +                            /* The status and communication nibble. */ \
        1u                              /* The CRC nibble. */ \
        )

/** The total number of nibbles in a SENT message that must be encoded before transmission by the PPM hardware. */
#define SENT_MAX_NR_OF_NIBBLES_TO_ENCODE ( \
        SENT_NR_OF_NON_DATA_NIBBLES_TO_ENCODE + /* The non-data nibbles to encode. */ \
        SENT_MAX_NR_OF_DATA_NIBBLES             /* The maximum number of data nibbles. */ \
        )

/** The total number of nibbles in a SENT message that must be decoded when received by the PPM hardware.
 *
 * @note The calibration pulse is not decoded because its value is assumed to be #SENT_SYNC_TICKS ticks in length. The
 *       PPM timer value of the calibration pulse will be used to decode all other nibbles. Also, the pause pulse is
 *       not decoded because it does not store data, it is only used to trigger the PPM time out.
 */
#define SENT_MAX_NR_OF_NIBBLES_TO_DECODE ( \
        SENT_NR_OF_NON_DATA_NIBBLES_TO_DECODE + /* The non-data nibbles to decode. */ \
        SENT_MAX_NR_OF_DATA_NIBBLES             /* The maximum number of data nibbles. */ \
        )

/** The clock frequency of the PPM hardware in MHz.
 *
 * If the PPM hardware uses the same clock as the MCU, this number equals the clock frequency of the MCU in MHz.
 */
#define PPM_CLOCK_FREQUENCY_MHZ             (uint16_t)(FPLL / 1000)

/** The PPM MODE configuration, as can be found in the PPM specification.
 *
 * When PPM_MODE = 0u, TOTAL times only are stored in RAM.
 * When PPM_MODE = 1u, both ON and TOTAL times are stored in RAM.
 */
#define PPM_CTRL_MODE       0u

/** Protection for the **PPM_MODE** field. */
#if (PPM_CTRL_MODE != 0u)
    #error "The 'PPM_MODE' field must be set to 0. The software assumes only 1 PPM timer value per SENT pulse."
#endif

/** The size of the PPM data in number of 16-bit words.
 *
 * When PPM_MODE = 0u, the size of the PPM data is 8 words (of 16 bits each).
 * When PPM_MODE = 1u, the size of the PPM data is 16 words (of 16 bits each).
 *
 * The length of the PPM buffer must be configured in multiples of the #PPM_DATA_SIZE, which depends on the
 * configuration of the PPM hardware.
 */
#define PPM_DATA_SIZE       (8u * (PPM_CTRL_MODE + 1u))

/** The length of the DMA buffer must at least be equal to the #SENT_MAX_NR_OF_NIBBLES_TO_ENCODE, rounded up to a
 *  multiple of the #PPM_DATA_SIZE as defined in the PPM specification.
 */
#define PPM_BUFFER_LENGTH   CEIL(SENT_MAX_NR_OF_NIBBLES_TO_ENCODE, PPM_DATA_SIZE)

/** Type definition for the PPM DMA buffer storing the PPM timer values for the receiver and transmitter configurations.
 *
 * The PPM hardware can only be used for receiving **or** transmitting. Receiving and transmitting simultaneously is
 * impossible, so only one PPM buffer is needed. It will be shared and accessed by the PPM hardware via DMA.
 *
 * @note The size of the buffer is rounded up to the first multiple of the PPM data size (see #PPM_DATA_SIZE), because
 *       the PPM hardware only allows DMA memory allocation in multiples of the PPM data size.
 *
 * @note Depending on the number of data nibbles configured in #SentConfiguration_t.nrOfDataNibbles, the index of the
 *       CRC checksum and the pause pulse inside the PPM buffer.
 *
 * @note The table below shows which entries of the #SentMessage_t buffer match with entries of the PPM buffer.
 *
 * \code
 *
 * +-------+-------------+-------------------+---------------+
 * | Index | PpmBuffer_t | SentMessage_t     | Description   |
 * +=======+=============+===================+===============+ -----------+ PPM_BUFFER_LENGTH
 * | 0     | Sync.       |                   |               |            |
 * +-------+-------------+-------------------+---------------+ -------+ SENT_MAX_NR_OF_NIBBLES_TO_DECODE
 * | 1     | Stat./Comm. | SentMessage_t[0]  | Stat./Comm.   |        |   |
 * +-------+-------------+-------------------+---------------+ ---+ SENT_MAX_NR_OF_DATA_NIBBLES
 * | 2     |             | SentMessage_t[1]  | Data nibble 1 |    |   |   |
 * +-------+-------------+-------------------+---------------+    |   |   |
 * | 3     |             | SentMessage_t[2]  | Data nibble 2 |    |   |   |
 * +-------+-------------+-------------------+---------------+    |   |   |
 * | 4     |             | SentMessage_t[3]  | Data nibble 3 |    |   |   |
 * +-------+-------------+-------------------+---------------+    |   |   |
 * | 5     |             | SentMessage_t[4]  | Data nibble 4 |    |   |   |
 * +-------+-------------+-------------------+---------------+    |   |   |
 * | 6     |             | SentMessage_t[5]  | Data nibble 5 |    |   |   |
 * +-------+-------------+-------------------+---------------+    |   |   |
 * | 7     |             | SentMessage_t[6]  | Data nibble 6 |    |   |   |
 * +-------+-------------+-------------------+---------------+ ---+   |   |
 * | 8     | CRC         | SentMessage_t[7]  | CRC           |        |   |
 * +-------+-------------+-------------------+---------------+ -------+   |
 * | 9     | Pause       |                   |               |            |
 * +-------+-------------+-------------------+---------------+            |
 * | 10    |             |                   |               |            |
 * +-------+-------------+-------------------+---------------+            |
 * | 11    |             |                   |               |            |
 * +-------+-------------+-------------------+---------------+            |
 * | 12    |             |                   |               |            |
 * +-------+-------------+-------------------+---------------+            |
 * | 13    |             |                   |               |            |
 * +-------+-------------+-------------------+---------------+            |
 * | 14    |             |                   |               |            |
 * +-------+-------------+-------------------+---------------+            |
 * | 15    |             |                   |               |            |
 * +-------+-------------+-------------------+---------------+ -----------+
 *
 * \endcode
 */
typedef uint16_t PpmBuffer_t[PPM_BUFFER_LENGTH];

/** An array storing a SENT message.
 *
 * The entries of this array map to nibble pulses of a SENT frame. The sync and pause pulses are not included.
 *
 * @note The #PpmBuffer_t documentation shows a table of the #SentMessage_t buffer.
 *
 * \code
 *
 * +---+---------------+
 * | 0 | Status/Comm.  |
 * +---+---------------+
 * | 1 | Data nibble 1 |
 * +---+---------------+
 * | 2 | Data nibble 2 |
 * +---+---------------+
 * | 3 | Data nibble 3 |
 * +---+---------------+
 * | 4 | Data nibble 4 |
 * +---+---------------+
 * | 5 | Data nibble 5 |
 * +---+---------------+
 * | 6 | Data nibble 6 |
 * +---+---------------+
 * | 7 | CRC checksum  |
 * +---+---------------+
 *
 * \endcode
 */
typedef uint8_t SentMessage_t[SENT_MAX_NR_OF_NIBBLES_TO_DECODE];

/** An enumeration for the SPC mode */
typedef enum SpcMode_e {
    SPC_MODE_NONE,                              /**< No SPC enabled. */
    SPC_SYNCHRONOUS_MODE,                       /**< Synchronous point-to-point (one sensor) mode. */
    SPC_BUS_MODE_CONSTANT_LENGTH_TRIGGER_PULSE, /**< Bus mode with constant length trigger pulse. */
    SPC_BUS_MODE_VARIABLE_LENGTH_TRIGGER_PULSE, /**< Bus mode with variable length trigger pulse. */
    SPC_BUS_MODE_OVERLAPPING_TRIGGER_PULSE      /**< Bus mode with fully overlapping trigger pulse. */
} SpcMode_t;

/** An enumeration to indicate the type of CRC checksum to be used.
 *
 * The SENT specification provides two different implementation for calculating the CRC checksum:
 *
 * - the legacy implementation;
 * - the recommended implementation.
 */
typedef enum CrcType_e {
    LEGACY,         /**< Use the legacy CRC checksum. */
    RECOMMENDED     /**< Use the recommended CRC checksum. */
} CrcType_t;

/** Enum for reporting SENT diagnostics. */
typedef enum SentDiagnostic_e {
    SENT_MESSAGE_OK = 0u,       /**< Message encoding/decoding success. */
    SENT_MESSAGE_CRC_ERROR,     /**< A CRC error was detected during decoding. */
    SENT_MESSAGE_ERROR          /**< Message encoding/decoding failure. */
} SentDiagnostic_t;

/** A struct storing the application-specific SENT configuration.
 *
 * @note In case the callback functions are not used, they must be initialized with #SENT_NO_CALLBACK.
 */
typedef struct SentConfiguration_s {
    /** The number of PPM clock ticks equal to 1 SENT tick. See #SENT_INIT_CLOCKS_PER_TICK for initialization. */
    uint16_t clocksPerTick;
    /** The number of data nibbles in a SENT frame. */
    uint8_t nrOfDataNibbles;
    /** The PPM_ERR ISR priority.  */
    uint8_t ppmIsrPrio;
    /** Mode for SPC */
    SpcMode_t mode;
    /** The type of CRC implementation. */
    CrcType_t crcType;
    /** The PPM input pin selector. */
    ppm_InputPin ppmInSel;
    /** Pointer to PPM DMA buffer storing the PPM timer values. */
    PpmBuffer_t* ppmBufferPtr;
    /** Callback triggered (interrupt context) when a message has been received */
    void (* receiveCallback)(SentMessage_t* message);
    /** Callback triggered (interrupt context) when a message has been transmitted */
    void (* transmitCallback)(void);
    /** Callback triggered (interrupt context) when a trigger pulse has been received */
    void (* receivedTriggerPulseCallback)(void);
    /** Callback triggered (interrupt context) when a trigger pulse has been transmitted */
    void (* transmittedTriggerPulseCallback)(void);
    /** Callback triggered (interrupt context) when an error occurs. */
    void (* errorCallback)(SentDiagnostic_t);
} SentConfiguration_t;

/** Initialize the PPM hardware for SENT receiver mode.
 *
 * This function will stop and re-initialize the PPM hardware.
 *
 * @attention This function must be called in system mode to access interrupt configuration ports.
 *
 * @param sentConfigurationPtr A pointer to the #SentConfiguration_t to be used.
 */
void sentReceiveInit(const SentConfiguration_t* const sentConfigurationPtr);

/** Start receiving SENT frames.
 *
 * This function enables the PPM hardware.
 *
 * @note This function assumes that the PPM hardware is already initialized using #sentReceiveInit.
 */
void sentReceiveStart(void);

/** Immediately stop the SENT component and the PPM hardware.
 *
 * This disables the PPM interrupt and disables the PPM hardware.
 *
 * @attention This function must be called in system mode to access interrupt configuration ports.
 *
 */
void sentStop(void);

/** Verify the configuration of the PPM hardware when configured as SENT receiver.
 *
 * @note This function does not verify the **PPM_EN** field inside the **PPM_CTRL** port. This field changes value
 *       if the application calls the #sentReceiveStart or #sentStop functions.
 *
 * @param sentConfigurationPtr A pointer to the #SentConfiguration_t to be used.
 *
 * @retval true If the PPM hardware is configured correctly for SENT receiver mode.
 * @retval false Otherwise
 */
bool sentReceiveCheckHardwareConfiguration(const SentConfiguration_t* const sentConfigurationPtr);

/** Initialize the PPM hardware for SENT transmitter mode.
 *
 * This function will stop and re-initialize the PPM hardware. Then, messages can be transmitted with #sentTransmitMessage.
 *
 * @attention This function must be called in system mode to access interrupt configuration ports.
 *
 * @param sentConfigurationPtr A pointer to the #SentConfiguration_t to be used.
 */
void sentTransmitInit(const SentConfiguration_t* const sentConfigurationPtr);

/** Verify the configuration of the PPM hardware when configured as SENT transmitter.
 *
 * @note This function does not verify the **PPM_EN** field inside the **PPM_CTRL** port. This field changes value
 *       if the application calls the #sentReceiveStart or #sentStop functions.
 *
 * @param sentConfigurationPtr A pointer to the #SentConfiguration_t to be used.
 *
 * @retval true If the PPM hardware is configured correctly for SENT transmit mode.
 * @retval false Otherwise
 */
bool sentTransmitCheckHardwareConfiguration(const SentConfiguration_t* const sentConfigurationPtr);

/** Transmit a raw (unformatted) SENT message.
 *
 * @note This function assumes that the PPM hardware is already initialized using #sentReceiveInit.
 *
 * @param sentConfigurationPtr A pointer to the #SentConfiguration_t to be used.
 * @param message A pointer to the buffer storing the message to be transmitted.
 *
 * @return An enum of type #SentDiagnostic_t, with:
 *         - #SENT_MESSAGE_OK indicating transmission success,
 *         - all other values indicating transmission failure.
 */
SentDiagnostic_t sentTransmitMessage(const SentConfiguration_t* const sentConfigurationPtr,
                                     const SentMessage_t* const message
                                     );

/** SENT receiver mode interrupt handler.
 *
 * Each incoming SENT message is assumed to have a pause pulse. This pause pulse will cause a PPM time-out and trigger
 * the PPM \b ppm_err_it interrupt handler. This function shall used to handle this interrupt.
 *
 * @param sentConfigurationPtr A pointer to the #SentConfiguration_t to be used.
 *
 * @note The #SentConfiguration_t.errorCallback can be used to catch errors that occur while receiving SENT messages.
 */
void sentReceiveInterruptHandler(const SentConfiguration_t* const sentConfigurationPtr);

/** SENT transmitter mode interrupt handler.
 *
 * Each outgoing SENT message is encoded with a pause pulse at the end. This pause pulse will cause a PPM time-out and
 * trigger the PPM \b ppm_err_it interrupt handler. This function shall used to handle this interrupt.
 *
 * @param sentConfigurationPtr A pointer to the #SentConfiguration_t to be used.
 */
void sentTransmitInterruptHandler(const SentConfiguration_t* const sentConfigurationPtr);

#endif /* define SENT_HAL_H */
/// @}
