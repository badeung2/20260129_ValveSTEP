/**
 * @file mls_event_table.h
 * @brief The RAM based table of LIN events
 * @internal
 *
 * @copyright (C) 2016-2018 Melexis N.V.
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
 * @ingroup mlx_lin_slave_api
 *
 * @details The RAM based pointer table provides flexible way to subscribe to LIN events by changing the pointer.
 */

#ifdef UNITTEST
#include "mls_api.h"
#endif

#ifndef MLS_API_H_
#error "The file can't be included directly but via mls_api.h"
#endif /* MLS_API_H_ */

#ifndef MLS_EVENT_TABLE_H_
#define MLS_EVENT_TABLE_H_


/*-----------------------------------------------------------------------------
 * Typedefs                             (scope: global)
 */

/*! Type definition of LIN event table.
 * Application and loader will subscribe there LIN event handlers here.
 * By this mechanism the LIN events can be redirected to the application or to the loader.
 * Only one instance of the LIN stack is needed.
 * Add the handlers before starting the MLX4. Default value is null pointer.
 */
typedef struct LINEventTable_s {
    uint16_t nrOfFunctions; /*!< number of functions exported in the LINEventTable */

    /*
     * Melexis LIN Hardware abstraction layer
     */

    /*!<  Is called when the data of a relevant M2S frame was received.
     * Release the ML_DATA_LIN_FRAME_DATA_BUFFER after extracting the data.
     * Calls the lin diagnosis transfer layer.
     * Application should overwrite ldt_NonDiagMessageReceived to handle non diagnosis frames.
     * \param[in]  ml_MessageID_t Index
     * \param[in] uint8_t ML_DATA_LIN_FRAME_DATA_BUFFER[8]
     */
    void (*mlu_MessageReceived)(ml_MessageID_t index);

    /*!< Is called when a relevant S2M frame was received and data must be added to ML_DATA_LIN_FRAME_DATA_BUFFER.
     * Calls the lin diagnosis transfer layer.
     * Application should overwrite ldt_NonDiagDataRequest to handle non diagnosis frames.
     * \param[in]  ml_MessageID_t Index frame index
     * \param[out] uint8_t ML_DATA_LIN_FRAME_DATA_BUFFER[8]
     */
    void (*mlu_DataRequest)(ml_MessageID_t index);

    /*!< Is called when an error is detected in the MLX4 LIN stack.
     * Default function in LIN stack releases the lin frame buffer.
     * When LIN TL is enable the ldt_ErrorDetected() is called instead.
     * Application should overwrite this event.
     * \param[in]  ml_LinError_t Error
     */
    void (*mlu_ErrorDetected)(ml_LinError_t error);

    /*!< Is called if the transmission that was started with ml_DataReady() has finished.
     * ml_DataReady must be called with the parameter ML_END_OF_TX_ENABLED to enable this event.
     * Calls the lin diagnosis transfer layer.
     * Application should overwrite ldt_NonDiagDataTransmitted to handle non diagnosis frames.
     * \param[in]  ml_MessageID_t Index frame index
     */
    void (*mlu_DataTransmitted)(ml_MessageID_t index);

    /*!< Function is called when mlx16 should enter sleep after 3C 0 0 diagnosis frames or 4s timeout.
     * Default is to enter sleep but.
     * Application should overwrite this event.
     * \param[in]  ml_SleepReason_t Reason
     */
    void (*mlu_LinSleepMode)(ml_SleepReason_t reason);

    /*!< Cooling Auto-Addressing pulse
     * \param[in] uint8_t StepNumber
     */
    void (*mlu_AutoAddressingStep)(uint8_t stepNumber);

#if ((LDT_HAS_TRANSPORT_LAYER == 1) || (ML_HAS_LIN_EVENT_TABLE_IN_RAM == 1))

    /*
     * LIN diagnosis transfer layer
     */

    /*!<  Is called when a non diagnostic M2S frame(FrameID != 0x3C) was received.
     * Application should overwrite this event.
     * \param[in]  ml_MessageID_t Index
     * \param[in] uint8_t ML_DATA_LIN_FRAME_DATA_BUFFER[8]
     */
    void (*ldt_NonDiagMessageReceived)(ml_MessageID_t index);

    /*!<  Is called when
     * a non diagnostic S2M frame was transmitted.
     * Application should overwrite this event.
     */
    void (*ldt_NonDiagDataTransmitted)(ml_MessageID_t index);

    /*!< Is called when
     * a non diagnostic S2M frame(FrameID != 0x3D) was received.
     * Application should overwrite this event.
     * \param[in]  ml_MessageID_t Index
     * \param[out] uint8_t ML_DATA_LIN_FRAME_DATA_BUFFER[8]
     */
    void (*ldt_NonDiagDataRequest)(ml_MessageID_t index);

    /*!< Returns configured or initial NADs.
     * Despite the parameter has been added to support two NADs: initial (for Assign NAD)
     * and configured (for all other services), the Standard exclude the situation when both
     * NADs can be supported simultaneously (LIN22A: 4.2.2.1)
     *
     * The parameter still can be useful to notify the LIN TL user about all requested NADs on the LIN bus.
     * The parameter allows to catch even all LIN frames on the LIN bus. That enables the LIN Gateway application.
     */
    uint8_t (*ldt_GetLINNAD)(uint8_t nad);

    /*!< Is called when an error is detected in the MLX4 LIN stack.
     * When LIN TL is disabled the ml_ErrorDetected() is called instead.
     * Application should overwrite this event.
     * \param[in]  ml_LinError_t Error
     */
    void (*ldt_ErrorDetected)(ml_LinError_t error);

    /*
     * Multiple stages handlers
     */

    /*!< Is called if an LIN diagnosis functional (NAD=0x7E) request was received.
     * \param[in] LINDiagRequest_t *request Request buffer with the transmitted data.
     * \return status:
     * - true - some event handler blocks the next handler call in the list by returning the true;
     * - false - all event handlers in the list return false.
     */
    ldt_MultipleHandler_t *ldt_FunctionalRequest;

    /*!< Is called if an LIN diagnosis request was received.
     * \param[in] LINDiagTransfer_t *transfer Response buffer with the transmitted data.
     * \return status:
     * - true - transfer->response buffer is filled and should be transmitted;
     * - false - send no response.
     */
    ldt_MultipleHandler_t *ldt_Request;

    /* Is called if an LIN diagnosis response was
     * transmitted. The transmitted response is submitted as parameter.
     * \param[in] LINDiagResponse_t *response Response buffer with the transmitted data.
     * \return status:
     * - true - some event handler blocks the next handler call in the list by returning the true;
     * - false - all event handlers in the list return false.
     */
    ldt_MultipleHandler_t *ldt_ResponseTransmitted;
#endif /* ((LDT_HAS_TRANSPORT_LAYER == 1) || (ML_HAS_LIN_EVENT_TABLE_IN_RAM == 1)) */
} LINEventTable_t;


/*-----------------------------------------------------------------------------
 * Declaration of variables             (scope: global)
 */

/*! LIN event table. Application and loader will subscribe there LIN event handlers here.
 * Add the handlers before starting the MLX4. Default value is null pointer.
 */
#if ML_HAS_LIN_EVENT_TABLE_IN_RAM == 1
extern ML_VAR_ATTR LINEventTable_t linEventTable;
#define plinEventTable ((LINEventTable_t *)&linEventTable)
#else /* ML_HAS_LIN_EVENT_TABLE_IN_RAM */
extern ML_VAR_ATTR LINEventTable_t* plinEventTable;
#endif /* ML_HAS_LIN_EVENT_TABLE_IN_RAM */

#endif /* MLS_EVENT_TABLE_H_ */
