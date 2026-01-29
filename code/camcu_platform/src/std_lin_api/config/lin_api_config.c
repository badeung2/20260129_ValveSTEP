/**
 * @file lin_api_config.c
 * @brief STD LIN API configurations
 * @internal
 *
 * @copyright (C) 2018 Melexis N.V.
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
 * @ingroup user_startup
 *
 * @details This file contains configurations for STD LIN API.
 * @note The file can be replaced by application specific from application folder.
 */

#include "lin_api.h"
#include "fw_mls_api.h"
#include "lib_miscio.h"

#if LIN_SLAVE_API == 1


/*-----------------------------------------------------------------------------
 * Function prototypes
 */

static void l_mls_InitEventTable(void);


/*-----------------------------------------------------------------------------
 * Functions
 */

/** Initializes MLX LIN API.
 * Main function to intitialize MLX LIN API called by STD LIN API.
 * @return void
 */
void l_mls_Init(void)
{
#if ML_MLX4_VERSION >= 412
    /* Supported only with MLX4 >= 4.1.2 */
    const l_u8 l_s_ver = (SL_vLIN_1_3 ? ML_VER_1_3 : ML_VER_2_X);
#else /* ML_MLX4_VERSION >= 412 */
    const l_u8 l_s_ver = ML_VER_DEFAULT;
#endif /* ML_MLX4_VERSION >= 412 */

    /* Start and initialize the LIN Module
     * Note: MLX LIN API shall be initialized first with proper settings and only TL then */
    if (ml_Init(SL_LIN_AUTOBAUDRATE_MODE, SL_LIN_BAUDRATE, l_s_ver, MLX4_FPLL) == ML_SUCCESS) {
        /* Initialize MLX LIN API event table */
        l_mls_InitEventTable();
    }
}


/** Initializes MLX LIN Slave API event table.
 * Subscribe handlers for MLX LIN API events.
 * @return void
 */
static void l_mls_InitEventTable(void)
{
    /* General events processing */
    plinEventTable->mlu_LinSleepMode = l_LinSleepMode;
    plinEventTable->mlu_AutoAddressingStep = l_AutoAddressingStep;

    /* Non-diagnostic frames are processed always by std. LIN API when it's initialized
     * The std. LIN API uses event from LIN transport layer */
    plinEventTable->ldt_NonDiagMessageReceived = l_MessageReceived;
    plinEventTable->ldt_NonDiagDataRequest = l_DataRequest;
    plinEventTable->ldt_NonDiagDataTransmitted = l_DataTransmitted;

    /* Subscribe to errors reporting */
    uint8_t* idx = &ld_ErrorDetectedMultipleHandler.eventsCount;
    if (*idx < ld_ErrorDetectedMultipleHandler.eventsMaxLength) {
        ld_ErrorDetectedMultipleHandler.eventHandlerBuffer[*idx] = l_ErrorDetected;
        *idx = *idx + 1;
    }

#if SL_vLIN_2_x || SL_vISO17987_2016 || SL_vSAE_J2602_2012
    /* Diagnostic frames: node configuration / identification */
    (void)ldt_SubscribeMultipleHandler(plinEventTable->ldt_Request, ld_DiagRequest);
    plinEventTable->ldt_GetLINNAD = ld_GetLINNAD;
#endif /* SL_vLIN_2_x || SL_vISO17987_2016 || SL_vSAE_J2602_2012 */
}

#endif /* LIN_SLAVE_API */

#if LIN_HAS_DEFAULT_IRQ_CALLOUTS == 1
/** Disables all interrupts which are accessing the LIN API resources.
 * @return CPU priority level before the function call.
 */
l_irqmask l_sys_irq_disable(void)
{
    l_u16 mreg_saved = builtin_mlx16_get_status();

    /* Keep User-bit unchanged */
    builtin_mlx16_set_status((mreg_saved & ~((l_u16)7u << 8)) |
                             (((l_u16)LINAPI_RESOURCES_BLOCKING_PRIORITY & 7u) << 8));
    return mreg_saved;
}

/** Restores CPU priority level
 * @param[in] previous CPU priority level to be restored.
 * @return void
 */
void l_sys_irq_restore(l_irqmask previous)
{
    builtin_mlx16_set_status(previous);
}
#endif /* LIN_HAS_DEFAULT_IRQ_CALLOUTS */

/* EOF */
