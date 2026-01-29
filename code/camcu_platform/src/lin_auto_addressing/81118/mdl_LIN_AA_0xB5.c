/*
 * Copyright (C) 2021 Melexis GmbH
 *
 * This file is part of the Mlx81118 module library.
 *
 * File mdl_LIN_AA_0xB5.c
 *
 * Module prefix: mdl_LIN_AA
 * All module interface functions start with this letters.
 *
 * LIN auto addressing module. Includes the sequence for the 0xB5 Service for
 * the LIN auto addressing algorithm according to the BSM.
 *
 * Revision: 2.1.0
 * Date:     04.05.2021
 *
 *
 * ==========================================================================
 * History:
 *   Revision 1.0.0
 *     - Initial release
 *   Revision 2.0.0
 *     - Using MLX LIN TL
 *   Revision 2.1.0
 *     - changed "twisted LINAA connection" handling in case of intermediate
 *       LIN messages
 *
 * ========================================================================== */

#include <plib.h>
#include <lin_api.h>
#include <lin_core_sl.h>
#include "mdl_LIN_AA.h"


/** LIN AA SID=0xB5 diagnostic frames processing.
 * Proceed LIN AA 0xB5 configuration frames: sub functions 1..4.
 */
bool mdl_LIN_AA_proceed_0xB5_diagframe(LINDiagTransfer_t *transfer) {
    if (   (transfer->request.dataLen == 0x5u)  \
           && (transfer->request.data[0] == 0xFFu) \
           && (transfer->request.data[1] == 0x7Fu) \
           && (transfer->request.data[3] == 0x02u)) {

        switch(transfer->request.data[2]) {
            case LIN_AA_SUBFUNCTION1: /* SNPD sub function ID 0x01 "All BSM-nodes enter the unconfigured mode" */
                LIN_AA_AutoAddressingFlags = LIN_AA_AUTOADDRESSENABLE;
                break;

            case LIN_AA_SUBFUNCTION2: /* SNPD sub function ID 0x02 "Informing all slaves about the next NAD" */
                if((LIN_AA_AutoAddressingFlags & LIN_AA_LASTSLAVE) == LIN_AA_LASTSLAVE) {

                    ml_ConfiguredNAD = transfer->request.data[4];   /* use the new NAD */

                    mdl_LIN_AA_set_AutoAddressingFlags(LIN_AA_SLAVEADDRESSED);

                    mdl_LIN_AA_reset_AutoAddressingFlags(LIN_AA_LASTSLAVE);
                }
                if((LIN_AA_AutoAddressingFlags & LIN_AA_INVALID_HW_DETECT) == LIN_AA_INVALID_HW_DETECT){
                    mdl_LIN_AA_set_AutoAddressingFlags(LIN_AA_INVALID_HW_CONNECT);
                }
                break;

            case LIN_AA_SUBFUNCTION3: /* SNPD sub function ID 0x03 "Store the assigned NADs in to the NVM of the slaves, if available" */
                s_ifcStatus.mapped.SaveConfig = true;
                break;

            case LIN_AA_SUBFUNCTION4: /* SNPD sub function ID 0x04 "Informing all slaves that the procedure is finished" */
                mdl_LIN_AA_reset_AutoAddressingFlags(LIN_AA_RESETALLFLAGS);
                break;

            default:
                break;
        }
    }

    /* No response prepared */
    return false;
} /* mdl_LIN_AA_proceed_0xB5_diagframe */

/* EOF */
