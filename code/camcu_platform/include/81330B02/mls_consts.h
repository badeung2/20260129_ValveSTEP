/**
 * @file
 * @brief Constants definition
 * @internal
 *
 * @copyright (C) 2008-2018 Melexis N.V.
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
 * @details Constants used to communicate with LIN Slave module.
 */

#ifndef MLS_CONSTS_H_
#define MLS_CONSTS_H_

/* TODO: issue #9: the ML_HAS_ONLY_FAST_LIN options is always disabled */
#define ML_HAS_ONLY_FAST_LIN   0

#define ML_MRF_ID       0x3Cu           /**< ID of Master Request Frame */
#define ML_SRF_ID       0x3Du           /**< ID of Slave Response Frame */

#define ML_MRF_INDEX    0x10u           /**< index of Master Request Frame */
#define ML_SRF_INDEX    0x11u           /**< index of Slave Response Frame */

/** Aliases for compatibility with ancient versions */
#define D_DIA           ML_MRF_INDEX    /**< Demand diagnostic   */
#define R_DIA           ML_SRF_INDEX    /**< Response diagnostic */

/** Checksum Type */
#define chk13           0 /**< Classical checksum */
#define chk20           1 /**< Enhanced checksum */

/** ----------------------------------------------------------------------------
 * Possible values for LINcmnd, the primary command code for LIN:
 */
#define pcNONE      0   /**< No commands in the buffer                            */
#define pcSTCH      1   /**< General command for LIN state changes                */
#define pcCNFBR     2   /**< Set the target baudrate mode/prescaler value         */
#define pcSLEEPTO   3   /**< Set the sleep timeout due to bus inactivity          */
#define pcNotUsed2  4   /**< Not used                                             */
#define pcSETFRAME  5   /**< Configure the frame setting register                 */
#define pcCNFID     6   /**< Configure a LIN ID                                   */
#define pcCNFSR     7   /**< Configure the Slew Rate                              */
#define pcSFTVER    10  /**< Software version request                             */
#define pcOPTION    11  /**< Set the option registers                             */
#define pcGETST     12  /**< Get the state of the Mlx4                            */
#define pcCNFAUTO   13  /**< Configure the auto addressing mode                   */
#define pcNotUsed3  14  /**< Not used                                             */
#define pcRELBUF    15  /**< Release frame buffer                                 */

/** ----------------------------------------------------------------------------
 * Possible values for LINmess, the message sent to the MLX16
 */
#define evNONE      0   /**< no events available at the moment (sync error) */
#define evSTCH      1   /**< state of the LIN task has changed              */
#define evERR       2   /**< error detected by the LIN task                 */
#define evMESSrcvd  3   /**< Message received (data is available)           */
#define evMESSrqst  4   /**< Message request (ID received, TX message)      */
#define evENDtx     5   /**< EndEvent : end of a transmit                   */
#define evCOOLAUTO  7   /**< Cooling Auto-Addressing pulse                  */

/** ----------------------------------------------------------------------------
 * Possible values for LINstate, the state of the LIN firmware (pcGETST)
 */
#define stINIT      0   /**< INIT state (before getting to DISCONNECTED */
#define stDISC      1   /**< DISCONNECTED state                         */
#define stACT       2   /**< ACTIVE state                               */
#define stSLEEP     4   /**< SLEEP state                                */
#define stWKUP      8   /**< WAKE-UP state                              */
#define stSHORT     14  /**< SHORT state (LIN bus dominant)			    */
#define stFAST      15  /**< FAST mode                                  */

/** ----------------------------------------------------------------------------
 * Possible values for LINstate parameters (evSTCH)
 */
#define slMST       1   /**< Sleep State entered with a master command		                                */
#define slAPP       2   /**< Sleep State entered with an application command		                        */
#define slWKUP      3   /**< Leaving the Sleep State because a wake-up pulse was detected	                */
#define slWKUPabort 4   /**< Sleep State entered because there was no response to a wakeup                  */
#define slWKUPresp  5   /**< Entering Active state by response to a wake-up pulse detected                  */
#define slWKUPbreak 8   /**< Entering Active state by dominant state detected while sending a wake-up pulse */
#define slTO        6   /**< Sleep State entered with a timeout with LIN bus recessive                      */
#define slTODOM     7   /**< Sleep State entered with a timeout with LIN bus dominant	                    */
#define slREG       10  /**< Entering Active state by any other reason                                      */

/** ----------------------------------------------------------------------------
 * Possible values of secondary event codes for LIN errors (evERR)
 */
#define erNOERR     0   /**< This error should not be generated                                       */
#define erSHORTDONE 1   /**< A short state has been seen on the bus, but now recovered                */
#define erCRASH     2   /**< LIN task crashed, trying to reboot                                       */
#define erIDPAR     3   /**< Parity error in ID field received                                        */
#define erCKSUM     4   /**< Checksum error in message received                                       */
#define erTXCOL     5   /**< Data collision during the transmit cycle                                 */
#define erRX        6   /**< Stop or Start bit error while receiving data                             */
#define erIDSTOP    7   /**< Stop bit error of the ID field                                           */
#define erSYNC      8   /**< Sync field timing error                                                  */
#define erRXOVR     9   /**< Message received but buffer was full                                     */
#define erSHORT     10  /**< Short detected on the LIN bus                                            */
#define erTORESP    11  /**< Response timeout (RX timeout is checked only in 1.3 mode)                */
#define erBRFRM     12  /**< A header has been detected inside a frame                                */
#define erWKUPINIT  15  /**< A valid wakeup pulse has been seen while waiting to enter in sleep state */

/** ----------------------------------------------------------------------------
 * Possible values of secondary error events
 */
#define erSYNClo    1   /**< Baudrate detected is lower than requested one (in FABR or FBR modes)  */
#define erSYNChi    2   /**< Baudrate detected is higher than requested one (in FABR or FBR modes) */
#define erSYNCsbhi  3   /**< SYNC field bit 7 too long (it's probably a dominant stop bit)         */
#define erSYNCbhi   4   /**< SYNC field byte incorrect (only sent in FBR mode for JAE2602)         */
#define erSYNChead  5   /**< The break + delimiter are too long (more than 28 Tbit)                */

#define erCRASHIT   0   /**< Mlx4 issued an interruption (interruptions should be disabled)        */
#define erCRASHPLL  1   /**< The PLL is not running                                                */
#define erCRASHTX   2   /**< The propagation delay between TX and RX is too long                   */

#define erRXSTART   1   /**< Stop bit error (regular data byte or checksum)                        */
#define erRXSTOP    2   /**< Start bit recessive at 1/2 Tbit level check                           */

/** ----------------------------------------------------------------------------
 * Possible values of secondary pcSFTVER command
 */
#define verLIN      1   /**< LIN Software version         */
#define verEDIT     2   /**< LIN Software edition version */

#endif /* LIN_CONSTS_H_ */
