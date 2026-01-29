/*!*************************************************************************** *
 * \file        ioports_MLX81160.h
 * \brief       MLX81160 I/O-ports-map
 *
 * \note        project MLX81160
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     1.0 - preliminary
 *
 *
 * \copyright   MELEXIS Microelectronic Integrated Systems
 *
 * Copyright (C) 2017-2022 Melexis N.V.
 * The Software is being delivered 'AS IS' and Melexis, whether explicitly or
 * implicitly, makes no warranty as to its Use or performance.
 * The user accepts the Melexis Firmware License Agreement.
 *
 * Melexis confidential & proprietary
 *
 * @attention Important note: I/O-portnames adding with "_S" are system ports,
 * and can therefore only be written in System-mode of the MLX16 CPU.
 * *************************************************************************** */

/*!*************************************************************************** *
 *                              I N C L U D E S                                *
 * *************************************************************************** */
#include <syslib.h>

#if defined (__MLX81160__)

#ifndef __MLX81160A01__
#error "Wrong ioports.h file"
#endif /* __MLX81160A01__ */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */


/* *************** */
/* Block: RST_CTRL */
/* *************** */
extern volatile uint16_t IO_RST_CTRL_S __attribute__((io, addr(0x02)));         /*!< IO_RST_CTRL_S (System) */
#define B_RST_CTRL_HVDIG_OK                     (1U << 15)                      /*!< R: 1: High Voltage Digital Okay 0: High Voltage Digital not Okay */
#define B_RST_CTRL_HVDIG_USED                   (1U << 14)                      /*!< RW: 0: No high-voltage digital part used 1: High-voltage digital part used */
#define B_RST_CTRL_SOFT_RESET                   (1U << 13)                      /*!< RW: 0: No request to reset hardware 1: Request to reset hardware (automatically cleared) */
#define B_RST_CTRL_IO5_WBOOT                    (1U << 5)                       /*!< R: Warm boot reset: 1: Due to IO5 source 0: Not due to IO5 source; W: 1: Clear bit 0: No effect */
#define B_RST_CTRL_IWD_WBOOT                    (1U << 4)                       /*!< R: Warm boot reset: 1: Due to this source 0: Not due to this source; W: 1: Clear bit 0: No effect */
#define B_RST_CTRL_DBG_WBOOT                    (1U << 3)                       /*!< R: Warm boot reset: 1: Due to debugger 0: Not due to debugger; W: 1: Clear bit 0: No effect */
#define B_RST_CTRL_HVDIG_WBOOT                  (1U << 2)                       /*!< R: Warm boot reset: 1: Due to High Voltage Digital lost (HVDIG_OK = 0 and HVDIG_USED = 1) 0: Not due to High Voltage Digital; W: 1: Clear bit 0: No effect */
#define B_RST_CTRL_SOFT_WBOOT                   (1U << 1)                       /*!< R: Warm boot reset: 1: Due to SOFT_RESET 0: Not due to SOFT_RESET; W: 1: Clear bit 0: No effect */
#define B_RST_CTRL_AWD_WBOOT                    (1U << 0)                       /*!< R: Warm boot reset: 1: Due to Absolute Watchdog 0: Not due to Absolute Watchdog; W: 1: Clear bit 0: No effect */

/* **************** */
/* Block: FUNC_TEST */
/* **************** */
extern volatile uint16_t IO_FUNC_TEST_S __attribute__((io, addr(0x06)));        /*!< IO_FUNC_TEST_S (System) */
#define M_FUNC_TEST_FTC_DIV                     (15U << 10)                     /*!< RW: ??? */
#define M_FUNC_TEST_FTC_SEL                     (3U << 8)                       /*!< RW: See below
                                                                                 * 0x: FT_APP_GCLK = APP_CLK
                                                                                 * 10: FT_APP_GCLK = TAP_CLK
                                                                                 * 11: FT_APP_GCLK = OPCG_APP_CLK */
#define B_FUNC_TEST_DISABLE_IRQ                 (1U << 2)                       /*!< RW: 0: All interrupts working as defined by application 1: All interrupts masked */
#define B_FUNC_TEST_DISABLE_SUSPEND             (1U << 1)                       /*!< RW: 0: Suspend enabled 1: Suspend disabled */
#define B_FUNC_TEST_DISABLE_RESET               (1U << 0)                       /*!< RW: 0: Application warm resets enabled 1: Application warm resets disabled */

extern volatile uint16_t IO_FUNC_TEST_UNPROT_S __attribute__((io, addr(0x08)));  /*!< IO_FUNC_TEST_UNPROT_S (System) */
#define M_FUNC_TEST_TEST_UNPROT                 (65535U << 0)                   /*!< R: 0x0000 if protected (no valid key has been entered) 0x0001 if not protected (a valid key has been entered); W: First write 0x668D, second write 0x6B21 */

/* ************ */
/* Block: MUPET */
/* ************ */
extern volatile uint16_t IO_MUPET_SR __attribute__((io, addr(0x0A)));           /*!< IO_MUPET_SR */
#define M_MUPET_PTC_SEND                        (65535U << 0)                   /*!< W: Data to be send at next PTCR instruction; R: if PTCA_PEND = 1: 0x0, ID[11:0] (Received from PTCA instruction) If PTCS_PEND = 1: DATA[15:0] (Received from PTCS instruction) */
#define M_MUPET_PTC_RECEIVE                     (65535U << 0)                   /*!< R: if PTCA_PEND = 1: 0x0, ID[11:0] (Received from PTCA instruction) If PTCS_PEND = 1: DATA[15:0] (Received from PTCS instruction) */

extern volatile uint16_t IO_MUPET_CTRL __attribute__((io, addr(0x0C)));         /*!< IO_MUPET_CTRL */
#define B_MUPET_IN_APPLICATION                  (1U << 15)                      /*!< W: 0: Read of this bit bit gives MUPeT activation status 1: Read of this bits returns 1; R: High if MUPET has not been activated at power on reset or if this bit has been written to 1 */
#define M_MUPET_CONNECTION                      (3U << 13)                      /*!< R: 01: MUPeT is connected via interface 1
                                                                                 *      10: MUPeT is connected via interface 2 */
#define M_MUPET_WARM_TRIGGER                    (31U << 8)                      /*!< RW: 0x07: Warm Trigger 1
                                                                                 *       0x19: Warm Trigger 2
                                                                                 *       Any other value: No Warm Trigger request */
#define B_MUPET_PTC_KEY                         (1U << 7)                       /*!< R: 0: PTC key has not been sent 1: PTC key has been sent */
#define B_MUPET_PTCR_PEND                       (1U << 2)                       /*!< R: PTCR request pending: Set by PTCR instruction received Cleared by writing to PTC_SEND */
#define B_MUPET_PTCS_PEND                       (1U << 1)                       /*!< R: PTCS request pending: Set by PTCS instruction received Cleared by writing to PTC_SEND */
#define B_MUPET_PTCA_PEND                       (1U << 0)                       /*!< R: PTCA request pending: Set by PTCA instruction received Cleared by writing to PTC_SEND */

/* ********** */
/* Block: AWD */
/* ********** */
extern volatile uint8_t IO_AWD_WIN __attribute__((io, addr(0x0F)));             /*!< IO_AWD_WIN */
#define B_AWD_WIN_OPEN                          (1U << 7)                       /*!< R: 1 = Window is open, software can acknowledge the watchdog 0 = Window is closed; W: No effect */
extern volatile uint8_t IO_AWD_ACK __attribute__((io, addr(0x0E)));             /*!< IO_AWD_ACK */
#define B_AWD_ACK                               (1U << 6)                       /*!< R: Always read 0; W: 1 = Acknowledge the watchdog if the window is open, else generate a CPU reset 0 = No effect */

/* **************** */
/* Block: ROM_SHELL */
/* **************** */
extern volatile uint16_t IO_ROM_SHELL_READY __attribute__((io, addr(0x10)));    /*!< IO_ROM_SHELL_READY */
#define M_ROM_SHELL_READY                       (31U << 0)                      /*!< RW: Number of clock pulses between the access start and the READY rising edge */

extern volatile uint16_t IO_ROM_SHELL_PWR __attribute__((io, addr(0x12)));      /*!< IO_ROM_SHELL_PWR */
#define B_ROM_SHELL_ROM_WREN                    (1U << 1)                       /*!< RW: 1: Allow Writes to "ROM" when mapped to FPGA this bit is only available when the XILINX_SYNTHESIS macro is defined */
#define B_ROM_SHELL_DISABLE_MEM                 (1U << 0)                       /*!< RW: If this port field is set to one, then the memory is forced In a low power consumption mode */

/* *************** */
/* Block: ROM_BIST */
/* *************** */
extern volatile uint16_t IO_ROM_BIST_ADD_START_L __attribute__((io, addr(0x14)));  /*!< IO_ROM_BIST_ADD_START_L */
#define M_ROM_BIST_ADD_START_L                  (65535U << 0)                   /*!< W: Start address of the BIST (16 LSB); Note: Address has to be aligned on a word 32 bits (2 LSB are ignored); R: Current value of the start address pointer (16 LSB) */

extern volatile uint16_t IO_ROM_BIST_ADD_START_H __attribute__((io, addr(0x16)));  /*!< IO_ROM_BIST_ADD_START_H */
#define M_ROM_BIST_ADD_START_H                  (15U << 0)                      /*!< W: Start address of the BIST (4 MSB); R: Current value of the start address pointer (4 MSB) */

extern volatile uint16_t IO_ROM_BIST_ADD_STOP_L __attribute__((io, addr(0x18)));  /*!< IO_ROM_BIST_ADD_STOP_L */
#define M_ROM_BIST_ADD_STOP_L                   (65535U << 0)                   /*!< W: Stop address of the BIST (16 LSB); R: Current value of the stop address pointer (16 LSB); Note: LSB is always equal to 1 */

extern volatile uint16_t IO_ROM_BIST_ADD_STOP_H __attribute__((io, addr(0x1A)));  /*!< IO_ROM_BIST_ADD_STOP_H */
#define M_ROM_BIST_ADD_STOP_H                   (15U << 0)                      /*!< W: Stop address of the BIST (4 MSB); R: Current value of the stop address pointer (4MSB) */

extern volatile uint16_t IO_ROM_BIST_SIG_EXPECTED_L __attribute__((io, addr(0x1C)));  /*!< IO_ROM_BIST_SIG_EXPECTED_L */
#define M_ROM_BIST_SIG_EXPECTED_L               (65535U << 0)                   /*!< RW: Signature expected at the end of the BIST (16 LSB) */

extern volatile uint16_t IO_ROM_BIST_SIG_EXPECTED_H __attribute__((io, addr(0x1E)));  /*!< IO_ROM_BIST_SIG_EXPECTED_H */
#define M_ROM_BIST_SIG_EXPECTED_H               (255U << 0)                     /*!< RW: Signature expected at the end of the BIST (8 MSB) */

extern volatile uint16_t IO_ROM_BIST_CTRL __attribute__((io, addr(0x20)));      /*!< IO_ROM_BIST_CTRL */
#define B_ROM_BIST_COMPLETED                    (1U << 15)                      /*!< R: 1 = BIST completed; 0 = BIST in progress */
#define B_ROM_BIST_VALID_CLOCK                  (1U << 11)                      /*!< R: 1 = Application clock is valid; 0 = Application clock is invalid */
#define B_ROM_BIST_BIST_REQUEST                 (1U << 9)                       /*!< R: 1 = BIST requested; 0 = No BIST requested */
#define B_ROM_BIST_MASK_SIG_ERR                 (1U << 6)                       /*!< RW: 1 = Mask the error signal at the end of BIST if the signature expected is different to the signature calculated; 0 = Do not mask the error signal at the end of BIST if the signature expected is different to the signature calculated */
#define B_ROM_BIST_SINGLE_RAMP                  (1U << 5)                       /*!< RW: 1 = Monotonic up BIST (only one pointer from add start to add stop used); 0 = Alternative BIST (two pointers alternating used) */
#define B_ROM_BIST_BIST                         (1U << 4)                       /*!< RW: 1 = Test ECC + non corrected values; 0 = Test corrected values */
#define M_ROM_BIST_ECC_POSITION                 (3U << 0)                       /*!< RW: Indicates all how many words an ECC is present
                                                                                 * 00 : Every word
                                                                                 * 01 : Every 2 words
                                                                                 * 10 : Every 4 words
                                                                                 * 11 : Every 8 words */

extern volatile uint16_t IO_ROM_BIST_START_BIST __attribute__((io, addr(0x22)));  /*!< IO_ROM_BIST_START_BIST */
#define M_ROM_BIST_START_BIST                   (65535U << 0)                   /*!< W: BIST request : KEY = 0x11EB; R: Always read 0 */

extern volatile uint16_t IO_ROM_BIST_SIG_RECEIVED_L __attribute__((io, addr(0x24)));  /*!< IO_ROM_BIST_SIG_RECEIVED_L */
#define M_ROM_BIST_SIG_RECEIVED_L               (65535U << 0)                   /*!< R: Signature calculated (16 LSB); W: Initialise the signature value (16 LSB) */

extern volatile uint16_t IO_ROM_BIST_SIG_RECEIVED_H __attribute__((io, addr(0x26)));  /*!< IO_ROM_BIST_SIG_RECEIVED_H */
#define M_ROM_BIST_SIG_RECEIVED_H               (255U << 0)                     /*!< R: Signature calculated (8 MSB); W: Initialise the signature value (8 MSB) */

/* **************** */
/* Block: RAM_SHELL */
/* **************** */
extern volatile uint16_t IO_RAM_SHELL __attribute__((io, addr(0x2A)));          /*!< IO_RAM_SHELL */
#define B_RAM_SHELL_DISABLE_MEM                 (1U << 0)                       /*!< RW: If this port field is set to one, then the memory is forced In a low power consumption mode */

/* ********** */
/* Block: IWD */
/* ********** */
extern volatile uint16_t IO_IWD_T __attribute__((io, addr(0x2C)));              /*!< IO_IWD_T */
extern volatile uint8_t IO_IWD_DIV __attribute__((io, addr(0x2D)));             /*!< IO_IWD_DIV */
extern volatile uint8_t IO_IWD_WDT __attribute__((io, addr(0x2C)));             /*!< IO_IWD_WDT */
#define M_IWD_DIV                               (7U << 8)                       /*!< RW: Counter clock: WDC_CLK = MCU_CLK / 2^(5+2*DIV) */
#define M_IWD_WDT                               (255U << 0)                     /*!< RW: Timeout */

extern volatile uint16_t IO_IWD_CTRL __attribute__((io, addr(0x2E)));           /*!< IO_IWD_CTRL */
extern volatile uint8_t IO_IWD_WIN __attribute__((io, addr(0x2F)));             /*!< IO_IWD_WIN */
extern volatile uint8_t IO_IWD_WTG __attribute__((io, addr(0x2E)));             /*!< IO_IWD_WTG */
#define B_IWD_WIN_OPEN                          (1U << 15)                      /*!< R: Window state: 0: Window is closed 1: Window is opened */
#define B_IWD_ATT_INT                           (1U << 14)                      /*!< R: 1 = Attention interrupt has been generated 0 = Attention interrupt has not been generated; W: 1 = Disable Window. Acknowledges can occur at any time 0 = No effect */
#define B_IWD_WIN_DISABLE                       (1U << 13)                      /*!< W: 1 = Disable Window. Acknowledges can occur at any time 0 = No effect; R: 1: Windowing disabled; 0: Windowing enabled */
#define B_IWD_WIN_ENABLE                        (1U << 12)                      /*!< W: 1 = Enable Window. Acknowledges have to occur when the window is open 0 = No effect; R: 1: Windowing enabled 0: Windowing disabled */
#define M_IWD_WTG                               (255U << 0)                     /*!< RW: Watchdog tag register */

/* ************* */
/* Block: STIMER */
/* ************* */
extern volatile uint16_t IO_STIMER_CNT __attribute__((io, addr(0x30)));         /*!< IO_STIMER_CNT */
#define M_STIMER_CURRENT                        (16383U << 0)                   /*!< R: Current counter value */

extern volatile uint16_t IO_STIMER_CTRL __attribute__((io, addr(0x32)));        /*!< IO_STIMER_CTRL */
#define M_STIMER_MODE                           (3U << 14)                      /*!< RW: See below */
#define C_STIMER_MODE_OFF                       (0U << 14)                      /*!< 00 = Disable */
#define C_STIMER_MODE_CPU                       (1U << 14)                      /*!< 01 = on CPU clock */
#define C_STIMER_MODE_1MHz                      (2U << 14)                      /*!< 10 = on ce_1us clock */
#define C_STIMER_MODE_10kHz                     (3U << 14)                      /*!< 11 = on ce_100us clock */
#define M_STIMER_VALUE                          (16383U << 0)                   /*!< RW: Period value */

/* *************** */
/* Block: TRIM_ADC */
/* *************** */
extern volatile uint16_t IO_TRIM_ADC_S __attribute__((io, addr(0x34)));         /*!< IO_TRIM_ADC_S (System) */
#define B_TRIM_ADC_LOCK                         (1U << 15)                      /*!< W: Lock the port. Write is invalid when set. Always read as 0.; R: Always read 0 */
#define B_TRIM_ADC_ADC_BLOCK_BYPASSBUF          (1U << 5)                       /*!< RW: Calibration. Write is invalid if LOCK is set */
#define B_TRIM_ADC_ADC_BLOCK_INTERCHOP          (1U << 4)                       /*!< RW: Calibration. Write is invalid if LOCK is set */
#define B_TRIM_ADC_ADC_BLOCK_LATCHCTRL          (1U << 3)                       /*!< RW: Calibration. Write is invalid if LOCK is set */
#define M_TRIM_ADC_ADC_INTREF_TRM               (7U << 0)                       /*!< RW: Calibration. Write is invalid if LOCK is set */

/* ******************** */
/* Block: PORT_LIN_XKEY */
/* ******************** */
extern volatile uint16_t IO_PORT_LIN_XKEY_S __attribute__((io, addr(0x36)));    /*!< IO_PORT_LIN_XKEY_S (System) */
#define M_PORT_LIN_XKEY_LIN_XKEY                (65535U << 0)                       /*!< RW: store a valid key for LIN XCFG */

/* ************ */
/* Block: MLX16 */
/* ************ */
extern volatile uint16_t IO_MLX16_DBG_DATA0 __attribute__((nodp, addr(0x00038)));  /*!< IO_MLX16_DBG_DATA0 */
#define M_MLX16_DBG_DATA0                       (65535U << 0)                   /*!< RW: Data for breakpoint Replacement instruction for patch Data send or receive from mark */

extern volatile uint16_t IO_MLX16_DBG_DATA1 __attribute__((nodp, addr(0x0003A)));  /*!< IO_MLX16_DBG_DATA1 */
#define M_MLX16_DBG_DATA1                       (65535U << 0)                   /*!< RW: Data for breakpoint Replacement instruction for patch Data send or receive from mark */

extern volatile uint16_t IO_MLX16_DBG_DATA2 __attribute__((nodp, addr(0x0003C)));  /*!< IO_MLX16_DBG_DATA2 */
#define M_MLX16_DBG_DATA2                       (65535U << 0)                   /*!< RW: Data for breakpoint Replacement instruction for patch Data send or receive from mark */

extern volatile uint16_t IO_MLX16_DBG_DATA3 __attribute__((nodp, addr(0x0003E)));  /*!< IO_MLX16_DBG_DATA3 */
#define M_MLX16_DBG_DATA3                       (65535U << 0)                   /*!< RW: Data for breakpoint Replacement instruction for patch Data send or receive from mark */

extern volatile uint16_t IO_MLX16_DBG_ADDRESS0_S __attribute__((nodp, addr(0x00040)));  /*!< IO_MLX16_DBG_ADDRESS0_S (System) */
#define M_MLX16_DBG_ADDRESS0                    (65535U << 0)                   /*!< RW: 16 LSB of breakpoint or patch address */

extern volatile uint16_t IO_MLX16_DBG_CTRL0_S __attribute__((nodp, addr(0x00042)));  /*!< IO_MLX16_DBG_CTRL0_S (System) */
#define B_MLX16_DBG_READY0                      (1U << 15)                      /*!< RW: 1 if the debugger or CPU has written the data register or if a breakpoint has been triggered */
#define B_MLX16_DBG_OVR0                        (1U << 14)                      /*!< RW: 1 If the data register is written (by the debugger or the CPU) wile READY is 1 */
#define B_MLX16_DBG_MARKEXT0                    (1U << 12)                      /*!< RW: 1 If external mark are allowed */
#define B_MLX16_DBG_LOCK0                       (1U << 11)                      /*!< R: 1 If the CPU cannot write in debug register */
#define M_MLX16_DBG_MODE0                       (3U << 9)                       /*!< RW: Usage of corresponding register:
                                                                                 * 00 Disable
                                                                                 * 01 Mark
                                                                                 * 10 Patch
                                                                                 * 11 Break */
#define B_MLX16_DBG_RANGE0                      (1U << 8)                       /*!< RW: 0 Use equality comparator 1: Use equal or greater (for register 0 and 2), lower (for register 1 and 2) */
#define M_MLX16_DBG_COND0                       (15U << 4)                      /*!< RW: Break condition:
                                                                                 * 0000 Disable
                                                                                 * 0001 Fetch
                                                                                 * 0010 Write word
                                                                                 * 0011 Write byte
                                                                                 * 0100 Read word
                                                                                 * 0101 Read byte
                                                                                 * 0110 Access word
                                                                                 * 0111 Access byte
                                                                                 * 1010 Write word value
                                                                                 * 1011 Write byte value
                                                                                 * 1100 Read word value
                                                                                 * 1101 Read byte value
                                                                                 * 1110 Access word value
                                                                                 * 1111 Access byte value */
#define M_MLX16_DBG_CONTROL0                    (65535U << 0)                   /*!< RW: Debugger control port */
#define M_MLX16_DBG_ADREXT0                     (15U << 0)                      /*!< RW: 4 MSB of breakpoint or patch address */

extern volatile uint16_t IO_MLX16_DBG_ADDRESS1_S __attribute__((nodp, addr(0x00044)));  /*!< IO_MLX16_DBG_ADDRESS1_S (System) */
#define M_MLX16_DBG_ADDRESS1                    (65535U << 0)                   /*!< RW: 16 LSB of breakpoint or patch address */

extern volatile uint16_t IO_MLX16_DBG_CTRL1_S __attribute__((nodp, addr(0x00046)));  /*!< IO_MLX16_DBG_CTRL1_S (System) */
#define B_MLX16_DBG_READY1                      (1U << 15)                      /*!< RW: 1 if the debugger or CPU has written the data register or if a breakpoint has been triggered */
#define B_MLX16_DBG_OVR1                        (1U << 14)                      /*!< RW: 1 If the data register is written (by the debugger or the CPU) wile READY is 1 */
#define B_MLX16_DBG_MARKEXT1                    (1U << 12)                      /*!< RW: 1 If external mark are allowed */
#define B_MLX16_DBG_LOCK1                       (1U << 11)                      /*!< R: 1 If the CPU cannot write in debug register */
#define M_MLX16_DBG_MODE1                       (3U << 9)                       /*!< RW: Usage of corresponding register: 00 Disable 01 Mark 10 Patch 11 Break */
#define B_MLX16_DBG_RANGE1                      (1U << 8)                       /*!< RW: 0 Use equality comparator 1: Use equal or greater (for register 0 and 2), lower (for register 1 and 2) */
#define M_MLX16_DBG_COND1                       (15U << 4)                      /*!< RW: Break condition: 0000 Disable 0001 Fetch 0010 Write word 0011 Write byte 0100 Read word 0101 Read byte 0110 Access word 0111 Access byte 1010 Write word value 1011 Write byte value 1100 Read word value 1101 Read byte value 1110 Access word value 1111 Access byte value */
#define M_MLX16_DBG_CONTROL1                    (65535U << 0)                   /*!< RW: Debugger control port */
#define M_MLX16_DBG_ADREXT1                     (15U << 0)                      /*!< RW: 4 MSB of breakpoint or patch address */

extern volatile uint16_t IO_MLX16_DBG_ADDRESS2_S __attribute__((nodp, addr(0x00048)));  /*!< IO_MLX16_DBG_ADDRESS2_S (System) */
#define M_MLX16_DBG_ADDRESS2                    (65535U << 0)                   /*!< RW: 16 LSB of breakpoint or patch address */

extern volatile uint16_t IO_MLX16_DBG_CTRL2_S __attribute__((nodp, addr(0x0004A)));  /*!< IO_MLX16_DBG_CTRL2_S (System) */
#define B_MLX16_DBG_READY2                      (1U << 15)                      /*!< RW: 1 if the debugger or CPU has written the data register or if a breakpoint has been triggered */
#define B_MLX16_DBG_OVR2                        (1U << 14)                      /*!< RW: 1 If the data register is written (by the debugger or the CPU) wile READY is 1 */
#define B_MLX16_DBG_MARKEXT2                    (1U << 12)                      /*!< RW: 1 If external mark are allowed */
#define B_MLX16_DBG_LOCK2                       (1U << 11)                      /*!< R: 1 If the CPU cannot write in debug register */
#define M_MLX16_DBG_MODE2                       (3U << 9)                       /*!< RW: Usage of corresponding register: 00 Disable 01 Mark 10 Patch 11 Break */
#define B_MLX16_DBG_RANGE2                      (1U << 8)                       /*!< RW: 0 Use equality comparator 1: Use equal or greater (for register 0 and 2), lower (for register 1 and 2) */
#define M_MLX16_DBG_COND2                       (15U << 4)                      /*!< RW: Break condition: 0000 Disable 0001 Fetch 0010 Write word 0011 Write byte 0100 Read word 0101 Read byte 0110 Access word 0111 Access byte 1010 Write word value 1011 Write byte value 1100 Read word value 1101 Read byte value 1110 Access word value 1111 Access byte value */
#define M_MLX16_DBG_CONTROL2                    (65535U << 0)                   /*!< RW: Debugger control port */
#define M_MLX16_DBG_ADREXT2                     (15U << 0)                      /*!< RW: 4 MSB of breakpoint or patch address */

extern volatile uint16_t IO_MLX16_DBG_ADDRESS3_S __attribute__((nodp, addr(0x0004C)));  /*!< IO_MLX16_DBG_ADDRESS3_S (System) */
#define M_MLX16_DBG_ADDRESS3                    (65535U << 0)                   /*!< RW: 16 LSB of breakpoint or patch address */

extern volatile uint16_t IO_MLX16_DBG_CTRL3_S __attribute__((nodp, addr(0x0004E)));  /*!< IO_MLX16_DBG_CTRL3_S (System) */
#define B_MLX16_DBG_READY3                      (1U << 15)                      /*!< RW: 1 if the debugger or CPU has written the data register or if a breakpoint has been triggered */
#define B_MLX16_DBG_OVR3                        (1U << 14)                      /*!< RW: 1 If the data register is written (by the debugger or the CPU) wile READY is 1 */
#define B_MLX16_DBG_MARKEXT3                    (1U << 12)                      /*!< RW: 1 If external mark are allowed */
#define B_MLX16_DBG_LOCK3                       (1U << 11)                      /*!< R: 1 If the CPU cannot write in debug register */
#define M_MLX16_DBG_MODE3                       (3U << 9)                       /*!< RW: Usage of corresponding register: 00 Disable 01 Mark 10 Patch 11 Break */
#define B_MLX16_DBG_RANGE3                      (1U << 8)                       /*!< RW: 0 Use equality comparator 1: Use equal or greater (for register 0 and 2), lower (for register 1 and 2) */
#define M_MLX16_DBG_COND3                       (15U << 4)                      /*!< RW: Break condition: 0000 Disable 0001 Fetch 0010 Write word 0011 Write byte 0100 Read word 0101 Read byte 0110 Access word 0111 Access byte 1010 Write word value 1011 Write byte value 1100 Read word value 1101 Read byte value 1110 Access word value 1111 Access byte value */
#define M_MLX16_DBG_CONTROL3                    (65535U << 0)                   /*!< RW: Debugger control port */
#define M_MLX16_DBG_ADREXT3                     (15U << 0)                      /*!< RW: 4 MSB of breakpoint or patch address */

extern volatile uint16_t IO_MLX16_ITC_PEND0_S __attribute__((nodp, addr(0x00050)));  /*!< IO_MLX16_ITC_PEND0_S (System) */
#define M_MLX16_ITC_PEND0                       (65535U << 0)                   /*!< R: 1 If the interrupt[INT0_COUNT+15:INT0_COUNT] is pending; W: Write 1 to clear pending interrupt, 0 does nothing */
#define B_MLX16_ITC_PEND0_MLX16_EXCHG           (1U << 0)                       /*!< 0x0030 */
#define B_MLX16_ITC_PEND0_MLX16_DMAERR          (1U << 1)                       /*!< 0x0038 */
#define B_MLX16_ITC_PEND0_AWD_ATT               (1U << 2)                       /*!< 0x0040 */
#define B_MLX16_ITC_PEND0_IWD_ATT               (1U << 3)                       /*!< 0x0048 */
#define B_MLX16_ITC_PEND0_FL_ECC                (1U << 4)                       /*!< 0x0050 */
#define B_MLX16_ITC_PEND0_EE_ECC                (1U << 5)                       /*!< 0x0058 */
#define B_MLX16_ITC_PEND0_EE_SH_ECC             (1U << 5)                       /*!< 0x0058 */
#define B_MLX16_ITC_PEND0_UV_VDDA               (1U << 6)                       /*!< 0x0060 */
#define B_MLX16_ITC_PEND0_UV_VSM                (1U << 7)                       /*!< 0x0068 */
#define B_MLX16_ITC_PEND0_UV_VDDAF              (1U << 8)                       /*!< 0x0070 */
#define B_MLX16_ITC_PEND0_ANA_PLL_ERR           (1U << 9)                       /*!< 0x0078 */
#define B_MLX16_ITC_PEND0_OVT                   (1U << 10)                      /*!< 0x0080 */
#define B_MLX16_ITC_PEND0_OVC0                  (1U << 11)                      /*!< 0x0088 */
#define B_MLX16_ITC_PEND0_OVC1                  (1U << 12)                      /*!< 0x0090 */
#define B_MLX16_ITC_PEND0_OC_VDDA               (1U << 13)                      /*!< 0x0098 */
#define B_MLX16_ITC_PEND0_STIMER                (1U << 14)                      /*!< 0x00A0 */
#define B_MLX16_ITC_PEND0_CTIMER0_1             (1U << 15)                      /*!< 0x00A8 */

extern volatile uint16_t IO_MLX16_ITC_PEND1_S __attribute__((nodp, addr(0x00052)));  /*!< IO_MLX16_ITC_PEND1_S (System) */
#define M_MLX16_ITC_PEND1                       (65535U << 0)                   /*!< R: 1 If the interrupt[INT0_COUNT+31:INT0_COUNT+16] is pending; W: Write 1 to clear pending interrupt, 0 does nothing */
#define B_MLX16_ITC_PEND1_CTIMER0_2             (1U << 0)                       /*!< 0x00B0 */
#define B_MLX16_ITC_PEND1_CTIMER0_3             (1U << 1)                       /*!< 0x00B8 */
#define B_MLX16_ITC_PEND1_CTIMER1_1             (1U << 2)                       /*!< 0x00C0 */
#define B_MLX16_ITC_PEND1_CTIMER1_2             (1U << 3)                       /*!< 0x00C8 */
#define B_MLX16_ITC_PEND1_CTIMER1_3             (1U << 4)                       /*!< 0x00D0 */
#define B_MLX16_ITC_PEND1_SPI_TE                (1U << 5)                       /*!< 0x00D8 */
#define B_MLX16_ITC_PEND1_SPI_RF                (1U << 6)                       /*!< 0x00E0 */
#define B_MLX16_ITC_PEND1_SPI_ER                (1U << 7)                       /*!< 0x00E8 */
#define B_MLX16_ITC_PEND1_PWM_MASTER1_CMP       (1U << 8)                       /*!< 0x00F0 */
#define B_MLX16_ITC_PEND1_PWM_MASTER1_END       (1U << 9)                       /*!< 0x00F8 */
#define B_MLX16_ITC_PEND1_PWM_SLAVE1_CMP        (1U << 10)                      /*!< 0x0100 */
#define B_MLX16_ITC_PEND1_PWM_SLAVE2_CMP        (1U << 11)                      /*!< 0x0108 */
#define B_MLX16_ITC_PEND1_PWM_MASTER2_CMP       (1U << 12)                      /*!< 0x0110 */
#define B_MLX16_ITC_PEND1_PWM_MASTER2_END       (1U << 13)                      /*!< 0x0118 */
#define B_MLX16_ITC_PEND1_PWM_SLAVE3_CMP        (1U << 14)                      /*!< 0x0120 */
#define B_MLX16_ITC_PEND1_PWM_SLAVE4_CMP        (1U << 15)                      /*!< 0x0128 */

extern volatile uint16_t IO_MLX16_ITC_PEND2_S __attribute__((nodp, addr(0x00054)));  /*!< IO_MLX16_ITC_PEND2_S (System) */
#define M_MLX16_ITC_PEND2                       (65535U << 0)                   /*!< R: 1 If the interrupt[INT0_COUNT+47:INT0_COUNT+32] is pending; W: Write 1 to clear pending interrupt, 0 does nothing */
#define B_MLX16_ITC_PEND2_ADC_BLOCK             (1U << 0)                       /*!< 0x0130 */
#define B_MLX16_ITC_PEND2_EE_COMPLETE           (1U << 1)                       /*!< 0x0138 */
#define B_MLX16_ITC_PEND2_FL_COMPLETE           (1U << 2)                       /*!< 0x0140 */
#define B_MLX16_ITC_PEND2_COLIN_OWNMTX          (1U << 3)                       /*!< 0x0148 */
#define B_MLX16_ITC_PEND2_COLIN_LIN             (1U << 4)                       /*!< 0x0150 */
#define B_MLX16_ITC_PEND2_OV_VSM                (1U << 5)                       /*!< 0x0158 */
#define B_MLX16_ITC_PEND2_OV_VDDA               (1U << 6)                       /*!< 0x0160 */
#define B_MLX16_ITC_PEND2_DIAG                  (1U << 7)                       /*!< 0x0168 */
#define B_MLX16_ITC_PEND2_I2C_GLOBAL_RESET      (1U << 8)                       /*!< 0x0170 */
#define B_MLX16_ITC_PEND2_I2C_EOW               (1U << 9)                       /*!< 0x0178 */
#define B_MLX16_ITC_PEND2_I2C_EOR               (1U << 10)                      /*!< 0x0180 */
#define B_MLX16_ITC_PEND2_I2C_ERR               (1U << 11)                      /*!< 0x0188 */
#define B_MLX16_ITC_PEND2_PPM_RX                (1U << 12)                      /*!< 0x0190 */
#define B_MLX16_ITC_PEND2_PPM_TX                (1U << 13)                      /*!< 0x0198 */
#define B_MLX16_ITC_PEND2_PPM_ERR               (1U << 14)                      /*!< 0x01A0 */
#define B_MLX16_ITC_PEND2_TX_TIMEOUT            (1U << 15)                      /*!< 0x01A8 */

extern volatile uint16_t IO_MLX16_ITC_PEND3_S __attribute__((nodp, addr(0x00056)));  /*!< IO_MLX16_ITC_PEND3_S (System) */
#define M_MLX16_ITC_PEND3                       (65535U << 0)                   /*!< R: 1 If the interrupt[INT0_COUNT+63:INT0_COUNT+48] is pending; W: Write 1 to clear pending interrupt, 0 does nothing */
#define B_MLX16_ITC_PEND3_UART0_SB              (1U << 0)                       /*!< 0x01B0 UART #0 Stop Bit Error */
#define B_MLX16_ITC_PEND3_UART0_RS              (1U << 1)                       /*!< 0x01B8 UART #0 Receive Error */
#define B_MLX16_ITC_PEND3_UART0_RR              (1U << 2)                       /*!< 0x01C0 UART #0 Receive */
#define B_MLX16_ITC_PEND3_UART0_TS              (1U << 3)                       /*!< 0x01C8 UART #0 Transmit End */
#define B_MLX16_ITC_PEND3_UART0_TR              (1U << 4)                       /*!< 0x01D0 UART #0 Transmit Begin */
#define B_MLX16_ITC_PEND3_UART0_TE              (1U << 5)                       /*!< 0x01D8 UART #0 Transmit Error */
#define B_MLX16_ITC_PEND3_UDFR0                 (1U << 6)                       /*!< 0x01E0 UART #0 DMA Frame Received */
#define B_MLX16_ITC_PEND3_UDFT0                 (1U << 7)                       /*!< 0x01E8 UART #0 DMA Frame Transmitted */
#define B_MLX16_ITC_PEND3_UART1_SB              (1U << 8)                       /*!< 0x01F0 UART #1 Stop Bit Error */
#define B_MLX16_ITC_PEND3_UART1_RS              (1U << 9)                       /*!< 0x01F8 UART #1 Receive Error */
#define B_MLX16_ITC_PEND3_UART1_RR              (1U << 10)                      /*!< 0x0200 UART #1 Receive */
#define B_MLX16_ITC_PEND3_UART1_TS              (1U << 11)                      /*!< 0x0208 UART #1 Transmit End */
#define B_MLX16_ITC_PEND3_UART1_TR              (1U << 12)                      /*!< 0x0210 UART #1 Transmit Begin */
#define B_MLX16_ITC_PEND3_UART1_TE              (1U << 13)                      /*!< 0x0218 UART #1 Transmit Error */
#define B_MLX16_ITC_PEND3_UDFR1                 (1U << 14)                      /*!< 0x0220 UART #1 DMA Frame Received */
#define B_MLX16_ITC_PEND3_UDFT1                 (1U << 15)                      /*!< 0x0228 UART #1 DMA Frame Transmitted */

extern volatile uint16_t IO_MLX16_ITC_PEND4_S __attribute__((nodp, addr(0x00058)));  /*!< IO_MLX16_ITC_PEND4_S (System) */
#define M_MLX16_ITC_PEND4                       (65535U << 0)                   /*!< R: 1 If the interrupt[INT0_COUNT+79:INT0_COUNT+64] is pending; W: Write 1 to clear pending interrupt, 0 does nothing */
#define B_MLX16_ITC_PEND4_IO_IN0                (1U << 0)                       /*!< 0x0230 */
#define B_MLX16_ITC_PEND4_IO_IN1                (1U << 1)                       /*!< 0x0238 */
#define B_MLX16_ITC_PEND4_IO_IN2                (1U << 2)                       /*!< 0x0240 */
#define B_MLX16_ITC_PEND4_IO_IN3                (1U << 3)                       /*!< 0x0248 */
#define B_MLX16_ITC_PEND4_IO_IN4                (1U << 4)                       /*!< 0x0250 */
#define B_MLX16_ITC_PEND4_IO_IN5                (1U << 5)                       /*!< 0x0258 */
#define B_MLX16_ITC_PEND4_IO_IN6                (1U << 6)                       /*!< 0x0260 */
#define B_MLX16_ITC_PEND4_IO_IN7                (1U << 7)                       /*!< 0x0268 */
#define B_MLX16_ITC_PEND4_IO_IN8                (1U << 8)                       /*!< 0x0270 */
#define B_MLX16_ITC_PEND4_IO_IN9                (1U << 9)                       /*!< 0x0278 */
#define B_MLX16_ITC_PEND4_IO_IN10               (1U << 10)                      /*!< 0x0280 */
#define B_MLX16_ITC_PEND4_IO_IN11               (1U << 11)                      /*!< 0x0288 */
#define B_MLX16_ITC_PEND4_PH_IN0                (1U << 12)                      /*!< 0x0290 */
#define B_MLX16_ITC_PEND4_PH_IN1                (1U << 13)                      /*!< 0x0298 */
#define B_MLX16_ITC_PEND4_PH_IN2                (1U << 14)                      /*!< 0x02A0 */
#define B_MLX16_ITC_PEND4_PH_IN3                (1U << 15)                      /*!< 0x02A8 */

extern volatile uint16_t IO_MLX16_ITC_PEND5_S __attribute__((nodp, addr(0x0005A)));  /*!< IO_MLX16_ITC_PEND5_S (System) */
#define M_MLX16_ITC_PEND5                       (15U << 0)                      /*!< R: 1 If the interrupt[INT0_COUNT+79:INT0_COUNT+64] is pending; W: Write 1 to clear pending interrupt, 0 does nothing */
#define B_MLX16_ITC_PEND5_PH_IN5                (1U << 0)                       /*!< 0x02B0 */
#define B_MLX16_ITC_PEND5_PH_IN6                (1U << 1)                       /*!< 0x02B8 */
#define B_MLX16_ITC_PEND5_I2C_MST               (1U << 2)                       /*!< 0x02C0 I2C Master */
#define B_MLX16_ITC_PEND5_MLX16_SOFT            (1U << 3)                       /*!< 0x02C8 */

extern volatile uint16_t IO_MLX16_SWI_S __attribute__((nodp, addr(0x0006A)));     /*!< IO_MLX16_SWI_S (System) */
#define B_MLX16_SWI                             (1U << 15)                      /*!< W: Request a software interrupt; R: Always read 0 */

extern volatile uint16_t IO_MLX16_ITC_MASK0_S __attribute__((nodp, addr(0x0006C)));  /*!< IO_MLX16_ITC_MASK0_S (System) */
#define M_MLX16_ITC_MASK0                       (65535U << 0)                       /*!< RW: 1 to enable interrupt[INT0_COUNT+15:INT0_COUNT] */
#define B_MLX16_ITC_MASK0_MLX16_EXCHG           (1U << 0)                       /*!< 0x0030 */
#define B_MLX16_ITC_MASK0_MLX16_DMAERR          (1U << 1)                       /*!< 0x0038 */
#define B_MLX16_ITC_MASK0_AWD_ATT               (1U << 2)                       /*!< 0x0040 */
#define B_MLX16_ITC_MASK0_IWD_ATT               (1U << 3)                       /*!< 0x0048 */
#define B_MLX16_ITC_MASK0_FL_ECC                (1U << 4)                       /*!< 0x0050 */
#define B_MLX16_ITC_MASK0_EE_ECC                (1U << 5)                       /*!< 0x0058 */
#define B_MLX16_ITC_MASK0_UV_VDDA               (1U << 6)                       /*!< 0x0060 */
#define B_MLX16_ITC_MASK0_UV_VSM                (1U << 7)                       /*!< 0x0068 */
#define B_MLX16_ITC_MASK0_UV_VDDAF              (1U << 8)                       /*!< 0x0070 */
#define B_MLX16_ITC_MASK0_ANA_PLL_ERR           (1U << 9)                       /*!< 0x0078 */
#define B_MLX16_ITC_MASK0_OVT                   (1U << 10)                      /*!< 0x0080 */
#define B_MLX16_ITC_MASK0_OVC0                  (1U << 11)                      /*!< 0x0088 */
#define B_MLX16_ITC_MASK0_OVC1                  (1U << 12)                      /*!< 0x0090 */
#define B_MLX16_ITC_MASK0_OC_VDDA               (1U << 13)                      /*!< 0x0098 */
#define B_MLX16_ITC_MASK0_STIMER                (1U << 14)                      /*!< 0x00A0 */
#define B_MLX16_ITC_MASK0_CTIMER0_1             (1U << 15)                      /*!< 0x00A8 */

extern volatile uint16_t IO_MLX16_ITC_MASK1_S __attribute__((nodp, addr(0x0006E)));  /*!< IO_MLX16_ITC_MASK1_S (System) */
#define M_MLX16_ITC_MASK1                       (65535U << 0)                   /*!< RW: 1 to enable interrupt[INT0_COUNT+31:INT0_COUNT+16] */
#define B_MLX16_ITC_MASK1_CTIMER0_2             (1U << 0)                       /*!< 0x00B0 */
#define B_MLX16_ITC_MASK1_CTIMER0_3             (1U << 1)                       /*!< 0x00B8 */
#define B_MLX16_ITC_MASK1_CTIMER1_1             (1U << 2)                       /*!< 0x00C0 */
#define B_MLX16_ITC_MASK1_CTIMER1_2             (1U << 3)                       /*!< 0x00C8 */
#define B_MLX16_ITC_MASK1_CTIMER1_3             (1U << 4)                       /*!< 0x00D0 */
#define B_MLX16_ITC_MASK1_SPI_TE                (1U << 5)                       /*!< 0x00D8 */
#define B_MLX16_ITC_MASK1_SPI_RF                (1U << 6)                       /*!< 0x00E0 */
#define B_MLX16_ITC_MASK1_SPI_ER                (1U << 7)                       /*!< 0x00E8 */
#define B_MLX16_ITC_MASK1_PWM_MASTER1_CMP       (1U << 8)                       /*!< 0x00F0 */
#define B_MLX16_ITC_MASK1_PWM_MASTER1_END       (1U << 9)                       /*!< 0x00F8 */
#define B_MLX16_ITC_MASK1_PWM_SLAVE1_CMP        (1U << 10)                      /*!< 0x0100 */
#define B_MLX16_ITC_MASK1_PWM_SLAVE2_CMP        (1U << 11)                      /*!< 0x0108 */
#define B_MLX16_ITC_MASK1_PWM_MASTER2_CMP       (1U << 12)                      /*!< 0x0110 */
#define B_MLX16_ITC_MASK1_PWM_MASTER2_END       (1U << 13)                      /*!< 0x0118 */
#define B_MLX16_ITC_MASK1_PWM_SLAVE3_CMP        (1U << 14)                      /*!< 0x0120 */
#define B_MLX16_ITC_MASK1_PWM_SLAVE4_CMP        (1U << 15)                      /*!< 0x0128 */

extern volatile uint16_t IO_MLX16_ITC_MASK2_S __attribute__((nodp, addr(0x00070)));  /*!< IO_MLX16_ITC_MASK2_S (System) */
#define M_MLX16_ITC_MASK2                       (65535U << 0)                   /*!< RW: 1 to enable interrupt[INT0_COUNT+47:INT0_COUNT+32] */
#define B_MLX16_ITC_MASK2_ADC_BLOCK             (1U << 0)                       /*!< 0x0130 */
#define B_MLX16_ITC_MASK2_EE_COMPLETE           (1U << 1)                       /*!< 0x0138 */
#define B_MLX16_ITC_MASK2_FL_COMPLETE           (1U << 2)                       /*!< 0x0140 */
#define B_MLX16_ITC_MASK2_COLIN_OWNMTX          (1U << 3)                       /*!< 0x0148 */
#define B_MLX16_ITC_MASK2_COLIN_LIN             (1U << 4)                       /*!< 0x0150 */
#define B_MLX16_ITC_MASK2_OV_VSM                (1U << 5)                       /*!< 0x0158 */
#define B_MLX16_ITC_MASK2_OV_VDDA               (1U << 6)                       /*!< 0x0160 */
#define B_MLX16_ITC_MASK2_DIAG                  (1U << 7)                       /*!< 0x0168 */
#define B_MLX16_ITC_MASK2_I2C_GLOBAL_RESET      (1U << 8)                       /*!< 0x0170 */
#define B_MLX16_ITC_MASK2_I2C_EOW               (1U << 9)                       /*!< 0x0178 */
#define B_MLX16_ITC_MASK2_I2C_EOR               (1U << 10)                      /*!< 0x0180 */
#define B_MLX16_ITC_MASK2_I2C_ERR               (1U << 11)                      /*!< 0x0188 */
#define B_MLX16_ITC_MASK2_PPM_RX                (1U << 12)                      /*!< 0x0190 */
#define B_MLX16_ITC_MASK2_PPM_TX                (1U << 13)                      /*!< 0x0198 */
#define B_MLX16_ITC_MASK2_PPM_ERR               (1U << 14)                      /*!< 0x01A0 */
#define B_MLX16_ITC_MASK2_TX_TIMEOUT            (1U << 15)                      /*!< 0x01A8 */

extern volatile uint16_t IO_MLX16_ITC_MASK3_S __attribute__((nodp, addr(0x00072)));  /*!< IO_MLX16_ITC_MASK3_S (System) */
#define M_MLX16_ITC_MASK3                       (65535U << 0)                   /*!< RW: 1 to enable interrupt[INT0_COUNT+63:INT0_COUNT+48] */
#define B_MLX16_ITC_MASK3_UART0_SB              (1U << 0)                       /*!< 0x01B0 UART #0 Stop Bit Error */
#define B_MLX16_ITC_MASK3_UART0_RS              (1U << 1)                       /*!< 0x01B8 UART #0 Receive Error */
#define B_MLX16_ITC_MASK3_UART0_RR              (1U << 2)                       /*!< 0x01C0 UART #0 Receive */
#define B_MLX16_ITC_MASK3_UART0_TS              (1U << 3)                       /*!< 0x01C8 UART #0 Transmit End */
#define B_MLX16_ITC_MASK3_UART0_TR              (1U << 4)                       /*!< 0x01D0 UART #0 Transmit Begin */
#define B_MLX16_ITC_MASK3_UART0_TE              (1U << 5)                       /*!< 0x01D8 UART #0 Transmit Error */
#define B_MLX16_ITC_MASK3_UDFR0                 (1U << 6)                       /*!< 0x01E0 UART #0 DMA Frame Received */
#define B_MLX16_ITC_MASK3_UDFT0                 (1U << 7)                       /*!< 0x01E8 UART #0 DMA Frame Transmitted */
#define B_MLX16_ITC_MASK3_UART1_SB              (1U << 8)                       /*!< 0x01F0 UART #1 Stop Bit Error */
#define B_MLX16_ITC_MASK3_UART1_RS              (1U << 9)                       /*!< 0x01F8 UART #1 Receive Error */
#define B_MLX16_ITC_MASK3_UART1_RR              (1U << 10)                      /*!< 0x0200 UART #1 Receive */
#define B_MLX16_ITC_MASK3_UART1_TS              (1U << 11)                      /*!< 0x0208 UART #1 Transmit End */
#define B_MLX16_ITC_MASK3_UART1_TR              (1U << 12)                      /*!< 0x0210 UART #1 Transmit Begin */
#define B_MLX16_ITC_MASK3_UART1_TE              (1U << 13)                      /*!< 0x0218 UART #1 Transmit Error */
#define B_MLX16_ITC_MASK3_UDFR1                 (1U << 14)                      /*!< 0x0220 UART #1 DMA Frame Received */
#define B_MLX16_ITC_MASK3_UDFT1                 (1U << 15)                      /*!< 0x0228 UART #1 DMA Frame Transmitted */

extern volatile uint16_t IO_MLX16_ITC_MASK4_S __attribute__((nodp, addr(0x00074)));  /*!< IO_MLX16_ITC_MASK4_S (System) */
#define M_MLX16_ITC_MASK4                       (2047U << 0)                    /*!< RW: 1 to enable interrupt[INT0_COUNT+79:INT0_COUNT+64] */
#define B_MLX16_ITC_MASK4_IO_IN0                (1U << 0)                       /*!< 0x0230 */
#define B_MLX16_ITC_MASK4_IO_IN1                (1U << 1)                       /*!< 0x0238 */
#define B_MLX16_ITC_MASK4_IO_IN2                (1U << 2)                       /*!< 0x0240 */
#define B_MLX16_ITC_MASK4_IO_IN3                (1U << 3)                       /*!< 0x0248 */
#define B_MLX16_ITC_MASK4_IO_IN4                (1U << 4)                       /*!< 0x0250 */
#define B_MLX16_ITC_MASK4_IO_IN5                (1U << 5)                       /*!< 0x0258 */
#define B_MLX16_ITC_MASK4_IO_IN6                (1U << 6)                       /*!< 0x0260 */
#define B_MLX16_ITC_MASK4_IO_IN7                (1U << 7)                       /*!< 0x0268 */
#define B_MLX16_ITC_MASK4_IO_IN8                (1U << 8)                       /*!< 0x0270 */
#define B_MLX16_ITC_MASK4_IO_IN9                (1U << 9)                       /*!< 0x0278 */
#define B_MLX16_ITC_MASK4_IO_IN10               (1U << 10)                      /*!< 0x0280 */
#define B_MLX16_ITC_MASK4_IO_IN11               (1U << 11)                      /*!< 0x0288 */
#define B_MLX16_ITC_MASK4_PH_IN0                (1U << 12)                      /*!< 0x0290 */
#define B_MLX16_ITC_MASK4_PH_IN1                (1U << 13)                      /*!< 0x0298 */
#define B_MLX16_ITC_MASK4_PH_IN2                (1U << 14)                      /*!< 0x02A0 */
#define B_MLX16_ITC_MASK4_PH_IN3                (1U << 15)                      /*!< 0x02A8 */

extern volatile uint16_t IO_MLX16_ITC_MASK5_S __attribute__((nodp, addr(0x00076)));  /*!< IO_MLX16_ITC_MASK5_S (System) */
#define M_MLX16_ITC_MASK5                       (15U << 0)                      /*!< R: 1 If the interrupt[INT0_COUNT+79:INT0_COUNT+64] is pending; W: Write 1 to clear pending interrupt, 0 does nothing */
#define B_MLX16_ITC_MASK5_PH_IN5                (1U << 0)                       /*!< 0x02B0 */
#define B_MLX16_ITC_MASK5_PH_IN6                (1U << 1)                       /*!< 0x02B8 */
#define B_MLX16_ITC_MASK5_I2C_MST               (1U << 2)                       /*!< 0x02C0 I2C Master */
#define B_MLX16_ITC_MASK5_MLX16_SOFT            (1U << 3)                       /*!< 0x02C8 */

extern volatile uint16_t IO_MLX16_ITC_PRIO0_S __attribute__((nodp, addr(0x00088)));  /*!< IO_MLX16_ITC_PRIO0_S (System) */
#define M_MLX16_ITC_PRIO0                       (65535U << 0)                   /*!< RW: Configure priority for interrupt[INT0_COUNT+INT1_COUNT+INT2_COUNT+7:INT0_COUNT+INT1_COUNT+INT2_COUNT] */
#define M_MLX16_ITC_PRIO0_STIMER                (3U << 0)                       /*!< RW: priority STIMER */
#define C_MLX16_ITC_PRIO0_STIMER_PRIO3          (0U << 0)                       /*!< RW: priority 3 STIMER */
#define C_MLX16_ITC_PRIO0_STIMER_PRIO4          (1U << 0)                       /*!< RW: priority 4 STIMER */
#define C_MLX16_ITC_PRIO0_STIMER_PRIO5          (2U << 0)                       /*!< RW: priority 5 STIMER */
#define C_MLX16_ITC_PRIO0_STIMER_PRIO6          (3U << 0)                       /*!< RW: priority 6 STIMER */
#define M_MLX16_ITC_PRIO0_CTIMER0_1             (3U << 2)                       /*!< RW: priority CTIMER0_1 */
#define C_MLX16_ITC_PRIO0_CTIMER0_1_PRIO3       (0U << 2)                       /*!< RW: priority 3 CTIMER0_1 */
#define C_MLX16_ITC_PRIO0_CTIMER0_1_PRIO4       (1U << 2)                       /*!< RW: priority 4 CTIMER0_1 */
#define C_MLX16_ITC_PRIO0_CTIMER0_1_PRIO5       (2U << 2)                       /*!< RW: priority 5 CTIMER0_1 */
#define C_MLX16_ITC_PRIO0_CTIMER0_1_PRIO6       (3U << 2)                       /*!< RW: priority 6 CTIMER0_1 */
#define M_MLX16_ITC_PRIO0_CTIMER0_2             (3U << 4)                       /*!< RW: priority CTIMER0_2 */
#define C_MLX16_ITC_PRIO0_CTIMER0_2_PRIO3       (0U << 4)                       /*!< RW: priority 3 CTIMER0_2 */
#define C_MLX16_ITC_PRIO0_CTIMER0_2_PRIO4       (1U << 4)                       /*!< RW: priority 4 CTIMER0_2 */
#define C_MLX16_ITC_PRIO0_CTIMER0_2_PRIO5       (2U << 4)                       /*!< RW: priority 5 CTIMER0_2 */
#define C_MLX16_ITC_PRIO0_CTIMER0_2_PRIO6       (3U << 4)                       /*!< RW: priority 6 CTIMER0_2 */
#define M_MLX16_ITC_PRIO0_CTIMER0_3             (3U << 6)                       /*!< RW: priority CTIMER0_3 */
#define C_MLX16_ITC_PRIO0_CTIMER0_3_PRIO3       (0U << 6)                       /*!< RW: priority 3 CTIMER0_3 */
#define C_MLX16_ITC_PRIO0_CTIMER0_3_PRIO4       (1U << 6)                       /*!< RW: priority 4 CTIMER0_3 */
#define C_MLX16_ITC_PRIO0_CTIMER0_3_PRIO5       (2U << 6)                       /*!< RW: priority 5 CTIMER0_3 */
#define C_MLX16_ITC_PRIO0_CTIMER0_3_PRIO6       (3U << 6)                       /*!< RW: priority 6 CTIMER0_3 */
#define M_MLX16_ITC_PRIO0_CTIMER1_1             (3U << 8)                       /*!< RW: priority CTIMER1_1 */
#define C_MLX16_ITC_PRIO0_CTIMER1_1_PRIO3       (0U << 8)                       /*!< RW: priority 3 CTIMER1_1 */
#define C_MLX16_ITC_PRIO0_CTIMER1_1_PRIO4       (1U << 8)                       /*!< RW: priority 4 CTIMER1_1 */
#define C_MLX16_ITC_PRIO0_CTIMER1_1_PRIO5       (2U << 8)                       /*!< RW: priority 5 CTIMER1_1 */
#define C_MLX16_ITC_PRIO0_CTIMER1_1_PRIO6       (3U << 8)                       /*!< RW: priority 6 CTIMER1_1 */
#define M_MLX16_ITC_PRIO0_CTIMER1_2             (3U << 10)                      /*!< RW: priority CTIMER1_2 */
#define C_MLX16_ITC_PRIO0_CTIMER1_2_PRIO3       (0U << 10)                      /*!< RW: priority 3 CTIMER1_2 */
#define C_MLX16_ITC_PRIO0_CTIMER1_2_PRIO4       (1U << 10)                      /*!< RW: priority 4 CTIMER1_2 */
#define C_MLX16_ITC_PRIO0_CTIMER1_2_PRIO5       (2U << 10)                      /*!< RW: priority 5 CTIMER1_2 */
#define C_MLX16_ITC_PRIO0_CTIMER1_2_PRIO6       (3U << 10)                      /*!< RW: priority 6 CTIMER1_2 */
#define M_MLX16_ITC_PRIO0_CTIMER1_3             (3U << 12)                      /*!< RW: priority CTIMER1_3 */
#define C_MLX16_ITC_PRIO0_CTIMER1_3_PRIO3       (0U << 12)                      /*!< RW: priority 3 CTIMER1_3 */
#define C_MLX16_ITC_PRIO0_CTIMER1_3_PRIO4       (1U << 12)                      /*!< RW: priority 4 CTIMER1_3 */
#define C_MLX16_ITC_PRIO0_CTIMER1_3_PRIO5       (2U << 12)                      /*!< RW: priority 5 CTIMER1_3 */
#define C_MLX16_ITC_PRIO0_CTIMER1_3_PRIO6       (3U << 12)                      /*!< RW: priority 6 CTIMER1_3 */
#define M_MLX16_ITC_PRIO0_SPI_TE                (3U << 14)                      /*!< RW: priority SPI_TE */
#define C_MLX16_ITC_PRIO0_SPI_TE_PRIO3          (0U << 14)                      /*!< RW: priority 3 SPI_TE */
#define C_MLX16_ITC_PRIO0_SPI_TE_PRIO4          (1U << 14)                      /*!< RW: priority 4 SPI_TE */
#define C_MLX16_ITC_PRIO0_SPI_TE_PRIO5          (2U << 14)                      /*!< RW: priority 5 SPI_TE */
#define C_MLX16_ITC_PRIO0_SPI_TE_PRIO6          (3U << 14)                      /*!< RW: priority 6 SPI_TE */

extern volatile uint16_t IO_MLX16_ITC_PRIO1_S __attribute__((nodp, addr(0x0008A)));  /*!< IO_MLX16_ITC_PRIO1_S (System) */
#define M_MLX16_ITC_PRIO1                       (65535U << 0)                   /*!< RW: Configure priority for interrupt[INT0_COUNT+INT1_COUNT+INT2_COUNT+15:INT0_COUNT+INT1_COUNT+INT2_COUNT+8] */
#define M_MLX16_ITC_PRIO1_SPI_RF                (3U << 0)                       /*!< RW: priority SPI_RF */
#define C_MLX16_ITC_PRIO1_SPI_RF_PRIO3          (0U << 0)                       /*!< RW: priority 3 SPI_RF */
#define C_MLX16_ITC_PRIO1_SPI_RF_PRIO4          (1U << 0)                       /*!< RW: priority 4 SPI_RF */
#define C_MLX16_ITC_PRIO1_SPI_RF_PRIO5          (2U << 0)                       /*!< RW: priority 5 SPI_RF */
#define C_MLX16_ITC_PRIO1_SPI_RF_PRIO6          (3U << 0)                       /*!< RW: priority 6 SPI_RF */
#define M_MLX16_ITC_PRIO1_SPI_ER                (3U << 2)                       /*!< RW: priority SPI_ER */
#define C_MLX16_ITC_PRIO1_SPI_ER_PRIO3          (0U << 2)                       /*!< RW: priority 3 SPI_ER */
#define C_MLX16_ITC_PRIO1_SPI_ER_PRIO4          (1U << 2)                       /*!< RW: priority 4 SPI_ER */
#define C_MLX16_ITC_PRIO1_SPI_ER_PRIO5          (2U << 2)                       /*!< RW: priority 5 SPI_ER */
#define C_MLX16_ITC_PRIO1_SPI_ER_PRIO6          (3U << 2)                       /*!< RW: priority 6 SPI_ER */
#define M_MLX16_ITC_PRIO1_PWM_MASTER1_CMP       (3U << 4)                       /*!< RW: priority PWM_MASTER1_CMP */
#define C_MLX16_ITC_PRIO1_PWM_MASTER1_CMP_PRIO3 (0U << 4)                       /*!< RW: priority 3 PWM_MASTER1_CMP */
#define C_MLX16_ITC_PRIO1_PWM_MASTER1_CMP_PRIO4 (1U << 4)                       /*!< RW: priority 4 PWM_MASTER1_CMP */
#define C_MLX16_ITC_PRIO1_PWM_MASTER1_CMP_PRIO5 (2U << 4)                       /*!< RW: priority 5 PWM_MASTER1_CMP */
#define C_MLX16_ITC_PRIO1_PWM_MASTER1_CMP_PRIO6 (3U << 4)                       /*!< RW: priority 6 PWM_MASTER1_CMP */
#define M_MLX16_ITC_PRIO1_PWM_MASTER1_END       (3U << 6)                       /*!< RW: priority PWM_MASTER1_END */
#define C_MLX16_ITC_PRIO1_PWM_MASTER1_END_PRIO3 (0U << 6)                       /*!< RW: priority 3 PWM_MASTER1_END */
#define C_MLX16_ITC_PRIO1_PWM_MASTER1_END_PRIO4 (1U << 6)                       /*!< RW: priority 4 PWM_MASTER1_END */
#define C_MLX16_ITC_PRIO1_PWM_MASTER1_END_PRIO5 (2U << 6)                       /*!< RW: priority 5 PWM_MASTER1_END */
#define C_MLX16_ITC_PRIO1_PWM_MASTER1_END_PRIO6 (3U << 6)                       /*!< RW: priority 6 PWM_MASTER1_END */
#define M_MLX16_ITC_PRIO1_PWM_SLAVE1_CMP        (3U << 8)                       /*!< RW: priority PWM_SLAVE1_CMP */
#define C_MLX16_ITC_PRIO1_PWM_SLAVE1_CMP_PRIO3  (0U << 8)                       /*!< RW: priority 3 PWM_SLAVE1_CMP */
#define C_MLX16_ITC_PRIO1_PWM_SLAVE1_CMP_PRIO4  (1U << 8)                       /*!< RW: priority 4 PWM_SLAVE1_CMP */
#define C_MLX16_ITC_PRIO1_PWM_SLAVE1_CMP_PRIO5  (2U << 8)                       /*!< RW: priority 5 PWM_SLAVE1_CMP */
#define C_MLX16_ITC_PRIO1_PWM_SLAVE1_CMP_PRIO6  (3U << 8)                       /*!< RW: priority 6 PWM_SLAVE1_CMP */
#define M_MLX16_ITC_PRIO1_PWM_SLAVE2_CMP        (3U << 10)                      /*!< RW: priority PWM_SLAVE2_CMP */
#define C_MLX16_ITC_PRIO1_PWM_SLAVE2_CMP_PRIO3  (0U << 10)                      /*!< RW: priority 3 PWM_SLAVE2_CMP */
#define C_MLX16_ITC_PRIO1_PWM_SLAVE2_CMP_PRIO4  (1U << 10)                      /*!< RW: priority 4 PWM_SLAVE2_CMP */
#define C_MLX16_ITC_PRIO1_PWM_SLAVE2_CMP_PRIO5  (2U << 10)                      /*!< RW: priority 5 PWM_SLAVE2_CMP */
#define C_MLX16_ITC_PRIO1_PWM_SLAVE2_CMP_PRIO6  (3U << 10)                      /*!< RW: priority 6 PWM_SLAVE2_CMP */
#define M_MLX16_ITC_PRIO1_PWM_MASTER2_CMP       (3U << 12)                      /*!< RW: priority PWM_MASTER2_CMP */
#define C_MLX16_ITC_PRIO1_PWM_MASTER2_CMP_PRIO3 (0U << 12)                      /*!< RW: priority 3 PWM_MASTER2_CMP */
#define C_MLX16_ITC_PRIO1_PWM_MASTER2_CMP_PRIO4 (1U << 12)                      /*!< RW: priority 4 PWM_MASTER2_CMP */
#define C_MLX16_ITC_PRIO1_PWM_MASTER2_CMP_PRIO5 (2U << 12)                      /*!< RW: priority 5 PWM_MASTER2_CMP */
#define C_MLX16_ITC_PRIO1_PWM_MASTER2_CMP_PRIO6 (3U << 12)                      /*!< RW: priority 6 PWM_MASTER2_CMP */
#define M_MLX16_ITC_PRIO1_PWM_MASTER2_END       (3U << 14)                      /*!< RW: priority PWM_MASTER2_END */
#define C_MLX16_ITC_PRIO1_PWM_MASTER2_END_PRIO3 (0U << 14)                      /*!< RW: priority 3 PWM_MASTER2_END */
#define C_MLX16_ITC_PRIO1_PWM_MASTER2_END_PRIO4 (1U << 14)                      /*!< RW: priority 4 PWM_MASTER2_END */
#define C_MLX16_ITC_PRIO1_PWM_MASTER2_END_PRIO5 (2U << 14)                      /*!< RW: priority 5 PWM_MASTER2_END */
#define C_MLX16_ITC_PRIO1_PWM_MASTER2_END_PRIO6 (3U << 14)                      /*!< RW: priority 6 PWM_MASTER2_END */

extern volatile uint16_t IO_MLX16_ITC_PRIO2_S __attribute__((nodp, addr(0x0008C)));  /*!< IO_MLX16_ITC_PRIO2_S (System) */
#define M_MLX16_ITC_PRIO2                       (65535U << 0)                   /*!< RW: Configure priority for interrupt[INT0_COUNT+INT1_COUNT+INT2_COUNT+23:INT0_COUNT+INT1_COUNT+INT2_COUNT+16] */
#define M_MLX16_ITC_PRIO2_PWM_SLAVE3_CMP        (3U << 0)                       /*!< RW: priority PWM_SLAVE3_CMP */
#define C_MLX16_ITC_PRIO2_PWM_SLAVE3_CMP_PRIO3  (0U << 0)                       /*!< RW: priority 3 PWM_SLAVE3_CMP */
#define C_MLX16_ITC_PRIO2_PWM_SLAVE3_CMP_PRIO4  (1U << 0)                       /*!< RW: priority 4 PWM_SLAVE3_CMP */
#define C_MLX16_ITC_PRIO2_PWM_SLAVE3_CMP_PRIO5  (2U << 0)                       /*!< RW: priority 5 PWM_SLAVE3_CMP */
#define C_MLX16_ITC_PRIO2_PWM_SLAVE3_CMP_PRIO6  (3U << 0)                       /*!< RW: priority 6 PWM_SLAVE3_CMP */
#define M_MLX16_ITC_PRIO2_PWM_SLAVE4_CMP        (3U << 2)                       /*!< RW: priority PWM_SLAVE4_CMP */
#define C_MLX16_ITC_PRIO2_PWM_SLAVE4_CMP_PRIO3  (0U << 2)                       /*!< RW: priority 3 PWM_SLAVE4_CMP */
#define C_MLX16_ITC_PRIO2_PWM_SLAVE4_CMP_PRIO4  (1U << 2)                       /*!< RW: priority 4 PWM_SLAVE4_CMP */
#define C_MLX16_ITC_PRIO2_PWM_SLAVE4_CMP_PRIO5  (2U << 2)                       /*!< RW: priority 5 PWM_SLAVE4_CMP */
#define C_MLX16_ITC_PRIO2_PWM_SLAVE4_CMP_PRIO6  (3U << 2)                       /*!< RW: priority 6 PWM_SLAVE4_CMP */
#define M_MLX16_ITC_PRIO2_ADC_BLOCK             (3U << 4)                       /*!< RW: priority ADC_SAR */
#define C_MLX16_ITC_PRIO2_ADC_BLOCK_PRIO3       (0U << 4)                       /*!< RW: priority 3 ADC_SAR */
#define C_MLX16_ITC_PRIO2_ADC_BLOCK_PRIO4       (1U << 4)                       /*!< RW: priority 4 ADC_SAR */
#define C_MLX16_ITC_PRIO2_ADC_BLOCK_PRIO5       (2U << 4)                       /*!< RW: priority 5 ADC_SAR */
#define C_MLX16_ITC_PRIO2_ADC_BLOCK_PRIO6       (3U << 4)                       /*!< RW: priority 6 ADC_SAR */
#define M_MLX16_ITC_PRIO2_EE_COMPLETE           (3U << 6)                       /*!< RW: priority EE_COMPLETE */
#define C_MLX16_ITC_PRIO2_EE_COMPLETE_PRIO3     (0U << 6)                       /*!< RW: priority 3 EE_COMPLETE */
#define C_MLX16_ITC_PRIO2_EE_COMPLETE_PRIO4     (1U << 6)                       /*!< RW: priority 4 EE_COMPLETE */
#define C_MLX16_ITC_PRIO2_EE_COMPLETE_PRIO5     (2U << 6)                       /*!< RW: priority 5 EE_COMPLETE */
#define C_MLX16_ITC_PRIO2_EE_COMPLETE_PRIO6     (3U << 6)                       /*!< RW: priority 6 EE_COMPLETE */
#define M_MLX16_ITC_PRIO2_FL_COMPLETE           (3U << 8)                       /*!< RW: priority FL_COMPLETE */
#define C_MLX16_ITC_PRIO2_FL_COMPLETE_PRIO3     (0U << 8)                       /*!< RW: priority 3 FL_COMPLETE */
#define C_MLX16_ITC_PRIO2_FL_COMPLETE_PRIO4     (1U << 8)                       /*!< RW: priority 4 FL_COMPLETE */
#define C_MLX16_ITC_PRIO2_FL_COMPLETE_PRIO5     (2U << 8)                       /*!< RW: priority 5 FL_COMPLETE */
#define C_MLX16_ITC_PRIO2_FL_COMPLETE_PRIO6     (3U << 8)                       /*!< RW: priority 6 FL_COMPLETE */
#define M_MLX16_ITC_PRIO2_COLIN_OWNMTX          (3U << 10)                      /*!< RW: priority COLIN_OWNMTX */
#define C_MLX16_ITC_PRIO2_COLIN_OWNMTX_PRIO3    (0U << 10)                      /*!< RW: priority 3 COLIN_OWNMTX */
#define C_MLX16_ITC_PRIO2_COLIN_OWNMTX_PRIO4    (1U << 10)                      /*!< RW: priority 4 COLIN_OWNMTX */
#define C_MLX16_ITC_PRIO2_COLIN_OWNMTX_PRIO5    (2U << 10)                      /*!< RW: priority 5 COLIN_OWNMTX */
#define C_MLX16_ITC_PRIO2_COLIN_OWNMTX_PRIO6    (3U << 10)                      /*!< RW: priority 6 COLIN_OWNMTX */
#define M_MLX16_ITC_PRIO2_COLIN_LIN             (3U << 12)                      /*!< RW: priority COLIN_LINR */
#define C_MLX16_ITC_PRIO2_COLIN_LIN_PRIO3       (0U << 12)                      /*!< RW: priority 3 COLIN_LINR */
#define C_MLX16_ITC_PRIO2_COLIN_LIN_PRIO4       (1U << 12)                      /*!< RW: priority 4 COLIN_LINR */
#define C_MLX16_ITC_PRIO2_COLIN_LIN_PRIO5       (2U << 12)                      /*!< RW: priority 5 COLIN_LINR */
#define C_MLX16_ITC_PRIO2_COLIN_LIN_PRIO6       (3U << 12)                      /*!< RW: priority 6 COLIN_LINR */
#define M_MLX16_ITC_PRIO2_OV_VSM                (3U << 14)                      /*!< RW: priority OV_VSM */
#define C_MLX16_ITC_PRIO2_OV_VSM_PRIO3          (0U << 14)                      /*!< RW: priority 3 OV_VSM */
#define C_MLX16_ITC_PRIO2_OV_VSM_PRIO4          (1U << 14)                      /*!< RW: priority 4 OV_VSM */
#define C_MLX16_ITC_PRIO2_OV_VSM_PRIO5          (2U << 14)                      /*!< RW: priority 5 OV_VSM */
#define C_MLX16_ITC_PRIO2_OV_VSM_PRIO6          (3U << 14)                      /*!< RW: priority 6 OV_VSM */

extern volatile uint16_t IO_MLX16_ITC_PRIO3_S __attribute__((nodp, addr(0x0008E)));  /*!< IO_MLX16_ITC_PRIO3_S (System) */
#define M_MLX16_ITC_PRIO3                       (65535U << 0)                   /*!< RW: Configure priority for interrupt[INT0_COUNT+INT1_COUNT+INT2_COUNT+31:INT0_COUNT+INT1_COUNT+INT2_COUNT+24] */
#define M_MLX16_ITC_PRIO3_OV_VDDA               (3U << 0)                       /*!< RW: priority OV_VDDA */
#define C_MLX16_ITC_PRIO3_OV_VDDA_PRIO3         (0U << 0)                       /*!< RW: priority 3 OV_VDDA */
#define C_MLX16_ITC_PRIO3_OV_VDDA_PRIO4         (1U << 0)                       /*!< RW: priority 4 OV_VDDA */
#define C_MLX16_ITC_PRIO3_OV_VDDA_PRIO5         (2U << 0)                       /*!< RW: priority 5 OV_VDDA */
#define C_MLX16_ITC_PRIO3_OV_VDDA_PRIO6         (3U << 0)                       /*!< RW: priority 6 OV_VDDA */
#define M_MLX16_ITC_PRIO3_DIAG                  (3U << 2)                       /*!< RW: priority DIAG */
#define C_MLX16_ITC_PRIO3_DIAG_PRIO3            (0U << 2)                       /*!< RW: priority 3 DIAG */
#define C_MLX16_ITC_PRIO3_DIAG_PRIO4            (1U << 2)                       /*!< RW: priority 4 DIAG */
#define C_MLX16_ITC_PRIO3_DIAG_PRIO5            (2U << 2)                       /*!< RW: priority 5 DIAG */
#define C_MLX16_ITC_PRIO3_DIAG_PRIO6            (3U << 2)                       /*!< RW: priority 6 DIAG */
#define M_MLX16_ITC_PRIO3_I2C_GLOBAL_RESET      (3U << 4)                       /*!< RW: priority I2C_GLOBAL_RESET */
#define C_MLX16_ITC_PRIO3_I2C_GLOBAL_RESET_PRIO3 (0U << 4)                      /*!< RW: priority 3 I2C_GLOBAL_RESET */
#define C_MLX16_ITC_PRIO3_I2C_GLOBAL_RESET_PRIO4 (1U << 4)                      /*!< RW: priority 4 I2C_GLOBAL_RESET */
#define C_MLX16_ITC_PRIO3_I2C_GLOBAL_RESET_PRIO5 (2U << 4)                      /*!< RW: priority 5 I2C_GLOBAL_RESET */
#define C_MLX16_ITC_PRIO3_I2C_GLOBAL_RESET_PRIO6 (3U << 4)                      /*!< RW: priority 6 I2C_GLOBAL_RESET */
#define M_MLX16_ITC_PRIO3_I2C_EOW               (3U << 6)                       /*!< RW: priority I2C_EOW */
#define C_MLX16_ITC_PRIO3_I2C_EOW_PRIO3         (0U << 6)                       /*!< RW: priority 3 I2C_EOW */
#define C_MLX16_ITC_PRIO3_I2C_EOW_PRIO4         (1U << 6)                       /*!< RW: priority 4 I2C_EOW */
#define C_MLX16_ITC_PRIO3_I2C_EOW_PRIO5         (2U << 6)                       /*!< RW: priority 5 I2C_EOW */
#define C_MLX16_ITC_PRIO3_I2C_EOW_PRIO6         (3U << 6)                       /*!< RW: priority 6 I2C_EOW */
#define M_MLX16_ITC_PRIO3_I2C_EOR               (3U << 8)                       /*!< RW: priority I2C_EOR */
#define C_MLX16_ITC_PRIO3_I2C_EOR_PRIO3         (0U << 8)                       /*!< RW: priority 3 I2C_EOR */
#define C_MLX16_ITC_PRIO3_I2C_EOR_PRIO4         (1U << 8)                       /*!< RW: priority 4 I2C_EOR */
#define C_MLX16_ITC_PRIO3_I2C_EOR_PRIO5         (2U << 8)                       /*!< RW: priority 5 I2C_EOR */
#define C_MLX16_ITC_PRIO3_I2C_EOR_PRIO6         (3U << 8)                       /*!< RW: priority 6 I2C_EOR */
#define M_MLX16_ITC_PRIO3_I2C_ERR               (3U << 10)                      /*!< RW: priority I2C_ERR */
#define C_MLX16_ITC_PRIO3_I2C_ERR_PRIO3         (0U << 10)                      /*!< RW: priority 3 I2C_ERR */
#define C_MLX16_ITC_PRIO3_I2C_ERR_PRIO4         (1U << 10)                      /*!< RW: priority 4 I2C_ERR */
#define C_MLX16_ITC_PRIO3_I2C_ERR_PRIO5         (2U << 10)                      /*!< RW: priority 5 I2C_ERR */
#define C_MLX16_ITC_PRIO3_I2C_ERR_PRIO6         (3U << 10)                      /*!< RW: priority 6 I2C_ERR */
#define M_MLX16_ITC_PRIO3_PPM_RX                (3U << 12)                      /*!< RW: priority PPM_RX */
#define C_MLX16_ITC_PRIO3_PPM_RX_PRIO3          (0U << 12)                      /*!< RW: priority 3 PPM_RX */
#define C_MLX16_ITC_PRIO3_PPM_RX_PRIO4          (1U << 12)                      /*!< RW: priority 4 PPM_RX */
#define C_MLX16_ITC_PRIO3_PPM_RX_PRIO5          (2U << 12)                      /*!< RW: priority 5 PPM_RX */
#define C_MLX16_ITC_PRIO3_PPM_RX_PRIO6          (3U << 12)                      /*!< RW: priority 6 PPM_RX */
#define M_MLX16_ITC_PRIO3_PPM_TX                (3U << 14)                      /*!< RW: priority PPM_TX */
#define C_MLX16_ITC_PRIO3_PPM_TX_PRIO3          (0U << 14)                      /*!< RW: priority 3 PPM_TX */
#define C_MLX16_ITC_PRIO3_PPM_TX_PRIO4          (1U << 14)                      /*!< RW: priority 4 PPM_TX */
#define C_MLX16_ITC_PRIO3_PPM_TX_PRIO5          (2U << 14)                      /*!< RW: priority 5 PPM_TX */
#define C_MLX16_ITC_PRIO3_PPM_TX_PRIO6          (3U << 14)                      /*!< RW: priority 6 PPM_TX */

extern volatile uint16_t IO_MLX16_ITC_PRIO4_S __attribute__((nodp, addr(0x00090)));  /*!< IO_MLX16_ITC_PRIO4_S (System) */
#define M_MLX16_ITC_PRIO4                       (65535U << 0)                   /*!< RW: Configure priority for interrupt[INT0_COUNT+INT1_COUNT+INT2_COUNT+39:INT0_COUNT+INT1_COUNT+INT2_COUNT+32] */
#define M_MLX16_ITC_PRIO4_PPM_ERR               (3U << 0)                       /*!< RW: priority PPM_ERR */
#define C_MLX16_ITC_PRIO4_PPM_ERR_PRIO3         (0U << 0)                       /*!< RW: priority 3 PPM_ERR */
#define C_MLX16_ITC_PRIO4_PPM_ERR_PRIO4         (1U << 0)                       /*!< RW: priority 4 PPM_ERR */
#define C_MLX16_ITC_PRIO4_PPM_ERR_PRIO5         (2U << 0)                       /*!< RW: priority 5 PPM_ERR */
#define C_MLX16_ITC_PRIO4_PPM_ERR_PRIO6         (3U << 0)                       /*!< RW: priority 6 PPM_ERR */
#define M_MLX16_ITC_PRIO4_TX_TIMEOUT            (3U << 2)                       /*!< RW: priority TX_TIMEOUT */
#define C_MLX16_ITC_PRIO4_TX_TIMEOUT_PRIO3      (0U << 2)                       /*!< RW: priority 3 TX_TIMEOUT */
#define C_MLX16_ITC_PRIO4_TX_TIMEOUT_PRIO4      (1U << 2)                       /*!< RW: priority 4 TX_TIMEOUT */
#define C_MLX16_ITC_PRIO4_TX_TIMEOUT_PRIO5      (2U << 2)                       /*!< RW: priority 5 TX_TIMEOUT */
#define C_MLX16_ITC_PRIO4_TX_TIMEOUT_PRIO6      (3U << 2)                       /*!< RW: priority 6 TX_TIMEOUT */
#define M_MLX16_ITC_PRIO4_UART0_SB              (3U << 4)                       /*!< RW: priority UART0_SB */
#define C_MLX16_ITC_PRIO4_UART0_SB_PRIO3        (0U << 4)                       /*!< RW: priority 3 UART0_SB */
#define C_MLX16_ITC_PRIO4_UART0_SB_PRIO4        (1U << 4)                       /*!< RW: priority 4 UART0_SB */
#define C_MLX16_ITC_PRIO4_UART0_SB_PRIO5        (2U << 4)                       /*!< RW: priority 5 UART0_SB */
#define C_MLX16_ITC_PRIO4_UART0_SB_PRIO6        (3U << 4)                       /*!< RW: priority 6 UART0_SB */
#define M_MLX16_ITC_PRIO4_UART0_RS              (3U << 6)                       /*!< RW: priority UART0_RE */
#define C_MLX16_ITC_PRIO4_UART0_RS_PRIO3        (0U << 6)                       /*!< RW: priority 3 UART0_RE */
#define C_MLX16_ITC_PRIO4_UART0_RS_PRIO4        (1U << 6)                       /*!< RW: priority 4 UART0_RE */
#define C_MLX16_ITC_PRIO4_UART0_RS_PRIO5        (2U << 6)                       /*!< RW: priority 5 UART0_RE */
#define C_MLX16_ITC_PRIO4_UART0_RS_PRIO6        (3U << 6)                       /*!< RW: priority 6 UART0_RE */
#define M_MLX16_ITC_PRIO4_UART0_RR              (3U << 8)                       /*!< RW: priority UART0_RR */
#define C_MLX16_ITC_PRIO4_UART0_RR_PRIO3        (0U << 8)                       /*!< RW: priority 3 UART0_RR */
#define C_MLX16_ITC_PRIO4_UART0_RR_PRIO4        (1U << 8)                       /*!< RW: priority 4 UART0_RR */
#define C_MLX16_ITC_PRIO4_UART0_RR_PRIO5        (2U << 8)                       /*!< RW: priority 5 UART0_RR */
#define C_MLX16_ITC_PRIO4_UART0_RR_PRIO6        (3U << 8)                       /*!< RW: priority 6 UART0_RR */
#define M_MLX16_ITC_PRIO4_UART0_TS              (3U << 10)                      /*!< RW: priority UART0_TS */
#define C_MLX16_ITC_PRIO4_UART0_TS_PRIO3        (0U << 10)                      /*!< RW: priority 3 UART0_TS */
#define C_MLX16_ITC_PRIO4_UART0_TS_PRIO4        (1U << 10)                      /*!< RW: priority 4 UART0_TS */
#define C_MLX16_ITC_PRIO4_UART0_TS_PRIO5        (2U << 10)                      /*!< RW: priority 5 UART0_TS */
#define C_MLX16_ITC_PRIO4_UART0_TS_PRIO6        (3U << 10)                      /*!< RW: priority 6 UART0_TS */
#define M_MLX16_ITC_PRIO4_UART0_TR              (3U << 12)                      /*!< RW: priority UART0_TR */
#define C_MLX16_ITC_PRIO4_UART0_TR_PRIO3        (0U << 12)                      /*!< RW: priority 3 UART0_TR */
#define C_MLX16_ITC_PRIO4_UART0_TR_PRIO4        (1U << 12)                      /*!< RW: priority 4 UART0_TR */
#define C_MLX16_ITC_PRIO4_UART0_TR_PRIO5        (2U << 12)                      /*!< RW: priority 5 UART0_TR */
#define C_MLX16_ITC_PRIO4_UART0_TR_PRIO6        (3U << 12)                      /*!< RW: priority 6 UART0_TR */
#define M_MLX16_ITC_PRIO4_UART0_TE              (3U << 14)                      /*!< RW: priority UART0_TE */
#define C_MLX16_ITC_PRIO4_UART0_TE_PRIO3        (0U << 14)                      /*!< RW: priority 3 UART0_TE */
#define C_MLX16_ITC_PRIO4_UART0_TE_PRIO4        (1U << 14)                      /*!< RW: priority 4 UART0_TE */
#define C_MLX16_ITC_PRIO4_UART0_TE_PRIO5        (2U << 14)                      /*!< RW: priority 5 UART0_TE */
#define C_MLX16_ITC_PRIO4_UART0_TE_PRIO6        (3U << 14)                      /*!< RW: priority 6 UART0_TE */

extern volatile uint16_t IO_MLX16_ITC_PRIO5_S __attribute__((nodp, addr(0x00092)));  /*!< IO_MLX16_ITC_PRIO5_S (System) */
#define M_MLX16_ITC_PRIO5                       (65535U << 0)                   /*!< RW: Configure priority for interrupt[INT0_COUNT+INT1_COUNT+INT2_COUNT+47:INT0_COUNT+INT1_COUNT+INT2_COUNT+40] */
#define M_MLX16_ITC_PRIO5_UDFR0                 (3U << 0)                       /*!< RW: priority UDFR0 */
#define C_MLX16_ITC_PRIO5_UDFR0_PRIO3           (0U << 0)                       /*!< RW: priority 3 UDFR0 */
#define C_MLX16_ITC_PRIO5_UDFR0_PRIO4           (1U << 0)                       /*!< RW: priority 4 UDFR0 */
#define C_MLX16_ITC_PRIO5_UDFR0_PRIO5           (2U << 0)                       /*!< RW: priority 5 UDFR0 */
#define C_MLX16_ITC_PRIO5_UDFR0_PRIO6           (3U << 0)                       /*!< RW: priority 6 UDFR0 */
#define M_MLX16_ITC_PRIO5_UDFT0                 (3U << 2)                       /*!< RW: priority UDRF0 */
#define C_MLX16_ITC_PRIO5_UDFT0_PRIO3           (0U << 2)                       /*!< RW: priority 3 UDRF0 */
#define C_MLX16_ITC_PRIO5_UDFT0_PRIO4           (1U << 2)                       /*!< RW: priority 4 UDRF0 */
#define C_MLX16_ITC_PRIO5_UDFT0_PRIO5           (2U << 2)                       /*!< RW: priority 5 UDRF0 */
#define C_MLX16_ITC_PRIO5_UDFT0_PRIO6           (3U << 2)                       /*!< RW: priority 6 UDRF0 */
#define M_MLX16_ITC_PRIO5_UART1_SB              (3U << 4)                       /*!< RW: priority UART1_SB */
#define C_MLX16_ITC_PRIO5_UART1_SB_PRIO3        (0U << 4)                       /*!< RW: priority 3 UART1_SB */
#define C_MLX16_ITC_PRIO5_UART1_SB_PRIO4        (1U << 4)                       /*!< RW: priority 4 UART1_SB */
#define C_MLX16_ITC_PRIO5_UART1_SB_PRIO5        (2U << 4)                       /*!< RW: priority 5 UART1_SB */
#define C_MLX16_ITC_PRIO5_UART1_SB_PRIO6        (3U << 4)                       /*!< RW: priority 6 UART1_SB */
#define M_MLX16_ITC_PRIO5_UART1_RS              (3U << 6)                       /*!< RW: priority UART1_RE */
#define C_MLX16_ITC_PRIO5_UART1_RS_PRIO3        (0U << 6)                       /*!< RW: priority 3 UART1_RE */
#define C_MLX16_ITC_PRIO5_UART1_RS_PRIO4        (1U << 6)                       /*!< RW: priority 4 UART1_RE */
#define C_MLX16_ITC_PRIO5_UART1_RS_PRIO5        (2U << 6)                       /*!< RW: priority 5 UART1_RE */
#define C_MLX16_ITC_PRIO5_UART1_RS_PRIO6        (3U << 6)                       /*!< RW: priority 6 UART1_RE */
#define M_MLX16_ITC_PRIO5_UART1_RR              (3U << 8)                       /*!< RW: priority UART1_RR */
#define C_MLX16_ITC_PRIO5_UART1_RR_PRIO3        (0U << 8)                       /*!< RW: priority 3 UART1_RR */
#define C_MLX16_ITC_PRIO5_UART1_RR_PRIO4        (1U << 8)                       /*!< RW: priority 4 UART1_RR */
#define C_MLX16_ITC_PRIO5_UART1_RR_PRIO5        (2U << 8)                       /*!< RW: priority 5 UART1_RR */
#define C_MLX16_ITC_PRIO5_UART1_RR_PRIO6        (3U << 8)                       /*!< RW: priority 6 UART1_RR */
#define M_MLX16_ITC_PRIO5_UART1_TS              (3U << 10)                      /*!< RW: priority UART1_TS */
#define C_MLX16_ITC_PRIO5_UART1_TS_PRIO3        (0U << 10)                      /*!< RW: priority 3 UART1_TS */
#define C_MLX16_ITC_PRIO5_UART1_TS_PRIO4        (1U << 10)                      /*!< RW: priority 4 UART1_TS */
#define C_MLX16_ITC_PRIO5_UART1_TS_PRIO5        (2U << 10)                      /*!< RW: priority 5 UART1_TS */
#define C_MLX16_ITC_PRIO5_UART1_TS_PRIO6        (3U << 10)                      /*!< RW: priority 6 UART1_TS */
#define M_MLX16_ITC_PRIO5_UART1_TR              (3U << 12)                      /*!< RW: priority UART1_TR */
#define C_MLX16_ITC_PRIO5_UART1_TR_PRIO3        (0U << 12)                      /*!< RW: priority 3 UART1_TR */
#define C_MLX16_ITC_PRIO5_UART1_TR_PRIO4        (1U << 12)                      /*!< RW: priority 4 UART1_TR */
#define C_MLX16_ITC_PRIO5_UART1_TR_PRIO5        (2U << 12)                      /*!< RW: priority 5 UART1_TR */
#define C_MLX16_ITC_PRIO5_UART1_TR_PRIO6        (3U << 12)                      /*!< RW: priority 6 UART1_TR */
#define M_MLX16_ITC_PRIO5_UART1_TE              (3U << 14)                      /*!< RW: priority UART1_TE */
#define C_MLX16_ITC_PRIO5_UART1_TE_PRIO3        (0U << 14)                      /*!< RW: priority 3 UART1_TE */
#define C_MLX16_ITC_PRIO5_UART1_TE_PRIO4        (1U << 14)                      /*!< RW: priority 4 UART1_TE */
#define C_MLX16_ITC_PRIO5_UART1_TE_PRIO5        (2U << 14)                      /*!< RW: priority 5 UART1_TE */
#define C_MLX16_ITC_PRIO5_UART1_TE_PRIO6        (3U << 14)                      /*!< RW: priority 6 UART1_TE */

extern volatile uint16_t IO_MLX16_ITC_PRIO6_S __attribute__((nodp, addr(0x00094)));  /*!< IO_MLX16_ITC_PRIO6_S (System) */
#define M_MLX16_ITC_PRIO6                       (65535U << 0)                   /*!< RW: Configure priority for interrupt[INT0_COUNT+INT1_COUNT+INT2_COUNT+55:INT0_COUNT+INT1_COUNT+INT2_COUNT+48] */
#define M_MLX16_ITC_PRIO6_UDFR1                 (3U << 0)                       /*!< RW: priority UDFR1 */
#define C_MLX16_ITC_PRIO6_UDFR1_PRIO3           (0U << 0)                       /*!< RW: priority 3 UDFR1 */
#define C_MLX16_ITC_PRIO6_UDFR1_PRIO4           (1U << 0)                       /*!< RW: priority 4 UDFR1 */
#define C_MLX16_ITC_PRIO6_UDFR1_PRIO5           (2U << 0)                       /*!< RW: priority 5 UDFR1 */
#define C_MLX16_ITC_PRIO6_UDFR1_PRIO6           (3U << 0)                       /*!< RW: priority 6 UDFR1 */
#define M_MLX16_ITC_PRIO6_UDFT1                 (3U << 2)                       /*!< RW: priority UDRF1 */
#define C_MLX16_ITC_PRIO6_UDFT1_PRIO3           (0U << 2)                       /*!< RW: priority 3 UDRF1 */
#define C_MLX16_ITC_PRIO6_UDFT1_PRIO4           (1U << 2)                       /*!< RW: priority 4 UDRF1 */
#define C_MLX16_ITC_PRIO6_UDFT1_PRIO5           (2U << 2)                       /*!< RW: priority 5 UDRF1 */
#define C_MLX16_ITC_PRIO6_UDFT1_PRIO6           (3U << 2)                       /*!< RW: priority 6 UDRF1 */
#define M_MLX16_ITC_PRIO6_IO_IN0                (3U << 4)                       /*!< RW: priority IO_IN0 */
#define C_MLX16_ITC_PRIO6_IO_IN0_PRIO3          (0U << 4)                       /*!< RW: priority 3 IO_IN0 */
#define C_MLX16_ITC_PRIO6_IO_IN0_PRIO4          (1U << 4)                       /*!< RW: priority 4 IO_IN0 */
#define C_MLX16_ITC_PRIO6_IO_IN0_PRIO5          (2U << 4)                       /*!< RW: priority 5 IO_IN0 */
#define C_MLX16_ITC_PRIO6_IO_IN0_PRIO6          (3U << 4)                       /*!< RW: priority 6 IO_IN0 */
#define M_MLX16_ITC_PRIO6_IO_IN1                (3U << 6)                       /*!< RW: priority IO_IN1 */
#define C_MLX16_ITC_PRIO6_IO_IN1_PRIO3          (0U << 6)                       /*!< RW: priority 3 IO_IN1 */
#define C_MLX16_ITC_PRIO6_IO_IN1_PRIO4          (1U << 6)                       /*!< RW: priority 4 IO_IN1 */
#define C_MLX16_ITC_PRIO6_IO_IN1_PRIO5          (2U << 6)                       /*!< RW: priority 5 IO_IN1 */
#define C_MLX16_ITC_PRIO6_IO_IN1_PRIO6          (3U << 6)                       /*!< RW: priority 6 IO_IN1 */
#define M_MLX16_ITC_PRIO6_IO_IN2                (3U << 8)                       /*!< RW: priority IO_IN2 */
#define C_MLX16_ITC_PRIO6_IO_IN2_PRIO3          (0U << 8)                       /*!< RW: priority 3 IO_IN2 */
#define C_MLX16_ITC_PRIO6_IO_IN2_PRIO4          (1U << 8)                       /*!< RW: priority 4 IO_IN2 */
#define C_MLX16_ITC_PRIO6_IO_IN2_PRIO5          (2U << 8)                       /*!< RW: priority 5 IO_IN2 */
#define C_MLX16_ITC_PRIO6_IO_IN2_PRIO6          (3U << 8)                       /*!< RW: priority 6 IO_IN2 */
#define M_MLX16_ITC_PRIO6_IO_IN3                (3U << 10)                      /*!< RW: priority IO_IN3 */
#define C_MLX16_ITC_PRIO6_IO_IN3_PRIO3          (0U << 10)                      /*!< RW: priority 3 IO_IN3 */
#define C_MLX16_ITC_PRIO6_IO_IN3_PRIO4          (1U << 10)                      /*!< RW: priority 4 IO_IN3 */
#define C_MLX16_ITC_PRIO6_IO_IN3_PRIO5          (2U << 10)                      /*!< RW: priority 5 IO_IN3 */
#define C_MLX16_ITC_PRIO6_IO_IN3_PRIO6          (3U << 10)                      /*!< RW: priority 6 IO_IN3 */
#define M_MLX16_ITC_PRIO6_IO_IN4                (3U << 12)                      /*!< RW: priority IO_IN4 */
#define C_MLX16_ITC_PRIO6_IO_IN4_PRIO3          (0U << 12)                      /*!< RW: priority 3 IO_IN4 */
#define C_MLX16_ITC_PRIO6_IO_IN4_PRIO4          (1U << 12)                      /*!< RW: priority 4 IO_IN4 */
#define C_MLX16_ITC_PRIO6_IO_IN4_PRIO5          (2U << 12)                      /*!< RW: priority 5 IO_IN4 */
#define C_MLX16_ITC_PRIO6_IO_IN4_PRIO6          (3U << 12)                      /*!< RW: priority 6 IO_IN4 */
#define M_MLX16_ITC_PRIO6_IO_IN5                (3U << 14)                      /*!< RW: priority IO_IN5 */
#define C_MLX16_ITC_PRIO6_IO_IN5_PRIO3          (0U << 14)                      /*!< RW: priority 3 IO_IN5 */
#define C_MLX16_ITC_PRIO6_IO_IN5_PRIO4          (1U << 14)                      /*!< RW: priority 4 IO_IN5 */
#define C_MLX16_ITC_PRIO6_IO_IN5_PRIO5          (2U << 14)                      /*!< RW: priority 5 IO_IN5 */
#define C_MLX16_ITC_PRIO6_IO_IN5_PRIO6          (3U << 14)                      /*!< RW: priority 6 IO_IN5 */

extern volatile uint16_t IO_MLX16_ITC_PRIO7_S __attribute__((nodp, addr(0x00096)));  /*!< IO_MLX16_ITC_PRIO7_S (System) */
#define M_MLX16_ITC_PRIO7                       (65535U << 0)                   /*!< RW: Configure priority for interrupt[INT0_COUNT+INT1_COUNT+INT2_COUNT+63:INT0_COUNT+INT1_COUNT+INT2_COUNT+56] */
#define M_MLX16_ITC_PRIO7_IO_IN6                (3U << 0)                       /*!< RW: priority IO_IN6 */
#define C_MLX16_ITC_PRIO7_IO_IN6_PRIO3          (0U << 0)                       /*!< RW: priority 3 IO_IN6 */
#define C_MLX16_ITC_PRIO7_IO_IN6_PRIO4          (1U << 0)                       /*!< RW: priority 4 IO_IN6 */
#define C_MLX16_ITC_PRIO7_IO_IN6_PRIO5          (2U << 0)                       /*!< RW: priority 5 IO_IN6 */
#define C_MLX16_ITC_PRIO7_IO_IN6_PRIO6          (3U << 0)                       /*!< RW: priority 6 IO_IN6 */
#define M_MLX16_ITC_PRIO7_IO_IN7                (3U << 2)                       /*!< RW: priority IO_IN7 */
#define C_MLX16_ITC_PRIO7_IO_IN7_PRIO3          (0U << 2)                       /*!< RW: priority 3 IO_IN7 */
#define C_MLX16_ITC_PRIO7_IO_IN7_PRIO4          (1U << 2)                       /*!< RW: priority 4 IO_IN7 */
#define C_MLX16_ITC_PRIO7_IO_IN7_PRIO5          (2U << 2)                       /*!< RW: priority 5 IO_IN7 */
#define C_MLX16_ITC_PRIO7_IO_IN7_PRIO6          (3U << 2)                       /*!< RW: priority 6 IO_IN7 */
#define M_MLX16_ITC_PRIO7_IO_IN8                (3U << 4)                       /*!< RW: priority IO_IN8 */
#define C_MLX16_ITC_PRIO7_IO_IN8_PRIO3          (0U << 4)                       /*!< RW: priority 3 IO_IN8 */
#define C_MLX16_ITC_PRIO7_IO_IN8_PRIO4          (1U << 4)                       /*!< RW: priority 4 IO_IN8 */
#define C_MLX16_ITC_PRIO7_IO_IN8_PRIO5          (2U << 4)                       /*!< RW: priority 5 IO_IN8 */
#define C_MLX16_ITC_PRIO7_IO_IN8_PRIO6          (3U << 4)                       /*!< RW: priority 6 IO_IN8 */
#define M_MLX16_ITC_PRIO7_IO_IN9                (3U << 6)                       /*!< RW: priority IO_IN9 */
#define C_MLX16_ITC_PRIO7_IO_IN9_PRIO3          (0U << 6)                       /*!< RW: priority 3 IO_IN9 */
#define C_MLX16_ITC_PRIO7_IO_IN9_PRIO4          (1U << 6)                       /*!< RW: priority 4 IO_IN9 */
#define C_MLX16_ITC_PRIO7_IO_IN9_PRIO5          (2U << 6)                       /*!< RW: priority 5 IO_IN9 */
#define C_MLX16_ITC_PRIO7_IO_IN9_PRIO6          (3U << 6)                       /*!< RW: priority 6 IO_IN9 */
#define M_MLX16_ITC_PRIO7_IO_IN10               (3U << 8)                       /*!< RW: priority IO_IN10 */
#define C_MLX16_ITC_PRIO7_IO_IN10_PRIO3         (0U << 8)                       /*!< RW: priority 3 IO_IN10 */
#define C_MLX16_ITC_PRIO7_IO_IN10_PRIO4         (1U << 8)                       /*!< RW: priority 4 IO_IN10 */
#define C_MLX16_ITC_PRIO7_IO_IN10_PRIO5         (2U << 8)                       /*!< RW: priority 5 IO_IN10 */
#define C_MLX16_ITC_PRIO7_IO_IN10_PRIO6         (3U << 8)                       /*!< RW: priority 6 IO_IN10 */
#define M_MLX16_ITC_PRIO7_IO_IN11               (3U << 10)                      /*!< RW: priority IO_IN11 */
#define C_MLX16_ITC_PRIO7_IO_IN11_PRIO3         (0U << 10)                      /*!< RW: priority 3 IO_IN11 */
#define C_MLX16_ITC_PRIO7_IO_IN11_PRIO4         (1U << 10)                      /*!< RW: priority 4 IO_IN11 */
#define C_MLX16_ITC_PRIO7_IO_IN11_PRIO5         (2U << 10)                      /*!< RW: priority 5 IO_IN11 */
#define C_MLX16_ITC_PRIO7_IO_IN11_PRIO6         (3U << 10)                      /*!< RW: priority 6 IO_IN11 */
#define M_MLX16_ITC_PRIO7_PH_IN0                (3U << 12)                      /*!< RW: priority PH_IN0 */
#define C_MLX16_ITC_PRIO7_PH_IN0_PRIO3          (0U << 12)                      /*!< RW: priority 3 PH_IN0 */
#define C_MLX16_ITC_PRIO7_PH_IN0_PRIO4          (1U << 12)                      /*!< RW: priority 4 PH_IN0 */
#define C_MLX16_ITC_PRIO7_PH_IN0_PRIO5          (2U << 12)                      /*!< RW: priority 5 PH_IN0 */
#define C_MLX16_ITC_PRIO7_PH_IN0_PRIO6          (3U << 12)                      /*!< RW: priority 6 PH_IN0 */
#define M_MLX16_ITC_PRIO7_PH_IN1                (3U << 14)                      /*!< RW: priority PH_IN1 */
#define C_MLX16_ITC_PRIO7_PH_IN1_PRIO3          (0U << 14)                      /*!< RW: priority 3 PH_IN1 */
#define C_MLX16_ITC_PRIO7_PH_IN1_PRIO4          (1U << 14)                      /*!< RW: priority 4 PH_IN1 */
#define C_MLX16_ITC_PRIO7_PH_IN1_PRIO5          (2U << 14)                      /*!< RW: priority 5 PH_IN1 */
#define C_MLX16_ITC_PRIO7_PH_IN1_PRIO6          (3U << 14)                      /*!< RW: priority 6 PH_IN1 */

extern volatile uint16_t IO_MLX16_ITC_PRIO8_S __attribute__((nodp, addr(0x00098)));  /*!< IO_MLX16_ITC_PRIO8_S (System) */
#define M_MLX16_ITC_PRIO8                       (1023U << 0)                    /*!< RW: Configure priority for interrupt[INT0_COUNT+INT1_COUNT+INT2_COUNT+63:INT0_COUNT+INT1_COUNT+INT2_COUNT+56] */
#define M_MLX16_ITC_PRIO8_PH_IN2                (3U << 0)                       /*!< RW: priority PH_IN2 */
#define C_MLX16_ITC_PRIO8_PH_IN2_PRIO3          (0U << 0)                       /*!< RW: priority 3 PH_IN2 */
#define C_MLX16_ITC_PRIO8_PH_IN2_PRIO4          (1U << 0)                       /*!< RW: priority 4 PH_IN2 */
#define C_MLX16_ITC_PRIO8_PH_IN2_PRIO5          (2U << 0)                       /*!< RW: priority 5 PH_IN2 */
#define C_MLX16_ITC_PRIO8_PH_IN2_PRIO6          (3U << 0)                       /*!< RW: priority 6 PH_IN2 */
#define M_MLX16_ITC_PRIO8_PH_IN3                (3U << 2)                       /*!< RW: priority PH_IN3 */
#define C_MLX16_ITC_PRIO8_PH_IN3_PRIO3          (0U << 2)                       /*!< RW: priority 3 PH_IN3 */
#define C_MLX16_ITC_PRIO8_PH_IN3_PRIO4          (1U << 2)                       /*!< RW: priority 4 PH_IN3 */
#define C_MLX16_ITC_PRIO8_PH_IN3_PRIO5          (2U << 2)                       /*!< RW: priority 5 PH_IN3 */
#define C_MLX16_ITC_PRIO8_PH_IN3_PRIO6          (3U << 2)                       /*!< RW: priority 6 PH_IN3 */
#define M_MLX16_ITC_PRIO8_PH_IN4                (3U << 4)                       /*!< RW: priority PH_IN4 */
#define C_MLX16_ITC_PRIO8_PH_IN4_PRIO3          (0U << 4)                       /*!< RW: priority 3 PH_IN4 */
#define C_MLX16_ITC_PRIO8_PH_IN4_PRIO4          (1U << 4)                       /*!< RW: priority 4 PH_IN4 */
#define C_MLX16_ITC_PRIO8_PH_IN4_PRIO5          (2U << 4)                       /*!< RW: priority 5 PH_IN4 */
#define C_MLX16_ITC_PRIO8_PH_IN4_PRIO6          (3U << 4)                       /*!< RW: priority 6 PH_IN4 */
#define M_MLX16_ITC_PRIO8_PH_IN5                (3U << 6)                       /*!< RW: priority PH_IN5 */
#define C_MLX16_ITC_PRIO8_PH_IN5_PRIO3          (0U << 6)                       /*!< RW: priority 3 PH_IN5 */
#define C_MLX16_ITC_PRIO8_PH_IN5_PRIO4          (1U << 6)                       /*!< RW: priority 4 PH_IN5 */
#define C_MLX16_ITC_PRIO8_PH_IN5_PRIO5          (2U << 6)                       /*!< RW: priority 5 PH_IN5 */
#define C_MLX16_ITC_PRIO8_PH_IN5_PRIO6          (3U << 6)                       /*!< RW: priority 6 PH_IN5 */
#define M_MLX16_ITC_PRIO8_I2C_MST               (3U << 8)                       /*!< RW: priority I2C Master */
#define C_MLX16_ITC_PRIO8_I2C_MST_PRIO3         (0U << 8)                       /*!< RW: priority 3 I2C Master */
#define C_MLX16_ITC_PRIO8_I2C_MST_PRIO4         (1U << 8)                       /*!< RW: priority 4 I2C Master */
#define C_MLX16_ITC_PRIO8_I2C_MST_PRIO5         (2U << 8)                       /*!< RW: priority 5 I2C Master */
#define C_MLX16_ITC_PRIO8_I2C_MST_PRIO6         (3U << 8)                       /*!< RW: priority 6 I2C Master */

extern volatile uint16_t IO_MLX16_SHELL_VER __attribute__((nodp, addr(0x000BE)));  /*!< IO_MLX16_SHELL_VER */
#define M_MLX16_SHELL_VERSION                   (65535U << 0)                   /*!< R: Define the shell version */

extern volatile uint16_t IO_MLX16_CPU_FP0ADR_S __attribute__((nodp, addr(0x000C0)));  /*!< IO_MLX16_CPU_FP0ADR_S (System) */
#define M_MLX16_CPU_FP0ADR                      (4095U << 0)                    /*!< RW: Define the address of the far page 0 */

/* ************** */
/* Block: VERSION */
/* ************** */
extern volatile uint16_t IO_VERSION_L __attribute__((nodp, addr(0x00100)));     /*!< IO_VERSION_L */
#define M_VERSION_VERSION_L                     (65535U << 0)                   /*!< R: The 16 low significant bit of the version number. */

extern volatile uint16_t IO_VERSION_H __attribute__((nodp, addr(0x00102)));     /*!< IO_VERSION_H */
#define M_VERSION_VERSION_H                     (65535U << 0)                   /*!< R: The 16 high significant bit of the version number */

/* *************** */
/* Block: RAM_BIST */
/* *************** */
extern volatile uint16_t IO_RAM_BIST __attribute__((nodp, addr(0x00104)));      /*!< IO_RAM_BIST */
#define B_RAM_BIST_COMPLETED                    (1U << 15)                      /*!< R: 0: BIST not completed; 1: BIST completed */
#define B_RAM_BIST_REGULAR_BIST_ERROR           (1U << 12)                      /*!< R: 0: No error during a regular BIST; 1: An error occurred during a regular BIST */
#define B_RAM_BIST_VALID_CLOCK                  (1U << 11)                      /*!< R: 0: No valid application clock present for the BIST; 1: Valid application clock present for the BIST */
#define B_RAM_BIST_RUNNING                      (1U << 10)                      /*!< R: 0: No BIST running; 1: BIST running */
#define B_RAM_BIST_REGULAR                      (1U << 9)                       /*!< R: 0: No regular BIST requested; 1: Regular BIST requested */
#define B_RAM_BIST_TRANSPARENT                  (1U << 8)                       /*!< R: 0: No transparent BIST requested; 1: Transparent BIST requested; W: BIST request: KEY = 0x0A6E --> Transparent BIST request KEY = 0x0F59 --> Regular BIST request Any other value generates an invalid access */
#define M_RAM_BIST_KEY                          (65535U << 0)                   /*!< W: BIST request: KEY = 0x0A6E --> Transparent BIST request KEY = 0x0F59 --> Regular BIST request Any other value generates an invalid access; R: Regular BIST: Algorithm phase of the first error found */
#define M_RAM_BIST_PHASE                        (7U << 0)                       /*!< R: Regular BIST: Algorithm phase of the first error found */

extern volatile uint16_t IO_RAM_BIST_SIGN_DATA __attribute__((nodp, addr(0x00106)));  /*!< IO_RAM_BIST_SIGN_DATA */
#define M_RAM_BIST_LFSR                         (65535U << 0)                   /*!< R: Transparent BIST: Signature */
#define M_RAM_BIST_LSFR_GOT                     (65535U << 0)                   /*!< R: Regular BIST: Got Data */

extern volatile uint16_t IO_RAM_BIST_LFSR_EXPECT __attribute__((nodp, addr(0x00108)));  /*!< IO_RAM_BIST_LFSR_EXPECT */
#define M_RAM_BIST_LFSR_EXPECTED                (65535U << 0)                   /*!< R: Regular BIST: Expected Data */

extern volatile uint16_t IO_RAM_BIST_ADL __attribute__((nodp, addr(0x0010A)));  /*!< IO_RAM_BIST_ADL */
#define M_RAM_BIST_ADL                          (65535U << 0)                   /*!< R: Regular BIST: Address of error (16 LSBs) */

extern volatile uint16_t IO_RAM_BIST_ADH __attribute__((nodp, addr(0x0010C)));  /*!< IO_RAM_BIST_ADH */
#define M_RAM_BIST_ADH                          (15U << 0)                      /*!< R: Regular BIST: Address of error (4 MSBs) */

extern volatile uint16_t IO_RAM_BIST_ADD_START_L __attribute__((nodp, addr(0x0010E)));  /*!< IO_RAM_BIST_ADD_START_L */
#define M_RAM_BIST_ADD_START_L                  (65535U << 0)                   /*!< RW: First address accessed in the BIST (16 LSB) */

extern volatile uint16_t IO_RAM_BIST_ADD_START_H __attribute__((nodp, addr(0x00110)));  /*!< IO_RAM_BIST_ADD_START_H */
#define M_RAM_BIST_ADD_START_H                  (15U << 0)                      /*!< RW: First address accessed in the BIST (4 MSB) */

extern volatile uint16_t IO_RAM_BIST_ADD_STOP_L __attribute__((nodp, addr(0x00112)));  /*!< IO_RAM_BIST_ADD_STOP_L */
#define M_RAM_BIST_ADD_STOP_L                   (65535U << 0)                   /*!< RW: Last address accessed in the BIST (16 LSB) */

extern volatile uint16_t IO_RAM_BIST_ADD_STOP_H __attribute__((nodp, addr(0x00114)));  /*!< IO_RAM_BIST_ADD_STOP_H */
#define M_RAM_BIST_ADD_STOP_H                   (15U << 0)                      /*!< RW: Last address accessed in the BIST (4 MSB) */

extern volatile uint16_t IO_RAM_BIST_CTRL __attribute__((nodp, addr(0x00116)));  /*!< IO_RAM_BIST_CTRL */
#define B_RAM_BIST_SIGNATURE_INIT               (1U << 12)                      /*!< RW: 0: Signature initialised with 0x0000; 1: Signature initialised with 0xFFFF */
#define M_RAM_BIST_ADD_SCRAMBLE                 (7U << 8)                       /*!< RW: Indicate the number of bits used for address scrambling (only 2,3,4 or 5 is supported); Note: value 2 keep the address unchanged */
#define B_RAM_BIST_FUNCTIONAL_BIST              (1U << 6)                       /*!< RW: Indicate if we perform a functional BIST */
#define B_RAM_BIST_WORD_BIST                    (1U << 5)                       /*!< RW: 0: BIST in byte; 1: BIST in word */
#define M_RAM_BIST_NB_ECC_BITS                  (31U << 0)                      /*!< RW: Indicate the number of ECC bits of the RAM tested */

/* ************** */
/* Block: CTIMER0 */
/* ************** */
extern volatile uint16_t IO_CTIMER0_TREGB __attribute__((nodp, addr(0x00118)));  /*!< IO_CTIMER0_TREGB */
#define M_CTIMER0_TREGB                         (65535U << 0)                   /*!< RW: Timer Channel B */

extern volatile uint16_t IO_CTIMER0_TREGA __attribute__((nodp, addr(0x0011A)));  /*!< IO_CTIMER0_TREGA */
#define M_CTIMER0_TREGA                         (65535U << 0)                   /*!< RW: Timer Channel A */

extern volatile uint16_t IO_CTIMER0_TCNT __attribute__((nodp, addr(0x0011C)));  /*!< IO_CTIMER0_TCNT */
#define M_CTIMER0_TCNT                          (65535U << 0)                   /*!< R: Counter value */

extern volatile uint16_t IO_CTIMER0_CTRL __attribute__((nodp, addr(0x0011E)));  /*!< IO_CTIMER0_CTRL */
#define M_CTIMER0_DIV                           (3U << 14)                      /*!< RW: */
#define C_CTIMER0_DIV_CPU                       (0U << 14)                      /*!< 00 = Timer Clock = CPU Clock */
#define C_CTIMER0_DIV_CPU_DIV_16                (1U << 14)                      /*!< 01 = Timer Clock = CPU Clock/16 */
#define C_CTIMER0_DIV_CPU_DIV_256               (2U << 14)                      /*!< 1X = Timer Clock = CPU Clock/256 */
#define M_CTIMER0_MODE                          (7U << 11)                      /*!< RW: Timer mode */
#define C_CTIMER0_MODE_TIMER                    (0U << 11)                      /*!< Mode 0: Single 16-bits auto-reload timer */
#define C_CTIMER0_MODE_DUAL_COMP                (1U << 11)                      /*!< Mode 1: Dual 16-bits timer compare */
#define C_CTIMER0_MODE_DUAL_CAPT                (2U << 11)                      /*!< Mode 2: Dual 16-bits timer capture */
#define C_CTIMER0_MODE_COMP_CAPT                (3U << 11)                      /*!< Mode 3: 16-bits timer compare and capture */
#define C_CTIMER0_MODE_PWM                      (6U << 11)                      /*!< Mode 4: High resolution 16-bits PWM (no shadow register) */
#define B_CTIMER0_ENCMP                         (1U << 10)                      /*!< RW: Enable reset control for the 16 bits up counter */
#define B_CTIMER0_OVRB                          (1U << 9)                       /*!< R: Overrun interrupt signal, clear after reading */
#define B_CTIMER0_OVRA                          (1U << 8)                       /*!< R: Overrun interrupt signal, clear after reading */
#define B_CTIMER0_POL                           (1U << 7)                       /*!< RW: Define the polarity of the PWM */
#define B_CTIMER0_PWMI                          (1U << 6)                       /*!< R: Read back PWM output; W: Do nothing */
#define M_CTIMER0_EDGB                          (3U << 4)                       /*!< RW: Select edge sensitivity on input channel B */
#define C_CTIMER0_EDGB_FALLING                  (1U << 4)                       /*!< RW: CTimer0 FALLING Edge B */
#define C_CTIMER0_EDGB_RAISING                  (2U << 4)                       /*!< RW: CTimer0 RAISING Edge B */
#define C_CTIMER0_EDGB_BOTH                     (3U << 4)                       /*!< RW: CTimer0 BOTH Edge B */
#define M_CTIMER0_EDGA                          (3U << 2)                       /*!< RW: Select edge sensitivity on input channel A */
#define C_CTIMER0_EDGA_FALLING                  (1U << 2)                       /*!< RW: CTimer0 FALLING Edge A */
#define C_CTIMER0_EDGA_RAISING                  (2U << 2)                       /*!< RW: CTimer0 RAISING Edge A */
#define C_CTIMER0_EDGA_BOTH                     (3U << 2)                       /*!< RW: CTimer0 BOTH Edge A */
#define B_CTIMER0_STOP                          (1U << 1)                       /*!< W: 1:Stop the complex timer, all other bits are discarded 0: No effect; R: 1: Complex timer is stopped 0: Complex timer is running */
#define B_CTIMER0_START                         (1U << 0)                       /*!< W: 1:Start the complex timer, all other bits are discarded 0: No effect; R: 1: Complex timer is running 0: Complex timer is stopped */

/* ************** */
/* Block: CTIMER1 */
/* ************** */
extern volatile uint16_t IO_CTIMER1_TREGB __attribute__((nodp, addr(0x00120)));  /*!< IO_CTIMER1_TREGB */
#define M_CTIMER1_TREGB                         (65535U << 0)                   /*!< RW: Timer Channel B */

extern volatile uint16_t IO_CTIMER1_TREGA __attribute__((nodp, addr(0x00122)));  /*!< IO_CTIMER1_TREGA */
#define M_CTIMER1_TREGA                         (65535U << 0)                   /*!< RW: Timer Channel A */

extern volatile uint16_t IO_CTIMER1_TCNT __attribute__((nodp, addr(0x00124)));  /*!< IO_CTIMER1_TCNT */
#define M_CTIMER1_TCNT                          (65535U << 0)                   /*!< R: Counter value */

extern volatile uint16_t IO_CTIMER1_CTRL __attribute__((nodp, addr(0x00126)));  /*!< IO_CTIMER1_CTRL */
#define M_CTIMER1_DIV                           (3U << 14)                      /*!< RW: */
#define C_CTIMER1_DIV_CPU                       (0U << 14)                      /*!< 00 = Timer Clock = CPU Clock */
#define C_CTIMER1_DIV_CPU_DIV_16                (1U << 14)                      /*!< 01 = Timer Clock = CPU Clock/16 */
#define C_CTIMER1_DIV_CPU_DIV_256               (2U << 14)                      /*!< 1X = Timer Clock = CPU Clock/256 */
#define M_CTIMER1_MODE                          (7U << 11)                      /*!< RW: Timer mode */
#define C_CTIMER1_MODE_TIMER                    (0U << 11)                      /*!< Mode 0: Single 16-bits auto-reload timer */
#define C_CTIMER1_MODE_DUAL_COMP                (1U << 11)                      /*!< Mode 1: Dual 16-bits timer compare */
#define C_CTIMER1_MODE_DUAL_CAPT                (2U << 11)                      /*!< Mode 2: Dual 16-bits timer capture */
#define C_CTIMER1_MODE_COMP_CAPT                (3U << 11)                      /*!< Mode 3: 16-bits timer compare and capture */
#define C_CTIMER1_MODE_PWM                      (6U << 11)                      /*!< Mode 4: High resolution 16-bits PWM (no shadow register) */
#define B_CTIMER1_ENCMP                         (1U << 10)                      /*!< RW: Enable reset control for the 16 bits up counter */
#define B_CTIMER1_OVRB                          (1U << 9)                       /*!< R: Overrun interrupt signal, clear after reading */
#define B_CTIMER1_OVRA                          (1U << 8)                       /*!< R: Overrun interrupt signal, clear after reading */
#define B_CTIMER1_POL                           (1U << 7)                       /*!< RW: Define the polarity of the PWM */
#define B_CTIMER1_PWMI                          (1U << 6)                       /*!< R: Read back PWM output; W: Do nothing */
#define M_CTIMER1_EDGB                          (3U << 4)                       /*!< RW: Select edge sensitivity on input channel B */
#define C_CTIMER1_EDGB_FALLING                  (1U << 4)                       /*!< RW: CTimer1 FALLING Edge B */
#define C_CTIMER1_EDGB_RAISING                  (2U << 4)                       /*!< RW: CTimer1 RAISING Edge B */
#define C_CTIMER1_EDGB_BOTH                     (3U << 4)                       /*!< RW: CTimer1 BOTH Edge B */
#define M_CTIMER1_EDGA                          (3U << 2)                       /*!< RW: Select edge sensitivity on input channel A */
#define C_CTIMER1_EDGA_FALLING                  (1U << 2)                       /*!< RW: CTimer1 FALLING Edge A */
#define C_CTIMER1_EDGA_RAISING                  (2U << 2)                       /*!< RW: CTimer1 RAISING Edge A */
#define C_CTIMER1_EDGA_BOTH                     (3U << 2)                       /*!< RW: CTimer1 BOTH Edge A */
#define B_CTIMER1_STOP                          (1U << 1)                       /*!< W: 1:Stop the complex timer, all other bits are discarded 0: No effect; R: 1: Complex timer is stopped 0: Complex timer is running */
#define B_CTIMER1_START                         (1U << 0)                       /*!< W: 1:Start the complex timer, all other bits are discarded 0: No effect; R: 1: Complex timer is running 0: Complex timer is stopped */

/* ********** */
/* Block: SPI */
/* ********** */
extern volatile uint16_t IO_SPI_DR __attribute__((nodp, addr(0x00128)));        /*!< IO_SPI_DR */
extern volatile uint8_t IO_SPI_DR_BY __attribute__((nodp, addr(0x00128)));      /*!< IO_SPI_DR_BY */
#define M_SPI_DR                                (65535U << 0)                   /*!< W: Write to transmit register and Send data; R: Read the receive register */

extern volatile uint16_t IO_SPI_SBASE __attribute__((nodp, addr(0x0012A)));     /*!< IO_SPI_SBASE */
#define M_SPI_SBASE                             (65535U << 0)                   /*!< RW: Base address of data to send */

extern volatile uint16_t IO_SPI_RBASE __attribute__((nodp, addr(0x0012C)));     /*!< IO_SPI_RBASE */
#define M_SPI_RBASE                             (65535U << 0)                   /*!< RW: Base address of received data */

extern volatile uint16_t IO_SPI_MSGLEN __attribute__((nodp, addr(0x0012E)));    /*!< IO_SPI_MSGLEN */
#define M_SPI_MSGLEN                            (63U << 0)                      /*!< RW: Number of "Words" exchanged in DMA (0 means 64"Words") */

extern volatile uint16_t IO_SPI_CLOCK __attribute__((nodp, addr(0x00130)));     /*!< IO_SPI_CLOCK */
#define M_SPI_PSCLM                             (15U << 4)                      /*!< RW: Divide MCU_CLK frequency by (PSCLM + 1) */
#define M_SPI_PSCLN                             (15U << 0)                      /*!< RW: Divide MCU_CLK frequency by 2PSCLN, with 0 <= PSCLN <= 11 */

extern volatile uint16_t IO_SPI_CTRL __attribute__((nodp, addr(0x00132)));      /*!< IO_SPI_CTRL */
#define B_SPI_MODF                              (1U << 15)                      /*!< R: Mode fault error; W: Write a 1 to clear bit */
#define B_SPI_OVRF                              (1U << 14)                      /*!< R: Receive overflow interrupt; W: Write a 1 to clear bit */
#define B_SPI_RF                                (1U << 13)                      /*!< R: SPI Receive register full */
#define B_SPI_TE                                (1U << 12)                      /*!< R: SPI Transmit register empty */
#define B_SPI_RX_EN                             (1U << 9)                       /*!< RW: Enable errors due to reception */
#define B_SPI_TX_EN                             (1U << 8)                       /*!< RW: Enable errors due to transmission */
#define B_SPI_DMA                               (1U << 7)                       /*!< RW: Enable DMA mode */
#define B_SPI_CPHA                              (1U << 6)                       /*!< RW: Clock phase: 0: Get data on the leading edge, set data on the trailing edge 1: Set data on the leading edge, get data on the trailing edge */
#define B_SPI_CPOL                              (1U << 5)                       /*!< RW: Clock polarity: 0: Clock wait state at 0, 1: Clock wait state at 1 */
#define B_SPI_BYTE_MODE                         (1U << 4)                       /*!< RW: "Word" size: 0: 16 bits 1: 8 bits */
#define B_SPI_WORD_MODE                         (0U << 4)                       /*!< RW: "Word" size: 0: 16 bits 1: 8 bits */
#define B_SPI_SS_FORCE                          (1U << 3)                       /*!< RW: SPI_SS behaviour: 0: SPI_SS = 1 between "Words", SPI_SS = 0 while transmitting 1: SPI_SS always at 0 */
#define B_SPI_MASTER                            (1U << 2)                       /*!< RW: Master/Slave: 0: Slave mode (reception); 1: Master mode (emission) */
#define B_SPI_SLAVE                             (0U << 2)                       /*!< RW: Master/Slave: 0: Slave mode (reception); 1: Master mode (emission) */
#define B_SPI_STOP                              (1U << 1)                       /*!< W: 1: Stop the SPI, all other bits are discarded 0: No effect; R: 0: SPI is running 1: SPI is stopped */
#define B_SPI_START                             (1U << 0)                       /*!< W: 1: Start the SPI, all other bits are discarded 0: No effect; R: 0: SPI is stopped 1: SPI is running */

/* ****************** */
/* Block: PWM_MASTER1 */
/* ****************** */
extern volatile uint16_t IO_PWM_MASTER1_CMP __attribute__((nodp, addr(0x00134)));  /*!< IO_PWM_MASTER1_CMP */
#define M_PWM_MASTER1_CMP                       (65535U << 0)                   /*!< RW: Interrupt comparator register */

extern volatile uint16_t IO_PWM_MASTER1_HT __attribute__((nodp, addr(0x00136)));  /*!< IO_PWM_MASTER1_HT */
#define M_PWM_MASTER1_HT                        (65535U << 0)                   /*!< RW: High threshold value */

extern volatile uint16_t IO_PWM_MASTER1_LT __attribute__((nodp, addr(0x00138)));  /*!< IO_PWM_MASTER1_LT */
#define M_PWM_MASTER1_LT                        (65535U << 0)                   /*!< RW: Low threshold value */

extern volatile uint16_t IO_PWM_MASTER1_PER __attribute__((nodp, addr(0x0013A)));  /*!< IO_PWM_MASTER1_PER */
#define M_PWM_MASTER1_PER                       (65535U << 0)                   /*!< RW: PWM period */

extern volatile uint16_t IO_PWM_MASTER1_CTRL __attribute__((nodp, addr(0x0013C)));  /*!< IO_PWM_MASTER1_CTRL */
#define M_PWM_MASTER1_PSCLM                     (15U << 12)                     /*!< RW: Divide input clock by M+1 */
#define M_PWM_MASTER1_PSCLN                     (15U << 8)                      /*!< RW: Divide input clock by 2^N, with N between 0 and 11 included. */
#define B_PWM_MASTER1_PWM_IN                    (1U << 7)                       /*!< R: Read back PWM output; W: Do nothing */
#define B_PWM_MASTER1_IDLE                      (1U << 6)                       /*!< RW: When PWM is stopped, 1: Output is invert of POL, 0: Output is POL */
#define B_PWM_MASTER1_POL                       (1U << 5)                       /*!< RW: 1: Output start at 1, 0: Output start at 0 */
#define M_PWM_MASTER1_MODE                      (3U << 3)                       /*!< RW: See below */
#define C_PWM_MASTER1_MODE_SIMPLE               (0U << 3)                       /*!< 00: Simple mode */
#define C_PWM_MASTER1_MODE_MIRROR               (1U << 3)                       /*!< 01: Mirror mode */
#define C_PWM_MASTER1_MODE_INDEPENDENT          (2U << 3)                       /*!< 10: Independent mode */
#define B_PWM_MASTER1_SLAVE                     (1U << 2)                       /*!< RW: 1: Slave mode, 0: Master mode */
#define B_PWM_MASTER1_STOP                      (1U << 1)                       /*!< W: 1 stop the PWM, all other bits are unused, 0 do nothing; R: 0 if the PWM is running else 1 */
#define B_PWM_MASTER1_START                     (1U << 0)                       /*!< W: 1 to start the PWM, all other bits are unused, 0 do nothing; R: 1 if the PWM is running else 0 */

/* ***************** */
/* Block: PWM_SLAVE1 */
/* ***************** */
extern volatile uint16_t IO_PWM_SLAVE1_CMP __attribute__((nodp, addr(0x0013E)));  /*!< IO_PWM_SLAVE1_CMP */
#define M_PWM_SLAVE1_CMP                        (65535U << 0)                   /*!< RW: Interrupt comparator register */

extern volatile uint16_t IO_PWM_SLAVE1_HT __attribute__((nodp, addr(0x00140)));  /*!< IO_PWM_SLAVE1_HT */
#define M_PWM_SLAVE1_HT                         (65535U << 0)                   /*!< RW: High threshold value */

extern volatile uint16_t IO_PWM_SLAVE1_LT __attribute__((nodp, addr(0x00142)));  /*!< IO_PWM_SLAVE1_LT */
#define M_PWM_SLAVE1_LT                         (65535U << 0)                   /*!< RW: Low threshold value */

extern volatile uint16_t IO_PWM_SLAVE1_PER __attribute__((nodp, addr(0x00144)));  /*!< IO_PWM_SLAVE1_PER */
#define M_PWM_SLAVE1_PER                        (65535U << 0)                   /*!< R: Not exist because PWM is only a slave */

extern volatile uint16_t IO_PWM_SLAVE1_CTRL __attribute__((nodp, addr(0x00146)));  /*!< IO_PWM_SLAVE1_CTRL */
#define M_PWM_SLAVE1_PSCLM                      (15U << 12)                     /*!< RW: Divide input clock by M+1 */
#define M_PWM_SLAVE1_PSCLN                      (15U << 8)                      /*!< RW: Divide input clock by 2Nwith N between 0 and 11 included. */
#define B_PWM_SLAVE1_PWM_IN                     (1U << 7)                       /*!< R: Read back PWM output; W: Do nothing */
#define B_PWM_SLAVE1_IDLE                       (1U << 6)                       /*!< RW: When PWM is stopped, 1: Output is invert of POL, 0: Output is POL */
#define B_PWM_SLAVE1_POL                        (1U << 5)                       /*!< RW: 1: Output start at 1, 0: Output start at 0 */
#define M_PWM_SLAVE1_MODE                       (3U << 3)                       /*!< RW: See below */
#define C_PWM_SLAVE1_MODE_SIMPLE                (0U << 3)                       /*!< 00: Simple mode */
#define C_PWM_SLAVE1_MODE_MIRROR                (1U << 3)                       /*!< 01: Mirror mode */
#define C_PWM_SLAVE1_MODE_INDEPENDENT           (2U << 3)                       /*!< 10: Independent mode */
#define B_PWM_SLAVE1_SLAVE                      (1U << 2)                       /*!< RW: 1: Slave mode, 0: Master mode */
#define B_PWM_SLAVE1_STOP                       (1U << 1)                       /*!< W: 1 stop the PWM, all other bits are unused, 0 do nothing; R: 0 if the PWM is running else 1 */
#define B_PWM_SLAVE1_START                      (1U << 0)                       /*!< W: 1 to start the PWM, all other bits are unused, 0 do nothing; R: 1 if the PWM is running else 0 */

/* ***************** */
/* Block: PWM_SLAVE2 */
/* ***************** */
extern volatile uint16_t IO_PWM_SLAVE2_CMP __attribute__((nodp, addr(0x00148)));  /*!< IO_PWM_SLAVE2_CMP */
#define M_PWM_SLAVE2_CMP                        (65535U << 0)                   /*!< RW: Interrupt comparator register */

extern volatile uint16_t IO_PWM_SLAVE2_HT __attribute__((nodp, addr(0x0014A)));  /*!< IO_PWM_SLAVE2_HT*/
#define M_PWM_SLAVE2_HT                         (65535U << 0)                   /*!< RW: High threshold value */

extern volatile uint16_t IO_PWM_SLAVE2_LT __attribute__((nodp, addr(0x0014C)));  /*!< IO_PWM_SLAVE2_LT */
#define M_PWM_SLAVE2_LT                         (65535U << 0)                   /*!< RW: Low threshold value */

extern volatile uint16_t IO_PWM_SLAVE2_PER __attribute__((nodp, addr(0x0014E)));  /*!< IO_PWM_SLAVE2_PER */
#define M_PWM_SLAVE2_PER                        (65535U << 0)                   /*!< R: Not exist because PWM is only a slave */

extern volatile uint16_t IO_PWM_SLAVE2_CTRL __attribute__((nodp, addr(0x00150)));  /*!< IO_PWM_SLAVE2_CTRL */
#define M_PWM_SLAVE2_PSCLM                      (15U << 12)                     /*!< RW: Divide input clock by M+1 */
#define M_PWM_SLAVE2_PSCLN                      (15U << 8)                      /*!< RW: Divide input clock by 2Nwith N between 0 and 11 included. */
#define B_PWM_SLAVE2_PWM_IN                     (1U << 7)                       /*!< R: Read back PWM output; W: Do nothing */
#define B_PWM_SLAVE2_IDLE                       (1U << 6)                       /*!< RW: When PWM is stopped, 1: Output is invert of POL, 0: Output is POL */
#define B_PWM_SLAVE2_POL                        (1U << 5)                       /*!< RW: 1: Output start at 1, 0: Output start at 0 */
#define M_PWM_SLAVE2_MODE                       (3U << 3)                       /*!< RW: See below */
#define C_PWM_SLAVE2_MODE_SIMPLE                (0U << 3)                       /*!< 00: Simple mode */
#define C_PWM_SLAVE2_MODE_MIRROR                (1U << 3)                       /*!< 01: Mirror mode */
#define C_PWM_SLAVE2_MODE_INDEPENDENT           (2U << 3)                       /*!< 10: Independent mode */
#define B_PWM_SLAVE2_SLAVE                      (1U << 2)                       /*!< RW: 1: Slave mode, 0: Master mode */
#define B_PWM_SLAVE2_STOP                       (1U << 1)                       /*!< W: 1 stop the PWM, all other bits are unused, 0 do nothing; R: 0 if the PWM is running else 1 */
#define B_PWM_SLAVE2_START                      (1U << 0)                       /*!< W: 1 to start the PWM, all other bits are unused, 0 do nothing; R: 1 if the PWM is running else 0 */

/* ****************** */
/* Block: PWM_MASTER2 */
/* ****************** */
extern volatile uint16_t IO_PWM_MASTER2_CMP __attribute__((nodp, addr(0x00152)));  /*!< IO_PWM_MASTER2_CMP */
#define M_PWM_MASTER2_CMP                       (65535U << 0)                   /*!< RW: Interrupt comparator register */

extern volatile uint16_t IO_PWM_MASTER2_HT __attribute__((nodp, addr(0x00154)));  /*!< IO_PWM_MASTER2_HT */
#define M_PWM_MASTER2_HT                        (65535U << 0)                   /*!< RW: High threshold value */

extern volatile uint16_t IO_PWM_MASTER2_LT __attribute__((nodp, addr(0x00156)));  /*!< IO_PWM_MASTER2_LT */
#define M_PWM_MASTER2_LT                        (65535U << 0)                   /*!< RW: Low threshold value */

extern volatile uint16_t IO_PWM_MASTER2_PER __attribute__((nodp, addr(0x00158)));  /*!< IO_PWM_MASTER2_PER */
#define M_PWM_MASTER2_PER                       (65535U << 0)                   /*!< RW: PWM period */

extern volatile uint16_t IO_PWM_MASTER2_CTRL __attribute__((nodp, addr(0x0015A)));  /*!< IO_PWM_MASTER2_CTRL */
#define M_PWM_MASTER2_PSCLM                     (15U << 12)                     /*!< RW: Divide input clock by M+1 */
#define M_PWM_MASTER2_PSCLN                     (15U << 8)                      /*!< RW: Divide input clock by 2Nwith N between 0 and 11 included. */
#define B_PWM_MASTER2_PWM_IN                    (1U << 7)                       /*!< R: Read back PWM output; W: Do nothing */
#define B_PWM_MASTER2_IDLE                      (1U << 6)                       /*!< RW: When PWM is stopped, 1: Output is invert of POL, 0: Output is POL */
#define B_PWM_MASTER2_POL                       (1U << 5)                       /*!< RW: 1: Output start at 1, 0: Output start at 0 */
#define M_PWM_MASTER2_MODE                      (3U << 3)                       /*!< RW: See below */
#define C_PWM_MASTER2_MODE_SIMPLE               (0U << 3)                       /*!< 00: Simple mode */
#define C_PWM_MASTER2_MODE_MIRROR               (1U << 3)                       /*!< 01: Mirror mode */
#define C_PWM_MASTER2_MODE_INDEPENDENT          (2U << 3)                       /*!< 10: Independent mode */
#define B_PWM_MASTER2_SLAVE                     (1U << 2)                       /*!< RW: 1: Slave mode, 0: Master mode */
#define B_PWM_MASTER2_STOP                      (1U << 1)                       /*!< W: 1 stop the PWM, all other bits are unused, 0 do nothing; R: 0 if the PWM is running else 1 */
#define B_PWM_MASTER2_START                     (1U << 0)                       /*!< W: 1 to start the PWM, all other bits are unused, 0 do nothing; R: 1 if the PWM is running else 0 */

/* ***************** */
/* Block: PWM_SLAVE3 */
/* ***************** */
extern volatile uint16_t IO_PWM_SLAVE3_CMP __attribute__((nodp, addr(0x0015C)));  /*!< IO_PWM_SLAVE3_CMP */
#define M_PWM_SLAVE3_CMP                        (65535U << 0)                   /*!< RW: Interrupt comparator register */

extern volatile uint16_t IO_PWM_SLAVE3_HT __attribute__((nodp, addr(0x0015E)));  /*!< IO_PWM_SLAVE3_HT */
#define M_PWM_SLAVE3_HT                         (65535U << 0)                   /*!< RW: High threshold value */

extern volatile uint16_t IO_PWM_SLAVE3_LT __attribute__((nodp, addr(0x00160)));  /*!< IO_PWM_SLAVE3_LT */
#define M_PWM_SLAVE3_LT                         (65535U << 0)                   /*!< RW: Low threshold value */

extern volatile uint16_t IO_PWM_SLAVE3_PER __attribute__((nodp, addr(0x00162)));  /*!< IO_PWM_SLAVE3_PER */
#define M_PWM_SLAVE3_PER                        (65535U << 0)                   /*!< R: Not exist because PWM is only a slave */

extern volatile uint16_t IO_PWM_SLAVE3_CTRL __attribute__((nodp, addr(0x00164)));  /*!< IO_PWM_SLAVE3_CTRL */
#define M_PWM_SLAVE3_PSCLM                      (15U << 12)                     /*!< RW: Divide input clock by M+1 */
#define M_PWM_SLAVE3_PSCLN                      (15U << 8)                      /*!< RW: Divide input clock by 2Nwith N between 0 and 11 included. */
#define B_PWM_SLAVE3_PWM_IN                     (1U << 7)                       /*!< R: Read back PWM output; W: Do nothing */
#define B_PWM_SLAVE3_IDLE                       (1U << 6)                       /*!< RW: When PWM is stopped, 1: Output is invert of POL, 0: Output is POL */
#define B_PWM_SLAVE3_POL                        (1U << 5)                       /*!< RW: 1: Output start at 1, 0: Output start at 0 */
#define M_PWM_SLAVE3_MODE                       (3U << 3)                       /*!< RW: See below */
#define C_PWM_SLAVE3_MODE_SIMPLE                (0U << 3)                       /*!< 00: Simple mode */
#define C_PWM_SLAVE3_MODE_MIRROR                (1U << 3)                       /*!< 01: Mirror mode */
#define C_PWM_SLAVE3_MODE_INDEPENDENT           (2U << 3)                       /*!< 10: Independent mode */
#define B_PWM_SLAVE3_SLAVE                      (1U << 2)                       /*!< RW: 1: Slave mode, 0: Master mode */
#define B_PWM_SLAVE3_STOP                       (1U << 1)                       /*!< W: 1 stop the PWM, all other bits are unused, 0 do nothing; R: 0 if the PWM is running else 1 */
#define B_PWM_SLAVE3_START                      (1U << 0)                       /*!< W: 1 to start the PWM, all other bits are unused, 0 do nothing; R: 1 if the PWM is running else 0 */

/* ***************** */
/* Block: PWM_SLAVE4 */
/* ***************** */
extern volatile uint16_t IO_PWM_SLAVE4_CMP __attribute__((nodp, addr(0x00166)));  /*!< IO_PWM_SLAVE4_CMP */
#define M_PWM_SLAVE4_CMP                        (65535U << 0)                   /*!< RW: Interrupt comparator register */

extern volatile uint16_t IO_PWM_SLAVE4_HT __attribute__((nodp, addr(0x00168)));  /*!< IO_PWM_SLAVE4_HT */
#define M_PWM_SLAVE4_HT                         (65535U << 0)                   /*!< RW: High threshold value */

extern volatile uint16_t IO_PWM_SLAVE4_LT __attribute__((nodp, addr(0x0016A)));  /*!< IO_PWM_SLAVE4_LT */
#define M_PWM_SLAVE4_LT                         (65535U << 0)                   /*!< RW: Low threshold value */

extern volatile uint16_t IO_PWM_SLAVE4_PER __attribute__((nodp, addr(0x0016C)));  /*!< IO_PWM_SLAVE4_PER */
#define M_PWM_SLAVE4_PER                        (65535U << 0)                   /*!< R: Not exist because PWM is only a slave */

extern volatile uint16_t IO_PWM_SLAVE4_CTRL __attribute__((nodp, addr(0x0016E)));  /*!< IO_PWM_SLAVE4_CTRL */
#define M_PWM_SLAVE4_PSCLM                      (15U << 12)                     /*!< RW: Divide input clock by M+1 */
#define M_PWM_SLAVE4_PSCLN                      (15U << 8)                      /*!< RW: Divide input clock by 2Nwith N between 0 and 11 included. */
#define B_PWM_SLAVE4_PWM_IN                     (1U << 7)                       /*!< R: Read back PWM output */
#define B_PWM_SLAVE4_IDLE                       (1U << 6)                       /*!< RW: When PWM is stopped, 1: Output is invert of POL, 0: Output is POL */
#define B_PWM_SLAVE4_POL                        (1U << 5)                       /*!< RW: 1: Output start at 1, 0: Output start at 0 */
#define M_PWM_SLAVE4_MODE                       (3U << 3)                       /*!< RW: 10: Independent mode, 01: Mirror mode, 00: Simple mode */
#define C_PWM_SLAVE4_MODE_SIMPLE                (0U << 3)                       /*!< 00: Simple mode */
#define C_PWM_SLAVE4_MODE_MIRROR                (1U << 3)                       /*!< 01: Mirror mode */
#define C_PWM_SLAVE4_MODE_INDEPENDENT           (2U << 3)                       /*!< 10: Independent mode */
#define B_PWM_SLAVE4_SLAVE                      (1U << 2)                       /*!< RW: 1: Slave mode, 0: Master mode */
#define B_PWM_SLAVE4_STOP                       (1U << 1)                       /*!< W: 1 stop the PWM, all other bits are unused, 0 do nothing; R: 0 if the PWM is running else 1 */
#define B_PWM_SLAVE4_START                      (1U << 0)                       /*!< W: 1 to start the PWM, all other bits are unused, 0 do nothing; R: 1 if the PWM is running else 0 */

/* **************** */
/* Block: ADC_BLOCK */
/* **************** */
extern volatile uint16_t IO_ADC_BLOCK_CTRL __attribute__((nodp, addr(0x00170)));  /*!< IO_ADC_BLOCK_CTRL */
#define B_ADC_BLOCK_ADC_WIDTH                   (1U << 12)                      /*!< RW: ADC data width: 0: Up to 16 bits --> ADATA is one word per ADC 1: Up to 32 bits --> ADATA is two word per ADC */
#define M_ADC_BLOCK_ASB                         (3U << 10)                      /*!< RW: Auto Stand-By:
                                                                                 * 00: ADC is in stand-by only when not used (START = 0)
                                                                                 * 01: ADC is also in standby while waiting for triggers */
#define C_ADC_BLOCK_ASB_NEVER                   (2U << 10)                      /*!< 1x: ADC I never in stand-by */
#define M_ADC_BLOCK_INT_SCHEME                  (3U << 8)                       /*!< RW: Message interrupt and: */
#define C_ADC_BLOCK_INT_SCHEME_NOINT            (0U << 8)                       /*!< 00: No interrupt */
#define C_ADC_BLOCK_INT_SCHEME_EOC              (1U << 8)                       /*!< 01: Interrupt at each end of conversion */
#define C_ADC_BLOCK_INT_SCHEME_EOF              (2U << 8)                       /*!< 10: Interrupt at each end of frame */
#define C_ADC_BLOCK_INT_SCHEME_EOS              (3U << 8)                       /*!< 11: Interrupt at end of sequence */
#define B_ADC_BLOCK_SATURATE                    (1U << 7)                       /*!< RW: 0: ADC data is garbage in case of overflow or underflow 1: ADC ADATA is saturated to: 2N-1 in case of overflow (for a N bit ADATA) 0 in case of underflow */
#define B_ADC_BLOCK_NO_INTERLEAVE               (1U << 6)                       /*!< RW: 0: EOA triggers SDATA update 1: EOC triggers SDATA update */
#define M_ADC_BLOCK_SOC_SOURCE                  (3U << 4)                       /*!< RW: Start Of Conversion (SOC) triggered by: */
#define C_ADC_BLOCK_SOC_SOURCE_HARD_CTRIG       (0U << 4)                       /*!< 00: Hardware (HARD_CTRIG) */
#define C_ADC_BLOCK_SOC_SOURCE_SOFT_TRIG        (1U << 4)                       /*!< 01: Firmware (SOFT_TRIG) */
#define C_ADC_BLOCK_SOC_SOURCE_HARD_SOFT_TRIG   (2U << 4)                       /*!< 10: Hardware (HARD_CTRIG) validated by firmware (SOFT_TRIG) */
#define C_ADC_BLOCK_SOC_SOURCE_PERMANENT        (3U << 4)                       /*!< 11: Permanent */
#define M_ADC_BLOCK_SOS_SOURCE                  (3U << 2)                       /*!< RW: Start Of Sequence (SOS) triggered by: */
#define C_ADC_BLOCK_SOS_SOURCE_2ND_HARD_CTRIG   (0U << 2)                       /*!< 00: Second hardware trigger (HARD_STRIG) */
#define C_ADC_BLOCK_SOS_SOURCE_HARD_CTRIG       (1U << 2)                       /*!< 01: First hardware trigger (HARD_STRIG) */
#define C_ADC_BLOCK_SOS_SOURCE_SOFT_TRIG        (2U << 2)                       /*!< 10: Firmware (SOFT_TRIG) */
#define C_ADC_BLOCK_SOS_SOURCE_PERMANENT        (3U << 2)                       /*!< 11: Permanent */
#define B_ADC_BLOCK_STOP                        (1U << 1)                       /*!< W: 1: Stop the ADC, all other bits are discarded 0: No effect; R: 0: ADC is running 1: ADC is stopped */
#define B_ADC_BLOCK_START                       (1U << 0)                       /*!< W: 1: Start the ADC, all other bits are discarded 0: No effect; R: 0: ADC is stopped 1: ADC is running */

extern volatile uint16_t IO_ADC_BLOCK_SBASE __attribute__((nodp, addr(0x00172)));  /*!< IO_ADC_BLOCK_SBASE */
#define M_ADC_BLOCK_SBASE_0                     (65535U << 0)                   /*!< W: Initial SBASE pointer; R: Current pointer to SDATA */

/*! ADC Marker definition */
typedef enum
{
    C_ADC_NO_SIGN = 0U,                                                         /*!< Used when no signs are needed */
    C_ADC_NO_SIGN2 = 1U,                                                        /*!< Used when no signs are needed */
    C_ADC_EOF = 2U,                                                             /*!< End-of-Frame */
    C_ADC_EOS = 3U                                                              /*!< End-of-Sequence */
} AdcMarker_t;

typedef enum
{
    C_ADC_SIN_SDATA = 0U,                                                       /*!< ADC interface: SDATA[8:3] */
    C_ADC_SIN_ADC_SEL = 1U                                                      /*!< ADC Interface: ADC_SEL */
} AdcSin_t;

/*! ADC Channels definition (0..31) */
typedef enum
{
    C_ADC_CH0 = 0U,                                                             /*!< VS, Internal high ohmic, divider 1:26 */
    C_ADC_VS_HV = C_ADC_CH0,
    C_ADC_CH1 = 1U,                                                             /*!< TEMP, Internal temperature sensor */
    C_ADC_TEMP = C_ADC_CH1,
    C_ADC_CH2 = 2U,                                                             /*!< VDDD, Digital supply voltage, divided 1:2 */
    C_ADC_VDDD = C_ADC_CH2,
    C_ADC_CH3 = 3U,                                                             /*!< VDDA, Analogue supply voltage, divided 1:3 */
    C_ADC_VDDA = C_ADC_CH3,
    C_ADC_CH4 = 4U,                                                             /*!< Band-gap voltage digital */
    C_ADC_VBGD = C_ADC_CH4,
    C_ADC_CH5 = 5U,                                                             /*!< Vaux/4 */
    C_ADC_VAUX = C_ADC_CH5,
    C_ADC_CH6 = 6U,                                                             /*!< LIN auto configure amplifier output */
    C_ADC_LINAA_DM = C_ADC_CH6,
    C_ADC_CH7 = 7U,                                                             /*!< LIN auto configure common mode */
    C_ADC_LINAA_CM = C_ADC_CH7,
    C_ADC_CH8 = 8U,                                                             /*!< VSMF/26 Filtered motor supply voltage divided by 26 */
    C_ADC_VSMF_HV = C_ADC_CH8,
    C_ADC_CH9 = 9U,                                                             /*!< CSOUT1 - Current sense amplifier output voltage #1 */
    C_ADC_MCUR1 = C_ADC_CH9,
    C_ADC_CH10 = 10U,                                                           /*!< CSOUT2 - Current sense amplifier output voltage #2 */
    C_ADC_MCUR2 = C_ADC_CH10,
    C_ADC_CH11 = 11U,                                                           /*!< LIN Voltage divided by 3 */
    C_ADC_LIN = C_ADC_CH11,
    C_ADC_CH12 = 12U,                                                           /*!< IO[0]/2.5 */
    C_ADC_IO0_LV = C_ADC_CH12,
    C_ADC_CH13 = 13U,                                                           /*!< IO[1]/2.5 */
    C_ADC_IO1_LV = C_ADC_CH13,
    C_ADC_CH14 = 14U,                                                           /*!< IO[2]/2.5 */
    C_ADC_IO2_LV = C_ADC_CH14,
    C_ADC_CH15 = 15U,                                                           /*!< IO[3]/2.5 */
    C_ADC_IO3_LV = C_ADC_CH15,
    C_ADC_CH16 = 16U,                                                           /*!< IO[4]/2.5 */
    C_ADC_IO4_LV = C_ADC_CH16,
    C_ADC_CH17 = 17U,                                                           /*!< IO[5]/2.5 */
    C_ADC_IO5_LV = C_ADC_CH17,
    C_ADC_CH18 = 18U,                                                           /*!< IO[6]/2.5 */
    C_ADC_IO6_LV = C_ADC_CH18,
    C_ADC_CH19 = 19U,                                                           /*!< IO[7]/2.5 */
    C_ADC_IO7_LV = C_ADC_CH19,
    C_ADC_CH20 = 20U,                                                           /*!< IO[8]/2.5 */
    C_ADC_IO8_LV = C_ADC_CH20,
    C_ADC_CH21 = 21U,                                                           /*!< IO[9]/2.5 */
    C_ADC_IO9_LV = C_ADC_CH21,
    C_ADC_CH22 = 22U,                                                           /*!< IO[10]/2.5 */
    C_ADC_IO10_LV = C_ADC_CH22,
    C_ADC_CH23 = 23U,                                                           /*!< IO[11]/2.5 */
    C_ADC_IO11_LV = C_ADC_CH23,
    C_ADC_CH24 = 24U,                                                           /*!< IO[0]/21 */
    C_ADC_IO0_HV = C_ADC_CH24,
    C_ADC_CH25 = 25U,                                                           /*!< IO[1]/21 */
    C_ADC_IO1_HV = C_ADC_CH25,
    C_ADC_CH26 = 26U,                                                           /*!< IO[2]/21 */
    C_ADC_IO2_HV = C_ADC_CH26,
    C_ADC_CH27 = 27U,                                                           /*!< Phase U/26 */
    C_ADC_PH_R_HV = C_ADC_CH27,
    C_ADC_CH28 = 28U,                                                           /*!< Phase V/26 */
    C_ADC_PH_S_HV = C_ADC_CH28,
    C_ADC_CH29 = 29U,                                                           /*!< Phase W/26 */
    C_ADC_PH_T_HV = C_ADC_CH29,
    C_ADC_CH30 = 30U,                                                           /*!< Phase U/26 */
    C_ADC_PH_U_HV = C_ADC_CH30,
    C_ADC_CH31 = 31U,                                                           /*!< Phase V/26 */
    C_ADC_PH_V_HV = C_ADC_CH31,
    C_ADC_CH32 = 32U,                                                           /*!< Phase W/26 */
    C_ADC_PH_W_HV = C_ADC_CH32,
    C_ADC_CH33 = 33U,                                                           /*!< VSSA - Analogue Ground */
    C_ADC_AGND = C_ADC_CH33
} AdcChannel_t;

typedef enum
{
    C_ADC_TYPE_EC = 0U,                                                         /*!< Extended Count Mode */
    C_ADC_TYPE_CYCLIC = 1U                                                      /*!< Cyclic Mode */
} AdcType_t;

/*! Hardware trigger source. (0..63) */
typedef enum
{
    C_ADC_HW_TRIGGER_MSTR1_CMP = 0U,                                            /*!< ADC HW-Trigger PWM_MASTER1_CMP */
    C_ADC_HW_TRIGGER_MSTR1_CNT = 1U,                                            /*!< ADC HW-Trigger PWM_MASTER1_CNT */
    C_ADC_HW_TRIGGER_SLV1_CMP = 2U,                                             /*!< ADC HW-Trigger PWM_SLAVE1_CMP */
    C_ADC_HW_TRIGGER_SLV2_CMP = 3U,                                             /*!< ADC HW-Trigger PWM_SLAVE2_CMP */
    C_ADC_HW_TRIGGER_SLV3_CMP = 4U,                                             /*!< ADC HW-Trigger PWM_SLAVE3_CMP */
    C_ADC_HW_TRIGGER_SLV4_CMP = 5U,                                             /*!< ADC HW-Trigger PWM_SLAVE4_CMP */
    C_ADC_HW_TRIGGER_MSTR2_CMP = 6U,                                            /*!< ADC HW-Trigger PWM_MASTER2_CMP */
    C_ADC_HW_TRIGGER_MSTR2_CNT = 7U,                                            /*!< ADC HW-Trigger PWM_MASTER2_CNT */
    C_ADC_HW_TRIGGER_IO0 = 8U,                                                  /*!< ADC HW-Trigger IO0 */
    C_ADC_HW_TRIGGER_IO1 = 9U,                                                  /*!< ADC HW-Trigger IO1 */
    C_ADC_HW_TRIGGER_IO2 = 10U,                                                 /*!< ADC HW-Trigger IO2 */
    C_ADC_HW_TRIGGER_IO3 = 11U,                                                 /*!< ADC HW-Trigger IO3 */
    C_ADC_HW_TRIGGER_CTIMER0_INT1 = 12U,                                        /*!< ADC HW-Trigger CTIMER0_INT1 */
    C_ADC_HW_TRIGGER_CTIMER0_INT2 = 13U,                                        /*!< ADC HW-Trigger CTIMER0_INT2 */
    C_ADC_HW_TRIGGER_CTIMER0_INT3 = 14U,                                        /*!< ADC HW-Trigger CTIMER0_INT3 */
    C_ADC_HW_TRIGGER_CTIMER1_INT1 = 15U,                                        /*!< ADC HW-Trigger CTIMER1_INT1 */
    C_ADC_HW_TRIGGER_CTIMER1_INT2 = 16U,                                        /*!< ADC HW-Trigger CTIMER1_INT2 */
    C_ADC_HW_TRIGGER_CTIMER1_INT3 = 17U,                                        /*!< ADC HW-Trigger CTIMER1_INT3 */
    C_ADC_HW_TRIGGER_IO4 = 18U,                                                 /*!< ADC HW-Trigger IO4 */
    C_ADC_HW_TRIGGER_IO5 = 19U,                                                 /*!< ADC HW-Trigger IO5 */
    C_ADC_HW_TRIGGER_IO6 = 20U,                                                 /*!< ADC HW-Trigger IO6 */
    C_ADC_HW_TRIGGER_IO7 = 21U,                                                 /*!< ADC HW-Trigger IO7 */
    C_ADC_HW_TRIGGER_IO8 = 22U,                                                 /*!< ADC HW-Trigger IO8 */
    C_ADC_HW_TRIGGER_IO9 = 23U,                                                 /*!< ADC HW-Trigger IO9 */
    C_ADC_HW_TRIGGER_IO10 = 24U,                                                /*!< ADC HW-Trigger IO10 */
    C_ADC_HW_TRIGGER_IO11 = 25U,                                                /*!< ADC HW-Trigger IO11 */
    C_ADC_HW_TRIGGER_EOCpls1xMCU_CLOCK = 32U,                                   /*!< End of Conversion +  1x MCU-Clock */
    C_ADC_HW_TRIGGER_EOCpls3xMCU_CLOCK = 33U,                                   /*!< End of Conversion +  3x MCU-Clock */
    C_ADC_HW_TRIGGER_EOCpls5xMCU_CLOCK = 34U,                                   /*!< End of Conversion +  5x MCU-Clock */
    C_ADC_HW_TRIGGER_EOCpls7xMCU_CLOCK = 35U,                                   /*!< End of Conversion +  7x MCU-Clock */
    C_ADC_HW_TRIGGER_EOCpls9xMCU_CLOCK = 36U,                                   /*!< End of Conversion +  9x MCU-Clock */
    C_ADC_HW_TRIGGER_EOCpls11xMCU_CLOCK = 37U,                                  /*!< End of Conversion + 11x MCU-Clock */
    C_ADC_HW_TRIGGER_EOCpls13xMCU_CLOCK = 38U,                                  /*!< End of Conversion + 13x MCU-Clock */
    C_ADC_HW_TRIGGER_EOCpls15xMCU_CLOCK = 39U,                                  /*!< End of Conversion + 15x MCU-Clock */
    C_ADC_HW_TRIGGER_EOCpls1xADC_CLOCK = 48U,                                   /*!< End of Conversion +  1x ADC-Clock */
    C_ADC_HW_TRIGGER_EOCpls3xADC_CLOCK = 49U,                                   /*!< End of Conversion +  3x ADC-Clock */
    C_ADC_HW_TRIGGER_EOCpls5xADC_CLOCK = 50U,                                   /*!< End of Conversion +  5x ADC-Clock */
    C_ADC_HW_TRIGGER_EOCpls7xADC_CLOCK = 51U,                                   /*!< End of Conversion +  7x ADC-Clock */
    C_ADC_HW_TRIGGER_EOCpls9xADC_CLOCK = 52U,                                   /*!< End of Conversion +  9x ADC-Clock */
    C_ADC_HW_TRIGGER_EOCpls11xADC_CLOCK = 53U,                                  /*!< End of Conversion + 11x ADC-Clock */
    C_ADC_HW_TRIGGER_EOCpls13xADC_CLOCK = 54U,                                  /*!< End of Conversion + 13x ADC-Clock */
    C_ADC_HW_TRIGGER_EOCpls15xADC_CLOCK = 55U,                                  /*!< End of Conversion + 15x ADC-Clock */
} AdcHwTrigger_t;

/*! SBASE Field definition */
/* bit  1: 0 : ADC-marker
 * bit     2 : SIN
 * bit  8: 3 : Channel selection
 * bit     9 : Type
 * bit 14:10 : Trigger selection.
 */
typedef union
{
    struct __attribute__((packed))
    {
        AdcMarker_t u2AdcMarker         : 2;                                    /**< ADC Marker (end-of-sequence or end-of-frame) */
        AdcSin_t u1AdcSin               : 1;                                    /**< ADC Sin */
        AdcChannel_t u6AdcChannel       : 6;                                    /**< ADC Channel selection (0..31) */
        AdcType_t u1AdcType             : 1;                                    /**< ADC Type */
        AdcHwTrigger_t u6AdcTrigger     : 6;                                    /**< ADC Hardware Trigger selection */
    } s;                                                                        /**< Structure */
    uint16_t u16;                                                               /**< 16-bit SBASE field */
} ADC_SDATA_t;

#if defined (_OLD)
#define C_ADC_NORMAL                            (0U << 0)                       /*!< Normal (channel) */
#define C_ADC_EOF                               (2U << 0)                       /*!< End-of-Frame */
#define C_ADC_EOS                               (3U << 0)                       /*!< End-of-Sequence */
#define C_ADC_SIN_ADC_SEL                       (1U << 2)                       /*!< ADC Interface: ADC_SEL */
#define C_ADC_SIN_SDATA                         (0U << 2)                       /*!< ADC interface: SDATA[8:3] */
#define M_ADC_CHx                               (63U << 3)                      /*!< ADC Channel selection */
/* Channels (ADC Ref = 1.48V (1.25 * VBG) */
#define C_ADC_CH0                               (0U << 3)                       /*!< VS, Internal high ohmic divider 1:26 */
#define C_ADC_CH1                               (1U << 3)                       /*!< TEMP, Internal temperature sensor */
#define C_ADC_CH2                               (2U << 3)                       /*!< VDDD, Digital supply voltage, divided 1:2 */
#define C_ADC_CH3                               (3U << 3)                       /*!< VDDA, Analogue supply voltage, divided 1:3 */
#define C_ADC_CH4                               (4U << 3)                       /*!< Band-gap voltage digital */
#define C_ADC_CH5                               (5U << 3)                       /*!< Vaux/4 */
#define C_ADC_CH6                               (6U << 3)                       /*!< Reserved for LIN auto configuration amplifier output */
#define C_ADC_CH7                               (7U << 3)                       /*!< Reserved for LIN auto configuration common mode */
#define C_ADC_CH8                               (8U << 3)                       /*!< VSMF/26 Filtered motor supply voltage divided by 26 */
#define C_ADC_CH9                               (9U << 3)                       /*!< CSOUT1 - Current sense amplifier output voltage #1 */
#define C_ADC_CH10                              (10U << 3)                      /*!< CSOUT2 - Current sense amplifier output voltage #2 */
#define C_ADC_CH11                              (11U << 3)                      /*!< LIN Voltage divided by 3 */
#define C_ADC_CH12                              (12U << 3)                      /*!< IO[0]/2.4 */
#define C_ADC_CH13                              (13U << 3)                      /*!< IO[1]/2.4 */
#define C_ADC_CH14                              (14U << 3)                      /*!< IO[2]/2.4 */
#define C_ADC_CH15                              (15U << 3)                      /*!< IO[3]/2.4 */
#define C_ADC_CH16                              (16U << 3)                      /*!< IO[4]/2.4 */
#define C_ADC_CH17                              (17U << 3)                      /*!< IO[5]/2.4 */
#define C_ADC_CH18                              (18U << 3)                      /*!< IO[6]/2.4 */
#define C_ADC_CH19                              (19U << 3)                      /*!< IO[7]/2.4 */
#define C_ADC_CH20                              (20U << 3)                      /*!< IO[8]/2.4 */
#define C_ADC_CH21                              (21U << 3)                      /*!< IO[9]/2.4 */
#define C_ADC_CH22                              (22U << 3)                      /*!< IO[10]/2.4 */
#define C_ADC_CH23                              (23U << 3)                      /*!< IO[11]/2.4 */
#define C_ADC_CH24                              (24U << 3)                      /*!< IO[0]/26 */
#define C_ADC_CH25                              (25U << 3)                      /*!< IO[1]/26 */
#define C_ADC_CH26                              (26U << 3)                      /*!< IO[2]/26 */
#define C_ADC_CH27                              (27U << 3)                      /*!< Phase R/26 */
#define C_ADC_CH28                              (28U << 3)                      /*!< Phase S/26 */
#define C_ADC_CH29                              (29U << 3)                      /*!< Phase T/26 */
#define C_ADC_CH30                              (30U << 3)                      /*!< Phase U/26 */
#define C_ADC_CH31                              (31U << 3)                      /*!< Phase V/26 */
#define C_ADC_CH32                              (32U << 3)                      /*!< Phase W/26 */
#define C_ADC_CH33                              (33U << 3)                      /*!< VSSA - Analogue Ground */
#define C_ADC_TYPE_EC                           (0U << 9)                       /*!< Extended Count Mode */
#define C_ADC_TYPE_CYCLIC                       (1U << 9)                       /*!< Cyclic Mode */
#define M_ADC_HW_TRIGGER                        (63U << 10)                     /*!< ADC Trigger selection */
#define C_ADC_HW_TRIGGER_MSTR1_CMP              (0U << 10)                      /*!< PWM Master #1, CMP IT */
#define C_ADC_HW_TRIGGER_MSTR1_CNT              (1U << 10)                      /*!< PWM Master #1, CNT IT */
#define C_ADC_HW_TRIGGER_SLV1_CMP               (2U << 10)                      /*!< PWM Slave #1, CMP IT */
#define C_ADC_HW_TRIGGER_SLV2_CMP               (3U << 10)                      /*!< PWM Slave #2, CMP IT */
#define C_ADC_HW_TRIGGER_SLV3_CMP               (4U << 10)                      /*!< PWM Slave #3, CMP IT */
#define C_ADC_HW_TRIGGER_SLV4_CMP               (5U << 10)                      /*!< PWM Slave #4, CMP IT */
#define C_ADC_HW_TRIGGER_MSTR2_CMP              (6U << 10)                      /*!< PWM Master #2, CMP IT */
#define C_ADC_HW_TRIGGER_MSTR2_CNT              (7U << 10)                      /*!< PWM Master #2, CNT IT */
#define C_ADC_HW_TRIGGER_IO0                    (8U << 10)                      /*!< IO[0] */
#define C_ADC_HW_TRIGGER_IO1                    (9U << 10)                      /*!< IO[1] */
#define C_ADC_HW_TRIGGER_IO2                    (10U << 10)                     /*!< IO[2] */
#define C_ADC_HW_TRIGGER_IO3                    (11U << 10)                     /*!< IO[3] */
#define C_ADC_HW_TRIGGER_CTIMER0_INT1           (12U << 10)                     /*!< CTimer #0, INT #1 */
#define C_ADC_HW_TRIGGER_CTIMER0_INT2           (13U << 10)                     /*!< CTimer #0, INT #2 */
#define C_ADC_HW_TRIGGER_CTIMER0_INT3           (14U << 10)                     /*!< CTimer #0, INT #3 */
#define C_ADC_HW_TRIGGER_CTIMER1_INT1           (15U << 10)                     /*!< CTimer #1, INT #1 */
#define C_ADC_HW_TRIGGER_CTIMER1_INT2           (16U << 10)                     /*!< CTimer #1, INT #2 */
#define C_ADC_HW_TRIGGER_CTIMER1_INT3           (17U << 10)                     /*!< CTimer #1, INT #3 */
#define C_ADC_HW_TRIGGER_IO4                    (18U << 10)                     /*!< IO[4] */
#define C_ADC_HW_TRIGGER_IO5                    (19U << 10)                     /*!< IO[5] */
#define C_ADC_HW_TRIGGER_IO6                    (20U << 10)                     /*!< IO[6] */
#define C_ADC_HW_TRIGGER_IO7                    (21U << 10)                     /*!< IO[7] */
#define C_ADC_HW_TRIGGER_IO8                    (22U << 10)                     /*!< IO[8] */
#define C_ADC_HW_TRIGGER_IO9                    (23U << 10)                     /*!< IO[9] */
#define C_ADC_HW_TRIGGER_IO10                   (24U << 10)                     /*!< IO[10] */
#define C_ADC_HW_TRIGGER_IO11                   (25U << 10)                     /*!< IO[11] */
#define C_ADC_HW_TRIGGER_EOCpls1xMCU_CLOCK      (32U << 10)                     /*!< End of Conversion +  1x MCU-Clock */
#define C_ADC_HW_TRIGGER_EOCpls3xMCU_CLOCK      (33U << 10)                     /*!< End of Conversion +  3x MCU-Clock */
#define C_ADC_HW_TRIGGER_EOCpls5xMCU_CLOCK      (34U << 10)                     /*!< End of Conversion +  5x MCU-Clock */
#define C_ADC_HW_TRIGGER_EOCpls7xMCU_CLOCK      (35U << 10)                     /*!< End of Conversion +  7x MCU-Clock */
#define C_ADC_HW_TRIGGER_EOCpls9xMCU_CLOCK      (36U << 10)                     /*!< End of Conversion +  9x MCU-Clock */
#define C_ADC_HW_TRIGGER_EOCpls11xMCU_CLOCK     (37U << 10)                     /*!< End of Conversion + 11x MCU-Clock */
#define C_ADC_HW_TRIGGER_EOCpls13xMCU_CLOCK     (38U << 10)                     /*!< End of Conversion + 13x MCU-Clock */
#define C_ADC_HW_TRIGGER_EOCpls15xMCU_CLOCK     (39U << 10)                     /*!< End of Conversion + 15x MCU-Clock */
#define C_ADC_HW_TRIGGER_EOCpls1xADC_CLOCK      (48U << 10)                     /*!< End of Conversion +  1x ADC-Clock */
#define C_ADC_HW_TRIGGER_EOCpls3xADC_CLOCK      (49U << 10)                     /*!< End of Conversion +  3x ADC-Clock */
#define C_ADC_HW_TRIGGER_EOCpls5xADC_CLOCK      (50U << 10)                     /*!< End of Conversion +  5x ADC-Clock */
#define C_ADC_HW_TRIGGER_EOCpls7xADC_CLOCK      (51U << 10)                     /*!< End of Conversion +  7x ADC-Clock */
#define C_ADC_HW_TRIGGER_EOCpls9xADC_CLOCK      (52U << 10)                     /*!< End of Conversion +  9x ADC-Clock */
#define C_ADC_HW_TRIGGER_EOCpls11xADC_CLOCK     (53U << 10)                     /*!< End of Conversion + 11x ADC-Clock */
#define C_ADC_HW_TRIGGER_EOCpls13xADC_CLOCK     (54U << 10)                     /*!< End of Conversion + 13x ADC-Clock */
#define C_ADC_HW_TRIGGER_EOCpls15xADC_CLOCK     (55U << 10)                     /*!< End of Conversion + 15x ADC-Clock */

#define C_ADC_VS            (C_ADC_TYPE_CYCLIC | C_ADC_CH0 | C_ADC_SIN_SDATA)   /*!<  0           1.5V    VS-unfiltered (divided by 26) */
#define C_ADC_TJ            (C_ADC_TYPE_CYCLIC | C_ADC_CH1 | C_ADC_SIN_SDATA)   /*!<  1           1.5V    Chip-Junction temperature */
#define C_ADC_VDDD          (C_ADC_TYPE_CYCLIC | C_ADC_CH2 | C_ADC_SIN_SDATA)   /*!<  2           1.5V    VDDD voltage (divided by 2) */
#define C_ADC_VDDA          (C_ADC_TYPE_CYCLIC | C_ADC_CH3 | C_ADC_SIN_SDATA)   /*!<  3           1.5V    VDDA voltage (divided by 3) */
#define C_ADC_BGD           (C_ADC_TYPE_CYCLIC | C_ADC_CH4 | C_ADC_SIN_SDATA)   /*!<  4           1.5V    Band-gap voltage (Digital) */
#define C_ADC_VAUX          (C_ADC_TYPE_CYCLIC | C_ADC_CH5 | C_ADC_SIN_SDATA)   /*!<  5           1.5V    VAUX voltage (divided by 4) */
#define C_ADC_VSMF          (C_ADC_TYPE_CYCLIC | C_ADC_CH8 | C_ADC_SIN_SDATA)   /*!<  8           1.5V    VSM-Filtered (divided by 26) */
#define C_ADC_MCUR1         (C_ADC_TYPE_CYCLIC | C_ADC_CH9 | C_ADC_SIN_SDATA)   /*!<  9           1.5V    CSOUT - Current sense amplifier output voltage #1 */
#define C_ADC_MCUR2         (C_ADC_TYPE_CYCLIC | C_ADC_CH10 | C_ADC_SIN_SDATA)  /*!< 10           1.5V    CSOUT - Current sense amplifier output voltage #2 */
#define C_ADC_VLIN          (C_ADC_TYPE_CYCLIC | C_ADC_CH11 | C_ADC_SIN_SDATA)  /*!< 11           1.5V    LIN Voltage divided by 24 */
#define C_ADC_IO0           (C_ADC_TYPE_CYCLIC | C_ADC_CH12 | C_ADC_SIN_SDATA)  /*!< 12           1.5V    IO[0]/2.5 (X-Y Resolver) */
#define C_ADC_IO1           (C_ADC_TYPE_CYCLIC | C_ADC_CH13 | C_ADC_SIN_SDATA)  /*!< 13           1.5V    IO[1]/2.5 (X-Y Resolver) */
#define C_ADC_IO2           (C_ADC_TYPE_CYCLIC | C_ADC_CH14 | C_ADC_SIN_SDATA)  /*!< 14           1.5V    IO[2]/2.5 (X-Y Resolver) */
#define C_ADC_IO3           (C_ADC_TYPE_CYCLIC | C_ADC_CH15 | C_ADC_SIN_SDATA)  /*!< 15           1.5V    IO[3]/2.5 (X-Y Resolver) */
#define C_ADC_IO4           (C_ADC_TYPE_CYCLIC | C_ADC_CH16 | C_ADC_SIN_SDATA)  /*!< 16           1.5V    IO[4]/2.5 (X-Y Resolver) */
#define C_ADC_IO5           (C_ADC_TYPE_CYCLIC | C_ADC_CH17 | C_ADC_SIN_SDATA)  /*!< 17           1.5V    IO[5]/2.5 (X-Y Resolver) */
#define C_ADC_IO6           (C_ADC_TYPE_CYCLIC | C_ADC_CH18 | C_ADC_SIN_SDATA)  /*!< 18           1.5V    IO[6]/2.5 (X-Y Resolver) */
#define C_ADC_IO7           (C_ADC_TYPE_CYCLIC | C_ADC_CH19 | C_ADC_SIN_SDATA)  /*!< 19           1.5V    IO[7]/2.5 (X-Y Resolver) */
#define C_ADC_IO8           (C_ADC_TYPE_CYCLIC | C_ADC_CH20 | C_ADC_SIN_SDATA)  /*!< 20           1.5V    IO[8]/2.5 (X-Y Resolver) */
#define C_ADC_IO9           (C_ADC_TYPE_CYCLIC | C_ADC_CH21 | C_ADC_SIN_SDATA)  /*!< 21           1.5V    IO[9]/2.5 (X-Y Resolver) */
#define C_ADC_IO10          (C_ADC_TYPE_CYCLIC | C_ADC_CH22 | C_ADC_SIN_SDATA)  /*!< 22           1.5V    IO[10]/2.5 (X-Y Resolver) */
#define C_ADC_IO11          (C_ADC_TYPE_CYCLIC | C_ADC_CH23 | C_ADC_SIN_SDATA)  /*!< 23           1.5V    IO[11]/2.5 (X-Y Resolver) */
#define C_ADC_IO0HV         (C_ADC_TYPE_CYCLIC | C_ADC_CH24 | C_ADC_SIN_SDATA)  /*!< 24           1.5V    IO[0].HV test-purpose (divided by 26) */
#define C_ADC_IO1HV         (C_ADC_TYPE_CYCLIC | C_ADC_CH25 | C_ADC_SIN_SDATA)  /*!< 25           1.5V    IO[1].HV test-purpose (divided by 26) */
#define C_ADC_IO2HV         (C_ADC_TYPE_CYCLIC | C_ADC_CH26 | C_ADC_SIN_SDATA)  /*!< 26           1.5V    IO[2].HV test-purpose (divided by 26) */
#define C_ADC_VPHR          (C_ADC_TYPE_CYCLIC | C_ADC_CH27 | C_ADC_SIN_SDATA)  /*!< 27           1.5V    Phase R voltage (divided by 26) */
#define C_ADC_VPHS          (C_ADC_TYPE_CYCLIC | C_ADC_CH28 | C_ADC_SIN_SDATA)  /*!< 28           1.5V    Phase S voltage (divided by 26) */
#define C_ADC_VPHT          (C_ADC_TYPE_CYCLIC | C_ADC_CH29 | C_ADC_SIN_SDATA)  /*!< 29           1.5V    Phase T voltage (divided by 26) */
#define C_ADC_VPHU          (C_ADC_TYPE_CYCLIC | C_ADC_CH30 | C_ADC_SIN_SDATA)  /*!< 30           1.5V    Phase U voltage (divided by 26) */
#define C_ADC_VPHV          (C_ADC_TYPE_CYCLIC | C_ADC_CH31 | C_ADC_SIN_SDATA)  /*!< 31           1.5V    Phase V voltage (divided by 26) */
#define C_ADC_VPHW          (C_ADC_TYPE_CYCLIC | C_ADC_CH32 | C_ADC_SIN_SDATA)  /*!< 32           1.5V    Phase W voltage (divided by 26) */
#define C_ADC_GNDA          (C_ADC_TYPE_CYCLIC | C_ADC_CH33 | C_ADC_SIN_SDATA)  /*!< 33           1.5V    VSSA - Analogue Ground */
#endif /* defined (_OLD) */

extern volatile uint16_t IO_ADC_BLOCK_STATUS __attribute__((nodp, addr(0x00174)));  /*!< IO_ADC_BLOCK_STATUS */
#define B_ADC_BLOCK_ABORTED                     (1U << 12)                      /*!< R: 0: Sequence running normally 1: Sequence has been aborted; W: 0: No effect 1: Clear flag */
#define B_ADC_BLOCK_FRAME_ERR                   (1U << 11)                      /*!< R: 0: No frame error 1: A Frame error occurred; W: 0: No effect 1: Clear flag */
#define B_ADC_BLOCK_MEM_ERR                     (1U << 10)                      /*!< R: 0: No error occurred 1: A memory error occurred; W: 0: No effect 1: Clear flag */
#define B_ADC_BLOCK_ADC_ERR                     (1U << 9)                       /*!< R: 0: No ADC error occurred 1: An ADC error occurred; W: 0: No effect 1: Clear flag */
#define B_ADC_BLOCK_ADC_OVF                     (1U << 8)                       /*!< R: 0: No error occurred 1: An error occurred; W: 0: No effect 1: Clear flag */
#define M_ADC_BLOCK_STATE                       (3U << 6)                       /*!< R: 00: Idle 01: Memory transfer10: Conversion ongoing11: Waiting for trigger */
#define C_ADC_BLOCK_STATE_IDLE                  (0U << 6)                       /*!< 00: Idle */
#define C_ADC_BLOCK_STATE_MEM_TRANSFER          (1U << 6)                       /*!< 01: Memory transfer */
#define C_ADC_BLOCK_STATE_CONVERSION            (2U << 6)                       /*!< 10: Conversion ongoing */
#define C_ADC_BLOCK_STATE_WAIT_FOR_TRIGGER      (3U << 6)                       /*!< 11: Waiting for trigger */
#define M_ADC_BLOCK_LAST_INT_SRC                (3U << 4)                       /*!< R: See below; W: No effect
                                                                                 * 00: No interrupt occurred since last start of ADC
                                                                                 * 01: Last interrupt is due to an end of conversion
                                                                                 * 10: Last interrupt is due to an end of frame
                                                                                 * 11: Last interrupt is due to an end of sequence */
#define B_ADC_BLOCK_READY                       (1U << 3)                       /*!< R: READY signal from ADC; W: 1: Triggers a Start of Sequence or a Start of Conversion 0: No effect */
#define B_ADC_BLOCK_SW_TRIG                     (1U << 2)                       /*!< W: 1: Triggers a Start of Sequence or a Start of Conversion 0: No effect; R: Always 0 */
#define B_ADC_BLOCK_RESUME                      (1U << 1)                       /*!< W: 1: Enable all triggers, all other bits are discarded 0: No effect; R: 0: All triggers are disabled 1: All triggers are enabled */
#define B_ADC_BLOCK_PAUSE                       (1U << 0)                       /*!< W: 1: Disable all triggers, all other bits are discarded 0: No effect; R: 0: All triggers are enabled 1: All triggers are disabled */

extern volatile uint16_t IO_ADC_BLOCK_CLK __attribute__((nodp, addr(0x00176)));  /*!< IO_ADC_BLOCK_CLK */
#define M_ADC_BLOCK_CM_PRCHRG                   (3U << 13)                      /*!< ADC Pre-Charge mode */
#define C_ADC_BLOCK_CM_PRCHRG_NO_PRECHARGE      (0U << 13)                      /*!< 00: no pre-charge */
#define C_ADC_BLOCK_CM_PRCHRG_PRECHARGE_CONV    (1U << 13)                      /*!< 01: pre-charge leading the conversion */
#define C_ADC_BLOCK_CM_PRCHRG_CONST_PRECHARGE   (2U << 13)                      /*!< 10: constant pre-charge */
#define M_ADC_BLOCK_CM_PRCHRG_TIME              (7U << 10)                      /*!< RW: Pre-Charge Time: T_PRECHARGE = 2^CM_PRCHRG_TIME */
#define M_ADC_BLOCK_CLK_TO_INT                  (3U << 8)                       /*!< RW: Number of MCU_CLK between MS_PHIx_CLK and MS_PHIx_INT */
#define M_ADC_BLOCK_ADC_CLK_DIV                 (127U << 0)                     /*!< RW: ADC_CLK = MCU_CLK / ( ADC_CLK_DIV + 1 ) */

extern volatile uint16_t IO_ADC_BLOCK_X __attribute__((nodp, addr(0x00178)));   /*!< IO_ADC_BLOCK_X */
#define B_ADC_BLOCK_REF_ALWAYS_ON               (1U << 15)                      /*!< RW: ADC analogue test and internal reference buffers configuration 0 : driven by ADC standby signal 1 : leave on between conversions */
#define M_ADC_BLOCK_SPEED                       (3U << 13)                      /*!< RW: Bias current selection (max system frequency selection) 00 : full bias 01 : 2/3 bias 10 : 1/2 bias 11 : 1/3 bias */
#define B_ADC_BLOCK_INTREF                      (1U << 12)                      /*!< RW: ADC reference selection: 0 : select external reference voltage 1 : select internal reference voltage (internalVref = VDDA3V3/2) */
#define B_ADC_BLOCK_NOCHOP                      (1U << 11)                      /*!< RW: Disables the chopping of the amplifier 0 : chopping is active 1 : chopping is disabled */
#define M_ADC_BLOCK_FORCE                       (3U << 9)                       /*!< RW: For cyclic-only conversion. Overrides the first comparison of a cyclic conversion. 00: regular comparison 01: code {DH,DL} = 00 10: code {DH,DL}= 10 11: code {DH,DL} = 01 */
#define B_ADC_BLOCK_OUTMODE                     (1U << 8)                       /*!< RW: Output data format 0: the ADC returns the regular conversion data 1 : the ADC returns the 2-bit comparison result in each conversion step */
#define B_ADC_BLOCK_START_PHI                   (1U << 7)                       /*!< RW: Defines starting phase for a cyclic only conversion 0: same phase than in extended counting mode 1: opposite phase to the extended counting mode */
#define M_ADC_BLOCK_FR                          (3U << 5)                       /*!< RW: Change the ADC Frequency 00: CE used 01: CE / 2 used 10: CE / 4 used 11: CE / 8 used */
#define B_ADC_BLOCK_MODE                        (1U << 4)                       /*!< RW: Defines the operation mode 00 : normal mode 01 : test mode */
#define B_ADC_BLOCK_TYPE                        (1U << 3)                       /*!< R: COME FROM SDATA Type of conversion 0: Extended Counting 1: Cyclic-only */
#define M_ADC_BLOCK_COUNT                       (7U << 0)                       /*!< RW: Define number of counting step during an extended conversion number-of-Steps = 2(COUNT+3) */

extern volatile uint16_t IO_ADC_BLOCK_TEST __attribute__((nodp, addr(0x0017A)));  /*!< IO_ADC_BLOCK_TEST */
#define B_ADC_BLOCK_TEST_CHOP_TESTOV            (1U << 15)                      /*!< RW: Chopping of non-overlap time monitor */
#define B_ADC_BLOCK_TEST_NONOV_CTRL             (1U << 14)                      /*!< RW: Increase of non-overlapping time between ADC integrator phases */
#define M_ADC_BLOCK_TEST_CMM                    (3U << 12)                      /*!< RW: Test control inputs for measuring ready comparator threshold/ output of common mode monitor. 00: (default) application mode 01: measuring output of common-mode monitor on Vtest_cmm pin 10: forbidden 11: measuring ready comparator thresholds by supplying signal to Vtest_cmm pin */
#define B_ADC_BLOCK_TEST_COMPNODELAY            (1U << 11)                      /*!< RW: if set to '1': the comparators no delay test is enabled */
#define B_ADC_BLOCK_TEST_EN_TESTOV              (1U << 10)                      /*!< RW: if set to '1': the overlapping voltage test is enabled */
#define B_ADC_BLOCK_TEST_ANA                    (1U << 9)                       /*!< RW: if set to '1': it connects the outputs of the amplifier to VFT */
#define B_ADC_BLOCK_TEST_COMP                   (1U << 8)                       /*!< RW: if set to '1': the input signal is directly connected to the input of the comparators */
#define B_ADC_BLOCK_TEST_INN                    (1U << 7)                       /*!< RW: if set to '1': the ADC measures the voltage between the negative output of the amplifier and the reference common mode voltage (internal reference) */
#define B_ADC_BLOCK_TEST_INP                    (1U << 6)                       /*!< RW: if set to '1': the ADC measures the voltage between the positive output of the amplifier and the reference common mode voltage (internal reference) */
#define M_ADC_BLOCK_SRC                         (63U << 0)                      /*!< R: Signal source 000000 : regular input of the ADC Others: the input signal source is a subdivision of +/- VREF Comes from SDATA[8:3] in test mode (SDATA[2] == 1) or from this port in application mode (SDATA[2] == 0); W: Signal source in application mode: SDATA[2] == 0 */

extern volatile uint16_t IO_ADC_BLOCK_CORR_LSW __attribute__((nodp, addr(0x0017C)));  /*!< IO_ADC_BLOCK_CORR_LSW */
#define M_ADC_BLOCK_CORR_15_0                   (65535U << 0)                   /*!< RW: Bit [15:0] of the vector of correction */

extern volatile uint16_t IO_ADC_BLOCK_CORR_MSW __attribute__((nodp, addr(0x0017E)));  /*!< IO_ADC_BLOCK_CORR_MSW */
#define M_ADC_BLOCK_CORR_31_16                  (65535U << 0)                   /*!< RW: Bit [31:16] of the vector of correction */

extern volatile uint16_t IO_ADC_BLOCK_CORR_XSW __attribute__((nodp, addr(0x00180)));  /*!< IO_ADC_BLOCK_CORR_XSW */
#define M_ADC_BLOCK_CORR_46_32                  (32767U << 0)                   /*!< RW: Bit [46:32] of the vector of correction */

/* ******************* */
/* Block: EEPROM_FLASH */
/* ******************* */
extern volatile uint16_t IO_EEPROM_FLASH_T __attribute__((nodp, addr(0x001A0)));  /*!< IO_EEPROM_FLASH_T */
#define B_EEPROM_FLASH_LOCK_T                   (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_EEPROM_FLASH_T_50US                   (31U << 10)                     /*!< RW: This value is used to fine tune the 50us reference used in the shell The following equation is used: (T_50US+ 1 ) * 10us ~ 50us Reset value: 50us, precision = 10us */
#define M_EEPROM_FLASH_T_10US                   (31U << 5)                      /*!< RW: This value is used to fine tune the 10us reference used in the shell The following equation is used: (T_10US+ 1 ) * 1us ~ 10us Reset value: 10us, precision = 1us */
#define M_EEPROM_FLASH_T_50NS                   (7U << 2)                       /*!< RW: This value is used to fine tune the 50ns reference used in the shell ( T_50NS + 1 ) * MCU_CLK >= 50ns Reset value: 5 * MCU_CLK (Valid up to 100MHz), Precision = 1 MCU_CLK */
#define B_EEPROM_FLASH_T_10NS                   (1U << 0)                       /*!< RW: Indicate the number of mcu_clk period needed to reach 20 ns delay 0: 0.5 mcu_clk period 1: 1 mcu_clk period */

extern volatile uint16_t IO_EEPROM_FLASH_T_1US __attribute__((nodp, addr(0x001A2)));  /*!< IO_EEPROM_FLASH_T_1US */
#define B_EEPROM_FLASH_LOCK_1US                 (1U << 15)                      /*!< RW: Field used for the CAMCU project */
#define M_EEPROM_FLASH_T_1US                    (31U << 0)                      /*!< RW: This value is used to fine tune the 1us reference used in the shell The following equation is used: (T_1US+ 1 ) * 1us ~ 1us Reset value: 1us, precision = 1us */

extern volatile uint16_t IO_EEPROM_FLASH_DMA __attribute__((nodp, addr(0x001A4)));  /*!< IO_EEPROM_FLASH_DMA */
#define B_EEPROM_FLASH_EE_FL_PDN_LOW            (1U << 15)                      /*!< RW: Test moe which allows to clear Pdn pin */
#define B_EEPROM_FLASH_EE_BUFFER_MODE           (1U << 10)                      /*!< RW: 0: EEPROM_buffer [31:0] is used for EE_SIG_L and EE_SIG_H 1: EEPROM _buffer [63:32] is used for EE_SIG_L and EE_SIG_H */
#define B_EEPROM_FLASH_EE_EXTEND_DATA           (1U << 9)                       /*!< RW: During a write, allows to extend the 16 bits of the write bus on all the bits of the memory (including ECC). This is only for EEPROM */
#define B_EEPROM_FLASH_EE_DMA                   (1U << 8)                       /*!< RW: Direct memory access without ECC. This is only for EEPROM */
#define M_EEPROM_FLASH_EE_FL_VERSION            (15U << 4)                      /*!< R: Gives the version of the eeprom_flash shell; W: No effect */
#define B_EEPROM_FLASH_FL_BUFFER_MODE           (1U << 1)                       /*!< RW: 0: Flash_buffer [31:0] is used for FL_SIG_L and FL_SIG_H 1: Flash_buffer [63:32] is used for FL_SIG_L and FL_SIG_H */
#define B_EEPROM_FLASH_FL_DMA                   (1U << 0)                       /*!< RW: Direct memory access without ECC. This is only for FLASH */

extern volatile uint16_t IO_EEPROM_FLASH_CMD_STS __attribute__((nodp, addr(0x001A6)));  /*!< IO_EEPROM_FLASH_CMD_STS */
#define M_EEPROM_FLASH_FL_COMMAND               (65535U << 0)                   /*!< W: Command to be executed; R: Status of command */
#define M_EEPROM_FLASH_FL_STATUS                (15U << 0)                      /*!< R: Status of command */
#define C_EEPROM_FLASH_FL_STATUS_NORMAL         (0U << 0)                       /*!< R: Status NORMAL */
#define C_EEPROM_FLASH_FL_STATUS_STDBY          (1U << 0)                       /*!< R: Status STDBY */
#define C_EEPROM_FLASH_FL_STATUS_SECTOR_ERASE   (3U << 0)                       /*!< R: Status SECTOR_ERASE */
#define C_EEPROM_FLASH_FL_STATUS_PAGE_PROGRAM   (4U << 0)                       /*!< R: Status PAGE_PROGRAM */
#define C_EEPROM_FLASH_FL_STATUS_POWER_ON       (6U << 0)                       /*!< R: Status POWER_ON */
#define C_EEPROM_FLASH_FL_STATUS_CONFIGRABLE    (7U << 0)                       /*!< R: Status CONFIGRABLE */
#define C_EEPROM_FLASH_FL_STATUS_POWER_OFF      (15U << 0)                      /*!< R: Status POWER_OFF */

extern volatile uint16_t IO_EEPROM_FLASH_FL_TIME __attribute__((nodp, addr(0x001A8)));  /*!< IO_EEPROM_FLASH_FL_TIME */
#define B_EEPROM_FLASH_FL_LOCK_ER_WR            (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_EEPROM_FLASH_FL_WR_TIME               (127U << 8)                     /*!< RW: Write time = ( WR_TIME + 1 ) * 100us 100us <= Write time <= 12.7 ms, resolution = 100us Reset value: 5ms */
#define M_EEPROM_FLASH_FL_ER_TIME               (255U << 0)                     /*!< RW: Erase time = ( ER_TIME + 1 ) *100us 0ms <= Erase time <= 25.5ms, resolution = 100us Reset value: 20ms */

extern volatile uint16_t IO_EEPROM_FLASH_FL_CTRL __attribute__((nodp, addr(0x001AA)));  /*!< IO_EEPROM_FLASH_FL_CTRL */
#define B_EEPROM_FLASH_FL_LOCK_RDY              (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_EEPROM_FLASH_FL_HALT_BEHAVIOR         (3U << 12)                      /*!< RW: Define the flash behaviour when application is halted
                                                                                 * 2'b00: Stay in the current state
                                                                                 * 2'b01: Go in mode Stand-by
                                                                                 * 2'b1x: Go in Deep power down mode (will make behave the EEPROM as if EE_HALT_BEHAVIOR is 2'b1x) */
#define M_EEPROM_FLASH_FL_DED_RETRY             (7U << 8)                       /*!< RW: Double Error Detected (DED) strategy: = 0: No retry in case of DED > 0: In case DED, re-read DED_RETRY times */
#define M_EEPROM_FLASH_FL_PREDICTION_BEHAVIOR   (3U << 6)                       /*!< RW: Define the behaviour of the flash prediction
                                                                                 * 2'b00: no prediction
                                                                                 * 2'b01: Prediction available. Uncorrected predictive access will not be aborted
                                                                                 * 2'b10: Prediction available. Uncorrected predictive access can be aborted after 2 clock period. (only usable if FL_WAIT_STATES >2)
                                                                                 * 2'b11: Prediction available. Uncorrected predictive access will be aborted if the bus request a read at an other address */
#define B_EEPROM_FLASH_FL_BYPASS_QUEUE          (1U << 4)                       /*!< RW: 0: Queue used 1: Queue never used */
#define M_EEPROM_FLASH_FL_WAIT_STATES           (7U << 0)                       /*!< RW: Number of clock pulses between the read access start and the wishbone READY rising edge. */

extern volatile uint16_t IO_EEPROM_FLASH_FL __attribute__((nodp, addr(0x001AC)));  /*!< IO_EEPROM_FLASH_FL */
#define B_EEPROM_FLASH_FL_EXTENDED_DATA         (1U << 8)                       /*!< RW: During a write, allows to extend the 16 bits of the write bus on all the bits of the memory (including ECC) */
#define B_EEPROM_FLASH_FL_DATA_CORRUPTED        (1U << 1)                       /*!< R: Corruption of data is detected (2 errors in data or 1 error in address); W: Write 1 to clear it */
#define B_EEPROM_FLASH_FL_SBE                   (1U << 0)                       /*!< R: Single bit error correction. The data has been successfully corrected; W: Write 1 to clear it */

extern volatile uint16_t IO_EEPROM_FLASH_FL_SEC_PG __attribute__((nodp, addr(0x001AE)));  /*!< IO_EEPROM_FLASH_FL_SEC_PG */
#define M_EEPROM_FLASH_FL_32K_SELEC             (3U << 12)                      /*!< RW: Indicate which 32KB part of the memory will be programmed 0: Select first 32KB (it is the only possibilty for 32KB flash 1: Select second 32KB. For a 64Kb memory, it allows to program higher part of the memory 2,3: Not implemented yet. For future use. */
#define B_EEPROM_FLASH_FL_CS_AREA               (1U << 8)                       /*!< RW: Set if the program or erase operation impacts MFA or CS */
#define M_EEPROM_FLASH_FL_SECTOR_NUMBER         (15U << 4)                      /*!< RW: Define which sector of the MFA or CS will be impacted by erase/program command. */
#define M_EEPROM_FLASH_FL_PAGE_NUMBER           (15U << 0)                      /*!< RW: Define which page of the MFA or CS will be impacted by erase/program command. */

extern volatile uint16_t IO_EEPROM_FLASH_EE_CTRL_S __attribute__((nodp, addr(0x001B0)));  /*!< IO_EEPROM_FLASH_EE_CTRL_S (System) */
#define B_EEPROM_FLASH_EE_BUSY                  (1U << 15)                      /*!< R: The EEPROM is busy, any access is invalid */
#define B_EEPROM_FLASH_EE_BUSY_STBY             (1U << 14)                      /*!< R: The EEPROM is in stand-by mode */
#define B_EEPROM_FLASH_EE_BUSY_WR               (1U << 13)                      /*!< R: A write is in progress */
#define B_EEPROM_FLASH_EE_BUSY_BUF_NOT_EMPTY    (1U << 12)                      /*!< R: Buffers are not empty */
#define M_EEPROM_FLASH_EE_W_MODE                (7U << 8)                       /*!< RW: Indicate the NVOP mode:
                                                                                 * 000: performs the complete XFAB flow (pre-write, erase, write steps)
                                                                                 * 001: performs only write
                                                                                 * 010: performs pre-write and erase
                                                                                 * 011: performs only erase
                                                                                 * 100: performs only pre-write
                                                                                 * Others : The state machine performs XFAB flow (prewrite, erase, write) */
#define M_EEPROM_FLASH_EE_WE_KEY                (15U << 4)                      /*!< RW: Protection bit: Key for write enable WE_KEY = 0x7 --> write accesses are valid WE_KEY != 0x7 --> write accesses are invalid This port is automatically reset at the end of a write access in normal mode. Read access is only possible if WE_KEY != 0x7. */
#define B_EEPROM_FLASH_EE_CONFIGURED            (1U << 1)                       /*!< RW: When set, the EEPROM leaves configurable state. */
#define B_EEPROM_FLASH_EE_ACTIVE                (1U << 0)                       /*!< RW: When set, the EEPROM is not in stand-by mode */

extern volatile uint16_t IO_EEPROM_FLASH_EE_DATA_ERROR __attribute__((nodp, addr(0x001B2)));  /*!< IO_EEPROM_FLASH_EE_DATA_ERROR */
#define B_EEPROM_FLASH_EE_DATA_CORRUPTED_2      (1U << 3)                       /*!< R: Corruption of the second 32 bits word is detected (2 errors in data or 1 error in address);
                                                                                 * W: Write 1 to clear it */
#define B_EEPROM_FLASH_EE_SBE_2                 (1U << 2)                       /*!< R: Single bit error correction of the second 32 bits word. If one, then the data has been correctly corrected;
                                                                                 * W: Write 1 to clear it */
#define B_EEPROM_FLASH_EE_DATA_CORRUPTED_1      (1U << 1)                       /*!< R: Corruption of the first 32 bits word is detected (2 errors in data or 1 error in address); W: Write 1 to clear it */
#define B_EEPROM_FLASH_EE_SBE_1                 (1U << 0)                       /*!< R: Single bit error correction of the first 32 bits word. If one, then the data has been correctly corrected; W: Write 1 to clear it */

extern volatile uint16_t IO_EEPROM_FLASH_EE_WTIME __attribute__((nodp, addr(0x001B4)));  /*!< IO_EEPROM_FLASH_EE_WTIME */
#define B_EEPROM_FLASH_EE_LOCK_WR               (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_EEPROM_FLASH_EE_WR_TIME               (127U << 8)                     /*!< RW: Write time = (PRE_WR_TIME + 1) * 10us 10us <= Write time <= 1.27ms, resolution = 10us Reset value: 300us */
#define M_EEPROM_FLASH_EE_PRE_WR_TIME           (127U << 0)                     /*!< RW: Pre-Write time = (PRE_WR_TIME + 1) * 10us 10us <= Write time <= 1.27ms, resolution = 10us Reset value: 300us */

extern volatile uint16_t IO_EEPROM_FLASH_EE_ERTIME __attribute__((nodp, addr(0x001B6)));  /*!< IO_EEPROM_FLASH_EE_ERTIME */
#define B_EEPROM_FLASH_EE_LOCK_ER               (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_EEPROM_FLASH_EE_ER_TIME               (63U << 0)                      /*!< RW: Erase time = ( ER_TIME + 1 ) *100us 0ms <= Erase time <= 6.3ms, resolution = 100us Reset value: 6ms */

extern volatile uint16_t IO_EEPROM_FLASH_EE_PROG_CYCLE __attribute__((nodp, addr(0x001B8)));  /*!< IO_EEPROM_FLASH_EE_PROG_CYCLE */
#define B_EEPROM_FLASH_EE_PROGRAM_CYCLE_LOCK    (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_EEPROM_FLASH_EE_PROGRAM_CYCLE         (31U << 0)                      /*!< RW: This field defined the number of program cycles needed for writing a word in the EEPROM Number of program cycles = PROGRAM_CYCLE + 1 1 <= Number of program cycles <= 32, Reset value: 16 */

extern volatile uint16_t IO_EEPROM_FLASH_EE_WS __attribute__((nodp, addr(0x001BA)));  /*!< IO_EEPROM_FLASH_EE_WS */
#define B_EEPROM_FLASH_EE_LOCK_RD               (1U << 15)                      /*!< RW: Lock the port */
#define M_EEPROM_FLASH_EE_WAIT_STATES           (15U << 8)                      /*!< RW: Number of MCU clock periods between the rise of CLK and the rise of wishbone ready signal. Ready delay = (EE_WAIT_STATES+1) * MCU_CLK */
#define M_EEPROM_FLASH_EE_HALT_BEHAVIOR         (3U << 0)                       /*!< RW: Define the EEPROM behaviour when application is halted 2'b00: Stay in the current state 2'b01: Go in mode Stand-by 2'b1x: Deep power down mode (will make behave the FLASH as if FL_HALT_BEHAVIOR is 2'b1x) */

extern volatile uint16_t IO_EEPROM_FLASH_EE_INT_INV_SRC __attribute__((nodp, addr(0x001BC)));  /*!< IO_EEPROM_FLASH_EE_INT_INV_SRC */
#define M_EEPROM_FLASH_EE_INT_INVALID_SRC       (15U << 0)                      /*!< R: Indicate the source of the last CEEPROM memory error interrupt
                                                                                 * 0x1: Write access in user mode without being in test mode
                                                                                 * 0x2: Write access in a memory protected area without correct autorization or being in test mode
                                                                                 * 0x3: Write access in bit/byte mode
                                                                                 * 0x4: Write access at an odd address without being in DMA mode
                                                                                 * 0x5: Write access to configuration registers without being in configurable mode. Not possible to trigger it in test mode
                                                                                 * 0x6: Memory access when BUSY = 1
                                                                                 * 0x7: Write access with WE_KEY != 0x7
                                                                                 * 0x8: Read access with WE_KEY = 0x7
                                                                                 * 0x9: Write 2 times the same buffer without being in test mode
                                                                                 * 0xA: Read the memory after starting to write the first 16 bits buffers
                                                                                 * 0xB: Set WE_KEY= 0x7 when buffers are not empty;
                                                                                 * W: Write 0xF to clear it */

extern volatile uint16_t IO_EEPROM_FLASH __attribute__((nodp, addr(0x001C6)));  /*!< IO_EEPROM_FLASH */
#define B_EEPROM_FLASH_FL_COMPLETED             (1U << 15)                      /*!< R: If 1'b1, the test previously requested is finished; W: Write 1 to clear the flag */
#define B_EEPROM_FLASH_FL_ERROR                 (1U << 14)                      /*!< R: Flag set when an error happen during a test. Flag cleared when a new test starts */
#define M_EEPROM_FLASH_FL_TEST_AREA             (3U << 10)                      /*!< RW: Select which area is tested by the test function : 2'b00: All the memory 2'b01: All the memory except last physical word 2'b10: Only MFA 2'b11: Only CS */
#define M_EEPROM_FLASH_FL_ACCESS_ORDER          (3U << 8)                       /*!< RW: Define read access order 2'b00: Read from bottom to top 2'b01: Read from top to bottom 2'b10: Pseudo random access 2'b11: Not used */
#define M_EEPROM_FLASH_FL_PATTERN_ID            (7U << 4)                       /*!< RW: Expected pattern in the memory 3'b000: FL_SBE and FL_DATA_CORRUPTED memory content check 3'b001: All 0's 3'b010: All 1's 3'b011: Functional Chess pattern 3'b100: Inverse functional chess pattern 3'b101: Chess pattern 3'b110: Inverse chess pattern 3'b111: Not used */

extern volatile uint16_t IO_EEPROM_FLASH_FL_BITS_ERROR __attribute__((nodp, addr(0x001C8)));  /*!< IO_EEPROM_FLASH_FL_BITS_ERROR */
#define M_EEPROM_FLASH_FL_BITS_ERRORS           (511U << 0)                     /*!< RW: Saturing count of bit mismatching pattern defined by FL_PATTERN_ID */

extern volatile uint16_t IO_EEPROM_FLASH_FL_SIG_L __attribute__((nodp, addr(0x001CA)));  /*!< IO_EEPROM_FLASH_FL_SIG_L */
#define M_EEPROM_FLASH_FL_SIG_L                 (65535U << 0)                   /*!< R: Read the 16 LSB of the signature in test; W: Initialise the 16 LSB of the signature in test */

extern volatile uint16_t IO_EEPROM_FLASH_FL_SIG_H __attribute__((nodp, addr(0x001CC)));  /*!< IO_EEPROM_FLASH_FL_SIG_H */
#define M_EEPROM_FLASH_FL_SIG_H                 (65535U << 0)                   /*!< R: Read the 16 MSB of the signature in test; W: Initialise the 16 MSB of the signature in test */

extern volatile uint16_t IO_EEPROM_FLASH_EE_CTRL2 __attribute__((nodp, addr(0x001CE)));  /*!< IO_EEPROM_FLASH_EE_CTRL2 */
#define B_EEPROM_FLASH_EE_COMPLETED             (1U << 15)                      /*!< R: If 1'b1, the test previously requested is finished; W: Write 1 to clear the flag */
#define B_EEPROM_FLASH_EE_ERROR                 (1U << 14)                      /*!< R: Flag set when an error happen during a test. Flag cleared when a new test starts */
#define B_EEPROM_FLASH_EE_IN_PROGRESS           (1U << 13)                      /*!< R: 0: No test running 1: Test in progress */
#define M_EEPROM_FLASH_EE_TEST_AREA             (3U << 10)                      /*!< RW: Select which area is tested by the test function: 00: All the memory 01: All the memory except last physical word 10: Only MFA 11: Only CS except last physical word */
#define M_EEPROM_FLASH_EE_ACCESS_ORDER          (3U << 8)                       /*!< RW: Define read access order 2'b00: Read from bottom to top 2'b01: Read from top to bottom 2'b10: Pseudo random access 2'b11: Not used */
#define M_EEPROM_FLASH_EE_PATTERN_ID            (7U << 4)                       /*!< RW: Expected pattern in the memory 3'b000: EE_SBE and EE_DATA_CORRUPTED memory content check 3'b001: All 0's 3'b010: All 1's 3'b011: Functional Chess pattern 3'b100: Inverse functional chess pattern 3'b101: Chess pattern (same as 3'b011) 3'b110: Inverse chess pattern (same as 3'b101) 3'b111: Not used */
#define M_EEPROM_FLASH_EE_CMD                   (3U << 0)                       /*!< RW: Enable a test function 1'b1: Margin read test function */

extern volatile uint16_t IO_EEPROM_FLASH_EE_BITS_ERROR __attribute__((nodp, addr(0x001D0)));  /*!< IO_EEPROM_FLASH_EE_BITS_ERROR */
#define M_EEPROM_FLASH_EE_BITS_ERRORS           (511U << 0)                     /*!< RW: Saturing count of bit mismatching pattern defined by EE_PATTERN_ID */

extern volatile uint16_t IO_EEPROM_FLASH_EE_SIG_L __attribute__((nodp, addr(0x001D2)));  /*!< IO_EEPROM_FLASH_EE_SIG_L */
#define M_EEPROM_FLASH_EE_SIG_L                 (65535U << 0)                   /*!< R: Read the 16 MSB of the signature in test; W: Initialise the 16 MSB of the signature in test */

extern volatile uint16_t IO_EEPROM_FLASH_EE_SIG_H __attribute__((nodp, addr(0x001D4)));  /*!< IO_EEPROM_FLASH_EE_SIG_H */
#define M_EEPROM_FLASH_EE_SIG_H                 (65535U << 0)                   /*!< R: Read the 16 LSB of the signature in test; W: Initialise the 16 LSB of the signature in test */

/* ************ */
/* Block: COLIN */
/* ************ */
extern volatile uint16_t IO_COLIN_ADDR_PATCH0 __attribute__((nodp, addr(0x001D6)));  /*!< IO_COLIN_ADDR_PATCH0 */
#define B_COLIN_CTRL_PATCH0                     (1U << 12)                      /*!< RW: 0 : patch is not activated 1 : patch is activated */
#define M_COLIN_ADDR_PATCH0                     (4095U << 0)                    /*!< RW: Dedicated protocol handler ROM address to patch */

extern volatile uint16_t IO_COLIN_DATA_PATCH0 __attribute__((nodp, addr(0x001D8)));  /*!< IO_COLIN_DATA_PATCH0 */
#define M_COLIN_DATA_PATCH0                     (65535U << 0)                   /*!< RW: When patch is activated, this DATA replace the instruction coming from the Dedicated protocol handler ROM at address selected by ADDR */

extern volatile uint16_t IO_COLIN_ADDR_PATCH1 __attribute__((nodp, addr(0x001DA)));  /*!< IO_COLIN_ADDR_PATCH1 */
#define B_COLIN_CTRL_PATCH1                     (1U << 12)                      /*!< RW: 0 : patch is not activated 1 : patch is activated */
#define M_COLIN_ADDR_PATCH1                     (4095U << 0)                    /*!< RW: Dedicated protocol handler ROM address to patch */

extern volatile uint16_t IO_COLIN_DATA_PATCH1 __attribute__((nodp, addr(0x001DC)));  /*!< IO_COLIN_DATA_PATCH1 */
#define M_COLIN_DATA_PATCH1                     (65535U << 0)                   /*!< RW: When patch is activated, this DATA replace the instruction coming from the Dedicated protocol handler ROM at address selected by ADDR */

extern volatile uint16_t IO_COLIN_ADDR_PATCH2 __attribute__((nodp, addr(0x001DE)));  /*!< IO_COLIN_ADDR_PATCH2 */
#define B_COLIN_CTRL_PATCH2                     (1U << 12)                      /*!< RW: 0 : patch is not activated 1 : patch is activated */
#define M_COLIN_ADDR_PATCH2                     (4095U << 0)                    /*!< RW: Dedicated protocol handler ROM address to patch */

extern volatile uint16_t IO_COLIN_DATA_PATCH2 __attribute__((nodp, addr(0x001E0)));  /*!< IO_COLIN_DATA_PATCH2 */
#define M_COLIN_DATA_PATCH2                     (65535U << 0)                   /*!< RW: When patch is activated, this DATA replace the instruction coming from the Dedicated protocol handler ROM at address selected by ADDR */

extern volatile uint16_t IO_COLIN_ADDR_PATCH3 __attribute__((nodp, addr(0x001E2)));  /*!< IO_COLIN_ADDR_PATCH3 */
#define B_COLIN_CTRL_PATCH3                     (1U << 12)                      /*!< RW: 0 : patch is not activated 1 : patch is activated */
#define M_COLIN_ADDR_PATCH3                     (4095U << 0)                    /*!< RW: Dedicated protocol handler ROM address to patch */

extern volatile uint16_t IO_COLIN_DATA_PATCH3 __attribute__((nodp, addr(0x001E4)));  /*!< IO_COLIN_DATA_PATCH3 */
#define M_COLIN_DATA_PATCH3                     (65535U << 0)                   /*!< RW: When patch is activated, this DATA replace the instruction coming from the Dedicated protocol handler ROM at address selected by ADDR */

extern volatile uint8_t IO_COLIN_CFG __attribute__((nodp, addr(0x001E6)));      /*!< IO_COLIN_CFG */
#define B_COLIN_RUN                             (1U << 3)                       /*!< RW: 0 : Dedicated protocol handler is under reset and not running 1 : Dedicated protocol handler is processed */
#define M_COLIN_SPEED                           (7U << 0)                       /*!< RW: Dedicated protocol handler is running a different clock frequency, integer ratio between master and dedicated protocol handler clocks.
                                                                                 * 000 : 1/1 of master CPU speed
                                                                                 * 001 : 1/2 of master CPU speed
                                                                                 * 010 : 1/3 of master CPU speed
                                                                                 * 011 : 1/4 of master CPU speed
                                                                                 * 100 : 1/5 of master CPU speed
                                                                                 * 101 : 1/6 of master CPU speed
                                                                                 * 110 : 1/7 of master CPU speed
                                                                                 * 111 : 1/8 of master CPU speed */

extern volatile uint8_t IO_COLIN_STS __attribute__((nodp, addr(0x001E7)));      /*!< IO_COLIN_STS */
#define B_COLIN_EVENT                           (1U << 3)                       /*!< R: Master event; W: Writing "0" will clear the bit. Writing "1" will let the bit unchanged ("0" or "1"). */
#define B_COLIN_HANDSHAKE                       (1U << 2)                       /*!< R: Master handshake; W: Writing "0" will clear the bit. Writing "1" will let the bit unchanged ("0" or "1"). */
#define B_COLIN_ERROR                           (1U << 1)                       /*!< R: Master error; W: Writing "0" will clear the bit. Writing "1" will let the bit unchanged ("0" or "1"). */
#define B_COLIN_SIGNAL                          (1U << 0)                       /*!< R: Master signal; W: Writing "0" will clear the bit. Writing "1" will let the bit unchanged ("0" or "1"). */

extern volatile uint16_t IO_COLIN_CMD_SLVIT __attribute__((nodp, addr(0x001E8)));  /*!< IO_COLIN_CMD_SLVIT */
#define M_COLIN_SLAVE_IT                        (255U << 8)                     /*!< W: [15] 0 : MUTEX interrupt mask is not updated 1 : MUTEX interrupt mask is updated
                                                                                 *    [14] When bit 15 is set, update MUTEX interrupt mask
                                                                                 *    [13] 0 : SIGNAL interrupt mask is not updated 1 : SIGNAL interrupt mask is updated
                                                                                 *    [12] When bit 13 is set, update SIGNAL interrupt mask
                                                                                 *    [11] 0 : HANDSHAKE interrupt mask is not updated 1 : HANDSHAKE interrupt mask is updated
                                                                                 *    [10] When bit 11 is set, update HANDSHAKE interrupt mask
                                                                                 *    [9] 0 : SIGNAL interrupt mask is not updated 1 : SIGNAL interrupt mask is updated
                                                                                 *    [8] When bit 9 is set, update SIGNAL interrupt mask;
                                                                                 * R: [15] Always read 0
                                                                                 *    [14] MUTEX interrupt mask
                                                                                 *    [13] Always read 0
                                                                                 *    [12] SIGNAL interrupt mask
                                                                                 *    [11] Always read 0
                                                                                 *    [10] HANDSHAKE interrupt mask
                                                                                 *    [9] Always read 0
                                                                                 *    [8] SIGNAL interrupt mask */
#define M_COLIN_COMMAND                         (255U << 0)                     /*!< W: Send instruction to dedicated protocol handler;
                                                                                 * R: [7:4] Always read 0
                                                                                 *    [3] Read the "MST_OWNMTX" bit
                                                                                 *    [2] Read the "WWSIGN bit"
                                                                                 *    [1] Read inverted value of "SLV_EVENT" bit
                                                                                 *    [0] Read "MST_EVENT" bit */

extern volatile uint16_t IO_COLIN_ROM_RAM __attribute__((nodp, addr(0x001EA)));  /*!< IO_COLIN_ROM_RAM */
#define B_COLIN_ROM_SETUP                       (1U << 12)                      /*!< RW: Select dedicated protocol ROM address setup time 0 : 1 period of main clock 1 : 2 periods of main clock */
#define M_COLIN_ROM_ACCESS                      (7U << 8)                       /*!< RW: Select dedicated protocol ROM access time 000 : 1 period of main clock 001 : 2 periods of main clock 010 : 3 periods of main clock 011 : 4 periods of main clock 100 : 5 periods of main clock 101 : 6 periods of main clock 110 : 7 periods of main clock 111 : 8 periods of main clock */
#define B_COLIN_RAM_SETUP                       (1U << 4)                       /*!< RW: Select dedicated protocol RAM address setup time 0 : 1 period of main clock 1 : 2 periods of main clock */
#define M_COLIN_RAM_ACCESS                      (7U << 0)                       /*!< RW: Select dedicated protocol RAM access time 000 : 1 period of main clock 001 : 2 periods of main clock 010 : 3 periods of main clock 011 : 4 periods of main clock 100 : 5 periods of main clock 101 : 6 periods of main clock 110 : 7 periods of main clock 111 : 8 periods of main clock */

extern volatile uint16_t IO_COLIN_LPWR_ROM_RAM __attribute__((nodp, addr(0x001EC)));  /*!< IO_COLIN_LPWR_ROM_RAM */
#define B_COLIN_DISABLE_RAM                     (1U << 8)                       /*!< RW: If this port field is set to one, then the RAM is forced In a low power consumption mode */
#define B_COLIN_DISABLE_ROM                     (1U << 0)                       /*!< RW: If this port field is set to one, then the ROM is forced In a low power consumption mode */

extern volatile uint16_t IO_COLIN_RAM_PROT __attribute__((nodp, addr(0x001EE)));  /*!< IO_COLIN_RAM_PROT */
#define B_COLIN_LOCK                            (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_COLIN_RAM_PROT_LIMIT                  (255U << 0)                     /*!< RW: Bottom RAM Area ProtectionLimit. Write is invalid if LOCK is set */

/* ******************* */
/* Block: TRIM_BG_BIAS */
/* ******************* */
extern volatile uint16_t IO_TRIM_BG_BIAS __attribute__((nodp, addr(0x001F0)));  /*!< IO_TRIM_BG_BIAS */
#define B_TRIM_BG_BIAS_LOCK                     (1U << 15)                      /*!< W: Lock the port. Write is invalid when set. Always read as 0.; R: Always read 0 */
#define M_TRIM_BG_BIAS_TR_BIAS                  (63U << 8)                      /*!< RW: current bias source trimming */
#define M_TRIM_BG_BIAS_TR_BGD                   (15U << 4)                      /*!< RW: digital bandgap trimming */
#define M_TRIM_BG_BIAS_TR_BGA                   (15U << 0)                      /*!< RW: analogue bandgap trimming */

/* *************** */
/* Block: TRIM_VDD */
/* *************** */
extern volatile uint16_t IO_TRIM_VDD __attribute__((nodp, addr(0x001F2)));      /*!< IO_TRIM_VDD */
#define B_TRIM_VDD_LOCK                         (1U << 15)                      /*!< W: Lock the port. Write is invalid when set. Always read as 0.; R: Always read 0 */
#define M_TRIM_VDD_TR_SUP                       (15U << 6)                      /*!< RW: spare control output bits for supply system */
#define M_TRIM_VDD_TR_VDDD                      (7U << 3)                       /*!< RW: digital bandgap trimming */
#define M_TRIM_VDD_TR_VDDA                      (7U << 0)                       /*!< RW: analogue bandgap trimming */

/* ****************** */
/* Block: TRIM_RCO32M */
/* ****************** */
extern volatile uint16_t IO_TRIM_RCO32M __attribute__((nodp, addr(0x001F4)));   /*!< IO_TRIM_RCO32M */
#define B_TRIM_RCO32M_LOCK                      (1U << 15)                      /*!< W: Lock the port. Write is invalid when set. Always read as 0.; R: Always read 0 */
#define M_TRIM_RCO32M_TR_RCO32M_IN              (1023U << 0)                    /*!< RW: 32 MHz RC oscillator trimming */

/* ***************** */
/* Block: TRIM_RCO1M */
/* ***************** */
extern volatile uint16_t IO_TRIM_RCO1M __attribute__((nodp, addr(0x001F6)));    /*!< IO_TRIM_RCO1M */
#define B_TRIM_RCO1M_LOCK                       (1U << 15)                      /*!< W: Lock the port. Write is invalid when set. Always read as 0.; R: Always read 0 */
#define M_TRIM_RCO1M_TR_LIN_SLVTERM             (7U << 11)                      /*!< RW: slave termination trimming */
#define M_TRIM_RCO1M_TR_LIN_SLEWRATE            (7U << 8)                       /*!< RW: slew rate trimming */
#define M_TRIM_RCO1M_TR_RCO1M                   (255U << 0)                     /*!< RW: 1 MHz RC oscillator trimming */

/* ********************* */
/* Block: PORT_SSCM_CONF */
/* ********************* */
extern volatile uint16_t IO_PORT_SSCM_CONF __attribute__((nodp, addr(0x001F8)));  /*!< IO_PORT_SSCM_CONF */
#define B_PORT_SSCM_CONF_SSCM_CENTERED          (1U << 2)                       /*!< RW: Value for analogue */
#define B_PORT_SSCM_CONF_SSCM_SINGLEBIT         (1U << 1)                       /*!< RW: output shall provide a code with Hamming distance of 1 instead of triangle */
#define B_PORT_SSCM_CONF_SSCM_EN                (1U << 0)                       /*!< RW: enable the spread spectrum modulation */

/* ********************* */
/* Block: PORT_STEP_CONF */
/* ********************* */
extern volatile uint16_t IO_PORT_STEP_CONF __attribute__((nodp, addr(0x001FA)));  /*!< IO_PORT_STEP_CONF */
#define M_PORT_STEP_CONF_STEP_CNT               (255U << 8)                     /*!< RW: step count per period of triangular modulation for Spread Spectrum */
#define M_PORT_STEP_CONF_STEP_DUR               (15U << 4)                      /*!< RW: step duration in main clock pulses for Spread Spectrum */
#define M_PORT_STEP_CONF_STEP_INC               (15U << 0)                      /*!< RW: step increment for Spread Spectrum */

/* ********************* */
/* Block: PORT_SUPP_TEST */
/* ********************* */
extern volatile uint16_t IO_PORT_SUPP_TEST __attribute__((nodp, addr(0x001FC)));  /*!< IO_PORT_SUPP_TEST */
#define B_PORT_SUPP_TEST_SHOVE_VAUX             (1U << 14)                      /*!< RW: VAUX increased by 20% for Shove */
#define B_PORT_SUPP_TEST_SHOVE_VDDA             (1U << 13)                      /*!< RW: VDDA increased by 20% for Shove */
#define B_PORT_SUPP_TEST_SHOVE_VDDD             (1U << 12)                      /*!< RW: VDDD set to 2.4V */
#define B_PORT_SUPP_TEST_LOW_VDDD               (1U << 11)                      /*!< RW: VDDD set to 1.4V */
#define B_PORT_SUPP_TEST_SBY_BIAS               (1U << 10)                      /*!< RW: disable current bias source */
#define B_PORT_SUPP_TEST_SWITCHOFFUV_VDDD_RES   (1U << 9)                       /*!< RW: switch off resistor divider of HPORB for IDDQ test, must be additional to DIS_HPORB */
#define M_PORT_SUPP_TEST_SWITCHOFFREG_VDDD      (3U << 7)                       /*!< RW: disable VDDD regulator to drive from external */
#define M_PORT_SUPP_TEST_SWITCHOFFREG_VDDA      (3U << 5)                       /*!< RW: disable VDDA regulator to drive from external */
#define B_PORT_SUPP_TEST_PORTEST                (1U << 4)                       /*!< RW: signal to disable reset for level shifters inside analogue, shall be set before digital IDDQ test, needs to be set together with DIS_HPORB in digital test controller */
#define B_PORT_SUPP_TEST_TEST_5U_BIASAUX        (1U << 3)                       /*!< RW: switch 5u VAUX related BIAS to analogue test bus */
#define B_PORT_SUPP_TEST_TEST_10U_BIAS          (1U << 2)                       /*!< RW: switch 10u VDDA related BIAS to analogue test bus */
#define B_PORT_SUPP_TEST_TEST_VAUX_ADDCURRENT   (1U << 1)                       /*!< RW: load VAUX with add current 20uA, together with TEST_VAUX */
#define B_PORT_SUPP_TEST_TEST_BGA               (1U << 0)                       /*!< RW: switch VBGA to analogue test bus */

/* ********************** */
/* Block: PORT_SUPP2_TEST */
/* ********************** */
extern volatile uint16_t IO_PORT_SUPP2_TEST __attribute__((nodp, addr(0x001FE)));  /*!< IO_PORT_SUPP2_TEST */
#define B_PORT_SUPP2_TEST_FSTOP                 (1U << 10)                      /*!< RW: set SETP_STOP for stop mode test, no output, will be used to generate SET_STOP on digital only */
#define B_PORT_SUPP2_TEST_FGTSM                 (1U << 9)                       /*!< RW: set GTSM for wake up test, no output, will be used to generate GTSM on digital only */
#define B_PORT_SUPP2_TEST_IDDQ_CLK10K           (1U << 8)                       /*!< RW: switch off 10 Khz clock for IDDQ test */
#define B_PORT_SUPP2_TEST_TST_GNDD_TA1          (1U << 7)                       /*!< RW: added for 81340 instead of unused iddq_tempsense */
#define B_PORT_SUPP2_TEST_TEST_MEM_ANA          (1U << 6)                       /*!< RW: control signal to switch memories analogue test output to analogue test bus */
#define B_PORT_SUPP2_TEST_TEST_TA1_EN           (1U << 5)                       /*!< RW: control signal for analogue test bus TA1 ground switch */
#define B_PORT_SUPP2_TEST_TEST_TA0_EN           (1U << 4)                       /*!< RW: control signal for analogue test bus TA0 ground switch, TA0 is connected to GND if 0 */
#define B_PORT_SUPP2_TEST_INT_WU_TEST           (1U << 3)                       /*!< RW: active test mode for internal wake up timer */
#define B_PORT_SUPP2_TEST_IDDQSENS_REG_VDDD     (1U << 2)                       /*!< RW: switch VDDD regulator in special mode to be able to use IDDQ sensor */
#define B_PORT_SUPP2_TEST_IDDQ_REG_VDDD         (1U << 1)                       /*!< RW: disable resistive divider in VDDD regulator for digital IDDQ leakage test */
#define B_PORT_SUPP2_TEST_IDDQ_REG_VDDA         (1U << 0)                       /*!< RW: disable resistive divider in VDDA regulator for capacitors leakage test */

/* ******************** */
/* Block: PORT_LIN_TEST */
/* ******************** */
extern volatile uint16_t IO_PORT_LIN_TEST __attribute__((nodp, addr(0x00200)));  /*!< IO_PORT_LIN_TEST */
#define B_PORT_LIN_TEST_TEST_2U8_BIASLIN        (1U << 0)                       /*!< RW: switches 2.8uA low side bias current from LIN cell to analogue test bus */

/* ********************** */
/* Block: PORT_SPARE_TEST */
/* ********************** */
extern volatile uint16_t IO_PORT_SPARE_TEST __attribute__((nodp, addr(0x00202)));  /*!< IO_PORT_SPARE_TEST */
#define M_PORT_SPARE_TEST_TEST_SUP              (31U << 3)                      /*!< RW: spare test control bits for supply system */
#define M_PORT_SPARE_TEST_TST_CPCLK_SEL         (7U << 0)                       /*!< RW: Value for analogue */

/* ***************** */
/* Block: PORT_IO_IN */
/* ***************** */
extern volatile uint16_t IO_PORT_IO_IN __attribute__((nodp, addr(0x00204)));    /*!< IO_PORT_IO_IN */
#define M_PORT_IO_IN_IO_IN_SYNC                 (4095U << 0)                    /*!< R: From Schmitt trigger I/O (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_11              (1U << 11)                      /*!< R: From Schmitt trigger IO11 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_10              (1U << 10)                      /*!< R: From Schmitt trigger IO10 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_9               (1U << 9)                       /*!< R: From Schmitt trigger IO9 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_8               (1U << 8)                       /*!< R: From Schmitt trigger IO8 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_7               (1U << 7)                       /*!< R: From Schmitt trigger IO7 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_6               (1U << 6)                       /*!< R: From Schmitt trigger IO6 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_5               (1U << 5)                       /*!< R: From Schmitt trigger IO5 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_4               (1U << 4)                       /*!< R: From Schmitt trigger IO4 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_3               (1U << 3)                       /*!< R: From Schmitt trigger IO3 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_2               (1U << 2)                       /*!< R: From Schmitt trigger IO2 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_1               (1U << 1)                       /*!< R: From Schmitt trigger IO1 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_0               (1U << 0)                       /*!< R: From Schmitt trigger IO0 (after resynchro) */

/* ******************* */
/* Block: PORT_SUPP_IN */
/* ******************* */
extern volatile uint16_t IO_PORT_SUPP_IN __attribute__((nodp, addr(0x00206)));  /*!< IO_PORT_SUPP_IN */
#define B_PORT_SUPP_IN_OV_VDDA_SYNC             (1U << 15)                      /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_OV_VDDA_IT               (1U << 14)                      /*!< R: Over voltage condition at VDDA */
#define B_PORT_SUPP_IN_UV_VDDAF_SYNC            (1U << 13)                      /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_UV_VDDAF_IT              (1U << 12)                      /*!< R: Driver supply under voltage detection */
#define M_PORT_SUPP_IN_OVC_SYNC                 (3U << 10)                      /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_OVC1_IT                  (1U << 9)                       /*!< R: Over current detection */
#define B_PORT_SUPP_IN_OVC0_IT                  (1U << 8)                       /*!< R: Over current detection */
#define B_PORT_SUPP_IN_OVT_SYNC                 (1U << 7)                       /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_OVT_IT                   (1U << 6)                       /*!< R: Over temperature interrupt flag */
#define B_PORT_SUPP_IN_OV_VSM_SYNC              (1U << 5)                       /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_OV_VSM_IT                (1U << 4)                       /*!< R: Over voltage at VSM */
#define B_PORT_SUPP_IN_UV_VSM_SYNC              (1U << 3)                       /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_UV_VSM_IT                (1U << 2)                       /*!< R: Under voltage at VSM */
#define B_PORT_SUPP_IN_UV_VDDA_SYNC             (1U << 1)                       /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_UV_VDDA_IT               (1U << 0)                       /*!< R: Under voltage condition at VDDA */

/* ******************** */
/* Block: PORT_SUPP2_IN */
/* ******************** */
extern volatile uint16_t IO_PORT_SUPP2_IN __attribute__((nodp, addr(0x00208)));  /*!< IO_PORT_SUPP2_IN */
#define B_PORT_SUPP2_IN_OC_VDDA_SYNC            (1U << 1)                       /*!< R: Before digital filtering */
#define B_PORT_SUPP2_IN_OC_VDDA_IT              (1U << 0)                       /*!< R: Over current condition at VDDA */

/* ******************* */
/* Block: PORT_MISC_IN */
/* ******************* */
extern volatile uint16_t IO_PORT_MISC_IN __attribute__((nodp, addr(0x0020A)));  /*!< IO_PORT_MISC_IN */
#define B_PORT_MISC_IN_XOR_WU_IO                (1U << 15)                      /*!< R: xor'ed IO wake up signals before wake up detection logic */
#define B_PORT_MISC_IN_LOCAL_WU                 (1U << 14)                      /*!< R: flag indicating an IO wake up */
#define B_PORT_MISC_IN_LIN_RXD                  (1U << 13)                      /*!< R: Digital input from LIN shell */
#define B_PORT_MISC_IN_PHYSTAT1                 (1U << 12)                      /*!< R: From LIN Physical layer */
#define B_PORT_MISC_IN_LIN_WU                   (1U << 11)                      /*!< R: flag indicating LIN as wake up source */
#define B_PORT_MISC_IN_INTERNAL_WU              (1U << 10)                      /*!< R: flag indicating wake up from internal timer */
#define B_PORT_MISC_IN_RSTAT                    (1U << 9)                       /*!< R: status of RSTAT flip flop in analogue */
#define B_PORT_MISC_IN_STOP_MODE                (1U << 8)                       /*!< R: indicates state after digital power up: if 1: return from STOP_MODE, else from POR or SLEEP */
#define M_PORT_MISC_IN_AIN_SUP                  (255U << 0)                     /*!< R: spare inputs for supply system */

/* ******************** */
/* Block: PORT_SUPP_CFG */
/* ******************** */
extern volatile uint16_t IO_PORT_SUPP_CFG __attribute__((nodp, addr(0x0020C)));  /*!< IO_PORT_SUPP_CFG */
#define B_PORT_SUPP_CFG_OC_VDDA_FILT_SEL        (1U << 8)                       /*!< RW: 0: 1-2us filtering, 1: 100-200us filtering */
#define B_PORT_SUPP_CFG_OV_VDDA_FILT_SEL        (1U << 7)                       /*!< RW: 0: 1-2us filtering, 1: 100-200us filtering */
#define B_PORT_SUPP_CFG_OVT_FILT_SEL            (1U << 6)                       /*!< RW: 0: 1-2us filtering, 1: 100-200us filtering */
#define B_PORT_SUPP_CFG_OVC_FILT_SEL            (1U << 5)                       /*!< RW: 0: 1-2us filtering, 1: 100-200us filtering */
#define B_PORT_SUPP_CFG_OV_VSM_FILT_SEL         (1U << 4)                       /*!< RW: 0: 1-2us filtering, 1: 100-200us filtering */
#define B_PORT_SUPP_CFG_UV_VSM_FILT_SEL         (1U << 3)                       /*!< RW: 0: 1-2us filtering, 1: 100-200us filtering */
#define M_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL       (3U << 1)                       /*!< RW: 00: 1-2us filtering, 01: 10-11us filtering, 10: 20-21us filtering, 11: 100-110us filtering */
#define C_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL_00    (0U << 1)                       /*!< VDDAF 1-2us filtering */
#define C_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL_01    (1U << 1)                       /*!< VDDAF 10-11us filtering */
#define C_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL_10    (2U << 1)                       /*!< VDDAF 20-21us filtering */
#define C_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL_11    (3U << 1)                       /*!< VDDAF 100-110us filtering */
#define B_PORT_SUPP_CFG_UV_VDDA_FILT_SEL        (1U << 0)                       /*!< RW: 0: 1-2us filtering, 1: 100-200us filtering */

/* *********************** */
/* Block: PORT_IO_OUT_SOFT */
/* *********************** */
extern volatile uint16_t IO_PORT_IO_OUT_SOFT __attribute__((nodp, addr(0x0020E)));  /*!< IO_PORT_IO_OUT_SOFT */
#define M_PORT_IO_OUT_SOFT_IO_OUT_SOFT          (4095U << 0)                    /*!< RW: To GPIO glue logic control */
#define C_PORT_IO_OUT_SOFT_IO11_OUT             (1U << 11)                      /*!< IO[11] output */
#define C_PORT_IO_OUT_SOFT_IO10_OUT             (1U << 10)                      /*!< IO[10] output */
#define C_PORT_IO_OUT_SOFT_IO9_OUT              (1U << 9)                       /*!< IO[9] output */
#define C_PORT_IO_OUT_SOFT_IO8_OUT              (1U << 8)                       /*!< IO[8] output */
#define C_PORT_IO_OUT_SOFT_IO7_OUT              (1U << 7)                       /*!< IO[7] output */
#define C_PORT_IO_OUT_SOFT_IO6_OUT              (1U << 6)                       /*!< IO[6] output */
#define C_PORT_IO_OUT_SOFT_IO5_OUT              (1U << 5)                       /*!< IO[5] output */
#define C_PORT_IO_OUT_SOFT_IO4_OUT              (1U << 4)                       /*!< IO[4] output */
#define C_PORT_IO_OUT_SOFT_IO3_OUT              (1U << 3)                       /*!< IO[3] output */
#define C_PORT_IO_OUT_SOFT_IO2_OUT              (1U << 2)                       /*!< IO[2] output */
#define C_PORT_IO_OUT_SOFT_IO1_OUT              (1U << 1)                       /*!< IO[1] output */
#define C_PORT_IO_OUT_SOFT_IO0_OUT              (1U << 0)                       /*!< IO[0] output */

/* ********************* */
/* Block: PORT_IO_OUT_EN */
/* ********************* */
extern volatile uint16_t IO_PORT_IO_OUT_EN __attribute__((nodp, addr(0x00210)));  /*!< IO_PORT_IO_OUT_EN */
#define M_PORT_IO_OUT_EN_IO_LV_ENABLE           (7U << 10)                      /*!< RW: Set IO[2:0] in LVI */
#define B_PORT_IO_OUT_EN_IO_LV_ENABLE_2         (1U << 12)                      /*!< RW: Set IO[2] in LVI */
#define B_PORT_IO_OUT_EN_IO_LV_ENABLE_1         (1U << 11)                      /*!< RW: Set IO[1] in LVI */
#define B_PORT_IO_OUT_EN_IO_LV_ENABLE_0         (1U << 10)                      /*!< RW: Set IO[0] in LVI */
#define M_PORT_IO_OUT_EN_IO_HS_ENABLE           (7U << 5)                       /*!< RW: Set IO[2:0] in High Side */
#define B_PORT_IO_OUT_EN_IO_HS_ENABLE_2         (1U << 7)                       /*!< RW: Set IO[2] in High Side */
#define B_PORT_IO_OUT_EN_IO_HS_ENABLE_1         (1U << 6)                       /*!< RW: Set IO[1] in High Side */
#define B_PORT_IO_OUT_EN_IO_HS_ENABLE_0         (1U << 5)                       /*!< RW: Set IO[0] in High Side */
#define M_PORT_IO_OUT_EN_IO_OD_ENABLE           (7U << 0)                       /*!< RW: Set IO[2:0] in open-drain */
#define B_PORT_IO_OUT_EN_IO_OD_ENABLE_2         (1U << 2)                       /*!< RW: Set IO[2] in open-drain */
#define B_PORT_IO_OUT_EN_IO_OD_ENABLE_1         (1U << 1)                       /*!< RW: Set IO[1] in open-drain */
#define B_PORT_IO_OUT_EN_IO_OD_ENABLE_0         (1U << 0)                       /*!< RW: Set IO[0] in open-drain */

/* ********************* */
/* Block: PORT_IO_ENABLE */
/* ********************* */
extern volatile uint16_t IO_PORT_IO_ENABLE __attribute__((nodp, addr(0x00212)));  /*!< IO_PORT_IO_ENABLE */
#define M_PORT_IO_ENABLE_IO_DISREC              (255U << 8)                     /*!< RW: Disable Receiver IO[7:0] from internal PU for digital in/out */
#define B_PORT_IO_ENABLE_IO_DISREC_7            ((1U << 7) << 8)                /*!< Disable IO[7] from internal PU for digital in/out */
#define B_PORT_IO_ENABLE_IO_DISREC_6            ((1U << 6) << 8)                /*!< Disable IO[6] from internal PU for digital in/out */
#define B_PORT_IO_ENABLE_IO_DISREC_5            ((1U << 5) << 8)                /*!< Disable IO[5] from internal PU for digital in/out */
#define B_PORT_IO_ENABLE_IO_DISREC_4            ((1U << 4) << 8)                /*!< Disable IO[4] from internal PU for digital in/out */
#define B_PORT_IO_ENABLE_IO_DISREC_3            ((1U << 3) << 8)                /*!< Disable IO[3] from internal PU for digital in/out */
#define B_PORT_IO_ENABLE_IO_DISREC_2            ((1U << 2) << 8)                /*!< Disable IO[2] from internal PU for digital in/out */
#define B_PORT_IO_ENABLE_IO_DISREC_1            ((1U << 1) << 8)                /*!< Disable IO[1] from internal PU for digital in/out */
#define B_PORT_IO_ENABLE_IO_DISREC_0            ((1U << 0) << 8)                /*!< Disable IO[0] from internal PU for digital in/out */
#define M_PORT_IO_ENABLE_IO_ENABLE              (255U << 0)                     /*!< RW: Set the state of the IO interface */
#define B_PORT_IO_ENABLE_IO_ENABLE_7            (1U << 7)                       /*!< RW: Set the state of the IO[7] interface */
#define B_PORT_IO_ENABLE_IO_ENABLE_6            (1U << 6)                       /*!< RW: Set the state of the IO[6] interface */
#define B_PORT_IO_ENABLE_IO_ENABLE_5            (1U << 5)                       /*!< RW: Set the state of the IO[5] interface */
#define B_PORT_IO_ENABLE_IO_ENABLE_4            (1U << 4)                       /*!< RW: Set the state of the IO[4] interface */
#define B_PORT_IO_ENABLE_IO_ENABLE_3            (1U << 3)                       /*!< RW: Set the state of the IO[3] interface */
#define B_PORT_IO_ENABLE_IO_ENABLE_2            (1U << 2)                       /*!< RW: Set the state of the IO[2] interface */
#define B_PORT_IO_ENABLE_IO_ENABLE_1            (1U << 1)                       /*!< RW: Set the state of the IO[1] interface */
#define B_PORT_IO_ENABLE_IO_ENABLE_0            (1U << 0)                       /*!< RW: Set the state of the IO[0] interface */

/* ********************** */
/* Block: PORT_IO_ENABLE1 */
/* ********************** */
extern volatile uint16_t IO_PORT_IO_ENABLE1 __attribute__((nodp, addr(0x00214)));  /*!< IO_PORT_IO_ENABLE1 */
#define M_PORT_IO_ENABLE1_PH_IN                 (3FU << 8)
#define B_PORT_IO_ENABLE1_PH_IN5                ((1U << 5) << 8)                /*!< RW: Enable Phase IN[5] */
#define B_PORT_IO_ENABLE1_PH_IN4                ((1U << 4) << 8)                /*!< RW: Enable Phase IN[4] */
#define B_PORT_IO_ENABLE1_PH_IN3                ((1U << 3) << 8)                /*!< RW: Enable Phase IN[3] */
#define B_PORT_IO_ENABLE1_PH_IN2                ((1U << 2) << 8)                /*!< RW: Enable Phase IN[2] */
#define B_PORT_IO_ENABLE1_PH_IN1                ((1U << 1) << 8)                /*!< RW: Enable Phase IN[1] */
#define B_PORT_IO_ENABLE1_PH_IN0                ((1U << 0) << 8)                /*!< RW: Enable Phase IN[0] */
#define M_PORT_IO_ENABLE1_IO_DISREC             (15U << 4)                      /*!< RW: Disable Receiver IO[11:8] from internal PU for digital in/out */
#define B_PORT_IO_ENABLE1_IO_DISREC_11          (1U << 7)                       /*!< RW: Disable IO[11] from internal PU for digital in/out */
#define B_PORT_IO_ENABLE1_IO_DISREC_10          (1U << 6)                       /*!< RW: Disable IO[10] from internal PU for digital in/out */
#define B_PORT_IO_ENABLE1_IO_DISREC_9           (1U << 5)                       /*!< RW: Disable IO[9] from internal PU for digital in/out */
#define B_PORT_IO_ENABLE1_IO_DISREC_8           (1U << 4)                       /*!< RW: Disable IO[8] from internal PU for digital in/out */
#define M_PORT_IO_ENABLE1_IO_ENABLE             (15U << 0)                      /*!< RW: Set the state of the IO interface */
#define B_PORT_IO_ENABLE1_IO_ENABLE_11          (1U << 3)                       /*!< RW: Set the state of the IO[11] interface */
#define B_PORT_IO_ENABLE1_IO_ENABLE_10          (1U << 2)                       /*!< RW: Set the state of the IO[10] interface */
#define B_PORT_IO_ENABLE1_IO_ENABLE_9           (1U << 1)                       /*!< RW: Set the state of the IO[9] interface */
#define B_PORT_IO_ENABLE1_IO_ENABLE_8           (1U << 0)                       /*!< RW: Set the state of the IO[8] interface */

/* ******************* */
/* Block: PORT_IO_CFG0 */
/* ******************* */
extern volatile uint16_t IO_PORT_IO_CFG0 __attribute__((nodp, addr(0x00216)));  /*!< IO_PORT_IO_CFG0 */
#define M_PORT_IO_CFG0_IO3_OUT_SEL              (15U << 12)                     /*!< RW: Output selection for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PWM_MSTR1    (0U << 12)                      /*!< RW: Output selection PWM_MASTER1 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PWM_SLV1     (1U << 12)                      /*!< RW: Output selection PWM_SLAVE1 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PWM_SLV2     (2U << 12)                      /*!< RW: Output selection PWM_SLAVE2 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PWM_MSTR2    (3U << 12)                      /*!< RW: Output selection PWM_MASTER2 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PWM_SLV3     (4U << 12)                      /*!< RW: Output selection PWM_SLAVE3 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_CTIMER0      (5U << 12)                      /*!< RW: Output selection CTIMER0 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_CTIMER1      (6U << 12)                      /*!< RW: Output selection CTIMER1 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_SOFT         (7U << 12)                      /*!< RW: Output selection SOFTWARE for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_LIN          (8U << 12)                      /*!< RW: Output selection LIN for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_COLIN_TX     (9U << 12)                      /*!< RW: Output selection COLIN_TX for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_I2C_CLK_OUT  (10U << 12)                     /*!< RW: Output selection I2C CLK for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_MxSx     (11U << 12)                     /*!< RW: Output selection SPI_MOSI(Master) or SPI_MISO(Slave) for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_SCK      (12U << 12)                     /*!< RW: Output selection SPI_SCK for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_SS       (13U << 12)                     /*!< RW: Output selection SPI_SS for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PPM          (14U << 12)                     /*!< RW: Output selection PPM for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_UART         (15U << 12)                     /*!< RW: Output selection UART for IO[3] */
#define M_PORT_IO_CFG0_IO2_OUT_SEL              (15U << 8)                      /*!< RW: Output selection for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PWM_MSTR1    (0U << 8)                       /*!< RW: Output selection PWM_MASTER1 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PWM_SLV1     (1U << 8)                       /*!< RW: Output selection PWM_SLAVE1 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PWM_SLV2     (2U << 8)                       /*!< RW: Output selection PWM_SLAVE2 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PWM_MSTR2    (3U << 8)                       /*!< RW: Output selection PWM_MASTER2 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PWM_SLV3     (4U << 8)                       /*!< RW: Output selection PWM_SLAVE3 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_CTIMER0      (5U << 8)                       /*!< RW: Output selection CTIMER0 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_CTIMER1      (6U << 8)                       /*!< RW: Output selection CTIMER1 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_SOFT         (7U << 8)                       /*!< RW: Output selection SOFTWARE for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_LIN          (8U << 8)                       /*!< RW: Output selection LIN for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_COLIN_TX     (9U << 8)                       /*!< RW: Output selection COLIN_TX for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_I2C_CLK_OUT  (10U << 8)                      /*!< RW: Output selection I2C CLK for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_MxSx     (11U << 8)                      /*!< RW: Output selection SPI_MOSI(Master) or SPI_MISO(Slave) for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_SCK      (12U << 8)                      /*!< RW: Output selection SPI_SCK for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_SS       (13U << 8)                      /*!< RW: Output selection SPI_SS for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PPM          (14U << 8)                      /*!< RW: Output selection PPM for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_UART         (15U << 8)                      /*!< RW: Output selection UART for IO[2] */
#define M_PORT_IO_CFG0_IO1_OUT_SEL              (15U << 4)                      /*!< RW: Output selection for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PWM_MSTR1    (0U << 4)                       /*!< RW: Output selection PWM_MASTER1 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PWM_SLV1     (1U << 4)                       /*!< RW: Output selection PWM_SLAVE1 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PWM_SLV2     (2U << 4)                       /*!< RW: Output selection PWM_SLAVE2 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PWM_MSTR2    (3U << 4)                       /*!< RW: Output selection PWM_MASTER2 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PWM_SLV3     (4U << 4)                       /*!< RW: Output selection PWM_SLAVE3 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_CTIMER0      (5U << 4)                       /*!< RW: Output selection CTIMER0 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_CTIMER1      (6U << 4)                       /*!< RW: Output selection CTIMER1 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_SOFT         (7U << 4)                       /*!< RW: Output selection SOFTWARE for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_LIN          (8U << 4)                       /*!< RW: Output selection LIN for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_COLIN_TX     (9U << 4)                       /*!< RW: Output selection COLIN_TX for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_I2C_CLK_OUT  (10U << 4)                      /*!< RW: Output selection I2C CLK for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_MxSx     (11U << 4)                      /*!< RW: Output selection SPI_MOSI(Master) or SPI_MISO(Slave) for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_SCK      (12U << 4)                      /*!< RW: Output selection SPI_SCK for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_SS       (13U << 4)                      /*!< RW: Output selection SPI_SS for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PPM          (14U << 4)                      /*!< RW: Output selection PPM for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_UART         (15U << 4)                      /*!< RW: Output selection UART for IO[1] */
#define M_PORT_IO_CFG0_IO0_OUT_SEL              (15U << 0)                      /*!< RW: Output selection for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PWM_MSTR1    (0U << 0)                       /*!< RW: Output selection PWM_MASTER1 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PWM_SLV1     (1U << 0)                       /*!< RW: Output selection PWM_SLAVE1 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PWM_SLV2     (2U << 0)                       /*!< RW: Output selection PWM_SLAVE2 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PWM_MSTR2    (3U << 0)                       /*!< RW: Output selection PWM_MASTER2 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PWM_SLV3     (4U << 0)                       /*!< RW: Output selection PWM_SLAVE3 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_CTIMER0      (5U << 0)                       /*!< RW: Output selection CTIMER0 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_CTIMER1      (6U << 0)                       /*!< RW: Output selection CTIMER1 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_SOFT         (7U << 0)                       /*!< RW: Output selection SOFTWARE for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_LIN          (8U << 0)                       /*!< RW: Output selection LIN for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_COLIN_TX     (9U << 0)                       /*!< RW: Output selection COLIN_TX for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_I2C_CLK_OUT  (10U << 0)                      /*!< RW: Output selection I2C CLK for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_MxSx     (11U << 0)                      /*!< RW: Output selection SPI_MOSI(Master) or SPI_MISO(Slave) for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_SCK      (12U << 0)                      /*!< RW: Output selection SPI_SCK for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_SS       (13U << 0)                      /*!< RW: Output selection SPI_SS for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PPM          (14U << 0)                      /*!< RW: Output selection PPM for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_UART         (15U << 0)                      /*!< RW: Output selection UART for IO[0] */

/* ******************* */
/* Block: PORT_IO_CFG1 */
/* ******************* */
extern volatile uint16_t IO_PORT_IO_CFG1 __attribute__((nodp, addr(0x00218)));  /*!< IO_PORT_IO_CFG1 */
#define M_PORT_IO_CFG1_IO7_OUT_SEL              (15U << 12)                     /*!< RW: Output selection for IO[7] */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_PWM_MSTR1    (0U << 12)                      /*!< RW: Output selection PWM_MASTER1 for IO[7] */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_PWM_SLV1     (1U << 12)                      /*!< RW: Output selection PWM_SLAVE1 for IO[7] */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_PWM_SLV2     (2U << 12)                      /*!< RW: Output selection PWM_SLAVE2 for IO[7] */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_PWM_MSTR2    (3U << 12)                      /*!< RW: Output selection PWM_MASTER2 for IO[7] */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_PWM_SLV3     (4U << 12)                      /*!< RW: Output selection PWM_SLAVE3 for IO[7] */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_CTIMER0      (5U << 12)                      /*!< RW: Output selection CTIMER0 for IO[7] */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_CTIMER1      (6U << 12)                      /*!< RW: Output selection CTIMER1 for IO[7] */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_SOFT         (7U << 12)                      /*!< RW: Output selection SOFTWARE for IO[7] */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_LIN          (8U << 12)                      /*!< RW: Output selection LIN for IO[7] */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_COLIN_TX     (9U << 12)                      /*!< RW: Output selection COLIN_TX for IO[7] */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_I2C_CLK_OUT  (10U << 12)                     /*!< RW: Output selection I2C CLK for IO[7] */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_SPI_MxSx     (11U << 12)                     /*!< RW: Output selection SPI_MOSI(Master) or SPI_MISO(Slave) for IO[7] */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_SPI_SCK      (12U << 12)                     /*!< RW: Output selection SPI_SCK for IO[7] */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_SPI_SS       (13U << 12)                     /*!< RW: Output selection SPI_SS for IO[7] */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_PPM          (14U << 12)                     /*!< RW: Output selection PPM for IO[7] */
#define C_PORT_IO_CFG1_IO7_OUT_SEL_UART         (15U << 12)                     /*!< RW: Output selection UART for IO[7] */
#define M_PORT_IO_CFG1_IO6_OUT_SEL              (15U << 8)                      /*!< RW: Output selection for IO[6] */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_PWM_MSTR1    (0U << 8)                       /*!< RW: Output selection PWM_MASTER1 for IO[6] */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_PWM_SLV1     (1U << 8)                       /*!< RW: Output selection PWM_SLAVE1 for IO[6] */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_PWM_SLV2     (2U << 8)                       /*!< RW: Output selection PWM_SLAVE2 for IO[6] */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_PWM_MSTR2    (3U << 8)                       /*!< RW: Output selection PWM_MASTER2 for IO[6] */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_PWM_SLV3     (4U << 8)                       /*!< RW: Output selection PWM_SLAVE3 for IO[6] */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_CTIMER0      (5U << 8)                       /*!< RW: Output selection CTIMER0 for IO[6] */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_CTIMER1      (6U << 8)                       /*!< RW: Output selection CTIMER1 for IO[6] */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_SOFT         (7U << 8)                       /*!< RW: Output selection SOFTWARE for IO[6] */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_LIN          (8U << 8)                       /*!< RW: Output selection LIN for IO[6] */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_COLIN_TX     (9U << 8)                       /*!< RW: Output selection COLIN_TX for IO[6] */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_I2C_CLK_OUT  (10U << 8)                      /*!< RW: Output selection I2C CLK for IO[6] */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_SPI_MxSx     (11U << 8)                      /*!< RW: Output selection SPI_MOSI(Master) or SPI_MISO(Slave) for IO[6] */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_SPI_SCK      (12U << 8)                      /*!< RW: Output selection SPI_SCK for IO[6] */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_SPI_SS       (13U << 8)                      /*!< RW: Output selection SPI_SS for IO[6] */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_PPM          (14U << 8)                      /*!< RW: Output selection PPM for IO[6] */
#define C_PORT_IO_CFG1_IO6_OUT_SEL_UART         (15U << 8)                      /*!< RW: Output selection UART for IO[6] */
#define M_PORT_IO_CFG1_IO5_OUT_SEL              (15U << 4)                      /*!< RW: Output selection for IO[5] */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_PWM_MSTR1    (0U << 4)                       /*!< RW: Output selection PWM_MASTER1 for IO[5] */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_PWM_SLV1     (1U << 4)                       /*!< RW: Output selection PWM_SLAVE1 for IO[5] */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_PWM_SLV2     (2U << 4)                       /*!< RW: Output selection PWM_SLAVE2 for IO[5] */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_PWM_MSTR2    (3U << 4)                       /*!< RW: Output selection PWM_MASTER2 for IO[5] */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_PWM_SLV3     (4U << 4)                       /*!< RW: Output selection PWM_SLAVE3 for IO[5] */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_CTIMER0      (5U << 4)                       /*!< RW: Output selection CTIMER0 for IO[5] */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_CTIMER1      (6U << 4)                       /*!< RW: Output selection CTIMER1 for IO[5] */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_SOFT         (7U << 4)                       /*!< RW: Output selection SOFTWARE for IO[5] */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_LIN          (8U << 4)                       /*!< RW: Output selection LIN for IO[5] */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_COLIN_TX     (9U << 4)                       /*!< RW: Output selection COLIN_TX for IO[5] */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_I2C_CLK_OUT  (10U << 4)                      /*!< RW: Output selection I2C CLK for IO[5] */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_SPI_MxSx     (11U << 4)                      /*!< RW: Output selection SPI_MOSI(Master) or SPI_MISO(Slave) for IO[5] */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_SPI_SCK      (12U << 4)                      /*!< RW: Output selection SPI_SCK for IO[5] */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_SPI_SS       (13U << 4)                      /*!< RW: Output selection SPI_SS for IO[5] */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_PPM          (14U << 4)                      /*!< RW: Output selection PPM for IO[5] */
#define C_PORT_IO_CFG1_IO5_OUT_SEL_UART         (15U << 4)                      /*!< RW: Output selection UART for IO[5] */
#define M_PORT_IO_CFG1_IO4_OUT_SEL              (15U << 0)                      /*!< RW: Output selection for IO[4] */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_PWM_MSTR1    (0U << 0)                       /*!< RW: Output selection PWM_MASTER1 for IO[4] */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_PWM_SLV1     (1U << 0)                       /*!< RW: Output selection PWM_SLAVE1 for IO[4] */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_PWM_SLV2     (2U << 0)                       /*!< RW: Output selection PWM_SLAVE2 for IO[4] */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_PWM_MSTR2    (3U << 0)                       /*!< RW: Output selection PWM_MASTER2 for IO[4] */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_PWM_SLV3     (4U << 0)                       /*!< RW: Output selection PWM_SLAVE3 for IO[4] */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_CTIMER0      (5U << 0)                       /*!< RW: Output selection CTIMER0 for IO[4] */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_CTIMER1      (6U << 0)                       /*!< RW: Output selection CTIMER1 for IO[4] */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_SOFT         (7U << 0)                       /*!< RW: Output selection SOFTWARE for IO[4] */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_LIN          (8U << 0)                       /*!< RW: Output selection LIN for IO[4] */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_COLIN_TX     (9U << 0)                       /*!< RW: Output selection COLIN_TX for IO[4] */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_I2C_CLK_OUT  (10U << 0)                      /*!< RW: Output selection I2C CLK for IO[4] */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_SPI_MxSx     (11U << 0)                      /*!< RW: Output selection SPI_MOSI(Master) or SPI_MISO(Slave) for IO[4] */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_SPI_SCK      (12U << 0)                      /*!< RW: Output selection SPI_SCK for IO[4] */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_SPI_SS       (13U << 0)                      /*!< RW: Output selection SPI_SS for IO[4] */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_PPM          (14U << 0)                      /*!< RW: Output selection PPM for IO[4] */
#define C_PORT_IO_CFG1_IO4_OUT_SEL_UART         (15U << 0)                      /*!< RW: Output selection UART for IO[4] */

/* ******************* */
/* Block: PORT_IO_CFG2 */
/* ******************* */
extern volatile uint16_t IO_PORT_IO_CFG2 __attribute__((nodp, addr(0x0021A)));  /*!< IO_PORT_IO_CFG2 */
#define M_PORT_IO_CFG2_IO11_OUT_SEL             (15U << 12)                     /*!< RW: Output selection for IO[11] */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_PWM_MSTR1   (0U << 12)                      /*!< RW: Output selection PWM_MASTER1 for IO[11] */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_PWM_SLV1    (1U << 12)                      /*!< RW: Output selection PWM_SLAVE1 for IO[11] */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_PWM_SLV2    (2U << 12)                      /*!< RW: Output selection PWM_SLAVE2 for IO[11] */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_PWM_MSTR2   (3U << 12)                      /*!< RW: Output selection PWM_MASTER2 for IO[11] */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_PWM_SLV3    (4U << 12)                      /*!< RW: Output selection PWM_SLAVE3 for IO[11] */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_CTIMER0     (5U << 12)                      /*!< RW: Output selection CTIMER0 for IO[11] */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_CTIMER1     (6U << 12)                      /*!< RW: Output selection CTIMER1 for IO[11] */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_SOFT        (7U << 12)                      /*!< RW: Output selection SOFTWARE for IO[11] */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_LIN         (8U << 12)                      /*!< RW: Output selection LIN for IO[11] */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_COLIN_TX    (9U << 12)                      /*!< RW: Output selection COLIN_TX for IO[11] */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_I2C_CLK_OUT (10U << 12)                     /*!< RW: Output selection I2C CLK for IO[11] */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_SPI_MxSx    (11U << 12)                     /*!< RW: Output selection SPI_MOSI(Master) or SPI_MISO(Slave) for IO[11] */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_SPI_SCK     (12U << 12)                     /*!< RW: Output selection SPI_SCK for IO[11] */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_SPI_SS      (13U << 12)                     /*!< RW: Output selection SPI_SS for IO[11] */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_PPM         (14U << 12)                     /*!< RW: Output selection PPM for IO[11] */
#define C_PORT_IO_CFG2_IO11_OUT_SEL_UART        (15U << 12)                     /*!< RW: Output selection UART for IO[11] */
#define M_PORT_IO_CFG2_IO10_OUT_SEL             (15U << 8)                      /*!< RW: Output selection for IO[10] */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_PWM_MSTR1   (0U << 8)                       /*!< RW: Output selection PWM_MASTER1 for IO[10] */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_PWM_SLV1    (1U << 8)                       /*!< RW: Output selection PWM_SLAVE1 for IO[10] */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_PWM_SLV2    (2U << 8)                       /*!< RW: Output selection PWM_SLAVE2 for IO[10] */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_PWM_MSTR2   (3U << 8)                       /*!< RW: Output selection PWM_MASTER2 for IO[10] */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_PWM_SLV3    (4U << 8)                       /*!< RW: Output selection PWM_SLAVE3 for IO[10] */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_CTIMER0     (5U << 8)                       /*!< RW: Output selection CTIMER0 for IO[10] */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_CTIMER1     (6U << 8)                       /*!< RW: Output selection CTIMER1 for IO[10] */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_SOFT        (7U << 8)                       /*!< RW: Output selection SOFTWARE for IO[10] */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_LIN         (8U << 8)                       /*!< RW: Output selection LIN for IO[10] */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_COLIN_TX    (9U << 8)                       /*!< RW: Output selection COLIN_TX for IO[10] */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_I2C_CLK_OUT (10U << 8)                      /*!< RW: Output selection I2C CLK for IO[10] */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_SPI_MxSx    (11U << 8)                      /*!< RW: Output selection SPI_MOSI(Master) or SPI_MISO(Slave) for IO[10] */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_SPI_SCK     (12U << 8)                      /*!< RW: Output selection SPI_SCK for IO[10] */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_SPI_SS      (13U << 8)                      /*!< RW: Output selection SPI_SS for IO[10] */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_PPM         (14U << 8)                      /*!< RW: Output selection PPM for IO[10] */
#define C_PORT_IO_CFG2_IO10_OUT_SEL_UART        (15U << 8)                      /*!< RW: Output selection UART for IO[10] */
#define M_PORT_IO_CFG2_IO9_OUT_SEL              (15U << 4)                      /*!< RW: Output selection for IO[9] */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_PWM_MSTR1    (0U << 4)                       /*!< RW: Output selection PWM_MASTER1 for IO[9] */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_PWM_SLV1     (1U << 4)                       /*!< RW: Output selection PWM_SLAVE1 for IO[9] */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_PWM_SLV2     (2U << 4)                       /*!< RW: Output selection PWM_SLAVE2 for IO[9] */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_PWM_MSTR2    (3U << 4)                       /*!< RW: Output selection PWM_MASTER2 for IO[9] */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_PWM_SLV3     (4U << 4)                       /*!< RW: Output selection PWM_SLAVE3 for IO[9] */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_CTIMER0      (5U << 4)                       /*!< RW: Output selection CTIMER0 for IO[9] */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_CTIMER1      (6U << 4)                       /*!< RW: Output selection CTIMER1 for IO[9] */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_SOFT         (7U << 4)                       /*!< RW: Output selection SOFTWARE for IO[9] */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_LIN          (8U << 4)                       /*!< RW: Output selection LIN for IO[9] */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_COLIN_TX     (9U << 4)                       /*!< RW: Output selection COLIN_TX for IO[9] */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_I2C_CLK_OUT  (10U << 4)                      /*!< RW: Output selection I2C CLK for IO[9] */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_SPI_MxSx     (11U << 4)                      /*!< RW: Output selection SPI_MOSI(Master) or SPI_MISO(Slave) for IO[9] */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_SPI_SCK      (12U << 4)                      /*!< RW: Output selection SPI_SCK for IO[9] */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_SPI_SS       (13U << 4)                      /*!< RW: Output selection SPI_SS for IO[9] */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_PPM          (14U << 4)                      /*!< RW: Output selection PPM for IO[9] */
#define C_PORT_IO_CFG2_IO9_OUT_SEL_UART         (15U << 4)                      /*!< RW: Output selection UART for IO[9] */
#define M_PORT_IO_CFG2_IO8_OUT_SEL              (15U << 0)                      /*!< RW: Output selection for IO[8] */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_PWM_MSTR1    (0U << 0)                       /*!< RW: Output selection PWM_MASTER1 for IO[8] */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_PWM_SLV1     (1U << 0)                       /*!< RW: Output selection PWM_SLAVE1 for IO[8] */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_PWM_SLV2     (2U << 0)                       /*!< RW: Output selection PWM_SLAVE2 for IO[8] */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_PWM_MSTR2    (3U << 0)                       /*!< RW: Output selection PWM_MASTER2 for IO[8] */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_PWM_SLV3     (4U << 0)                       /*!< RW: Output selection PWM_SLAVE3 for IO[8] */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_CTIMER0      (5U << 0)                       /*!< RW: Output selection CTIMER0 for IO[8] */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_CTIMER1      (6U << 0)                       /*!< RW: Output selection CTIMER1 for IO[8] */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_SOFT         (7U << 0)                       /*!< RW: Output selection SOFTWARE for IO[8] */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_LIN          (8U << 0)                       /*!< RW: Output selection LIN for IO[8] */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_COLIN_TX     (9U << 0)                       /*!< RW: Output selection COLIN_TX for IO[8] */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_I2C_CLK_OUT  (10U << 0)                      /*!< RW: Output selection I2C CLK for IO[8] */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_SPI_MxSx     (11U << 0)                      /*!< RW: Output selection SPI_MOSI(Master) or SPI_MISO(Slave) for IO[8] */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_SPI_SCK      (12U << 0)                      /*!< RW: Output selection SPI_SCK for IO[8] */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_SPI_SS       (13U << 0)                      /*!< RW: Output selection SPI_SS for IO[8] */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_PPM          (14U << 0)                      /*!< RW: Output selection PPM for IO[8] */
#define C_PORT_IO_CFG2_IO8_OUT_SEL_UART         (15U << 0)                      /*!< RW: Output selection UART for IO[8] */

/* *********************** */
/* Block: PORT_IO_UART_SEL */
/* *********************** */
extern volatile uint16_t IO_PORT_IO_UART_SEL __attribute__((nodp, addr(0x0021C)));  /*!< IO_PORT_IO_UART_SEL */
#define M_PORT_IO_UART_SEL_IO_UART_SEL          (4095U << 0)                    /*!< RW: Select between both UARTs for IO outputs */
#define C_PORT_IO_UART_SEL_IO11                 (1U << 11)                      /*!< RW: UART-selection [0:1] for IO[11] */
#define C_PORT_IO_UART_SEL_IO10                 (1U << 10)                      /*!< RW: UART-selection [0:1] for IO[10] */
#define C_PORT_IO_UART_SEL_IO9                  (1U << 9)                       /*!< RW: UART-selection [0:1] for IO[9] */
#define C_PORT_IO_UART_SEL_IO8                  (1U << 8)                       /*!< RW: UART-selection [0:1] for IO[8] */
#define C_PORT_IO_UART_SEL_IO7                  (1U << 7)                       /*!< RW: UART-selection [0:1] for IO[7] */
#define C_PORT_IO_UART_SEL_IO6                  (1U << 6)                       /*!< RW: UART-selection [0:1] for IO[6] */
#define C_PORT_IO_UART_SEL_IO5                  (1U << 5)                       /*!< RW: UART-selection [0:1] for IO[5] */
#define C_PORT_IO_UART_SEL_IO4                  (1U << 4)                       /*!< RW: UART-selection [0:1] for IO[4] */
#define C_PORT_IO_UART_SEL_IO3                  (1U << 3)                       /*!< RW: UART-selection [0:1] for IO[3] */
#define C_PORT_IO_UART_SEL_IO2                  (1U << 2)                       /*!< RW: UART-selection [0:1] for IO[2] */
#define C_PORT_IO_UART_SEL_IO1                  (1U << 1)                       /*!< RW: UART-selection [0:1] for IO[1] */
#define C_PORT_IO_UART_SEL_IO0                  (1U << 0)                       /*!< RW: UART-selection [0:1] for IO[0] */

/* **************************** */
/* Block: PORT_IO_TRIG_EDGE_CFG */
/* **************************** */
extern volatile uint16_t IO_PORT_IO_TRIG_EDGE_CFG __attribute__((nodp, addr(0x0021E)));  /*!< IO_PORT_IO_TRIG_EDGE_CFG */
#define M_PORT_IO_TRIG_EDGE_CFG_IO7_TRIG_EDGE_SEL (3U << 14)                    /*!< RW: Edge selection for IO[7] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO7_SEL_RAISING   (0U << 14)                    /*!< RW: Edge selection RAISING for IO[7] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO7_SEL_FALLING   (1U << 14)                    /*!< RW: Edge selection FALLING for IO[7] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO7_SEL_BOTH      (2U << 14)                    /*!< RW: Edge selection BOTH for IO[7] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO7_SEL_LEVEL     (3U << 14)                    /*!< RW: Edge selection LEVEL for IO[7] */
#define M_PORT_IO_TRIG_EDGE_CFG_IO6_TRIG_EDGE_SEL (3U << 12)                    /*!< RW: Edge selection for IO[6] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO6_SEL_RAISING   (0U << 12)                    /*!< RW: Edge selection RAISING for IO[6] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO6_SEL_FALLING   (1U << 12)                    /*!< RW: Edge selection FALLING for IO[6] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO6_SEL_BOTH      (2U << 12)                    /*!< RW: Edge selection BOTH for IO[6] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO6_SEL_LEVEL     (3U << 12)                    /*!< RW: Edge selection LEVEL for IO[6] */
#define M_PORT_IO_TRIG_EDGE_CFG_IO5_TRIG_EDGE_SEL (3U << 10)                    /*!< RW: Edge selection for IO[5] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO5_SEL_RAISING   (0U << 10)                    /*!< RW: Edge selection RAISING for IO[5] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO5_SEL_FALLING   (1U << 10)                    /*!< RW: Edge selection FALLING for IO[5] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO5_SEL_BOTH      (2U << 10)                    /*!< RW: Edge selection BOTH for IO[5] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO5_SEL_LEVEL     (3U << 10)                    /*!< RW: Edge selection LEVEL for IO[5] */
#define M_PORT_IO_TRIG_EDGE_CFG_IO4_TRIG_EDGE_SEL (3U << 8)                     /*!< RW: Edge selection for IO[4] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO4_SEL_RAISING   (0U << 8)                     /*!< RW: Edge selection RAISING for IO[4] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO4_SEL_FALLING   (1U << 8)                     /*!< RW: Edge selection FALLING for IO[4] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO4_SEL_BOTH      (2U << 8)                     /*!< RW: Edge selection BOTH for IO[4] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO4_SEL_LEVEL     (3U << 8)                     /*!< RW: Edge selection LEVEL for IO[4] */
#define M_PORT_IO_TRIG_EDGE_CFG_IO3_TRIG_EDGE_SEL (3U << 6)                     /*!< RW: Edge selection for IO[3] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO3_SEL_RAISING   (0U << 6)                     /*!< RW: Edge selection RAISING for IO[3] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO3_SEL_FALLING   (1U << 6)                     /*!< RW: Edge selection FALLING for IO[3] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO3_SEL_BOTH      (2U << 6)                     /*!< RW: Edge selection BOTH for IO[3] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO3_SEL_LEVEL     (3U << 6)                     /*!< RW: Edge selection LEVEL for IO[3] */
#define M_PORT_IO_TRIG_EDGE_CFG_IO2_TRIG_EDGE_SEL (3U << 4)                     /*!< RW: Edge selection for IO[2] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO2_SEL_RAISING   (0U << 4)                     /*!< RW: Edge selection RAISING for IO[2] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO2_SEL_FALLING   (1U << 4)                     /*!< RW: Edge selection FALLING for IO[2] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO2_SEL_BOTH      (2U << 4)                     /*!< RW: Edge selection BOTH for IO[2] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO2_SEL_LEVEL     (3U << 4)                     /*!< RW: Edge selection LEVEL for IO[2] */
#define M_PORT_IO_TRIG_EDGE_CFG_IO1_TRIG_EDGE_SEL (3U << 2)                     /*!< RW: Edge selection for IO[1] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO1_SEL_RAISING   (0U << 2)                     /*!< RW: Edge selection RAISING for IO[1] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO1_SEL_FALLING   (1U << 2)                     /*!< RW: Edge selection FALLING for IO[1] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO1_SEL_BOTH      (2U << 2)                     /*!< RW: Edge selection BOTH for IO[1] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO1_SEL_LEVEL     (3U << 2)                     /*!< RW: Edge selection LEVEL for IO[1] */
#define M_PORT_IO_TRIG_EDGE_CFG_IO0_TRIG_EDGE_SEL (3U << 0)                     /*!< RW: Edge selection for IO[0] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO0_SEL_RAISING   (0U << 0)                     /*!< RW: Edge selection RAISING for IO[0] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO0_SEL_FALLING   (1U << 0)                     /*!< RW: Edge selection FALLING for IO[0] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO0_SEL_BOTH      (2U << 0)                     /*!< RW: Edge selection BOTH for IO[0] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO0_SEL_LEVEL     (3U << 0)                     /*!< RW: Edge selection LEVEL for IO[0] */

/* ***************************** */
/* Block: PORT_IO_TRIG_EDGE_CFG1 */
/* ***************************** */
extern volatile uint16_t IO_PORT_IO_TRIG_EDGE_CFG1 __attribute__((nodp, addr(0x00220)));  /*!< IO_PORT_IO_TRIG_EDGE_CFG1 */
#define M_PORT_IO_TRIG_EDGE_CFG_IO11_TRIG_EDGE_SEL (3U << 6)                    /*!< RW: Edge selection for IO[11] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO11_SEL_RAISING   (0U << 6)                    /*!< RW: Edge selection RAISING for IO[11] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO11_SEL_FALLING   (1U << 6)                    /*!< RW: Edge selection FALLING for IO[11] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO11_SEL_BOTH      (2U << 6)                    /*!< RW: Edge selection BOTH for IO[11] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO11_SEL_LEVEL     (3U << 6)                    /*!< RW: Edge selection LEVEL for IO[11] */
#define M_PORT_IO_TRIG_EDGE_CFG_IO10_TRIG_EDGE_SEL (3U << 4)                    /*!< RW: Edge selection for IO[10] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO10_SEL_RAISING   (0U << 4)                    /*!< RW: Edge selection RAISING for IO[10] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO10_SEL_FALLING   (1U << 4)                    /*!< RW: Edge selection FALLING for IO[10] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO10_SEL_BOTH      (2U << 4)                    /*!< RW: Edge selection BOTH for IO[10] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO10_SEL_LEVEL     (3U << 4)                    /*!< RW: Edge selection LEVEL for IO[10] */
#define M_PORT_IO_TRIG_EDGE_CFG_IO9_TRIG_EDGE_SEL (3U << 2)                     /*!< RW: Edge selection for IO[9] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO9_SEL_RAISING   (0U << 2)                     /*!< RW: Edge selection RAISING for IO[9] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO9_SEL_FALLING   (1U << 2)                     /*!< RW: Edge selection FALLING for IO[9] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO9_SEL_BOTH      (2U << 2)                     /*!< RW: Edge selection BOTH for IO[9] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO9_SEL_LEVEL     (3U << 2)                     /*!< RW: Edge selection LEVEL for IO[9] */
#define M_PORT_IO_TRIG_EDGE_CFG_IO8_TRIG_EDGE_SEL (3U << 0)                     /*!< RW: Edge selection for IO[8] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO8_SEL_RAISING   (0U << 0)                     /*!< RW: Edge selection RAISING for IO[8] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO8_SEL_FALLING   (1U << 0)                     /*!< RW: Edge selection FALLING for IO[8] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO8_SEL_BOTH      (2U << 0)                     /*!< RW: Edge selection BOTH for IO[8] */
#define C_PORT_IO_TRIG_EDGE_CFG_IO8_SEL_LEVEL     (3U << 0)                     /*!< RW: Edge selection LEVEL for IO[8] */

/* *********************** */
/* Block: PORT_LIN_XTX_CFG */
/* *********************** */
extern volatile uint16_t IO_PORT_LIN_XTX_CFG __attribute__((nodp, addr(0x00222)));  /*!< IO_PORT_LIN_XTX_CFG */
#define B_PORT_LIN_XTX_CFG_LIN_IN_SOFT          (1U << 8)                       /*!< RW: Software port for LIN reception (connected via mux to LIN_XTX) */
#define B_PORT_LIN_XTX_CFG_LIN_OUT_SOFT         (1U << 7)                       /*!< RW: Software port for LIN transmission (connected via mux to LIN_XTX) */
#define M_PORT_LIN_XTX_CFG_LIN_COLIN_RX_SEL     (3U << 5)                       /*!< RW: Input selection for Colin RX input */
#define M_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL      (31U << 0)                      /*!< RW: Output selection for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_CTIMER0      (0U << 0)               /*!< RW: Output selection CTIMRE0 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_CTIMER1      (1U << 0)               /*!< RW: Output selection CTIMRE1 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PWM_MASTER1  (2U << 0)               /*!< RW: Output selection PWM_MASTER1 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PWM_SLAVE1   (3U << 0)               /*!< RW: Output selection PWM_SLAVE1 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PWM_SLAVE2   (4U << 0)               /*!< RW: Output selection PWM_SLAVE2 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PWM_MASTER2  (5U << 0)               /*!< RW: Output selection PWM_MASTER2 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PWM_SLAVE3   (6U << 0)               /*!< RW: Output selection PWM_SLAVE3 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_LIN_OUT_SOFT (7U << 0)               /*!< RW: Output selection SOFTWARE for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO0          (8U << 0)               /*!< RW: Output selection IO0 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO1          (9U << 0)               /*!< RW: Output selection IO1 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO2          (10U << 0)              /*!< RW: Output selection IO2 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO3          (11U << 0)              /*!< RW: Output selection IO3 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PPM_OUT      (12U << 0)              /*!< RW: Output selection PPM for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO4          (13U << 0)              /*!< RW: Output selection IO4 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO5          (14U << 0)              /*!< RW: Output selection IO5 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO6          (15U << 0)              /*!< RW: Output selection IO6 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO7          (16U << 0)              /*!< RW: Output selection IO7 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO8          (17U << 0)              /*!< RW: Output selection IO8 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO9          (18U << 0)              /*!< RW: Output selection IO9 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO10         (19U << 0)              /*!< RW: Output selection IO10 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO11         (20U << 0)              /*!< RW: Output selection IO11 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_UART0_TX     (21U << 0)              /*!< RW: Output selection UART0_TX for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_UART1_TX     (22U << 0)              /*!< RW: Output selection UART1_TX for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PWM_SLAVE4   (23U << 0)              /*!< RW: Output selection PWM_SLAVE4 for LIN_XTX */

/* ********************* */
/* Block: PORT_TIMER_CFG */
/* ********************* */
extern volatile uint16_t IO_PORT_TIMER_CFG __attribute__((nodp, addr(0x00224)));  /*!< IO_PORT_TIMER_CFG */
#define M_PORT_TIMER_CFG_TIMER0_CHB_SEL         (31U << 8)                      /*!< RW: Input selection for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO0     (0U << 8)                       /*!< RW: Input selection IO0 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO1     (1U << 8)                       /*!< RW: Input selection IO1 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO2     (2U << 8)                       /*!< RW: Input selection IO2 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO3     (3U << 8)                       /*!< RW: Input selection IO3 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_LIN_XRX (4U << 8)                       /*!< RW: Input selection LIN_XRX for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PWM_MASTER1 (5U << 8)                   /*!< RW: Input selection PWM_MASTER1 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PWM_SLAVE1  (6U << 8)                   /*!< RW: Input selection PWM_SLAVE1 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PWM_SLAVE2  (7U << 8)                   /*!< RW: Input selection PWM_SLAVE2 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PWM_MASTER2 (8U << 8)                   /*!< RW: Input selection PWM_MASTER2 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PWM_SLAVE3  (9U << 8)                   /*!< RW: Input selection PWM_SLAVE3 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO4     (10U << 8)                      /*!< RW: Input selection IO4 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO5     (11U << 8)                      /*!< RW: Input selection IO5 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO6     (12U << 8)                      /*!< RW: Input selection IO6 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO7     (13U << 8)                      /*!< RW: Input selection IO7 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO8     (14U << 8)                      /*!< RW: Input selection IO8 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO9     (15U << 8)                      /*!< RW: Input selection IO9 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO10    (16U << 8)                      /*!< RW: Input selection IO10 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO11    (17U << 8)                      /*!< RW: Input selection IO11 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PWM_SLAVE4  (18U << 8)                  /*!< RW: Input selection PWM_SLAVE4 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PH_IN0  (19U << 8)                      /*!< RW: Input selection PH_IN0 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PH_IN1  (20U << 8)                      /*!< RW: Input selection PH_IN1 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PH_IN2  (21U << 8)                      /*!< RW: Input selection PH_IN2 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PH_IN3  (22U << 8)                      /*!< RW: Input selection PH_IN3 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PH_IN4  (23U << 8)                      /*!< RW: Input selection PH_IN4 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PH_IN5  (24U << 8)                      /*!< RW: Input selection PH_IN5 for timer[0] channel B */
#define M_PORT_TIMER_CFG_TIMER0_CHA_SEL         (31U << 0)                      /*!< RW: Input selection for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO0     (0U << 0)                       /*!< RW: Input selection IO0 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO1     (1U << 0)                       /*!< RW: Input selection IO1 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO2     (2U << 0)                       /*!< RW: Input selection IO2 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO3     (3U << 0)                       /*!< RW: Input selection IO3 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_LIN_XRX (4U << 0)                       /*!< RW: Input selection LIN_XRX for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PWM_MASTER1 (5U << 0)                   /*!< RW: Input selection PWM_MASTER1 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PWM_SLAVE1  (6U << 0)                   /*!< RW: Input selection PWM_SLAVE1 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PWM_SLAVE2  (7U << 0)                   /*!< RW: Input selection PWM_SLAVE2 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PWM_MASTER2 (8U << 0)                   /*!< RW: Input selection PWM_MASTER2 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PWM_SLAVE3  (9U << 0)                   /*!< RW: Input selection PWM_SLAVE3 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO4     (10U << 0)                      /*!< RW: Input selection IO4 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO5     (11U << 0)                      /*!< RW: Input selection IO5 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO6     (12U << 0)                      /*!< RW: Input selection IO6 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO7     (13U << 0)                      /*!< RW: Input selection IO7 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO8     (14U << 0)                      /*!< RW: Input selection IO8 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO9     (15U << 0)                      /*!< RW: Input selection IO9 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO10    (16U << 0)                      /*!< RW: Input selection IO10 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO11    (17U << 0)                      /*!< RW: Input selection IO11 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PWM_SLAVE4  (18U << 0)                  /*!< RW: Input selection PWM_SLAVE4 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PH_IN0  (19U << 0)                      /*!< RW: Input selection PH_IN0 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PH_IN1  (20U << 0)                      /*!< RW: Input selection PH_IN1 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PH_IN2  (21U << 0)                      /*!< RW: Input selection PH_IN2 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PH_IN3  (22U << 0)                      /*!< RW: Input selection PH_IN3 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PH_IN4  (23U << 0)                      /*!< RW: Input selection PH_IN4 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PH_IN5  (24U << 0)                      /*!< RW: Input selection PH_IN5 for timer[0] channel A */

/* ********************** */
/* Block: PORT_TIMER_CFG1 */
/* ********************** */
extern volatile uint16_t IO_PORT_TIMER_CFG1 __attribute__((nodp, addr(0x00226)));  /*!< IO_PORT_TIMER_CFG1 */
#define M_PORT_TIMER_CFG1_TIMER1_CHB_SEL        (31U << 8)                      /*!< RW: Input selection for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO0    (0U << 8)                       /*!< RW: Input selection IO0 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO1    (1U << 8)                       /*!< RW: Input selection IO1 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO2    (2U << 8)                       /*!< RW: Input selection IO2 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO3    (3U << 8)                       /*!< RW: Input selection IO3 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_LIN_XRX (4U << 8)                      /*!< RW: Input selection LIN_XRX for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PWM_MASTER1 (5U << 8)                  /*!< RW: Input selection PWM_MASTER1 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PWM_SLAVE1  (6U << 8)                  /*!< RW: Input selection PWM_SLAVE1 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PWM_SLAVE2  (7U << 8)                  /*!< RW: Input selection PWM_SLAVE2 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PWM_MASTER2 (8U << 8)                  /*!< RW: Input selection PWM_MASTER2 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PWM_SLAVE3  (9U << 8)                  /*!< RW: Input selection PWM_SLAVE3 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO4    (10U << 8)                      /*!< RW: Input selection IO4 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO5    (11U << 8)                      /*!< RW: Input selection IO5 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO6    (12U << 8)                      /*!< RW: Input selection IO6 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO7    (13U << 8)                      /*!< RW: Input selection IO7 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO8    (14U << 8)                      /*!< RW: Input selection IO8 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO9    (15U << 8)                      /*!< RW: Input selection IO9 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO10   (16U << 8)                      /*!< RW: Input selection IO10 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO11   (17U << 8)                      /*!< RW: Input selection IO11 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PWM_SLAVE4  (18U << 8)                 /*!< RW: Input selection PWM_SLAVE4 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PH_IN0 (19U << 8)                      /*!< RW: Input selection PH_IN0 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PH_IN1 (20U << 8)                      /*!< RW: Input selection PH_IN1 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PH_IN2 (21U << 8)                      /*!< RW: Input selection PH_IN2 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PH_IN3 (22U << 8)                      /*!< RW: Input selection PH_IN3 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PH_IN4 (23U << 8)                      /*!< RW: Input selection PH_IN4 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PH_IN5 (24U << 8)                      /*!< RW: Input selection PH_IN5 for timer[1] channel B */
#define M_PORT_TIMER_CFG1_TIMER1_CHA_SEL        (31U << 0)                      /*!< RW: Input selection for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO0    (0U << 0)                       /*!< RW: Input selection IO0 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO1    (1U << 0)                       /*!< RW: Input selection IO1 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO2    (2U << 0)                       /*!< RW: Input selection IO2 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO3    (3U << 0)                       /*!< RW: Input selection IO3 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_LIN_XRX (4U << 0)                      /*!< RW: Input selection LIN_XRX for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PWM_MASTER1 (5U << 0)                  /*!< RW: Input selection PWM_MASTER1 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PWM_SLAVE1  (6U << 0)                  /*!< RW: Input selection PWM_SLAVE1 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PWM_SLAVE2  (7U << 0)                  /*!< RW: Input selection PWM_SLAVE2 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PWM_MASTER2 (8U << 0)                  /*!< RW: Input selection PWM_MASTER2 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PWM_SLAVE3  (9U << 0)                  /*!< RW: Input selection PWM_SLAVE3 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO4    (10U << 0)                      /*!< RW: Input selection IO4 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO5    (11U << 0)                      /*!< RW: Input selection IO5 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO6    (12U << 0)                      /*!< RW: Input selection IO6 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO7    (13U << 0)                      /*!< RW: Input selection IO7 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO8    (14U << 0)                      /*!< RW: Input selection IO8 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO9    (15U << 0)                      /*!< RW: Input selection IO9 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO10   (16U << 0)                      /*!< RW: Input selection IO10 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO11   (17U << 0)                      /*!< RW: Input selection IO11 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PWM_SLAVE4  (18U << 0)                 /*!< RW: Input selection PWM_SLAVE4 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PH_IN0 (19U << 0)                      /*!< RW: Input selection PH_IN0 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PH_IN1 (20U << 0)                      /*!< RW: Input selection PH_IN1 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PH_IN2 (21U << 0)                      /*!< RW: Input selection PH_IN2 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PH_IN3 (22U << 0)                      /*!< RW: Input selection PH_IN3 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PH_IN4 (23U << 0)                      /*!< RW: Input selection PH_IN4 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PH_IN5 (24U << 0)                      /*!< RW: Input selection PH_IN5 for timer[1] channel A */

/* ******************** */
/* Block: PORT_COMM_CFG */
/* ******************** */
extern volatile uint16_t IO_PORT_COMM_CFG __attribute__((nodp, addr(0x00228)));  /*!< IO_PORT_COMM_CFG */
#define M_PORT_COMM_CFG_SPI_SS_IN_SEL           (15U << 12)                     /*!< RW: IO selection for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO0       (0U << 12)                      /*!< RW: IO selection IO0 for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO1       (1U << 12)                      /*!< RW: IO selection IO1 for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO2       (2U << 12)                      /*!< RW: IO selection IO2 for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO3       (3U << 12)                      /*!< RW: IO selection IO3 for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO4       (4U << 12)                      /*!< RW: IO selection IO4 for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO5       (5U << 12)                      /*!< RW: IO selection IO5 for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO6       (6U << 12)                      /*!< RW: IO selection IO6 for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO7       (7U << 12)                      /*!< RW: IO selection IO7 for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO8       (8U << 12)                      /*!< RW: IO selection IO8 for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO9       (9U << 12)                      /*!< RW: IO selection IO9 for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO10      (10U << 12)                     /*!< RW: IO selection IO10 for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO11      (11U << 12)                     /*!< RW: IO selection IO11 for SPI_SS_IN */
#define M_PORT_COMM_CFG_SPI_SCK_IN_SEL          (15U << 8)                      /*!< RW: IO selection for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO0      (0U << 8)                       /*!< RW: IO selection IO0 for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO1      (1U << 8)                       /*!< RW: IO selection IO1 for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO2      (2U << 8)                       /*!< RW: IO selection IO2 for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO3      (3U << 8)                       /*!< RW: IO selection IO3 for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO4      (4U << 8)                       /*!< RW: IO selection IO4 for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO5      (5U << 8)                       /*!< RW: IO selection IO5 for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO6      (6U << 8)                       /*!< RW: IO selection IO6 for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO7      (7U << 8)                       /*!< RW: IO selection IO7 for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO8      (8U << 8)                       /*!< RW: IO selection IO8 for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO9      (9U << 8)                       /*!< RW: IO selection IO9 for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO10     (10U << 8)                      /*!< RW: IO selection IO10 for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO11     (11U << 8)                      /*!< RW: IO selection IO11 for SPI_SCK_IN */
#define M_PORT_COMM_CFG_SPI_MISO_IN_SEL         (15U << 4)                      /*!< RW: IO selection for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO0     (0U << 4)                       /*!< RW: IO selection IO0 for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO1     (1U << 4)                       /*!< RW: IO selection IO1 for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO2     (2U << 4)                       /*!< RW: IO selection IO2 for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO3     (3U << 4)                       /*!< RW: IO selection IO3 for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO4     (4U << 4)                       /*!< RW: IO selection IO4 for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO5     (5U << 4)                       /*!< RW: IO selection IO5 for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO6     (6U << 4)                       /*!< RW: IO selection IO6 for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO7     (7U << 4)                       /*!< RW: IO selection IO7 for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO8     (8U << 4)                       /*!< RW: IO selection IO8 for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO9     (9U << 4)                       /*!< RW: IO selection IO9 for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO10    (10U << 4)                      /*!< RW: IO selection IO10 for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO11    (11U << 4)                      /*!< RW: IO selection IO11 for SPI_MISO_IN */
#define M_PORT_COMM_CFG_SPI_MOSI_IN_SEL         (15U << 0)                      /*!< RW: IO selection for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO0     (0U << 0)                       /*!< RW: IO selection IO0 for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO1     (1U << 0)                       /*!< RW: IO selection IO1 for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO2     (2U << 0)                       /*!< RW: IO selection IO2 for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO3     (3U << 0)                       /*!< RW: IO selection IO3 for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO4     (4U << 0)                       /*!< RW: IO selection IO4 for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO5     (5U << 0)                       /*!< RW: IO selection IO5 for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO6     (6U << 0)                       /*!< RW: IO selection IO6 for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO7     (7U << 0)                       /*!< RW: IO selection IO7 for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO8     (8U << 0)                       /*!< RW: IO selection IO8 for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO9     (9U << 0)                       /*!< RW: IO selection IO9 for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO10    (10U << 0)                      /*!< RW: IO selection IO10 for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO11    (11U << 0)                      /*!< RW: IO selection IO11 for SPI_MOSI_IN */

/* ********************* */
/* Block: PORT_COMM_CFG1 */
/* ********************* */
extern volatile uint16_t IO_PORT_COMM_CFG1 __attribute__((nodp, addr(0x0022A)));  /*!< IO_PORT_COMM_CFG1 */
#define M_PORT_COMM_CFG1_UART1_RX_SEL           (15U << 4)                      /*!< RW: IO selection for UART1 RX */
#define C_PORT_COMM_CFG1_UART1_RX_SEL_IO11      (11U << 4)                      /*!< IO_IN[11] */
#define C_PORT_COMM_CFG1_UART1_RX_SEL_IO10      (10U << 4)                      /*!< IO_IN[10] */
#define C_PORT_COMM_CFG1_UART1_RX_SEL_IO9       (9U << 4)                       /*!< IO_IN[9] */
#define C_PORT_COMM_CFG1_UART1_RX_SEL_IO8       (8U << 4)                       /*!< IO_IN[8] */
#define C_PORT_COMM_CFG1_UART1_RX_SEL_IO7       (7U << 4)                       /*!< IO_IN[7] */
#define C_PORT_COMM_CFG1_UART1_RX_SEL_IO6       (6U << 4)                       /*!< IO_IN[6] */
#define C_PORT_COMM_CFG1_UART1_RX_SEL_IO5       (5U << 4)                       /*!< IO_IN[5] */
#define C_PORT_COMM_CFG1_UART1_RX_SEL_IO4       (4U << 4)                       /*!< IO_IN[4] */
#define C_PORT_COMM_CFG1_UART1_RX_SEL_IO3       (3U << 4)                       /*!< IO_IN[3] */
#define C_PORT_COMM_CFG1_UART1_RX_SEL_IO2       (2U << 4)                       /*!< IO_IN[2] */
#define C_PORT_COMM_CFG1_UART1_RX_SEL_IO1       (1U << 4)                       /*!< IO_IN[1] */
#define C_PORT_COMM_CFG1_UART1_RX_SEL_IO0       (0U << 4)                       /*!< IO_IN[0] */
#define M_PORT_COMM_CFG1_UART0_RX_SEL           (15U << 0)                      /*!< RW: IO selection for UART0 RX */
#define C_PORT_COMM_CFG1_UART0_RX_SEL_IO11      (11U << 0)                      /*!< IO_IN[11] */
#define C_PORT_COMM_CFG1_UART0_RX_SEL_IO10      (10U << 0)                      /*!< IO_IN[10] */
#define C_PORT_COMM_CFG1_UART0_RX_SEL_IO9       (9U << 0)                       /*!< IO_IN[9] */
#define C_PORT_COMM_CFG1_UART0_RX_SEL_IO8       (8U << 0)                       /*!< IO_IN[8] */
#define C_PORT_COMM_CFG1_UART0_RX_SEL_IO7       (7U << 0)                       /*!< IO_IN[7] */
#define C_PORT_COMM_CFG1_UART0_RX_SEL_IO6       (6U << 0)                       /*!< IO_IN[6] */
#define C_PORT_COMM_CFG1_UART0_RX_SEL_IO5       (5U << 0)                       /*!< IO_IN[5] */
#define C_PORT_COMM_CFG1_UART0_RX_SEL_IO4       (4U << 0)                       /*!< IO_IN[4] */
#define C_PORT_COMM_CFG1_UART0_RX_SEL_IO3       (3U << 0)                       /*!< IO_IN[3] */
#define C_PORT_COMM_CFG1_UART0_RX_SEL_IO2       (2U << 0)                       /*!< IO_IN[2] */
#define C_PORT_COMM_CFG1_UART0_RX_SEL_IO1       (1U << 0)                       /*!< IO_IN[1] */
#define C_PORT_COMM_CFG1_UART0_RX_SEL_IO0       (0U << 0)                       /*!< IO_IN[0] */

/* ******************** */
/* Block: PORT_MISC_OUT */
/* ******************** */
extern volatile uint16_t IO_PORT_MISC_OUT __attribute__((nodp, addr(0x0022C)));  /*!< IO_PORT_MISC_OUT */
#define M_PORT_MISC_OUT_SEL_TEMP                (31U << 11)                     /*!< RW: select the temperature diode input for the temp. sensor */
#define C_PORT_MISC_OUT_SEL_TEMP_VBGA           (0U << 11)                      /*!< RW: Diode inside band-gap VDDA */
#define C_PORT_MISC_OUT_SEL_TEMP_VBGD           (1U << 11)                      /*!< RW: Diode inside band-gap VDDD */
#define C_PORT_MISC_OUT_SEL_TEMP_VDDA           (2U << 11)                      /*!< RW: Diode inside regulator VDDA */
#define C_PORT_MISC_OUT_SEL_TEMP_VDDD           (3U << 11)                      /*!< RW: Diode inside regulator VDDD */
#define C_PORT_MISC_OUT_SEL_TEMP_DEIM           (4U << 11)                      /*!< No temperature diode, monitor integrity of die edge */
#define C_PORT_MISC_OUT_SEL_TEMP_ADCMUX         (11U << 11)                     /*!< Diode inside ADC multiplexer */
#define M_PORT_MISC_OUT_PROV_VS                 (3U << 9)                       /*!< RW: control for VS over voltage monitor */
#define C_PORT_MISC_OUT_PROV_0                  (1U << 9)                       /*!< RW: control for VS over voltage unit */
#define M_PORT_MISC_OUT_PRUV_VS                 (7U << 6)                       /*!< RW: under voltage programming for VS; detection level (V) = (PRUV_VS+4) -> from 4 to 9V */
#define C_PORT_MISC_OUT_PRUV_0                  (1U << 6)                       /*!< RW: under voltage programming for VS-unit */
#define B_PORT_MISC_OUT_SWITCH_VDDA_TO_5V       (1U << 5)                       /*!< RW: switch VDDA supply to 5V */
#define M_PORT_MISC_OUT_WUI                     (3U << 3)                       /*!< RW: define internal wake up time delay in periods of CK10K : */
#define C_PORT_MISC_OUT_WUI_OFF                 (0U << 3)                       /*!< 00 = Off */
#define C_PORT_MISC_OUT_WUI_400ms               (1U << 3)                       /*!< 01 = 4096/10kHz */
#define C_PORT_MISC_OUT_WUI_800ms               (2U << 3)                       /*!< 10 = 8192/10kHz */
#define C_PORT_MISC_OUT_WUI_1600ms              (3U << 3)                       /*!< 11 = 16384/10kHz */
#define B_PORT_MISC_OUT_CLEAR_RSTAT             (1U << 2)                       /*!< RW: setting this bit will force the RSTAST flip flop in analogue asynchronously to low */
#define B_PORT_MISC_OUT_SET_RSTAT               (1U << 1)                       /*!< RW: setting this bit will force the RSTAST flip flop in analogue asynchronously to high */
#define B_PORT_MISC_OUT_CLEAR_STOP              (1U << 0)                       /*!< RW: signal to clear STOP_MODE flip flop in analogue part, must be cleared before entering STOP mode */

/* ********************* */
/* Block: PORT_MISC2_OUT */
/* ********************* */
extern volatile uint16_t IO_PORT_MISC2_OUT __attribute__((nodp, addr(0x0022E)));  /*!< IO_PORT_MISC2_OUT */
#define B_PORT_MISC2_OUT_DIS_TEMPSENSE          (1U << 11)                      /*!< RW: Disable the ADC temperature sensor */
#define B_PORT_MISC2_OUT_VSM_FILT_ON            (1U << 10)                      /*!< RW: Enable VSM supply filtering  */
#define B_PORT_MISC2_OUT_ENABLE_OTD             (1U << 9)                       /*!< RW: Enable over-temperature detection */
#define B_PORT_MISC2_OUT_WU_IO_EN               (1U << 8)                       /*!< RW: wake up enable for IOs */
#define M_PORT_MISC2_OUT_AOUT_SUP               (15U << 4)                      /*!< RW: spare control output bits for supply system */
#define B_PORT_MISC2_OUT_PRUV_VDDA              (1U << 0)                       /*!< RW: under voltage programming for VDDA */

/* *********************** */
/* Block: PORT_STOPMD_CTRL */
/* *********************** */
extern volatile uint16_t IO_PORT_STOPMD_CTRL_S __attribute__((nodp, addr(0x00230)));  /*!< IO_PORT_STOPMD_CTRL_S (System) */
#define B_PORT_STOPMD_CTRL_SEL_STOP_MODE        (1U << 0)                       /*!< RW: CPU HALT will enter STOPMODE, not SLEEP */

/* ********************** */
/* Block: PORT_STOPMD_CFG */
/* ********************** */
extern volatile uint16_t IO_PORT_STOPMD_CFG __attribute__((nodp, addr(0x00232)));  /*!< IO_PORT_STOPMD_CFG */
#define B_PORT_STOPMD_CFG_PRE_SBY_RCO1M         (1U << 4)                       /*!< RW: set 1 MHz Oscillator in standby mode */
#define B_PORT_STOPMD_CFG_PRE_SBY_RCO32M        (1U << 3)                       /*!< RW: set 32 MHz Oscillator in standby mode */
#define B_PORT_STOPMD_CFG_PRE_SWITCHOFFOV_VS    (1U << 2)                       /*!< RW: disable over voltage detection for VS */
#define B_PORT_STOPMD_CFG_PRE_SWITCHOFFUV_VS    (1U << 1)                       /*!< RW: disable under voltage detection for VS */
#define B_PORT_STOPMD_CFG_PRE_SWITCHOFFUV_VDDA  (1U << 0)                       /*!< RW: disable under voltage detection of VDDA regulator to allow low voltage test */

/* ******************** */
/* Block: PORT_DIS_GTSM */
/* ******************** */
extern volatile uint16_t IO_PORT_DIS_GTSM __attribute__((nodp, addr(0x00234)));  /*!< IO_PORT_DIS_GTSM */
#define B_PORT_DIS_GTSM_DIS_GTSM                (1U << 0)                       /*!< RW: disable GTSM, if set, HALTED will not trigger a GTSM signal */

/* ******************** */
/* Block: PORT_LIN_XCFG */
/* ******************** */
extern volatile uint16_t IO_PORT_LIN_XCFG_S __attribute__((nodp, addr(0x00236)));  /*!< IO_PORT_LIN_XCFG_S (System) */
#define M_PORT_LIN_XCFG_LIN_XCFG                (65535U << 0)                   /*!< RW: result taken in account only if XKEY is valid */
#define B_PORT_LIN_XCFG_LIN_TEST_WARM           (1U << 15)                      /*!< Enable test mode warm activation over LIN pin */
#define B_PORT_LIN_XCFG_CXPI_DIS_WU_DEB         (1U << 13)                      /*!< Decrease the debounce time of the wake-up comparator from 70us to 5.5us to support CXPI protocol */
#define B_PORT_LIN_XCFG_ENA_LIN_REV_PROT        (1U << 12)                      /*!< Disconnects the reverse polarity protection from internal LIN node, is needed to measure LIN level by ADC or to run fast protocol at 5V level (PPM, CXPI) */
#define B_PORT_LIN_XCFG_SEL_RXD_ATDI            (1U << 11)                      /*!< 1: the fast comparator used in test mode (ATDI) will be switched to the RX input (this allows protocol with higher baudrate, e.g. PPM, FASTLIN or CXPI) */
#define B_PORT_LIN_XCFG_RX_INVERT               (1U << 10)                      /*!< Invert the RX input before any multiplexing */
#define B_PORT_LIN_XCFG_DISTERM                 (1U << 9)                       /*!< Disable bus termination for auto-addressing */
#define B_PORT_LIN_XCFG_BYPASS                  (1U << 8)                       /*!< Bypass the receiver for high-speed mode */
#define B_PORT_LIN_XCFG_HSM                     (1U << 7)                       /*!< High-speed mode (slew rate disabled) */
#define B_PORT_LIN_XCFG_LSM                     (1U << 6)                       /*!< Low speed slope control */
#define B_PORT_LIN_XCFG_SLEEPB                  (1U << 5)                       /*!< Disable sleep mode */
#define B_PORT_LIN_XCFG_SEL_COLIN_B             (1U << 4)                       /*!< Select COLIN-B */
#define B_PORT_LIN_XCFG_SEL_IO_TO_COLINRX       (1U << 3)                       /*!< Select COLIN_RX driven from IO */
#define B_PORT_LIN_XCFG_SEL_RX_IO               (1U << 2)                       /*!< Select RX driven from IO */
#define B_PORT_LIN_XCFG_TX_INVERT               (1U << 1)                       /*!< Invert TX output */
#define B_PORT_LIN_XCFG_SEL_TX_EXT              (1U << 0)                       /*!< Select TX driver from IO */

/* ************************** */
/* Block: PORT_LIN_XCFG_VALID */
/* ************************** */
extern volatile uint16_t IO_PORT_LIN_XCFG_VALID __attribute__((nodp, addr(0x00238)));  /*!< IO_PORT_LIN_XCFG_VALID */
#define M_PORT_LIN_XCFG_VALID_LIN_XCFG_VALID    (65535U << 0)                   /*!< R: Value for digital read */

/* ********************** */
/* Block: PORT_CLOCK_CTRL */
/* ********************** */
extern volatile uint16_t IO_PORT_CLOCK_CTRL_S __attribute__((nodp, addr(0x0023A)));  /*!< IO_PORT_CLOCK_CTRL_S (System) */
#define B_PORT_CLOCK_CTRL_AC_SEL                (1U << 0)                       /*!< RW: switch to 16 MHz clock if set */

/* **************** */
/* Block: TRIM_MISC */
/* **************** */
extern volatile uint16_t IO_TRIM_MISC __attribute__((nodp, addr(0x0023C)));     /*!< IO_TRIM_MISC */
#define B_TRIM_MISC_LOCK                        (1U << 15)                      /*!< W: Lock the port. Write is invalid when set. Always read as 0.; R: Always read 0 */
#define B_TRIM_MISC_ENA_IO5_RESETB              (1U << 8)                       /*!< Enable IO5 as Reset-IO; Reset when IO goes to '0' */
#define M_TRIM_MISC_TRIM_SDAFILT_IO             (3U << 6)                       /*!< RW: Calibration. Write is invalid if LOCK is set */
#define M_TRIM_MISC_TRIM_OTD                    (63U << 0)                      /*!< RW: Calibration. Write is invalid if LOCK is set */

/* **************** */
/* Block: TRIM1_DRV */
/* **************** */
extern volatile uint16_t IO_TRIM1_DRV __attribute__((nodp, addr(0x0023E)));     /*!< IO_TRIM1_DRV */
#define B_TRIM1_DRV_LOCK                        (1U << 15)                      /*!< W: Lock the port. Write is invalid when set. Always read as 0.; R: Always read 0 */
#define M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK       (1023U << 2)                    /*!< RW: trim frequency of driver clock */
#define M_TRIM1_DRV_TRIM_DRVSUP                 (3U << 0)                       /*!< RW: trim output level of driver supply */

/* **************** */
/* Block: TRIM2_DRV */
/* **************** */
extern volatile uint16_t IO_TRIM2_DRV __attribute__((nodp, addr(0x00240)));     /*!< IO_TRIM2_DRV */
#define B_TRIM2_DRV_LOCK                        (1U << 15)                      /*!< W: Lock the port. Write is invalid when set. Always read as 0.; R: Always read 0 */
#define M_TRIM2_DRV_TRIM_SLWRT                  (15U << 0)                      /*!< RW: trim slew/rate / slope of drivers */

/* ******************* */
/* Block: PORT_DRV_OUT */
/* ******************* */
extern volatile uint16_t IO_PORT_DRV_OUT __attribute__((nodp, addr(0x00242)));  /*!< IO_PORT_DRV_OUT */
#define B_PORT_DRV_OUT_PARALLEL_MODE_DRV        (1U << 10)                      /*!< RW: Phase RS, TU and VW switch parallel for cross-current detection (DC-mode) */
#define M_PORT_DRV_OUT_DRVMOD_OPTION            (3U << 8)                       /*!< RW: select divider ratio of driver clock */
#define M_PORT_DRV_OUT_ENABLE_DRV               (63U << 2)                      /*!< RW: enable output stages */
#define B_PORT_DRV_OUT_ENABLE_DRV0              ((1U << 0) << 2)                /*!< RW: enable output stages DRV0 (R) */
#define B_PORT_DRV_OUT_ENABLE_DRV1              ((1U << 1) << 2)                /*!< RW: enable output stages DRV1 (S) */
#define B_PORT_DRV_OUT_ENABLE_DRV2              ((1U << 2) << 2)                /*!< RW: enable output stages DRV2 (T) */
#define B_PORT_DRV_OUT_ENABLE_DRV3              ((1U << 3) << 2)                /*!< RW: enable output stages DRV3 (U) */
#define B_PORT_DRV_OUT_ENABLE_DRV4              ((1U << 4) << 2)                /*!< RW: enable output stages DRV4 (V) */
#define B_PORT_DRV_OUT_ENABLE_DRV5              ((1U << 5) << 2)                /*!< RW: enable output stages DRV5 (W) */
#define B_PORT_DRV_OUT_ENABLE_DRVMOD_CPCLK      (1U << 1)                       /*!< RW: enable driver clock */
#define B_PORT_DRV_OUT_ENABLE_DRVSUP            (1U << 0)                       /*!< RW: enable driver supply */

/* ********************* */
/* Block: PORT_DRV1_TEST */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV1_TEST __attribute__((nodp, addr(0x00244)));  /*!< IO_PORT_DRV1_TEST */
#define B_PORT_DRV1_TEST_TST_STRS_VHS_LS_DRV    (1U << 15)                      /*!< RW: Value for analogue */
#define B_PORT_DRV1_TEST_TST_STRS_VHS_HS_DRV    (1U << 14)                      /*!< RW: Value for analogue */
#define M_PORT_DRV1_TEST_TST_OUT_HV0            (63U << 8)                      /*!< RW: Value for analogue */
#define M_PORT_DRV1_TEST_TST_OUT_LV1            (63U << 2)                      /*!< RW: Value for analogue */
#define B_PORT_DRV1_TEST_TST_VBGCS_LV0_DRVSUP   (1U << 1)                       /*!< RW: Value for analogue */
#define B_PORT_DRV1_TEST_TST_UVCMP_LV1_DRVSUP   (1U << 0)                       /*!< RW: Value for analogue */

/* ********************* */
/* Block: PORT_DRV2_TEST */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV2_TEST __attribute__((nodp, addr(0x00246)));  /*!< IO_PORT_DRV2_TEST */
#define M_PORT_DRV2_TEST_TST_OUT_LV0            (63U << 5)                      /*!< RW: Value for analogue */
#define B_PORT_DRV2_TEST_TST_DRVSUP_DISC_CMP    (1U << 4)                       /*!< RW: Value for analogue */
#define M_PORT_DRV2_TEST_TEST_DRV_ILD           (3U << 2)                       /*!< RW: Value for analogue */
#define B_PORT_DRV2_TEST_TEST_DRVMOD_CPCLK      (1U << 1)                       /*!< RW: Value for analogue */
#define B_PORT_DRV2_TEST_TST_DRVSUP             (1U << 0)                       /*!< RW: Value for analogue */

/* ********************* */
/* Block: PORT_DRV3_TEST */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV3_TEST __attribute__((nodp, addr(0x00248)));  /*!< IO_PORT_DRV3_TEST */
#define B_PORT_DRV3_TEST_TST_DRVMOD_CPCLK_DAC   (1U << 7)                       /*!< RW: Value for analogue */
#define M_PORT_DRV3_TEST_TST_GHS_NOCURR         (63U << 0)                      /*!< RW: Value for analogue */

/* ********************* */
/* Block: PORT_DRV1_CTRL */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV1_CTRL __attribute__((nodp, addr(0x0024A)));  /*!< IO_PORT_DRV1_CTRL */
#define M_PORT_DRV1_CTRL_DRV3_CTRL              (15U << 12)                     /*!< RW: Control of PWM output for phase 3 (U) */
#define C_PORT_DRV1_CTRL_DRV3_MASTER1           (0U << 12)                      /*!< RW: Control DRV2 by PWM_MASTER1 */
#define C_PORT_DRV1_CTRL_DRV3_SLAVE1            (1U << 12)                      /*!< RW: Control DRV2 by PWM_SLAVE1 */
#define C_PORT_DRV1_CTRL_DRV3_SLAVE2            (2U << 12)                      /*!< RW: Control DRV2 by PWM_SLAVE2 */
#define C_PORT_DRV1_CTRL_DRV3_MASTER2           (3U << 12)                      /*!< RW: Control DRV2 by PWM_MASTER2 */
#define C_PORT_DRV1_CTRL_DRV3_SLAVE3            (4U << 12)                      /*!< RW: Control DRV2 by PWM_SLAVE3 */
#define C_PORT_DRV1_CTRL_DRV3_SLAVE4            (5U << 12)                      /*!< RW: Control DRV2 by PWM_SLAVE4 */
#define C_PORT_DRV1_CTRL_DRV3_IO0               (6U << 12)                      /*!< RW: Control DRV2 by IO0 */
#define C_PORT_DRV1_CTRL_DRV3_IO1               (7U << 12)                      /*!< RW: Control DRV2 by IO1 */
#define C_PORT_DRV1_CTRL_DRV3_IO2               (8U << 12)                      /*!< RW: Control DRV2 by IO2 */
#define C_PORT_DRV1_CTRL_DRV3_IO3               (9U << 12)                      /*!< RW: Control DRV2 by IO3 */
#define C_PORT_DRV1_CTRL_DRV3_LIN_XRX           (10U << 12)                     /*!< RW: Control DRV2 by LIN_XRX */
#define C_PORT_DRV1_CTRL_DRV3_CTIMER0           (11U << 12)                     /*!< RW: Control DRV2 by CTIMER0 */
#define C_PORT_DRV1_CTRL_DRV3_CTIMER1           (12U << 12)                     /*!< RW: Control DRV2 by CTIMER1 */
#define C_PORT_DRV1_CTRL_DRV3_L                 (13U << 12)                     /*!< RW: Control DRV2 LOW */
#define C_PORT_DRV1_CTRL_DRV3_TRISTATE          (14U << 12)                     /*!< RW: Control DRV2 TRI-STATE */
#define C_PORT_DRV1_CTRL_DRV3_H                 (15U << 12)                     /*!< RW: Control DRV2 HIGH */
#define M_PORT_DRV1_CTRL_DRV2_CTRL              (15U << 8)                      /*!< RW: Control of PWM output for phase 2 (T) */
#define C_PORT_DRV1_CTRL_DRV2_MASTER1           (0U << 8)                       /*!< RW: Control DRV2 by PWM_MASTER1 */
#define C_PORT_DRV1_CTRL_DRV2_SLAVE1            (1U << 8)                       /*!< RW: Control DRV2 by PWM_SLAVE1 */
#define C_PORT_DRV1_CTRL_DRV2_SLAVE2            (2U << 8)                       /*!< RW: Control DRV2 by PWM_SLAVE2 */
#define C_PORT_DRV1_CTRL_DRV2_MASTER2           (3U << 8)                       /*!< RW: Control DRV2 by PWM_MASTER2 */
#define C_PORT_DRV1_CTRL_DRV2_SLAVE3            (4U << 8)                       /*!< RW: Control DRV2 by PWM_SLAVE3 */
#define C_PORT_DRV1_CTRL_DRV2_SLAVE4            (5U << 8)                       /*!< RW: Control DRV2 by PWM_SLAVE4 */
#define C_PORT_DRV1_CTRL_DRV2_IO0               (6U << 8)                       /*!< RW: Control DRV2 by IO0 */
#define C_PORT_DRV1_CTRL_DRV2_IO1               (7U << 8)                       /*!< RW: Control DRV2 by IO1 */
#define C_PORT_DRV1_CTRL_DRV2_IO2               (8U << 8)                       /*!< RW: Control DRV2 by IO2 */
#define C_PORT_DRV1_CTRL_DRV2_IO3               (9U << 8)                       /*!< RW: Control DRV2 by IO3 */
#define C_PORT_DRV1_CTRL_DRV2_LIN_XRX           (10U << 8)                      /*!< RW: Control DRV2 by LIN_XRX */
#define C_PORT_DRV1_CTRL_DRV2_CTIMER0           (11U << 8)                      /*!< RW: Control DRV2 by CTIMER0 */
#define C_PORT_DRV1_CTRL_DRV2_CTIMER1           (12U << 8)                      /*!< RW: Control DRV2 by CTIMER1 */
#define C_PORT_DRV1_CTRL_DRV2_L                 (13U << 8)                      /*!< RW: Control DRV2 LOW */
#define C_PORT_DRV1_CTRL_DRV2_TRISTATE          (14U << 8)                      /*!< RW: Control DRV2 TRI-STATE */
#define C_PORT_DRV1_CTRL_DRV2_H                 (15U << 8)                      /*!< RW: Control DRV2 HIGH */
#define M_PORT_DRV1_CTRL_DRV1_CTRL              (15U << 4)                      /*!< RW: Control of PWM output for phase 1 (S) */
#define C_PORT_DRV1_CTRL_DRV1_MASTER1           (0U << 4)                       /*!< RW: Control DRV1 by PWM_MASTER1 */
#define C_PORT_DRV1_CTRL_DRV1_SLAVE1            (1U << 4)                       /*!< RW: Control DRV1 by PWM_SLAVE1 */
#define C_PORT_DRV1_CTRL_DRV1_SLAVE2            (2U << 4)                       /*!< RW: Control DRV1 by PWM_SLAVE2 */
#define C_PORT_DRV1_CTRL_DRV1_MASTER2           (3U << 4)                       /*!< RW: Control DRV1 by PWM_MASTER2 */
#define C_PORT_DRV1_CTRL_DRV1_SLAVE3            (4U << 4)                       /*!< RW: Control DRV1 by PWM_SLAVE3 */
#define C_PORT_DRV1_CTRL_DRV1_SLAVE4            (5U << 4)                       /*!< RW: Control DRV1 by PWM_SLAVE4 */
#define C_PORT_DRV1_CTRL_DRV1_IO0               (6U << 4)                       /*!< RW: Control DRV1 by IO0 */
#define C_PORT_DRV1_CTRL_DRV1_IO1               (7U << 4)                       /*!< RW: Control DRV1 by IO1 */
#define C_PORT_DRV1_CTRL_DRV1_IO2               (8U << 4)                       /*!< RW: Control DRV1 by IO2 */
#define C_PORT_DRV1_CTRL_DRV1_IO3               (9U << 4)                       /*!< RW: Control DRV1 by IO3 */
#define C_PORT_DRV1_CTRL_DRV1_LIN_XRX           (10U << 4)                      /*!< RW: Control DRV1 by LIN_XRX */
#define C_PORT_DRV1_CTRL_DRV1_CTIMER0           (11U << 4)                      /*!< RW: Control DRV1 by CTIMER0 */
#define C_PORT_DRV1_CTRL_DRV1_CTIMER1           (12U << 4)                      /*!< RW: Control DRV1 by CTIMER1 */
#define C_PORT_DRV1_CTRL_DRV1_L                 (13U << 4)                      /*!< RW: Control DRV1 LOW */
#define C_PORT_DRV1_CTRL_DRV1_TRISTATE          (14U << 4)                      /*!< RW: Control DRV1 TRI-STATE */
#define C_PORT_DRV1_CTRL_DRV1_H                 (15U << 4)                      /*!< RW: Control DRV1 HIGH */
#define M_PORT_DRV1_CTRL_DRV0_CTRL              (15U << 0)                      /*!< RW: Control of PWM output for phase 0 (R) */
#define C_PORT_DRV1_CTRL_DRV0_MASTER1           (0U << 0)                       /*!< RW: Control DRV0 by PWM_MASTER1 */
#define C_PORT_DRV1_CTRL_DRV0_SLAVE1            (1U << 0)                       /*!< RW: Control DRV0 by PWM_SLAVE1 */
#define C_PORT_DRV1_CTRL_DRV0_SLAVE2            (2U << 0)                       /*!< RW: Control DRV0 by PWM_SLAVE2 */
#define C_PORT_DRV1_CTRL_DRV0_MASTER2           (3U << 0)                       /*!< RW: Control DRV0 by PWM_MASTER2 */
#define C_PORT_DRV1_CTRL_DRV0_SLAVE3            (4U << 0)                       /*!< RW: Control DRV0 by PWM_SLAVE3 */
#define C_PORT_DRV1_CTRL_DRV0_SLAVE4            (5U << 0)                       /*!< RW: Control DRV0 by PWM_SLAVE4 */
#define C_PORT_DRV1_CTRL_DRV0_IO0               (6U << 0)                       /*!< RW: Control DRV0 by IO0 */
#define C_PORT_DRV1_CTRL_DRV0_IO1               (7U << 0)                       /*!< RW: Control DRV0 by IO1 */
#define C_PORT_DRV1_CTRL_DRV0_IO2               (8U << 0)                       /*!< RW: Control DRV0 by IO2 */
#define C_PORT_DRV1_CTRL_DRV0_IO3               (9U << 0)                       /*!< RW: Control DRV0 by IO3 */
#define C_PORT_DRV1_CTRL_DRV0_LIN_XRX           (10U << 0)                      /*!< RW: Control DRV0 by LIN_XRX */
#define C_PORT_DRV1_CTRL_DRV0_CTIMER0           (11U << 0)                      /*!< RW: Control DRV0 by CTIMER0 */
#define C_PORT_DRV1_CTRL_DRV0_CTIMER1           (12U << 0)                      /*!< RW: Control DRV0 by CTIMER1 */
#define C_PORT_DRV1_CTRL_DRV0_L                 (13U << 0)                      /*!< RW: Control DRV0 LOW */
#define C_PORT_DRV1_CTRL_DRV0_TRISTATE          (14U << 0)                      /*!< RW: Control DRV0 TRI-STATE */
#define C_PORT_DRV1_CTRL_DRV0_H                 (15U << 0)                      /*!< RW: Control DRV0 HIGH */

/* ********************* */
/* Block: PORT_DRV2_CTRL */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV2_CTRL __attribute__((nodp, addr(0x0024C)));  /*!< IO_PORT_DRV2_CTRL */
#define M_PORT_DRV2_CTRL_DRV5_CTRL              (15U << 4)                      /*!< RW: Control of PWM output for phase 5 (V) */
#define C_PORT_DRV2_CTRL_DRV5_MASTER1           (0U << 4)                       /*!< RW: Control DRV5 by PWM_MASTER1 */
#define C_PORT_DRV2_CTRL_DRV5_SLAVE1            (1U << 4)                       /*!< RW: Control DRV5 by PWM_SLAVE1 */
#define C_PORT_DRV2_CTRL_DRV5_SLAVE2            (2U << 4)                       /*!< RW: Control DRV5 by PWM_SLAVE2 */
#define C_PORT_DRV2_CTRL_DRV5_MASTER2           (3U << 4)                       /*!< RW: Control DRV5 by PWM_MASTER2 */
#define C_PORT_DRV2_CTRL_DRV5_SLAVE3            (4U << 4)                       /*!< RW: Control DRV5 by PWM_SLAVE3 */
#define C_PORT_DRV2_CTRL_DRV5_SLAVE4            (5U << 4)                       /*!< RW: Control DRV5 by PWM_SLAVE4 */
#define C_PORT_DRV2_CTRL_DRV5_IO0               (6U << 4)                       /*!< RW: Control DRV5 by IO0 */
#define C_PORT_DRV2_CTRL_DRV5_IO1               (7U << 4)                       /*!< RW: Control DRV5 by IO1 */
#define C_PORT_DRV2_CTRL_DRV5_IO2               (8U << 4)                       /*!< RW: Control DRV5 by IO2 */
#define C_PORT_DRV2_CTRL_DRV5_IO3               (9U << 4)                       /*!< RW: Control DRV5 by IO3 */
#define C_PORT_DRV2_CTRL_DRV5_LIN_XRX           (10U << 4)                      /*!< RW: Control DRV5 by LIN_XRX */
#define C_PORT_DRV2_CTRL_DRV5_CTIMER0           (11U << 4)                      /*!< RW: Control DRV5 by CTIMER0 */
#define C_PORT_DRV2_CTRL_DRV5_CTIMER1           (12U << 4)                      /*!< RW: Control DRV5 by CTIMER1 */
#define C_PORT_DRV2_CTRL_DRV5_L                 (13U << 4)                      /*!< RW: Control DRV5 LOW */
#define C_PORT_DRV2_CTRL_DRV5_TRISTATE          (14U << 4)                      /*!< RW: Control DRV5 TRI-STATE */
#define C_PORT_DRV2_CTRL_DRV5_H                 (15U << 4)                      /*!< RW: Control DRV5 HIGH */
#define M_PORT_DRV2_CTRL_DRV4_CTRL              (15U << 0)                      /*!< RW: Control of PWM output for phase 4 (W) */
#define C_PORT_DRV2_CTRL_DRV4_MASTER1           (0U << 0)                       /*!< RW: Control DRV4 by PWM_MASTER1 */
#define C_PORT_DRV2_CTRL_DRV4_SLAVE1            (1U << 0)                       /*!< RW: Control DRV4 by PWM_SLAVE1 */
#define C_PORT_DRV2_CTRL_DRV4_SLAVE2            (2U << 0)                       /*!< RW: Control DRV4 by PWM_SLAVE2 */
#define C_PORT_DRV2_CTRL_DRV4_MASTER2           (3U << 0)                       /*!< RW: Control DRV4 by PWM_MASTER2 */
#define C_PORT_DRV2_CTRL_DRV4_SLAVE3            (4U << 0)                       /*!< RW: Control DRV4 by PWM_SLAVE3 */
#define C_PORT_DRV2_CTRL_DRV4_SLAVE4            (5U << 0)                       /*!< RW: Control DRV4 by PWM_SLAVE4 */
#define C_PORT_DRV2_CTRL_DRV4_IO0               (6U << 0)                       /*!< RW: Control DRV4 by IO0 */
#define C_PORT_DRV2_CTRL_DRV4_IO1               (7U << 0)                       /*!< RW: Control DRV4 by IO1 */
#define C_PORT_DRV2_CTRL_DRV4_IO2               (8U << 0)                       /*!< RW: Control DRV4 by IO2 */
#define C_PORT_DRV2_CTRL_DRV4_IO3               (9U << 0)                       /*!< RW: Control DRV4 by IO3 */
#define C_PORT_DRV2_CTRL_DRV4_LIN_XRX           (10U << 0)                      /*!< RW: Control DRV4 by LIN_XRX */
#define C_PORT_DRV2_CTRL_DRV4_CTIMER0           (11U << 0)                      /*!< RW: Control DRV4 by CTIMER0 */
#define C_PORT_DRV2_CTRL_DRV4_CTIMER1           (12U << 0)                      /*!< RW: Control DRV4 by CTIMER1 */
#define C_PORT_DRV2_CTRL_DRV4_L                 (13U << 0)                      /*!< RW: Control DRV4 LOW */
#define C_PORT_DRV2_CTRL_DRV4_TRISTATE          (14U << 0)                      /*!< RW: Control DRV4 TRI-STATE */
#define C_PORT_DRV2_CTRL_DRV4_H                 (15U << 0)                      /*!< RW: Control DRV4 HIGH */

/* ********************* */
/* Block: PORT_DRV1_PROT */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV1_PROT __attribute__((nodp, addr(0x0024E)));  /*!< IO_PORT_DRV1_PROT */
#define B_PORT_DRV1_PROT_DIS_OC_VDDA            (1U << 15)                      /*!< RW: Disable Over Current VDDA HW-protection */
#define B_PORT_DRV1_PROT_OC_VDDA_PM             (1U << 14)                      /*!< RW: Protection mode: 0: Tri-state; 1: Switch LS */
#define B_PORT_DRV1_PROT_DIS_OV_VDDA            (1U << 13)                      /*!< RW: Disable Over Voltage VDDA HW-protection */
#define B_PORT_DRV1_PROT_OV_VDDA_PM             (1U << 12)                      /*!< RW: Protection mode: 0: Tri-state; 1: Switch LS */
#define B_PORT_DRV1_PROT_DIS_OC                 (1U << 11)                      /*!< RW: Disable Over Current HW-protection */
#define B_PORT_DRV1_PROT_OC_PM                  (1U << 10)                      /*!< RW: Protection mode: 0: Tri-state; 1: Switch LS */
#define B_PORT_DRV1_PROT_DIS_UV_VDDAF           (1U << 9)                       /*!< RW: Disable Under Voltage VDDAF HW-protection */
#define B_PORT_DRV1_PROT_UV_VDDAF_PM            (1U << 8)                       /*!< RW: Protection mode: 0: Tri-state; 1: Switch LS */
#define B_PORT_DRV1_PROT_DIS_UV_VDDA            (1U << 7)                       /*!< RW: Disable Under Voltage VDDA HW-protection */
#define B_PORT_DRV1_PROT_UV_VDDA_PM             (1U << 6)                       /*!< RW: Protection mode: 0: Tri-state; 1: Switch LS */
#define B_PORT_DRV1_PROT_DIS_UV_VSM             (1U << 5)                       /*!< RW: Disable Under Voltage VSM HW-protection */
#define B_PORT_DRV1_PROT_UV_VSM_PM              (1U << 4)                       /*!< RW: Protection mode: 0: Tri-state; 1: Switch LS */
#define B_PORT_DRV1_PROT_DIS_OV_VSM             (1U << 3)                       /*!< RW: Disable Over Voltage VSM HW-protection */
#define B_PORT_DRV1_PROT_OV_VSM_PM              (1U << 2)                       /*!< RW: Protection mode: 0: Tri-state; 1: Switch LS */
#define B_PORT_DRV1_PROT_DIS_OVT                (1U << 1)                       /*!< RW: Disable Over Temperature HW-protection */
#define B_PORT_DRV1_PROT_OVT_PM                 (1U << 0)                       /*!< RW: Protection mode: 0: Tri-state; 1: Switch LS */

/* ********************* */
/* Block: PORT_DRV2_PROT */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV2_PROT __attribute__((nodp, addr(0x00250)));  /*!< IO_PORT_DRV2_PROT */
#define B_PORT_DRV2_PROT_DIS_DRV                (1U << 0)                       /*!< RW: Disable drivers */

/* ******************* */
/* Block: PORT_DIAG_IN */
/* ******************* */
extern volatile uint16_t IO_PORT_DIAG_IN __attribute__((nodp, addr(0x00252)));  /*!< IO_PORT_DIAG_IN */
#define B_PORT_DIAG_IN_OC_VDDA_MEM              (1U << 8)                       /*!< R: Cleared if dis_oc_vdda */
#define B_PORT_DIAG_IN_OV_VDDA_MEM              (1U << 7)                       /*!< R: Cleared if dis_ov_vdda */
#define M_PORT_DIAG_IN_OVC_MEM                  (3U << 5)                       /*!< R: Cleared if dis_oc */
#define B_PORT_DIAG_IN_UV_VDDAF_MEM             (1U << 4)                       /*!< R: Cleared if dis_uv_vddaf */
#define B_PORT_DIAG_IN_UV_VDDA_MEM              (1U << 3)                       /*!< R: Cleared if dis_uv_vdda */
#define B_PORT_DIAG_IN_UV_VSM_MEM               (1U << 2)                       /*!< R: Cleared if dis_uv_vsm */
#define B_PORT_DIAG_IN_OV_VSM_MEM               (1U << 1)                      /*!< R: Cleared if dis_ov_vsm */
#define B_PORT_DIAG_IN_OVT_MEM                  (1U << 0)                       /*!< R: Cleared if dis_ovt */

/* ******************** */
/* Block: PORT_I2C_CONF */
/* ******************** */
extern volatile uint16_t IO_PORT_I2C_CONF __attribute__((nodp, addr(0x00254)));  /*!< IO_PORT_I2C_CONF */
#define M_PORT_I2C_CONF_I2C_SCL_CLK_SEL         (15U << 9)                      /*!< RW: Value for analogue */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_IO1     (0U << 9)                       /*!< RW: IO selection IO1 for I2C_SCL_CLK */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_IO2     (1U << 9)                       /*!< RW: IO selection IO2 for I2C_SCL_CLK */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_IO3     (2U << 9)                       /*!< RW: IO selection IO3 for I2C_SCL_CLK */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_LIN     (3U << 9)                       /*!< RW: IO selection LIN for I2C_SCL_CLK */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_IO4     (4U << 9)                       /*!< RW: IO selection IO4 for I2C_SCL_CLK */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_IO5     (5U << 9)                       /*!< RW: IO selection IO5 for I2C_SCL_CLK */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_IO6     (6U << 9)                       /*!< RW: IO selection IO6 for I2C_SCL_CLK */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_IO7     (7U << 9)                       /*!< RW: IO selection IO7 for I2C_SCL_CLK */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_IO8     (8U << 9)                       /*!< RW: IO selection IO8 for I2C_SCL_CLK */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_IO9     (9U << 9)                       /*!< RW: IO selection IO9 for I2C_SCL_CLK */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_IO10    (10U << 9)                      /*!< RW: IO selection IO10 for I2C_SCL_CLK */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_IO11    (11U << 9)                      /*!< RW: IO selection IO11 for I2C_SCL_CLK */
#define B_PORT_I2C_CONF_SDA_FILT_ENABLE         (1U << 8)                       /*!< RW: Value for analogue */
#define B_PORT_I2C_CONF_I2C_ADDR_VALID          (1U << 7)                       /*!< RW: enable I2C */
#define M_PORT_I2C_CONF_I2C_ADDR                (127U << 0)                     /*!< RW: 7-bit address for I2C */

/* ************************** */
/* Block: PORT_I2C_DMA_OFFSET */
/* ************************** */
extern volatile uint16_t IO_PORT_I2C_DMA_OFFSET __attribute__((nodp, addr(0x00256)));  /*!< IO_PORT_I2C_DMA_OFFSET */
#define M_PORT_I2C_DMA_OFFSET_I2C_DMA_OFFSET    (65535U << 0)                   /*!< RW: BYTE address defines the start of DMA accesses (so for beginning of RAM it is 0x1000). LSB is ignored to align on word for DMA */

/* *************************** */
/* Block: PORT_I2C_READ_OFFSET */
/* *************************** */
extern volatile uint16_t IO_PORT_I2C_READ_OFFSET __attribute__((nodp, addr(0x00258)));  /*!< IO_PORT_I2C_READ_OFFSET */
#define M_PORT_I2C_READ_OFFSET_I2C_READ_OFFSET  (255U << 0)                     /*!< RW: WORD address for "direct read" startup offset. Up to 255 */

/* ********************* */
/* Block: PORT_MISC_TEST */
/* ********************* */
extern volatile uint16_t IO_PORT_MISC_TEST __attribute__((nodp, addr(0x0025A)));  /*!< IO_PORT_MISC_TEST */
#define B_PORT_MISC_TEST_TST_CS2_DAC            (1U << 13)                      /*!< RW: switch the current sensor DAC to TA0 */
#define B_PORT_MISC_TEST_TST_CS1_DAC            (1U << 12)                      /*!< RW: switch the current sensor DAC to TA0 */
#define B_PORT_MISC_TEST_TST_SHOVE_IO           (1U << 11)                      /*!< RW: active VDDA shove test mode for Ios */
#define B_PORT_MISC_TEST_SWI_1K                 (1U << 10)                      /*!< RW: connect 1k resistor for flash current measurements */
#define B_PORT_MISC_TEST_SWI_5K                 (1U << 9)                       /*!< RW: connect 5k resistor for flash current measurements */
#define B_PORT_MISC_TEST_TST_VDDD_IO4           (1U << 8)                       /*!< RW: connection for external VDDD supply through IO4 */
#define B_PORT_MISC_TEST_TST_TA0_BUF_ENABLE     (1U << 7)                       /*!< RW: activate the buffer for test bus TA0 */
#define M_PORT_MISC_TEST_TST_TA0_IO             (3U << 5)                       /*!< RW: enable the IO test mode for IO4:3 */
#define B_PORT_MISC_TEST_TEST_SCREEN            (1U << 4)                       /*!< RW: Value for analogue */
#define B_PORT_MISC_TEST_TST_VDDA_VT_MON_SUP    (1U << 3)                       /*!< RW: Value for analogue */
#define B_PORT_MISC_TEST_TST_RESREF_LV0_OTD     (1U << 2)                       /*!< RW: Value for analogue */
#define B_PORT_MISC_TEST_TST_DISC_DIO_OTD       (1U << 1)                       /*!< RW: Value for analogue */
#define B_PORT_MISC_TEST_TST_CMP_LV0_OTD        (1U << 0)                       /*!< RW: Value for analogue */

/* ********************** */
/* Block: PORT_MISC2_TEST */
/* ********************** */
extern volatile uint16_t IO_PORT_MISC2_TEST __attribute__((nodp, addr(0x0025C)));  /*!< IO_PORT_MISC2_TEST */
#define B_PORT_MISC2_TEST_ADC_BLOCK_TOGGLE      (1U << 7)                       /*!< RW: added for 81344BA ADC toggle test */
#define B_PORT_MISC2_TEST_VDDD_VT_MONITOR_TEST  (1U << 6)                       /*!< RW: added for 81340 */
#define B_PORT_MISC2_TEST_VDDD_VT_MONITOR_NOTEST (1U << 5)                      /*!< RW: added for 81340 */
#define M_PORT_MISC2_TEST_TA_CTRL_ADC           (7U << 2)                       /*!< RW: ADC related test multiplexer control signals */
#define M_PORT_MISC2_TEST_TST_ADC_IN            (3U << 0)                       /*!< RW: ADC mux test connect ADC_P/ADC_N input to test buses TA0/TA1 */

/* ************************* */
/* Block: PORT_PPM_RBASE_ADD */
/* ************************* */
extern volatile uint16_t IO_PORT_PPM_RBASE_ADD __attribute__((nodp, addr(0x0025E)));  /*!< IO_PORT_PPM_RBASE_ADD */
#define M_PORT_PPM_RBASE_ADD_PPM_RBASE_ADD      (65535U << 0)                   /*!< RW: Base address of the PPM DMA message in RX mode */

/* ************************* */
/* Block: PORT_PPM_TBASE_ADD */
/* ************************* */
extern volatile uint16_t IO_PORT_PPM_TBASE_ADD __attribute__((nodp, addr(0x00260)));  /*!< IO_PORT_PPM_TBASE_ADD */
#define M_PORT_PPM_TBASE_ADD_PPM_TBASE_ADD      (65535U << 0)                   /*!< RW: Base address of the PPM DMA message in TX mode */

/* *********************** */
/* Block: PORT_PPM_TIMEOUT */
/* *********************** */
extern volatile uint16_t IO_PORT_PPM_TIMEOUT __attribute__((nodp, addr(0x00262)));  /*!< IO_PORT_PPM_TIMEOUT */
#define M_PORT_PPM_TIMEOUT_PPM_TIMEOUT          (65535U << 0)                   /*!< RW: Timeout value */

/* ******************** */
/* Block: PORT_PPM_CTRL */
/* ******************** */
extern volatile uint16_t IO_PORT_PPM_CTRL __attribute__((nodp, addr(0x00264)));  /*!< IO_PORT_PPM_CTRL */
#define M_PORT_PPM_CTRL_PPM_IN_SEL              (15U << 11)                     /*!< RW: IO or LIN_XRX for PPM input */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_LIN_XRX      (0U << 11)                      /*!< RW: IO selection LIN XRX for PPM */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO0          (1U << 11)                      /*!< RW: IO selection IO0 for PPM */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO1          (2U << 11)                      /*!< RW: IO selection IO1 for PPM */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO2          (3U << 11)                      /*!< RW: IO selection IO2 for PPM */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO3          (4U << 11)                      /*!< RW: IO selection IO3 for PPM */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO4          (5U << 11)                      /*!< RW: IO selection IO4 for PPM */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO5          (6U << 11)                      /*!< RW: IO selection IO5 for PPM */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO6          (7U << 11)                      /*!< RW: IO selection IO6 for PPM */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO7          (8U << 11)                      /*!< RW: IO selection IO7 for PPM */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO8          (9U << 11)                      /*!< RW: IO selection IO8 for PPM */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO9          (10U << 11)                     /*!< RW: IO selection IO9 for PPM */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO10         (11U << 11)                     /*!< RW: IO selection IO10 for PPM */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO11         (12U << 11)                     /*!< RW: IO selection IO11 for PPM */
#define B_PORT_PPM_CTRL_PPM_TX_RX               (1U << 10)                      /*!< RW: 0 : RX mode. 1 : TX mode */
#define M_PORT_PPM_CTRL_PPM_DAT_IT_NB           (3U << 8)                       /*!< RW: program ppm_dat_it to trigger every 2^DAT_IT_NB PPM data saved in RAM */
#define B_PORT_PPM_CTRL_PPM_MODE                (1U << 7)                       /*!< RW: 0: TOTAL times only are stored in RAM. 1 : both ON and TOTAL */
#define B_PORT_PPM_CTRL_PPM_FILT_BYP            (1U << 6)                       /*!< RW: Input filter bypass */
#define M_PORT_PPM_CTRL_PPM_DMA_LEN             (15U << 2)                      /*!< RW: DMA buffer length : DMA_LEN * 8 words */
#define B_PORT_PPM_CTRL_PPM_RE_FE               (1U << 1)                       /*!< RW: 1 reference on rising edge, 0 on falling edge */
#define B_PORT_PPM_CTRL_PPM_EN                  (1U << 0)                       /*!< RW: Enable PPM timer */

/* ************************ */
/* Block: PORT_PPM_BUF_DATA */
/* ************************ */
extern volatile uint16_t IO_PORT_PPM_BUF_DATA __attribute__((nodp, addr(0x00266)));  /*!< IO_PORT_PPM_BUF_DATA */
#define M_PORT_PPM_BUF_DATA_PPM_BUF_DATA        (65535U << 0)                   /*!< R: Value for digital read */

/* **************** */
/* Block: PPM_TIMER */
/* **************** */
extern volatile uint16_t IO_PPM_TIMER __attribute__((nodp, addr(0x00268)));     /*!< IO_PPM_TIMER */
#define M_PPM_TIMER_PPM_ON_TIME                 (65535U << 0)                   /*!< RW: PPM On Time for TX mode */

/* ********************* */
/* Block: PORT_PPM_FLAGS */
/* ********************* */
extern volatile uint16_t IO_PORT_PPM_FLAGS __attribute__((nodp, addr(0x0026A)));  /*!< IO_PORT_PPM_FLAGS */
#define B_PORT_PPM_FLAGS_PPM_DMA_ERR            (1U << 3)                       /*!< R: Try to access non-existing area or non-writable area */
#define B_PORT_PPM_FLAGS_PPM_TOUT               (1U << 2)                       /*!< R: Time-out is reached before next edge */
#define B_PORT_PPM_FLAGS_PPM_RCVF               (1U << 1)                       /*!< R: RX: receive buffer overflow/TX: end of transmission */
#define B_PORT_PPM_FLAGS_PPM_DMA_OP             (1U << 0)                       /*!< R: DMA access (R/W) on going */

/* ********************** */
/* Block: PORT_SSCM2_CONF */
/* ********************** */
extern volatile uint16_t IO_PORT_SSCM2_CONF __attribute__((nodp, addr(0x0026C)));  /*!< IO_PORT_SSCM2_CONF */
#define B_PORT_SSCM2_CONF_SSCM2_CENTERED        (1U << 2)                       /*!< RW: Value for analogue */
#define B_PORT_SSCM2_CONF_SSCM2_SINGLEBIT       (1U << 1)                       /*!< RW: output shall provide a code with hamming distance of 1 instead of triangle */
#define B_PORT_SSCM2_CONF_SSCM2_EN              (1U << 0)                       /*!< RW: enable the spread spectrum modulation */

/* ********************** */
/* Block: PORT_STEP2_CONF */
/* ********************** */
extern volatile uint16_t IO_PORT_STEP2_CONF __attribute__((nodp, addr(0x0026E)));  /*!< IO_PORT_STEP2_CONF */
#define M_PORT_STEP2_CONF_STEP2_CNT             (255U << 8)                      /*!< RW: step count per period of triangular modulation for Spread Spectrum */
#define M_PORT_STEP2_CONF_STEP2_DUR             (15U << 4)                       /*!< RW: step duration in main clock pulses for Spread Spectrum */
#define M_PORT_STEP2_CONF_STEP2_INC             (15U << 0)                       /*!< RW: step increment for Spread Spectrum */

/* ********************** */
/* Block: PORT_TX_TIMEOUT */
/* ********************** */
extern volatile uint16_t IO_PORT_TX_TIMEOUT __attribute__((nodp, addr(0x00270)));  /*!< IO_PORT_TX_TIMEOUT */
#define B_PORT_TX_TIMEOUT_TX_TIMEOUT            (1U << 0)                       /*!< R: Value for digital read */

/* ********************* */
/* Block: PORT_UDMA0_RDA */
/* ********************* */
extern volatile uint16_t IO_PORT_UDMA0_RDA __attribute__((nodp, addr(0x00272)));  /*!< IO_PORT_UDMA0_RDA */
#define M_PORT_UDMA0_RDA_UDMA0_RDA              (65535U << 0)                   /*!< RW: Receive Buffer A address */

/* ********************* */
/* Block: PORT_UDMA0_RDB */
/* ********************* */
extern volatile uint16_t IO_PORT_UDMA0_RDB __attribute__((nodp, addr(0x00274)));  /*!< IO_PORT_UDMA0_RDB */
#define M_PORT_UDMA0_RDB_UDMA0_RDB              (65535U << 0)                   /*!< RW: Receive Buffer B address */

/* ******************** */
/* Block: PORT_UDMA0_TX */
/* ******************** */
extern volatile uint16_t IO_PORT_UDMA0_TX __attribute__((nodp, addr(0x00276)));  /*!< IO_PORT_UDMA0_TX */
#define M_PORT_UDMA0_TX_UDMA0_TX                (65535U << 0)                   /*!< RW: Transmit Buffer address */

/* ********************** */
/* Block: PORT_UDMA0_SIZE */
/* ********************** */
extern volatile uint16_t IO_PORT_UDMA0_SIZE __attribute__((nodp, addr(0x00278)));  /*!< IO_PORT_UDMA0_SIZE */
#define M_PORT_UDMA0_SIZE_UDMA0_SIZTX           (255U << 8)                     /*!< RW: Length transmit buffer in Bytes */
#define M_PORT_UDMA0_SIZE_UDMA0_SIZRX           (255U << 0)                     /*!< RW: length of receive buffer 'A' and 'B' in Bytes */

/* ********************** */
/* Block: PORT_UDMA0_CTRL */
/* ********************** */
extern volatile uint16_t IO_PORT_UDMA0_CTRL __attribute__((nodp, addr(0x0027A)));  /*!< IO_PORT_UDMA0_CTRL */
#define B_PORT_UDMA0_CTRL_UART0_STOP_MODE       (1U << 4)                       /*!< RW: Value for analogue */
#define B_PORT_UDMA0_CTRL_UDMA0_LOWBYTE_FIRST   (1U << 3)                       /*!< RW: Data is sent/received high byte first (0), or low byte first (1) */
#define B_PORT_UDMA0_CTRL_UDMA0_LSB_FIRST       (1U << 2)                       /*!< RW: Data is sent/received most significant bit (MSB) first (0) or least significant bit (LSB) first (1) */
#define B_PORT_UDMA0_CTRL_UDMA0_TXSTART         (1U << 1)                       /*!< RW: A rising edge will begin the transmission of current buffer; The bit needed to be cleared by software prior to the next frame transfer. Clearing the bit during transmission will abort after the current byte has been sent. */
#define B_PORT_UDMA0_CTRL_UDMA0_EN              (1U << 0)                       /*!< RW: Block is disabled (0) or enabled (1) */

/* ************ */
/* Block: UART0 */
/* ************ */
extern volatile uint16_t IO_UART0_BRRD __attribute__((nodp, addr(0x0027C)));    /*!< IO_UART0_BRRD */
#define M_UART0_BRRD                            (65535U << 0)                   /*!< RW: Baud rate register data */

extern volatile uint16_t IO_UART0_TRD __attribute__((nodp, addr(0x0027E)));     /*!< IO_UART0_TRD */
#define M_UART0_TRD                             (65535U << 0)                   /*!< RW: Transmit register data */

extern volatile uint16_t IO_UART0_RRD __attribute__((nodp, addr(0x00280)));     /*!< IO_UART0_RRD */
#define M_UART0_RRD                             (65535U << 0)                   /*!< R: Receive register data */

extern volatile uint16_t IO_UART0_LF __attribute__((nodp, addr(0x00282)));      /*!< IO_UART0_LF */
#define M_UART0_LFDD                            (255U << 8)                     /*!< RW: LIN frame duration data */
#define M_UART0_LFCD                            (255U << 0)                     /*!< R: LIN frame bit counter data */

extern volatile uint16_t IO_UART0_TRE __attribute__((nodp, addr(0x00284)));     /*!< IO_UART0_TRE */
#define B_UART0_LDC                             (1U << 15)                      /*!< RW: LIN duration control */
#define B_UART0_LSC                             (1U << 14)                      /*!< RW: LIN start bit control Double buffer mechanism */
#define B_UART0_LBC                             (1U << 13)                      /*!< RW: LIN bit control Double buffer mechanism */
#define M_UART0_BSC                             (3U << 11)                      /*!< RW: Bit scrambler control */
#define C_UART0_BSC_00                          (0U << 11)                      /*!<     Transmit default setting is NRZ, LSBit first */
#define C_UART0_BSC_01                          (1U << 11)                      /*!<     Transmit default setting is NRZ, MSBit first */
#define M_UART0_MLS                             (7U << 8)                       /*!< RW: Message length selector Double buffer mechanism */
#define C_UART0_MLS_8N1                         (1U << 8)                       /*!<     Message length for 8N1 format (8+2 bits) */
#define B_UART0_LTE                             (1U << 7)                       /*!< R: LIN time out error Set to 0 if TRE is asserted low Read and clear bit */
#define B_UART0_LSE                             (1U << 6)                       /*!< R: LIN start error Set to 0 if TRE is asserted low Read and clear bit */
#define B_UART0_LBE                             (1U << 5)                       /*!< R: LIN bit error Set to 0 if TRE is asserted low Read and clear bit */
#define B_UART0_ISB                             (1U << 3)                       /*!< RW: Transmit IDLE state */
#define B_UART0_REE                             (1U << 2)                       /*!< RW: Receiver enable */
#define B_UART0_TRE                             (1U << 1)                       /*!< RW: Transmitter enable */

extern volatile uint16_t IO_UART0_STS __attribute__((nodp, addr(0x00286)));     /*!< IO_UART0_STS */
#define B_UART0_SBE                             (1U << 15)                      /*!< R: Stop bit error Set to 0 if REE is asserted low Read and clear bit */
#define B_UART0_NBR                             (1U << 14)                      /*!< R: Noisy bit reception Set to 0 if REE is asserted low Read and clear bit */
#define B_UART0_RRF                             (1U << 13)                      /*!< R: Receive register full Set to 0 if REE is asserted low */
#define B_UART0_RSB                             (1U << 12)                      /*!< R: Receive shifter busy Set to 0 if REE is asserted low */
#define B_UART0_RSO                             (1U << 11)                      /*!< R: Receive shifter overrun Set to 0 if REE is asserted low Read and clear bit */
#define B_UART0_TSB                             (1U << 10)                      /*!< R: Transmit shifter busyIn LIN mode, bit set to 0 if LSE = 1 or LBE = 1Set to 0 if TRE is asserted low */
#define B_UART0_TRB                             (1U << 9)                       /*!< R: Transmit register busy In LIN mode, bit set to 0 if read UCTRL Set to 0 if TRE is asserted low */
#define B_UART0_TRO                             (1U << 8)                       /*!< R: Transmit register overrun In LIN mode, bit set to 0 if read UCTRL Set to 0 if TRE is asserted low Read and clear bit */

/* ************************ */
/* Block: PORT_UDMA0_STATUS */
/* ************************ */
extern volatile uint16_t IO_PORT_UDMA0_STATUS __attribute__((nodp, addr(0x00288)));  /*!< IO_PORT_UDMA0_STATUS */
#define B_PORT_UDMA0_STATUS_CUSTOM_UART0_DMA_ERR (1U << 3)                      /*!< R: DMA error occurred during RAM buffer access */
#define B_PORT_UDMA0_STATUS_UDMA0_FTR           (1U << 2)                       /*!< R: Value for digital read */
#define B_PORT_UDMA0_STATUS_UDMA0_FRC           (1U << 1)                       /*!< R: Value for digital read */
#define B_PORT_UDMA0_STATUS_UDMA0_RD_BUFFER_VALID (1U << 0)                     /*!< R: Read buffer 'A' is valid (1) or Read buffer 'B' is valid (0) */

/* ********************* */
/* Block: PORT_UDMA1_RDA */
/* ********************* */
extern volatile uint16_t IO_PORT_UDMA1_RDA __attribute__((nodp, addr(0x0028A)));  /*!< IO_PORT_UDMA1_RDA */
#define M_PORT_UDMA1_RDA_UDMA1_RDA              (65535U << 0)                   /*!< RW: Receive Buffer A address */

/* ********************* */
/* Block: PORT_UDMA1_RDB */
/* ********************* */
extern volatile uint16_t IO_PORT_UDMA1_RDB __attribute__((nodp, addr(0x0028C)));  /*!< IO_PORT_UDMA1_RDB */
#define M_PORT_UDMA1_RDB_UDMA1_RDB              (65535U << 0)                   /*!< RW: Receive Buffer B address */

/* ******************** */
/* Block: PORT_UDMA1_TX */
/* ******************** */
extern volatile uint16_t IO_PORT_UDMA1_TX __attribute__((nodp, addr(0x0028E)));  /*!< IO_PORT_UDMA1_TX */
#define M_PORT_UDMA1_TX_UDMA1_TX                (65535U << 0)                   /*!< RW: Transmit Buffer address */

/* ********************** */
/* Block: PORT_UDMA1_SIZE */
/* ********************** */
extern volatile uint16_t IO_PORT_UDMA1_SIZE __attribute__((nodp, addr(0x00290)));  /*!< IO_PORT_UDMA1_SIZE */
#define M_PORT_UDMA1_SIZE_UDMA1_SIZTX           (255U << 8)                     /*!< RW: Length transmit buffer in Bytes */
#define M_PORT_UDMA1_SIZE_UDMA1_SIZRX           (255U << 0)                     /*!< RW: length of receive buffer 'A' and 'B' in Bytes */

/* ********************** */
/* Block: PORT_UDMA1_CTRL */
/* ********************** */
extern volatile uint16_t IO_PORT_UDMA1_CTRL __attribute__((nodp, addr(0x00292)));  /*!< IO_PORT_UDMA1_CTRL */
#define B_PORT_UDMA1_CTRL_UART1_STOP_MODE       (1U << 4)                       /*!< RW: Value for analogue */
#define B_PORT_UDMA1_CTRL_UDMA1_LOWBYTE_FIRST   (1U << 3)                       /*!< RW: Data is sent/received high byte first (0), or low byte first (1) */
#define B_PORT_UDMA1_CTRL_UDMA1_LSB_FIRST       (1U << 2)                       /*!< RW: Data is sent/received most significant bit (MSB) first (0) or least significant bit (LSB) first (1) */
#define B_PORT_UDMA1_CTRL_UDMA1_TXSTART         (1U << 1)                       /*!< RW: A rising edge will begin the transmission of current buffer; The bit needed to be cleared by software prior to the next frame transfer. Clearing the bit during transmission will abort after the current byte has been sent. */
#define B_PORT_UDMA1_CTRL_UDMA1_EN              (1U << 0)                       /*!< RW: Block is disabled (0) or enabled (1) */

/* ************ */
/* Block: UART1 */
/* ************ */
extern volatile uint16_t IO_UART1_BRRD __attribute__((nodp, addr(0x00294)));    /*!< IO_UART1_BRRD */
#define M_UART1_BRRD                            (65535U << 0)                   /*!< RW: Baud rate register data */

extern volatile uint16_t IO_UART1_TRD __attribute__((nodp, addr(0x00296)));     /*!< IO_UART1_TRD */
#define M_UART1_TRD                             (65535U << 0)                   /*!< RW: Transmit register data */

extern volatile uint16_t IO_UART1_RRD __attribute__((nodp, addr(0x00298)));     /*!< IO_UART1_RRD */
#define M_UART1_RRD                             (65535U << 0)                   /*!< R: Receive register data */

extern volatile uint16_t IO_UART1_LF __attribute__((nodp, addr(0x0029A)));      /*!< IO_UART1_LF */
#define M_UART1_LFDD                            (255U << 8)                     /*!< RW: LIN frame duration data */
#define M_UART1_LFCD                            (255U << 0)                     /*!< R: LIN frame bit counter data */

extern volatile uint16_t IO_UART1_TRE __attribute__((nodp, addr(0x0029C)));     /*!< IO_UART1_TRE */
#define B_UART1_LDC                             (1U << 15)                      /*!< RW: LIN duration control */
#define B_UART1_LSC                             (1U << 14)                      /*!< RW: LIN start bit control Double buffer mechanism */
#define B_UART1_LBC                             (1U << 13)                      /*!< RW: LIN bit control Double buffer mechanism */
#define M_UART1_BSC                             (3U << 11)                      /*!< RW: Bit scrambler control */
#define C_UART1_BSC_00                          (0U << 11)                      /*!<     Transmit default setting is NRZ, LSBit first */
#define C_UART1_BSC_01                          (1U << 11)                      /*!<     Transmit default setting is NRZ, MSBit first */
#define M_UART1_MLS                             (7U << 8)                       /*!< RW: Message length selector Double buffer mechanism */
#define C_UART1_MLS_8N1                         (1U << 8)                       /*!<     Message length for 8N1 format (8+2 bits) */
#define B_UART1_LTE                             (1U << 7)                       /*!< R: LIN time out error Set to 0 if TRE is asserted low Read and clear bit */
#define B_UART1_LSE                             (1U << 6)                       /*!< R: LIN start error Set to 0 if TRE is asserted low Read and clear bit */
#define B_UART1_LBE                             (1U << 5)                       /*!< R: LIN bit error Set to 0 if TRE is asserted low Read and clear bit */
#define B_UART1_ISB                             (1U << 3)                       /*!< RW: Transmit IDLE state */
#define B_UART1_REE                             (1U << 2)                       /*!< RW: Receiver enable */
#define B_UART1_TRE                             (1U << 1)                       /*!< RW: Transmitter enable */

extern volatile uint16_t IO_UART1_STS __attribute__((nodp, addr(0x0029E)));     /*!< IO_UART1_STS */
#define B_UART1_SBE                             (1U << 15)                      /*!< R: Stop bit error Set to 0 if REE is asserted low Read and clear bit */
#define B_UART1_NBR                             (1U << 14)                      /*!< R: Noisy bit reception Set to 0 if REE is asserted low Read and clear bit */
#define B_UART1_RRF                             (1U << 13)                      /*!< R: Receive register full Set to 0 if REE is asserted low */
#define B_UART1_RSB                             (1U << 12)                      /*!< R: Receive shifter busy Set to 0 if REE is asserted low */
#define B_UART1_RSO                             (1U << 11)                      /*!< R: Receive shifter overrun Set to 0 if REE is asserted low Read and clear bit */
#define B_UART1_TSB                             (1U << 10)                      /*!< R: Transmit shifter busyIn LIN mode, bit set to 0 if LSE = 1 or LBE = 1Set to 0 if TRE is asserted low */
#define B_UART1_TRB                             (1U << 9)                       /*!< R: Transmit register busy In LIN mode, bit set to 0 if read UCTRL Set to 0 if TRE is asserted low */
#define B_UART1_TRO                             (1U << 8)                       /*!< R: Transmit register overrun In LIN mode, bit set to 0 if read UCTRL Set to 0 if TRE is asserted low Read and clear bit */

/* ************************ */
/* Block: PORT_UDMA1_STATUS */
/* ************************ */
extern volatile uint16_t IO_PORT_UDMA1_STATUS __attribute__((nodp, addr(0x002A0)));  /*!< IO_PORT_UDMA1_STATUS */
#define B_PORT_UDMA1_STATUS_CUSTOM_UART1_DMA_ERR (1U << 3)                      /*!< R: DMA error occurred during RAM buffer access */
#define B_PORT_UDMA1_STATUS_UDMA1_FTR           (1U << 2)                       /*!< R: Value for digital read */
#define B_PORT_UDMA1_STATUS_UDMA1_FRC           (1U << 1)                       /*!< R: Value for digital read */
#define B_PORT_UDMA1_STATUS_UDMA1_RD_BUFFER_VALID (1U << 0)                     /*!< R: Read buffer 'A' is valid (1) or Read buffer 'B' is valid (0) */

/* ****************************** */
/* Block: PORT_I2C_MST_ADD_BUFFER */
/* ****************************** */
extern volatile uint16_t IO_PORT_I2C_MST_ADD_BUFFER __attribute__((nodp, addr(0x002A2)));  /*!< IO_PORT_I2C_MST_ADD_BUFFER */
#define M_PORT_I2C_MST_ADD_BUFFER               (65535U << 0)                   /*!< Read/write buffer address for I2C master */

/* ************************ */
/* Block: PORT_I2C_MST_CTRL */
/* ************************ */
extern volatile uint16_t IO_PORT_I2C_MST_CTRL __attribute__((nodp, addr(0x002A4)));  /*!< IO_PORT_I2C_MST_CTRL */
#define B_PORT_I2C_MST_CTRL_I2C_MST_NO_STOP_BIT (1U << 1)                       /*!< RW: No Stop bit generated at the end (repeated Start for next transfer) */
#define B_PORT_I2C_MST_CTRL_I2C_MST_EN          (1U << 0)                       /*!< RW: Start I2C master transfer (shall remain 1 during transfer) */

/* *********************** */
/* Block: PORT_I2C_MST_PER */
/* *********************** */
extern volatile uint16_t IO_PORT_I2C_MST_PER __attribute__((nodp, addr(0x002A6)));  /*!< IO_PORT_I2C_MST_PER */
#define M_PORT_I2C_MST_PER_I2C_MST_SDA_DELAY    (255U << 8)                     /*!< RW: Delay for SDA vs SCL falling edge */
#define M_PORT_I2C_MST_PER_I2C_MST_HALF_PERIOD  (255U << 0)                     /*!< RW: Half period of I2C master transfer */

/* ************************ */
/* Block: PORT_I2C_MST_STAT */
/* ************************ */
extern volatile uint16_t IO_PORT_I2C_MST_STAT __attribute__((nodp, addr(0x002A8)));  /*!< IO_PORT_I2C_MST_STAT */
#define M_PORT_I2C_MST_STAT_I2C_MST_SIZE_CNT    (255U << 8)                     /*!< R: Used to find out where NAK occurs during byte transfer to Slave */
#define M_PORT_I2C_MST_STAT_I2C_MST_CURRENT_STATE (15U << 4)                    /*!< R: I2C master FSM state */
#define B_PORT_I2C_MST_STAT_I2C_MST_DMA_ERR     (1U << 3)                       /*!< R: DMA error occurred */
#define B_PORT_I2C_MST_STAT_DATA_ACK_NOK        (1U << 2)                       /*!< R: Slave gave NACK during byte transfer */
#define B_PORT_I2C_MST_STAT_ADDR_ACK_NOK        (1U << 1)                       /*!< R: Slave address not recognised on bus */
#define B_PORT_I2C_MST_STAT_I2C_MST_END         (1U << 0)                       /*!< R: End of transfer when set (reset when i2c_mst_en cleared) */

/* ********************** */
/* Block: PORT_CURR_SENS1 */
/* ********************** */
extern volatile uint16_t IO_PORT_CURR_SENS1 __attribute__((nodp, addr(0x002AA)));  /*!< IO_PORT_CURR_SENS1 */
#define B_PORT_CURR_SENS1_CSA1_BYPASS           (1U << 13)                      /*!< RW: Enable bypass mode (gain=1) */
#define B_PORT_CURR_SENS1_CSA1_CHOP             (1U << 12)                      /*!< RW: Enable chopper mode */
#define B_PORT_CURR_SENS1_CSA1_EN_OC            (1U << 11)                      /*!< RW: Enable Over-current comparator */
#define B_PORT_CURR_SENS1_CSA1_HIGHGAIN         (1U << 10)                      /*!< RW: Enable gain=20 of CSA, else gain=10 */
#define B_PORT_CURR_SENS1_CSA1_ZERO             (1U << 9)                       /*!< RW: Set the current sensor to 0 */
#define B_PORT_CURR_SENS1_EN_CSA1               (1U << 8)                       /*!< RW: Current sensor enable */
#define M_PORT_CURR_SENS1_SET_CS1               (255U << 0)                     /*!< RW: Set the level of the current sensor */

/* ********************** */
/* Block: PORT_CURR_SENS2 */
/* ********************** */
extern volatile uint16_t IO_PORT_CURR_SENS2 __attribute__((nodp, addr(0x002AC)));  /*!< IO_PORT_CURR_SENS2 */
#define B_PORT_CURR_SENS2_CSA2_BYPASS           (1U << 13)                      /*!< RW: Enable bypass mode (gain=1) */
#define B_PORT_CURR_SENS2_CSA2_CHOP             (1U << 12)                      /*!< RW: Enable chopper mode */
#define B_PORT_CURR_SENS2_CSA2_EN_OC            (1U << 11)                      /*!< RW: Enable Over-current comparator */
#define B_PORT_CURR_SENS2_CSA2_HIGHGAIN         (1U << 10)                      /*!< RW: Enable gain=20 of CSA, else gain=10 */
#define B_PORT_CURR_SENS2_CSA2_ZERO             (1U << 9)                       /*!< RW: Set the current sensor to 0 */
#define B_PORT_CURR_SENS2_EN_CSA2               (1U << 8)                       /*!< RW: Current sensor enable */
#define M_PORT_CURR_SENS2_SET_CS2               (255U << 0)                     /*!< RW: Set the level of the current sensor */

extern volatile uint16_t IO_PORT_PH_TRIG_EDGE_CFG __attribute__((nodp, addr(0x002AE)));  /*!< IO_PORT_PH_TRIG_EDGE_CFG */
#define M_PORT_PH_TRIG_EDGE_CFG_PH_IN5_SEL          (3U << 10)                  /*!< RW: Edge selection for PH_IN5 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN5_SEL_RAISING  (0U << 10)                  /*!< RW: Edge selection RAISING for PH_IN5 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN5_SEL_FALLING  (1U << 10)                  /*!< RW: Edge selection FALLING for PH_IN5 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN5_SEL_BOTH     (2U << 10)                  /*!< RW: Edge selection BOTH for PH_IN5 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN5_SEL_LEVEL    (3U << 10)                  /*!< RW: Edge selection LEVEL for PH_IN5 */
#define M_PORT_PH_TRIG_EDGE_CFG_PH_IN4_SEL          (3U << 8)                   /*!< RW: Edge selection for PH_IN4 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN4_SEL_RAISING  (0U << 8)                   /*!< RW: Edge selection RAISING for PH_IN4 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN4_SEL_FALLING  (1U << 8)                   /*!< RW: Edge selection FALLING for PH_IN4 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN4_SEL_BOTH     (2U << 8)                   /*!< RW: Edge selection BOTH for PH_IN4 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN4_SEL_LEVEL    (3U << 8)                   /*!< RW: Edge selection LEVEL for PH_IN4 */
#define M_PORT_PH_TRIG_EDGE_CFG_PH_IN3_SEL          (3U << 6)                   /*!< RW: Edge selection for PH_IN3 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN3_SEL_RAISING  (0U << 6)                   /*!< RW: Edge selection RAISING for PH_IN3 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN3_SEL_FALLING  (1U << 6)                   /*!< RW: Edge selection FALLING for PH_IN3 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN3_SEL_BOTH     (2U << 6)                   /*!< RW: Edge selection BOTH for PH_IN3 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN3_SEL_LEVEL    (3U << 6)                   /*!< RW: Edge selection LEVEL for PH_IN3 */
#define M_PORT_PH_TRIG_EDGE_CFG_PH_IN2_SEL          (3U << 4)                   /*!< RW: Edge selection for PH_IN2 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN2_SEL_RAISING  (0U << 4)                   /*!< RW: Edge selection RAISING for PH_IN2 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN2_SEL_FALLING  (1U << 4)                   /*!< RW: Edge selection FALLING for PH_IN2 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN2_SEL_BOTH     (2U << 4)                   /*!< RW: Edge selection BOTH for PH_IN2 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN2_SEL_LEVEL    (3U << 4)                   /*!< RW: Edge selection LEVEL for PH_IN2 */
#define M_PORT_PH_TRIG_EDGE_CFG_PH_IN1_SEL          (3U << 2)                   /*!< RW: Edge selection for PH_IN1 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN1_SEL_RAISING  (0U << 2)                   /*!< RW: Edge selection RAISING for PH_IN1 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN1_SEL_FALLING  (1U << 2)                   /*!< RW: Edge selection FALLING for PH_IN1 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN1_SEL_BOTH     (2U << 2)                   /*!< RW: Edge selection BOTH for PH_IN1 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN1_SEL_LEVEL    (3U << 2)                   /*!< RW: Edge selection LEVEL for PH_IN1 */
#define M_PORT_PH_TRIG_EDGE_CFG_PH_IN0_SEL          (3U << 0)                   /*!< RW: Edge selection for PH_IN0 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN0_SEL_RAISING  (0U << 0)                   /*!< RW: Edge selection RAISING for PH_IN0 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN0_SEL_FALLING  (1U << 0)                   /*!< RW: Edge selection FALLING for PH_IN0 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN0_SEL_BOTH     (2U << 0)                   /*!< RW: Edge selection BOTH for PH_IN0 */
#define C_PORT_IO_TRIG_EDGE_CFG_PH_IN0_SEL_LEVEL    (3U << 0)                   /*!< RW: Edge selection LEVEL for PH_IN0 */

extern volatile uint16_t IO_PORT_PH_IN __attribute__((nodp, addr(0x002B0)));  /*!< IO_PORT_PH_IN */
#define B_PORT_PH_IN_PH_IN_SYNC_5               (1U << 5)                       /*!< R: From Schmitt trigger PH_IN5 (after resynchro) */
#define B_PORT_PH_IN_PH_IN_SYNC_4               (1U << 4)                       /*!< R: From Schmitt trigger PH_IN4 (after resynchro) */
#define B_PORT_PH_IN_PH_IN_SYNC_3               (1U << 3)                       /*!< R: From Schmitt trigger PH_IN3 (after resynchro) */
#define B_PORT_PH_IN_PH_IN_SYNC_2               (1U << 2)                       /*!< R: From Schmitt trigger PH_IN2 (after resynchro) */
#define B_PORT_PH_IN_PH_IN_SYNC_1               (1U << 1)                       /*!< R: From Schmitt trigger PH_IN1 (after resynchro) */
#define B_PORT_PH_IN_PH_IN_SYNC_0               (1U << 0)                       /*!< R: From Schmitt trigger PH_IN0 (after resynchro) */

#endif /* defined (__MLX81160__) */

/* EOF */
