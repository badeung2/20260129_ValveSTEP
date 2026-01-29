/*!*************************************************************************** *
 * \file        ioports_MLX81330.h
 * \brief       MLX81330 I/O-ports-map
 *
 * \note        project MLX81330
 *
 * \author      Marcel Braat
 *
 * \date        2017-08-15
 *
 * \version     1.1 - preliminary
 *              I/O Port map for MLX81330A and MLX81330B
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

#if defined (__MLX81330__)

#ifndef __MLX81330A01__
#ifndef __MLX81330B01__
#ifndef __MLX81330B02__
#error "Wrong ioports.h file"
#endif /* __MLX81330B02__ */
#endif /* __MLX81330B01__ */
#endif /* __MLX81330A01__ */

/*!*************************************************************************** */
/*                           DEFINITIONS                                       */
/* *************************************************************************** */
/*!*************** */
/* Block: RST_CTRL */
/* *************** */
extern volatile uint16_t IO_RST_CTRL_S __attribute__((io, addr(0x02)));         /*!< IO_RST_CTRL_S (System) */
#define B_RST_CTRL_HVDIG_OK                     (1U << 15)                      /*!< R: 1: High Voltage Digital Okay 0: High Voltage Digital not Okay; W: No effect */
#define B_RST_CTRL_HVDIG_USED                   (1U << 14)                      /*!< RW: 0: No high-voltage digital part used 1: High-voltage digital part used */
#define B_RST_CTRL_SOFT_RESET                   (1U << 13)                      /*!< RW: 0: No request to reset hardware 1: Request to reset hardware (automatically cleared) */
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
#define B_IWD_WIN_OPEN                          (1U << 15)                      /*!< R: Window state: 0: Window is closed 1: Window is opened; W: no effect */
#define B_IWD_ATT_INT                           (1U << 14)                      /*!< R: 1 = Attention interrupt has been generated 0 = Attention interrupt has not been generated; W: no effect */
#define B_IWD_WIN_DISABLE                       (1U << 13)                      /*!< W: 1 = Disable Window. Acknowledges can occur at any time 0 = No effect; R: 1: Windowing disabled 0: Windowing enabled */
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

/* ******************** */
/* Block: PORT_ADC_CTRL */
/* ******************** */
extern volatile uint16_t IO_PORT_ADC_CTRL_S __attribute__((io, addr(0x34)));    /*!< IO_PORT_ADC_CTRL_S (System) */
#define B_PORT_ADC_CTRL_ADC_EN                  (1U << 1)                       /*!< RW: ADC enable */
#define B_PORT_ADC_CTRL_ADC_INV_COMPOUT         (1U << 0)                       /*!< RW: Invert ADC comparator output in SAR shell */

/* ******************** */
/* Block: PORT_TEST_ADC */
/* ******************** */
extern volatile uint16_t IO_PORT_TEST_ADC __attribute__((io, addr(0x36)));      /*!< IO_PORT_TEST_ADC */
#define M_PORT_TEST_ADC_TEST_ADC_IN             (3U << 7)                       /*!< RW: connect the output of ADC mux to analogue test bus */
#define M_PORT_TEST_ADC_TEST_ADC_REF            (3U << 5)                       /*!< RW: connect the ADC reference to analogue test bus */
#define B_PORT_TEST_ADC_TEST_ADC_CURSRC         (1U << 4)                       /*!< RW: test mode for the ADC current source */
#define B_PORT_TEST_ADC_TEST_HDAC               (1U << 3)                       /*!< RW: enable test of high DAC string */
#define B_PORT_TEST_ADC_TEST_LDAC               (1U << 2)                       /*!< RW: enable test of low DAC string */
#define B_PORT_TEST_ADC_TEST_DAC_BUF            (1U << 1)                       /*!< RW: switch the DAC buffered output to analogue test bus TA1 (to drive) */
#define B_PORT_TEST_ADC_TEST_DAC                (1U << 0)                       /*!< RW: switch the DAC output to analogue test bus TA0 */

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
#define B_MLX16_ITC_PEND0_FL_SH_ECC             (1U << 4)                       /*!< 0x0050 */
#define B_MLX16_ITC_PEND0_EE_SH_ECC             (1U << 5)                       /*!< 0x0058 */
#define B_MLX16_ITC_PEND0_UV_VDDA               (1U << 6)                       /*!< 0x0060 */
#define B_MLX16_ITC_PEND0_UV_VS                 (1U << 7)                       /*!< 0x0068 */
#define B_MLX16_ITC_PEND0_UV_VDDAF              (1U << 8)                       /*!< 0x0070 */
#define B_MLX16_ITC_PEND0_ANA_PLL_ERR           (1U << 9)                       /*!< 0x0078 */
#define B_MLX16_ITC_PEND0_OVT                   (1U << 10)                      /*!< 0x0080 */
#define B_MLX16_ITC_PEND0_OVC                   (1U << 11)                      /*!< 0x0088 */
#define B_MLX16_ITC_PEND0_OV_HS_VDS0            (1U << 12)                      /*!< 0x0090 */
#define B_MLX16_ITC_PEND0_OV_HS_VDS1            (1U << 13)                      /*!< 0x0098 */
#define B_MLX16_ITC_PEND0_OV_HS_VDS2            (1U << 14)                      /*!< 0x00A0 */
#define B_MLX16_ITC_PEND0_OV_HS_VDS3            (1U << 15)                      /*!< 0x00A8 */

extern volatile uint16_t IO_MLX16_ITC_PEND1_S __attribute__((nodp, addr(0x00052)));  /*!< IO_MLX16_ITC_PEND1_S (System) */
#define M_MLX16_ITC_PEND1                       (65535U << 0)                   /*!< R: 1 If the interrupt[INT0_COUNT+31:INT0_COUNT+16] is pending; W: Write 1 to clear pending interrupt, 0 does nothing */
#define B_MLX16_ITC_PEND1_OV_LS_VDS0            (1U << 0)                       /*!< 0x00B0 */
#define B_MLX16_ITC_PEND1_OV_LS_VDS1            (1U << 1)                       /*!< 0x00B8 */
#define B_MLX16_ITC_PEND1_OV_LS_VDS2            (1U << 2)                       /*!< 0x00C0 */
#define B_MLX16_ITC_PEND1_OV_LS_VDS3            (1U << 3)                       /*!< 0x00C8 */
#define B_MLX16_ITC_PEND1_STIMER                (1U << 4)                       /*!< 0x00D0 */
#define B_MLX16_ITC_PEND1_CTIMER0_1             (1U << 5)                       /*!< 0x00D8 */
#define B_MLX16_ITC_PEND1_CTIMER0_2             (1U << 6)                       /*!< 0x00E0 */
#define B_MLX16_ITC_PEND1_CTIMER0_3             (1U << 7)                       /*!< 0x00E8 */
#define B_MLX16_ITC_PEND1_CTIMER1_1             (1U << 8)                       /*!< 0x00F0 */
#define B_MLX16_ITC_PEND1_CTIMER1_2             (1U << 9)                       /*!< 0x00F8 */
#define B_MLX16_ITC_PEND1_CTIMER1_3             (1U << 10)                      /*!< 0x0100 */
#define B_MLX16_ITC_PEND1_SPI_TE                (1U << 11)                      /*!< 0x0108 */
#define B_MLX16_ITC_PEND1_SPI_RF                (1U << 12)                      /*!< 0x0110 */
#define B_MLX16_ITC_PEND1_SPI_ER                (1U << 13)                      /*!< 0x0118 */
#define B_MLX16_ITC_PEND1_PWM_MASTER1_CMP       (1U << 14)                      /*!< 0x0120 */
#define B_MLX16_ITC_PEND1_PWM_MASTER1_END       (1U << 15)                      /*!< 0x0128 */

extern volatile uint16_t IO_MLX16_ITC_PEND2_S __attribute__((nodp, addr(0x00054)));  /*!< IO_MLX16_ITC_PEND2_S (System) */
#define M_MLX16_ITC_PEND2                       (65535U << 0)                   /*!< R: 1 If the interrupt[INT0_COUNT+47:INT0_COUNT+32] is pending; W: Write 1 to clear pending interrupt, 0 does nothing */
#define B_MLX16_ITC_PEND2_PWM_SLAVE1_CMP        (1U << 0)                       /*!< 0x0130 */
#define B_MLX16_ITC_PEND2_PWM_SLAVE2_CMP        (1U << 1)                       /*!< 0x0138 */
#define B_MLX16_ITC_PEND2_PWM_SLAVE3_CMP        (1U << 2)                       /*!< 0x0140 */
#define B_MLX16_ITC_PEND2_PWM_MASTER2_CMP       (1U << 3)                       /*!< 0x0148 */
#define B_MLX16_ITC_PEND2_PWM_MASTER2_END       (1U << 4)                       /*!< 0x0150 */
#define B_MLX16_ITC_PEND2_ADC_SAR               (1U << 5)                       /*!< 0x0158 */
#define B_MLX16_ITC_PEND2_EE_COMPLETE           (1U << 6)                       /*!< 0x0160 */
#define B_MLX16_ITC_PEND2_FL_COMPLETE           (1U << 7)                       /*!< 0x0168 */
#define B_MLX16_ITC_PEND2_COLIN_OWNMTX          (1U << 8)                       /*!< 0x0170 */
#define B_MLX16_ITC_PEND2_COLIN_LIN             (1U << 9)                       /*!< 0x0178 */
#define B_MLX16_ITC_PEND2_OV_VS                 (1U << 10)                      /*!< 0x0180 */
#define B_MLX16_ITC_PEND2_DIAG                  (1U << 11)                      /*!< 0x0188 */
#ifdef __MLX81330A01__
#define B_MLX16_ITC_PEND2_IO_IN0                (1U << 12)                      /*!< 0x0190 */
#define B_MLX16_ITC_PEND2_IO_IN1                (1U << 13)                      /*!< 0x0198 */
#define B_MLX16_ITC_PEND2_IO_IN2                (1U << 14)                      /*!< 0x01A0 */
#define B_MLX16_ITC_PEND2_IO_IN3                (1U << 15)                      /*!< 0x01A8 */
#else
#define B_MLX16_ITC_PEND2_I2C_GLOBAL_RESET      (1U << 12)                      /*!< 0x0190 */
#define B_MLX16_ITC_PEND2_PPM_RX                (1U << 13)                      /*!< 0x0198 */
#define B_MLX16_ITC_PEND2_PPM_TX                (1U << 14)                      /*!< 0x01A0 */
#define B_MLX16_ITC_PEND2_PPM_ERR               (1U << 15)                      /*!< 0x01A8 */
#endif

extern volatile uint16_t IO_MLX16_ITC_PEND3_S __attribute__((nodp, addr(0x00056)));  /*!< IO_MLX16_ITC_PEND3_S (System) */
#define M_MLX16_ITC_PEND3                       (31U << 0)                      /*!< R: 1 If the interrupt[INT0_COUNT+63:INT0_COUNT+48] is pending; W: Write 1 to clear pending interrupt, 0 does nothing */
#ifdef __MLX81330A01__
#define B_MLX16_ITC_PEND3_I2C_GLOBAL_RESET      (1U << 0)                       /*!< 0x01B0 */
#define B_MLX16_ITC_PEND3_PPM_RX                (1U << 1)                       /*!< 0x01B8 */
#define B_MLX16_ITC_PEND3_PPM_TX                (1U << 2)                       /*!< 0x01C0 */
#define B_MLX16_ITC_PEND3_PPM_ERR               (1U << 3)                       /*!< 0x01C8 */
#define B_MLX16_ITC_PEND3_MLX16_SOFT            (1U << 4)                       /*!< 0x01D0 */
#else
#define B_MLX16_ITC_PEND3_IO_IN0                (1U << 0)                       /*!< 0x01B0 */
#define B_MLX16_ITC_PEND3_IO_IN1                (1U << 1)                       /*!< 0x01B8 */
#define B_MLX16_ITC_PEND3_IO_IN2                (1U << 2)                       /*!< 0x01C0 */
#define B_MLX16_ITC_PEND3_IO_IN3                (1U << 3)                       /*!< 0x01C8 */
#define B_MLX16_ITC_PEND3_TX_TIMEOUT            (1U << 4)                       /*!< 0x01D0 */
#define B_MLX16_ITC_PEND3_MLX16_SOFT            (1U << 5)                       /*!< 0x01D8 */
#endif

extern volatile uint16_t IO_MLX16_SWI_S __attribute__((nodp, addr(0x0006A)));   /*!< IO_MLX16_SWI_S (System) */
#define B_MLX16_SWI                             (1U << 15)                      /*!< W: Request a software interrupt; R: Always read 0 */

extern volatile uint16_t IO_MLX16_ITC_MASK0_S __attribute__((nodp, addr(0x0006C)));  /*!< IO_MLX16_ITC_MASK0_S (System) */
#define M_MLX16_ITC_MASK0                       (65535U << 0)                   /*!< RW: 1 to enable interrupt[INT0_COUNT+15:INT0_COUNT] */
#define B_MLX16_ITC_MASK0_MLX16_EXCHG           (1U << 0)                       /*!< 0x0030 */
#define B_MLX16_ITC_MASK0_MLX16_DMAERR          (1U << 1)                       /*!< 0x0038 */
#define B_MLX16_ITC_MASK0_AWD_ATT               (1U << 2)                       /*!< 0x0040 */
#define B_MLX16_ITC_MASK0_IWD_ATT               (1U << 3)                       /*!< 0x0048 */
#define B_MLX16_ITC_MASK0_FL_SH_ECC             (1U << 4)                       /*!< 0x0050 */
#define B_MLX16_ITC_MASK0_EE_SH_ECC             (1U << 5)                       /*!< 0x0058 */
#define B_MLX16_ITC_MASK0_UV_VDDA               (1U << 6)                       /*!< 0x0060 */
#define B_MLX16_ITC_MASK0_UV_VS                 (1U << 7)                       /*!< 0x0068 */
#define B_MLX16_ITC_MASK0_UV_VDDAF              (1U << 8)                       /*!< 0x0070 */
#define B_MLX16_ITC_MASK0_ANA_PLL_ERR           (1U << 9)                       /*!< 0x0078 */
#define B_MLX16_ITC_MASK0_OVT                   (1U << 10)                      /*!< 0x0080 */
#define B_MLX16_ITC_MASK0_OVC                   (1U << 11)                      /*!< 0x0088 */
#define B_MLX16_ITC_MASK0_OV_HS_VDS0            (1U << 12)                      /*!< 0x0090 */
#define B_MLX16_ITC_MASK0_OV_HS_VDS1            (1U << 13)                      /*!< 0x0098 */
#define B_MLX16_ITC_MASK0_OV_HS_VDS2            (1U << 14)                      /*!< 0x00A0 */
#define B_MLX16_ITC_MASK0_OV_HS_VDS3            (1U << 15)                      /*!< 0x00A8 */

extern volatile uint16_t IO_MLX16_ITC_MASK1_S __attribute__((nodp, addr(0x0006E)));  /*!< IO_MLX16_ITC_MASK1_S (System) */
#define M_MLX16_ITC_MASK1                       (65535U << 0)                   /*!< RW: 1 to enable interrupt[INT0_COUNT+31:INT0_COUNT+16] */
#define B_MLX16_ITC_MASK1_OV_LS_VDS0            (1U << 0)                       /*!< 0x00B0 */
#define B_MLX16_ITC_MASK1_OV_LS_VDS1            (1U << 1)                       /*!< 0x00B8 */
#define B_MLX16_ITC_MASK1_OV_LS_VDS2            (1U << 2)                       /*!< 0x00C0 */
#define B_MLX16_ITC_MASK1_OV_LS_VDS3            (1U << 3)                       /*!< 0x00C8 */
#define B_MLX16_ITC_MASK1_STIMER                (1U << 4)                       /*!< 0x00D0 */
#define B_MLX16_ITC_MASK1_CTIMER0_1             (1U << 5)                       /*!< 0x00D8 */
#define B_MLX16_ITC_MASK1_CTIMER0_2             (1U << 6)                       /*!< 0x00E0 */
#define B_MLX16_ITC_MASK1_CTIMER0_3             (1U << 7)                       /*!< 0x00E8 */
#define B_MLX16_ITC_MASK1_CTIMER1_1             (1U << 8)                       /*!< 0x00F0 */
#define B_MLX16_ITC_MASK1_CTIMER1_2             (1U << 9)                       /*!< 0x00F8 */
#define B_MLX16_ITC_MASK1_CTIMER1_3             (1U << 10)                      /*!< 0x0100 */
#define B_MLX16_ITC_MASK1_SPI_TE                (1U << 11)                      /*!< 0x0108 */
#define B_MLX16_ITC_MASK1_SPI_RF                (1U << 12)                      /*!< 0x0110 */
#define B_MLX16_ITC_MASK1_SPI_ER                (1U << 13)                      /*!< 0x0118 */
#define B_MLX16_ITC_MASK1_PWM_MASTER1_CMP       (1U << 14)                      /*!< 0x0120 */
#define B_MLX16_ITC_MASK1_PWM_MASTER1_END       (1U << 15)                      /*!< 0x0128 */

extern volatile uint16_t IO_MLX16_ITC_MASK2_S __attribute__((nodp, addr(0x00070)));  /*!< IO_MLX16_ITC_MASK2_S (System) */
#define M_MLX16_ITC_MASK2                       (65535U << 0)                   /*!< RW: 1 to enable interrupt[INT0_COUNT+47:INT0_COUNT+32] */
#define B_MLX16_ITC_MASK2_PWM_SLAVE1_CMP        (1U << 0)                       /*!< 0x0130 */
#define B_MLX16_ITC_MASK2_PWM_SLAVE2_CMP        (1U << 1)                       /*!< 0x0138 */
#define B_MLX16_ITC_MASK2_PWM_SLAVE3_CMP        (1U << 2)                       /*!< 0x0140 */
#define B_MLX16_ITC_MASK2_PWM_MASTER2_CMP       (1U << 3)                       /*!< 0x0148 */
#define B_MLX16_ITC_MASK2_PWM_MASTER2_END       (1U << 4)                       /*!< 0x0150 */
#define B_MLX16_ITC_MASK2_ADC_SAR               (1U << 5)                       /*!< 0x0158 */
#define B_MLX16_ITC_MASK2_EE_COMPLETE           (1U << 6)                       /*!< 0x0160 */
#define B_MLX16_ITC_MASK2_FL_COMPLETE           (1U << 7)                       /*!< 0x0168 */
#define B_MLX16_ITC_MASK2_COLIN_OWNMTX          (1U << 8)                       /*!< 0x0170 */
#define B_MLX16_ITC_MASK2_COLIN_LIN             (1U << 9)                       /*!< 0x0178 */
#define B_MLX16_ITC_MASK2_OV_VS                 (1U << 10)                      /*!< 0x0180 */
#define B_MLX16_ITC_MASK2_DIAG                  (1U << 11)                      /*!< 0x0188 */
#ifdef __MLX81330A01__
#define B_MLX16_ITC_MASK2_IO_IN0                (1U << 12)                      /*!< 0x0190 */
#define B_MLX16_ITC_MASK2_IO_IN1                (1U << 13)                      /*!< 0x0198 */
#define B_MLX16_ITC_MASK2_IO_IN2                (1U << 14)                      /*!< 0x01A0 */
#define B_MLX16_ITC_MASK2_IO_IN3                (1U << 15)                      /*!< 0x01A8 */
#else
#define B_MLX16_ITC_MASK2_I2C_GLOBAL_RESET      (1U << 12)                      /*!< 0x0190 */
#define B_MLX16_ITC_MASK2_PPM_RX                (1U << 13)                      /*!< 0x0198 */
#define B_MLX16_ITC_MASK2_PPM_TX                (1U << 14)                      /*!< 0x01A0 */
#define B_MLX16_ITC_MASK2_PPM_ERR               (1U << 15)                      /*!< 0x01A8 */
#endif

extern volatile uint16_t IO_MLX16_ITC_MASK3_S __attribute__((nodp, addr(0x00072)));  /*!< IO_MLX16_ITC_MASK3_S (System) */
#define M_MLX16_ITC_MASK3                       (31U << 0)                      /*!< RW: 1 to enable interrupt[INT0_COUNT+63:INT0_COUNT+48] */
#ifdef __MLX81330A01__
#define B_MLX16_ITC_MASK3_I2C_GLOBAL_RESET      (1U << 0)                       /*!< 0x01B0 */
#define B_MLX16_ITC_MASK3_PPM_RX                (1U << 1)                       /*!< 0x01B8 */
#define B_MLX16_ITC_MASK3_PPM_TX                (1U << 2)                       /*!< 0x01C0 */
#define B_MLX16_ITC_MASK3_PPM_ERR               (1U << 3)                       /*!< 0x01C8 */
#define B_MLX16_ITC_MASK3_MLX16_SOFT            (1U << 4)                       /*!< 0x01D0 */
#else
#define B_MLX16_ITC_MASK3_IO_IN0                (1U << 0)                       /*!< 0x01B0 */
#define B_MLX16_ITC_MASK3_IO_IN1                (1U << 1)                       /*!< 0x01B8 */
#define B_MLX16_ITC_MASK3_IO_IN2                (1U << 2)                       /*!< 0x01C0 */
#define B_MLX16_ITC_MASK3_IO_IN3                (1U << 3)                       /*!< 0x01C8 */
#define B_MLX16_ITC_MASK3_TX_TIMEOUT            (1U << 4)                       /*!< 0x01D0 */
#define B_MLX16_ITC_MASK3_MLX16_SOFT            (1U << 5)                       /*!< 0x01D8 */
#endif

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
#define M_MLX16_ITC_PRIO1_PWM_SLAVE3_CMP        (3U << 12)                      /*!< RW: priority PWM_SLAVE3_CMP */
#define C_MLX16_ITC_PRIO1_PWM_SLAVE3_CMP_PRIO3  (0U << 12)                      /*!< RW: priority 3 PWM_SLAVE3_CMP */
#define C_MLX16_ITC_PRIO1_PWM_SLAVE3_CMP_PRIO4  (1U << 12)                      /*!< RW: priority 4 PWM_SLAVE3_CMP */
#define C_MLX16_ITC_PRIO1_PWM_SLAVE3_CMP_PRIO5  (2U << 12)                      /*!< RW: priority 5 PWM_SLAVE3_CMP */
#define C_MLX16_ITC_PRIO1_PWM_SLAVE3_CMP_PRIO6  (3U << 12)                      /*!< RW: priority 6 PWM_SLAVE3_CMP */
#define M_MLX16_ITC_PRIO1_PWM_MASTER2_CMP       (3U << 14)                      /*!< RW: priority PWM_MASTER2_CMP */
#define C_MLX16_ITC_PRIO1_PWM_MASTER2_CMP_PRIO3 (0U << 14)                      /*!< RW: priority 3 PWM_MASTER2_CMP */
#define C_MLX16_ITC_PRIO1_PWM_MASTER2_CMP_PRIO4 (1U << 14)                      /*!< RW: priority 4 PWM_MASTER2_CMP */
#define C_MLX16_ITC_PRIO1_PWM_MASTER2_CMP_PRIO5 (2U << 14)                      /*!< RW: priority 5 PWM_MASTER2_CMP */
#define C_MLX16_ITC_PRIO1_PWM_MASTER2_CMP_PRIO6 (3U << 14)                      /*!< RW: priority 6 PWM_MASTER2_CMP */

extern volatile uint16_t IO_MLX16_ITC_PRIO2_S __attribute__((nodp, addr(0x0008C)));  /*!< IO_MLX16_ITC_PRIO2_S (System) */
#define M_MLX16_ITC_PRIO2                       (65535U << 0)                   /*!< RW: Configure priority for interrupt[INT0_COUNT+INT1_COUNT+INT2_COUNT+23:INT0_COUNT+INT1_COUNT+INT2_COUNT+16] */
#define M_MLX16_ITC_PRIO2_PWM_MASTER2_END       (3U << 0)                       /*!< RW: priority PWM_MASTER2_END */
#define C_MLX16_ITC_PRIO2_PWM_MASTER2_END_PRIO3 (0U << 0)                       /*!< RW: priority 3 PWM_MASTER2_END */
#define C_MLX16_ITC_PRIO2_PWM_MASTER2_END_PRIO4 (1U << 0)                       /*!< RW: priority 4 PWM_MASTER2_END */
#define C_MLX16_ITC_PRIO2_PWM_MASTER2_END_PRIO5 (2U << 0)                       /*!< RW: priority 5 PWM_MASTER2_END */
#define C_MLX16_ITC_PRIO2_PWM_MASTER2_END_PRIO6 (3U << 0)                       /*!< RW: priority 6 PWM_MASTER2_END */
#define M_MLX16_ITC_PRIO2_ADC_SAR               (3U << 2)                       /*!< RW: priority ADC_SAR */
#define C_MLX16_ITC_PRIO2_ADC_SAR_PRIO3         (0U << 2)                       /*!< RW: priority 3 ADC_SAR */
#define C_MLX16_ITC_PRIO2_ADC_SAR_PRIO4         (1U << 2)                       /*!< RW: priority 4 ADC_SAR */
#define C_MLX16_ITC_PRIO2_ADC_SAR_PRIO5         (2U << 2)                       /*!< RW: priority 5 ADC_SAR */
#define C_MLX16_ITC_PRIO2_ADC_SAR_PRIO6         (3U << 2)                       /*!< RW: priority 6 ADC_SAR */
#define M_MLX16_ITC_PRIO2_EE_COMPLETE           (3U << 4)                       /*!< RW: priority EE_COMPLETE */
#define C_MLX16_ITC_PRIO2_EE_COMPLETE_PRIO3     (0U << 4)                       /*!< RW: priority 3 EE_COMPLETE */
#define C_MLX16_ITC_PRIO2_EE_COMPLETE_PRIO4     (1U << 4)                       /*!< RW: priority 4 EE_COMPLETE */
#define C_MLX16_ITC_PRIO2_EE_COMPLETE_PRIO5     (2U << 4)                       /*!< RW: priority 5 EE_COMPLETE */
#define C_MLX16_ITC_PRIO2_EE_COMPLETE_PRIO6     (3U << 4)                       /*!< RW: priority 6 EE_COMPLETE */
#define M_MLX16_ITC_PRIO2_FL_COMPLETE           (3U << 6)                       /*!< RW: priority FL_COMPLETE */
#define C_MLX16_ITC_PRIO2_FL_COMPLETE_PRIO3     (0U << 6)                       /*!< RW: priority 3 FL_COMPLETE */
#define C_MLX16_ITC_PRIO2_FL_COMPLETE_PRIO4     (1U << 6)                       /*!< RW: priority 4 FL_COMPLETE */
#define C_MLX16_ITC_PRIO2_FL_COMPLETE_PRIO5     (2U << 6)                       /*!< RW: priority 5 FL_COMPLETE */
#define C_MLX16_ITC_PRIO2_FL_COMPLETE_PRIO6     (3U << 6)                       /*!< RW: priority 6 FL_COMPLETE */
#define M_MLX16_ITC_PRIO2_COLIN_OWNMTX          (3U << 8)                       /*!< RW: priority COLIN_OWNMTX */
#define C_MLX16_ITC_PRIO2_COLIN_OWNMTX_PRIO3    (0U << 8)                       /*!< RW: priority 3 COLIN_OWNMTX */
#define C_MLX16_ITC_PRIO2_COLIN_OWNMTX_PRIO4    (1U << 8)                       /*!< RW: priority 4 COLIN_OWNMTX */
#define C_MLX16_ITC_PRIO2_COLIN_OWNMTX_PRIO5    (2U << 8)                       /*!< RW: priority 5 COLIN_OWNMTX */
#define C_MLX16_ITC_PRIO2_COLIN_OWNMTX_PRIO6    (3U << 8)                       /*!< RW: priority 6 COLIN_OWNMTX */
#define M_MLX16_ITC_PRIO2_COLIN_LIN             (3U << 10)                      /*!< RW: priority COLIN_LINR */
#define C_MLX16_ITC_PRIO2_COLIN_LIN_PRIO3       (0U << 10)                      /*!< RW: priority 3 COLIN_LINR */
#define C_MLX16_ITC_PRIO2_COLIN_LIN_PRIO4       (1U << 10)                      /*!< RW: priority 4 COLIN_LINR */
#define C_MLX16_ITC_PRIO2_COLIN_LIN_PRIO5       (2U << 10)                      /*!< RW: priority 5 COLIN_LINR */
#define C_MLX16_ITC_PRIO2_COLIN_LIN_PRIO6       (3U << 10)                      /*!< RW: priority 6 COLIN_LINR */
#define M_MLX16_ITC_PRIO2_OV_VS                 (3U << 12)                      /*!< RW: priority OV_VS */
#define C_MLX16_ITC_PRIO2_OV_VS_PRIO3           (0U << 12)                      /*!< RW: priority 3 OV_VS */
#define C_MLX16_ITC_PRIO2_OV_VS_PRIO4           (1U << 12)                      /*!< RW: priority 4 OV_VS */
#define C_MLX16_ITC_PRIO2_OV_VS_PRIO5           (2U << 12)                      /*!< RW: priority 5 OV_VS */
#define C_MLX16_ITC_PRIO2_OV_VS_PRIO6           (3U << 12)                      /*!< RW: priority 6 OV_VS */
#define M_MLX16_ITC_PRIO2_DIAG                  (3U << 14)                      /*!< RW: priority DIAG */
#define C_MLX16_ITC_PRIO2_DIAG_PRIO3            (0U << 14)                      /*!< RW: priority 3 DIAG */
#define C_MLX16_ITC_PRIO2_DIAG_PRIO4            (1U << 14)                      /*!< RW: priority 4 DIAG */
#define C_MLX16_ITC_PRIO2_DIAG_PRIO5            (2U << 14)                      /*!< RW: priority 5 DIAG */
#define C_MLX16_ITC_PRIO2_DIAG_PRIO6            (3U << 14)                      /*!< RW: priority 6 DIAG */

extern volatile uint16_t IO_MLX16_ITC_PRIO3_S __attribute__((nodp, addr(0x0008E)));  /*!< IO_MLX16_ITC_PRIO3_S (System) */
#define M_MLX16_ITC_PRIO3                       (65535U << 0)                   /*!< RW: Configure priority for interrupt[INT0_COUNT+INT1_COUNT+INT2_COUNT+31:INT0_COUNT+INT1_COUNT+INT2_COUNT+24] */
#ifdef __MLX81330A01__
#define M_MLX16_ITC_PRIO3                       (65535U << 0)                   /*!< RW: Configure priority for interrupt[INT0_COUNT+INT1_COUNT+INT2_COUNT+31:INT0_COUNT+INT1_COUNT+INT2_COUNT+24] */
#define M_MLX16_ITC_PRIO3_IO_IN0                (3U << 0)                       /*!< RW: priority IO_IN0 */
#define C_MLX16_ITC_PRIO3_IO_IN0_PRIO3          (0U << 0)
#define C_MLX16_ITC_PRIO3_IO_IN0_PRIO4          (1U << 0)
#define C_MLX16_ITC_PRIO3_IO_IN0_PRIO5          (2U << 0)
#define C_MLX16_ITC_PRIO3_IO_IN0_PRIO6          (3U << 0)
#define M_MLX16_ITC_PRIO3_IO_IN1                (3U << 2)                       /*!< RW: priority IO_IN1 */
#define C_MLX16_ITC_PRIO3_IO_IN1_PRIO3          (0U << 2)
#define C_MLX16_ITC_PRIO3_IO_IN1_PRIO4          (1U << 2)
#define C_MLX16_ITC_PRIO3_IO_IN1_PRIO5          (2U << 2)
#define C_MLX16_ITC_PRIO3_IO_IN1_PRIO6          (3U << 2)
#define M_MLX16_ITC_PRIO3_IO_IN2                (3U << 4)                       /*!< RW: priority IO_IN2 */
#define C_MLX16_ITC_PRIO3_IO_IN2_PRIO3          (0U << 4)
#define C_MLX16_ITC_PRIO3_IO_IN2_PRIO4          (1U << 4)
#define C_MLX16_ITC_PRIO3_IO_IN2_PRIO5          (2U << 4)
#define C_MLX16_ITC_PRIO3_IO_IN2_PRIO6          (3U << 4)
#define M_MLX16_ITC_PRIO3_IO_IN3                (3U << 6)                       /*!< RW: priority IO_IN3 */
#define C_MLX16_ITC_PRIO3_IO_IN3_PRIO3          (0U << 6)
#define C_MLX16_ITC_PRIO3_IO_IN3_PRIO4          (1U << 6)
#define C_MLX16_ITC_PRIO3_IO_IN3_PRIO5          (2U << 6)
#define C_MLX16_ITC_PRIO3_IO_IN3_PRIO6          (3U << 6)
#define M_MLX16_ITC_PRIO3_I2C_GLOBAL_RESET      (3U << 8)                       /*!< RW: priority I2C_GLOBAL_RESET */
#define C_MLX16_ITC_PRIO3_I2C_GLOBAL_RESET_PRIO3 (0U << 8)
#define C_MLX16_ITC_PRIO3_I2C_GLOBAL_RESET_PRIO4 (1U << 8)
#define C_MLX16_ITC_PRIO3_I2C_GLOBAL_RESET_PRIO5 (2U << 8)
#define C_MLX16_ITC_PRIO3_I2C_GLOBAL_RESET_PRIO6 (3U << 8)
#define M_MLX16_ITC_PRIO3_PPM_RX                (3U << 10)                      /*!< RW: priority PPM_RX */
#define C_MLX16_ITC_PRIO3_PPM_RX_PRIO3          (0U << 10)
#define C_MLX16_ITC_PRIO3_PPM_RX_PRIO4          (1U << 10)
#define C_MLX16_ITC_PRIO3_PPM_RX_PRIO5          (2U << 10)
#define C_MLX16_ITC_PRIO3_PPM_RX_PRIO6          (3U << 10)
#define M_MLX16_ITC_PRIO3_PPM_TX                (3U << 12)                      /*!< RW: priority PPM_TX */
#define C_MLX16_ITC_PRIO3_PPM_TX_PRIO3          (0U << 12)
#define C_MLX16_ITC_PRIO3_PPM_TX_PRIO4          (1U << 12)
#define C_MLX16_ITC_PRIO3_PPM_TX_PRIO5          (2U << 12)
#define C_MLX16_ITC_PRIO3_PPM_TX_PRIO6          (3U << 12)
#define M_MLX16_ITC_PRIO3_PPM_ERR               (3U << 14)                      /*!< RW: priority PPM_ERR */
#define C_MLX16_ITC_PRIO3_PPM_ERR_PRIO3         (0U << 14)
#define C_MLX16_ITC_PRIO3_PPM_ERR_PRIO4         (1U << 14)
#define C_MLX16_ITC_PRIO3_PPM_ERR_PRIO5         (2U << 14)
#define C_MLX16_ITC_PRIO3_PPM_ERR_PRIO6         (3U << 14)
#else
#define M_MLX16_ITC_PRIO3_I2C_GLOBAL_RESET      (3U << 0)                       /*!< RW: priority I2C_GLOBAL_RESET */
#define C_MLX16_ITC_PRIO3_I2C_GLOBAL_RESET_PRIO3 (0U << 0)                      /*!< RW: priority 3 I2C_GLOBAL_RESET */
#define C_MLX16_ITC_PRIO3_I2C_GLOBAL_RESET_PRIO4 (1U << 0)                      /*!< RW: priority 4 I2C_GLOBAL_RESET */
#define C_MLX16_ITC_PRIO3_I2C_GLOBAL_RESET_PRIO5 (2U << 0)                      /*!< RW: priority 5 I2C_GLOBAL_RESET */
#define C_MLX16_ITC_PRIO3_I2C_GLOBAL_RESET_PRIO6 (3U << 0)                      /*!< RW: priority 6 I2C_GLOBAL_RESET */
#define M_MLX16_ITC_PRIO3_PPM_RX                (3U << 2)                       /*!< RW: priority PPM_RX */
#define C_MLX16_ITC_PRIO3_PPM_RX_PRIO3          (0U << 2)                       /*!< RW: priority 3 PPM_RX */
#define C_MLX16_ITC_PRIO3_PPM_RX_PRIO4          (1U << 2)                       /*!< RW: priority 4 PPM_RX */
#define C_MLX16_ITC_PRIO3_PPM_RX_PRIO5          (2U << 2)                       /*!< RW: priority 5 PPM_RX */
#define C_MLX16_ITC_PRIO3_PPM_RX_PRIO6          (3U << 2)                       /*!< RW: priority 6 PPM_RX */
#define M_MLX16_ITC_PRIO3_PPM_TX                (3U << 4)                       /*!< RW: priority PPM_TX */
#define C_MLX16_ITC_PRIO3_PPM_TX_PRIO3          (0U << 4)                       /*!< RW: priority 3 PPM_TX */
#define C_MLX16_ITC_PRIO3_PPM_TX_PRIO4          (1U << 4)                       /*!< RW: priority 4 PPM_TX */
#define C_MLX16_ITC_PRIO3_PPM_TX_PRIO5          (2U << 4)                       /*!< RW: priority 5 PPM_TX */
#define C_MLX16_ITC_PRIO3_PPM_TX_PRIO6          (3U << 4)                       /*!< RW: priority 6 PPM_TX */
#define M_MLX16_ITC_PRIO3_PPM_ERR               (3U << 6)                       /*!< RW: priority PPM_ERR */
#define C_MLX16_ITC_PRIO3_PPM_ERR_PRIO3         (0U << 6)                       /*!< RW: priority 3 PPM_ERR */
#define C_MLX16_ITC_PRIO3_PPM_ERR_PRIO4         (1U << 6)                       /*!< RW: priority 4 PPM_ERR */
#define C_MLX16_ITC_PRIO3_PPM_ERR_PRIO5         (2U << 6)                       /*!< RW: priority 5 PPM_ERR */
#define C_MLX16_ITC_PRIO3_PPM_ERR_PRIO6         (3U << 6)                       /*!< RW: priority 6 PPM_ERR */
#define M_MLX16_ITC_PRIO3_IO_IN0                (3U << 8)                       /*!< RW: priority IO_IN0 */
#define C_MLX16_ITC_PRIO3_IO_IN0_PRIO3          (0U << 8)                       /*!< RW: priority 3 IO_IN0 */
#define C_MLX16_ITC_PRIO3_IO_IN0_PRIO4          (1U << 8)                       /*!< RW: priority 4 IO_IN0 */
#define C_MLX16_ITC_PRIO3_IO_IN0_PRIO5          (2U << 8)                       /*!< RW: priority 5 IO_IN0 */
#define C_MLX16_ITC_PRIO3_IO_IN0_PRIO6          (3U << 8)                       /*!< RW: priority 6 IO_IN0 */
#define M_MLX16_ITC_PRIO3_IO_IN1                (3U << 10)                      /*!< RW: priority IO_IN1 */
#define C_MLX16_ITC_PRIO3_IO_IN1_PRIO3          (0U << 10)                      /*!< RW: priority 3 IO_IN1 */
#define C_MLX16_ITC_PRIO3_IO_IN1_PRIO4          (1U << 10)                      /*!< RW: priority 4 IO_IN1 */
#define C_MLX16_ITC_PRIO3_IO_IN1_PRIO5          (2U << 10)                      /*!< RW: priority 5 IO_IN1 */
#define C_MLX16_ITC_PRIO3_IO_IN1_PRIO6          (3U << 10)                      /*!< RW: priority 6 IO_IN1 */
#define M_MLX16_ITC_PRIO3_IO_IN2                (3U << 12)                      /*!< RW: priority IO_IN2 */
#define C_MLX16_ITC_PRIO3_IO_IN2_PRIO3          (0U << 12)                      /*!< RW: priority 3 IO_IN2 */
#define C_MLX16_ITC_PRIO3_IO_IN2_PRIO4          (1U << 12)                      /*!< RW: priority 4 IO_IN2 */
#define C_MLX16_ITC_PRIO3_IO_IN2_PRIO5          (2U << 12)                      /*!< RW: priority 5 IO_IN2 */
#define C_MLX16_ITC_PRIO3_IO_IN2_PRIO6          (3U << 12)                      /*!< RW: priority 6 IO_IN2 */
#define M_MLX16_ITC_PRIO3_IO_IN3                (3U << 14)                      /*!< RW: priority IO_IN3 */
#define C_MLX16_ITC_PRIO3_IO_IN3_PRIO3          (0U << 14)                      /*!< RW: priority 3 IO_IN3 */
#define C_MLX16_ITC_PRIO3_IO_IN3_PRIO4          (1U << 14)                      /*!< RW: priority 4 IO_IN3 */
#define C_MLX16_ITC_PRIO3_IO_IN3_PRIO5          (2U << 14)                      /*!< RW: priority 5 IO_IN3 */
#define C_MLX16_ITC_PRIO3_IO_IN3_PRIO6          (3U << 14)                      /*!< RW: priority 6 IO_IN3 */
#endif

#ifndef __MLX81330A01__
extern volatile uint16_t IO_MLX16_ITC_PRIO4_S __attribute__((nodp, addr(0x00090)));  /*!< IO_MLX16_ITC_PRIO4_S (System) */
#define M_MLX16_ITC_PRIO4                       (3U << 0)                       /*!< RW: Configure priority for interrupt[INT0_COUNT+INT1_COUNT+INT2_COUNT+31:INT0_COUNT+INT1_COUNT+INT2_COUNT+24] */
#define M_MLX16_ITC_PRIO4_TX_TIMEOUT            (3U << 0)                       /*!< RW: priority TX_TIMEOUT */
#define C_MLX16_ITC_PRIO4_TX_TIMEOUT_PRIO3      (0U << 0)                       /*!< RW: priority 3 TX_TIMEOUT */
#define C_MLX16_ITC_PRIO4_TX_TIMEOUT_PRIO4      (1U << 0)                       /*!< RW: priority 4 TX_TIMEOUT */
#define C_MLX16_ITC_PRIO4_TX_TIMEOUT_PRIO5      (2U << 0)                       /*!< RW: priority 5 TX_TIMEOUT */
#define C_MLX16_ITC_PRIO4_TX_TIMEOUT_PRIO6      (3U << 0)                       /*!< RW: priority 6 TX_TIMEOUT */
#endif

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
#define B_SPI_SS_FORCE                          (1U << 3)                       /*!< RW: SPI_SS behavior: 0: SPI_SS = 1 between "Words", SPI_SS = 0 while transmitting 1: SPI_SS always at 0 */
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

/* ***************** */
/* Block: PWM_SLAVE3 */
/* ***************** */
extern volatile uint16_t IO_PWM_SLAVE3_CMP __attribute__((nodp, addr(0x00152)));  /*!< IO_PWM_SLAVE3_CMP */
#define M_PWM_SLAVE3_CMP                        (65535U << 0)                   /*!< RW: Interrupt comparator register */

extern volatile uint16_t IO_PWM_SLAVE3_HT __attribute__((nodp, addr(0x00154)));  /*!< IO_PWM_SLAVE3_HT */
#define M_PWM_SLAVE3_HT                         (65535U << 0)                   /*!< RW: High threshold value */

extern volatile uint16_t IO_PWM_SLAVE3_LT __attribute__((nodp, addr(0x00156)));  /*!< IO_PWM_SLAVE3_LT */
#define M_PWM_SLAVE3_LT                         (65535U << 0)                   /*!< RW: Low threshold value */

extern volatile uint16_t IO_PWM_SLAVE3_PER __attribute__((nodp, addr(0x00158)));  /*!< IO_PWM_SLAVE3_PER */
#define M_PWM_SLAVE3_PER                        (65535U << 0)                   /*!< R: Not exist because PWM is only a slave */

extern volatile uint16_t IO_PWM_SLAVE3_CTRL __attribute__((nodp, addr(0x0015A)));  /*!< IO_PWM_SLAVE3_CTRL */
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

/* ****************** */
/* Block: PWM_MASTER2 */
/* ****************** */
extern volatile uint16_t IO_PWM_MASTER2_CMP __attribute__((nodp, addr(0x0015C)));  /*!< IO_PWM_MASTER2_CMP */
#define M_PWM_MASTER2_CMP                       (65535U << 0)                   /*!< RW: Interrupt comparator register */

extern volatile uint16_t IO_PWM_MASTER2_HT __attribute__((nodp, addr(0x0015E)));  /*!< IO_PWM_MASTER2_HT */
#define M_PWM_MASTER2_HT                        (65535U << 0)                   /*!< RW: High threshold value */

extern volatile uint16_t IO_PWM_MASTER2_LT __attribute__((nodp, addr(0x00160)));  /*!< IO_PWM_MASTER2_LT */
#define M_PWM_MASTER2_LT                        (65535U << 0)                   /*!< RW: Low threshold value */

extern volatile uint16_t IO_PWM_MASTER2_PER __attribute__((nodp, addr(0x00162)));  /*!< IO_PWM_MASTER2_PER */
#define M_PWM_MASTER2_PER                       (65535U << 0)                   /*!< RW: PWM period */

extern volatile uint16_t IO_PWM_MASTER2_CTRL __attribute__((nodp, addr(0x00164)));  /*!< IO_PWM_MASTER2_CTRL */
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

/* ************** */
/* Block: ADC_SAR */
/* ************** */
extern volatile uint16_t IO_ADC_SAR_CTRL __attribute__((nodp, addr(0x00166)));  /*!< IO_ADC_SAR_CTRL */
#define B_ADC_SAR_ADC_WIDTH                     (1U << 12)                      /*!< RW: ADC data width: 0: Up to 16 bits --> ADATA is one word per ADC 1: Up to 32 bits --> ADATA is two word per ADC */
#define M_ADC_SAR_ASB                           (3U << 10)                      /*!< RW: Auto Stand-By:
                                                                                 * 00: ADC is in stand-by only when not used (START = 0)
                                                                                 * 01: ADC is also in standby while waiting for triggers */
#define C_ADC_SAR_ASB_NEVER                     (2U << 10)                      /*!< 1x: ADC I never in stand-by */
#define M_ADC_SAR_INT_SCHEME                    (3U << 8)                       /*!< RW: Message interrupt and: */
#define C_ADC_SAR_INT_SCHEME_NOINT              (0U << 8)                       /*!< 00: No interrupt */
#define C_ADC_SAR_INT_SCHEME_EOC                (1U << 8)                       /*!< 01: Interrupt at each end of conversion */
#define C_ADC_SAR_INT_SCHEME_EOF                (2U << 8)                       /*!< 10: Interrupt at each end of frame */
#define C_ADC_SAR_INT_SCHEME_EOS                (3U << 8)                       /*!< 11: Interrupt at end of sequence */
#define B_ADC_SAR_SATURATE                      (1U << 7)                       /*!< RW: 0: ADC data is garbage in case of overflow or underflow 1: ADC ADATA is saturated to: 2N-1 in case of overflow (for a N bit ADATA) 0 in case of underflow */
#define B_ADC_SAR_NO_INTERLEAVE                 (1U << 6)                       /*!< RW: 0: EOA triggers SDATA update 1: EOC triggers SDATA update */
#define M_ADC_SAR_SOC_SOURCE                    (3U << 4)                       /*!< RW: Start Of Conversion (SOC) triggered by: */
#define C_ADC_SAR_SOC_SOURCE_HARD_CTRIG         (0U << 4)                       /*!< 00: Hardware (HARD_CTRIG) */
#define C_ADC_SAR_SOC_SOURCE_SOFT_TRIG          (1U << 4)                       /*!< 01: Firmware (SOFT_TRIG) */
#define C_ADC_SAR_SOC_SOURCE_HARD_SOFT_TRIG     (2U << 4)                       /*!< 10: Hardware (HARD_CTRIG) validated by firmware (SOFT_TRIG) */
#define C_ADC_SAR_SOC_SOURCE_PERMANENT          (3U << 4)                       /*!< 11: Permanent */
#define M_ADC_SAR_SOS_SOURCE                    (3U << 2)                       /*!< RW: Start Of Sequence (SOS) triggered by: */
#define C_ADC_SAR_SOS_SOURCE_2ND_HARD_CTRIG     (0U << 2)                       /*!< 00: Second hardware trigger (HARD_STRIG) */
#define C_ADC_SAR_SOS_SOURCE_HARD_CTRIG         (1U << 2)                       /*!< 01: First hardware trigger (HARD_STRIG) */
#define C_ADC_SAR_SOS_SOURCE_SOFT_TRIG          (2U << 2)                       /*!< 10: Firmware (SOFT_TRIG) */
#define C_ADC_SAR_SOS_SOURCE_PERMANENT          (3U << 2)                       /*!< 11: Permanent */
#define B_ADC_SAR_STOP                          (1U << 1)                       /*!< W: 1: Stop the ADC, all other bits are discarded 0: No effect; R: 0: ADC is running 1: ADC is stopped */
#define B_ADC_SAR_START                         (1U << 0)                       /*!< W: 1: Start the ADC, all other bits are discarded 0: No effect; R: 0: ADC is stopped 1: ADC is running */

extern volatile uint16_t IO_ADC_SAR_SBASE __attribute__((nodp, addr(0x00168)));  /*!< IO_ADC_SAR_SBASE */
#define M_ADC_SAR_SBASE_0                       (65535U << 0)                   /*!< W: Initial SBASE pointer; R: Current pointer to SDATA */

/* COPIED FROM MULAN2 (AMALTHEA!!!) */
/* bit  1: 0 :
 * bit  6: 2 : Channel selection
 * bit  9: 7 : Reference voltage selection.
 * bit 14:10 : Trigger selection.
 * bit 15    : ADC Divider
 */
/*! ADC Marker definition */
typedef enum
{
    C_ADC_NO_SIGN = 0U,                                                         /*!< Used when no signs are needed */
    C_ADC_NO_SIGN2 = 1U,                                                        /*!< Used when no signs are needed */
    C_ADC_EOF = 2U,                                                             /*!< End-of-Frame */
    C_ADC_EOS = 3U                                                              /*!< End-of-Sequence */
} AdcMarker_t;

/*! ADC Channels definition (0..31) */
typedef enum
{
    C_ADC_CH0 = 0U,                                                             /*!< VS, Internal high ohmic divider 1:21 */
    C_ADC_VS_HV = C_ADC_CH0,
    C_ADC_CH1 = 1U,                                                             /*!< TEMP, Internal temperature sensor */
    C_ADC_TEMP = C_ADC_CH1,
    C_ADC_CH2 = 2U,                                                             /*!< VDDD, Digital supply voltage */
    C_ADC_VDDD = C_ADC_CH2,
    C_ADC_CH3 = 3U,                                                             /*!< VDDA/2, Analogue supply voltage */
    C_ADC_VDDA = C_ADC_CH3,
    C_ADC_CH4 = 4U,                                                             /*!< Band-gap voltage digital */
    C_ADC_VBGD = C_ADC_CH4,
    C_ADC_CH5 = 5U,                                                             /*!< Vaux/4 */
    C_ADC_VAUX = C_ADC_CH5,
    C_ADC_CH6 = 6U,                                                             /*!< LIN auto configure amplifier output */
    C_ADC_LINAA_DM = C_ADC_CH6,
    C_ADC_CH7 = 7U,                                                             /*!< LIN auto configure common mode */
    C_ADC_LINAA_CM = C_ADC_CH7,
    C_ADC_CH8 = 8U,                                                             /*!< IO[0]/1.36 */
    C_ADC_IO0_LV = C_ADC_CH8,
    C_ADC_CH9 = 9U,                                                             /*!< IO[1]/1.36 */
    C_ADC_IO1_LV = C_ADC_CH9,
    C_ADC_CH10 = 10U,                                                           /*!< IO[2]/1.36 */
    C_ADC_IO2_LV = C_ADC_CH10,
    C_ADC_CH11 = 11U,                                                           /*!< IO[3]/1.36 */
    C_ADC_IO3_LV = C_ADC_CH11,
    C_ADC_CH12 = 12U,                                                           /*!< Phase U/21 */
    C_ADC_PH_U_HV = C_ADC_CH12,
    C_ADC_CH13 = 13U,                                                           /*!< Phase V/21 */
    C_ADC_PH_V_HV = C_ADC_CH13,
    C_ADC_CH14 = 14U,                                                           /*!< Phase W/21 */
    C_ADC_PH_W_HV = C_ADC_CH14,
    C_ADC_CH15 = 15U,                                                           /*!< Phase T/21 */
    C_ADC_PH_T_HV = C_ADC_CH15,
    C_ADC_CH16 = 16U,                                                           /*!< CSOUT - Current sense amplifier output voltage */
    C_ADC_MCUR = C_ADC_CH16,
    C_ADC_CH17 = 17U,                                                           /*!< VSM/21 Motor supply voltage divided by 21 */
    C_ADC_VSM_HV = C_ADC_CH17,
    C_ADC_CH18 = 18U,                                                           /*!< IO[0]/21 */
    C_ADC_IO0_HV = C_ADC_CH18,
    C_ADC_CH19 = 19U,                                                           /*!< CSOUTF - Filtered current sense amplifier output voltage */
    C_ADC_MCURF = C_ADC_CH19,
    C_ADC_CH20 = 20U,                                                           /*!< VSMF/21 Filtered motor supply voltage divided by 21 */
    C_ADC_VSMF_HV = C_ADC_CH20,
    C_ADC_CH25 = 25U,                                                            /*!< LIN */
    C_ADC_LIN = C_ADC_CH25
} AdcChannel_t;

/*! ADC reference voltage definition (0..7) */
typedef enum
{
    C_ADC_VREF_OFF = 0U,                                                        /*!< ADC Reference Voltage: OFF */
    C_ADC_VREF_0_75_V = 1U,                                                     /*!< ADC Reference Voltage: 0.75V */
    C_ADC_VREF_1_50_V = 2U,                                                     /*!< ADC Reference Voltage: 1.5V */
    C_ADC_VREF_2_50_V = 3U,                                                     /*!< ADC Reference Voltage: 2.5V */
    C_ADC_VREF_VDDA = 4U                                                        /*!< ADC Reference Voltage: VDDA */
} AdcVref_t;

/*! Hardware trigger source. (0..15) */
typedef enum
{
    C_ADC_HW_TRIGGER_MSTR1_CMP = 0U,                                            /*!< ADC HW-Trigger PWM_MASTER1_CMP */
    C_ADC_HW_TRIGGER_MSTR1_CNT = 1U,                                            /*!< ADC HW-Trigger PWM_MASTER1_CNT */
    C_ADC_HW_TRIGGER_SLV1_CMP = 2U,                                             /*!< ADC HW-Trigger PWM_SLAVE1_CMP */
    C_ADC_HW_TRIGGER_SLV2_CMP = 3U,                                             /*!< ADC HW-Trigger PWM_SLAVE2_CMP */
    C_ADC_HW_TRIGGER_SLV3_CMP = 4U,                                             /*!< ADC HW-Trigger PWM_SLAVE3_CMP */
    C_ADC_HW_TRIGGER_MSTR2_CMP = 5U,                                            /*!< ADC HW-Trigger PWM_MASTER2_CMP */
    C_ADC_HW_TRIGGER_MSTR2_CNT = 6U,                                            /*!< ADC HW-Trigger PWM_MASTER2_CNT */
    C_ADC_HW_TRIGGER_IO0 = 7U,                                                  /*!< ADC HW-Trigger IO0 */
    C_ADC_HW_TRIGGER_IO1 = 8U,                                                  /*!< ADC HW-Trigger IO1 */
    C_ADC_HW_TRIGGER_IO2 = 9U,                                                  /*!< ADC HW-Trigger IO2 */
    C_ADC_HW_TRIGGER_IO3 = 10U,                                                 /*!< ADC HW-Trigger IO3 */
    C_ADC_HW_TRIGGER_CTIMER0_INT1 = 11U,                                        /*!< ADC HW-Trigger CTIMER0_INT1 */
    C_ADC_HW_TRIGGER_CTIMER0_INT2 = 12U,                                        /*!< ADC HW-Trigger CTIMER0_INT2 */
    C_ADC_HW_TRIGGER_CTIMER0_INT3 = 13U,                                        /*!< ADC HW-Trigger CTIMER0_INT3 */
    C_ADC_HW_TRIGGER_CTIMER1_INT1 = 14U,                                        /*!< ADC HW-Trigger CTIMER1_INT1 */
    C_ADC_HW_TRIGGER_CTIMER1_INT2 = 15U,                                        /*!< ADC HW-Trigger CTIMER1_INT2 */
    C_ADC_HW_TRIGGER_CTIMER1_INT3 = 16U,                                        /*!< ADC HW-Trigger CTIMER1_INT3 */
    C_ADC_HW_TRIGGER_EOCpls2xADC_CLOCK = 24U,                                   /*!< End of Conversion +  2x ADC-Clock */
    C_ADC_HW_TRIGGER_EOCpls4xADC_CLOCK = 25U,                                   /*!< End of Conversion +  4x ADC-Clock */
    C_ADC_HW_TRIGGER_EOCpls6xADC_CLOCK = 26U,                                   /*!< End of Conversion +  6x ADC-Clock */
    C_ADC_HW_TRIGGER_EOCpls8xADC_CLOCK = 27U,                                   /*!< End of Conversion +  8x ADC-Clock */
    C_ADC_HW_TRIGGER_EOCpls10xADC_CLOCK = 28U,                                  /*!< End of Conversion + 10x ADC-Clock */
    C_ADC_HW_TRIGGER_EOCpls12xADC_CLOCK = 29U,                                  /*!< End of Conversion + 12x ADC-Clock */
    C_ADC_HW_TRIGGER_EOCpls14xADC_CLOCK = 30U,                                  /*!< End of Conversion + 14x ADC-Clock */
    C_ADC_HW_TRIGGER_EOCpls16xADC_CLOCK = 31U                                   /*!< End of Conversion + 16x ADC-Clock */
} AdcHwTrigger_t;

/*! SBASE Field definition */
/* bit  1: 0 : ADC-marker
 * bit  6: 2 : Channel selection
 * bit  9: 7 : Reference voltage selection.
 * bit 14:10 : Trigger selection.
 */
typedef union
{
    struct __attribute__((packed))
    {
        AdcMarker_t u2AdcMarker         : 2;                                    /**< ADC Marker (end-of-sequence or end-of-frame) */
        AdcChannel_t u5AdcChannel       : 5;                                    /**< ADC Channel selection (0..31) */
        AdcVref_t u3AdcVref             : 3;                                    /**< ADC reference voltage selection */
        AdcHwTrigger_t u5AdcTrigger     : 5;                                    /**< ADC Hardware Trigger selection */
        uint16_t u1AdcReserved          : 1;                                    /**< ADC data selection */
    } s;                                                                        /**< Structure */
    uint16_t u16;                                                               /**< 16-bit SBASE field */
} ADC_SDATA_t;

#if defined (_OLD)
/* Reference voltage */
#define C_ADC_REF_VDDA                          (4U << 7)                       /*!< ADC Reference Voltage: VDDA */
#define C_ADC_REF_2_50_V                        (3U << 7)                       /*!< ADC Reference Voltage: 2.5V */
#define C_ADC_REF_1_50_V                        (2U << 7)                       /*!< ADC Reference Voltage: 1.5V */
#define C_ADC_REF_0_75_V                        (1U << 7)                       /*!< ADC Reference Voltage: 0.75V */
#define C_ADC_REF_OFF                           (0U << 7)                       /*!< ADC Reference Voltage: OFF */

/* HW trigger source */
#define C_ADC_HW_TRIGGER_MSTR1_CMP              (0U << 10)                      /*!< Trigger Master 1 PWM CMP */
#define C_ADC_HW_TRIGGER_MSTR1_CNT              (1U << 10)                      /*!< Trigger Master 1 PWM CNT */
#define C_ADC_HW_TRIGGER_SLV1_CMP               (2U << 10)                      /*!< Trigger Slave 1 PWM CMP */
#define C_ADC_HW_TRIGGER_SLV2_CMP               (3U << 10)                      /*!< Trigger Slave 2 PWM CMP */
#define C_ADC_HW_TRIGGER_SLV3_CMP               (4U << 10)                      /*!< Trigger Slave 3 PWM CMP */
#define C_ADC_HW_TRIGGER_MSTR2_CMP              (5U << 10)                      /*!< Trigger Master 2 PWM CMP */
#define C_ADC_HW_TRIGGER_MSTR2_CNT              (6U << 10)                      /*!< Trigger Master 2 PWM CNT */
#define C_ADC_HW_TRIGGER_IO0                    (7U << 10)                      /*!< Trigger by IO[0] */
#define C_ADC_HW_TRIGGER_IO1                    (8U << 10)                      /*!< Trigger by IO[1] */
#define C_ADC_HW_TRIGGER_IO2                    (9U << 10)                      /*!< Trigger by IO[2] */
#define C_ADC_HW_TRIGGER_IO3                    (10U << 10)                     /*!< Trigger by IO[3] */
#define C_ADC_HW_TRIGGER_CTIMER0_INT1           (11U << 10)                     /*!< Trigger by CTimer0, Interrupt 1 */
#define C_ADC_HW_TRIGGER_CTIMER0_INT2           (12U << 10)                     /*!< Trigger by CTimer0, Interrupt 2 */
#define C_ADC_HW_TRIGGER_CTIMER0_INT3           (13U << 10)                     /*!< Trigger by CTimer0, Interrupt 3 */
#define C_ADC_HW_TRIGGER_CTIMER1_INT1           (14U << 10)                     /*!< Trigger by CTimer1, Interrupt 1 */
#define C_ADC_HW_TRIGGER_CTIMER1_INT2           (15U << 10)                     /*!< Trigger by CTimer1, Interrupt 2 */
#define C_ADC_HW_TRIGGER_CTIMER1_INT3           (16U << 10)                     /*!< Trigger by CTimer1, Interrupt 3 */
#define C_ADC_HW_TRIGGER_EOCpls2xADC_CLOCK      (24U << 10)                     /*!< End of Conversion +  2x ADC-Clock */
#define C_ADC_HW_TRIGGER_EOCpls4xADC_CLOCK      (25U << 10)                     /*!< End of Conversion +  4x ADC-Clock */
#define C_ADC_HW_TRIGGER_EOCpls6xADC_CLOCK      (26U << 10)                     /*!< End of Conversion +  6x ADC-Clock */
#define C_ADC_HW_TRIGGER_EOCpls8xADC_CLOCK      (27U << 10)                     /*!< End of Conversion +  8x ADC-Clock */
#define C_ADC_HW_TRIGGER_EOCpls10xADC_CLOCK     (28U << 10)                     /*!< End of Conversion + 10x ADC-Clock */
#define C_ADC_HW_TRIGGER_EOCpls12xADC_CLOCK     (29U << 10)                     /*!< End of Conversion + 12x ADC-Clock */
#define C_ADC_HW_TRIGGER_EOCpls14xADC_CLOCK     (30U << 10)                     /*!< End of Conversion + 14x ADC-Clock */
#define C_ADC_HW_TRIGGER_EOCpls16xADC_CLOCK     (31U << 10)                     /*!< End of Conversion + 16x ADC-Clock */

/* Channels */
#define C_ADC_CH0                               (0U << 2)                       /*!< VS, Internal high ohmic divider 1:14 */
#define C_ADC_CH1                               (1U << 2)                       /*!< TEMP, Internal temperature sensor */
#define C_ADC_CH2                               (2U << 2)                       /*!< VDDD, Digital supply voltage */
#define C_ADC_CH3                               (3U << 2)                       /*!< VDDA/2, Analogue supply voltage */
#define C_ADC_CH4                               (4U << 2)                       /*!< Band-gap voltage digital */
#define C_ADC_CH5                               (5U << 2)                       /*!< Vaux/4 */
#define C_ADC_CH6                               (6U << 2)                       /*!< LIN auto configuration amplifier output */
#define C_ADC_CH7                               (7U << 2)                       /*!< LIN auto configuration common mode */
#define C_ADC_CH8                               (8U << 2)                       /*!< IO[0]/1.36 */
#define C_ADC_CH9                               (9U << 2)                       /*!< IO[1]/1.36 */
#define C_ADC_CH10                              (10U << 2)                      /*!< IO[2]/1.36 */
#define C_ADC_CH11                              (11U << 2)                      /*!< IO[3]/1.36 */
#define C_ADC_CH12                              (12U << 2)                      /*!< Phase U/21 */
#define C_ADC_CH13                              (13U << 2)                      /*!< Phase V/21 */
#define C_ADC_CH14                              (14U << 2)                      /*!< Phase W/21 */
#define C_ADC_CH15                              (15U << 2)                      /*!< Phase T/21 */
#define C_ADC_CH16                              (16U << 2)                      /*!< CSOUT - Current sense amplifier output voltage */
#define C_ADC_CH17                              (17U << 2)                      /*!< VSM/21 Motor supply voltage divided by 21 */
#define C_ADC_CH18                              (18U << 2)                      /*!< IO[0]/21 */
#define C_ADC_CH19                              (19U << 2)                      /*!< CSOUTF - Filtered current sense amplifier output voltage */
#define C_ADC_CH20                              (20U << 2)                      /*!< VSMF/21 Filtered motor supply voltage divided by 21 */
#define C_ADC_CH25                              (25U << 2)                      /*!< LIN */

#define C_ADC_VS            (C_ADC_CH0 | (uint16_t)C_ADC_REF_2_50_V)            /*!<  0           2.5V    Vs-unfiltered (divided by 21) */
#define C_ADC_TJ            (C_ADC_CH1 | (uint16_t)C_ADC_REF_2_50_V)            /*!<  1           2.5V    Chip-Junction temperature */
#define C_ADC_VDDD          (C_ADC_CH2 | (uint16_t)C_ADC_REF_2_50_V)            /*!<  2           2.5V    VDDD voltage */
#define C_ADC_VDDA          (C_ADC_CH3 | (uint16_t)C_ADC_REF_2_50_V)            /*!<  3           2.5V    VDDA voltage (divided by 2) */
#define C_ADC_BGD           (C_ADC_CH4 | (uint16_t)C_ADC_REF_2_50_V)            /*!<  4           2.5V    Band-gap voltage (Digital) */
#define C_ADC_VAUX          (C_ADC_CH5 | (uint16_t)C_ADC_REF_2_50_V)            /*!<  5           2.5V    VAUX voltage (divided by 4) */
#define C_ADC_LINAA_DM      (C_ADC_CH6 | (uint16_t)C_ADC_REF_2_50_V)            /*!<  6           2.5V    LIN-AA Differential-mode */
#define C_ADC_LINAA_CM      (C_ADC_CH7 | (uint16_t)C_ADC_REF_2_50_V)            /*!<  7           2.5V    LIN-AA Common-mode */
#define C_ADC_IO0           (C_ADC_CH8 | (uint16_t)C_ADC_REF_2_50_V)            /*!<  8           2.5V    IO[0] (X-Y Resolver) */
#define C_ADC_IO1           (C_ADC_CH9 | (uint16_t)C_ADC_REF_2_50_V)            /*!<  9           2.5V    IO[1] (X-Y Resolver) */
#define C_ADC_IO2           (C_ADC_CH10 | (uint16_t)C_ADC_REF_2_50_V)           /*!< 10           2.5V    IO[2] (X-Y Resolver) */
#define C_ADC_IO3           (C_ADC_CH11 | (uint16_t)C_ADC_REF_2_50_V)           /*!< 11           2.5V    IO[3] (X-Y Resolver) */
#define C_ADC_VPHU          (C_ADC_CH12 | (uint16_t)C_ADC_REF_2_50_V)           /*!< 12           2.5V    Phase U voltage (divided by 21) */
#define C_ADC_VPHV          (C_ADC_CH13 | (uint16_t)C_ADC_REF_2_50_V)           /*!< 13           2.5V    Phase V voltage (divided by 21) */
#define C_ADC_VPHW          (C_ADC_CH14 | (uint16_t)C_ADC_REF_2_50_V)           /*!< 14           2.5V    Phase W voltage (divided by 21) */
#define C_ADC_VPHT          (C_ADC_CH15 | (uint16_t)C_ADC_REF_2_50_V)           /*!< 15           2.5V    Phase T voltage (divided by 21) */
#define C_ADC_MCUR          (C_ADC_CH16 | (uint16_t)C_ADC_REF_2_50_V)           /*!< 16           2.5V    CSOUT */
#define C_ADC_VSM           (C_ADC_CH17 | (uint16_t)C_ADC_REF_2_50_V)           /*!< 17           2.5V    VSM-unfiltered (divided by 21) */
#define C_ADC_IO0HV         (C_ADC_CH18 | (uint16_t)C_ADC_REF_2_50_V)           /*!< 18           2.5V    IO[0].HV test-purpose */
#define C_ADC_MCURF         (C_ADC_CH19 | (uint16_t)C_ADC_REF_2_50_V)           /*!< 19           2.5V    CSOUT-filtered */
#define C_ADC_VSMF          (C_ADC_CH20 | (uint16_t)C_ADC_REF_2_50_V)           /*!< 20           2.5V    VSM-unfiltered (divided by 21) */
#define C_ADC_LIN           (C_ADC_CH25 | (uint16_t)C_ADC_REF_2_50_V)           /*!< 25           2.5V    LIN */

#define C_ADC_EOF           0x0002U                                             /*!< End-of-Frame */
#define C_ADC_EOS           0x0003U                                             /*!< End-of-Sequence */
#endif /* defined (_OLD) */

#define C_ADC_SBASE_MSW                         0x0000U                         /*!< ADC SBASE MSW */

extern volatile uint16_t IO_ADC_SAR_STATUS __attribute__((nodp, addr(0x0016A)));  /*!< IO_ADC_SAR_STATUS */
#define B_ADC_SAR_ABORTED                       (1U << 12)                      /*!< R: 0: Sequence running normally 1: Sequence has been aborted; W: 0: No effect 1: Clear flag */
#define B_ADC_SAR_FRAME_ERR                     (1U << 11)                      /*!< R: 0: No frame error 1: A Frame error occurred; W: 0: No effect 1: Clear flag */
#define B_ADC_SAR_MEM_ERR                       (1U << 10)                      /*!< R: 0: No error occurred 1: A memory error occurred; W: 0: No effect 1: Clear flag */
#define B_ADC_SAR_ADC_ERR                       (1U << 9)                       /*!< R: 0: No ADC error occurred 1: An ADC error occurred; W: 0: No effect 1: Clear flag */
#define B_ADC_SAR_ADC_OVF                       (1U << 8)                       /*!< R: 0: No error occurred 1: An error occurred; W: 0: No effect 1: Clear flag */
#define M_ADC_SAR_STATE                         (3U << 6)                       /*!< R: See below; W: No effect */
#define C_ADC_SAR_STATE_IDLE                    (0U << 6)                       /*!< 00: Idle */
#define C_ADC_SAR_STATE_MEM_TRANSFER            (1U << 6)                       /*!< 01: Memory transfer */
#define C_ADC_SAR_STATE_CONVERSION              (2U << 6)                       /*!< 10: Conversion ongoing */
#define C_ADC_SAR_STATE_WAIT_FOR_TRIGGER        (3U << 6)                       /*!< 11: Waiting for trigger */
#define M_ADC_SAR_LAST_INT_SRC                  (3U << 4)                       /*!< R: See below; W: No effect
                                                                                 * 00: No interrupt occurred since last start of ADC
                                                                                 * 01: Last interrupt is due to an end of conversion
                                                                                 * 10: Last interrupt is due to an end of frame
                                                                                 * 11: Last interrupt is due to an end of sequence */
#define B_ADC_SAR_READY                         (1U << 3)                       /*!< R: READY signal from ADC; W: No effect */
#define B_ADC_SAR_SW_TRIG                       (1U << 2)                       /*!< W: 1: Triggers a Start of Sequence or a Start of Conversion 0: No effect; R: Always 0 */
#define B_ADC_SAR_RESUME                        (1U << 1)                       /*!< W: 1: Enable all triggers, all other bits are discarded 0: No effect; R: 0: All triggers are disabled 1: All triggers are enabled */
#define B_ADC_SAR_PAUSE                         (1U << 0)                       /*!< W: 1: Disable all triggers, all other bits are discarded 0: No effect; R: 0: All triggers are enabled 1: All triggers are disabled */

extern volatile uint16_t IO_ADC_SAR_CLK_DIV __attribute__((nodp, addr(0x0016C)));  /*!< IO_ADC_SAR_CLK_DIV */
#define M_ADC_SAR_ADC_CLK_DIV                   (127U << 0)                     /*!< RW: ADC_CLK = MCU_CLK / ( ADC_CLK_DIV + 1 ) */

extern volatile uint16_t IO_ADC_SAR_TEST __attribute__((nodp, addr(0x0016E)));  /*!< IO_ADC_SAR_TEST */
#define B_ADC_SAR_TEST_TRIM                     (1U << 3)                       /*!< RW: activate trimming sequence */
#define B_ADC_SAR_TEST_CONV4                    (1U << 2)                       /*!< RW: activate test conversion sequence 4 */
#define B_ADC_SAR_TEST_CONV3                    (1U << 1)                       /*!< RW: activate test conversion sequence 3 */
#define B_ADC_SAR_TEST_CONV2                    (1U << 0)                       /*!< RW: activate test conversion sequence 2 */

extern volatile uint16_t IO_ADC_SAR_DATA __attribute__((nodp, addr(0x00170)));  /*!< IO_ADC_SAR_DATA */
#define B_ADC_SAR_SEL_TR_ADCREF                 (1U << 10)                      /*!< RW: if 1: connect ADC_TRIMPORT to TR_ADCREF pins, else connect from ports. */
#define M_ADC_SAR_ADC_DATA                      (1023U << 0)                    /*!< RW: Digital output that controls DAC */

extern volatile uint16_t IO_ADC_SAR_TR_ADCREF12 __attribute__((nodp, addr(0x00172)));  /*!< IO_ADC_SAR_TR_ADCREF12 */
#define M_ADC_SAR_TR_ADCREF2                    (127U << 7)                     /*!< RW: ADC reference trimming for ADC_REF[1:0]=10 */
#define M_ADC_SAR_TR_ADCREF1                    (127U << 0)                     /*!< RW: ADC reference trimming for ADC_REF[1:0]=01 */

extern volatile uint16_t IO_ADC_SAR_TR_ADCREF3 __attribute__((nodp, addr(0x00174)));  /*!< IO_ADC_SAR_TR_ADCREF3 */
#define B_ADC_SAR_IDDQ_ADC                      (1U << 7)                       /*!< RW: put ADC in a very low power mode (for IDDQ and STOP mode) */
#define M_ADC_SAR_TR_ADCREF3                    (127U << 0)                     /*!< RW: ADC reference trimming for ADC_REF[1:0]=11 */

/* ******************* */
/* Block: EEPROM_FLASH */
/* ******************* */
extern volatile uint16_t IO_EEPROM_FLASH_T __attribute__((nodp, addr(0x00176)));  /*!< IO_EEPROM_FLASH_T */
#define B_EEPROM_FLASH_LOCK_T                   (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_EEPROM_FLASH_T_50US                   (31U << 10)                     /*!< RW: This value is used to fine tune the 50us reference used in the shell The following equation is used: (T_50US+ 1 ) * 10us ~ 50us Reset value: 50us, precision = 10us */
#define M_EEPROM_FLASH_T_10US                   (31U << 5)                      /*!< RW: This value is used to fine tune the 10us reference used in the shell The following equation is used: (T_10US+ 1 ) * 1us ~ 10us Reset value: 10us, precision = 1us */
#define M_EEPROM_FLASH_T_50NS                   (7U << 2)                       /*!< RW: This value is used to fine tune the 50ns reference used in the shell ( T_50NS + 1 ) * MCU_CLK >= 50ns Reset value: 5 * MCU_CLK (Valid up to 100MHz), Precision = 1 MCU_CLK */
#define B_EEPROM_FLASH_T_10NS                   (1U << 0)                       /*!< RW: Indicate the number of mcu_clk period needed to reach 20 ns delay 0: 0.5 mcu_clk period 1: 1 mcu_clk period */

extern volatile uint16_t IO_EEPROM_FLASH_T_1US __attribute__((nodp, addr(0x00178)));  /*!< IO_EEPROM_FLASH_T_1US */
#define B_EEPROM_FLASH_LOCK_1US                 (1U << 15)                      /*!< RW: Field used for the CAMCU project */
#define M_EEPROM_FLASH_T_1US                    (31U << 0)                      /*!< RW: This value is used to fine tune the 1us reference used in the shell The following equation is used: (T_1US+ 1 ) * 1us ~ 1us Reset value: 1us, precision = 1us */

extern volatile uint16_t IO_EEPROM_FLASH_DMA __attribute__((nodp, addr(0x0017A)));  /*!< IO_EEPROM_FLASH_DMA */
#define B_EEPROM_FLASH_EE_FL_PDN_LOW            (1U << 15)                      /*!< RW: Test moe which allows to clear Pdn pin */
#define B_EEPROM_FLASH_EE_BUFFER_MODE           (1U << 10)                      /*!< RW: 0: EEPROM_buffer [31:0] is used for EE_SIG_L and EE_SIG_H 1: EEPROM _buffer [63:32] is used for EE_SIG_L and EE_SIG_H */
#define B_EEPROM_FLASH_EE_EXTEND_DATA           (1U << 9)                       /*!< RW: During a write, allows to extend the 16 bits of the write bus on all the bits of the memory (including ECC). This is only for EEPROM */
#define B_EEPROM_FLASH_EE_DMA                   (1U << 8)                       /*!< RW: Direct memory access without ECC. This is only for EEPROM */
#define M_EEPROM_FLASH_EE_FL_VERSION            (15U << 4)                      /*!< R: Gives the version of the eeprom_flash shell; W: No effect */
#define B_EEPROM_FLASH_FL_BUFFER_MODE           (1U << 1)                       /*!< RW: 0: Flash_buffer [31:0] is used for FL_SIG_L and FL_SIG_H 1: Flash_buffer [63:32] is used for FL_SIG_L and FL_SIG_H */
#define B_EEPROM_FLASH_FL_DMA                   (1U << 0)                       /*!< RW: Direct memory access without ECC. This is only for FLASH */

extern volatile uint16_t IO_EEPROM_FLASH_CMD_STS __attribute__((nodp, addr(0x0017C)));  /*!< IO_EEPROM_FLASH_CMD_STS */
#define M_EEPROM_FLASH_FL_COMMAND               (65535U << 0)                   /*!< W: Command to be executed; R: Status of command */
#define M_EEPROM_FLASH_FL_STATUS                (15U << 0)                      /*!< R: Status of command */
#define C_EEPROM_FLASH_FL_STATUS_NORMAL         (0U << 0)                       /*!< R: Status NORMAL */
#define C_EEPROM_FLASH_FL_STATUS_STDBY          (1U << 0)                       /*!< R: Status STDBY */
#define C_EEPROM_FLASH_FL_STATUS_SECTOR_ERASE   (3U << 0)                       /*!< R: Status SECTOR_ERASE */
#define C_EEPROM_FLASH_FL_STATUS_PAGE_PROGRAM   (4U << 0)                       /*!< R: Status PAGE_PROGRAM */
#define C_EEPROM_FLASH_FL_STATUS_POWER_ON       (6U << 0)                       /*!< R: Status POWER_ON */
#define C_EEPROM_FLASH_FL_STATUS_CONFIGRABLE    (7U << 0)                       /*!< R: Status CONFIGRABLE */
#define C_EEPROM_FLASH_FL_STATUS_POWER_OFF      (15U << 0)                      /*!< R: Status POWER_OFF */

extern volatile uint16_t IO_EEPROM_FLASH_FL_TIME __attribute__((nodp, addr(0x0017E)));  /*!< IO_EEPROM_FLASH_FL_TIME */
#define B_EEPROM_FLASH_FL_LOCK_ER_WR            (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_EEPROM_FLASH_FL_WR_TIME               (127U << 8)                     /*!< RW: Write time = ( WR_TIME + 1 ) * 100us 100us <= Write time <= 12.7 ms, resolution = 100us Reset value: 5ms */
#define M_EEPROM_FLASH_FL_ER_TIME               (255U << 0)                     /*!< RW: Erase time = ( ER_TIME + 1 ) *100us 0ms <= Erase time <= 25.5ms, resolution = 100us Reset value: 20ms */

extern volatile uint16_t IO_EEPROM_FLASH_FL_CTRL __attribute__((nodp, addr(0x00180)));  /*!< IO_EEPROM_FLASH_FL_CTRL */
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

extern volatile uint16_t IO_EEPROM_FLASH_FL __attribute__((nodp, addr(0x00182)));  /*!< IO_EEPROM_FLASH_FL */
#define B_EEPROM_FLASH_FL_EXTENDED_DATA         (1U << 8)                       /*!< RW: During a write, allows to extend the 16 bits of the write bus on all the bits of the memory (including ECC) */
#define B_EEPROM_FLASH_FL_DATA_CORRUPTED        (1U << 1)                       /*!< R: Corruption of data is detected (2 errors in data or 1 error in address); W: Write 1 to clear it */
#define B_EEPROM_FLASH_FL_SBE                   (1U << 0)                       /*!< R: Single bit error correction. The data has been successfully corrected; W: Write 1 to clear it */

extern volatile uint16_t IO_EEPROM_FLASH_FL_SEC_PG __attribute__((nodp, addr(0x00184)));  /*!< IO_EEPROM_FLASH_FL_SEC_PG */
#define B_EEPROM_FLASH_FL_CS_AREA               (1U << 8)                       /*!< RW: Set if the program or erase operation impacts MFA or CS */
#define M_EEPROM_FLASH_FL_SECTOR_NUMBER         (15U << 4)                      /*!< RW: Define which sector of the MFA or CS will be impacted by erase/program command. */
#define M_EEPROM_FLASH_FL_PAGE_NUMBER           (15U << 0)                      /*!< RW: Define which page of the MFA or CS will be impacted by erase/program command. */

extern volatile uint16_t IO_EEPROM_FLASH_EE_CTRL_S __attribute__((nodp, addr(0x00186)));  /*!< IO_EEPROM_FLASH_EE_CTRL_S (System) */
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
                                                                                 * Others : The state machine performs XFAB flow(prewrite, erase, write) */
#define M_EEPROM_FLASH_EE_WE_KEY                (15U << 4)                      /*!< RW: Protection bit: Key for write enable WE_KEY = 0x7 --> write accesses are valid WE_KEY != 0x7 --> write accesses are invalid This port is automatically reset at the end of a write access in normal mode. Read access is only possible if WE_KEY != 0x7. */
#define B_EEPROM_FLASH_EE_CONFIGURED            (1U << 1)                       /*!< RW: When set, the EEPROM leaves configurable state. */
#define B_EEPROM_FLASH_EE_ACTIVE                (1U << 0)                       /*!< RW: When set, the EEPROM is not in stand-by mode */

extern volatile uint16_t IO_EEPROM_FLASH_EE_DATA_ERROR __attribute__((nodp, addr(0x00188)));  /*!< IO_EEPROM_FLASH_EE_DATA_ERROR */
#define B_EEPROM_FLASH_EE_DATA_CORRUPTED_2      (1U << 3)                       /*!< R: Corruption of the second 32 bits word is detected (2 errors in data or 1 error in address);
                                                                                 * W: Write 1 to clear it */
#define B_EEPROM_FLASH_EE_SBE_2                 (1U << 2)                       /*!< R: Single bit error correction of the second 32 bits word. If one, then the data has been correctly corrected;
                                                                                 * W: Write 1 to clear it */
#define B_EEPROM_FLASH_EE_DATA_CORRUPTED_1      (1U << 1)                       /*!< R: Corruption of the first 32 bits word is detected (2 errors in data or 1 error in address); W: Write 1 to clear it */
#define B_EEPROM_FLASH_EE_SBE_1                 (1U << 0)                       /*!< R: Single bit error correction of the first 32 bits word. If one, then the data has been correctly corrected; W: Write 1 to clear it */

extern volatile uint16_t IO_EEPROM_FLASH_EE_WTIME __attribute__((nodp, addr(0x0018A)));  /*!< IO_EEPROM_FLASH_EE_WTIME */
#define B_EEPROM_FLASH_EE_LOCK_WR               (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_EEPROM_FLASH_EE_WR_TIME               (127U << 8)                     /*!< RW: Write time = (PRE_WR_TIME + 1) * 10us 10us <= Write time <= 1.27ms, resolution = 10us Reset value: 300us */
#define M_EEPROM_FLASH_EE_PRE_WR_TIME           (127U << 0)                     /*!< RW: Pre-Write time = (PRE_WR_TIME + 1) * 10us 10us <= Write time <= 1.27ms, resolution = 10us Reset value: 300us */

extern volatile uint16_t IO_EEPROM_FLASH_EE_ERTIME __attribute__((nodp, addr(0x0018C)));  /*!< IO_EEPROM_FLASH_EE_ERTIME */
#define B_EEPROM_FLASH_EE_LOCK_ER               (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_EEPROM_FLASH_EE_ER_TIME               (63U << 0)                      /*!< RW: Erase time = ( ER_TIME + 1 ) *100us 0ms <= Erase time <= 6.3ms, resolution = 100us Reset value: 6ms */

extern volatile uint16_t IO_EEPROM_FLASH_EE_PROG_CYCLE __attribute__((nodp, addr(0x0018E)));  /*!< IO_EEPROM_FLASH_EE_PROG_CYCLE */
#define B_EEPROM_FLASH_EE_PROGRAM_CYCLE_LOCK    (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_EEPROM_FLASH_EE_PROGRAM_CYCLE         (31U << 0)                      /*!< RW: This field defined the number of program cycles needed for writing a word in the EEPROM Number of program cycles = PROGRAM_CYCLE + 1 1 <= Number of program cycles <= 32, Reset value: 16 */

extern volatile uint16_t IO_EEPROM_FLASH_EE_WS __attribute__((nodp, addr(0x00190)));  /*!< IO_EEPROM_FLASH_EE_WS */
#define B_EEPROM_FLASH_EE_LOCK_RD               (1U << 15)                      /*!< RW: Lock the port */
#define M_EEPROM_FLASH_EE_WAIT_STATES           (15U << 8)                      /*!< RW: Number of MCU clock periods between the rise of CLK and the rise of wishbone ready signal. Ready delay = (EE_WAIT_STATES+1) * MCU_CLK */
#define M_EEPROM_FLASH_EE_HALT_BEHAVIOR         (3U << 0)                       /*!< RW: Define the EEPROM behavior when application is halted 2'b00: Stay in the current state 2'b01: Go in mode Stand-by 2'b1x: Deep power down mode (will make behave the FLASH as if FL_HALT_BEHAVIOR is 2'b1x) */

extern volatile uint16_t IO_EEPROM_FLASH_EE_INT_INV_SRC __attribute__((nodp, addr(0x00192)));  /*!< IO_EEPROM_FLASH_EE_INT_INV_SRC */
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

extern volatile uint16_t IO_EEPROM_FLASH __attribute__((nodp, addr(0x0019C)));  /*!< IO_EEPROM_FLASH */
#define B_EEPROM_FLASH_FL_COMPLETED             (1U << 15)                      /*!< R: If 1'b1, the test previously requested is finished; W: Write 1 to clear the flag */
#define B_EEPROM_FLASH_FL_ERROR                 (1U << 14)                      /*!< R: Flag set when an error happen during a test. Flag cleared when a new test starts */
#define M_EEPROM_FLASH_FL_TEST_AREA             (3U << 10)                      /*!< RW: Select which area is tested by the test function : 2'b00: All the memory 2'b01: All the memory except last physical word 2'b10: Only MFA 2'b11: Only CS */
#define M_EEPROM_FLASH_FL_ACCESS_ORDER          (3U << 8)                       /*!< RW: Define read access order 2'b00: Read from bottom to top 2'b01: Read from top to bottom 2'b10: Pseudo random access 2'b11: Not used */
#define M_EEPROM_FLASH_FL_PATTERN_ID            (7U << 4)                       /*!< RW: Expected pattern in the memory 3'b000: FL_SBE and FL_DATA_CORRUPTED memory content check 3'b001: All 0's 3'b010: All 1's 3'b011: Functional Chess pattern 3'b100: Inverse functional chess pattern 3'b101: Chess pattern 3'b110: Inverse chess pattern 3'b111: Not used */

extern volatile uint16_t IO_EEPROM_FLASH_FL_BITS_ERROR __attribute__((nodp, addr(0x0019E)));  /*!< IO_EEPROM_FLASH_FL_BITS_ERROR */
#define M_EEPROM_FLASH_FL_BITS_ERRORS           (511U << 0)                     /*!< RW: Saturing count of bit mismatching pattern defined by FL_PATTERN_ID */

extern volatile uint16_t IO_EEPROM_FLASH_FL_SIG_L __attribute__((nodp, addr(0x001A0)));  /*!< IO_EEPROM_FLASH_FL_SIG_L */
#define M_EEPROM_FLASH_FL_SIG_L                 (65535U << 0)                   /*!< R: Read the 16 LSB of the signature in test; W: Initialise the 16 LSB of the signature in test */

extern volatile uint16_t IO_EEPROM_FLASH_FL_SIG_H __attribute__((nodp, addr(0x001A2)));  /*!< IO_EEPROM_FLASH_FL_SIG_H */
#define M_EEPROM_FLASH_FL_SIG_H                 (65535U << 0)                   /*!< R: Read the 16 MSB of the signature in test; W: Initialise the 16 MSB of the signature in test */

extern volatile uint16_t IO_EEPROM_FLASH_EE_CTRL2 __attribute__((nodp, addr(0x001A4)));  /*!< IO_EEPROM_FLASH_EE_CTRL2 */
#define B_EEPROM_FLASH_EE_COMPLETED             (1U << 15)                      /*!< R: If 1'b1, the test previously requested is finished; W: Write 1 to clear the flag */
#define B_EEPROM_FLASH_EE_ERROR                 (1U << 14)                      /*!< R: Flag set when an error happen during a test. Flag cleared when a new test starts */
#define B_EEPROM_FLASH_EE_IN_PROGRESS           (1U << 13)                      /*!< R: 0: No test running 1: Test in progress */
#define M_EEPROM_FLASH_EE_TEST_AREA             (3U << 10)                      /*!< RW: Select which area is tested by the test function: 00: All the memory 01: All the memory except last physical word 10: Only MFA 11: Only CS except last physical word */
#define M_EEPROM_FLASH_EE_ACCESS_ORDER          (3U << 8)                       /*!< RW: Define read access order 2'b00: Read from bottom to top 2'b01: Read from top to bottom 2'b10: Pseudo random access 2'b11: Not used */
#define M_EEPROM_FLASH_EE_PATTERN_ID            (7U << 4)                       /*!< RW: Expected pattern in the memory 3'b000: EE_SBE and EE_DATA_CORRUPTED memory content check 3'b001: All 0's 3'b010: All 1's 3'b011: Functional Chess pattern 3'b100: Inverse functional chess pattern 3'b101: Chess pattern (same as 3'b011) 3'b110: Inverse chess pattern (same as 3'b101) 3'b111: Not used */
#define M_EEPROM_FLASH_EE_CMD                   (3U << 0)                       /*!< RW: Enable a test function 1'b1: Margin read test function */

extern volatile uint16_t IO_EEPROM_FLASH_EE_BITS_ERROR __attribute__((nodp, addr(0x001A6)));  /*!< IO_EEPROM_FLASH_EE_BITS_ERROR */
#define M_EEPROM_FLASH_EE_BITS_ERRORS           (511U << 0)                     /*!< RW: Saturing count of bit mismatching pattern defined by EE_PATTERN_ID */

extern volatile uint16_t IO_EEPROM_FLASH_EE_SIG_L __attribute__((nodp, addr(0x001A8)));  /*!< IO_EEPROM_FLASH_EE_SIG_L */
#define M_EEPROM_FLASH_EE_SIG_L                 (65535U << 0)                   /*!< R: Read the 16 MSB of the signature in test; W: Initialise the 16 MSB of the signature in test */

extern volatile uint16_t IO_EEPROM_FLASH_EE_SIG_H __attribute__((nodp, addr(0x001AA)));  /*!< IO_EEPROM_FLASH_EE_SIG_H */
#define M_EEPROM_FLASH_EE_SIG_H                 (65535U << 0)                   /*!< R: Read the 16 LSB of the signature in test; W: Initialise the 16 LSB of the signature in test */

/* ************ */
/* Block: COLIN */
/* ************ */
extern volatile uint16_t IO_COLIN_ADDR_PATCH0 __attribute__((nodp, addr(0x001AC)));  /*!< IO_COLIN_ADDR_PATCH0 */
#define B_COLIN_CTRL_PATCH0                     (1U << 12)                      /*!< RW: 0 : patch is not activated 1 : patch is activated */
#define M_COLIN_ADDR_PATCH0                     (4095U << 0)                    /*!< RW: Dedicated protocol handler ROM address to patch */

extern volatile uint16_t IO_COLIN_DATA_PATCH0 __attribute__((nodp, addr(0x001AE)));  /*!< IO_COLIN_DATA_PATCH0 */
#define M_COLIN_DATA_PATCH0                     (65535U << 0)                   /*!< RW: When patch is activated, this DATA replace the instruction coming from the Dedicated protocol handler ROM at address selected by ADDR */

extern volatile uint16_t IO_COLIN_ADDR_PATCH1 __attribute__((nodp, addr(0x001B0)));  /*!< IO_COLIN_ADDR_PATCH1 */
#define B_COLIN_CTRL_PATCH1                     (1U << 12)                      /*!< RW: 0 : patch is not activated 1 : patch is activated */
#define M_COLIN_ADDR_PATCH1                     (4095U << 0)                    /*!< RW: Dedicated protocol handler ROM address to patch */

extern volatile uint16_t IO_COLIN_DATA_PATCH1 __attribute__((nodp, addr(0x001B2)));  /*!< IO_COLIN_DATA_PATCH1 */
#define M_COLIN_DATA_PATCH1                     (65535U << 0)                   /*!< RW: When patch is activated, this DATA replace the instruction coming from the Dedicated protocol handler ROM at address selected by ADDR */

extern volatile uint16_t IO_COLIN_ADDR_PATCH2 __attribute__((nodp, addr(0x001B4)));  /*!< IO_COLIN_ADDR_PATCH2 */
#define B_COLIN_CTRL_PATCH2                     (1U << 12)                      /*!< RW: 0 : patch is not activated 1 : patch is activated */
#define M_COLIN_ADDR_PATCH2                     (4095U << 0)                    /*!< RW: Dedicated protocol handler ROM address to patch */

extern volatile uint16_t IO_COLIN_DATA_PATCH2 __attribute__((nodp, addr(0x001B6)));  /*!< IO_COLIN_DATA_PATCH2 */
#define M_COLIN_DATA_PATCH2                     (65535U << 0)                   /*!< RW: When patch is activated, this DATA replace the instruction coming from the Dedicated protocol handler ROM at address selected by ADDR */

extern volatile uint16_t IO_COLIN_ADDR_PATCH3 __attribute__((nodp, addr(0x001B8)));  /*!< IO_COLIN_ADDR_PATCH3 */
#define B_COLIN_CTRL_PATCH3                     (1U << 12)                      /*!< RW: 0 : patch is not activated 1 : patch is activated */
#define M_COLIN_ADDR_PATCH3                     (4095U << 0)                    /*!< RW: Dedicated protocol handler ROM address to patch */

extern volatile uint16_t IO_COLIN_DATA_PATCH3 __attribute__((nodp, addr(0x001BA)));  /*!< IO_COLIN_DATA_PATCH3 */
#define M_COLIN_DATA_PATCH3                     (65535U << 0)                   /*!< RW: When patch is activated, this DATA replace the instruction coming from the Dedicated protocol handler ROM at address selected by ADDR */

extern volatile uint8_t IO_COLIN_CFG __attribute__((nodp, addr(0x001BC)));      /*!< IO_COLIN_CFG */
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

extern volatile uint8_t IO_COLIN_STS __attribute__((nodp, addr(0x001BD)));      /*!< IO_COLIN_STS */
#define B_COLIN_EVENT                           (1U << 3)                       /*!< R: Master event; W: Writing "0" will clear the bit. Writing "1" will let the bit unchanged ("0" or "1"). */
#define B_COLIN_HANDSHAKE                       (1U << 2)                       /*!< R: Master handshake; W: Writing "0" will clear the bit. Writing "1" will let the bit unchanged ("0" or "1"). */
#define B_COLIN_ERROR                           (1U << 1)                       /*!< R: Master error; W: Writing "0" will clear the bit. Writing "1" will let the bit unchanged ("0" or "1"). */
#define B_COLIN_SIGNAL                          (1U << 0)                       /*!< R: Master signal; W: Writing "0" will clear the bit. Writing "1" will let the bit unchanged ("0" or "1"). */

extern volatile uint16_t IO_COLIN_CMD_SLVIT __attribute__((nodp, addr(0x001BE)));  /*!< IO_COLIN_CMD_SLVIT */
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

extern volatile uint16_t IO_COLIN_ROM_RAM __attribute__((nodp, addr(0x001C0)));  /*!< IO_COLIN_ROM_RAM */
#define B_COLIN_ROM_SETUP                       (1U << 12)                      /*!< RW: Select dedicated protocol ROM address setup time 0 : 1 period of main clock 1 : 2 periods of main clock */
#define M_COLIN_ROM_ACCESS                      (7U << 8)                       /*!< RW: Select dedicated protocol ROM access time 000 : 1 period of main clock 001 : 2 periods of main clock 010 : 3 periods of main clock 011 : 4 periods of main clock 100 : 5 periods of main clock 101 : 6 periods of main clock 110 : 7 periods of main clock 111 : 8 periods of main clock */
#define B_COLIN_RAM_SETUP                       (1U << 4)                       /*!< RW: Select dedicated protocol RAM address setup time 0 : 1 period of main clock 1 : 2 periods of main clock */
#define M_COLIN_RAM_ACCESS                      (7U << 0)                       /*!< RW: Select dedicated protocol RAM access time 000 : 1 period of main clock 001 : 2 periods of main clock 010 : 3 periods of main clock 011 : 4 periods of main clock 100 : 5 periods of main clock 101 : 6 periods of main clock 110 : 7 periods of main clock 111 : 8 periods of main clock */

extern volatile uint16_t IO_COLIN_LPWR_ROM_RAM __attribute__((nodp, addr(0x001C2)));  /*!< IO_COLIN_LPWR_ROM_RAM */
#define B_COLIN_DISABLE_RAM                     (1U << 8)                       /*!< RW: If this port field is set to one, then the RAM is forced In a low power consumption mode */
#define B_COLIN_DISABLE_ROM                     (1U << 0)                       /*!< RW: If this port field is set to one, then the ROM is forced In a low power consumption mode */

#ifdef __MLX81330A01__
/* ******************** */
/* Block: PORT_LIN_XKEY */
/* ******************** */
extern volatile uint16_t IO_PORT_LIN_XKEY_S __attribute__((nodp, addr(0x001C4)));  /*!< IO_PORT_LIN_XKEY */
#define M_PORT_LIN_XKEY_LIN_XKEY                (65535U << 0)                   /*!< RW: store a valid key for LIN XCFG */

/* ******************* */
/* Block: TRIM_BG_BIAS */
/* ******************* */
extern volatile uint16_t IO_TRIM_BG_BIAS __attribute__((nodp, addr(0x001C6)));  /*!< IO_TRIM_BG_BIAS */
#define B_TRIM_BG_BIAS_LOCK                     (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define B_TRIM_BG_BIAS_TR_UNUSED                (1U << 14)                      /*!< RW: free for spare use */
#define M_TRIM_BG_BIAS_PRE_TR_BIAS              (63U << 8)                      /*!< RW: current bias source trimming */
#define M_TRIM_BG_BIAS_PRE_TR_BGD               (15U << 4)                      /*!< RW: digital bandgap trimming */
#define M_TRIM_BG_BIAS_PRE_TR_BGA               (15U << 0)                      /*!< RW: analogue bandgap trimming */

/* *************** */
/* Block: TRIM_VDD */
/* *************** */
extern volatile uint16_t IO_TRIM_VDD __attribute__((nodp, addr(0x001C8)));      /*!< IO_TRIM_VDD */
#define B_TRIM_VDD_LOCK                         (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define B_TRIM_VDD_TR_UNUSED                    (1U << 14)                      /*!< RW: free for spare use */
#define M_TRIM_VDD_PRE_TR_SUP                   (255U << 6)                     /*!< RW: spare control output bits for supply system */
#define M_TRIM_VDD_PRE_TR_VDDD                  (7U << 3)                       /*!< RW: digital bandgap trimming */
#define M_TRIM_VDD_PRE_TR_VDDA                  (7U << 0)                       /*!< RW: analogue bandgap trimming */

/* ****************** */
/* Block: TRIM_RCO32M */
/* ****************** */
extern volatile uint16_t IO_TRIM_RCO32M __attribute__((nodp, addr(0x001CA)));   /*!< IO_TRIM_RCO32M */
#define B_TRIM_RCO32M_LOCK                      (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_TRIM_RCO32M_TR_UNUSED                 (31U << 10)                     /*!< RW: free for spare use */
#define M_TRIM_RCO32M_TR_RCO32M_IN              (1023U << 0)                    /*!< RW: 32 MHz RC oscillator trimming */

/* ***************** */
/* Block: TRIM_RCO1M */
/* ***************** */
extern volatile uint16_t IO_TRIM_RCO1M __attribute__((nodp, addr(0x001CC)));    /*!< IO_TRIM_RCO1M */
#define B_TRIM_RCO1M_LOCK                       (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define B_TRIM_RCO1M_TR_UNUSED                  (1U << 14)                      /*!< RW: free for spare use */
#define M_TRIM_RCO1M_PRE_TR_LIN_SLVTERM         (7U << 11)                      /*!< RW: slave termination trimming */
#define M_TRIM_RCO1M_PRE_TR_LIN_SLEWRATE        (7U << 8)                       /*!< RW: slew rate trimming */
#define M_TRIM_RCO1M_PRE_TR_RCO1M               (255U << 0)                     /*!< RW: 1 MHz RC oscillator trimming */

/* ********************* */
/* Block: PORT_SSCM_CONF */
/* ********************* */
extern volatile uint16_t IO_PORT_SSCM_CONF __attribute__((nodp, addr(0x001CE)));  /*!< IO_PORT_SSCM_CONF */
#define B_PORT_SSCM_CONF_SSCM_SINGLEBIT         (1U << 1)                       /*!< RW: output shall provide a code with Hamming distance of 1 instead of triangle */
#define B_PORT_SSCM_CONF_SSCM_EN                (1U << 0)                       /*!< RW: enable the spread spectrum modulation */

/* ********************* */
/* Block: PORT_STEP_CONF */
/* ********************* */
extern volatile uint16_t IO_PORT_STEP_CONF __attribute__((nodp, addr(0x001D0)));  /*!< IO_PORT_STEP_CONF */
#define M_PORT_STEP_CONF_STEP_CNT               (255U << 8)                     /*!< RW: step count per period of triangular modulation for Spread Spectrum */
#define M_PORT_STEP_CONF_STEP_DUR               (15U << 4)                      /*!< RW: step duration in main clock pulses for Spread Spectrum */
#define M_PORT_STEP_CONF_STEP_INC               (15U << 0)                      /*!< RW: step increment for Spread Spectrum */

/* ********************* */
/* Block: PORT_SUPP_TEST */
/* ********************* */
extern volatile uint16_t IO_PORT_SUPP_TEST __attribute__((nodp, addr(0x001D2)));  /*!< IO_PORT_SUPP_TEST */
#define B_PORT_SUPP_TEST_SHOVE_VAUX             (1U << 14)                      /*!< RW: VAUX increased by 20% for Shove */
#define B_PORT_SUPP_TEST_SHOVE_VDDA             (1U << 13)                      /*!< RW: VDDA increased by 20% for Shove */
#define B_PORT_SUPP_TEST_SHOVE_VDDD             (1U << 12)                      /*!< RW: VDDD set to 2.4V */
#define B_PORT_SUPP_TEST_LOW_VDDD               (1U << 11)                      /*!< RW: VDDD set to 1.4V */
#define B_PORT_SUPP_TEST_SBY_BIAS               (1U << 10)                      /*!< RW: disable current bias source */
#define B_PORT_SUPP_TEST_SWITCHOFFUV_VDDD_RES   (1U << 9)                       /*!< RW: switch off resistor divider of HPORB for IDDQ test, must be additional to DIS_HPORB */
#define M_PORT_SUPP_TEST_SWITCHOFFREG_VDDD      (3U << 7)                       /*!< RW: disable VDDD regulator to drive from extern */
#define M_PORT_SUPP_TEST_SWITCHOFFREG_VDDA      (3U << 5)                       /*!< RW: disable VDDA regulator to drive from extern */
#define B_PORT_SUPP_TEST_PORTEST                (1U << 4)                       /*!< RW: signal to disable reset for level shifters inside analogue, shall be set before digital IDDQ test, needs to be set together with DIS_HPORB in digital test controller */
#define B_PORT_SUPP_TEST_TEST_5U_BIASAUX        (1U << 3)                       /*!< RW: switch 5u VAUX related BIAS to analogue test bus */
#define B_PORT_SUPP_TEST_TEST_10U_BIAS          (1U << 2)                       /*!< RW: switch 10u VDDA related BIAS to analogue test bus */
#define B_PORT_SUPP_TEST_TEST_VAUX_ADDCURRENT   (1U << 1)                       /*!< RW: load VAUX with add current 20uA, together with TEST_VAUX */
#define B_PORT_SUPP_TEST_TEST_BGA               (1U << 0)                       /*!< RW: switch VBGA to analogue test bus */

/* ********************** */
/* Block: PORT_SUPP2_TEST */
/* ********************** */
extern volatile uint16_t IO_PORT_SUPP2_TEST __attribute__((nodp, addr(0x001D4)));  /*!< IO_PORT_SUPP2_TEST */
#define B_PORT_SUPP2_TEST_FSTOP                 (1U << 13)                      /*!< RW: set SETP_STOP for stop mode test, no output, will be used to generate SET_STOP on digital only */
#define B_PORT_SUPP2_TEST_FGTSM                 (1U << 12)                      /*!< RW: set GTSM for wake up test, no output, will be used to generate GTSM on digital only */
#define B_PORT_SUPP2_TEST_TST_IO_LV             (1U << 11)                      /*!< RW: Value for analogue */
#define B_PORT_SUPP2_TEST_TST_VDDD_OUT2_FV      (1U << 10)                      /*!< RW: VDDD test switch */
#define B_PORT_SUPP2_TEST_TST_VDDA_OUT2_FV      (1U << 9)                       /*!< RW: VDDA test switch */
#define B_PORT_SUPP2_TEST_IDDQ_CLK10K           (1U << 8)                       /*!< RW: switch off 10 kHz clock for IDDQ test */
#define B_PORT_SUPP2_TEST_IDDQ_TEMPSENSE        (1U << 7)                       /*!< RW: switch off current flowing into temperature sensor */
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
extern volatile uint16_t IO_PORT_LIN_TEST __attribute__((nodp, addr(0x001D6)));  /*!< IO_PORT_LIN_TEST */
#define B_PORT_LIN_TEST_TEST_2U8_BIASLIN        (1U << 0)                       /*!< RW: switches 2.8uA low side bias current from LIN cell to analogue test bus */

/* ********************** */
/* Block: PORT_SPARE_TEST */
/* ********************** */
extern volatile uint16_t IO_PORT_SPARE_TEST __attribute__((nodp, addr(0x001D8)));  /*!< IO_PORT_SPARE_TEST */
#define M_PORT_SPARE_TEST_TEST_SUP              (7U << 7)                       /*!< RW: spare test control bits for supply system */
#define M_PORT_SPARE_TEST_TST_HS_SHORT_DET      (3U << 5)                       /*!< RW: Value for analogue */
#define M_PORT_SPARE_TEST_CPCLKSEL_DRV          (3U << 3)                       /*!< Value for analogue */
#define B_PORT_SPARE_TEST_TST_VDDA_VT_MON_SUP   (1U << 2)                       /*!< Value for analogue */
#define B_PORT_SPARE_TEST_RST_IOX_CONN2PHV      (1U << 1)                       /*!< Value for analogue */
#define B_PORT_SPARE_TEST_SET_IOX_CONN2PHV      (1U << 0)                       /*!< Value for analogue */

/* ************************* */
/* Block: PORT_ADC_TRIM_TEST */
/* ************************* */
extern volatile uint16_t IO_PORT_ADC_TRIM_TEST __attribute__((nodp, addr(0x001DA)));  /*!< IO_PORT_ADC_TRIM_TEST */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_RCO1M       (1U << 9)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_RCO1M pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_RCO32M      (1U << 8)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_RCO32M pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_LIN_SLVTERM (1U << 7)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_LIN_SLVTERM pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_LIN_SLEWRATE (1U << 6)                      /*!< RW: if 1: connect ADC_TRIMPORT to TR_LIN_SLEWRATE pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_SUP         (1U << 5)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_SUP pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_BIAS        (1U << 4)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_BIAS pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_VDDD        (1U << 3)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_VDDD pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_VDDA        (1U << 2)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_VDDA pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_BGD         (1U << 1)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_BGD pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_BGA         (1U << 0)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_BGA pins, else connect from port */

/* ***************** */
/* Block: PORT_IO_IN */
/* ***************** */
extern volatile uint16_t IO_PORT_IO_IN __attribute__((nodp, addr(0x001DC)));    /*!< IO_PORT_IO_IN */
#define M_PORT_IO_IN_IO_IN_SYNC                 (15U << 0)                      /*!< R: From Schmitt trigger I/O (after re-synchronisation) */
#define B_PORT_IO_IN_IO_IN_SYNC_3               (1U << 3)                       /*!< R: From Schmitt trigger IO3 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_2               (1U << 2)                       /*!< R: From Schmitt trigger IO2 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_1               (1U << 1)                       /*!< R: From Schmitt trigger IO1 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_0               (1U << 0)                       /*!< R: From Schmitt trigger IO0 (after resynchro) */

/* ******************* */
/* Block: PORT_SUPP_IN */
/* ******************* */
extern volatile uint16_t IO_PORT_SUPP_IN __attribute__((nodp, addr(0x001DE)));  /*!< IO_PORT_SUPP_IN */
#define B_PORT_SUPP_IN_UV_VDDAF_SYNC            (1U << 11)                      /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_UV_VDDAF_IT              (1U << 10)                      /*!< R: Driver supply under voltage detection */
#define B_PORT_SUPP_IN_OVC_SYNC                 (1U << 9)                       /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_OVC_IT                   (1U << 8)                       /*!< R: Over current detection */
#define B_PORT_SUPP_IN_OVT_SYNC                 (1U << 7)                       /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_OVT_IT                   (1U << 6)                       /*!< R: Over temperature interrupt flag */
#define B_PORT_SUPP_IN_OV_VS_SYNC               (1U << 5)                       /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_OV_VS_IT                 (1U << 4)                       /*!< R: Over voltage at VS */
#define B_PORT_SUPP_IN_UV_VS_SYNC               (1U << 3)                       /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_UV_VS_IT                 (1U << 2)                       /*!< R: Under voltage at VS */
#define B_PORT_SUPP_IN_UV_VDDA_SYNC             (1U << 1)                       /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_UV_VDDA_IT               (1U << 0)                       /*!< R: Under voltage condition at VDDA */

/* ******************* */
/* Block: PORT_MISC_IN */
/* ******************* */
extern volatile uint16_t IO_PORT_MISC_IN __attribute__((nodp, addr(0x001E0)));  /*!< IO_PORT_MISC_IN */
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
extern volatile uint16_t IO_PORT_SUPP_CFG __attribute__((nodp, addr(0x001E2)));  /*!< IO_PORT_SUPP_CFG */
#define B_PORT_SUPP_CFG_OVT_FILT_SEL            (1U << 5)                       /*!< RW: 0 : 1-2us filtering, 1 : 100-200us filtering */
#define B_PORT_SUPP_CFG_OVC_FILT_SEL            (1U << 4)                       /*!< RW: 0 : 1-2us filtering, 1 : 100-200us filtering */
#define B_PORT_SUPP_CFG_OV_VS_FILT_SEL          (1U << 3)                       /*!< RW: 0 : 1-2us filtering, 1 : 100-200us filtering */
#define B_PORT_SUPP_CFG_UV_VS_FILT_SEL          (1U << 2)                       /*!< RW: 0 : 1-2us filtering, 1 : 100-200us filtering */
#define B_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL       (1U << 1)                       /*!< RW: 0 : 1-2us filtering, 1 : 100-200us filtering */
#define B_PORT_SUPP_CFG_UV_VDDA_FILT_SEL        (1U << 0)                       /*!< RW: 0 : 1-2us filtering, 1 : 100-200us filtering */

/* *********************** */
/* Block: PORT_IO_OUT_SOFT */
/* *********************** */
extern volatile uint16_t IO_PORT_IO_OUT_SOFT __attribute__((nodp, addr(0x001E4)));  /*!< IO_PORT_IO_OUT_SOFT */
#define M_PORT_IO_OUT_SOFT_IO_OUT_SOFT          (15U << 0)                      /*!< RW: To GPIO glue logic control */
#define C_PORT_IO_OUT_SOFT_IO3_OUT              (1U << 3)                       /*!< IO[3] output */
#define C_PORT_IO_OUT_SOFT_IO2_OUT              (1U << 2)                       /*!< IO[2] output */
#define C_PORT_IO_OUT_SOFT_IO1_OUT              (1U << 1)                       /*!< IO[1] output */
#define C_PORT_IO_OUT_SOFT_IO0_OUT              (1U << 0)                       /*!< IO[0] output */

/* ********************* */
/* Block: PORT_IO_OUT_EN */
/* ********************* */
extern volatile uint16_t IO_PORT_IO_OUT_EN __attribute__((nodp, addr(0x001E6)));  /*!< IO_PORT_IO_OUT_EN */
#define B_PORT_IO_OUT_EN_IOX_OD_ENABLE          (1U << 9)                       /*!< RW: Value for analogue */
#define B_PORT_IO_OUT_EN_IO0_OD_ENABLE          (1U << 8)                       /*!< RW: Value for analogue */
#define B_PORT_IO_OUT_EN_IO0_LV_ENABLE          (1U << 7)                       /*!< RW: Value for analogue */
#define M_PORT_IO_OUT_EN_IO_CH_SEL              (7U << 4)                       /*!< RW: Value for analogue */
#define M_PORT_IO_OUT_EN_IO_EN                  (15U << 0)                      /*!< RW: Set the state of the IO interface */
#define C_PORT_IO_OUT_EN_IO3_EN                 (1U << 3)                       /*!< IO[3] enable */
#define C_PORT_IO_OUT_EN_IO2_EN                 (1U << 2)                       /*!< IO[2] enable */
#define C_PORT_IO_OUT_EN_IO1_EN                 (1U << 1)                       /*!< IO[1] enable */
#define C_PORT_IO_OUT_EN_IO0_EN                 (1U << 0)                       /*!< IO[0] enable */

/* ******************* */
/* Block: PORT_IO_CFG0 */
/* ******************* */
extern volatile uint16_t IO_PORT_IO_CFG0 __attribute__((nodp, addr(0x001E8)));  /*!< IO_PORT_IO_CFG0 */
#define M_PORT_IO_CFG0_IO3_OUT_SEL              (15U << 12)                     /*!< RW: Output selection for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PWM_MSTR1    (0U << 12)                      /*!< RW: Output selection PWM_MASTER1 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PWM_SLV1     (1U << 12)                      /*!< RW: Output selection PWM_SLAVE1 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PWM_SLV2     (2U << 12)                      /*!< RW: Output selection PWM_SLAVE2 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PWM_SLV3     (3U << 12)                      /*!< RW: Output selection PWM_SLAVE3 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PWM_MSTR2    (4U << 12)                      /*!< RW: Output selection PWM_MASTER2 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_CTIMER0      (5U << 12)                      /*!< RW: Output selection CTIMER0 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_CTIMER1      (6U << 12)                      /*!< RW: Output selection CTIMER1 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_SOFT         (7U << 12)                      /*!< RW: Output selection SOFTWARE for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_LIN          (8U << 12)                      /*!< RW: Output selection LIN for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_COLIN_TX     (9U << 12)                      /*!< RW: Output selection COLIN_TX for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_MOSI     (10U << 12)                     /*!< RW: Output selection SPI_MOSI for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_MISO     (11U << 12)                     /*!< RW: Output selection SPI_MISO for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_SCK      (12U << 12)                     /*!< RW: Output selection SPI_SCK for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_SS       (13U << 12)                     /*!< RW: Output selection SPI_SS for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PPM          (14U << 12)                     /*!< RW: Output selection PPM for IO[3] */
#define M_PORT_IO_CFG0_IO2_OUT_SEL              (15U << 8)                      /*!< RW: Output selection for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PWM_MSTR1    (0U << 8)                       /*!< RW: Output selection PWM_MASTER1 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PWM_SLV1     (1U << 8)                       /*!< RW: Output selection PWM_SLAVE1 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PWM_SLV2     (2U << 8)                       /*!< RW: Output selection PWM_SLAVE2 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PWM_SLV3     (3U << 8)                       /*!< RW: Output selection PWM_SLAVE3 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PWM_MSTR2    (4U << 8)                       /*!< RW: Output selection PWM_MASTER2 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_CTIMER0      (5U << 8)                       /*!< RW: Output selection CTIMER0 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_CTIMER1      (6U << 8)                       /*!< RW: Output selection CTIMER1 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_SOFT         (7U << 8)                       /*!< RW: Output selection SOFTWARE for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_LIN          (8U << 8)                       /*!< RW: Output selection LIN for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_COLIN_TX     (9U << 8)                       /*!< RW: Output selection COLIN_TX for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_MOSI     (10U << 8)                      /*!< RW: Output selection SPI_MOSI for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_MISO     (11U << 8)                      /*!< RW: Output selection SPI_MISO for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_SCK      (12U << 8)                      /*!< RW: Output selection SPI_SCK for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_SS       (13U << 8)                      /*!< RW: Output selection SPI_SS for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PPM          (14U << 8)                      /*!< RW: Output selection PPM for IO[2] */
#define M_PORT_IO_CFG0_IO1_OUT_SEL              (15U << 4)                      /*!< RW: Output selection for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PWM_MSTR1    (0U << 4)                       /*!< RW: Output selection PWM_MASTER1 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PWM_SLV1     (1U << 4)                       /*!< RW: Output selection PWM_SLAVE1 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PWM_SLV2     (2U << 4)                       /*!< RW: Output selection PWM_SLAVE2 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PWM_SLV3     (3U << 4)                       /*!< RW: Output selection PWM_SLAVE3 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PWM_MSTR2    (4U << 4)                       /*!< RW: Output selection PWM_MASTER2 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_CTIMER0      (5U << 4)                       /*!< RW: Output selection CTIMER0 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_CTIMER1      (6U << 4)                       /*!< RW: Output selection CTIMER1 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_SOFT         (7U << 4)                       /*!< RW: Output selection SOFTWARE for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_LIN          (8U << 4)                       /*!< RW: Output selection LIN for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_COLIN_TX     (9U << 4)                       /*!< RW: Output selection COLIN_TX for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_MOSI     (10U << 4)                      /*!< RW: Output selection SPI_MOSI for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_MISO     (11U << 4)                      /*!< RW: Output selection SPI_MISO for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_SCK      (12U << 4)                      /*!< RW: Output selection SPI_SCK for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_SS       (13U << 4)                      /*!< RW: Output selection SPI_SS for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PPM          (14U << 4)                      /*!< RW: Output selection PPM for IO[1] */
#define M_PORT_IO_CFG0_IO0_OUT_SEL              (15U << 0)                      /*!< RW: Output selection for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PWM_MSTR1    (0U << 0)                       /*!< RW: Output selection PWM_MASTER1 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PWM_SLV1     (1U << 0)                       /*!< RW: Output selection PWM_SLAVE1 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PWM_SLV2     (2U << 0)                       /*!< RW: Output selection PWM_SLAVE2 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PWM_SLV3     (3U << 0)                       /*!< RW: Output selection PWM_SLAVE3 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PWM_MSTR2    (4U << 0)                       /*!< RW: Output selection PWM_MASTER2 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_CTIMER0      (5U << 0)                       /*!< RW: Output selection CTIMER0 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_CTIMER1      (6U << 0)                       /*!< RW: Output selection CTIMER1 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_SOFT         (7U << 0)                       /*!< RW: Output selection SOFTWARE for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_LIN          (8U << 0)                       /*!< RW: Output selection LIN for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_COLIN_TX     (9U << 0)                       /*!< RW: Output selection COLIN_TX for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_MOSI     (10U << 0)                      /*!< RW: Output selection SPI_MOSI for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_MISO     (11U << 0)                      /*!< RW: Output selection SPI_MISO for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_SCK      (12U << 0)                      /*!< RW: Output selection SPI_SCK for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_SS       (13U << 0)                      /*!< RW: Output selection SPI_SS for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PPM          (14U << 0)                      /*!< RW: Output selection PPM for IO[0] */

/* *********************** */
/* Block: PORT_LIN_XTX_CFG */
/* *********************** */
extern volatile uint16_t IO_PORT_LIN_XTX_CFG __attribute__((nodp, addr(0x001EA)));  /*!< IO_PORT_LIN_XTX_CFG */
#define B_PORT_LIN_XTX_CFG_LIN_IN_SOFT          (1U << 7)                       /*!< RW: Software port for LIN reception (connected via mux to LIN_XTX) */
#define B_PORT_LIN_XTX_CFG_LIN_OUT_SOFT         (1U << 6)                       /*!< RW: Software port for LIN transmission (connected via mux to LIN_XTX) */
#define M_PORT_LIN_XTX_CFG_LIN_COLIN_RX_SEL     (3U << 4)                       /*!< RW: Input selection for Colin RX input */
#define C_PORT_LIN_XTX_CFG_LIN_COLIN_RX_SEL_IO0 (0U << 4)                       /*!< RW: Input selection IO0 for Colin RX input */
#define C_PORT_LIN_XTX_CFG_LIN_COLIN_RX_SEL_IO1 (1U << 4)                       /*!< RW: Input selection IO1 for Colin RX input */
#define C_PORT_LIN_XTX_CFG_LIN_COLIN_RX_SEL_IO2 (2U << 4)                       /*!< RW: Input selection IO2 for Colin RX input */
#define C_PORT_LIN_XTX_CFG_LIN_COLIN_RX_SEL_IO3 (3U << 4)                       /*!< RW: Input selection IO3 for Colin RX input */
#define M_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL      (15U << 0)                      /*!< RW: Output selection for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_CTIMER0      (0U << 0)               /*!< RW: Output selection CTIMRE0 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_CTIMER1      (1U << 0)               /*!< RW: Output selection CTIMRE1 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PWM_MASTER1  (2U << 0)               /*!< RW: Output selection PWM_MASTER1 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PWM_SLAVE1   (3U << 0)               /*!< RW: Output selection PWM_SLAVE1 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PWM_SLAVE2   (4U << 0)               /*!< RW: Output selection PWM_SLAVE2 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PWM_SLAVE3   (5U << 0)               /*!< RW: Output selection PWM_SLAVE3 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PWM_MASTER2  (6U << 0)               /*!< RW: Output selection PWM_MASTER2 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_LIN_OUT_SOFT (7U << 0)               /*!< RW: Output selection SOFTWARE for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO0          (8U << 0)               /*!< RW: Output selection IO0 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO1          (9U << 0)               /*!< RW: Output selection IO1 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO2          (10U << 0)              /*!< RW: Output selection IO2 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO3          (11U << 0)              /*!< RW: Output selection IO3 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PPM_OUT      (12U << 0)              /*!< RW: Output selection PPM for LIN_XTX */

/* ********************* */
/* Block: PORT_TIMER_CFG */
/* ********************* */
extern volatile uint16_t IO_PORT_TIMER_CFG __attribute__((nodp, addr(0x001EC)));
#define M_PORT_TIMER_CFG_TIMER1_CHB_SEL         (15U << 12)                     /* RW: Input selection for timer[1] channel B */
#define C_PORT_TIMER_CFG_TIMER1_CHB_SEL_IO0     (0U << 12)
#define C_PORT_TIMER_CFG_TIMER1_CHB_SEL_IO1     (1U << 12)
#define C_PORT_TIMER_CFG_TIMER1_CHB_SEL_IO2     (2U << 12)
#define C_PORT_TIMER_CFG_TIMER1_CHB_SEL_IO3     (3U << 12)
#define C_PORT_TIMER_CFG_TIMER1_CHB_SEL_LIN_XRX (4U << 12)
#define C_PORT_TIMER_CFG_TIMER1_CHB_SEL_PWM_MASTER1 (5U << 12)
#define C_PORT_TIMER_CFG_TIMER1_CHB_SEL_PWM_SLAVE1  (6U << 12)
#define C_PORT_TIMER_CFG_TIMER1_CHB_SEL_PWM_SLAVE2  (7U << 12)
#define C_PORT_TIMER_CFG_TIMER1_CHB_SEL_PWM_SLAVE3  (8U << 12)
#define C_PORT_TIMER_CFG_TIMER1_CHB_SEL_PWM_MASTER2 (9U << 12)
#define M_PORT_TIMER_CFG_TIMER1_CHA_SEL         (15U << 8)                      /* RW: Input selection for timer[1] channel A */
#define C_PORT_TIMER_CFG_TIMER1_CHA_SEL_IO0     (0U << 8)
#define C_PORT_TIMER_CFG_TIMER1_CHA_SEL_IO1     (1U << 8)
#define C_PORT_TIMER_CFG_TIMER1_CHA_SEL_IO2     (2U << 8)
#define C_PORT_TIMER_CFG_TIMER1_CHA_SEL_IO3     (3U << 8)
#define C_PORT_TIMER_CFG_TIMER1_CHA_SEL_LIN_XRX (4U << 8)
#define C_PORT_TIMER_CFG_TIMER1_CHA_SEL_PWM_MASTER1 (5U << 8)
#define C_PORT_TIMER_CFG_TIMER1_CHA_SEL_PWM_SLAVE1  (6U << 8)
#define C_PORT_TIMER_CFG_TIMER1_CHA_SEL_PWM_SLAVE2  (7U << 8)
#define C_PORT_TIMER_CFG_TIMER1_CHA_SEL_PWM_SLAVE3  (8U << 8)
#define C_PORT_TIMER_CFG_TIMER1_CHA_SEL_PWM_MASTER2 (9U << 8)
#define M_PORT_TIMER_CFG_TIMER0_CHB_SEL         (15U << 4)                      /* RW: Input selection for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO0     (0U << 4)
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO1     (1U << 4)
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO2     (2U << 4)
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO3     (3U << 4)
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_LIN_XRX (4U << 4)
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PWM_MASTER1 (5U << 4)
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PWM_SLAVE1  (6U << 4)
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PWM_SLAVE2  (7U << 4)
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PWM_SLAVE3  (8U << 4)
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PWM_MASTER2 (9U << 4)
#define M_PORT_TIMER_CFG_TIMER0_CHA_SEL         (15U << 0)                      /* RW: Input selection for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO0     (0U << 0)
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO1     (1U << 0)
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO2     (2U << 0)
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO3     (3U << 0)
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_LIN_XRX (4U << 0)
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PWM_MASTER1 (5U << 0)
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PWM_SLAVE1  (6U << 0)
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PWM_SLAVE2  (7U << 0)
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PWM_SLAVE3  (8U << 0)
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PWM_MASTER2 (9U << 0)

/* ******************** */
/* Block: PORT_COMM_CFG */
/* ******************** */
extern volatile uint16_t IO_PORT_COMM_CFG __attribute__((nodp, addr(0x001EE)));  /*!< IO_PORT_COMM_CFG */
#define M_PORT_COMM_CFG_SPI_SS_IN_SEL           (3U << 6)                       /*!< RW: IO selection for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO0       (0U << 6)                       /*!< RW: IO[0] selection for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO1       (1U << 6)                       /*!< RW: IO[1] selection for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO2       (2U << 6)                       /*!< RW: IO[2] selection for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO3       (3U << 6)                       /*!< RW: IO[3] selection for SPI_SS_IN */
#define M_PORT_COMM_CFG_SPI_SCK_IN_SEL          (3U << 4)                       /*!< RW: IO selection for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO0      (0U << 4)                       /*!< RW: IO[0] selection for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO1      (1U << 4)                       /*!< RW: IO[1] selection for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO2      (2U << 4)                       /*!< RW: IO[2] selection for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO3      (3U << 4)                       /*!< RW: IO[3] selection for SPI_SCK_IN */
#define M_PORT_COMM_CFG_SPI_MISO_IN_SEL         (3U << 2)                       /*!< RW: IO selection for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO0     (0U << 2)                       /*!< RW: IO[0] selection for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO1     (1U << 2)                       /*!< RW: IO[1] selection for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO2     (2U << 2)                       /*!< RW: IO[2] selection for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO3     (3U << 2)                       /*!< RW: IO[3] selection for SPI_MISO_IN */
#define M_PORT_COMM_CFG_SPI_MOSI_IN_SEL         (3U << 0)                       /*!< RW: IO selection for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO0     (0U << 0)                       /*!< RW: IO[0] selection for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO1     (1U << 0)                       /*!< RW: IO[1] selection for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO2     (2U << 0)                       /*!< RW: IO[2] selection for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO3     (3U << 0)                       /*!< RW: IO[3] selection for SPI_MOSI_IN */

/* ******************** */
/* Block: PORT_MISC_OUT */
/* ******************** */
extern volatile uint16_t IO_PORT_MISC_OUT __attribute__((nodp, addr(0x001F0)));  /*!< IO_PORT_MISC_OUT */
#define M_PORT_MISC_OUT_SEL_TEMP                (15U << 12)                     /*!< RW: select the temperature diode input for the temp. sensor */
#define C_PORT_MISC_OUT_SEL_TEMP_MAIN           (8U << 12)                      /*!< Channel 8: VBGA Temperature sensor */
#define B_PORT_MISC_OUT_PRUV_VDDA               (1U << 11)                      /*!< RW: under voltage programming for VDDA */
#define M_PORT_MISC_OUT_PROV_VS                 (3U << 9)                       /*!< RW: control for VS over voltage monitor */
#define C_PORT_MISC_OUT_PROV_0                  (1U << 9)                       /*!< RW: control for VS over voltage unit */
#define M_PORT_MISC_OUT_PRUV_VS                 (7U << 6)                       /*!< RW: under voltage programming for VS; detection level (V) = (PRUV_VS+4) -> from 4 to 9V */
#define C_PORT_MISC_OUT_PRUV_0                  (1U << 6)                       /*!< RW: under voltage programming for VS-unit */
#define B_PORT_MISC_OUT_SWITCH_VDDA_TO_5V       (1U << 5)                       /*!< RW: switch VDDA supply to 5V */
#define M_PORT_MISC_OUT_WUI                     (3U << 3)                       /*!< RW: define internal wake up time delay in periodes of CK10K : */
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
extern volatile uint16_t IO_PORT_MISC2_OUT __attribute__((nodp, addr(0x001F2)));  /*!< IO_PORT_MISC2_OUT */
#define B_PORT_MISC2_OUT_VSM_FILT_ON            (1U << 10)                      /*!< RW: Value for analogue */
#define B_PORT_MISC2_OUT_ENABLE_OTD             (1U << 9)                       /*!< RW: Enable Over Temperature Detector */
#define B_PORT_MISC2_OUT_WU_IO_EN               (1U << 8)                       /*!< RW: wake up enable for IOs */
#define M_PORT_MISC2_OUT_AOUT_SUP               (15U << 4)                      /*!< RW: spare control output bits for supply system */
#define M_PORT_MISC2_OUT_UNUSED                 (3U << 2)                       /*!< RW: Value for analogue */
#define M_PORT_MISC2_OUT_ENABLE_OSD_DRV         (3U << 0)                       /*!< RW: Value for analogue */

/* *********************** */
/* Block: PORT_STOPMD_CTRL */
/* *********************** */
extern volatile uint16_t IO_PORT_STOPMD_CTRL_S __attribute__((nodp, addr(0x001F4)));  /*!< IO_PORT_STOPMD_CTRL_S (System) */
#define B_PORT_STOPMD_CTRL_SEL_STOP_MODE        (1U << 0)                       /*!< RW: CPU HALT will enter STOPMODE, not SLEEP */

/* ********************** */
/* Block: PORT_STOPMD_CFG */
/* ********************** */
extern volatile uint16_t IO_PORT_STOPMD_CFG __attribute__((nodp, addr(0x001F6)));  /*!< IO_PORT_STOPMD_CFG */
#define B_PORT_STOPMD_CFG_PRE_SBY_RCO1M         (1U << 4)                       /*!< RW: set 1 MHz Oscillator in standby mode */
#define B_PORT_STOPMD_CFG_PRE_SBY_RCO32M        (1U << 3)                       /*!< RW: set 32 MHz Oscillator in standby mode */
#define B_PORT_STOPMD_CFG_PRE_SWITCHOFFOV_VS    (1U << 2)                       /*!< RW: disable over voltage detection for VS */
#define B_PORT_STOPMD_CFG_PRE_SWITCHOFFUV_VS    (1U << 1)                       /*!< RW: disable under voltage detection for VS */
#define B_PORT_STOPMD_CFG_PRE_SWITCHOFFUV_VDDA  (1U << 0)                       /*!< RW: disable under voltage detection of VDDA regulator to allow low voltage test */

/* ******************** */
/* Block: PORT_DIS_GTSM */
/* ******************** */
extern volatile uint16_t IO_PORT_DIS_GTSM __attribute__((nodp, addr(0x001F8)));  /*!< IO_PORT_DIS_GTSM */
#define B_PORT_DIS_GTSM_DIS_GTSM                (1U << 0)                       /*!< RW: disable GTSM, if set, HALTED will not trigger a GTSM signal */

/* ******************** */
/* Block: PORT_LIN_XCFG */
/* ******************** */
extern volatile uint16_t IO_PORT_LIN_XCFG_S __attribute__((nodp, addr(0x001FA)));  /*!< IO_PORT_LIN_XCFG_S (System) */
#define M_PORT_LIN_XCFG_LIN_XCFG                (65535U << 0)                   /*!< RW: result taken in account only if XKEY is valid */
#define B_PORT_LIN_XCFG_LIN_TEST_WARM           (1U << 15)                      /*!< Enable test mode warm activation over LIN pin */
#define B_PORT_LIN_XCFG_DIS_TX_TIMEOUT          (1U << 14)                      /*!< Disable time-out protection (10-20ms) on TX output */
#define B_PORT_LIN_XCFG_CXPI_DIS_WU_DEB         (1U << 13)                      /*!< Decrease the debounce time of the wake-up comparator from 70us to 5.5us to support CXPI protocol */
#define B_PORT_LIN_XCFG_ENA_LIN_REV_PROT        (1U << 12)                      /*!< Disconnects the reverse polarity protection from internal LIN node, is needed to measure LIN level by ADC or to run fast protocol at 5V level (PPM, CXPI) */
#define B_PORT_LIN_XCFG_SEL_RXD_ATDI            (1U << 11)                      /*!< 1: the fast comparator used in test mode (ATDI) will be switched to the RX input (this allows protocol with higher baudrate, e.g. PPM, FASTLIN or CXPI) */
#define B_PORT_LIN_XCFG_RX_INVERT               (1U << 10)                      /*!< Invert the RX input before any multiplexing */
#define B_PORT_LIN_XCFG_DISTERM                 (1U << 9)                       /*!< Disable bus termination for auto-addressing */
#define B_PORT_LIN_XCFG_BYPASS                  (1U << 8)                       /*!< Bypass the receiver for high-speed mode */
#define B_PORT_LIN_XCFG_HSM                     (1U << 7)                       /*!< High-speed mode (slew rate disabled) */
#define B_PORT_LIN_XCFG_LSM                     (1U << 6)                       /*!< Low speed slope control */
#define B_PORT_LIN_XCFG_SLEEPB                  (1U << 5)                       /*!< Disable sleep mode */
#define B_PORT_LIN_XCFG_SEL_COLIN_B             (1U << 4)                       /*!< Select Colin-B */
#define B_PORT_LIN_XCFG_SEL_IO_TO_COLINRX       (1U << 3)                       /*!< Select COLIN_RX driven from IO */
#define B_PORT_LIN_XCFG_SEL_RX_IO               (1U << 2)                       /*!< Select RX driven from IO */
#define B_PORT_LIN_XCFG_TX_INVERT               (1U << 1)                       /*!< Invert TX output */
#define B_PORT_LIN_XCFG_SEL_TX_EXT              (1U << 0)                       /*!< Select TX driver from IO */

/* ************************** */
/* Block: PORT_LIN_XCFG_VALID */
/* ************************** */
extern volatile uint16_t IO_PORT_LIN_XCFG_VALID __attribute__((nodp, addr(0x001FC)));  /*!< IO_PORT_LIN_XCFG_VALID */
#define M_PORT_LIN_XCFG_VALID_LIN_XCFG_VALID    (65535U << 0)                   /*!< R: Value for digital read */

/* ********************** */
/* Block: PORT_CLOCK_CTRL */
/* ********************** */
extern volatile uint16_t IO_PORT_CLOCK_CTRL_S __attribute__((nodp, addr(0x001FE)));  /*!< IO_PORT_CLOCK_CTRL_S (System) */
#define B_PORT_CLOCK_CTRL_AC_SEL                (1U << 0)                       /*!< RW: switch to 16 MHz clock if set */

/* ****************** */
/* Block: PORT_LINAA1 */
/* ****************** */
extern volatile uint16_t IO_PORT_LINAA1 __attribute__((nodp, addr(0x00200)));   /*!< IO_PORT_LINAA1 */
#define B_PORT_LINAA1_LINAA_CDOUTEN             (1U << 12)                      /*!< RW: Switch of the monitor current for LED monitoring or LINAA */
#define B_PORT_LINAA1_LINAA_EN                  (1U << 11)                      /*!< RW: Enable OpAmp of the LINAA amplifier */
#define B_PORT_LINAA1_LINAA_RST2                (1U << 10)                      /*!< RW: Reset second variable gain amplifier */
#define B_PORT_LINAA1_LINAA_RST1                (1U << 9)                       /*!< RW: Reset first amplifier gain = 40 */
#define M_PORT_LINAA1_LINAA_DIV                 (31U << 4)                      /*!< RW: Common mode suppression adjustments bits */
#define M_PORT_LINAA1_LINAA_GAIN                (15U << 0)                      /*!< RW: Gain control bits of the variable gain amp */

/* ****************** */
/* Block: PORT_LINAA2 */
/* ****************** */
extern volatile uint16_t IO_PORT_LINAA2 __attribute__((nodp, addr(0x00202)));   /*!< IO_PORT_LINAA2 */
#define B_PORT_LINAA2_LCD_DIS_LINAA             (1U << 4)                       /*!< RW: Value for analogue */
#define B_PORT_LINAA2_LCD_ON_LINAA              (1U << 3)                       /*!< RW: Value for analogue */
#define M_PORT_LINAA2_LCD_SEL_LINAA             (7U << 0)                       /*!< RW: Value for analogue */

/* **************** */
/* Block: TRIM_MISC */
/* **************** */
extern volatile uint16_t IO_TRIM_MISC __attribute__((nodp, addr(0x00204)));     /*!< IO_TRIM_MISC */
#define B_TRIM_MISC_LOCK                        (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_TRIM_MISC_TRIM_SDAFILT_IO             (3U << 12)                      /*!< RW: Calibration. Write is invalid if LOCK is set */
#define M_TRIM_MISC_TRIM_OTD                    (63U << 6)                      /*!< RW: Calibration. Write is invalid if LOCK is set */
#define M_TRIM_MISC_TRIM_LCD_LINAA              (63U << 0)                      /*!< RW: Calibration. Write is invalid if LOCK is set */

/* **************** */
/* Block: TRIM1_DRV */
/* **************** */
extern volatile uint16_t IO_TRIM1_DRV __attribute__((nodp, addr(0x00206)));     /*!< IO_TRIM1_DRV */
#define B_TRIM1_DRV_LOCK                        (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK       (1023U << 2)                    /*!< RW: trim frequency of driver clock */
#define M_TRIM1_DRV_TRIM_DRVSUP                 (3U << 0)                       /*!< RW: trim output level of driver supply */

/* **************** */
/* Block: TRIM2_DRV */
/* **************** */
extern volatile uint16_t IO_TRIM2_DRV __attribute__((nodp, addr(0x00208)));     /*!< IO_TRIM2_DRV */
#define B_TRIM2_DRV_LOCK                        (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_TRIM2_DRV_TRIM_CSA_GAIN               (31U << 4)                      /*!< RW: trim gain of current sense amplifier */
#define M_TRIM2_DRV_TRIM_SLWRT                  (15U << 0)                      /*!< RW: trim slew/rate / slope of drivers */

/* **************** */
/* Block: TRIM3_DRV */
/* **************** */
extern volatile uint16_t IO_TRIM3_DRV __attribute__((nodp, addr(0x0020A)));     /*!< IO_TRIM3_DRV */
#define B_TRIM3_DRV_LOCK                        (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_TRIM3_DRV_TRIM_CSA_CL                 (255U << 0)                     /*!< RW: trim over-current limit of current sense amplifier */

/* ******************* */
/* Block: PORT_DRV_OUT */
/* ******************* */
extern volatile uint16_t IO_PORT_DRV_OUT __attribute__((nodp, addr(0x0020C)));  /*!< IO_PORT_DRV_OUT */
#define B_PORT_DRV_OUT_PARALLEL_MODE_DRV        (1U << 11)                      /*!< RW: Phase UV and WT switch parallel for cross-current detection (DC-mode) */
#define M_PORT_DRV_OUT_DRVMOD_OPTION            (3U << 9)                       /*!< RW: select divider ratio of driver clock
                                                                                 * 0b00: No division for CPCLK
                                                                                 * 0b01: CPCLK is constantly divided by 4
                                                                                 * 0b10: CPCLK starts at full speed, then is divided by 4, ~3us after enabling a HS FET
                                                                                 * 0b11: CPCLK starts at full speed, then is divided by 4, ~7us after enabling a HS FET */
#define B_PORT_DRV_OUT_ENABLE_LS_OC             (1U << 8)                       /*!< RW: enable low-side FET VDS over-voltage / over-current detection */
#define B_PORT_DRV_OUT_ENABLE_HS_OC             (1U << 7)                       /*!< RW: enable high-side FET VDS over-voltage / over-current detection */
#define B_PORT_DRV_OUT_ENABLE_CSA               (1U << 6)                       /*!< RW: enable current sense amplifier */
#define M_PORT_DRV_OUT_ENABLE_DRV               (15U << 2)                      /*!< RW: enable output stages */
#define B_PORT_DRV_OUT_ENABLE_DRV0              ((1U << 0) << 2)                /*!< RW: enable output stages DRV0 */
#define B_PORT_DRV_OUT_ENABLE_DRV1              ((1U << 1) << 2)                /*!< RW: enable output stages DRV1 */
#define B_PORT_DRV_OUT_ENABLE_DRV2              ((1U << 2) << 2)                /*!< RW: enable output stages DRV2 */
#define B_PORT_DRV_OUT_ENABLE_DRV3              ((1U << 3) << 2)                /*!< RW: enable output stages DRV3 */
#define B_PORT_DRV_OUT_ENABLE_DRVMOD_CPCLK      (1U << 1)                       /*!< RW: enable driver clock */
#define B_PORT_DRV_OUT_ENABLE_DRVSUP            (1U << 0)                       /*!< RW: enable driver supply */

/* ********************* */
/* Block: PORT_DRV1_TEST */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV1_TEST __attribute__((nodp, addr(0x0020E)));  /*!< IO_PORT_DRV1_TEST */
#define B_PORT_DRV1_TEST_TST_VBGCS_LV0_DRVSUP   (1U << 15)                      /*!< RW: Value for analogue */
#define B_PORT_DRV1_TEST_TST_UVCMP_LV1_DRVSUP   (1U << 14)                      /*!< RW: Value for analogue */
#define M_PORT_DRV1_TEST_TST_OUT_LV1            (15U << 10)                     /*!< RW: Value for analogue */
#define B_PORT_DRV1_TEST_TST_STRS_VHS_LS_DRV    (1U << 9)                       /*!< RW: Value for analogue */
#define B_PORT_DRV1_TEST_TST_STRS_VHS_HS_DRV    (1U << 8)                       /*!< RW: Value for analogue */
#define M_PORT_DRV1_TEST_TST_LS_OC_ISENSE       (15U << 4)                      /*!< RW: Value for analogue */
#define M_PORT_DRV1_TEST_TST_HS_OC_ISENSE       (15U << 0)                      /*!< RW: Value for analogue */

/* ********************* */
/* Block: PORT_DRV2_TEST */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV2_TEST __attribute__((nodp, addr(0x00210)));  /*!< IO_PORT_DRV2_TEST */
#define B_PORT_DRV2_TEST_TST_DRVSUP_DISC_CMP    (1U << 13)                      /*!< RW: Value for analogue */
#define B_PORT_DRV2_TEST_TST_OC_REDUCE_LVL      (1U << 12)                      /*!< RW: Value for analogue */
#define M_PORT_DRV2_TEST_TEST_DRV_ILD           (3U << 10)                      /*!< RW: Value for analogue */
#define B_PORT_DRV2_TEST_TEST_DRVMOD_CPCLK      (1U << 9)                       /*!< RW: Value for analogue */
#define B_PORT_DRV2_TEST_TST_DRVSUP             (1U << 8)                       /*!< RW: Value for analogue */
#define M_PORT_DRV2_TEST_TST_OUT_LV0            (15U << 4)                      /*!< RW: Value for analogue */
#define M_PORT_DRV2_TEST_TST_OUT_HV0            (15U << 0)                      /*!< RW: Value for analogue */

/* ********************* */
/* Block: PORT_DRV3_TEST */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV3_TEST __attribute__((nodp, addr(0x00212)));  /*!< IO_PORT_DRV3_TEST */
#define B_PORT_DRV3_TEST_TST_DRVMOD_CPCLK_DAC   (1U << 11)                      /*!< RW: Value for analogue */
#define B_PORT_DRV3_TEST_TST_V5V_DIV3           (1U << 10)                      /*!< RW: Value for analogue */
#define B_PORT_DRV3_TEST_TST_DAC_LV0_CSA        (1U << 9)                       /*!< RW: Value for analogue */
#define B_PORT_DRV3_TEST_TST_AMP_DIV_LV0_CSA    (1U << 8)                       /*!< RW: Value for analogue */
#define B_PORT_DRV3_TEST_TST_CSA_OC             (1U << 7)                       /*!< RW: Value for analogue */
#define B_PORT_DRV3_TEST_TST_CSA_FR             (1U << 6)                       /*!< RW: Value for analogue */
#define B_PORT_DRV3_TEST_TST_CSA_RS             (1U << 5)                       /*!< RW: Value for analogue */
#define B_PORT_DRV3_TEST_TST_HS_GHS_IBOOST_HRDOFFSW (1U << 4)                   /*!< RW: Value for analogue */
#define M_PORT_DRV3_TEST_TST_GHS_NOCURR         (15U << 0)                      /*!< RW: Value for analogue */

/* ******************** */
/* Block: PORT_DRV_CTRL */
/* ******************** */
extern volatile uint16_t IO_PORT_DRV_CTRL __attribute__((nodp, addr(0x00214)));  /*!< IO_PORT_DRV_CTRL */
#define M_PORT_DRV_CTRL_DRV3_CTRL               (15U << 12)                     /*!< RW: Control of PWM output for phase 3 */
#define C_PORT_DRV_CTRL_DRV3_MASTER1            (0U << 12)                      /*!< RW: Control DRV3 by PWM_MASTER1 */
#define C_PORT_DRV_CTRL_DRV3_SLAVE1             (1U << 12)                      /*!< RW: Control DRV3 by PWM_SLAVE1 */
#define C_PORT_DRV_CTRL_DRV3_SLAVE2             (2U << 12)                      /*!< RW: Control DRV3 by PWM_SLAVE2 */
#define C_PORT_DRV_CTRL_DRV3_SLAVE3             (3U << 12)                      /*!< RW: Control DRV3 by PWM_SLAVE3 */
#define C_PORT_DRV_CTRL_DRV3_MASTER2            (4U << 12)                      /*!< RW: Control DRV3 by PWM_MASTER2 */
#define C_PORT_DRV_CTRL_DRV3_IO0                (5U << 12)                      /*!< RW: Control DRV3 by IO0 */
#define C_PORT_DRV_CTRL_DRV3_IO1                (6U << 12)                      /*!< RW: Control DRV3 by IO1 */
#define C_PORT_DRV_CTRL_DRV3_IO2                (7U << 12)                      /*!< RW: Control DRV3 by IO2 */
#define C_PORT_DRV_CTRL_DRV3_IO3                (8U << 12)                      /*!< RW: Control DRV3 by IO3 */
#define C_PORT_DRV_CTRL_DRV3_LIN_XRX            (9U << 12)                      /*!< RW: Control DRV3 by LIN_XRX */
#define C_PORT_DRV_CTRL_DRV3_CTIMER0            (10U << 12)                     /*!< RW: Control DRV3 by CTIMER0 */
#define C_PORT_DRV_CTRL_DRV3_CTIMER1            (11U << 12)                     /*!< RW: Control DRV3 by CTIMER1 */
#define C_PORT_DRV_CTRL_DRV3_L                  (13U << 12)                     /*!< RW: Control DRV3 LOW */
#define C_PORT_DRV_CTRL_DRV3_TRISTATE           (14U << 12)                     /*!< RW: Control DRV3 TRI-STATE */
#define C_PORT_DRV_CTRL_DRV3_H                  (15U << 12)                     /*!< RW: Control DRV3 HIGH */
#define M_PORT_DRV_CTRL_DRV2_CTRL               (15U << 8)                      /*!< RW: Control of PWM output for phase 2 */
#define C_PORT_DRV_CTRL_DRV2_MASTER1            (0U << 8)                       /*!< RW: Control DRV2 by PWM_MASTER1 */
#define C_PORT_DRV_CTRL_DRV2_SLAVE1             (1U << 8)                       /*!< RW: Control DRV2 by PWM_SLAVE1 */
#define C_PORT_DRV_CTRL_DRV2_SLAVE2             (2U << 8)                       /*!< RW: Control DRV2 by PWM_SLAVE2 */
#define C_PORT_DRV_CTRL_DRV2_SLAVE3             (3U << 8)                       /*!< RW: Control DRV2 by PWM_SLAVE3 */
#define C_PORT_DRV_CTRL_DRV2_MASTER2            (4U << 8)                       /*!< RW: Control DRV2 by PWM_MASTER2 */
#define C_PORT_DRV_CTRL_DRV2_IO0                (5U << 8)                       /*!< RW: Control DRV2 by IO0 */
#define C_PORT_DRV_CTRL_DRV2_IO1                (6U << 8)                       /*!< RW: Control DRV2 by IO1 */
#define C_PORT_DRV_CTRL_DRV2_IO2                (7U << 8)                       /*!< RW: Control DRV2 by IO2 */
#define C_PORT_DRV_CTRL_DRV2_IO3                (8U << 8)                       /*!< RW: Control DRV2 by IO3 */
#define C_PORT_DRV_CTRL_DRV2_LIN_XRX            (9U << 8)                       /*!< RW: Control DRV2 by LIN_XRX */
#define C_PORT_DRV_CTRL_DRV2_CTIMER0            (10U << 8)                      /*!< RW: Control DRV2 by CTIMER0 */
#define C_PORT_DRV_CTRL_DRV2_CTIMER1            (11U << 8)                      /*!< RW: Control DRV2 by CTIMER1 */
#define C_PORT_DRV_CTRL_DRV2_L                  (13U << 8)                      /*!< RW: Control DRV2 LOW */
#define C_PORT_DRV_CTRL_DRV2_TRISTATE           (14U << 8)                      /*!< RW: Control DRV2 TRI-STATE */
#define C_PORT_DRV_CTRL_DRV2_H                  (15U << 8)                      /*!< RW: Control DRV2 HIGH */
#define M_PORT_DRV_CTRL_DRV1_CTRL               (15U << 4)                      /*!< RW: Control of PWM output for phase 1 */
#define C_PORT_DRV_CTRL_DRV1_MASTER1            (0U << 4)                       /*!< RW: Control DRV1 by PWM_MASTER1 */
#define C_PORT_DRV_CTRL_DRV1_SLAVE1             (1U << 4)                       /*!< RW: Control DRV1 by PWM_SLAVE1 */
#define C_PORT_DRV_CTRL_DRV1_SLAVE2             (2U << 4)                       /*!< RW: Control DRV1 by PWM_SLAVE2 */
#define C_PORT_DRV_CTRL_DRV1_SLAVE3             (3U << 4)                       /*!< RW: Control DRV1 by PWM_SLAVE3 */
#define C_PORT_DRV_CTRL_DRV1_MASTER2            (4U << 4)                       /*!< RW: Control DRV1 by PWM_MASTER2 */
#define C_PORT_DRV_CTRL_DRV1_IO0                (5U << 4)                       /*!< RW: Control DRV1 by IO0 */
#define C_PORT_DRV_CTRL_DRV1_IO1                (6U << 4)                       /*!< RW: Control DRV1 by IO1 */
#define C_PORT_DRV_CTRL_DRV1_IO2                (7U << 4)                       /*!< RW: Control DRV1 by IO2 */
#define C_PORT_DRV_CTRL_DRV1_IO3                (8U << 4)                       /*!< RW: Control DRV1 by IO3 */
#define C_PORT_DRV_CTRL_DRV1_LIN_XRX            (9U << 4)                       /*!< RW: Control DRV1 by LIN_XRX */
#define C_PORT_DRV_CTRL_DRV1_CTIMER0            (10U << 4)                      /*!< RW: Control DRV1 by CTIMER0 */
#define C_PORT_DRV_CTRL_DRV1_CTIMER1            (11U << 4)                      /*!< RW: Control DRV1 by CTIMER1 */
#define C_PORT_DRV_CTRL_DRV1_L                  (13U << 4)                      /*!< RW: Control DRV1 LOW */
#define C_PORT_DRV_CTRL_DRV1_TRISTATE           (14U << 4)                      /*!< RW: Control DRV1 TRI-STATE */
#define C_PORT_DRV_CTRL_DRV1_H                  (15U << 4)                      /*!< RW: Control DRV1 HIGH */
#define M_PORT_DRV_CTRL_DRV0_CTRL               (15U << 0)                      /*!< RW: Control of PWM output for phase 0 */
#define C_PORT_DRV_CTRL_DRV0_MASTER1            (0U << 0)                       /*!< RW: Control DRV0 by PWM_MASTER1 */
#define C_PORT_DRV_CTRL_DRV0_SLAVE1             (1U << 0)                       /*!< RW: Control DRV0 by PWM_SLAVE1 */
#define C_PORT_DRV_CTRL_DRV0_SLAVE2             (2U << 0)                       /*!< RW: Control DRV0 by PWM_SLAVE2 */
#define C_PORT_DRV_CTRL_DRV0_SLAVE3             (3U << 0)                       /*!< RW: Control DRV0 by PWM_SLAVE3 */
#define C_PORT_DRV_CTRL_DRV0_MASTER2            (4U << 0)                       /*!< RW: Control DRV0 by PWM_MASTER2 */
#define C_PORT_DRV_CTRL_DRV0_IO0                (5U << 0)                       /*!< RW: Control DRV0 by IO0 */
#define C_PORT_DRV_CTRL_DRV0_IO1                (6U << 0)                       /*!< RW: Control DRV0 by IO1 */
#define C_PORT_DRV_CTRL_DRV0_IO2                (7U << 0)                       /*!< RW: Control DRV0 by IO2 */
#define C_PORT_DRV_CTRL_DRV0_IO3                (8U << 0)                       /*!< RW: Control DRV0 by IO3 */
#define C_PORT_DRV_CTRL_DRV0_LIN_XRX            (9U << 0)                       /*!< RW: Control DRV0 by LIN_XRX */
#define C_PORT_DRV_CTRL_DRV0_CTIMER0            (10U << 0)                      /*!< RW: Control DRV0 by CTIMER0 */
#define C_PORT_DRV_CTRL_DRV0_CTIMER1            (11U << 0)                      /*!< RW: Control DRV0 by CTIMER1 */
#define C_PORT_DRV_CTRL_DRV0_L                  (13U << 0)                      /*!< RW: Control DRV0 LOW */
#define C_PORT_DRV_CTRL_DRV0_TRISTATE           (14U << 0)                      /*!< RW: Control DRV0 TRI-STATE */
#define C_PORT_DRV_CTRL_DRV0_H                  (15U << 0)                      /*!< RW: Control DRV0 HIGH */

/* ********************* */
/* Block: PORT_DRV1_PROT */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV1_PROT __attribute__((nodp, addr(0x00216)));  /*!< IO_PORT_DRV1_PROT */
#define B_PORT_DRV1_PROT_DIS_OV_LS_VDS          (1U << 15)                      /*!< RW: Disable Low Side VDS Over-voltage HW-protection */
#define B_PORT_DRV1_PROT_OV_LS_VDS_PM           (1U << 14)                      /*!< RW: Protection mode: 0: Switch HS; 1: Tri-state */
#define B_PORT_DRV1_PROT_DIS_OV_HS_VDS          (1U << 13)                      /*!< RW: Disable High Side VDS Over-voltage HW-protection */
#define B_PORT_DRV1_PROT_OV_HS_VDS_PM           (1U << 12)                      /*!< RW: Protection mode: 0: Switch LS; 1: Tri-state */
#define B_PORT_DRV1_PROT_DIS_OC                 (1U << 11)                      /*!< RW: Disable Over Current HW-protection */
#define B_PORT_DRV1_PROT_OC_PM                  (1U << 10)                      /*!< RW: Protection mode: 0: Switch LS; 1: Tri-state */
#define B_PORT_DRV1_PROT_DIS_UV_VDDAF           (1U << 9)                       /*!< RW: Disable Under Voltage VDDAF HW-protection */
#define B_PORT_DRV1_PROT_UV_VDDAF_PM            (1U << 8)                       /*!< RW: Protection mode: 0: Switch LS; 1: Tri-state */
#define B_PORT_DRV1_PROT_DIS_UV_VDDA            (1U << 7)                       /*!< RW: Disable Under Voltage VDDA HW-protection */
#define B_PORT_DRV1_PROT_UV_VDDA_PM             (1U << 6)                       /*!< RW: Protection mode: 0: Switch LS; 1: Tri-state */
#define B_PORT_DRV1_PROT_DIS_UV_VS              (1U << 5)                       /*!< RW: Disable Under Voltage VS HW-protection */
#define B_PORT_DRV1_PROT_UV_VS_PM               (1U << 4)                       /*!< RW: Protection mode: 0: Switch LS; 1: Tri-state */
#define B_PORT_DRV1_PROT_DIS_OV_VS              (1U << 3)                       /*!< RW: Disable Over Voltage VS HW-protection */
#define B_PORT_DRV1_PROT_OV_VS_PM               (1U << 2)                       /*!< RW: Protection mode: 0: Switch LS; 1: Tri-state */
#define B_PORT_DRV1_PROT_DIS_OVT                (1U << 1)                       /*!< RW: Disable Over Temperature HW-protection */
#define B_PORT_DRV1_PROT_OVT_PM                 (1U << 0)                       /*!< RW: Protection mode: 0: Switch LS; 1: Tri-state */

/* ********************* */
/* Block: PORT_DRV2_PROT */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV2_PROT __attribute__((nodp, addr(0x00218)));  /*!< IO_PORT_DRV2_PROT */
#define B_PORT_DRV2_PROT_DIS_DRV                (1U << 0)                       /*!< RW: Disable drivers */

/* ****************** */
/* Block: PORT_DRV_IN */
/* ****************** */
extern volatile uint16_t IO_PORT_DRV_IN __attribute__((nodp, addr(0x0021A)));   /*!< IO_PORT_DRV_IN */
#define B_PORT_DRV_IN_OV_LS_VDS3_IT             (1U << 15)                      /*!< R: After filtering and masking */
#define B_PORT_DRV_IN_OV_LS_VDS2_IT             (1U << 14)                      /*!< R: After filtering and masking */
#define B_PORT_DRV_IN_OV_LS_VDS1_IT             (1U << 13)                      /*!< R: After filtering and masking */
#define B_PORT_DRV_IN_OV_LS_VDS0_IT             (1U << 12)                      /*!< R: After filtering and masking */
#define M_PORT_DRV_IN_OV_LS_VDS_SYNC            (15U << 8)                      /*!< R: Before digital filtering */
#define B_PORT_DRV_IN_OV_LS_VDS_SYNC_VDS0       ((1U << 0) << 8)                /*!< LS VDS Sync VDS0 */
#define B_PORT_DRV_IN_OV_LS_VDS_SYNC_VDS1       ((1U << 1) << 8)                /*!< LS VDS Sync VDS1 */
#define B_PORT_DRV_IN_OV_LS_VDS_SYNC_VDS2       ((1U << 2) << 8)                /*!< LS VDS Sync VDS2 */
#define B_PORT_DRV_IN_OV_LS_VDS_SYNC_VDS3       ((1U << 3) << 8)                /*!< LS VDS Sync VDS3 */
#define B_PORT_DRV_IN_OV_HS_VDS3_IT             (1U << 7)                       /*!< R: After filtering and masking */
#define B_PORT_DRV_IN_OV_HS_VDS2_IT             (1U << 6)                       /*!< R: After filtering and masking */
#define B_PORT_DRV_IN_OV_HS_VDS1_IT             (1U << 5)                       /*!< R: After filtering and masking */
#define B_PORT_DRV_IN_OV_HS_VDS0_IT             (1U << 4)                       /*!< R: After filtering and masking */
#define M_PORT_DRV_IN_OV_HS_VDS_SYNC            (15U << 0)                      /*!< R: Before digital filtering */
#define B_PORT_DRV_IN_OV_HS_VDS_SYNC_VDS0       ((1U << 0) << 0)                /*!< HS VDS Sync VDS0 */
#define B_PORT_DRV_IN_OV_HS_VDS_SYNC_VDS1       ((1U << 1) << 0)                /*!< HS VDS Sync VDS1 */
#define B_PORT_DRV_IN_OV_HS_VDS_SYNC_VDS2       ((1U << 2) << 0)                /*!< HS VDS Sync VDS2 */
#define B_PORT_DRV_IN_OV_HS_VDS_SYNC_VDS3       ((1U << 3) << 0)                /*!< HS VDS Sync VDS3 */

/* ******************* */
/* Block: PORT_DIAG_IN */
/* ******************* */
extern volatile uint16_t IO_PORT_DIAG_IN __attribute__((nodp, addr(0x0021C)));  /*!< IO_PORT_DIAG_IN */
#define M_PORT_DIAG_IN_OV_LS_VDS_MEM            (15U << 10)                     /*!< R: Cleared if dis_ov_ls_vds */
#define B_PORT_DIAG_IN_OV_LS_VDS_MEM_VDS0       ((1U << 0) << 10)               /*!< R: Cleared if dis_ov_ls_vds0 */
#define B_PORT_DIAG_IN_OV_LS_VDS_MEM_VDS1       ((1U << 1) << 10)               /*!< R: Cleared if dis_ov_ls_vds1 */
#define B_PORT_DIAG_IN_OV_LS_VDS_MEM_VDS2       ((1U << 2) << 10)               /*!< R: Cleared if dis_ov_ls_vds2 */
#define B_PORT_DIAG_IN_OV_LS_VDS_MEM_VDS3       ((1U << 3) << 10)               /*!< R: Cleared if dis_ov_ls_vds3 */
#define M_PORT_DIAG_IN_OV_HS_VDS_MEM            (15U << 6)                      /*!< R: Cleared if dis_ov_hs_vds */
#define B_PORT_DIAG_IN_OV_HS_VDS_MEM_VDS0       ((1U << 0) << 6)                /*!< R: Cleared if dis_ov_hs_vds0 */
#define B_PORT_DIAG_IN_OV_HS_VDS_MEM_VDS1       ((1U << 1) << 6)                /*!< R: Cleared if dis_ov_hs_vds1 */
#define B_PORT_DIAG_IN_OV_HS_VDS_MEM_VDS2       ((1U << 2) << 6)                /*!< R: Cleared if dis_ov_hs_vds2 */
#define B_PORT_DIAG_IN_OV_HS_VDS_MEM_VDS3       ((1U << 3) << 6)                /*!< R: Cleared if dis_ov_hs_vds3 */
#define B_PORT_DIAG_IN_OVC_MEM                  (1U << 5)                       /*!< R: Cleared if dis_oc */
#define B_PORT_DIAG_IN_UV_VDDAF_MEM             (1U << 4)                       /*!< R: Cleared if dis_uv_vddaf */
#define B_PORT_DIAG_IN_UV_VDDA_MEM              (1U << 3)                       /*!< R: Cleared if dis_uv_vdda */
#define B_PORT_DIAG_IN_UV_VS_MEM                (1U << 2)                       /*!< R: Cleared if dis_uv_vs */
#define B_PORT_DIAG_IN_OV_VS_MEM                (1U << 1)                       /*!< R: Cleared if dis_ov_vs */
#define B_PORT_DIAG_IN_OVT_MEM                  (1U << 0)                       /*!< R: Cleared if dis_ovt */

/* ******************** */
/* Block: PORT_I2C_CONF */
/* ******************** */
extern volatile uint16_t IO_PORT_I2C_CONF __attribute__((nodp, addr(0x0021E)));  /*!< IO_PORT_I2C_CONF */
#define M_PORT_I2C_CONF_I2C_SCL_CLK_SEL         (3U << 9)                       /*!< RW: Value for analogue */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_SCL     (0U << 9)                       /*!< RW: I2C Clock Selection SCL */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_IO2     (1U << 9)                       /*!< RW: I2C Clock Selection IO2 */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_IO3     (2U << 9)                       /*!< RW: I2C Clock Selection IO3 */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_LIN_RXD (3U << 9)                       /*!< RW: I2C Clock Selection LIN_RXD */
#define B_PORT_I2C_CONF_SDA_FILT_ENABLE         (1U << 8)                       /*!< RW: Value for analogue */
#define B_PORT_I2C_CONF_I2C_ADDR_VALID          (1U << 7)                       /*!< RW: enable I2C */
#define M_PORT_I2C_CONF_I2C_ADDR                (127U << 0)                     /*!< RW: 7-bit address for I2C */

/* ************************** */
/* Block: PORT_I2C_DMA_OFFSET */
/* ************************** */
extern volatile uint16_t IO_PORT_I2C_DMA_OFFSET __attribute__((nodp, addr(0x00220)));  /*!< IO_PORT_I2C_DMA_OFFSET */
#define M_PORT_I2C_DMA_OFFSET_I2C_DMA_OFFSET    (65535U << 0)                   /*!< RW: BYTE address defines the start of DMA accesses (so for beginning of RAM it is 0x1000). LSB is ignored to align on word for DMA */

/* *************************** */
/* Block: PORT_I2C_READ_OFFSET */
/* *************************** */
extern volatile uint16_t IO_PORT_I2C_READ_OFFSET __attribute__((nodp, addr(0x00222)));  /*!< IO_PORT_I2C_READ_OFFSET */
#define M_PORT_I2C_READ_OFFSET_I2C_READ_OFFSET  (255U << 0)                     /*!< RW: WORD address for "direct read" startup offset. Up to 255 */

/* ********************* */
/* Block: PORT_MISC_TEST */
/* ********************* */
extern volatile uint16_t IO_PORT_MISC_TEST __attribute__((nodp, addr(0x00224)));  /*!< IO_PORT_MISC_TEST */
#define B_PORT_MISC_TEST_DIS_TX_TIMEOUT         (1U << 8)                       /*!< RW: disable timeout protection (10-20ms) on TX output */
#define M_PORT_MISC_TEST_LINAA_TST_VREF         (3U << 6)                       /*!< RW: Value for analogue */
#define B_PORT_MISC_TEST_TST_LV1_IO0            (1U << 5)                       /*!< RW: Value for analogue */
#define B_PORT_MISC_TEST_TST_LV0_IO0            (1U << 4)                       /*!< RW: Value for analogue */
#define B_PORT_MISC_TEST_TST_LV0_BUF_ENABLE     (1U << 3)                       /*!< RW: Value for analogue */
#define B_PORT_MISC_TEST_TST_RESREF_LV0_OTD     (1U << 2)                       /*!< RW: Value for analogue */
#define B_PORT_MISC_TEST_TST_DISC_DIO_OTD       (1U << 1)                       /*!< RW: Value for analogue */
#define B_PORT_MISC_TEST_TST_CMP_LV0_OTD        (1U << 0)                       /*!< RW: Value for analogue */

/* ************************* */
/* Block: PORT_PPM_RBASE_ADD */
/* ************************* */
extern volatile uint16_t IO_PORT_PPM_RBASE_ADD __attribute__((nodp, addr(0x00226)));  /*!< IO_PORT_PPM_RBASE_ADD */
#define M_PORT_PPM_RBASE_ADD_PPM_RBASE_ADD      (65535U << 0)                   /*!< RW: Base address of the PPM DMA message in RX mode */

/* ************************* */
/* Block: PORT_PPM_TBASE_ADD */
/* ************************* */
extern volatile uint16_t IO_PORT_PPM_TBASE_ADD __attribute__((nodp, addr(0x00228)));  /*!< IO_PORT_PPM_TBASE_ADD */
#define M_PORT_PPM_TBASE_ADD_PPM_TBASE_ADD      (65535U << 0)                   /*!< RW: Base address of the PPM DMA message in TX mode */

/* *********************** */
/* Block: PORT_PPM_TIMEOUT */
/* *********************** */
extern volatile uint16_t IO_PORT_PPM_TIMEOUT __attribute__((nodp, addr(0x0022A)));  /*!< IO_PORT_PPM_TIMEOUT */
#define M_PORT_PPM_TIMEOUT_PPM_TIMEOUT          (65535U << 0)                   /*!< RW: Timeout value */

/* ******************** */
/* Block: PORT_PPM_CTRL */
/* ******************** */
extern volatile uint16_t IO_PORT_PPM_CTRL __attribute__((nodp, addr(0x0022C)));  /*!< IO_PORT_PPM_CTRL */
#define M_PORT_PPM_CTRL_PPM_IN_SEL              (7U << 11)                      /*!< RW: IO or LIN_XRX for PPM input */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_LIN_XRX      (0U << 11)                      /*!< RW: LIN_XRX for PPM input */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO0          (1U << 11)                      /*!< RW: IO0 for PPM input */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO1          (2U << 11)                      /*!< RW: IO1 for PPM input */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO2          (3U << 11)                      /*!< RW: IO2 for PPM input */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO3          (4U << 11)                      /*!< RW: IO3 for PPM input */
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
extern volatile uint16_t IO_PORT_PPM_BUF_DATA __attribute__((nodp, addr(0x0022E)));  /*!< IO_PORT_PPM_BUF_DATA */
#define M_PORT_PPM_BUF_DATA_PPM_BUF_DATA        (65535U << 0)                   /*!< R: Value for digital read */

/* **************** */
/* Block: PPM_TIMER */
/* **************** */
extern volatile uint16_t IO_PPM_TIMER __attribute__((nodp, addr(0x00230)));     /*!< IO_PPM_TIMER */
#define M_PPM_TIMER_PPM_ON_TIME                 (65535U << 0)                   /*!< RW: PPM On Time for TX mode */

/* ********************* */
/* Block: PORT_PPM_FLAGS */
/* ********************* */
extern volatile uint16_t IO_PORT_PPM_FLAGS __attribute__((nodp, addr(0x00232)));  /*!< IO_PORT_PPM_FLAGS */
#define B_PORT_PPM_FLAGS_PPM_DMA_ERR            (1U << 3)                       /*!< R: Value for digital read */
#define B_PORT_PPM_FLAGS_PPM_TOUT               (1U << 2)                       /*!< R: Value for digital read */
#define B_PORT_PPM_FLAGS_PPM_RCVF               (1U << 1)                       /*!< R: Value for digital read */
#define B_PORT_PPM_FLAGS_PPM_DMA_OP             (1U << 0)                       /*!< R: Value for digital read */

/* ********************** */
/* Block: PORT_SSCM2_CONF */
/* ********************** */
extern volatile uint16_t IO_PORT_SSCM2_CONF __attribute__((nodp, addr(0x00234)));  /*!< IO_PORT_SSCM2_CONF */
#define B_PORT_SSCM2_CONF_SSCM2_SINGLEBIT       (1U << 1)                       /*!< RW: output shall provide a code with Hamming distance of 1 instead of triangle */
#define B_PORT_SSCM2_CONF_SSCM2_EN              (1U << 0)                       /*!< RW: enable the spread spectrum modulation */

/* ********************** */
/* Block: PORT_STEP2_CONF */
/* ********************** */
extern volatile uint16_t IO_PORT_STEP2_CONF __attribute__((nodp, addr(0x00236)));  /*!< IO_PORT_STEP2_CONF */
#define M_PORT_STEP2_CONF_STEP2_CNT             (255U << 8)                     /*!< RW: step count per period of triangular modulation for Spread Spectrum */
#define M_PORT_STEP2_CONF_STEP2_DUR             (15U << 4)                      /*!< RW: step duration in main clock pulses for Spread Spectrum */
#define M_PORT_STEP2_CONF_STEP2_INC             (15U << 0)                      /*!< RW: step increment for Spread Spectrum */

#else

extern volatile uint16_t IO_COLIN_RAM_PROT __attribute__((nodp, addr(0x001C4)));  /*!< IO_COLIN_RAM_PROT */
#define B_COLIN_LOCK                            (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_COLIN_RAM_PROT_LIMIT                  (255U << 0)                     /*!< RW: Bottom RAM Area ProtectionLimit. Write is invalid if LOCK is set */

/* ******************** */
/* Block: PORT_LIN_XKEY */
/* ******************** */
extern volatile uint16_t IO_PORT_LIN_XKEY_S __attribute__((nodp, addr(0x001C6)));  /*!< IO_PORT_LIN_XKEY_S (System) */
#define M_PORT_LIN_XKEY_LIN_XKEY                (65535U << 0)                   /*!< RW: store a valid key for LIN XCFG */

/* ******************* */
/* Block: TRIM_BG_BIAS */
/* ******************* */
extern volatile uint16_t IO_TRIM_BG_BIAS __attribute__((nodp, addr(0x001C8)));  /*!< IO_TRIM_BG_BIAS */
#define B_TRIM_BG_BIAS_LOCK                     (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define B_TRIM_BG_BIAS_TR_UNUSED                (1U << 14)                      /*!< RW: free for spare use */
#define M_TRIM_BG_BIAS_PRE_TR_BIAS              (63U << 8)                      /*!< RW: current bias source trimming */
#define M_TRIM_BG_BIAS_PRE_TR_BGD               (15U << 4)                      /*!< RW: digital bandgap trimming */
#define M_TRIM_BG_BIAS_PRE_TR_BGA               (15U << 0)                      /*!< RW: analogue bandgap trimming */

/* *************** */
/* Block: TRIM_VDD */
/* *************** */
extern volatile uint16_t IO_TRIM_VDD __attribute__((nodp, addr(0x001CA)));      /*!< IO_TRIM_VDD */
#define B_TRIM_VDD_LOCK                         (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define B_TRIM_VDD_TR_UNUSED                    (1U << 14)                      /*!< RW: free for spare use */
#define M_TRIM_VDD_PRE_TR_SUP                   (255U << 6)                     /*!< RW: spare control output bits for supply system */
#define M_TRIM_VDD_PRE_TR_VDDD                  (7U << 3)                       /*!< RW: digital bandgap trimming */
#define M_TRIM_VDD_PRE_TR_VDDA                  (7U << 0)                       /*!< RW: analogue bandgap trimming */

/* ****************** */
/* Block: TRIM_RCO32M */
/* ****************** */
extern volatile uint16_t IO_TRIM_RCO32M __attribute__((nodp, addr(0x001CC)));   /*!< IO_TRIM_RCO32M */
#define B_TRIM_RCO32M_LOCK                      (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_TRIM_RCO32M_TR_UNUSED                 (31U << 10)                     /*!< RW: free for spare use */
#define M_TRIM_RCO32M_TR_RCO32M_IN              (1023U << 0)                    /*!< RW: 32 MHz RC oscillator trimming */

/* ***************** */
/* Block: TRIM_RCO1M */
/* ***************** */
extern volatile uint16_t IO_TRIM_RCO1M __attribute__((nodp, addr(0x001CE)));    /*!< IO_TRIM_RCO1M */
#define B_TRIM_RCO1M_LOCK                       (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define B_TRIM_RCO1M_TR_UNUSED                  (1U << 14)                      /*!< RW: free for spare use */
#define M_TRIM_RCO1M_PRE_TR_LIN_SLVTERM         (7U << 11)                      /*!< RW: slave termination trimming */
#define M_TRIM_RCO1M_PRE_TR_LIN_SLEWRATE        (7U << 8)                       /*!< RW: slew rate trimming */
#define M_TRIM_RCO1M_PRE_TR_RCO1M               (255U << 0)                     /*!< RW: 1 MHz RC oscillator trimming */

/* ********************* */
/* Block: PORT_SSCM_CONF */
/* ********************* */
extern volatile uint16_t IO_PORT_SSCM_CONF __attribute__((nodp, addr(0x001D0)));  /*!< IO_PORT_SSCM_CONF */
#define B_PORT_SSCM_CONF_SSCM_CENTERED          (1U << 2)                       /*!< RW: Spectrum around middle */
#define B_PORT_SSCM_CONF_SSCM_SINGLEBIT         (1U << 1)                       /*!< RW: output shall provide a code with Hamming distance of 1 instead of triangle */
#define B_PORT_SSCM_CONF_SSCM_EN                (1U << 0)                       /*!< RW: enable the spread spectrum modulation */

/* ********************* */
/* Block: PORT_STEP_CONF */
/* ********************* */
extern volatile uint16_t IO_PORT_STEP_CONF __attribute__((nodp, addr(0x001D2)));  /*!< IO_PORT_STEP_CONF */
#define M_PORT_STEP_CONF_STEP_CNT               (255U << 8)                     /*!< RW: step count per period of triangular modulation for Spread Spectrum */
#define M_PORT_STEP_CONF_STEP_DUR               (15U << 4)                      /*!< RW: step duration in main clock pulses for Spread Spectrum */
#define M_PORT_STEP_CONF_STEP_INC               (15U << 0)                      /*!< RW: step increment for Spread Spectrum */

/* ********************* */
/* Block: PORT_SUPP_TEST */
/* ********************* */
extern volatile uint16_t IO_PORT_SUPP_TEST __attribute__((nodp, addr(0x001D4)));  /*!< IO_PORT_SUPP_TEST */
#define B_PORT_SUPP_TEST_SHOVE_VAUX             (1U << 14)                      /*!< RW: VAUX increased by 20% for Shove */
#define B_PORT_SUPP_TEST_SHOVE_VDDA             (1U << 13)                      /*!< RW: VDDA increased by 20% for Shove */
#define B_PORT_SUPP_TEST_SHOVE_VDDD             (1U << 12)                      /*!< RW: VDDD set to 2.4V */
#define B_PORT_SUPP_TEST_LOW_VDDD               (1U << 11)                      /*!< RW: VDDD set to 1.4V */
#define B_PORT_SUPP_TEST_SBY_BIAS               (1U << 10)                      /*!< RW: disable current bias source */
#define B_PORT_SUPP_TEST_SWITCHOFFUV_VDDD_RES   (1U << 9)                       /*!< RW: switch off resistor divider of HPORB for IDDQ test, must be additional to DIS_HPORB */
#define M_PORT_SUPP_TEST_SWITCHOFFREG_VDDD      (3U << 7)                       /*!< RW: disable VDDD regulator to drive from extern */
#define M_PORT_SUPP_TEST_SWITCHOFFREG_VDDA      (3U << 5)                       /*!< RW: disable VDDA regulator to drive from extern */
#define B_PORT_SUPP_TEST_PORTEST                (1U << 4)                       /*!< RW: signal to disable reset for level shifters inside analogue, shall be set before digital IDDQ test, needs to be set together with DIS_HPORB in digital test controller */
#define B_PORT_SUPP_TEST_TEST_5U_BIASAUX        (1U << 3)                       /*!< RW: switch 5u VAUX related BIAS to analogue test bus */
#define B_PORT_SUPP_TEST_TEST_10U_BIAS          (1U << 2)                       /*!< RW: switch 10u VDDA related BIAS to analogue test bus */
#define B_PORT_SUPP_TEST_TEST_VAUX_ADDCURRENT   (1U << 1)                       /*!< RW: load VAUX with add current 20uA, together with TEST_VAUX */
#define B_PORT_SUPP_TEST_TEST_BGA               (1U << 0)                       /*!< RW: switch VBGA to analogue test bus */

/* ********************** */
/* Block: PORT_SUPP2_TEST */
/* ********************** */
extern volatile uint16_t IO_PORT_SUPP2_TEST __attribute__((nodp, addr(0x001D6)));  /*!< IO_PORT_SUPP2_TEST */
#define B_PORT_SUPP2_TEST_FSTOP                 (1U << 13)                      /*!< RW: set SETP_STOP for stop mode test, no output, will be used to generate SET_STOP on digital only */
#define B_PORT_SUPP2_TEST_FGTSM                 (1U << 12)                      /*!< RW: set GTSM for wake up test, no output, will be used to generate GTSM on digital only */
#define B_PORT_SUPP2_TEST_TST_IO_LV             (1U << 11)                      /*!< RW: Value for analogue */
#define B_PORT_SUPP2_TEST_TST_VDDD_OUT2_FV      (1U << 10)                      /*!< RW: VDDD test switch */
#define B_PORT_SUPP2_TEST_TST_VDDA_OUT2_FV      (1U << 9)                       /*!< RW: VDDA test switch */
#define B_PORT_SUPP2_TEST_IDDQ_CLK10K           (1U << 8)                       /*!< RW: switch off 10 kHz clock for IDDQ test */
#define B_PORT_SUPP2_TEST_IDDQ_TEMPSENSE        (1U << 7)                       /*!< RW: switch off current flowing into temperature sensor */
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
extern volatile uint16_t IO_PORT_LIN_TEST __attribute__((nodp, addr(0x001D8)));  /*!< IO_PORT_LIN_TEST */
#define B_PORT_LIN_TEST_TEST_2U8_BIASLIN        (1U << 0)                       /*!< RW: switches 2.8uA low side bias current from LIN cell to analogue test bus */

/* ********************** */
/* Block: PORT_SPARE_TEST */
/* ********************** */
extern volatile uint16_t IO_PORT_SPARE_TEST __attribute__((nodp, addr(0x001DA)));  /*!< IO_PORT_SPARE_TEST */
#define M_PORT_SPARE_TEST_TEST_SUP              (7U << 7)                       /*!< RW: spare test control bits for supply system */
#define M_PORT_SPARE_TEST_TST_HS_SHORT_DET      (3U << 5)                       /*!< RW: Value for analogue */
#define M_PORT_SPARE_TEST_CPCLKSEL_DRV          (3U << 3)                       /*!< Value for analogue */
#define B_PORT_SPARE_TEST_TST_VDDA_VT_MON_SUP   (1U << 2)                       /*!< Value for analogue */
#define B_PORT_SPARE_TEST_RST_IOX_CONN2PHV      (1U << 1)                       /*!< Value for analogue */
#define B_PORT_SPARE_TEST_SET_IOX_CONN2PHV      (1U << 0)                       /*!< Value for analogue */

/* ************************* */
/* Block: PORT_ADC_TRIM_TEST */
/* ************************* */
extern volatile uint16_t IO_PORT_ADC_TRIM_TEST __attribute__((nodp, addr(0x001DC)));  /*!< IO_PORT_ADC_TRIM_TEST */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_RCO1M       (1U << 9)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_RCO1M pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_RCO32M      (1U << 8)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_RCO32M pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_LIN_SLVTERM (1U << 7)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_LIN_SLVTERM pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_LIN_SLEWRATE (1U << 6)                      /*!< RW: if 1: connect ADC_TRIMPORT to TR_LIN_SLEWRATE pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_SUP         (1U << 5)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_SUP pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_BIAS        (1U << 4)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_BIAS pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_VDDD        (1U << 3)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_VDDD pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_VDDA        (1U << 2)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_VDDA pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_BGD         (1U << 1)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_BGD pins, else connect from port */
#define B_PORT_ADC_TRIM_TEST_SEL_TR_BGA         (1U << 0)                       /*!< RW: if 1: connect ADC_TRIMPORT to TR_BGA pins, else connect from port */

/* ***************** */
/* Block: PORT_IO_IN */
/* ***************** */
extern volatile uint16_t IO_PORT_IO_IN __attribute__((nodp, addr(0x001DE)));    /*!< IO_PORT_IO_IN */
#define M_PORT_IO_IN_IO_IN_SYNC                 (15U << 0)                      /*!< R: From Schmitt trigger I/O (after re-synchronisation) */
#define B_PORT_IO_IN_IO_IN_SYNC_3               (1U << 3)                       /*!< R: From Schmitt trigger IO3 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_2               (1U << 2)                       /*!< R: From Schmitt trigger IO2 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_1               (1U << 1)                       /*!< R: From Schmitt trigger IO1 (after resynchro) */
#define B_PORT_IO_IN_IO_IN_SYNC_0               (1U << 0)                       /*!< R: From Schmitt trigger IO0 (after resynchro) */

/* ******************* */
/* Block: PORT_SUPP_IN */
/* ******************* */
extern volatile uint16_t IO_PORT_SUPP_IN __attribute__((nodp, addr(0x001E0)));  /*!< IO_PORT_SUPP_IN */
#define B_PORT_SUPP_IN_UV_VDDAF_SYNC            (1U << 11)                      /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_UV_VDDAF_IT              (1U << 10)                      /*!< R: Driver supply under voltage detection */
#define B_PORT_SUPP_IN_OVC_SYNC                 (1U << 9)                       /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_OVC_IT                   (1U << 8)                       /*!< R: Over current detection */
#define B_PORT_SUPP_IN_OVT_SYNC                 (1U << 7)                       /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_OVT_IT                   (1U << 6)                       /*!< R: Over temperature interrupt flag */
#define B_PORT_SUPP_IN_OV_VS_SYNC               (1U << 5)                       /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_OV_VS_IT                 (1U << 4)                       /*!< R: Over voltage at VS */
#define B_PORT_SUPP_IN_UV_VS_SYNC               (1U << 3)                       /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_UV_VS_IT                 (1U << 2)                       /*!< R: Under voltage at VS */
#define B_PORT_SUPP_IN_UV_VDDA_SYNC             (1U << 1)                       /*!< R: Before digital filtering */
#define B_PORT_SUPP_IN_UV_VDDA_IT               (1U << 0)                       /*!< R: Under voltage condition at VDDA */

/* ******************* */
/* Block: PORT_MISC_IN */
/* ******************* */
extern volatile uint16_t IO_PORT_MISC_IN __attribute__((nodp, addr(0x001E2)));  /*!< IO_PORT_MISC_IN */
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
extern volatile uint16_t IO_PORT_SUPP_CFG __attribute__((nodp, addr(0x001E4)));  /*!< IO_PORT_SUPP_CFG */
#define B_PORT_SUPP_CFG_OVT_FILT_SEL            (1U << 6)                       /*!< RW: 0 : 1-2us filtering, 1 : 100-200us filtering */
#define B_PORT_SUPP_CFG_OVC_FILT_SEL            (1U << 5)                       /*!< RW: 0 : 1-2us filtering, 1 : 100-200us filtering */
#define B_PORT_SUPP_CFG_OV_VS_FILT_SEL          (1U << 4)                       /*!< RW: 0 : 1-2us filtering, 1 : 100-200us filtering */
#define B_PORT_SUPP_CFG_UV_VS_FILT_SEL          (1U << 3)                       /*!< RW: 0 : 1-2us filtering, 1 : 100-200us filtering */
#define M_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL       (3U << 1)                       /*!< RW: 00: 1-2us filtering, 01: 10-11us filtering, 10: 20-21us filtering, 11: 100-110us filtering */
#define C_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL_00    (0U << 1)                       /*!< VDDAF 1-2us filtering */
#define C_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL_01    (1U << 1)                       /*!< VDDAF 10-11us filtering */
#define C_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL_10    (2U << 1)                       /*!< VDDAF 20-21us filtering */
#define C_PORT_SUPP_CFG_UV_VDDAF_FILT_SEL_11    (3U << 1)                       /*!< VDDAF 100-110us filtering */
#define B_PORT_SUPP_CFG_UV_VDDA_FILT_SEL        (1U << 0)                       /*!< RW: 0 : 1-2us filtering, 1 : 100-200us filtering */

/* *********************** */
/* Block: PORT_IO_OUT_SOFT */
/* *********************** */
extern volatile uint16_t IO_PORT_IO_OUT_SOFT __attribute__((nodp, addr(0x001E6)));  /*!< IO_PORT_IO_OUT_SOFT */
#define M_PORT_IO_OUT_SOFT_IO_OUT_SOFT          (15U << 0)                      /*!< RW: To GPIO glue logic control */
#define C_PORT_IO_OUT_SOFT_IO3_OUT              (1U << 3)                       /*!< IO[3] output */
#define C_PORT_IO_OUT_SOFT_IO2_OUT              (1U << 2)                       /*!< IO[2] output */
#define C_PORT_IO_OUT_SOFT_IO1_OUT              (1U << 1)                       /*!< IO[1] output */
#define C_PORT_IO_OUT_SOFT_IO0_OUT              (1U << 0)                       /*!< IO[0] output */

/* ********************* */
/* Block: PORT_IO_OUT_EN */
/* ********************* */
extern volatile uint16_t IO_PORT_IO_OUT_EN __attribute__((nodp, addr(0x001E8)));  /*!< IO_PORT_IO_OUT_EN */
#define B_PORT_IO_OUT_EN_IOX_OD_ENABLE          (1U << 9)                       /*!< RW: Enable the open drain mode for IO2 */
#define B_PORT_IO_OUT_EN_IO0_OD_ENABLE          (1U << 8)                       /*!< RW: Enable the open drain mode for IO0 */
#define B_PORT_IO_OUT_EN_IO0_LV_ENABLE          (1U << 7)                       /*!< RW: Enable the LV mode from IO0 */
#define M_PORT_IO_OUT_EN_IO_CH_SEL              (7U << 4)                       /*!< RW: Enable IOx connection to corresponding phase */
#define M_PORT_IO_OUT_EN_UNUSED                 (15U << 0)                      /*!< RW: Value for analogue */

/* ******************* */
/* Block: PORT_IO_CFG0 */
/* ******************* */
extern volatile uint16_t IO_PORT_IO_CFG0 __attribute__((nodp, addr(0x001EA)));  /*!< IO_PORT_IO_CFG0 */
#define M_PORT_IO_CFG0_IO3_OUT_SEL              (15U << 12)                     /*!< RW: Output selection for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PWM_MSTR1    (0U << 12)                      /*!< RW: Output selection PWM_MASTER1 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PWM_SLV1     (1U << 12)                      /*!< RW: Output selection PWM_SLAVE1 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PWM_SLV2     (2U << 12)                      /*!< RW: Output selection PWM_SLAVE2 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PWM_SLV3     (3U << 12)                      /*!< RW: Output selection PWM_SLAVE3 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PWM_MSTR2    (4U << 12)                      /*!< RW: Output selection PWM_MASTER2 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_CTIMER0      (5U << 12)                      /*!< RW: Output selection CTIMER0 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_CTIMER1      (6U << 12)                      /*!< RW: Output selection CTIMER1 for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_SOFT         (7U << 12)                      /*!< RW: Output selection SOFTWARE for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_LIN          (8U << 12)                      /*!< RW: Output selection LIN for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_COLIN_TX     (9U << 12)                      /*!< RW: Output selection COLIN_TX for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_MOSI     (10U << 12)                     /*!< RW: Output selection SPI_MOSI for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_MISO     (11U << 12)                     /*!< RW: Output selection SPI_MISO for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_SCK      (12U << 12)                     /*!< RW: Output selection SPI_SCK for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_SPI_SS       (13U << 12)                     /*!< RW: Output selection SPI_SS for IO[3] */
#define C_PORT_IO_CFG0_IO3_OUT_SEL_PPM          (14U << 12)                     /*!< RW: Output selection PPM for IO[3] */
#define M_PORT_IO_CFG0_IO2_OUT_SEL              (15U << 8)                      /*!< RW: Output selection for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PWM_MSTR1    (0U << 8)                       /*!< RW: Output selection PWM_MASTER1 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PWM_SLV1     (1U << 8)                       /*!< RW: Output selection PWM_SLAVE1 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PWM_SLV2     (2U << 8)                       /*!< RW: Output selection PWM_SLAVE2 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PWM_SLV3     (3U << 8)                       /*!< RW: Output selection PWM_SLAVE3 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PWM_MSTR2    (4U << 8)                       /*!< RW: Output selection PWM_MASTER2 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_CTIMER0      (5U << 8)                       /*!< RW: Output selection CTIMER0 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_CTIMER1      (6U << 8)                       /*!< RW: Output selection CTIMER1 for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_SOFT         (7U << 8)                       /*!< RW: Output selection SOFTWARE for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_LIN          (8U << 8)                       /*!< RW: Output selection LIN for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_COLIN_TX     (9U << 8)                       /*!< RW: Output selection COLIN_TX for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_MOSI     (10U << 8)                      /*!< RW: Output selection SPI_MOSI for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_MISO     (11U << 8)                      /*!< RW: Output selection SPI_MISO for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_SCK      (12U << 8)                      /*!< RW: Output selection SPI_SCK for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_SPI_SS       (13U << 8)                      /*!< RW: Output selection SPI_SS for IO[2] */
#define C_PORT_IO_CFG0_IO2_OUT_SEL_PPM          (14U << 8)                      /*!< RW: Output selection PPM for IO[2] */
#define M_PORT_IO_CFG0_IO1_OUT_SEL              (15U << 4)                      /*!< RW: Output selection for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PWM_MSTR1    (0U << 4)                       /*!< RW: Output selection PWM_MASTER1 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PWM_SLV1     (1U << 4)                       /*!< RW: Output selection PWM_SLAVE1 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PWM_SLV2     (2U << 4)                       /*!< RW: Output selection PWM_SLAVE2 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PWM_SLV3     (3U << 4)                       /*!< RW: Output selection PWM_SLAVE3 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PWM_MSTR2    (4U << 4)                       /*!< RW: Output selection PWM_MASTER2 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_CTIMER0      (5U << 4)                       /*!< RW: Output selection CTIMER0 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_CTIMER1      (6U << 4)                       /*!< RW: Output selection CTIMER1 for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_SOFT         (7U << 4)                       /*!< RW: Output selection SOFTWARE for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_LIN          (8U << 4)                       /*!< RW: Output selection LIN for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_COLIN_TX     (9U << 4)                       /*!< RW: Output selection COLIN_TX for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_MOSI     (10U << 4)                      /*!< RW: Output selection SPI_MOSI for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_MISO     (11U << 4)                      /*!< RW: Output selection SPI_MISO for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_SCK      (12U << 4)                      /*!< RW: Output selection SPI_SCK for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_SPI_SS       (13U << 4)                      /*!< RW: Output selection SPI_SS for IO[1] */
#define C_PORT_IO_CFG0_IO1_OUT_SEL_PPM          (14U << 4)                      /*!< RW: Output selection PPM for IO[1] */
#define M_PORT_IO_CFG0_IO0_OUT_SEL              (15U << 0)                      /*!< RW: Output selection for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PWM_MSTR1    (0U << 0)                       /*!< RW: Output selection PWM_MASTER1 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PWM_SLV1     (1U << 0)                       /*!< RW: Output selection PWM_SLAVE1 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PWM_SLV2     (2U << 0)                       /*!< RW: Output selection PWM_SLAVE2 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PWM_SLV3     (3U << 0)                       /*!< RW: Output selection PWM_SLAVE3 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PWM_MSTR2    (4U << 0)                       /*!< RW: Output selection PWM_MASTER2 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_CTIMER0      (5U << 0)                       /*!< RW: Output selection CTIMER0 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_CTIMER1      (6U << 0)                       /*!< RW: Output selection CTIMER1 for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_SOFT         (7U << 0)                       /*!< RW: Output selection SOFTWARE for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_LIN          (8U << 0)                       /*!< RW: Output selection LIN for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_COLIN_TX     (9U << 0)                       /*!< RW: Output selection COLIN_TX for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_MOSI     (10U << 0)                      /*!< RW: Output selection SPI_MOSI for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_MISO     (11U << 0)                      /*!< RW: Output selection SPI_MISO for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_SCK      (12U << 0)                      /*!< RW: Output selection SPI_SCK for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_SPI_SS       (13U << 0)                      /*!< RW: Output selection SPI_SS for IO[0] */
#define C_PORT_IO_CFG0_IO0_OUT_SEL_PPM          (14U << 0)                      /*!< RW: Output selection PPM for IO[0] */

/* *********************** */
/* Block: PORT_LIN_XTX_CFG */
/* *********************** */
extern volatile uint16_t IO_PORT_LIN_XTX_CFG __attribute__((nodp, addr(0x001EC)));  /*!< IO_PORT_LIN_XTX_CFG */
#define B_PORT_LIN_XTX_CFG_LIN_IN_SOFT          (1U << 7)                       /*!< RW: Software port for LIN reception (connected via mux to LIN_XTX) */
#define B_PORT_LIN_XTX_CFG_LIN_OUT_SOFT         (1U << 6)                       /*!< RW: Software port for LIN transmission (connected via mux to LIN_XTX) */
#define M_PORT_LIN_XTX_CFG_LIN_COLIN_RX_SEL     (3U << 4)                       /*!< RW: Input selection for Colin RX input */
#define C_PORT_LIN_XTX_CFG_LIN_COLIN_RX_SEL_IO0 (0U << 4)                       /*!< RW: Input selection IO0 for Colin RX input */
#define C_PORT_LIN_XTX_CFG_LIN_COLIN_RX_SEL_IO1 (1U << 4)                       /*!< RW: Input selection IO1 for Colin RX input */
#define C_PORT_LIN_XTX_CFG_LIN_COLIN_RX_SEL_IO2 (2U << 4)                       /*!< RW: Input selection IO2 for Colin RX input */
#define C_PORT_LIN_XTX_CFG_LIN_COLIN_RX_SEL_IO3 (3U << 4)                       /*!< RW: Input selection IO3 for Colin RX input */
#define M_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL      (15U << 0)                      /*!< RW: Output selection for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_CTIMER0      (0U << 0)               /*!< RW: Output selection CTIMRE0 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_CTIMER1      (1U << 0)               /*!< RW: Output selection CTIMRE1 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PWM_MASTER1  (2U << 0)               /*!< RW: Output selection PWM_MASTER1 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PWM_SLAVE1   (3U << 0)               /*!< RW: Output selection PWM_SLAVE1 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PWM_SLAVE2   (4U << 0)               /*!< RW: Output selection PWM_SLAVE2 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PWM_SLAVE3   (5U << 0)               /*!< RW: Output selection PWM_SLAVE3 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PWM_MASTER2  (6U << 0)               /*!< RW: Output selection PWM_MASTER2 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_LIN_OUT_SOFT (7U << 0)               /*!< RW: Output selection SOFTWARE for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO0          (8U << 0)               /*!< RW: Output selection IO0 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO1          (9U << 0)               /*!< RW: Output selection IO1 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO2          (10U << 0)              /*!< RW: Output selection IO2 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_IO3          (11U << 0)              /*!< RW: Output selection IO3 for LIN_XTX */
#define C_PORT_LIN_XTX_CFG_LIN_XTX_OUT_SEL_PPM_OUT      (12U << 0)              /*!< RW: Output selection PPM for LIN_XTX */

/* ********************* */
/* Block: PORT_TIMER_CFG */
/* ********************* */
extern volatile uint16_t IO_PORT_TIMER_CFG __attribute__((nodp, addr(0x001EE)));  /*!< IO_PORT_TIMER_CFG */
#define M_PORT_TIMER_CFG_TIMER0_CHB_SEL         (31U << 8)                      /*!< RW: Input selection for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO0     (0U << 8)                       /*!< RW: Input selection IO0 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO1     (1U << 8)                       /*!< RW: Input selection IO1 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO2     (2U << 8)                       /*!< RW: Input selection IO2 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_IO3     (3U << 8)                       /*!< RW: Input selection IO3 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_LIN_XRX (4U << 8)                       /*!< RW: Input selection LIN_XRX for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PWM_MASTER1 (5U << 8)                   /*!< RW: Input selection PWM_MASTER1 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PWM_SLAVE1  (6U << 8)                   /*!< RW: Input selection PWM_SLAVE1 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PWM_SLAVE2  (7U << 8)                   /*!< RW: Input selection PWM_SLAVE2 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PWM_SLAVE3  (8U << 8)                   /*!< RW: Input selection PWM_SLAVE3 for timer[0] channel B */
#define C_PORT_TIMER_CFG_TIMER0_CHB_SEL_PWM_MASTER2 (9U << 8)                   /*!< RW: Input selection PWM_MASTER2 for timer[0] channel B */
#define M_PORT_TIMER_CFG_TIMER0_CHA_SEL         (31U << 0)                      /*!< RW: Input selection for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO0     (0U << 0)                       /*!< RW: Input selection IO0 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO1     (1U << 0)                       /*!< RW: Input selection IO1 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO2     (2U << 0)                       /*!< RW: Input selection IO2 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_IO3     (3U << 0)                       /*!< RW: Input selection IO3 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_LIN_XRX (4U << 0)                       /*!< RW: Input selection LIN_XRX for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PWM_MASTER1 (5U << 0)                   /*!< RW: Input selection PWM_MASTER1 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PWM_SLAVE1  (6U << 0)                   /*!< RW: Input selection PWM_SLAVE1 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PWM_SLAVE2  (7U << 0)                   /*!< RW: Input selection PWM_SLAVE2 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PWM_SLAVE3  (8U << 0)                   /*!< RW: Input selection PWM_SLAVE3 for timer[0] channel A */
#define C_PORT_TIMER_CFG_TIMER0_CHA_SEL_PWM_MASTER2 (9U << 0)                   /*!< RW: Input selection PWM_MASTER2 for timer[0] channel A */

/* ******************** */
/* Block: PORT_COMM_CFG */
/* ******************** */
extern volatile uint16_t IO_PORT_COMM_CFG __attribute__((nodp, addr(0x001F0)));  /*!< IO_PORT_COMM_CFG */
#define M_PORT_COMM_CFG_SPI_SS_IN_SEL           (3U << 12)                      /*!< RW: IO selection for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO0       (0U << 12)                      /*!< RW: IO[0] selection for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO1       (1U << 12)                      /*!< RW: IO[1] selection for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO2       (2U << 12)                      /*!< RW: IO[2] selection for SPI_SS_IN */
#define C_PORT_COMM_CFG_SPI_SS_IN_SEL_IO3       (3U << 12)                      /*!< RW: IO[3] selection for SPI_SS_IN */
#define M_PORT_COMM_CFG_SPI_SCK_IN_SEL          (3U << 8)                       /*!< RW: IO selection for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO0      (0U << 8)                       /*!< RW: IO[0] selection for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO1      (1U << 8)                       /*!< RW: IO[1] selection for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO2      (2U << 8)                       /*!< RW: IO[2] selection for SPI_SCK_IN */
#define C_PORT_COMM_CFG_SPI_SCK_IN_SEL_IO3      (3U << 8)                       /*!< RW: IO[3] selection for SPI_SCK_IN */
#define M_PORT_COMM_CFG_SPI_MISO_IN_SEL         (3U << 4)                       /*!< RW: IO selection for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO0     (0U << 4)                       /*!< RW: IO[0] selection for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO1     (1U << 4)                       /*!< RW: IO[1] selection for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO2     (2U << 4)                       /*!< RW: IO[2] selection for SPI_MISO_IN */
#define C_PORT_COMM_CFG_SPI_MISO_IN_SEL_IO3     (3U << 4)                       /*!< RW: IO[3] selection for SPI_MISO_IN */
#define M_PORT_COMM_CFG_SPI_MOSI_IN_SEL         (3U << 0)                       /*!< RW: IO selection for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO0     (0U << 0)                       /*!< RW: IO[0] selection for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO1     (1U << 0)                       /*!< RW: IO[1] selection for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO2     (2U << 0)                       /*!< RW: IO[2] selection for SPI_MOSI_IN */
#define C_PORT_COMM_CFG_SPI_MOSI_IN_SEL_IO3     (3U << 0)                       /*!< RW: IO[3] selection for SPI_MOSI_IN */

/* ******************** */
/* Block: PORT_MISC_OUT */
/* ******************** */
extern volatile uint16_t IO_PORT_MISC_OUT __attribute__((nodp, addr(0x001F2)));  /*!< IO_PORT_MISC_OUT */
#define M_PORT_MISC_OUT_SEL_TEMP                (15U << 12)                     /*!< RW: select the temperature diode input for the temp. sensor */
#define C_PORT_MISC_OUT_SEL_TEMP_MAIN           (8U << 12)                      /*!< Channel 8: VBGA Temperature sensor */
#define B_PORT_MISC_OUT_PRUV_VDDA               (1U << 11)                      /*!< RW: under voltage programming for VDDA */
#define M_PORT_MISC_OUT_PROV_VS                 (3U << 9)                       /*!< RW: control for VS over voltage monitor */
#define C_PORT_MISC_OUT_PROV_0                  (1U << 9)                       /*!< RW: control for VS over voltage unit */
#define M_PORT_MISC_OUT_PRUV_VS                 (7U << 6)                       /*!< RW: under voltage programming for VS; detection level (V) = (PRUV_VS+4) -> from 4 to 9V */
#define C_PORT_MISC_OUT_PRUV_0                  (1U << 6)                       /*!< RW: under voltage programming for VS-unit */
#define B_PORT_MISC_OUT_SWITCH_VDDA_TO_5V       (1U << 5)                       /*!< RW: switch VDDA supply to 5V */
#define M_PORT_MISC_OUT_WUI                     (3U << 3)                       /*!< RW: define internal wake up time delay in periodes of CK10K : */
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
extern volatile uint16_t IO_PORT_MISC2_OUT __attribute__((nodp, addr(0x001F4)));  /*!< IO_PORT_MISC2_OUT */
#define B_PORT_MISC2_OUT_VSM_FILT_ON            (1U << 10)                      /*!< RW: Value for analogue */
#define B_PORT_MISC2_OUT_ENABLE_OTD             (1U << 9)                       /*!< RW: Enable Over Temperature Detector */
#define B_PORT_MISC2_OUT_WU_IO_EN               (1U << 8)                       /*!< RW: wake up enable for IOs */
#define M_PORT_MISC2_OUT_AOUT_SUP               (15U << 4)                      /*!< RW: spare control output bits for supply system */
#define M_PORT_MISC2_OUT_UNUSED                 (3U << 2)                       /*!< RW: Value for analogue */
#define M_PORT_MISC2_OUT_ENABLE_OSD_DRV         (3U << 0)                       /*!< RW: Value for analogue */

/* *********************** */
/* Block: PORT_STOPMD_CTRL */
/* *********************** */
extern volatile uint16_t IO_PORT_STOPMD_CTRL_S __attribute__((nodp, addr(0x001F6)));  /*!< IO_PORT_STOPMD_CTRL_S (System) */
#define B_PORT_STOPMD_CTRL_SEL_STOP_MODE        (1U << 0)                       /*!< RW: CPU HALT will enter STOPMODE, not SLEEP */

/* ********************** */
/* Block: PORT_STOPMD_CFG */
/* ********************** */
extern volatile uint16_t IO_PORT_STOPMD_CFG __attribute__((nodp, addr(0x001F8)));  /*!< IO_PORT_STOPMD_CFG */
#define B_PORT_STOPMD_CFG_PRE_SBY_RCO1M         (1U << 4)                       /*!< RW: set 1 MHz Oscillator in standby mode */
#define B_PORT_STOPMD_CFG_PRE_SBY_RCO32M        (1U << 3)                       /*!< RW: set 32 MHz Oscillator in standby mode */
#define B_PORT_STOPMD_CFG_PRE_SWITCHOFFOV_VS    (1U << 2)                       /*!< RW: disable over voltage detection for VS */
#define B_PORT_STOPMD_CFG_PRE_SWITCHOFFUV_VS    (1U << 1)                       /*!< RW: disable under voltage detection for VS */
#define B_PORT_STOPMD_CFG_PRE_SWITCHOFFUV_VDDA  (1U << 0)                       /*!< RW: disable under voltage detection of VDDA regulator to allow low voltage test */

/* ******************** */
/* Block: PORT_DIS_GTSM */
/* ******************** */
extern volatile uint16_t IO_PORT_DIS_GTSM __attribute__((nodp, addr(0x001FA)));  /*!< IO_PORT_DIS_GTSM */
#define B_PORT_DIS_GTSM_DIS_GTSM                (1U << 0)                       /*!< RW: disable GTSM, if set, HALTED will not trigger a GTSM signal */

/* ******************** */
/* Block: PORT_LIN_XCFG */
/* ******************** */
extern volatile uint16_t IO_PORT_LIN_XCFG_S __attribute__((nodp, addr(0x001FC)));  /*!< IO_PORT_LIN_XCFG_S (System) */
#define M_PORT_LIN_XCFG_LIN_XCFG                (65535U << 0)                   /*!< RW: result taken in account only if XKEY is valid */
#define B_PORT_LIN_XCFG_LIN_TEST_WARM           (1U << 15)                      /*!< Enable test mode warm activation over LIN pin */
#define B_PORT_LIN_XCFG_DIS_TX_TIMEOUT          (1U << 14)                      /*!< Disable time-out protection (10-20ms) on TX output */
#define B_PORT_LIN_XCFG_CXPI_DIS_WU_DEB         (1U << 13)                      /*!< Decrease the debounce time of the wake-up comparator from 70us to 5.5us to support CXPI protocol */
#define B_PORT_LIN_XCFG_ENA_LIN_REV_PROT        (1U << 12)                      /*!< Disconnects the reverse polarity protection from internal LIN node, is needed to measure LIN level by ADC or to run fast protocol at 5V level (PPM, CXPI) */
#define B_PORT_LIN_XCFG_SEL_RXD_ATDI            (1U << 11)                      /*!< 1: the fast comparator used in test mode (ATDI) will be switched to the RX input (this allows protocol with higher baudrate, e.g. PPM, FASTLIN or CXPI) */
#define B_PORT_LIN_XCFG_RX_INVERT               (1U << 10)                      /*!< Invert the RX input before any multiplexing */
#define B_PORT_LIN_XCFG_DISTERM                 (1U << 9)                       /*!< Disable bus termination for auto-addressing */
#define B_PORT_LIN_XCFG_BYPASS                  (1U << 8)                       /*!< Bypass the receiver for high-speed mode */
#define B_PORT_LIN_XCFG_HSM                     (1U << 7)                       /*!< High-speed mode (slew rate disabled) */
#define B_PORT_LIN_XCFG_LSM                     (1U << 6)                       /*!< Low speed slope control */
#define B_PORT_LIN_XCFG_SLEEPB                  (1U << 5)                       /*!< Disable sleep mode */
#define B_PORT_LIN_XCFG_SEL_COLIN_B             (1U << 4)                       /*!< Select Colin-B */
#define B_PORT_LIN_XCFG_SEL_IO_TO_COLINRX       (1U << 3)                       /*!< Select COLIN_RX driven from IO */
#define B_PORT_LIN_XCFG_SEL_RX_IO               (1U << 2)                       /*!< Select RX driven from IO */
#define B_PORT_LIN_XCFG_TX_INVERT               (1U << 1)                       /*!< Invert TX output */
#define B_PORT_LIN_XCFG_SEL_TX_EXT              (1U << 0)                       /*!< Select TX driver from IO */

/* ************************** */
/* Block: PORT_LIN_XCFG_VALID */
/* ************************** */
extern volatile uint16_t IO_PORT_LIN_XCFG_VALID __attribute__((nodp, addr(0x001FE)));  /*!< IO_PORT_LIN_XCFG_VALID */
#define M_PORT_LIN_XCFG_VALID_LIN_XCFG_VALID    (65535U << 0)                   /*!< R: Value for digital read */

/* ********************** */
/* Block: PORT_CLOCK_CTRL */
/* ********************** */
extern volatile uint16_t IO_PORT_CLOCK_CTRL_S __attribute__((nodp, addr(0x00200)));  /*!< IO_PORT_CLOCK_CTRL_S (System) */
#define B_PORT_CLOCK_CTRL_AC_SEL                (1U << 0)                       /*!< RW: switch to 16 MHz clock if set */

/* ****************** */
/* Block: PORT_LINAA1 */
/* ****************** */
extern volatile uint16_t IO_PORT_LINAA1 __attribute__((nodp, addr(0x00202)));   /*!< IO_PORT_LINAA1 */
#define B_PORT_LINAA1_LINAA_CDOUTEN             (1U << 12)                      /*!< RW: Switch of the monitor current for LED monitoring or LINAA */
#define B_PORT_LINAA1_LINAA_EN                  (1U << 11)                      /*!< RW: Enable OpAmp of the LINAA amplifier */
#define B_PORT_LINAA1_LINAA_RST2                (1U << 10)                      /*!< RW: Reset second variable gain amplifier */
#define B_PORT_LINAA1_LINAA_RST1                (1U << 9)                       /*!< RW: Reset first amplifier gain = 40 */
#define M_PORT_LINAA1_LINAA_DIV                 (31U << 4)                      /*!< RW: Common mode suppression adjustments bits */
#define M_PORT_LINAA1_LINAA_GAIN                (15U << 0)                      /*!< RW: Gain control bits of the variable gain amp */

/* ****************** */
/* Block: PORT_LINAA2 */
/* ****************** */
extern volatile uint16_t IO_PORT_LINAA2 __attribute__((nodp, addr(0x00204)));   /*!< IO_PORT_LINAA2 */
#define B_PORT_LINAA2_LCD_DIS_LINAA             (1U << 4)                       /*!< RW: Value for analogue */
#define B_PORT_LINAA2_LCD_ON_LINAA              (1U << 3)                       /*!< RW: Value for analogue */
#define M_PORT_LINAA2_LCD_SEL_LINAA             (7U << 0)                       /*!< RW: Value for analogue */

/* **************** */
/* Block: TRIM_MISC */
/* **************** */
extern volatile uint16_t IO_TRIM_MISC __attribute__((nodp, addr(0x00206)));     /*!< IO_TRIM_MISC */
#define B_TRIM_MISC_LOCK                        (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_TRIM_MISC_TRIM_SDAFILT_IO             (3U << 12)                      /*!< RW: Calibration. Write is invalid if LOCK is set */
#define M_TRIM_MISC_TRIM_OTD                    (63U << 6)                      /*!< RW: Calibration. Write is invalid if LOCK is set */
#define M_TRIM_MISC_TRIM_LCD_LINAA              (63U << 0)                      /*!< RW: Calibration. Write is invalid if LOCK is set */

/* **************** */
/* Block: TRIM1_DRV */
/* **************** */
extern volatile uint16_t IO_TRIM1_DRV __attribute__((nodp, addr(0x00208)));     /*!< IO_TRIM1_DRV */
#define B_TRIM1_DRV_LOCK                        (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_TRIM1_DRV_PRE_TRIM_DRVMOD_CPCLK       (1023U << 2)                    /*!< RW: trim frequency of driver clock */
#define M_TRIM1_DRV_TRIM_DRVSUP                 (3U << 0)                       /*!< RW: trim output level of driver supply */

/* **************** */
/* Block: TRIM2_DRV */
/* **************** */
extern volatile uint16_t IO_TRIM2_DRV __attribute__((nodp, addr(0x0020A)));     /*!< IO_TRIM2_DRV */
#define B_TRIM2_DRV_LOCK                        (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_TRIM2_DRV_TRIM_CSA_GAIN               (31U << 4)                      /*!< RW: trim gain of current sense amplifier */
#define M_TRIM2_DRV_TRIM_SLWRT                  (15U << 0)                      /*!< RW: trim slew/rate / slope of drivers */

/* **************** */
/* Block: TRIM3_DRV */
/* **************** */
extern volatile uint16_t IO_TRIM3_DRV __attribute__((nodp, addr(0x0020C)));     /*!< IO_TRIM3_DRV */
#define B_TRIM3_DRV_LOCK                        (1U << 15)                      /*!< RW: Lock the port. Write is invalid when set */
#define M_TRIM3_DRV_TRIM_CSA_CL                 (255U << 0)                     /*!< RW: trim over-current limit of current sense amplifier */

/* ******************* */
/* Block: PORT_DRV_OUT */
/* ******************* */
extern volatile uint16_t IO_PORT_DRV_OUT __attribute__((nodp, addr(0x0020E)));  /*!< IO_PORT_DRV_OUT */
#define B_PORT_DRV_OUT_PARALLEL_MODE_DRV        (1U << 11)                      /*!< RW: Phase UV and WT switch parallel for cross-current detection (DC-mode) */
#define M_PORT_DRV_OUT_DRVMOD_OPTION            (3U << 9)                       /*!< RW: select divider ratio of driver clock
                                                                                 * 0b00: No division for CPCLK
                                                                                 * 0b01: CPCLK is constantly divided by 4
                                                                                 * 0b10: CPCLK starts at full speed, then is divided by 4, ~3us after enabling a HS FET
                                                                                 * 0b11: CPCLK starts at full speed, then is divided by 4, ~7us after enabling a HS FET */
#define B_PORT_DRV_OUT_ENABLE_LS_OC             (1U << 8)                       /*!< RW: enable low-side FET VDS over-voltage / over-current detection */
#define B_PORT_DRV_OUT_ENABLE_HS_OC             (1U << 7)                       /*!< RW: enable high-side FET VDS over-voltage / over-current detection */
#define B_PORT_DRV_OUT_ENABLE_CSA               (1U << 6)                       /*!< RW: enable current sense amplifier */
#define M_PORT_DRV_OUT_ENABLE_DRV               (15U << 2)                      /*!< RW: enable output stages */
#define B_PORT_DRV_OUT_ENABLE_DRV0              ((1U << 0) << 2)                /*!< RW: enable output stages DRV0 */
#define B_PORT_DRV_OUT_ENABLE_DRV1              ((1U << 1) << 2)                /*!< RW: enable output stages DRV1 */
#define B_PORT_DRV_OUT_ENABLE_DRV2              ((1U << 2) << 2)                /*!< RW: enable output stages DRV2 */
#define B_PORT_DRV_OUT_ENABLE_DRV3              ((1U << 3) << 2)                /*!< RW: enable output stages DRV3 */
#define B_PORT_DRV_OUT_ENABLE_DRVMOD_CPCLK      (1U << 1)                       /*!< RW: enable driver clock */
#define B_PORT_DRV_OUT_ENABLE_DRVSUP            (1U << 0)                       /*!< RW: enable driver supply */

/* ********************* */
/* Block: PORT_DRV1_TEST */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV1_TEST __attribute__((nodp, addr(0x00210)));  /*!< IO_PORT_DRV1_TEST */
#define B_PORT_DRV1_TEST_TST_VBGCS_LV0_DRVSUP   (1U << 15)                      /*!< RW: Value for analogue */
#define B_PORT_DRV1_TEST_TST_UVCMP_LV1_DRVSUP   (1U << 14)                      /*!< RW: Value for analogue */
#define M_PORT_DRV1_TEST_TST_OUT_LV1            (15U << 10)                     /*!< RW: Value for analogue */
#define B_PORT_DRV1_TEST_TST_STRS_VHS_LS_DRV    (1U << 9)                       /*!< RW: Value for analogue */
#define B_PORT_DRV1_TEST_TST_STRS_VHS_HS_DRV    (1U << 8)                       /*!< RW: Value for analogue */
#define M_PORT_DRV1_TEST_TST_LS_OC_ISENSE       (15U << 4)                      /*!< RW: Value for analogue */
#define M_PORT_DRV1_TEST_TST_HS_OC_ISENSE       (15U << 0)                      /*!< RW: Value for analogue */

/* ********************* */
/* Block: PORT_DRV2_TEST */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV2_TEST __attribute__((nodp, addr(0x00212)));  /*!< IO_PORT_DRV2_TEST */
#define B_PORT_DRV2_TEST_TST_DRVSUP_DISC_CMP    (1U << 13)                      /*!< RW: Value for analogue */
#define B_PORT_DRV2_TEST_TST_OC_REDUCE_LVL      (1U << 12)                      /*!< RW: Value for analogue */
#define M_PORT_DRV2_TEST_TEST_DRV_ILD           (3U << 10)                      /*!< RW: Value for analogue */
#define B_PORT_DRV2_TEST_TEST_DRVMOD_CPCLK      (1U << 9)                       /*!< RW: Value for analogue */
#define B_PORT_DRV2_TEST_TST_DRVSUP             (1U << 8)                       /*!< RW: Value for analogue */
#define M_PORT_DRV2_TEST_TST_OUT_LV0            (15U << 4)                      /*!< RW: Value for analogue */
#define M_PORT_DRV2_TEST_TST_OUT_HV0            (15U << 0)                      /*!< RW: Value for analogue */

/* ********************* */
/* Block: PORT_DRV3_TEST */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV3_TEST __attribute__((nodp, addr(0x00214)));  /*!< IO_PORT_DRV3_TEST */
#define B_PORT_DRV3_TEST_TST_DRVMOD_CPCLK_DAC   (1U << 11)                      /*!< RW: Value for analogue */
#define B_PORT_DRV3_TEST_TST_V5V_DIV3           (1U << 10)                      /*!< RW: Value for analogue */
#define B_PORT_DRV3_TEST_TST_DAC_LV0_CSA        (1U << 9)                       /*!< RW: Value for analogue */
#define B_PORT_DRV3_TEST_TST_AMP_DIV_LV0_CSA    (1U << 8)                       /*!< RW: Value for analogue */
#define B_PORT_DRV3_TEST_TST_CSA_OC             (1U << 7)                       /*!< RW: Value for analogue */
#define B_PORT_DRV3_TEST_TST_CSA_FR             (1U << 6)                       /*!< RW: Value for analogue */
#define B_PORT_DRV3_TEST_TST_CSA_RS             (1U << 5)                       /*!< RW: Value for analogue */
#define B_PORT_DRV3_TEST_TST_HS_GHS_IBOOST_HRDOFFSW (1U << 4)                   /*!< RW: Value for analogue */
#define M_PORT_DRV3_TEST_TST_GHS_NOCURR         (15U << 0)                      /*!< RW: Value for analogue */

/* ******************** */
/* Block: PORT_DRV_CTRL */
/* ******************** */
extern volatile uint16_t IO_PORT_DRV_CTRL __attribute__((nodp, addr(0x00216)));  /*!< IO_PORT_DRV_CTRL */
#define M_PORT_DRV_CTRL_DRV3_CTRL               (15U << 12)                     /*!< RW: Control of PWM output for phase 3 */
#define C_PORT_DRV_CTRL_DRV3_MASTER1            (0U << 12)                      /*!< RW: Control DRV3 by PWM_MASTER1 */
#define C_PORT_DRV_CTRL_DRV3_SLAVE1             (1U << 12)                      /*!< RW: Control DRV3 by PWM_SLAVE1 */
#define C_PORT_DRV_CTRL_DRV3_SLAVE2             (2U << 12)                      /*!< RW: Control DRV3 by PWM_SLAVE2 */
#define C_PORT_DRV_CTRL_DRV3_SLAVE3             (3U << 12)                      /*!< RW: Control DRV3 by PWM_SLAVE3 */
#define C_PORT_DRV_CTRL_DRV3_MASTER2            (4U << 12)                      /*!< RW: Control DRV3 by PWM_MASTER2 */
#define C_PORT_DRV_CTRL_DRV3_IO0                (5U << 12)                      /*!< RW: Control DRV3 by IO0 */
#define C_PORT_DRV_CTRL_DRV3_IO1                (6U << 12)                      /*!< RW: Control DRV3 by IO1 */
#define C_PORT_DRV_CTRL_DRV3_IO2                (7U << 12)                      /*!< RW: Control DRV3 by IO2 */
#define C_PORT_DRV_CTRL_DRV3_IO3                (8U << 12)                      /*!< RW: Control DRV3 by IO3 */
#define C_PORT_DRV_CTRL_DRV3_LIN_XRX            (9U << 12)                      /*!< RW: Control DRV3 by LIN_XRX */
#define C_PORT_DRV_CTRL_DRV3_CTIMER0            (10U << 12)                     /*!< RW: Control DRV3 by CTIMER0 */
#define C_PORT_DRV_CTRL_DRV3_CTIMER1            (11U << 12)                     /*!< RW: Control DRV3 by CTIMER1 */
#define C_PORT_DRV_CTRL_DRV3_L                  (13U << 12)                     /*!< RW: Control DRV3 LOW */
#define C_PORT_DRV_CTRL_DRV3_TRISTATE           (14U << 12)                     /*!< RW: Control DRV3 TRI-STATE */
#define C_PORT_DRV_CTRL_DRV3_H                  (15U << 12)                     /*!< RW: Control DRV3 HIGH */
#define M_PORT_DRV_CTRL_DRV2_CTRL               (15U << 8)                      /*!< RW: Control of PWM output for phase 2 */
#define C_PORT_DRV_CTRL_DRV2_MASTER1            (0U << 8)                       /*!< RW: Control DRV2 by PWM_MASTER1 */
#define C_PORT_DRV_CTRL_DRV2_SLAVE1             (1U << 8)                       /*!< RW: Control DRV2 by PWM_SLAVE1 */
#define C_PORT_DRV_CTRL_DRV2_SLAVE2             (2U << 8)                       /*!< RW: Control DRV2 by PWM_SLAVE2 */
#define C_PORT_DRV_CTRL_DRV2_SLAVE3             (3U << 8)                       /*!< RW: Control DRV2 by PWM_SLAVE3 */
#define C_PORT_DRV_CTRL_DRV2_MASTER2            (4U << 8)                       /*!< RW: Control DRV2 by PWM_MASTER2 */
#define C_PORT_DRV_CTRL_DRV2_IO0                (5U << 8)                       /*!< RW: Control DRV2 by IO0 */
#define C_PORT_DRV_CTRL_DRV2_IO1                (6U << 8)                       /*!< RW: Control DRV2 by IO1 */
#define C_PORT_DRV_CTRL_DRV2_IO2                (7U << 8)                       /*!< RW: Control DRV2 by IO2 */
#define C_PORT_DRV_CTRL_DRV2_IO3                (8U << 8)                       /*!< RW: Control DRV2 by IO3 */
#define C_PORT_DRV_CTRL_DRV2_LIN_XRX            (9U << 8)                       /*!< RW: Control DRV2 by LIN_XRX */
#define C_PORT_DRV_CTRL_DRV2_CTIMER0            (10U << 8)                      /*!< RW: Control DRV2 by CTIMER0 */
#define C_PORT_DRV_CTRL_DRV2_CTIMER1            (11U << 8)                      /*!< RW: Control DRV2 by CTIMER1 */
#define C_PORT_DRV_CTRL_DRV2_L                  (13U << 8)                      /*!< RW: Control DRV2 LOW */
#define C_PORT_DRV_CTRL_DRV2_TRISTATE           (14U << 8)                      /*!< RW: Control DRV2 TRI-STATE */
#define C_PORT_DRV_CTRL_DRV2_H                  (15U << 8)                      /*!< RW: Control DRV2 HIGH */
#define M_PORT_DRV_CTRL_DRV1_CTRL               (15U << 4)                      /*!< RW: Control of PWM output for phase 1 */
#define C_PORT_DRV_CTRL_DRV1_MASTER1            (0U << 4)                       /*!< RW: Control DRV1 by PWM_MASTER1 */
#define C_PORT_DRV_CTRL_DRV1_SLAVE1             (1U << 4)                       /*!< RW: Control DRV1 by PWM_SLAVE1 */
#define C_PORT_DRV_CTRL_DRV1_SLAVE2             (2U << 4)                       /*!< RW: Control DRV1 by PWM_SLAVE2 */
#define C_PORT_DRV_CTRL_DRV1_SLAVE3             (3U << 4)                       /*!< RW: Control DRV1 by PWM_SLAVE3 */
#define C_PORT_DRV_CTRL_DRV1_MASTER2            (4U << 4)                       /*!< RW: Control DRV1 by PWM_MASTER2 */
#define C_PORT_DRV_CTRL_DRV1_IO0                (5U << 4)                       /*!< RW: Control DRV1 by IO0 */
#define C_PORT_DRV_CTRL_DRV1_IO1                (6U << 4)                       /*!< RW: Control DRV1 by IO1 */
#define C_PORT_DRV_CTRL_DRV1_IO2                (7U << 4)                       /*!< RW: Control DRV1 by IO2 */
#define C_PORT_DRV_CTRL_DRV1_IO3                (8U << 4)                       /*!< RW: Control DRV1 by IO3 */
#define C_PORT_DRV_CTRL_DRV1_LIN_XRX            (9U << 4)                       /*!< RW: Control DRV1 by LIN_XRX */
#define C_PORT_DRV_CTRL_DRV1_CTIMER0            (10U << 4)                      /*!< RW: Control DRV1 by CTIMER0 */
#define C_PORT_DRV_CTRL_DRV1_CTIMER1            (11U << 4)                      /*!< RW: Control DRV1 by CTIMER1 */
#define C_PORT_DRV_CTRL_DRV1_L                  (13U << 4)                      /*!< RW: Control DRV1 LOW */
#define C_PORT_DRV_CTRL_DRV1_TRISTATE           (14U << 4)                      /*!< RW: Control DRV1 TRI-STATE */
#define C_PORT_DRV_CTRL_DRV1_H                  (15U << 4)                      /*!< RW: Control DRV1 HIGH */
#define M_PORT_DRV_CTRL_DRV0_CTRL               (15U << 0)                      /*!< RW: Control of PWM output for phase 0 */
#define C_PORT_DRV_CTRL_DRV0_MASTER1            (0U << 0)                       /*!< RW: Control DRV0 by PWM_MASTER1 */
#define C_PORT_DRV_CTRL_DRV0_SLAVE1             (1U << 0)                       /*!< RW: Control DRV0 by PWM_SLAVE1 */
#define C_PORT_DRV_CTRL_DRV0_SLAVE2             (2U << 0)                       /*!< RW: Control DRV0 by PWM_SLAVE2 */
#define C_PORT_DRV_CTRL_DRV0_SLAVE3             (3U << 0)                       /*!< RW: Control DRV0 by PWM_SLAVE3 */
#define C_PORT_DRV_CTRL_DRV0_MASTER2            (4U << 0)                       /*!< RW: Control DRV0 by PWM_MASTER2 */
#define C_PORT_DRV_CTRL_DRV0_IO0                (5U << 0)                       /*!< RW: Control DRV0 by IO0 */
#define C_PORT_DRV_CTRL_DRV0_IO1                (6U << 0)                       /*!< RW: Control DRV0 by IO1 */
#define C_PORT_DRV_CTRL_DRV0_IO2                (7U << 0)                       /*!< RW: Control DRV0 by IO2 */
#define C_PORT_DRV_CTRL_DRV0_IO3                (8U << 0)                       /*!< RW: Control DRV0 by IO3 */
#define C_PORT_DRV_CTRL_DRV0_LIN_XRX            (9U << 0)                       /*!< RW: Control DRV0 by LIN_XRX */
#define C_PORT_DRV_CTRL_DRV0_CTIMER0            (10U << 0)                      /*!< RW: Control DRV0 by CTIMER0 */
#define C_PORT_DRV_CTRL_DRV0_CTIMER1            (11U << 0)                      /*!< RW: Control DRV0 by CTIMER1 */
#define C_PORT_DRV_CTRL_DRV0_L                  (13U << 0)                      /*!< RW: Control DRV0 LOW */
#define C_PORT_DRV_CTRL_DRV0_TRISTATE           (14U << 0)                      /*!< RW: Control DRV0 TRI-STATE */
#define C_PORT_DRV_CTRL_DRV0_H                  (15U << 0)                      /*!< RW: Control DRV0 HIGH */

/* ********************* */
/* Block: PORT_DRV1_PROT */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV1_PROT __attribute__((nodp, addr(0x00218)));  /*!< IO_PORT_DRV1_PROT */
#define B_PORT_DRV1_PROT_DIS_OV_LS_VDS          (1U << 15)                      /*!< RW: Disable Low Side VDS Over-voltage HW-protection */
#define B_PORT_DRV1_PROT_OV_LS_VDS_PM           (1U << 14)                      /*!< RW: Protection mode: 0: Switch HS; 1: Tri-state */
#define B_PORT_DRV1_PROT_DIS_OV_HS_VDS          (1U << 13)                      /*!< RW: Disable High Side VDS Over-voltage HW-protection */
#define B_PORT_DRV1_PROT_OV_HS_VDS_PM           (1U << 12)                      /*!< RW: Protection mode: 0: Switch LS; 1: Tri-state */
#define B_PORT_DRV1_PROT_DIS_OC                 (1U << 11)                      /*!< RW: Disable Over Current HW-protection */
#define B_PORT_DRV1_PROT_OC_PM                  (1U << 10)                      /*!< RW: Protection mode: 0: Switch LS; 1: Tri-state */
#define B_PORT_DRV1_PROT_DIS_UV_VDDAF           (1U << 9)                       /*!< RW: Disable Under Voltage VDDAF HW-protection */
#define B_PORT_DRV1_PROT_UV_VDDAF_PM            (1U << 8)                       /*!< RW: Protection mode: 0: Switch LS; 1: Tri-state */
#define B_PORT_DRV1_PROT_DIS_UV_VDDA            (1U << 7)                       /*!< RW: Disable Under Voltage VDDA HW-protection */
#define B_PORT_DRV1_PROT_UV_VDDA_PM             (1U << 6)                       /*!< RW: Protection mode: 0: Switch LS; 1: Tri-state */
#define B_PORT_DRV1_PROT_DIS_UV_VS              (1U << 5)                       /*!< RW: Disable Under Voltage VS HW-protection */
#define B_PORT_DRV1_PROT_UV_VS_PM               (1U << 4)                       /*!< RW: Protection mode: 0: Switch LS; 1: Tri-state */
#define B_PORT_DRV1_PROT_DIS_OV_VS              (1U << 3)                       /*!< RW: Disable Over Voltage VS HW-protection */
#define B_PORT_DRV1_PROT_OV_VS_PM               (1U << 2)                       /*!< RW: Protection mode: 0: Switch LS; 1: Tri-state */
#define B_PORT_DRV1_PROT_DIS_OVT                (1U << 1)                       /*!< RW: Disable Over Temperature HW-protection */
#define B_PORT_DRV1_PROT_OVT_PM                 (1U << 0)                       /*!< RW: Protection mode: 0: Switch LS; 1: Tri-state */

/* ********************* */
/* Block: PORT_DRV2_PROT */
/* ********************* */
extern volatile uint16_t IO_PORT_DRV2_PROT __attribute__((nodp, addr(0x0021A)));  /*!< IO_PORT_DRV2_PROT */
#define B_PORT_DRV2_PROT_DIS_DRV                (1U << 0)                       /*!< RW: Disable drivers */

/* ****************** */
/* Block: PORT_DRV_IN */
/* ****************** */
extern volatile uint16_t IO_PORT_DRV_IN __attribute__((nodp, addr(0x0021C)));   /*!< IO_PORT_DRV_IN */
#define B_PORT_DRV_IN_OV_LS_VDS3_IT             (1U << 15)                      /*!< R: After filtering and masking */
#define B_PORT_DRV_IN_OV_LS_VDS2_IT             (1U << 14)                      /*!< R: After filtering and masking */
#define B_PORT_DRV_IN_OV_LS_VDS1_IT             (1U << 13)                      /*!< R: After filtering and masking */
#define B_PORT_DRV_IN_OV_LS_VDS0_IT             (1U << 12)                      /*!< R: After filtering and masking */
#define M_PORT_DRV_IN_OV_LS_VDS_SYNC            (15U << 8)                      /*!< R: Before digital filtering */
#define B_PORT_DRV_IN_OV_LS_VDS_SYNC_VDS0       ((1U << 0) << 8)                /*!< LS VDS Sync VDS0 */
#define B_PORT_DRV_IN_OV_LS_VDS_SYNC_VDS1       ((1U << 1) << 8)                /*!< LS VDS Sync VDS1 */
#define B_PORT_DRV_IN_OV_LS_VDS_SYNC_VDS2       ((1U << 2) << 8)                /*!< LS VDS Sync VDS2 */
#define B_PORT_DRV_IN_OV_LS_VDS_SYNC_VDS3       ((1U << 3) << 8)                /*!< LS VDS Sync VDS3 */
#define B_PORT_DRV_IN_OV_HS_VDS3_IT             (1U << 7)                       /*!< R: After filtering and masking */
#define B_PORT_DRV_IN_OV_HS_VDS2_IT             (1U << 6)                       /*!< R: After filtering and masking */
#define B_PORT_DRV_IN_OV_HS_VDS1_IT             (1U << 5)                       /*!< R: After filtering and masking */
#define B_PORT_DRV_IN_OV_HS_VDS0_IT             (1U << 4)                       /*!< R: After filtering and masking */
#define M_PORT_DRV_IN_OV_HS_VDS_SYNC            (15U << 0)                      /*!< R: Before digital filtering */
#define B_PORT_DRV_IN_OV_HS_VDS_SYNC_VDS0       ((1U << 0) << 0)                /*!< HS VDS Sync VDS0 */
#define B_PORT_DRV_IN_OV_HS_VDS_SYNC_VDS1       ((1U << 1) << 0)                /*!< HS VDS Sync VDS1 */
#define B_PORT_DRV_IN_OV_HS_VDS_SYNC_VDS2       ((1U << 2) << 0)                /*!< HS VDS Sync VDS2 */
#define B_PORT_DRV_IN_OV_HS_VDS_SYNC_VDS3       ((1U << 3) << 0)                /*!< HS VDS Sync VDS3 */

/* ******************* */
/* Block: PORT_DIAG_IN */
/* ******************* */
extern volatile uint16_t IO_PORT_DIAG_IN __attribute__((nodp, addr(0x0021E)));  /*!< IO_PORT_DIAG_IN */
#define M_PORT_DIAG_IN_OV_LS_VDS_MEM            (15U << 10)                     /*!< R: Cleared if dis_ov_ls_vds */
#define B_PORT_DIAG_IN_OV_LS_VDS_MEM_VDS0       ((1U << 0) << 10)               /*!< R: Cleared if dis_ov_ls_vds0 */
#define B_PORT_DIAG_IN_OV_LS_VDS_MEM_VDS1       ((1U << 1) << 10)               /*!< R: Cleared if dis_ov_ls_vds1 */
#define B_PORT_DIAG_IN_OV_LS_VDS_MEM_VDS2       ((1U << 2) << 10)               /*!< R: Cleared if dis_ov_ls_vds2 */
#define B_PORT_DIAG_IN_OV_LS_VDS_MEM_VDS3       ((1U << 3) << 10)               /*!< R: Cleared if dis_ov_ls_vds3 */
#define M_PORT_DIAG_IN_OV_HS_VDS_MEM            (15U << 6)                      /*!< R: Cleared if dis_ov_hs_vds */
#define B_PORT_DIAG_IN_OV_HS_VDS_MEM_VDS0       ((1U << 0) << 6)                /*!< R: Cleared if dis_ov_hs_vds0 */
#define B_PORT_DIAG_IN_OV_HS_VDS_MEM_VDS1       ((1U << 1) << 6)                /*!< R: Cleared if dis_ov_hs_vds1 */
#define B_PORT_DIAG_IN_OV_HS_VDS_MEM_VDS2       ((1U << 2) << 6)                /*!< R: Cleared if dis_ov_hs_vds2 */
#define B_PORT_DIAG_IN_OV_HS_VDS_MEM_VDS3       ((1U << 3) << 6)                /*!< R: Cleared if dis_ov_hs_vds3 */
#define B_PORT_DIAG_IN_OVC_MEM                  (1U << 5)                       /*!< R: Cleared if dis_oc */
#define B_PORT_DIAG_IN_UV_VDDAF_MEM             (1U << 4)                       /*!< R: Cleared if dis_uv_vddaf */
#define B_PORT_DIAG_IN_UV_VDDA_MEM              (1U << 3)                       /*!< R: Cleared if dis_uv_vdda */
#define B_PORT_DIAG_IN_UV_VS_MEM                (1U << 2)                       /*!< R: Cleared if dis_uv_vs */
#define B_PORT_DIAG_IN_OV_VS_MEM                (1U << 1)                       /*!< R: Cleared if dis_ov_vs */
#define B_PORT_DIAG_IN_OVT_MEM                  (1U << 0)                       /*!< R: Cleared if dis_ovt */

/* ******************** */
/* Block: PORT_I2C_CONF */
/* ******************** */
extern volatile uint16_t IO_PORT_I2C_CONF __attribute__((nodp, addr(0x00220)));  /*!< IO_PORT_I2C_CONF */
#define M_PORT_I2C_CONF_I2C_SCL_CLK_SEL         (3U << 9)                       /*!< RW: Value for analogue */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_SCL     (0U << 9)                       /*!< RW: I2C Clock Selection SCL */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_IO2     (1U << 9)                       /*!< RW: I2C Clock Selection IO2 */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_IO3     (2U << 9)                       /*!< RW: I2C Clock Selection IO3 */
#define C_PORT_I2C_CONF_I2C_SCL_CLK_SEL_LIN_RXD (3U << 9)                       /*!< RW: I2C Clock Selection LIN_RXD */
#define B_PORT_I2C_CONF_SDA_FILT_ENABLE         (1U << 8)                       /*!< RW: Value for analogue */
#define B_PORT_I2C_CONF_I2C_ADDR_VALID          (1U << 7)                       /*!< RW: enable I2C */
#define M_PORT_I2C_CONF_I2C_ADDR                (127U << 0)                     /*!< RW: 7-bit address for I2C */

/* ************************** */
/* Block: PORT_I2C_DMA_OFFSET */
/* ************************** */
extern volatile uint16_t IO_PORT_I2C_DMA_OFFSET __attribute__((nodp, addr(0x00222)));  /*!< IO_PORT_I2C_DMA_OFFSET */
#define M_PORT_I2C_DMA_OFFSET_I2C_DMA_OFFSET    (65535U << 0)                   /*!< RW: BYTE address defines the start of DMA accesses (so for beginning of RAM it is 0x1000). LSB is ignored to align on word for DMA */

/* *************************** */
/* Block: PORT_I2C_READ_OFFSET */
/* *************************** */
extern volatile uint16_t IO_PORT_I2C_READ_OFFSET __attribute__((nodp, addr(0x00224)));  /*!< IO_PORT_I2C_READ_OFFSET */
#define M_PORT_I2C_READ_OFFSET_I2C_READ_OFFSET  (255U << 0)                     /*!< RW: WORD address for "direct read" startup offset. Up to 255 */

/* ********************* */
/* Block: PORT_MISC_TEST */
/* ********************* */
extern volatile uint16_t IO_PORT_MISC_TEST __attribute__((nodp, addr(0x00226)));  /*!< IO_PORT_MISC_TEST */
#define B_PORT_MISC_TEST_DIS_TX_TIMEOUT         (1U << 8)                       /*!< RW: disable timeout protection (10-20ms) on TX output */
#define M_PORT_MISC_TEST_LINAA_TST_VREF         (3U << 6)                       /*!< RW: Value for analogue */
#define B_PORT_MISC_TEST_TST_LV1_IO0            (1U << 5)                       /*!< RW: Value for analogue */
#define B_PORT_MISC_TEST_TST_LV0_IO0            (1U << 4)                       /*!< RW: Value for analogue */
#define B_PORT_MISC_TEST_TST_LV0_BUF_ENABLE     (1U << 3)                       /*!< RW: Value for analogue */
#define B_PORT_MISC_TEST_TST_RESREF_LV0_OTD     (1U << 2)                       /*!< RW: Value for analogue */
#define B_PORT_MISC_TEST_TST_DISC_DIO_OTD       (1U << 1)                       /*!< RW: Value for analogue */
#define B_PORT_MISC_TEST_TST_CMP_LV0_OTD        (1U << 0)                       /*!< RW: Value for analogue */

/* ************************* */
/* Block: PORT_PPM_RBASE_ADD */
/* ************************* */
extern volatile uint16_t IO_PORT_PPM_RBASE_ADD __attribute__((nodp, addr(0x00228)));  /*!< IO_PORT_PPM_RBASE_ADD */
#define M_PORT_PPM_RBASE_ADD_PPM_RBASE_ADD      (65535U << 0)                   /*!< RW: Base address of the PPM DMA message in RX mode */

/* ************************* */
/* Block: PORT_PPM_TBASE_ADD */
/* ************************* */
extern volatile uint16_t IO_PORT_PPM_TBASE_ADD __attribute__((nodp, addr(0x0022A)));  /*!< IO_PORT_PPM_TBASE_ADD */
#define M_PORT_PPM_TBASE_ADD_PPM_TBASE_ADD      (65535U << 0)                   /*!< RW: Base address of the PPM DMA message in TX mode */

/* *********************** */
/* Block: PORT_PPM_TIMEOUT */
/* *********************** */
extern volatile uint16_t IO_PORT_PPM_TIMEOUT __attribute__((nodp, addr(0x0022C)));  /*!< IO_PORT_PPM_TIMEOUT */
#define M_PORT_PPM_TIMEOUT_PPM_TIMEOUT          (65535U << 0)                   /*!< RW: Timeout value */

/* ******************** */
/* Block: PORT_PPM_CTRL */
/* ******************** */
extern volatile uint16_t IO_PORT_PPM_CTRL __attribute__((nodp, addr(0x0022E)));  /*!< IO_PORT_PPM_CTRL */
#define M_PORT_PPM_CTRL_PPM_IN_SEL              (7U << 11)                      /*!< RW: IO or LIN_XRX for PPM input */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_LIN_XRX      (0U << 11)                      /*!< RW: LIN_XRX for PPM input */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO0          (1U << 11)                      /*!< RW: IO0 for PPM input */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO1          (2U << 11)                      /*!< RW: IO1 for PPM input */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO2          (3U << 11)                      /*!< RW: IO2 for PPM input */
#define C_PORT_PPM_CTRL_PPM_IN_SEL_IO3          (4U << 11)                      /*!< RW: IO3 for PPM input */
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
extern volatile uint16_t IO_PORT_PPM_BUF_DATA __attribute__((nodp, addr(0x00230)));  /*!< IO_PORT_PPM_BUF_DATA */
#define M_PORT_PPM_BUF_DATA_PPM_BUF_DATA        (65535U << 0)                   /*!< R: Value for digital read */

/* **************** */
/* Block: PPM_TIMER */
/* **************** */
extern volatile uint16_t IO_PPM_TIMER __attribute__((nodp, addr(0x00232)));     /*!< IO_PPM_TIMER */
#define M_PPM_TIMER_PPM_ON_TIME                 (65535U << 0)                   /*!< RW: PPM On Time for TX mode */

/* ********************* */
/* Block: PORT_PPM_FLAGS */
/* ********************* */
extern volatile uint16_t IO_PORT_PPM_FLAGS __attribute__((nodp, addr(0x00234)));  /*!< IO_PORT_PPM_FLAGS */
#define B_PORT_PPM_FLAGS_PPM_DMA_ERR            (1U << 3)                       /*!< R: Try to access non-existing area or non-writable area */
#define B_PORT_PPM_FLAGS_PPM_TOUT               (1U << 2)                       /*!< R: Time-out is reached before next edge */
#define B_PORT_PPM_FLAGS_PPM_RCVF               (1U << 1)                       /*!< R: RX: receive buffer overflow/TX: end of transmission */
#define B_PORT_PPM_FLAGS_PPM_DMA_OP             (1U << 0)                       /*!< R: DMA access (R/W) on going */

/* ********************** */
/* Block: PORT_SSCM2_CONF */
/* ********************** */
extern volatile uint16_t IO_PORT_SSCM2_CONF __attribute__((nodp, addr(0x00236)));  /*!< IO_PORT_SSCM2_CONF */
#define B_PORT_SSCM2_CONF_SSCM2_CENTERED        (1U << 2)                       /*!< RW: Value for analogue */
#define B_PORT_SSCM2_CONF_SSCM2_SINGLEBIT       (1U << 1)                       /*!< RW: output shall provide a code with Hamming distance of 1 instead of triangle */
#define B_PORT_SSCM2_CONF_SSCM2_EN              (1U << 0)                       /*!< RW: enable the spread spectrum modulation */

/* ********************** */
/* Block: PORT_STEP2_CONF */
/* ********************** */
extern volatile uint16_t IO_PORT_STEP2_CONF __attribute__((nodp, addr(0x00238)));  /*!< IO_PORT_STEP2_CONF */
#define M_PORT_STEP2_CONF_STEP2_CNT             (255U << 8)                     /*!< RW: step count per period of triangular modulation for Spread Spectrum */
#define M_PORT_STEP2_CONF_STEP2_DUR             (15U << 4)                      /*!< RW: step duration in main clock pulses for Spread Spectrum */
#define M_PORT_STEP2_CONF_STEP2_INC             (15U << 0)                      /*!< RW: step increment for Spread Spectrum */

/* ********************* */
/* Block: PORT_IO_ENABLE */
/* ********************* */
extern volatile uint16_t IO_PORT_IO_ENABLE __attribute__((nodp, addr(0x0023A)));  /*!< IO_PORT_IO_ENABLE */
#define M_PORT_IO_ENABLE_IO_DISREC              (15U << 8)                      /*!< RW: Disable Receiver */
#define B_PORT_IO_ENABLE_IO_DISREC_3            ((1U << 3) << 8)                /*!< Disable IO[3] from internal PU for digital in/out */
#define B_PORT_IO_ENABLE_IO_DISREC_2            ((1U << 2) << 8)                /*!< Disable IO[2] from internal PU for digital in/out */
#define B_PORT_IO_ENABLE_IO_DISREC_1            ((1U << 1) << 8)                /*!< Disable IO[1] from internal PU for digital in/out */
#define B_PORT_IO_ENABLE_IO_DISREC_0            ((1U << 0) << 8)                /*!< Disable IO[0] from internal PU for digital in/out */
#define M_PORT_IO_ENABLE_UNUSED                 (15U << 4)                      /*!< RW: Value for analogue */
#define M_PORT_IO_ENABLE_IO_ENABLE              (15U << 0)                      /*!< RW: Set the state of the IO interface */
#define B_PORT_IO_ENABLE_IO_ENABLE_3            (1U << 3)                       /*!< Enable IO[3] */
#define B_PORT_IO_ENABLE_IO_ENABLE_2            (1U << 2)                       /*!< Enable IO[2] */
#define B_PORT_IO_ENABLE_IO_ENABLE_1            (1U << 1)                       /*!< Enable IO[1] */
#define B_PORT_IO_ENABLE_IO_ENABLE_0            (1U << 0)                       /*!< Enable IO[0] */

/* ********************** */
/* Block: PORT_TIMER_CFG1 */
/* ********************** */
extern volatile uint16_t IO_PORT_TIMER_CFG1 __attribute__((nodp, addr(0x0023C)));  /*!< IO_PORT_TIMER_CFG1 */
#define M_PORT_TIMER_CFG1_TIMER1_CHB_SEL        (31U << 8)                      /*!< RW: Input selection for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO0    (0U << 8)                       /*!< RW: Input selection IO0 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO1    (1U << 8)                       /*!< RW: Input selection IO1 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO2    (2U << 8)                       /*!< RW: Input selection IO2 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_IO3    (3U << 8)                       /*!< RW: Input selection IO3 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_LIN_XRX (4U << 8)                      /*!< RW: Input selection LIN for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PWM_MASTER1 (5U << 8)                  /*!< RW: Input selection PWM_MASTER1 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PWM_SLAVE1  (6U << 8)                  /*!< RW: Input selection PWM_SLAVE1 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PWM_SLAVE2  (7U << 8)                  /*!< RW: Input selection PWM_SLAVE2 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PWM_SLAVE3  (8U << 8)                  /*!< RW: Input selection PWM_SLAVE3 for timer[1] channel B */
#define C_PORT_TIMER_CFG1_TIMER1_CHB_SEL_PWM_MASTER2 (9U << 8)                  /*!< RW: Input selection PWM_MASTER2 for timer[1] channel B */
#define M_PORT_TIMER_CFG1_TIMER1_CHA_SEL        (31U << 0)                      /*!< RW: Input selection for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO0    (0U << 0)                       /*!< RW: Input selection IO0 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO1    (1U << 0)                       /*!< RW: Input selection IO1 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO2    (2U << 0)                       /*!< RW: Input selection IO2 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_IO3    (3U << 0)                       /*!< RW: Input selection IO3 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_LIN_XRX (4U << 0)                      /*!< RW: Input selection LIN for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PWM_MASTER1 (5U << 0)                  /*!< RW: Input selection PWM_MASTER1 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PWM_SLAVE1  (6U << 0)                  /*!< RW: Input selection PWM_SLAVE1 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PWM_SLAVE2  (7U << 0)                  /*!< RW: Input selection PWM_SLAVE2 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PWM_SLAVE3  (8U << 0)                  /*!< RW: Input selection PWM_SLAVE3 for timer[1] channel A */
#define C_PORT_TIMER_CFG1_TIMER1_CHA_SEL_PWM_MASTER2 (9U << 0)                  /*!< RW: Input selection PWM_MASTER2 for timer[1] channel A */

/* **************************** */
/* Block: PORT_IO_TRIG_EDGE_CFG */
/* **************************** */
extern volatile uint16_t IO_PORT_IO_TRIG_EDGE_CFG __attribute__((nodp, addr(0x0023E)));  /*!< IO_PORT_IO_TRIG_EDGE_CFG */
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

/* ********************** */
/* Block: PORT_TX_TIMEOUT */
/* ********************** */
extern volatile uint16_t IO_PORT_TX_TIMEOUT __attribute__((nodp, addr(0x00240)));  /*!< IO_PORT_TX_TIMEOUT */
#define B_PORT_TX_TIMEOUT_TX_TIMEOUT            (1U << 0)                       /*!< R: Value for digital read */
#endif

#endif /* defined (__MLX81330__) */

/* EOF */
