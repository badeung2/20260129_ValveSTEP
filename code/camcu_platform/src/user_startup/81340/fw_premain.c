/**
 * @file
 * @brief The flash firmware premain routines.
 * @internal
 *
 * @copyright (C) 2018-2020 Melexis N.V.
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
 * @ingroup fw_user_startup
 *
 * @details This file contains the implementations of the flash firmware premain routine.
 */

#include <stdint.h>
#include "io.h"
#include "lib_clock.h"
#include "mem_checks.h"
#include "eeprom_map.h"
#include "fw_startup.h"

#if defined (HAS_PATCH_COLIN)
#include "patch_colin.h"
#endif

#if defined (HAS_MLS_API)
#include "fw_mls_api.h"
#endif

#if defined (HAS_MLS_DEVICE_ID)
#include "fw_mls_device_id.h"
#endif

#if defined (HAS_LIN_AUTO_ADDRESSING)
#include "fw_lin_auto_addressing.h"
#endif

#if defined (HAS_MLS_LOADER)
#include "mls_loader.h"
#endif


/* ---------------------------------------------
 * Local Function Declarations
 * --------------------------------------------- */

__attribute__((weak)) void fw_low_level_init(void);
STATIC INLINE void set_Clock_Speed(void);


/* ---------------------------------------------
 * Function Definitions
 * --------------------------------------------- */

void fw_premain(void)
{
    fw_low_level_init();            /* optionally call more low level initialization */

    fw_ram_section_init();          /* RAM initialization */

#if defined (HAS_PATCH_COLIN)
    patch_colin_Init();
#endif /* HAS_PATCH_COLIN */

#if defined (HAS_MLS_API)
    fw_mls_Init();                  /* MLX LIN Slave initialization */
#endif /* HAS_MLS_API */

#if defined (HAS_STD_LIN_API) || defined (HAS_MLS_DEVICE_ID) || defined (HAS_MLS_LOADER) || defined (HAS_APP_UDS)
    fw_mls_TransportLayerInit();
#endif /* HAS_STD_LIN_API || HAS_MLS_DEVICE_ID || defined (HAS_MLS_LOADER) || HAS_APP_UDS */

#if defined (HAS_MLS_DEVICE_ID)
    fw_lepm_Init();                 /* MLX LIN device ID initialization */
#endif /* HAS_MLS_DEVICE_ID */

#if defined (HAS_LIN_AUTO_ADDRESSING)
    fw_linaa_Init();                /* MLX LIN auto addressing initialization */
#endif /* HAS_LIN_AUTO_ADDRESSING */

#if defined (HAS_MLS_LOADER)
    mls_loader_Init();
#endif /* HAS_MLS_LOADER */

    set_Clock_Speed();              /* set clock speed */
    builtin_mlx16_set_priority(7);  /* system mode, the lowest priority: 7 */
}

void fw_low_level_init(void)
{
    if (nvram_CalcCRC((uint16_t*)EE_APP_TRIM_AREA_START, EE_APP_TRIM_AREA_SIZE >> 1) == NVRAM_CORRECT_CRC) {
        /* load trimming data to io registers */
        IO_SET(TRIM_IILD,
               CNT_IILD, EE_GET(CNT_IILD));                 /* trim internal dead time */
        IO_SET(TRIM_CP,
               TR_CP, EE_GET(TR_CP),                        /* trim of the charge pump voltage */
               TR_RCO50K, EE_GET(TR_RCO50K));               /* trim of the 60k charge pump clock */
        IO_SET(TRIM_MISC,
               TRIM_OTD, EE_GET(TRIM_OTD),                  /* trim over temperature detection */
               TRIM_SDAFILT_IO, EE_GET(TRIM_SDAFILT_IO),    /* trim i2c sda filter/delay time */
               TRIM_LCD_LINAA, EE_GET(LINAA_TRIM_LCD_205)); /* trim linaa current */
        IO_SET(TRIM_ADC, ADC_INTREF_TRM, 3u);               /* select adc reference */
        IO_SET(PORT_LINAA2,
               LCD_SEL_LINAA, EE_GET(LINAA_LCD_SEL_205));   /* select adjustment of linaa current */
        IO_SET(PORT_LINAA1,
               LINAA_GAIN, EE_GET(LINAA_GAIN),              /* linaa amplifier gain adjustment */
               LINAA_DIV, EE_GET(LINAA_DIV));               /* linaa amplifier common-mode rejection adjustment */
    } else {
        /* load default values */
    }

    IO_SET(PORT_SUPP_CFG,
           UV_VDDA_FILT_SEL, 1u,                    /* 100-110us filtering for VDDA under voltage */
           OV_VDDA_FILT_SEL, 1u,                    /* 100-110us filtering for VDDA over voltage */
           OC_VDDA_FILT_SEL, 1u,                    /* 100-110us filtering for VDDA over current */
           UV_BOOST_FILT_SEL, 1u,                   /* 100-110us filtering for VBOOST under voltage */
           OV_BOOST_FILT_SEL, 1u,                   /* 100-110us filtering for VBOOST over voltage */
           UV_VS_FILT_SEL, 1u,                      /* 100-110us filtering for VS under voltage */
           OV_VS_FILT_SEL, 1u,                      /* 100-110us filtering for VS over voltage */
           OVT_FILT_SEL, 1u);                       /* 100-110us filtering for over temperature */

    IO_SET(PORT_MISC_OUT, SEL_TEMP, 11u);           /* temperature channel selection */

    IO_SET(PORT_MISC2_OUT, ENABLE_OTD, 1u);         /* enable over-temperature detector */

    IO_SET(PORT_DRV1_PROT, OC_PM, 1u);              /* switch drivers to tri-state in case of over-current */
}

/** Set the PLL clock speed according the user configuration.
 * This function will configure the PLL clock speed according to the user
 * selected configuration as in Makefile.configure.mk. Six different values
 * are supported as these are the values for which we have trimming data in
 * the eeprom calibration section.
 */
STATIC INLINE void set_Clock_Speed(void)
{
    RC_Settings_t tmp;
    uint8_t ac_sel = 0u;

#if FPLL == 12000
    tmp.u = EE_MS_TRIM6_VALUE;
    ac_sel = 1u;
#elif FPLL == 14000
    tmp.u = EE_MS_TRIM7_VALUE;
    ac_sel = 1u;
#elif FPLL == 16000
    tmp.u = EE_MS_TRIM8_VALUE;
    ac_sel = 1u;
#elif FPLL == 24000
    tmp.u = EE_MS_TRIM6_VALUE;
    ac_sel = 0u;
#elif FPLL == 28000
    tmp.u = EE_MS_TRIM7_VALUE;
    ac_sel = 0u;
#elif FPLL == 32000
    tmp.u = EE_MS_TRIM8_VALUE;
    ac_sel = 0u;
#else
    #warning "Clock Speed not valid, use 32MHz"
#endif

    SetSystemSpeed(tmp, ac_sel);
}

