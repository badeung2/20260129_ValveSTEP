/*
 * Copyright (C) 2017-2022 Melexis N.V.
 *
 * Software Platform
 *
 */


/*
 * LIN KEY is 32 or 64 KEY in specified FLASH area to lock reading from the FLASH via LIN. This is 2 16bit or 32bit values,
 * if these values are equal, defense is disabled. And if this KEY is equal to stored data in NVRAM (internal key, 0x1080)
 * reading is also available.
 */
#include <syslib.h>


#if (defined (LDR_HAS_PROTECTION_KEY) || defined (LDR_HAS_FLASH_READ_PROTECTION_KEY) )

#if LDR_PROTECTION_KEY_LENGTH == 32
/* Protection key */
__attribute__((section(".LIN_key"))) const uint32_t LIN_key[1] = {0x15151516ul};

#elif LDR_PROTECTION_KEY_LENGTH == 64
/* Protection key */
__attribute__((section(".LIN_key"))) const uint32_t LIN_key[2] = {0x15152020ul, 0x15152020ul};

#else
#error "Incorrect LDR_PROTECTION_KEY_LENGTH value"
#endif /* LDR_PROTECTION_KEY_LENGTH */

#endif /* LDR_HAS_PROTECTION_KEY || LDR_HAS_FLASH_READ_PROTECTION_KEY */


