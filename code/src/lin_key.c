#include <syslib.h>
#if (defined (LDR_HAS_PROTECTION_KEY) || defined (LDR_HAS_FLASH_READ_PROTECTION_KEY) )
#if LDR_PROTECTION_KEY_LENGTH == 32
__attribute__((section(".LIN_key"))) const uint32_t LIN_key[1] = {0x15151516ul};
#elif LDR_PROTECTION_KEY_LENGTH == 64
__attribute__((section(".LIN_key"))) const uint32_t LIN_key[2] = {0x15152020ul, 0x15152020ul};
#else
#error "Incorrect LDR_PROTECTION_KEY_LENGTH value"
#endif
#endif