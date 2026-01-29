#include "AppBuild.h"
#include "AppVersion.h"
#if !defined (PROTECTION_KEY)
static const uint32_t protection_key[] __attribute__((used, section(".fw_prot_key"))) = {
0x01230123u,
0x01230123u
};
#endif
#if !defined (APP_VERSION_MAJOR) && !defined (APP_VERSION_MINOR)
const uint32_t application_version __attribute__((section(".fw_app_version"))) =
(__APP_VERSION_MAJOR__) |
(__APP_VERSION_MINOR__ << 8) |
(__APP_VERSION_REVISION__ << 16);
#endif
#if !defined (APP_PRODUCT_ID)
const uint8_t product_id[8] __attribute__((section(".fw_product_id"))) =
"MLX_STEP";
#endif