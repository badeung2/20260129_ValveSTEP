# @file
# @brief Application sources list
# @internal
#
# @copyright (C) 2018-2024 Melexis N.V.
#
# Melexis N.V. is supplying this code for use with Melexis N.V. processor based microcontrollers only.
#
# THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
# INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.  MELEXIS N.V. SHALL NOT IN ANY CIRCUMSTANCES,
# BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
#
# @endinternal
#
# @ingroup application
#
# @details List of application sources to be build into the firmware application.
#


#
# SOURCE FILES LIST
# note: files can be located in subfolders of src/, the SRCS_APP entry needs to be
#       relative to src/ folder.
#
SRCS_APP += main.c AppVersion.c
SRCS_APP += camculib/Atan.S
SRCS_APP += camculib/fw_vectors_MLX81332.S
SRCS_APP += camculib/fw_fatal.c
SRCS_APP += camculib/Pack_Unpack.S
SRCS_APP += camculib/private_mathlib.c

SRCS_APP += hal_lib/hal_ADC.c
SRCS_APP += hal_lib/hal_ADC_convert.c
SRCS_APP += hal_lib/hal_STimer.c

SRCS_APP += drivelib/ADC.c
SRCS_APP += drivelib/AppFunctions.c
SRCS_APP += drivelib/Diagnostic.c
SRCS_APP += drivelib/ErrorCodes.c
SRCS_APP += drivelib/GlobalVars.c
SRCS_APP += drivelib/MotorDriver.c
SRCS_APP += drivelib/MotorDriverFOC.c
SRCS_APP += drivelib/MotorDriverSelfTest.c
SRCS_APP += drivelib/MotorDriverTables.c
SRCS_APP += drivelib/MotorStall.c
SRCS_APP += drivelib/NV_Functions.c
SRCS_APP += drivelib/PID_Control.c
SRCS_APP += drivelib/Timer.c

SRCS_APP += senselib/Triaxis_MLX90422_426.c
#SRCS_APP += senselib/Triaxis_MLX90427.c

SRCS_APP += commlib/lin2b_romtbl.S
SRCS_APP += commlib/LIN_Communication.c
SRCS_APP += commlib/LIN_AutoAddressing.c
SRCS_APP += commlib/LIN_2x_HVAC.c
SRCS_APP += commlib/LIN_Diagnostics.c
SRCS_APP += commlib/LIN_MlxDiagnostics.c
#SRCS_APP += commlib/LIN_UdsDiagnostics.c

SRCS_APP += ActADC.c


#
# EXTRA PLATFORM MODULES TO COMPILE IN
#
APP_MODULES += mls_api
APP_MODULES += mls_device_id
#APP_MODULES += std_lin_api
APP_MODULES += user_startup
# SAE J2602 Sync-error fix (Only platform V1.1.0.x and newer)
#APP_MODULES += patch_colin

APP_PRODUCT_ID = "MLX_STEP"
APP_VERSION_MAJOR = 0x01
APP_VERSION_MINOR = 0x01
APP_VERSION_PATCH = 0x00
APP_VERSION_BUILD = 0x01
PROTECTION_KEY = 0x0000000000000000
APP_MODULES += app_descriptor


#
# PLATFORM LIBRARIES USED BY THIS APPLICATION
#
PLTF_LIBS += $(PRODUCT)
PLTF_LIBS += math
PLTF_LIBS += mlx_lin_api
#PLTF_LIBS += fast_math


#
# EXTRA BU-LIBRARIES USED BY THIS APPLICATION
#
BU_LIBS +=

