@echo off
cd /D D:\Code\Nordic\CheckinToGitHub\Nordic_Sensor_Notes\build\zephyr\kconfig || (set FAIL_LINE=2& goto :ABORT)
C:\ncs\toolchains\2d382dcd92\opt\bin\cmake.exe -E env ZEPHYR_BASE=C:/ncs/v2.8.0/zephyr PYTHON_EXECUTABLE=C:/ncs/toolchains/2d382dcd92/opt/bin/python.exe srctree=C:/ncs/v2.8.0/zephyr KERNELVERSION= APPVERSION= APP_VERSION_EXTENDED_STRING= APP_VERSION_TWEAK_STRING= CONFIG_=SB_CONFIG_ KCONFIG_CONFIG=D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/zephyr/.config KCONFIG_BOARD_DIR=D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Kconfig/boards BOARD=nrf52840dk BOARD_REVISION= BOARD_QUALIFIERS=/nrf52840 HWM_SCHEME=v2 KCONFIG_BINARY_DIR=D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Kconfig APPLICATION_SOURCE_DIR=C:/ncs/v2.8.0/zephyr/share/sysbuild ZEPHYR_TOOLCHAIN_VARIANT= TOOLCHAIN_KCONFIG_DIR= TOOLCHAIN_HAS_NEWLIB=n TOOLCHAIN_HAS_PICOLIBC=n HIDE_CHILD_PARENT_CONFIG= EDT_PICKLE= NCS_MEMFAULT_FIRMWARE_SDK_KCONFIG=C:/ncs/v2.8.0/nrf/modules/memfault-firmware-sdk/Kconfig BOARD=nrf52840dk ZEPHYR_NRF_MODULE_DIR=C:/ncs/v2.8.0/nrf ZEPHYR_MCUBOOT_MODULE_DIR=C:/ncs/v2.8.0/bootloader/mcuboot ZEPHYR_MCUBOOT_KCONFIG=C:/ncs/v2.8.0/nrf/modules/mcuboot/Kconfig ZEPHYR_MBEDTLS_MODULE_DIR=C:/ncs/v2.8.0/modules/crypto/mbedtls ZEPHYR_MBEDTLS_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/mbedtls/Kconfig ZEPHYR_OBERON_PSA_CRYPTO_MODULE_DIR=C:/ncs/v2.8.0/modules/crypto/oberon-psa-crypto ZEPHYR_TRUSTED_FIRMWARE_M_MODULE_DIR=C:/ncs/v2.8.0/modules/tee/tf-m/trusted-firmware-m ZEPHYR_TRUSTED_FIRMWARE_M_KCONFIG=C:/ncs/v2.8.0/nrf/modules/trusted-firmware-m/Kconfig ZEPHYR_PSA_ARCH_TESTS_MODULE_DIR=C:/ncs/v2.8.0/modules/tee/tf-m/psa-arch-tests ZEPHYR_SOC_HWMV1_MODULE_DIR=C:/ncs/v2.8.0/modules/soc-hwmv1 ZEPHYR_CJSON_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/cjson ZEPHYR_CJSON_KCONFIG=C:/ncs/v2.8.0/nrf/modules/cjson/Kconfig ZEPHYR_AZURE_SDK_FOR_C_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/azure-sdk-for-c ZEPHYR_AZURE_SDK_FOR_C_KCONFIG=C:/ncs/v2.8.0/nrf/modules/azure-sdk-for-c/Kconfig ZEPHYR_CIRRUS_LOGIC_MODULE_DIR=C:/ncs/v2.8.0/modules/hal/cirrus-logic ZEPHYR_OPENTHREAD_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/openthread ZEPHYR_OPENTHREAD_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/openthread/Kconfig ZEPHYR_SUIT_GENERATOR_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/suit-generator ZEPHYR_SUIT_PROCESSOR_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/suit-processor ZEPHYR_MEMFAULT_FIRMWARE_SDK_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/memfault-firmware-sdk ZEPHYR_COREMARK_MODULE_DIR=C:/ncs/v2.8.0/modules/benchmark/coremark ZEPHYR_COREMARK_KCONFIG=C:/ncs/v2.8.0/nrf/modules/coremark/Kconfig ZEPHYR_CANOPENNODE_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/canopennode ZEPHYR_CANOPENNODE_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/canopennode/Kconfig ZEPHYR_CHRE_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/chre ZEPHYR_LZ4_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/lz4 ZEPHYR_LZ4_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/lz4/Kconfig ZEPHYR_NANOPB_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/nanopb ZEPHYR_NANOPB_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/nanopb/Kconfig ZEPHYR_TF_M_TESTS_MODULE_DIR=C:/ncs/v2.8.0/modules/tee/tf-m/tf-m-tests ZEPHYR_ZSCILIB_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/zscilib ZEPHYR_CMSIS_MODULE_DIR=C:/ncs/v2.8.0/modules/hal/cmsis ZEPHYR_CMSIS_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/cmsis/Kconfig ZEPHYR_CMSIS_DSP_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/cmsis-dsp ZEPHYR_CMSIS_DSP_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/cmsis-dsp/Kconfig ZEPHYR_CMSIS_NN_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/cmsis-nn ZEPHYR_CMSIS_NN_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/cmsis-nn/Kconfig ZEPHYR_FATFS_MODULE_DIR=C:/ncs/v2.8.0/modules/fs/fatfs ZEPHYR_FATFS_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/fatfs/Kconfig ZEPHYR_HAL_NORDIC_MODULE_DIR=C:/ncs/v2.8.0/modules/hal/nordic ZEPHYR_HAL_NORDIC_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/hal_nordic/Kconfig ZEPHYR_HAL_ST_MODULE_DIR=C:/ncs/v2.8.0/modules/hal/st ZEPHYR_HAL_ST_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/hal_st/Kconfig ZEPHYR_HAL_WURTHELEKTRONIK_MODULE_DIR=C:/ncs/v2.8.0/modules/hal/wurthelektronik ZEPHYR_HOSTAP_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/hostap ZEPHYR_HOSTAP_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/hostap/Kconfig ZEPHYR_LIBMETAL_MODULE_DIR=C:/ncs/v2.8.0/modules/hal/libmetal ZEPHYR_LIBLC3_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/liblc3 ZEPHYR_LIBLC3_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/liblc3/Kconfig ZEPHYR_LITTLEFS_MODULE_DIR=C:/ncs/v2.8.0/modules/fs/littlefs ZEPHYR_LITTLEFS_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/littlefs/Kconfig ZEPHYR_LORAMAC_NODE_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/loramac-node ZEPHYR_LORAMAC_NODE_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/loramac-node/Kconfig ZEPHYR_LVGL_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/gui/lvgl ZEPHYR_LVGL_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/lvgl/Kconfig ZEPHYR_MIPI_SYS_T_MODULE_DIR=C:/ncs/v2.8.0/modules/debug/mipi-sys-t ZEPHYR_NRF_HW_MODELS_MODULE_DIR=C:/ncs/v2.8.0/modules/bsim_hw_models/nrf_hw_models ZEPHYR_OPEN_AMP_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/open-amp ZEPHYR_PICOLIBC_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/picolibc ZEPHYR_SEGGER_MODULE_DIR=C:/ncs/v2.8.0/modules/debug/segger ZEPHYR_SEGGER_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/segger/Kconfig ZEPHYR_TINYCRYPT_MODULE_DIR=C:/ncs/v2.8.0/modules/crypto/tinycrypt ZEPHYR_UOSCORE_UEDHOC_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/uoscore-uedhoc ZEPHYR_UOSCORE_UEDHOC_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/uoscore-uedhoc/Kconfig ZEPHYR_ZCBOR_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/zcbor ZEPHYR_ZCBOR_KCONFIG=C:/ncs/v2.8.0/zephyr/modules/zcbor/Kconfig ZEPHYR_NRFXLIB_MODULE_DIR=C:/ncs/v2.8.0/nrfxlib ZEPHYR_CONNECTEDHOMEIP_MODULE_DIR=C:/ncs/v2.8.0/modules/lib/matter ARCH=* ARCH_DIR=C:/ncs/v2.8.0/zephyr/arch SHIELD_AS_LIST= DTS_POST_CPP= DTS_ROOT_BINDINGS= C:/ncs/toolchains/2d382dcd92/opt/bin/python.exe C:/ncs/v2.8.0/zephyr/scripts/kconfig/guiconfig.py C:/ncs/v2.8.0/zephyr/share/sysbuild/Kconfig || (set FAIL_LINE=3& goto :ABORT)
goto :EOF

:ABORT
set ERROR_CODE=%ERRORLEVEL%
echo Batch file failed at line %FAIL_LINE% with errorcode %ERRORLEVEL%
exit /b %ERROR_CODE%