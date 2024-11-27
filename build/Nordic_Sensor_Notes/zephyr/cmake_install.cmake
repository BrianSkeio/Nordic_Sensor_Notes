# Install script for directory: C:/ncs/v2.8.0/zephyr

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files/Zephyr-Kernel")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "C:/ncs/toolchains/2d382dcd92/opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-objdump.exe")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/zephyr/arch/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/zephyr/lib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/zephyr/soc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/zephyr/boards/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/zephyr/subsys/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/zephyr/drivers/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/nrf/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/mcuboot/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/mbedtls/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/trusted-firmware-m/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/cjson/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/azure-sdk-for-c/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/cirrus-logic/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/openthread/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/suit-processor/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/memfault-firmware-sdk/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/canopennode/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/chre/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/lz4/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/nanopb/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/zscilib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/cmsis/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/cmsis-dsp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/cmsis-nn/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/fatfs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/hal_nordic/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/hal_st/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/hal_wurthelektronik/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/hostap/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/libmetal/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/liblc3/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/littlefs/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/loramac-node/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/lvgl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/mipi-sys-t/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/nrf_hw_models/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/open-amp/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/picolibc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/segger/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/tinycrypt/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/uoscore-uedhoc/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/zcbor/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/nrfxlib/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/modules/connectedhomeip/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/zephyr/kernel/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/zephyr/cmake/flash/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/zephyr/cmake/usage/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("D:/Code/Nordic/CheckinToGitHub/Nordic_Sensor_Notes/build/Nordic_Sensor_Notes/zephyr/cmake/reports/cmake_install.cmake")
endif()

