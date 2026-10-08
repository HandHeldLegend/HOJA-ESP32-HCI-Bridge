cmake_minimum_required(VERSION 3.16)

# The link protocol (hlink.c/.h) lives in HOJA-LIB-RP2040 so both firmwares build the same file.
if(NOT DEFINED HOJA_LIB_RP2040_PATH)
    set(HOJA_LIB_RP2040_PATH "${CMAKE_CURRENT_LIST_DIR}/../hoja-device-fw/library/HOJA-LIB-RP2040")
endif()
get_filename_component(HOJA_LIB_RP2040_PATH "${HOJA_LIB_RP2040_PATH}" ABSOLUTE)
if(NOT EXISTS "${HOJA_LIB_RP2040_PATH}/src/drivers/bluetooth/hlink.c")
    message(FATAL_ERROR "hlink.c not found; set HOJA_LIB_RP2040_PATH to your HOJA-LIB-RP2040 checkout")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hoja_hci_bridge)
