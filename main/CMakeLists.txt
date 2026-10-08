# Only the link header is taken from HOJA-LIB-RP2040: its include tree has names (hal/...) that
# would shadow ESP-IDF's, so the header is copied into a private include directory.
# (The project-level variable is not visible during ESP-IDF's early requirements scan.)
set(HLINK_INC "${CMAKE_BINARY_DIR}/hlink_include")
if(HOJA_LIB_RP2040_PATH)
    file(COPY "${HOJA_LIB_RP2040_PATH}/include/drivers/bluetooth/hlink.h"
         DESTINATION "${HLINK_INC}/drivers/bluetooth")
endif()

idf_component_register(
    SRCS
        "main.c"
        "link_i2c.c"
        "${HOJA_LIB_RP2040_PATH}/src/drivers/bluetooth/hlink.c"
    INCLUDE_DIRS
        "."
        "${HLINK_INC}"
    LDFRAGMENTS "linker.lf"
    REQUIRES bt nvs_flash driver esp_driver_uart esp_driver_gpio esp_adc esp_hw_support esp_ringbuf hal soc
)
