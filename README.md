# HOJA-ESP32-HCI-Bridge

ESP32 firmware for HOJA boards with an RP2040 and an ESP32 (the GCU 1 so far).

The ESP32 only runs its Bluetooth controller. The Bluetooth host runs on the RP2040 (`BT_DRIVER_ESP32HCI` in HOJA-LIB-RP2040), and HCI packets travel over the board's I2C wires with the HOJA link (`hlink.c`, shared with HOJA-LIB-RP2040).

Needs RP2040 firmware with `BT_DRIVER_ESP32HCI`. Older RP2040 firmware expects HOJA-ESP32-Baseband.

## Building

ESP-IDF 5.5. HOJA-LIB-RP2040 is expected at `../hoja-device-fw/library/HOJA-LIB-RP2040`, or set `HOJA_LIB_RP2040_PATH`.

```
idf.py build
idf.py -p COMx flash
```
