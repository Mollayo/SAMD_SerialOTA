# SAMD_SerialOTA
This library allows you to update sketches on the SAMD21 and SAMD51 MCU over serial connection. The SAMD MCU must be connected to an ESP8266 (or any other MCU with WiFi connection) with a serial connection.

Dependencies:
- CRC32 library: https://github.com/bakercp/CRC32
- TRGN library for the SAMD51 MCU: https://github.com/SapientHetero/TRNG-for-ATSAMD51J19A-Adafruit-Metro-M4-
- Serial communication library: https://github.com/Mollayo/ArduinoSerialPackets
