# leuville-arduino-lmic-easy
Various programming utilities for Arduino devices designed to send and receive LoRaWAN messages with LMIC library.

This library is made on top of:

 - LMIC
	 - arduino port: [https://github.com/matthijskooijman/arduino-lmic](https://github.com/matthijskooijman/arduino-lmic)
	 - MCCI release: [https://github.com/mcci-catena/arduino-lmic](https://github.com/mcci-catena/arduino-lmic)
 - ArduinoSTL: [https://github.com/mike-matera/ArduinoSTL](https://github.com/mike-matera/ArduinoSTL)
 - Nanopb: C implementation of Google's [Protocol Buffers](http://code.google.com/apis/protocolbuffers/) data format targeted for 32 bit microcontrollers [https://github.com/nanopb/nanopb](https://github.com/nanopb/nanopb)

## Contents

 - LMICWrapper.h:
	 - LMICWrapper: object-oriented ISR wrapper
	 - ProtobufEndnode<>
 
## Example 1: TestLeuvilleLMIC.ino
This example builds 
 
<!--stackedit_data:
eyJoaXN0b3J5IjpbMTM3NTA3NDg5OCwtNTM1MzYxOTA0XX0=
-->