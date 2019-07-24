# leuville-arduino-lmic-easy
Various programming utilities for Arduino devices designed to send and receive LoRaWAN messages with LMIC library.

The classes in this library are normally intended to be used inside a class that overrides one or more of the virtual methods.

## Required libraries


 - LMIC
	 - arduino port: [https://github.com/matthijskooijman/arduino-lmic](https://github.com/matthijskooijman/arduino-lmic)
	 - MCCI release: [https://github.com/mcci-catena/arduino-lmic](https://github.com/mcci-catena/arduino-lmic)
 - ArduinoSTL: [https://github.com/mike-matera/ArduinoSTL](https://github.com/mike-matera/ArduinoSTL)
 - Nanopb: C implementation of Google's [Protocol Buffers](http://code.google.com/apis/protocolbuffers/) data format targeted for 32 bit microcontrollers [https://github.com/nanopb/nanopb](https://github.com/nanopb/nanopb)

## Contents

### LMICWrapper


### ProtobufEndnode<>

 
## Example 1: TestLMICWrapper.ino
This example builds a LoRaWAN device with

 ## Example 2: TestProtobufEndnode.ino
This example shows how to serialize/deserialize LoRaWAN messages with ProtocolBuffer.
 
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTI3ODI0NDI2OSwtMTczODc0NzM5NiwtNT
M1MzYxOTA0XX0=
-->