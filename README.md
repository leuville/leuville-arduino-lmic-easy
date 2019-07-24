# leuville-arduino-lmic-easy
Various programming utilities for Arduino devices designed to send and receive LoRaWAN messages with LMIC library.
LMICWrapper is a base class for LoRaWAN endnodes built on top of LMIC library.
ProtobufEndnode is a subclass of LMICWrapper which uses ProtocolBuffer to serialize/deserialize LoRaWAN messages.

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
This example builds a LoRaWAN device with:

 1. a callback set on button connected to A0 pin. This callback sends a "Button" message each time the button is pressed.
 2. a timer to send a "Ping" message each 5 mn
 3. a stanby mode feature

 ## Example 2: TestProtobufEndnode.ino
This example shows how to serialize/deserialize LoRaWAN messages with ProtocolBuffer.
The endnode device is the same as the one built in TestLMICWrapper.ino sample.

 
<!--stackedit_data:
eyJoaXN0b3J5IjpbMTQwMTI2NDk4Myw2NjYyNDA5ODMsMTc2Mj
AxNzcyMSwtMTczODc0NzM5NiwtNTM1MzYxOTA0XX0=
-->