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
LMICWrapper is a base class for LoRaWAN endnodes built on top of LMIC library. It has been tested with two LMIC implementations (see Required libraries section).

LMICWrapper stores LoRaWAN messages into a deque object (STL) which acts as a FIFO list of messages waiting to be sent. Each time runloopOnce() is called (tipically from main loop), a LMIC callback job is registered to send first back object of this deque. When the callback job is performed, the first-out message is sent and removed from the deque. By this way, LoRaWAN messages are not lost if the radio or the network are not available.

### ProtobufEndnode<>
ProtobufEndnode is a subclass of LMICWrapper which uses ProtocolBuffer to serialize/deserialize LoRaWAN messages.
 
## Example 1: TestLMICWrapper.ino
This example builds a LoRaWAN device as a subclass of LMICWrapper, with:

 1. a callback set on button connected to A0 pin. This callback sends a "CLICK" message each time the button is pressed.
 2. a timer to send a "TIMEOUT" message each 5 mn
 3. a stanby mode feature

 ## Example 2: TestProtobufEndnode.ino
This example shows how to serialize/deserialize LoRaWAN messages with ProtocolBuffer.
The endnode device is the same as the one built in TestLMICWrapper.ino sample.

 
<!--stackedit_data:
eyJoaXN0b3J5IjpbLTIwNjMxMDk2NjcsLTE3MTA3MzcwNjIsNj
Y2MjQwOTgzLDE3NjIwMTc3MjEsLTE3Mzg3NDczOTYsLTUzNTM2
MTkwNF19
-->