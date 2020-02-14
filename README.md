# leuville-arduino-lmic-easy
Various programming utilities for Arduino devices designed to send and receive LoRaWAN messages with LMIC library. It is designed to keep benefits of Object-Oriented programming.
LMICWrapper is a base class for LoRaWAN endnodes built on top of LMIC library.
ProtobufEndnode is a subclass of LMICWrapper which uses ProtocolBuffer to serialize/deserialize LoRaWAN messages.

The classes in this library are normally intended to be used inside a class that overrides one or more of the virtual methods.

## Required libraries


 - LMIC (any of these implementations)
	 - arduino port: [https://github.com/matthijskooijman/arduino-lmic](https://github.com/matthijskooijman/arduino-lmic)
	 - MCCI release: [https://github.com/mcci-catena/arduino-lmic](https://github.com/mcci-catena/arduino-lmic), version 2.X (for unknown reason, version 3.X does not work - investigations on the way)
 - Nanopb: C implementation of Google's [Protocol Buffers](http://code.google.com/apis/protocolbuffers/) data format targeted for 32 bit microcontrollers [https://github.com/nanopb/nanopb](https://github.com/nanopb/nanopb)
 - leuville-arduino_utilities: various Arduino programming utilities [https://github.com/leuville/leuville-arduino-utilities](https://github.com/leuville/leuville-arduino-utilities)

## Contents

### LMICWrapper
LMICWrapper is a base class for LoRaWAN endnodes built on top of LMIC library. It has been tested with two LMIC implementations (see Required libraries section).

LMICWrapper stores LoRaWAN messages into a deque object (STL) which acts as a FIFO list of messages waiting to be sent. Each time runloopOnce() is called (typically from main loop), a LMIC callback job is registered to send first back object of this deque. When the callback job is performed, the first-out message is sent and removed from the deque if isTxCompleted() returns true. By this way, LoRaWAN messages are not lost if the radio or the network are not available.

### ProtobufEndnode<>
ProtobufEndnode is a template subclass of LMICWrapper which uses ProtocolBuffer to serialize/deserialize LoRaWAN messages.

ProtobufEndnode is a template class parametrized by ProtocolBuffer message types generated by Nanopb implementation from .proto description. It behaves the same as LMICWrapper, except it encodes/decodes messages.

## Example 1: TestLMICWrapper.cpp
This example builds a LoRaWAN device as a subclass of LMICWrapper, with:

 1. a callback set on button connected to A0 pin. This callback sends a "CLICK" message each time the button is pressed.
 2. a timer to send a "TIMEOUT" message each 5 mn
 3. a standby mode feature

 ## Example 2: TestProtobufEndnode.cpp
This example shows how to serialize/deserialize LoRaWAN messages with ProtocolBuffer.
The endnode device is the same as the one built in TestLMICWrapper.ino sample.

The message sent by the device is defined with a .proto file like this one:

    syntax = "proto3";
    
    package leuville;
    
    enum Type {
    	PING 	= 0;
    	BUTTON 	= 1;
    }
    
    message Uplink {		
    	Type	type			= 1;
    	uint32	battery 		= 2;
    }
    
    message Downlink {
    	uint32	pingDelay		= 1;	// seconds
    }
In this example, the LoRaWAN uplink message is encoded with these data:

 1. A type which may be PING or BUTTON. PING is sent on timed regular basis, BUTTON is sent when an interrupt occurs.
 2. The battery level in range 0..100

A LoRaWAN downlink message may be sent by the server to the device. In this example, it contains the delay to use by the device to separate two PING messages.

The .proto file may have options. In our case it is:

    leuville.Uplink.battery		int_size:8
    leuville.Downlink.pingDelay	int_size:16

These definitions have to be compiled with Nanopb protoc compiler. It generates .h and .c files.

Then the endnode is defined as a template class based on protobuf generated types:

    using Base = ProtobufEndnode<
    	leuville_Uplink, leuville_Uplink_fields,
    	leuville_Downlink, leuville_Downlink_fields
    >;
    
    /*
     * LoraWan + ProtocolBuffer endnode with:
     * - a timer to trigger a PING message each 5 mn
     * - a callback set on button connected to A0 pin, which triggers a BUTTON message
     * - standby mode capacity
     */
    class EndNode : public Base, ISRTimer, ISRWrapper<A0>, StandbyMode { // ... };
    
## Platformio flags

This platformio.ini sample shows how to compile using VSCode + PlatformIO plugin.

    [env:adafruit_feather_m0]
    platform = atmelsam
    board = adafruit_feather_m0
    framework = arduino
    lib_deps = 
        leuville-common-init=C:\Dev\Arduino\LO_libraries\leuville-common-init
        Nanopb@0.4.1
        MCCI LoRaWAN LMIC library@2.3.2 ; 3.1.0 bug ?
        RTCZero
        Adafruit Unified Sensor
        Adafruit BME680 Library
        Wire
    lib_extra_dirs = 
        C:\Dev\Arduino\LO_libraries
        C:\Dev\Arduino\extra_libraries
    build_flags =
        -D ARDUINO_SAMD_FEATHER_M0
        -D ARDUINO_LMIC_PROJECT_CONFIG_H_SUPPRESS
        -D LMIC_LORAWAN_SPEC_VERSION=LMIC_LORAWAN_SPEC_VERSION_1_0_2
        -D CFG_eu868
        -D CFG_sx1276_radio
        -D DISABLE_PING
        -D DISABLE_BEACONS
        -D USE_ORIGINAL_AES
        -D LMIC_USE_INTERRUPTS

 


<!--stackedit_data:
eyJoaXN0b3J5IjpbMTM3MjU5MjA0OCwxMjMwODM3NTgyLDE4MD
k4NzY3NjAsODQzOTI1OTcyLC0zNzg1NjQ2MCwtMjQ2NTcxOTc2
LC0yMDYzMTA5NjY3LC0xNzEwNzM3MDYyLDY2NjI0MDk4MywxNz
YyMDE3NzIxLC0xNzM4NzQ3Mzk2LC01MzUzNjE5MDRdfQ==
-->