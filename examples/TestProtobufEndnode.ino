/*
 * TestProtobufEndnode
 */

#include <Arduino.h>

#define ARDUINO_SAMD_FEATHER_M0	1

// leuville-arduino-easy-lmic
#include <LMICWrapper.h>
// leuville-arduino-utilities
#include <misc-util.h>
#include <ISRWrapper.h>
#include <energy.h>
#include <StatusLed.h>

// nanopb (Protocol Buffer)
#include "message.pb.h"

/* 
 * ---------------------------------------------------------------------------------------
 * PIN mappings
 *
 * (!) DO NOT FORGET TO CONNECT DIO1 with D6 on Feather M0 LoRa board
 * ---------------------------------------------------------------------------------------
 */
const lmic_pinmap feather_m0_lora_pins = {
	.nss			= 8,						// CS
	.rxtx			= LMIC_UNUSED_PIN,
	.rst			= LMIC_UNUSED_PIN,
	.dio			= {3, 6, LMIC_UNUSED_PIN},	// {DIO0 = IRQ, DIO1, DIO2}
	.rxtx_rx_active = 0,
	.rssi_cal		= 8,						// LBT cal for the Adafruit Feather M0 LoRa, in dB
	.spi_freq		= 8000000
};

/* ---------------------------------------------------------------------------------------
 * Application classes
 * ---------------------------------------------------------------------------------------
 */

/*
 * Template base class of our device
 *
 * Instanciated with nanopb generated types for uplink & downlink messages
 */
using Base = ProtobufEndnode<
	leuville_Uplink, leuville_Uplink_size, leuville_Uplink_fields,
	leuville_Downlink, leuville_Downlink_size, leuville_Downlink_fields
>;

/*
 * LoraWan + ProtocolBuffer endnode with:
 * - a timer to trigger a PING message each 5 mn
 * - a callback set on button connected to A0 pin, which triggers a BUTTON message
 * - standby mode capacity
 */
class EndNode : public Base, ISRTimer, ISRWrapper<A0>, StandbyMode {

	/*
	 * Jobs for event callbacks
	 */
	osjob_t _buttonJob;
	osjob_t _timeoutJob;

	/*
	 * Message send retry mechanism
	 */
	uint8_t _nbRetries = 0;
	uint8_t _maxRetries = 1;

public:

	EndNode(RTCZero& rtc, const LoRaWANEnv& env)
		: Base(env),
		ISRTimer(rtc, 5 * 60, true),
		ISRWrapper<A0>(INPUT_PULLUP, LOW),
		StandbyMode(rtc)
	{
	}

	/*
	 * delegates begin() to each sub-component and send a PING message
	 */
	void begin() {
		ISRTimer::_rtc.begin(true);
		ISRTimer::_rtc.setDate(1, 1, 0);
		Base::begin();
		StandbyMode::begin();
		ISRWrapper<A0>::begin();

		ISRWrapper<A0>::enable();
		setCallback(_timeoutJob);
	}

	/*
	 * Button ISR
	 * job done by sending a LMIC callback
	 * see performJob()
	 */
	virtual void ISR_callback(uint8_t pin) override {
		setCallback(_buttonJob);
	}

	/*
	 * Timer ISR
	 * job done by sending a LMIC callback
	 * see performJob()
	 */
	virtual void ISR_timeout() override {
		setCallback(_timeoutJob);
	}

	virtual void joined(boolean ok) override {
		if (ok) {
			ISRTimer::enable();
		}
		else {
			ISRTimer::disable();
		}
	}

	/*
	 * In all cases: must call Base::postSend() to remove sent message from the FIFO and avoid another transmission
	 * this job is done by the Base implementation of txComplete()
	 * 
	 * In our case: implements a retry policy for important messages in case of unsuccessful transmission
	 */
	virtual boolean isTxCompleted(const leuville_Uplink& payload, boolean ackRequested, boolean ack) override {
		if (ack || !ackRequested) {
			_nbRetries = 0;
			return true;
		} else {
			if (++_nbRetries > _maxRetries) {
				_nbRetries = 0;
				return true;
			}
		}
		return false;
	};

	virtual void downlinkReceived(const leuville_Downlink& payload) override {
		ISRTimer::setTimeout(payload.pingDelay);
	}

	virtual void performJob(osjob_t* job) override {
		if (job == &_buttonJob) {
			ISRTimer::disable();
			ISRTimer::enable();
			send(buildPayload(leuville_Type_BUTTON), false); // true -> ACK -> slower
		} else if (job == &_timeoutJob) {
			if (!hasMessageToSend()) {
				send(buildPayload(leuville_Type_PING), false);
			}
		}
		// ***************
		// DO NOT REMOVE !
		// ***************
		Base::performJob(job);
	}

	/*
	 * Builds an uplink message
	 */
	leuville_Uplink buildPayload(leuville_Type uplinkType) {
		leuville_Uplink payload = leuville_Uplink_init_zero;
		payload.battery = getBatteryPower();
		payload.type = uplinkType;
		return payload;
	}

	/*
	 * Activates standby mode
	 */
	void standby() {
		StandbyMode::standby();
	}

};

/* ---------------------------------------------------------------------------------------
 * GLOBAL OBJECTS
 * ---------------------------------------------------------------------------------------
 */
RTCZero 	rtc;
BlinkingLed statusLed(LED_BUILTIN, 500);

LoRaWANEnv env(
	// APPEUI
	{ 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA },
	// DEVEUI
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE },
	// APPKEY
	{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
	// PINS
	feather_m0_lora_pins
);

EndNode endnode(rtc, env);

/* ---------------------------------------------------------------------------------------
 * ARDUINO FUNCTIONS
 * ---------------------------------------------------------------------------------------
 */
void setup()
{
	Serial.begin(115200);
	delay(1000);

	statusLed.begin();
	statusLed.on();

	endnode.begin();

	delay(5000);
	statusLed.off();
}

void loop()
{
	endnode.runLoopOnce();
	if (endnode.isReadyForStandby()) {
		statusLed.off();
		endnode.standby();
	}
	else {
		statusLed.blink();
	}
}
