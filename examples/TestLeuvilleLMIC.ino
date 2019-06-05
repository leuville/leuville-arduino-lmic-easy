/*
 * TestLeuvilleLMIC
 */

#include <Arduino.h>

#include <LMICWrapper.h>
#include <RTCZero.h>
#include <misc-util.h>
#include <ISRWrapper.h>
#include <energy.h>
#include <StatusLed.h>

#include "message.pb.h"

/* ---------------------------------------------------------------------------------------
 * PIN mappings
 *
 * (!) DO NOT FORGET TO CONNECT DIO1 with D6 on Feather M0 LoRa board
 * ---------------------------------------------------------------------------------------
 */
#define ARDUINO_SAMD_FEATHER_M0	1

const lmic_pinmap feather_m0_lora_pins = {
	.nss			= 8, // CS
	.rxtx			= LMIC_UNUSED_PIN,
	.rst			= LMIC_UNUSED_PIN,
	.dio			= {3, 6, LMIC_UNUSED_PIN},	// {DIO0 = IRQ, DIO1, DIO2}
	.rxtx_rx_active = 0,
	.rssi_cal		= 8, // LBT cal for the Adafruit Feather M0 LoRa, in dB
	.spi_freq		= 8000000
};

/* ---------------------------------------------------------------------------------------
 * Application classes
 * ---------------------------------------------------------------------------------------
 */
using Base = ProtobufEndnode<leuville_Uplink, leuville_Uplink_size, leuville_Downlink, leuville_Downlink_size>;

/*
 * LoraWan + ProtocolBuffer endnode with:
 * - a timer to trigger a PING message each 5 mn
 * - a callback set on button connected to A0 pin, which triggers a BUTTON message
 * - standby mode capacity
 */
class EndNode : public Base, ISRTimer, ISRWrapper<A0>, StandbyMode {

	osjob_t _buttonJob;
	osjob_t _timeoutJob;

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
		ISRWrapper::enable();
		StandbyMode::begin();

		leuville_Uplink payload = {
				.type = leuville_Type_PING,
				.battery = getRemainingPower()
		};
		send(payload, true);
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

	virtual const pb_field_t* getUplinkMessagePBFields() override {
		return leuville_Uplink_fields;
	}
	virtual const pb_field_t* getDownlinkMessagePBFields() override {
		return leuville_Downlink_fields;
	}

	virtual void joined() override {
		ISRTimer::enable();
	}

	virtual void txComplete(const leuville_Uplink& payload, boolean ackRequested, boolean received) override {

	};

	virtual void downlinkReceived(const leuville_Downlink& payload) override {
		ISRTimer::setTimeout(payload.pingDelay);
	}

	virtual void performJob(osjob_t* job) override {
		if (job == &_buttonJob) {
			leuville_Uplink payload = {
					.type = leuville_Type_BUTTON,
					.battery = getRemainingPower()
			};
			send(payload, true);
		} else if (job == &_timeoutJob) {
			leuville_Uplink payload = {
					.type = leuville_Type_PING,
					.battery = getRemainingPower()
			};
			send(payload, false);
		} else {
			Base::performJob(job);
		}
	}

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
	feather_m0_lora_pins,
	{ 0x03, 0x00, 0x00, 0xDA, 0xC0, 0x92, 0xB5, 0x7B },
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xBA, 0xA1 },
	{ 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C }
);

// LMIC PINS
const lmic_pinmap lmic_pins = env._pinmap;

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
