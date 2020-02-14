/*
 * TestProtobufEndnode
 */

// leuville-arduino-easy-lmic
#include <LMICWrapper.h>
// leuville-arduino-utilities
#include <misc-util.h>
#include <ISRWrapper.h>
#include <energy.h>
#include <StatusLed.h>

// nanopb (Protocol Buffer)
#include "message.pb.h"

// LoRaWAN configuration: see OTAAId in LMICWrapper.h
#include <lora-common-defs.h>

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
	leuville_Uplink, leuville_Uplink_fields,
	leuville_Downlink, leuville_Downlink_fields
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
	osjob_t _joinJob;
	osjob_t _txCompleteJob;

	/*
	 * Message send retry mechanism
	 */
	uint8_t _nbRetries = 0;
	uint8_t _maxRetries = 1;

public:

	EndNode(const lmic_pinmap *pinmap, RTCZero& rtc): 
		Base(pinmap),
		ISRTimer(rtc, 1 * 60, true),
		ISRWrapper<A0>(INPUT_PULLUP, LOW),
		StandbyMode(rtc)
	{
	}

	/*
	 * delegates begin() to each sub-component and send a PING message
	 */
	void begin(const OTAAId& id) {
		ISRTimer::begin(true);
		Base::begin(id);
		StandbyMode::begin();
		ISRWrapper<A0>::begin();

		ISRWrapper<A0>::enable();
		setCallback(&_buttonJob, 2000); // send first message
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

	/*
	 * Job done on join/unjoin
	 * see completeJob()
	 */
	virtual void joined(boolean ok) override {
		if (ok) {
			setCallback(_joinJob);
		}
	}

	/*
	 * Implements a retry policy for important messages in case of unsuccessful transmission
	 */
	virtual bool isTxCompleted(const UpstreamMessage & message, bool ack) override {
		setCallback(_txCompleteJob);
		leuville_Uplink payload;
		decode(message, leuville_Uplink_fields, payload);
		Serial.print("isTxCompleted ");Serial.print(payload.battery);
		Serial.print(" ");Serial.print(payload.type);Serial.print(" ");
		Serial.print(message._ackRequested); Serial.print(" ");Serial.println(ack);
		if (ack || !message._ackRequested) {
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

	/*
	 * Updates the timer delay
	 */
	virtual void downlinkReceived(const DownstreamMessage & message) override {
		leuville_Downlink payload;
		if (decode(message, leuville_Downlink_fields, payload)) {
			Serial.print("downlink delay: "),Serial.println(payload.pingDelay);
			ISRTimer::setTimeout(payload.pingDelay);
		}
	}

	/*
	 * Handle LMIC callbacks
	 */
	virtual void completeJob(osjob_t* job) override {
		if (job == &_buttonJob) {
			ISRTimer::disable();
			ISRTimer::enable();
			send(buildPayload(leuville_Type_BUTTON), true); 
		} else if (job == &_timeoutJob) {
			if (!hasMessageToSend()) {
				send(buildPayload(leuville_Type_PING), false);
			}
		} else if (job == &_joinJob) {
			LoRaWanSessionKeys keys = getSessionKeys();
			// https://www.thethingsnetwork.org/docs/lorawan/prefix-assignments.html
			Serial.print("netId: "); Serial.println(keys._netId, HEX);
			Serial.print("devAddr: "); Serial.println(keys._devAddr, HEX);
			Serial.print("nwkSKey: "); printHex(keys._nwkSKey, arrayCapacity(keys._nwkSKey));
			Serial.print("appSKey: "); printHex(keys._appSKey, arrayCapacity(keys._appSKey));
			if (keys._netId == 0x000013) { // TTN
				LMIC_setLinkCheckMode(0);	
			}
			ISRTimer::enable();
		} else if (job == &_txCompleteJob) {
			Serial.print("FIFO size: ");Serial.println(_messages.size());
		}
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
	 * Post a message in the send queue
	 */
	void send(const leuville_Uplink & payload, bool ack = false) {
		Serial.print("send ");
		Serial.print(payload.battery); Serial.print(" ");Serial.println(payload.type);
		Base::send(payload, ack);
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
EndNode 	endnode(Arduino_LMIC::GetPinmap_ThisBoard(), rtc); 


/* ---------------------------------------------------------------------------------------
 * ARDUINO FUNCTIONS
 * ---------------------------------------------------------------------------------------
 */
void setup()
{
	Serial.begin(115200);

	statusLed.begin();
	statusLed.on();

	while (!Serial.available()) {}

	endnode.begin(id[Config::TTN]);

	delay(5000);
	statusLed.off();
}

void loop()
{
	endnode.runLoopOnce();
	if (endnode.isReadyForStandby()) {
		statusLed.off();
		//endnode.standby(); 		// uncomment to enable powersaving mode
	}
	else {
		statusLed.blink();
	}
}
