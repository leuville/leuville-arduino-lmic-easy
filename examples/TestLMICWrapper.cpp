/*
 * TestLMICWrapper
 */

// leuville-arduino-easy-lmic
#include <LMICWrapper.h>
// leuville-arduino-utilities
#include <misc-util.h>
#include <ISRWrapper.h>
#include <energy.h>
#include <StatusLed.h>

#include <lora-common-defs.h>

/* ---------------------------------------------------------------------------------------
 * Application classes
 * ---------------------------------------------------------------------------------------
 */

/*
 * LoraWan endnode with:
 * - a timer to trigger a PING message each 5 mn
 * - a callback set on button connected to A0 pin, which triggers a BUTTON message
 * - standby mode capacity
 */
using Base = LMICWrapper;

class EndNode : public Base, ISRTimer, ISRWrapper<A0>, StandbyMode {

	/*
	 * Jobs for event callbacks
	 */
	osjob_t _buttonJob;
	osjob_t _timeoutJob;
	osjob_t _joinJob;
	osjob_t _txCompleteJob;

	int _count = 0;

public:

	EndNode(const lmic_pinmap *pinmap, RTCZero& rtc): 
		Base(pinmap),
		ISRTimer(rtc, 3 * 60, true),
		ISRWrapper<A0>(INPUT_PULLUP, LOW),
		StandbyMode(rtc)
	{
	}

	/*
	 * delegates begin() to each sub-component and send a first message
	 */
	virtual void begin(const OTAAId& id) override {
		ISRTimer::begin(true);
		Base::begin(id);
		StandbyMode::begin();
		ISRWrapper<A0>::begin();

		ISRWrapper<A0>::enable();
		setCallback(_buttonJob, 2000); // send first message with 2s delay
	}

	/*
	 * Button ISR
	 * job done by sending a LMIC callback
	 * see completeJob()
	 */
	virtual void ISR_callback(uint8_t pin) override {
		setCallback(_buttonJob);
	}

	/*
	 * Timer ISR
	 * job done by sending a LMIC callback
	 * see completeJob()
	 */
	virtual void ISR_timeout() override {
		setCallback(_timeoutJob);
	}

	/*
	 * Job done on join/unjoin
	 * see completeJob()
	 */
	virtual void joined(bool ok) override {
		if (ok) {
			setCallback(_joinJob);
		}
	}

	/*
	 * Updates the timer delay
	 */
	virtual void downlinkReceived(const DownstreamMessage & message)  {
		if (message._len > 0) {
			ISRTimer::setTimeout((uint32_t)strtoul((char*)message._buf, nullptr, 10));
		}
	}

	/*
	 * Handle LMIC callbacks
	 */
	virtual void completeJob(osjob_t* job) override {
		if (job == &_buttonJob) {
			send("CLICK", true);
		} else if (job == &_timeoutJob) {
			const char* format = "TIMEOUT %d";
			char msg[80];
			sprintf(msg, format, _count++);
			send(msg, false);
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
	 * Build and send Uplink message
	 */
	void send(const char* message, bool ack = false) {
		UpstreamMessage payload((uint8_t*)message, strlen(message)+1, ack);
		Serial.print("send ");Serial.println(message);
		Base::send(payload);
	}

	/*
	 * LMIC callback called on TX_COMPLETE event
	 */
	virtual bool isTxCompleted(const UpstreamMessage & message, bool ack) override {
		setCallback(_txCompleteJob);
		Serial.print("isTxCompleted ");Serial.print((char*)message._buf); 
		Serial.print(" / ");Serial.println(ack);
		return Base::isTxCompleted(message, ack);
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
		//endnode.standby(); 	// uncomment to enable powersaving mode
	}
	else {
		statusLed.blink();
	}
}
