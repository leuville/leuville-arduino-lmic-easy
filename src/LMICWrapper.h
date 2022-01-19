/*
 * Module: LMICWrapper
 *
 * Function: classes for Object-Oriented approach of LMIC
 *
 * Copyright and license: See accompanying LICENSE file.
 *
 * Author: Laurent Nel
 */

#pragma once

#include <fixed-deque.h>

#include <Arduino_lmic.h>
#include <hal/hal.h>
#include <lmic/oslmic.h>

#include <misc-util.h>

#ifndef LEUVILLE_LORA_QUEUE_LEN
#define LEUVILLE_LORA_QUEUE_LEN 10
#endif

namespace lstl = leuville::simple_template_library;

using namespace lstl;

namespace leuville {
namespace lora {

/*
 * LoRaWAN configuration: OTAA keys
 * 
 * This struct may be used like this:
 * 		enum Config { TTN, OPE1, OPE2, OPE3, ANOTHER1, ANOTHER2 };
 * 		OTAAId id[] = {
 *			  // APPEUI			  // DEVEUI			  // APPKEY
 *			{ "70B3D57EXXXXXXXX", "0000A06EXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" }, 	
 *			{ "7BB592C0XXXXXXXX", "A1BA1800XXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" }, 	
 *			{ "7BB592C0XXXXXXXX", "A2BAXXXXXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" }, 	
 *			{ "7BB592C0XXXXXXXX", "A3BA1XXXXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" }, 	 
 *			{ "70B3D59BXXXXXXXX", "70B3D5XXXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" },		
 * 			{ "7BB592C0XXXXXXXX", "000000XXXXXXXXXX", "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" }		
 * 		};
 * 		endnode.begin(id[Config::TTN], ...); // or Config::OPE1 etc ...
 */
struct OTAAId {

	OTAAId() {}
	/*
	 * AppEUI and DevEUI are NOT reordered by this constructor
	 */
	OTAAId(const u1_t *appEUI, const u1_t *devEUI, const u1_t *appKEY)
	{
		memcpy(_appEUI, appEUI, arrayCapacity(_appEUI));
		memcpy(_devEUI, devEUI, arrayCapacity(_devEUI));
		memcpy(_appKEY, appKEY, arrayCapacity(_appKEY));
	}
	/*
	 * appEUI and devEUI are reordered by this constructor
	 * 
	 * ie 10FF becomes FF10
	 */
	OTAAId(const char* appEUI, const char* devEUI, const char* appKEY)
	{
		hexCharacterStringToBytes(loraString(appEUI), _appEUI);
		hexCharacterStringToBytes(loraString(devEUI), _devEUI);
		hexCharacterStringToBytes(String(appKEY), _appKEY);
	}

	// LoRaWAN OTAA keys
	
	u1_t _appEUI[8] = { 0 };		
	u1_t _devEUI[8] = { 0 };		
	u1_t _appKEY[16]= { 0 };		
};

/*
 * LoRaWan session keys
 */
struct LoRaWanSessionKeys {
	u4_t _netId = 0;
    devaddr_t _devAddr = 0;
    u1_t _nwkSKey[16] = { 0 };
    u1_t _appSKey [16] = { 0 };

	void set() {
		LMIC_getSessionKeys(&_netId, &_devAddr, _nwkSKey, _appSKey);
	}
};

/* 
 * LMIC Channel
 */
struct LMICChannel {
	u1_t _chidx;
	u4_t _freq;
	u2_t _drmap; 
	s1_t _band;

	LMICChannel(u1_t chidx, u4_t freq, u2_t drmap, s1_t band)
		: _chidx(chidx), _freq(freq), _drmap(drmap), _band(band)
	{}
};

void initLMICChannels(LMICChannel *channels, u1_t nb) {
	for (u1_t i = 0; i < nb; i++) {
		LMIC_setupChannel(channels[i]._chidx, channels[i]._freq, channels[i]._drmap, channels[i]._band);
	}
}

/* 
 * Message buffer = uint8_t array + size
 * Acts as a base class for UpstreamMessage and DownstreamMessage
 */
constexpr uint8_t MAX_MESSAGE_LEN = MAX_FRAME_LEN;

struct Message {
	uint8_t _buf[MAX_MESSAGE_LEN] = { 0 };
	uint8_t _len = 0;

	Message() {}
	Message(uint8_t* buf, uint8_t len)
		: _len(len)
	{
		memcpy(_buf, buf, _len);
	}
};

/*
 * UpStream message = message buffer + ack request
 */
struct UpstreamMessage : Message {
	bool _ackRequested = false;
	UpstreamMessage() {}
	UpstreamMessage(uint8_t* buf, uint8_t len, bool ackRequested = false)
		: Message(buf, len), _ackRequested(ackRequested)
	{}
};

/*
 * Downstream message = message buffer
 */
struct DownstreamMessage : Message {
	using Message::Message;
};

/*
 * forward declarations
 */
void onEvent(ev_t ev);

constexpr uint32_t defaultNetworkTimeSyncDelay = 24 * 60 * 60;

/*
 * LMIC base class
 * - contains singleton reference needed by LMIC callbacks (delegation)
 * - manages LoRaWAN OTAA keys
 */
class LMICWrapper {
public:

	using LMICdeque = deque<UpstreamMessage, true, LEUVILLE_LORA_QUEUE_LEN>;

	enum {
		KEEP_RECENT	= LMICdeque::KEEP_FRONT,
		KEEP_OLD 	= LMICdeque::KEEP_BACK
	};

	friend void ::onEvent(ev_t ev);
	friend void ::os_getArtEui(u1_t* buf);
	friend void ::os_getDevEui(u1_t* buf);
	friend void ::os_getDevKey(u1_t* buf);

	/*
	 * The device send a network time sync request on a regular basis 
	 * (if networkTimeSyncDelay is not zero )
	 */
	LMICWrapper(const lmic_pinmap *pinmap, uint32_t networkTimeSyncDelay = defaultNetworkTimeSyncDelay, uint8_t policy = KEEP_RECENT)
		: _pinmap(pinmap), _messages(policy), _networkTimeSyncDelay(networkTimeSyncDelay)
	{
		LMICWrapper::_node = this;
	}

	virtual ~LMICWrapper() = default;

	/*
	 * Initializes LMIC
	 */
	virtual void begin(const OTAAId& env, u4_t network, bool adr = true) {
		_env = env;
		os_init_ex(_pinmap);
		LMIC_reset();
		initLMIC(network, adr);
	}

	/*
	 * Add a job to the callback list managed by LMIC
	 * if interval == 0 then callback is set with os_setCallback()
	 * else callback is set with os_setTimedCallback()
	 *
	 * interval = number of milliseconds from now
	 * exec time = current time + interval
	 */
	virtual void setCallback(osjob_t* job, unsigned long interval = 0) final {
		_sendJobRequested = (job == & _sendJob);
		_jobCount += 1;
		os_setTimedCallback(job, os_getTime() + ms2osticks(interval), LMICWrapper::jobCallback);
	}

	virtual void setCallback(osjob_t& job, unsigned long interval = 0) final {
		setCallback(&job, interval);
	}

	/*
	 * Clear a callback job
	 */
	virtual void unsetCallback(osjob_t* job) final {
		os_clearCallback(job);
	}

	/*
	 * Check if a callback job is registered
	 */
	virtual bool hasCallback(osjob_t* job) final {
		return os_jobIsTimed(job);
	}

	/*
	 * Return the time interval you have to wait to be able to send a message
	 * according to the duty cycle.
	 * 
	 * Returns a number of ms
	 */
	unsigned long dutyCycleWaitTimeInterval() {
		unsigned long now = millis();
		unsigned long when = osticks2ms(LMIC.globalDutyAvail);
		when = (when <= now ? 0 : when-now);  
		return when;
	}

	/*
	 * Must be called from loop()
	 *
	 * Register a sendJob callback if needed (messages waiting to be sent)
	 */
	virtual void runLoopOnce() final {
		// if msg pending and LMIC doing nothing -> create a new LMIC job
		if (!_sendJobRequested && hasMessageToSend() && !isRadioBusy()) {
			setCallback(&_sendJob, dutyCycleWaitTimeInterval());
		}
		os_runloop_once();
	}

	/*
	 * Push a message to the front of the FIFO waiting queue
	 *
	 * If FIFO is full, oldest message is removed if policy is KEEP_RECENT then current message is queued
	 * 
	 * Returns true if message queued, false otherwise
	 */
	virtual bool send(const UpstreamMessage & message) {
		return _messages.push_front(message);
	}

	/*
	 * Returns true is there at least one message waiting to be sent
	 */
	virtual bool hasMessageToSend() {
		return (_messages.size() > 0);
	}

	/* 
	 * Start JOIN sequence
	 */
	virtual void startJoining() {
		LMIC_startJoining();
	}

	/*
	 * Returns true if LMIC radio is pending
	 */
	bool isRadioBusy() {
		return (LMIC.opmode & OP_TXRXPEND) || (LMIC.opmode & OP_TXDATA);
	}

	/*
	 * Return true if the device can be put in standby mode.
	 */
	virtual bool isReadyForStandby() {
		return _joined && (_jobCount == 0) && !hasMessageToSend() && !isRadioBusy();
	}

	/*
	 * Returns LoRaWan session keys (netid, netaddr, nwskey, appskey) 
	 */
	const LoRaWanSessionKeys & getSessionKeys() {
		return _sessionKeys;
	}

	/*
	 * May be overriden to update system time when network time is received
	 */
	virtual void updateSystemTime(uint32_t newTime) {
	}

	static LMICWrapper & node() { 
		return *_node; 
	}

protected:

	// singleton
	static LMICWrapper* _node;

	// device pinmap
	const lmic_pinmap *_pinmap;

	// LoRaWAN environment
	OTAAId _env;
	LoRaWanSessionKeys _sessionKeys;

	// active osjob counter
	int _jobCount = 0;

	osjob_t _sendJob;
	bool _sendJobRequested = false;

	// LoRaWAN JOIN done ?
	bool _joined = false;

	// FIFO messages waiting to be sent
	LMICdeque _messages;			

	// members to manage network time sync
	uint32_t _lastNetworkTimeSync = 0;
	const uint32_t _networkTimeSyncDelay = defaultNetworkTimeSyncDelay; 

	/*
	 * Network time callback
	 */
	static void networkTimeCallback(void *paramUserUTCTime, int flagSuccess) {
		if (flagSuccess == 0)
			return;

		uint32_t newTime = 0;

		// A struct that will be populated by LMIC_getNetworkTimeReference.
		// It contains the following fields:
		//  - tLocal: the value returned by os_GetTime() when the time
		//            request was sent to the gateway, and
		//  - tNetwork: the seconds between the GPS epoch and the time
		//              the gateway received the time request
		lmic_time_reference_t lmicTimeReference;

		if (LMIC_getNetworkTimeReference(&lmicTimeReference) == 0) {
			return;
		}

		// Update userUTCTime, considering the difference between the GPS and UTC
		// epoch, and the leap seconds
		newTime = lmicTimeReference.tNetwork + 315964800;

		// Add the delay between the instant the time was transmitted and
		// the current time

		// Current time, in ticks
		ostime_t ticksNow = os_getTime();
		// Time when the request was sent, in ticks
		ostime_t ticksRequestSent = lmicTimeReference.tLocal;
		uint32_t requestDelaySec = osticks2ms(ticksNow - ticksRequestSent) / 1000;
		newTime += requestDelaySec;

		// time effective update is done by instance (polymorphism)
		LMICWrapper::_node->updateSystemTime(newTime);
		LMICWrapper::_node->_lastNetworkTimeSync = newTime;
	}

	/*
	 * Set ADR
	 */
	virtual void initLMIC(u4_t network = 0, bool adr = true) {
		LMIC_setAdrMode(adr ? 1 : 0);
	}

	/*
	 * Main LMIC job callback
	 */
	static void jobCallback(osjob_t* job) { 
		LMICWrapper::_node->performJob(job); 
	}

	/*
	 * Called by LMIC when a given job should be performed
	 * This job has been previously registered with setCallback()
	 * If other callbacks are used, override completeJob()
	 */
	virtual void performJob(osjob_t* job) final {
		_jobCount -= 1;
		if (job == &_sendJob) {
			_sendJobRequested = false;
			if (!isRadioBusy()) {
				lmicSend();
			}
		} else {
			completeJob(job);
		}
	}

	/*
	 * This method should be overriden by subclass if other callbacks are used
	 */
	virtual void completeJob(osjob_t* job) {
	}

	/*
	 * This method should be called to get the network time
	 * update done by call to virtual method updateSystemTime(uint32_t)
	 */
	virtual void requestNetworkTime() {
		LMIC_requestNetworkTime(LMICWrapper::networkTimeCallback, nullptr);
	} 

	/*
	 * Calls LMIC_setTxData2() if message to send
	 * request a network time update if delay exceeded
	 */
	virtual void lmicSend() {
		UpstreamMessage* msg = _messages.backPtr(); 
		if (msg != nullptr) {
			#if defined(LMIC_ENABLE_DeviceTimeReq)
			if ( _joined) {
				uint32_t now = osticks2ms(os_getTime()) / 1000;
				if (_lastNetworkTimeSync == 0 ||  (_lastNetworkTimeSync + _networkTimeSyncDelay) < now) {
					requestNetworkTime();
				}
			}
			#endif
			LMIC_setTxData2(1, msg->_buf, msg->_len, msg->_ackRequested);
		} 
	}

	/*
	  * LMIC event callback
	  * enum _ev_t { EV_SCAN_TIMEOUT=1, EV_BEACON_FOUND,
      *       EV_BEACON_MISSED, EV_BEACON_TRACKED, EV_JOINING,
      *       EV_JOINED, EV_RFU1, EV_JOIN_FAILED, EV_REJOIN_FAILED,
      *       EV_TXCOMPLETE, EV_LOST_TSYNC, EV_RESET,
      *       EV_RXCOMPLETE, EV_LINK_DEAD, EV_LINK_ALIVE, EV_SCAN_FOUND,
      *       EV_TXSTART, EV_TXCANCELED, EV_RXSTART, EV_JOIN_TXCOMPLETE };
	  */
	virtual void onEvent(ev_t ev) {
		switch (ev) {
			case EV_JOINED:
				_joined = true;
				_sessionKeys.set();
				joined(true);
				break;
			case EV_JOIN_FAILED:
			case EV_REJOIN_FAILED:
			case EV_RESET:
			case EV_LINK_DEAD:
				_joined = false;
				_sessionKeys = LoRaWanSessionKeys();
				joined(false);
				break;
			case EV_TXCOMPLETE:
				txComplete();
				break;
			default:
				break;
		}
	}
	
	/*
	 * Called if endnode is joined or not
	 * May be overriden
	 */
	virtual void joined(bool) {}

	/*
	 * Called by onEvent() calback
	 *
	 *  removes sent message from the FIFO to avoid another transmission
	 */
	virtual void txComplete() {
		UpstreamMessage *sent = _messages.backPtr(); 
		if (sent != nullptr) {
			if (isTxCompleted(*sent, (LMIC.txrxFlags & TXRX_NACK) == 0)) {
				_messages.pop_back(); // message is removed from FIFO
			}
		}
		// check if downlink message (RX Window)
		if (LMIC.dataLen > 0) {
			uint8_t buf[MAX_FRAME_LEN];
			for (uint8_t i = 0; i < LMIC.dataLen; i++) {
				buf[i] = (uint8_t)LMIC.frame[LMIC.dataBeg + i];
			}
			downlinkReceived(DownstreamMessage(buf, LMIC.dataLen));
		}
	}

	/*
	 * Default send completion policy
	 */
	virtual bool isTxCompleted(const UpstreamMessage & message, bool ack) {
		return ack;
	};

	/*
	 * Downlink message arrival callback
	 * 
	 * Override if needed
	 */
	virtual void downlinkReceived(const DownstreamMessage&) {
	}

};

/*
 * LoRaWAN endnode singleton
 */
LMICWrapper * LMICWrapper::_node = nullptr;

}
}

/*
 * LMIC callbacks
 * call delegation to LMICWrapper singleton
 */
void os_getArtEui (u1_t* buf) 	{ memcpy_P(buf, leuville::lora::LMICWrapper::_node->_env._appEUI, 8);}
void os_getDevEui (u1_t* buf) 	{ memcpy_P(buf, leuville::lora::LMICWrapper::_node->_env._devEUI, 8);}
void os_getDevKey (u1_t* buf) 	{ memcpy_P(buf, leuville::lora::LMICWrapper::_node->_env._appKEY, 16);}
void onEvent(ev_t ev) 			{ leuville::lora::LMICWrapper::_node->onEvent(ev); }

