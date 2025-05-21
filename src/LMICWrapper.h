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

#include <Arduino_lmic.h>
#include <hal/hal.h>
#include <lmic/oslmic.h>

#include <misc-util.h>
#include <fixed-deque.h>

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
	u4_t 		_netId = 0;
    devaddr_t 	_devAddr = 0;
    u1_t 		_nwkSKey[16] = { 0 };
    u1_t 		_appSKey [16] = { 0 };

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
	uint8_t 		_buf[MAX_MESSAGE_LEN] = { 0 };
	uint8_t 		_len = 0;
	u1_t 			_txrxFlags = 0;	// set after send or receive

	Message() {}
	Message(uint8_t* buf, uint8_t len, u1_t txrxFlags = 0)
		: _len(len), _txrxFlags(txrxFlags)
	{
		memcpy(_buf, buf, _len);
	}
	bool isAcknowledged() const {
		return (_txrxFlags & TXRX_ACK) != 0;
	}
};

/*
 * UpStream message = message buffer + ack request
 */
struct UpstreamMessage : Message {
	bool 			_ackRequested = false;
	lmic_tx_error_t _lmicTxError = 0; // set after send 

	UpstreamMessage() {}
	UpstreamMessage(uint8_t* buf, uint8_t len, bool ackRequested = false, u1_t txrxFlags = 0, lmic_tx_error_t lmicTxError = 0)
		: Message(buf, len, txrxFlags), _ackRequested(ackRequested), _lmicTxError(lmicTxError)
	{}
};

/*
 * Downstream message = message buffer
 */
struct DownstreamMessage : Message {
	using Message::Message;
};

// forward for friendship
void onLMICEvent(void *pUserData, ev_t ev);

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

	friend void leuville::lora::onLMICEvent(void *pUserData, ev_t ev);
	friend void ::os_getArtEui(u1_t* buf);
	friend void ::os_getDevEui(u1_t* buf);
	friend void ::os_getDevKey(u1_t* buf);

	/*
	 * Constructor
	 */
	LMICWrapper(const lmic_pinmap *pinmap,  uint8_t policy = KEEP_RECENT)
		: _pinmap(pinmap), _messages(policy)
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
		LMIC_registerEventCb(&leuville::lora::onLMICEvent, nullptr);
		LMIC_reset();
		initLMIC(network, adr);
	}

	/*
	 * Add a job to the callback list managed by LMIC
	 *
	 * interval = number of milliseconds from now
	 * exec time = current time + interval
	 */
	virtual void setCallback(osjob_t* job, unsigned long interval = 0) final {
		_sendJobRequested = (job == & _sendJob);
		auto now = os_getTime();
		auto when = now + ms2osticks(interval);
		when = (job == &_sendJob ? max(now + ms2osticks(dutyCycleWaitTimeInterval()),when) : when);
		_jobCount += 1;
		os_setTimedCallback(job, when, LMICWrapper::jobCallback);
	}

	virtual void setCallback(osjob_t& job, unsigned long interval = 0) final {
		setCallback(&job, interval);
	}

	/*
	 * Clear a callback job
	 */
	virtual void unsetCallback(osjob_t* job) final {
		_jobCount -= 1;
		os_clearCallback(job);
	}

	virtual void unsetCallback(osjob_t& job) final {
		unsetCallback(&job);
	}

	/*
	 * Return the time interval you have to wait to be able to send a message
	 * according to the duty cycle.
	 * 
	 * Returns a number of ms
	 */
	unsigned long dutyCycleWaitTimeInterval() {
		unsigned long now = osticks2ms(os_getTime());
		unsigned long when = osticks2ms(LMIC.globalDutyAvail);
		when = (when <= now ? 0 : when-now);  
		return when;
	}

	/*
	 * Must be called from loop()
	 */
	virtual void runLoopOnce() final {
		if (!_sendJobRequested && hasMessageToSend() && !isRadioBusy()) {
			setCallback(&_sendJob);
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
		return(_messages.size() > 0);
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
		return (LMIC.opmode & OP_TXRXPEND) || (LMIC.opmode & OP_TXDATA) || (LMIC.opmode & OP_POLL) || (LMIC.opmode & OP_JOINING);
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
	 * This method should be called to get the network time
	 * update done by call to virtual method updateSystemTime(uint32_t)
	 */
	virtual void requestNetworkTime() {
		#if defined(LMIC_ENABLE_DeviceTimeReq)
		if (_joined) {
			LMIC_requestNetworkTime(LMICWrapper::networkTimeCallback, nullptr);
		}
		#endif
	} 

	/*
	 * May be overriden to update system time when network time is received
	 */
	virtual void updateSystemTime(uint32_t newTime) {
	}

	virtual bool isSystemTimeSynced() {
		return _systemTimeSynced;
	}

	static LMICWrapper & node() { 
		return *_node; 
	}

	/*
	 * Set battery level for DevStatusAns
	 */
    u1_t setBatteryLevel(uint8_t percentage) {
		auto level = MCMD_DEVS_BATT_MIN + (MCMD_DEVS_BATT_MAX - MCMD_DEVS_BATT_MIN) * percentage / 100;
		return LMIC_setBatteryLevel(level);
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

	#if defined(LMIC_ENABLE_DeviceTimeReq)
	osjob_t _timeJob;
	#endif

	// LoRaWAN JOIN done ?
	bool _joined = false;

	// FIFO messages waiting to be sent
	LMICdeque _messages;		

	// Got systemTime from network ?
	bool _systemTimeSynced = false;

	/*
	 * Network time callback
	 */
	static void networkTimeCallback(void *paramUserUTCTime, int flagSuccess) {
		LMICWrapper::_node->_systemTimeSynced = false;
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
		LMICWrapper::_node->_systemTimeSynced = true;
	}

	/*
	 * Set ADR, channels and clock error
	 */
	virtual void initLMIC(u4_t network = 0, bool adr = true) {
		LMIC_setAdrMode(adr ? 1 : 0);
		if (!adr) {
			#if defined(CFG_eu868)
			LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band 
			LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band 
			LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band 
			LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band 
			LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band 
			LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band 
			LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band 
			LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band 
			#endif
		}
		#if defined(CLOCK_ERROR) && defined(MAX_CLOCK_ERROR)
		LMIC_setClockError(MAX_CLOCK_ERROR * CLOCK_ERROR / 100);
		#endif
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
			lmicSend();
		#if defined(LMIC_ENABLE_DeviceTimeReq)
		} else if (job == &_timeJob) {
			requestNetworkTime();
		#endif
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
	 * Calls LMIC_setTxData2() if message to send
	 * request a network time update if delay exceeded
	 */
	virtual lmic_tx_error_t lmicSend() final {
		if (isRadioBusy())
			return LMIC_ERROR_TX_BUSY;
		UpstreamMessage * msg = _messages.backPtr(); 
		if (msg != nullptr) {
			msg->_lmicTxError = LMIC_setTxData2(1, msg->_buf, msg->_len, msg->_ackRequested);
			return msg->_lmicTxError;
		}
		return LMIC_ERROR_TX_FAILED;
	}

	/*
	 * LMIC event callback
	 */
	virtual void onUserEvent(ev_t ev) {
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
				joined(false);
				#if defined(LMIC_ENABLE_DeviceTimeReq)
				unsetCallback(&_timeJob);
				#endif
				unsetCallback(&_sendJob);
				LMIC_unjoinAndRejoin();
				#if defined(LMIC_ENABLE_DeviceTimeReq)
				_systemTimeSynced = false;
				#endif
				break;
			case EV_TXCOMPLETE:
				txComplete();
				#if defined(LMIC_ENABLE_DeviceTimeReq)
				if (! isSystemTimeSynced() && _joined) {
					setCallback(&_timeJob);
				}
				#endif
				break;
			default:
				break;
		}
	}
	
	/*
	 * Called if endnode is joined or not
	 * May be overriden
	 */
	virtual void joined(bool ok) {
	}

	/*
	 * Called by onEvent() calback
	 *
	 * removes sent message from the FIFO to avoid another transmission
	 */
	virtual void txComplete() { 
		UpstreamMessage *ptr = _messages.backPtr();
		if (ptr != nullptr) {
			ptr->_txrxFlags = LMIC.txrxFlags;
			if (isTxCompleted(*ptr)) {
				_messages.pop_back(); // message is removed from FIFO
			}
		}
		// check if downlink message (RX Window) or MAC command
		if (isMACCommand(LMIC.frame)) {  
			macCommandReceived(LMIC.frame);
		} else if (LMIC.dataLen > 0) {
			uint8_t buf[MAX_FRAME_LEN];
			for (uint8_t i = 0; i < LMIC.dataLen; i++) {
				buf[i] = (uint8_t)LMIC.frame[LMIC.dataBeg + i];
			}
			downlinkReceived(DownstreamMessage(buf, LMIC.dataLen, LMIC.txrxFlags));
		} 
	}

	/*
	 * Check if frame is MAC command 
	 */
	virtual bool isMACCommand(uint8_t *frame) {
		uint8_t fctrl = frame[5];
		uint8_t foptsLen = fctrl & 0x0F;
	    uint8_t fportIndex = 8 + foptsLen;
    	uint8_t fport = frame[fportIndex];
		return (fport == 0) || (foptsLen > 0); 
	}

	/*
	 * Default send completion policy
	 */
	virtual bool isTxCompleted(const UpstreamMessage & message) {
		return message._ackRequested ? message.isAcknowledged() : true;
	};

	/*
	 * Downlink message arrival callback
	 * 
	 * Override if needed
	 */
	virtual void downlinkReceived(const DownstreamMessage&) {
	}

	/*
	 * MAC request arrival callback
	 * 
	 * Override if needed
	 */
	virtual void macCommandReceived(uint8_t *frame) {
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		decodeFOpts(LMIC.frame);
		#endif
	}

	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
	String _evNames[21] = {
		"zero",
		"EV_SCAN_TIMEOUT", "EV_BEACON_FOUND",
		"EV_BEACON_MISSED", "EV_BEACON_TRACKED", "EV_JOINING",
		"EV_JOINED", "EV_RFU1", "EV_JOIN_FAILED", "EV_REJOIN_FAILED",
		"EV_TXCOMPLETE", "EV_LOST_TSYNC", "EV_RESET",
		"EV_RXCOMPLETE", "EV_LINK_DEAD", "EV_LINK_ALIVE", "EV_SCAN_FOUND",
		"EV_TXSTART", "EV_TXCANCELED", "EV_RXSTART", "EV_JOIN_TXCOMPLETE"
	};

	const char* getMACCommandName(uint8_t cid) {
		switch (cid) {
		  case 0x02: return "LinkCheckAns";
		  case 0x03: return "LinkADRReq";
		  case 0x04: return "DutyCycleReq";
		  case 0x05: return "RXParamSetupReq";
		  case 0x06: return "DevStatusAns";
		  case 0x07: return "NewChannelReq";
		  case 0x08: return "RXTimingSetupReq";
		  case 0x09: return "TxParamSetupReq";
		  case 0x0A: return "DiChannelAns";
		  case 0x0D: return "DeviceTimeReq";
		  case 0x10: return "PingSlotInfoReq";
		  case 0x11: return "PingSlotChannelAns";
		  case 0x13: return "BeaconFreqAns";
		  default: return "Unknown / specific";
		}
	  }
	  
	void decodeFOpts(uint8_t* frame) {
		uint8_t fctrl = frame[5];
		uint8_t foptsLen = fctrl & 0x0F;
		uint8_t foptsStart = 8;
	  
		if (foptsLen == 0) {
		  LMIC_PRINTF_TO.println(F("No MAC commands in FOpts"));
		  return;
		}
	  
		LMIC_PRINTF_TO.print(F("FOpts ("));
		LMIC_PRINTF_TO.print(foptsLen);
		LMIC_PRINTF_TO.println(F(" bytes) :"));
	  
		uint8_t i = 0;
		while (i < foptsLen) {
		  uint8_t cid = frame[foptsStart + i];
		  LMIC_PRINTF_TO.print(" → CID 0x");
		  LMIC_PRINTF_TO.print(cid, HEX);
		  LMIC_PRINTF_TO.print(" : ");
		  LMIC_PRINTF_TO.println(getMACCommandName(cid));
		  i++;
	  
		  // Décodage des arguments selon CID
		  switch (cid) {
			case 0x02: // LinkCheckAns (margin + gateway count)
			  if (i + 1 < foptsLen) {
				uint8_t margin = frame[foptsStart + i];
				uint8_t gwcnt = frame[foptsStart + i + 1];
				LMIC_PRINTF_TO.print("   Margin: "); LMIC_PRINTF_TO.println(margin);
				LMIC_PRINTF_TO.print("   Gateways: "); LMIC_PRINTF_TO.println(gwcnt);
				i += 2;
			  }
			  break;
	  
			case 0x03: // LinkADRReq (4 bytes)
			  if (i + 3 < foptsLen) {
				LMIC_PRINTF_TO.print("   DataRate_TXPower: 0x"); LMIC_PRINTF_TO.println(frame[foptsStart + i], HEX);
				LMIC_PRINTF_TO.print("   ChMask: 0x");
				LMIC_PRINTF_TO.print(frame[foptsStart + i+1], HEX);
				LMIC_PRINTF_TO.print(" 0x");
				LMIC_PRINTF_TO.println(frame[foptsStart + i+2], HEX);
				LMIC_PRINTF_TO.print("   Redundancy: 0x"); LMIC_PRINTF_TO.println(frame[foptsStart + i+3], HEX);
				i += 4;
			  }
			  break;
	  
			case 0x04: // DutyCycleReq (1 byte)
			  if (i < foptsLen) {
				LMIC_PRINTF_TO.print("   MaxDutyCycle: "); LMIC_PRINTF_TO.println(frame[foptsStart + i], HEX);
				i += 1;
			  }
			  break;
	  
			case 0x06: // DevStatusAns (battery + margin)
			  if (i + 1 < foptsLen) {
				LMIC_PRINTF_TO.print("   Battery: "); LMIC_PRINTF_TO.println(frame[foptsStart + i]);
				LMIC_PRINTF_TO.print("   Margin: "); LMIC_PRINTF_TO.println((int8_t)frame[foptsStart + i + 1]); // signed
				i += 2;
			  }
			  break;
	  
			default:
			  LMIC_PRINTF_TO.println("   (undefined)");
			  break;
		  }
		}
	}
	  
	void printLmicChannels() {
		LMIC_PRINTF_TO.println("Active LMIC channels :");
		for (int i = 0; i < MAX_CHANNELS; i++) {
		  if (LMIC.channelFreq[i] != 0) {
			uint32_t freq = LMIC.channelFreq[i];
			LMIC_PRINTF_TO.print("Channel ");
			LMIC_PRINTF_TO.print(i);
			LMIC_PRINTF_TO.print(": ");
			LMIC_PRINTF_TO.print(freq / 1000000.0, 4);
			LMIC_PRINTF_TO.println(" MHz");
		  }
		}
	}

	#endif 
};

/*
 * LoRaWAN endnode singleton
 */
LMICWrapper * LMICWrapper::_node = nullptr;

/*
 * LMIC use event callback
 */
void onLMICEvent(void *pUserData, ev_t ev) { 
	LMICWrapper::_node->onUserEvent(ev); 
}

}
}

/*
 * LMIC callbacks
 * call delegation to LMICWrapper singleton
 */
void os_getArtEui (u1_t* buf) 	{ memcpy_P(buf, leuville::lora::LMICWrapper::_node->_env._appEUI, 8);}
void os_getDevEui (u1_t* buf) 	{ memcpy_P(buf, leuville::lora::LMICWrapper::_node->_env._devEUI, 8);}
void os_getDevKey (u1_t* buf) 	{ memcpy_P(buf, leuville::lora::LMICWrapper::_node->_env._appKEY, 16);}


