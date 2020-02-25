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

#include <lmic.h>
#include <hal/hal.h>
#include <lmic/oslmic.h>

#include <pb_encode.h>
#include <pb_decode.h>

#include <misc-util.h>

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
 * 		endnode.begin(id[Config::TTN]); // or Config::OPE1 etc ...
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
	 * ie 10FF become FF10
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
void do_it(osjob_t* j);

/*
 * LMIC base class
 * - contains singleton reference needed by LMIC callbacks (delegation)
 * - manages LoRaWAN OTAA keys
 */
class LMICWrapper {
public:
	friend void onEvent(ev_t ev);
	friend void do_it(osjob_t* j);
	friend void os_getArtEui(u1_t* buf);
	friend void os_getDevEui(u1_t* buf);
	friend void os_getDevKey(u1_t* buf);

	LMICWrapper(const lmic_pinmap *pinmap): _pinmap(pinmap)
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
		LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100); // tweak to speed up the JOIN time
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
		os_setTimedCallback(job, os_getTime() + ms2osticks(interval), do_it);
	}

	virtual void setCallback(osjob_t& job, unsigned long interval = 0) final {
		setCallback(&job, interval);
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
	 */
	virtual void send(const UpstreamMessage & message) {
		_messages.push_front(message);
	}

	/*
	 * Returns true is there at least one message waiting to be sent
	 */
	virtual bool hasMessageToSend() {
		return (_messages.size() > 0);
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

protected:

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
	deque<UpstreamMessage, true, 100> _messages;			

	/*
	 * Set ADR
	 */
	virtual void initLMIC(u4_t network, bool adr = true) {
		LMIC_setAdrMode(adr ? 1 : 0);
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
	 * Calls LMIC_setTxData2() if message to send
	 */
	virtual void lmicSend() {
		if (UpstreamMessage* msg = _messages.backPtr(); msg != nullptr) {
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
		if (UpstreamMessage *sent = _messages.backPtr(); sent != nullptr) {
			if (isTxCompleted(*sent, (LMIC.txrxFlags & TXRX_NACK) == 0)) {
				_messages.pop_back(); // message is removed from FIFO
			}
		}
		// RX1 or RX2 ?
		if (LMIC.dataLen > 0) {
			pb_byte_t buf[MAX_FRAME_LEN];
			for (uint8_t i = 0; i < LMIC.dataLen; i++) {
				buf[i] = (pb_byte_t)LMIC.frame[LMIC.dataBeg + i];
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

/*
 * LMIC callbacks
 * call delegation to LMICWrapper singleton
 */
void os_getArtEui (u1_t* buf) 	{ memcpy_P(buf, LMICWrapper::_node->_env._appEUI, 8);}
void os_getDevEui (u1_t* buf) 	{ memcpy_P(buf, LMICWrapper::_node->_env._devEUI, 8);}
void os_getDevKey (u1_t* buf) 	{ memcpy_P(buf, LMICWrapper::_node->_env._appKEY, 16);}
void onEvent(ev_t ev) 			{ LMICWrapper::_node->onEvent(ev); }
void do_it(osjob_t* j) 			{ LMICWrapper::_node->performJob(j); }

/*
 * Encodes src object using nanopb into dest
 */
template<typename PBType>
size_t encode(const PBType & src, const pb_msgdesc_t * fields, Message & dest) {
	pb_ostream_t stream = pb_ostream_from_buffer(dest._buf, arrayCapacity(dest._buf));
	if (pb_encode(&stream, fields, &src)) {
		dest._len = stream.bytes_written;
	}
	else {
		dest._len = 0;
	}
	return dest._len;
}

/*
 * Builds dest object using nanopb from src raw message
 */
template <typename PBType>
bool decode(const Message& src, const pb_msgdesc_t* fields, PBType & dest) {
	pb_istream_t stream = pb_istream_from_buffer(src._buf, src._len);
	return pb_decode(&stream, fields, &dest);
}

/*
 * ENDNODE abstract base class with ProtocolBuffer (nanopb) mechanisms
 * U = uplink message nanopb type
 * D = downlink message nanopb type
 */
template <typename U, const pb_msgdesc_t* UFIELDS, typename D, const pb_msgdesc_t* DFIELDS>
class ProtobufEndnode: public LMICWrapper {
public:

	using LMICWrapper::LMICWrapper;
	
	/*
	 * Build an UpstreamMessage filled with encoded bytes from payload
	 * This message is stored in the double-ended queue managed by LMICWrapper.
	 * Back of this deque is sent after call to runLoopOnce()
	 *
	 * fields parameter provides the way to send partial message
	 */
	virtual void send(const U & payload, bool ackRequested = true, const pb_msgdesc_t* fields = UFIELDS) {
		UpstreamMessage upMessage;
		upMessage._ackRequested = ackRequested;
		if (encode(payload, fields, upMessage)) {
			LMICWrapper::send(upMessage);
		}
	}

};
