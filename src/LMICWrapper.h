/*
 * Module: LMICWrapper
 *
 * Function: classes for Object-Oriented approach of LMIC
 *
 */

#pragma once

#include <ArduinoSTL.h>
#include <deque>

#include <lmic.h>
#include <lmic/oslmic.h>
#include <hal/hal.h>

#include <SPI.h>

#include <pb_encode.h>
#include <pb_decode.h>

using namespace std;

/*
 * LoRaWAN configuration: pinmaps, OTAA keys
 */
struct LoRaWANEnv {

	LoRaWANEnv(const lmic_pinmap & pinmap, initializer_list<u1_t> appEUI, initializer_list<u1_t> devEUI, initializer_list<u1_t> appKEY)
		: _pinmap(pinmap)
	{
		auto initTab = [] (u1_t * tab, initializer_list<u1_t> & l) {
			int i = 0;
			for (u1_t val: l) {
				tab[i++] = val;
			}
		};
		initTab(_appEUI, appEUI);
		initTab(_devEUI, devEUI);
		initTab(_appKEY, appKEY);
	}

	const lmic_pinmap _pinmap;

	// LoRaWAN OTAA keys
	u1_t _appEUI[8]		= { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u1_t _devEUI[8]		= { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u1_t _appKEY[16]	= {
							0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
							0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
																			};
};

/*
 * Message buffer = uint8_t array + size
 * Acts as a base class for UpstreamMessage and DownstreamMessage
 */
template <uint8_t MAXLEN = MAX_FRAME_LEN>
struct Message {
	uint8_t _buf[MAXLEN];
	uint8_t _len = 0;

	Message(uint8_t* buf, uint8_t len)
		: _len(len)
	{
		memset(_buf, 0, sizeof(_buf));
		memcpy(_buf, buf, _len);
	}
};

/*
 * UpStream message = message buffer + ack request
 */
template <uint8_t MAXLEN = MAX_FRAME_LEN>
struct UpstreamMessage : Message<MAXLEN> {
	boolean _ackRequested = false;

	UpstreamMessage(uint8_t* buf = nullptr, uint8_t len = 0, boolean ackRequested = false)
		: Message<MAXLEN>(buf, len), _ackRequested(ackRequested)
	{
	}
};

/*
 * Downstream message = message buffer
 */
template <uint8_t MAXLEN = MAX_FRAME_LEN>
struct DownstreamMessage : Message<MAXLEN> {
	using Message<MAXLEN>::Message;
};

/*
 * forward declarations
 */
void onEvent(ev_t ev);
void do_it(osjob_t* j);

using LMICMessage = Message<MAX_FRAME_LEN>;
using UpMessage = UpstreamMessage<MAX_FRAME_LEN>;
using DownMessage = DownstreamMessage<MAX_FRAME_LEN>;

/*
 * LMIC abstract base class
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

	LMICWrapper(const LoRaWANEnv& env)
		: _env(env)
	{
		LMICWrapper::_node = this;
	}

	virtual ~LMICWrapper() = default;

	/*
	 * Initializes LMIC
	 */
	virtual void begin() {
		os_init();
		initLMIC();
	}

	/*
	 * Add a job to the callback list managed by LMIC
	 */
	virtual void setCallback(osjob_t& job, long interval = 0) final {
		_jobCount += 1;
		if (interval) {
			os_setTimedCallback(&job, os_getTime() + interval, do_it);
		} else {
			os_setCallback(&job, do_it);
		}
	}

	/*
	 * Must be called from loop()
	 *
	 * Register a sendJob callback if needed (messages waiting to be sent)
	 */
	virtual void runLoopOnce() final {
		// if msg pending and LMIC doing nothing -> create a new LMIC job
		if (!_sendJobRequested && hasMessageToSend() && !isRadioBusy() && !isTxDataPending()) {
			_sendJobRequested = true;
			setCallback(_sendJob);
		}
		os_runloop_once();
	}

	/*
	 * Push a message to the front of the FIFO waiting queue
	 */
	virtual void send(const UpMessage & message) {
		_messages.push_front(message);
	}

	/*
	 * Returns true is there at least one message waiting to be sent
	 */
	virtual boolean hasMessageToSend() {
		return (_messages.size() > 0);
	}

	/*
	 * Returns true if LMIC radio is pending
	 */
	boolean isRadioBusy() {
		return (LMIC.opmode & OP_TXRXPEND);
	}

	/*
	 * Returns true if TX buffer contains data for sending
	 */
	boolean isTxDataPending() {
		return (LMIC.opmode & OP_TXDATA);
	}

	/*
	 * Return true if the device can be put in standby mode.
	 */
	virtual boolean isReadyForStandby() {
		return _joined && (_jobCount == 0) && !hasMessageToSend() && !isRadioBusy() && !isTxDataPending();
	}

protected:

	static LMICWrapper* _node;

	// LoRaWAN environment
	const LoRaWANEnv _env;

	// active osjob counter
	int _jobCount = 0;

	osjob_t _sendJob;
	boolean _sendJobRequested = false;

	// LoRaWAN JOIN done ?
	boolean _joined = false;

	// FIFO messages waiting to be sent
	deque<UpMessage> _messages;			

	/*
	 * Reset LMIC, and set LMIC parameters
	 */
	virtual void initLMIC() {
		LMIC_reset();
		LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100); // tweak to speed up the JOIN time
		LMIC_setAdrMode(1);
	}

	/*
	 * Called by LMIC when a given job should be performed
	 * This job has been previously registered with setCallback()
	 * This method should be overriden by subclass if other callbacks are used
	 * In this case do not forget to call LMICWrapper::performJob()
	 */
	virtual void performJob(osjob_t* job) {
		_jobCount -= 1;
		if (job == &_sendJob) {
			_sendJobRequested = false;
			send();
		}
	}

	/*
	 * Calls LMIC_setTxData2() if not busy
	 */
	virtual void send() {
		if (!isTxDataPending() && !isRadioBusy() && hasMessageToSend()) {
			UpMessage& msg = _messages.back();
			lmicSend(msg._buf, msg._len, msg._ackRequested);
		}
	}

	/*
	  * LMIC event callback
	  *
	  * lmic.h -> enum _ev_t { EV_SCAN_TIMEOUT=1, EV_BEACON_FOUND,
	  *        EV_BEACON_MISSED, EV_BEACON_TRACKED, EV_JOINING,
	  *        EV_JOINED, EV_RFU1, EV_JOIN_FAILED, EV_REJOIN_FAILED,
	  *        EV_TXCOMPLETE, EV_LOST_TSYNC, EV_RESET,
	  *        EV_RXCOMPLETE, EV_LINK_DEAD, EV_LINK_ALIVE, EV_SCAN_FOUND,
	  *        EV_TXSTART };
	  */
	virtual void onEvent(ev_t ev) {
		switch (ev) {
			case EV_JOINED:
				_joined = true;
				joined(true);
				break;
			case EV_JOIN_FAILED:
			case EV_REJOIN_FAILED:
			case EV_RESET:
			case EV_LINK_DEAD:
				_joined = false;
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
	virtual void joined(boolean) {}

	/*
	 * Called by onEvent() calback
	 *
	 *  removes sent message from the FIFO to avoid another transmission
	 */
	virtual void txComplete() {
		UpMessage& sent = _messages.back();
		if (isTxCompleted(sent, sent._ackRequested, (LMIC.txrxFlags & TXRX_NACK) == 0)) {
			_messages.pop_back(); // message is removed from FIFO
		}
		// RX1 or RX2 ?
		if (LMIC.dataLen > 0) {
			pb_byte_t buf[MAX_FRAME_LEN];
			for (uint8_t i = 0; i < LMIC.dataLen; i++) {
				buf[i] = (pb_byte_t)LMIC.frame[LMIC.dataBeg + i];
			}
			downlinkReceived(DownMessage(buf, LMIC.dataLen));
		}
	}

	/*
	 * Default send completion policy
	 */
	virtual boolean isTxCompleted(const UpMessage & message, boolean ackRequested, boolean ack) {
		return (ack == ackRequested);
	};

	virtual void downlinkReceived(const DownMessage&) {}

	/*
	 * Fills in LMIC buffer for sending
	 */
	void lmicSend(uint8_t* data, uint8_t len, boolean ackRequested) {
		LMIC_setTxData2(1, data, len, ackRequested);
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
 *
 * WARNING: message is encoded with delimiter (see Google API)
 */
template<typename PBType>
uint8_t encode(const PBType & src, const pb_field_t * fields, LMICMessage & dest) {
	pb_ostream_t stream = pb_ostream_from_buffer(dest._buf, sizeof(dest._buf));
	pb_encode_delimited(&stream, fields, &src);
	dest._len = stream.bytes_written;
	return dest._len;
}

/*
 * Builds dest object using nanopb from src raw message
 *
 * WARNING: message is encoded with delimiter (see Google API)
 */
template <typename PBType>
boolean decode(const LMICMessage & src, const pb_field_t* fields, PBType & dest) {
	pb_istream_t stream = pb_istream_from_buffer(src._buf, src._len);
	return pb_decode_delimited(&stream, fields, &dest);
}

/*
 * ENDNODE abstract base class with ProtocolBuffer (nanopb) mechanisms
 * U = uplink message nanopb type
 * D = downlink message nanopb type
 */
template <typename U, uint8_t USIZE, const pb_field_t * UFIELDS, typename D, uint8_t DSIZE, const pb_field_t* DFIELDS>
class ProtobufEndnode: public LMICWrapper {
public:

	ProtobufEndnode(const LoRaWANEnv &env)
		: LMICWrapper(env) {
	}

	/*
	 * Build an UpMessage and push it into front of _messages deque
	 * then this UpMessage is filled with encoded bytes from payload
	 *
	 * Back of this deque is sent after call to runLoopOnce()
	 */
	virtual void send(const U & payload, boolean ackRequested = true) {
		_messages.push_front(UpMessage());
		encode(payload, UFIELDS, _messages.front());
	}

protected:

	virtual void downlinkReceived(const DownMessage & message) override {
		D downstreamPayload;
		if (decode(message, DFIELDS, downstreamPayload)) {
			downlinkReceived(downstreamPayload);
		}
	}

	/*
	 * Override if needed
	 */
	virtual void downlinkReceived(const D& payload) {
		// do something if needed
	}

	/*
	 * Default send completion policy
	 */
	virtual boolean isTxCompleted(const UpMessage& message, boolean ackRequested, boolean ack) {
		U payload;
		decode(message, UFIELDS, payload);
		return isTxCompleted(payload, ackRequested, ack);
	};

	virtual boolean isTxCompleted(const U & message, boolean ackRequested, boolean ack) {
		return (ack == ackRequested);
	}
};
