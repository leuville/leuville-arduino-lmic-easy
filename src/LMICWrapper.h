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

void onEvent(ev_t ev);
void do_it(osjob_t* j);
/*
 * LMIC Base class
 * - contains singleton reference needed by LMIC callbacks (delegation)
 * - manages LoRaWAN OTAA keys
 */
class LMICWrapper {
public:
	friend void onEvent(ev_t ev);
	friend void do_it(osjob_t* j);
	friend void os_getArtEui (u1_t* buf);
	friend void os_getDevEui (u1_t* buf);
	friend void os_getDevKey (u1_t* buf);

	LMICWrapper(const LoRaWANEnv &env)
		: _env(env)
	{
		LMICWrapper::_node = this;
	}

	virtual ~LMICWrapper() = default;

	virtual void begin() {
		os_init();
		initLMIC();
	}

	virtual void initLMIC() {
		LMIC_reset();
		LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100); // tweak to speed up the JOIN time
	    LMIC_setAdrMode(1);
	}

	/*
	 * Add a job to the callback list managed by LMIC
	 */
	virtual void setCallback(osjob_t & job, long interval = 0) final {
		_jobCount += 1;
		if (interval) {
			os_setTimedCallback(&job, os_getTime()+interval, do_it);
		} else {
			os_setCallback(&job, do_it);
		}
	}

	/*
	 * Called by LMIC when a given job should be performed
	 * This job has been previously registered with setCallback()
	 * This method should be overriden by subclass
	 */
	virtual void performJob(osjob_t* job) {
		_jobCount -= 1;
	}

	boolean isRadioBusy() {
		return (LMIC.opmode & OP_TXRXPEND);
	}

	/*
	 * Returns true if TX buffer contains data for sending
	 */
	boolean isTxDataPending() {
		return (LMIC.opmode & OP_TXDATA);
	}

	static LMICWrapper * _node;

protected:

	// LoRaWAN environment
	const LoRaWANEnv _env;

	// active osjob counter
	int _jobCount = 0;

	/*
	 * LMIC event callback
	 */
	virtual void onEvent(ev_t ev) = 0;

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
 * Upstream Message
 * U = uplink message nanopb type
 * SIZ = buffer encoded size
 */
template <typename U, size_t SIZ>
struct UpstreamMessage {
	UpstreamMessage(const U &payload, boolean ackRequested, const pb_field_t * fields)
		: _ackRequested(ackRequested), _fields(fields) {
		encode(payload);
	}

	boolean _ackRequested = true;
	const pb_field_t *_fields;
	uint8_t _buf[SIZ];
	uint8_t _len = 0;

	bool operator!=(const UpstreamMessage<U,SIZ> & other) {
		Serial.println("!=");
		return !(*this == other);
	}

	bool operator==(const UpstreamMessage<U,SIZ> & other) {
		Serial.println("==");
		if (_len == other._len && _ackRequested == other._ackRequested && _fields == other._fields) {
			return memcmp(_buf, other._buf, _len) == 0;
		} else {
			return false;
		}
	}

	/*
	 * Encodes a message with ProtocolBuffer (nanopb)
	 * Returns the number of bytes produced
	 */
	uint8_t encode(const U &payload) {
		memset(_buf, 0, sizeof(_buf));
    	pb_ostream_t stream = pb_ostream_from_buffer(_buf, sizeof(_buf));
    	pb_encode_delimited(&stream, _fields, &payload);
		_len = stream.bytes_written;
		return _len;
	}
	/*
	 * Decodes a message encoded with ProtocolBuffer
	 */
	boolean decode(U & decoded) {
		pb_istream_t stream = pb_istream_from_buffer(_buf, _len);
		return pb_decode_delimited(&stream, _fields, &decoded);
	}

};

/*
 * ENDNODE abstract base class with ProtocolBuffer (nanopb) mechanisms
 * U = uplink message nanopb type
 * D = downlink message nanopb type
 */
template <typename U, uint8_t USIZE, typename D, uint8_t DSIZE>
class ProtobufEndnode: public LMICWrapper {
public:

	ProtobufEndnode(const LoRaWANEnv &env)
		: LMICWrapper(env) {
	}

	/*
	 * Return true if the board can be put in standby mode.
	 */
	virtual boolean isReadyForStandby() {
		return _joined && (_jobCount == 0) && !hasMessagesQueued() && !isRadioBusy() && !isTxDataPending();
	}

	/*
	 * Build an UpstreamMessage and push into front of _messages deque
	 * Back of this deque is sent after call to runLoopOnce()
	 */
	virtual void send(const U & payload, boolean ackRequested = true) {
		_messages.push_front(
				UpstreamMessage<U,USIZE>(payload, ackRequested, getUplinkMessagePBFields())
		);
	}

	/*
	 * Must be called from loop()
	 */
	virtual void runLoopOnce() final {
		// if msg pending and LMIC sleeping -> create a new LMIC job
		if (!_sendJobRequested && hasMessagesQueued() && !isRadioBusy() && !isTxDataPending()) {
			_sendJobRequested = true;
			setCallback(_sendJob);
		}
		os_runloop_once();
	}

	virtual void performJob(osjob_t* job) override {
		LMICWrapper::performJob(job);
		if (job == &_sendJob) {
			_sendJobRequested = false;
			send();
		}
	}

	boolean hasMessagesQueued() {
		return (_messages.size() > 0);
	}

protected:

	osjob_t _sendJob;
	boolean _sendJobRequested = false;
	boolean _joined = false;
	deque<UpstreamMessage<U,USIZE>> _messages;			// FIFO messages waiting to be sent

	/*
	 * Decodes a message encoded with ProtocolBuffer
	 * WARNING: message must be encoded with writeDelimitedTo() (see Google API)
	 *
	 * Return true if successful
	 */
	boolean decodeDownstream(D * payload, uint8_t * buf, uint8_t len) {
		pb_istream_t stream = pb_istream_from_buffer(buf, len);
		return pb_decode_delimited(&stream, getDownlinkMessagePBFields(), payload);
	}

	/*
	 * LMIC event callback
	 */
	virtual void onEvent(ev_t ev) final {
		Serial.print("onEvent ");Serial.println(ev);
		switch (ev) {
			case EV_SCAN_TIMEOUT:
				scanTimeout();
				break;
			case EV_BEACON_FOUND:
				beaconFound();
				break;
			case EV_BEACON_MISSED:
				beaconMissed();
				break;
			case EV_BEACON_TRACKED:
				beaconTracked();
				break;
			case EV_JOINING:
				joining();
				break;
			case EV_JOINED:
				_joined = true;
				joined();
				break;
			case EV_RFU1:
				rfu1();
				break;
			case EV_JOIN_FAILED:
				_joined = false;
				joinFailed();
				break;
			case EV_REJOIN_FAILED:
				_joined = false;
				rejoinFailed();
				break;
			case EV_TXCOMPLETE:
				txComplete();
				break;
			case EV_LOST_TSYNC:
				lostTsync();
				break;
			case EV_RESET:
				_joined = false;
				reset();
				break;
			case EV_RXCOMPLETE:
				rxComplete();
				break;
			case EV_LINK_DEAD:
				linkDead();
				break;
			case EV_LINK_ALIVE:
				linkAlive();
				break;
			default:
				unknown();
				break;
		}
	}

	/*
	 * Calls LMIC_setTxData2() if not busy
	 */
	virtual void send() final {
		if (!isTxDataPending() && !isRadioBusy() && hasMessagesQueued()) {
			Serial.println("LMIC_setTxData2");
			UpstreamMessage<U,USIZE> & msg = _messages.back();
			LMIC_setTxData2(1, msg._buf, msg._len, msg._ackRequested);
		}
	}

	virtual void txComplete() final {
		if (hasMessagesQueued()) {
			UpstreamMessage<U,USIZE> & sent = _messages.back();
			U original;
			sent.decode(original);
			if (sent._ackRequested) {
				if (LMIC.txrxFlags & TXRX_ACK) {
					txComplete(original, sent._ackRequested , true);
				} else if (LMIC.txrxFlags & TXRX_NACK) {
					txComplete(original, sent._ackRequested, false);
				}
			} else {
				txComplete(original, sent._ackRequested, true);
			}
			// message is removed from FIFO
			_messages.pop_back();
		}
		// RX1 or RX2 ?
		if (LMIC.dataLen > 0) {
			// message decoded with ProtocolBuffer (nanopb)
			pb_byte_t buf[MAX_FRAME_LEN];
			for (uint8_t i = 0; i < LMIC.dataLen; i++) {
				buf[i] = (pb_byte_t)LMIC.frame[LMIC.dataBeg + i];
			}
			D downstreamPayload;
			if (decodeDownstream(&downstreamPayload, buf, LMIC.dataLen)) {
				downlinkReceived(downstreamPayload);
			}
		}
	}

	virtual void scanTimeout() {};
	virtual void beaconFound() {};
	virtual void beaconMissed() {};
	virtual void beaconTracked() {};
	virtual void joining() {};
	virtual void joined() {};
	virtual void rfu1() {};
	virtual void joinFailed() {};
	virtual void rejoinFailed() {};
	virtual void txComplete(const U &payload, boolean ackRequested, boolean received) {};
	virtual void lostTsync() {};
	virtual void reset() {};
	virtual void rxComplete() {};
	virtual void linkDead() {};
	virtual void linkAlive() {};
	virtual void unknown() {};

	virtual const pb_field_t * getUplinkMessagePBFields() = 0;
	virtual const pb_field_t * getDownlinkMessagePBFields() = 0;

	virtual void downlinkReceived(const D &payload) = 0;

};

