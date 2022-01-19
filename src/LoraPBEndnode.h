/*
 * Module: LoraPBEndnode
 *
 * Function: classes for Object-Oriented approach of LMIC
 *
 * Copyright and license: See accompanying LICENSE file.
 *
 * Author: Laurent Nel
 */

#pragma once

#include <LMICWrapper.h>

#include <pb_encode.h>
#include <pb_decode.h>

namespace leuville {
namespace lora {

/*
 * Encodes src object using nanopb into dest
 */
template<typename PBType>
size_t encode(const PBType & src, const pb_msgdesc_t * fields, Message & dest) {
	pb_ostream_t stream = pb_ostream_from_buffer(dest._buf, arrayCapacity(dest._buf));
	if (pb_encode(&stream, fields, &src)) {
		dest._len = stream.bytes_written;
	} else {
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
	virtual bool send(const U & payload, bool ackRequested = true, const pb_msgdesc_t* fields = UFIELDS) {
		UpstreamMessage upMessage;
		upMessage._ackRequested = ackRequested;
		if (encode(payload, fields, upMessage)) {
			return LMICWrapper::send(upMessage);
		} else {
			return false;
		}
	}

	/*
	 * Send completion policy
	 * message is decoded to its original format
	 */
	virtual bool isTxCompleted(const UpstreamMessage & message, bool ack) override final {
		U payload;
		decode(message, UFIELDS, payload);
		return isTxCompleted(payload, message._ackRequested, ack);
	};

	/*
	 * Default send completion policy
	 */
	virtual bool isTxCompleted(const U & message, bool ackRequest, bool ack) {
		return ack;
	};

	/*
	 * Downlink message arrival callback
	 */
	virtual void downlinkReceived(const DownstreamMessage& message) override final {
		D payload;
		if (decode(message, DFIELDS, payload)) {
			downlinkReceived(payload);
		}
	}

	/*
	 * Downlink message arrival callback
	 * 
	 * Message is decoded before
	 * Override if needed
	 */
	virtual void downlinkReceived(const D& message) {
	}
};

}
}
