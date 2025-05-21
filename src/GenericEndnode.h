/*
 * Module: GenericEndnode
 *
 * Function: classes for Object-Oriented approach of LMIC
 *
 * Copyright and license: See accompanying LICENSE file.
 *
 * Author: Laurent Nel
 */

#pragma once

#include <LMICWrapper.h>

namespace leuville {
namespace lora {

template <typename T>
uint8_t * bufferOf(const T &);

template <typename T>
size_t * sizeOf(const T &);

template <typename T>
class GenericEndnode: public LMICWrapper {
public:
 
	using LMICWrapper::LMICWrapper;

	/*
	 * Build high-level T object from uint8_t buffer
	 */
	virtual T deserialize(uint8_t * buffer, uint8_t len) = 0;

	/*
	 * Fill uint8_t buffer with T object
	 * 
	 * Returns size of buffer
	 */
	virtual uint8_t serialize(conts T & object, uint8_t * buffer) = 0;

    /*
	 * Push a message built fron Json document to the front of the FIFO waiting queue
	 */
	virtual bool send(const T & doc, bool ack = false) {
		T doc = serialize()
        UpstreamMessage payload(bufferOf(doc), sizeOf(doc), ack);
		return LMICWrapper::send(payload);
	}

	/*
	 * Default send completion policy
     *
     * Override if needed
	 */
	virtual bool isTxCompleted(const JsonDocument & doc, bool ack) {
		return ack;
	};

	/*
	 * Downlink message arrival callback
     *
     * Override if needed
	 */
	virtual void downlinkReceived(const JsonDocument &) {
	}

protected:

	virtual bool isTxCompleted(const UpstreamMessage & message, bool ack) override final {
        JsonDocument doc;
        deserializeJson(doc, message._buf);
		return isTxCompleted(doc, ack);
	};

	virtual void downlinkReceived(const DownstreamMessage & message) override final {
        JsonDocument doc;
        deserializeJson(doc, message._buf);
        downlinkReceived(doc);
	}

};

}
}
