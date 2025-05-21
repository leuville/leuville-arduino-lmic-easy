/*
 * Module: JsonEndnode
 *
 * Function: classes for Object-Oriented approach of LMIC
 *
 * Copyright and license: See accompanying LICENSE file.
 *
 * Author: Laurent Nel
 */

#pragma once

#include <LMICWrapper.h>
#include <ArduinoJson.h>

namespace leuville {
namespace lora {

class JsonEndnode: public LMICWrapper {
public:

	using LMICWrapper::LMICWrapper;

    /*
	 * Serialize Json -> String
	 */
	String stringFrom(const JsonDocument & doc) {
		String message;
		serializeJson(doc, message);
		return message;
	}

    /*
	 * Push a message built fron Json document to the front of the FIFO waiting queue
	 */
	virtual bool send(const JsonDocument & doc, bool ack = false) {
        String msg;
        serializeJson(doc, msg);
        const char * content = msg.c_str();
        UpstreamMessage payload((uint8_t*)content, strlen(content)+1, ack);
		return LMICWrapper::send(payload);
	}

	/*
	 * Default send completion policy
     *
     * Override if needed
	 */
	virtual bool isTxCompleted(const JsonDocument & doc, const UpstreamMessage & rawMessage) {
		return LMICWrapper::isTxCompleted(rawMessage);
	};

	/*
	 * Downlink message arrival callback
     *
     * Override if needed
	 */
	virtual void downlinkReceived(const JsonDocument & message, const DownstreamMessage & rawMessage) {
	}

protected:

	virtual bool isTxCompleted(const UpstreamMessage & message) override final {
        JsonDocument doc;
        deserializeJson(doc, message._buf);
		return isTxCompleted(doc, message);
	};

	virtual void downlinkReceived(const DownstreamMessage & message) override final {
        JsonDocument doc;
        deserializeJson(doc, message._buf);
        downlinkReceived(doc, message);
	}

};

}
}
