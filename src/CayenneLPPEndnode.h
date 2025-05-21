/*
 * Module: CayenneLPPEndnode
 * see https://gist.github.com/Safrone/800b44ce9632f68e7639d93ac0438076
 *
 * Function: classes for Object-Oriented approach of LMIC
 *
 * Copyright and license: See accompanying LICENSE file.
 *
 * Author: Laurent Nel
 */

#pragma once

#include <LMICWrapper.h>
#include <CayenneLPP.h>

namespace leuville {
namespace lora {

class CayenneLPPEndnode: public LMICWrapper {
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
	 * Serialize CayenneLPP -> String
	 */
	String stringFrom(CayenneLPP & lpp) {
		JsonDocument doc = jsonFrom(lpp.getBuffer(), lpp.getSize());
		return stringFrom(doc);
	}

	/*
	 * Build JsonDocument from (uint8_t *) buffer containing CayenneLPP data
	 */
	JsonDocument jsonFrom(uint8_t * buffer, uint8_t len) {
      	JsonDocument doc{};
		JsonObject root = doc.to<JsonObject>();
		CayenneLPP lpp(len);
		if (lpp.decodeTTN(buffer, len, root)) {
 			doc.shrinkToFit();
		} else {
			doc.clear();
		}
		return doc;
	}

    /*
	 * Send CayenneLPP message
	 */
	virtual bool send(const CayenneLPP & doc, bool ack = false) {
		CayenneLPP & lpp = const_cast<CayenneLPP&>(doc);
        UpstreamMessage payload(lpp.getBuffer(), lpp.getSize(), ack);
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

	/*
	 * Convenience method to rebuild JsonDocument from UpstreamMessage containing CayenneLPP data
	 */
	virtual bool isTxCompleted(const UpstreamMessage & message) override {
        JsonDocument doc = jsonFrom((uint8_t*)message._buf, message._len);
		return isTxCompleted(doc, message);
	};

	/*
	 * Convenience method to rebuild JsonDocument from DownstreamMessage containing CayenneLPP data
	 */
	virtual void downlinkReceived(const DownstreamMessage & message) override {
		JsonDocument doc = jsonFrom((uint8_t*)message._buf, message._len);
		downlinkReceived(doc, message);
	}

};

}
}
