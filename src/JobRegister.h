#pragma once

#include <Arduino.h>
#include <lmic.h>
#include <functor.h>
#include <fixed-map.h>

namespace lstl = leuville::simple_template_library;
using namespace lstl;

namespace leuville {
namespace lora {

template <typename T>
using Callback = MemberFunction<T,void>;

/*
 * Stores a function map (osjob_t, member function pointer)
 * 
 * Intended for use with LMICWrapper::setCallback()
 */
template <typename T, uint8_t SIZ = 10>
class JobRegister {
    
	using MemberFuncPtr = void(T::*)();	// callback type (member function pointer)

    private:

        osjob_t                                     _jobs[SIZ];
        mapArray<osjob_t*, Callback<T>, false, SIZ> _callbacks;

    public:

        void define(uint8_t pos, T * target, MemberFuncPtr ptrF) {
            _callbacks.put(&(_jobs[pos]), Callback<T>(target, ptrF));        
        }

        osjob_t & operator[](uint8_t pos) {
            return _jobs[pos];
        }

        Callback<T> & operator[](osjob_t * job) {
            return _callbacks[job];
        }

        /*
         * iterator to get all jobs stored into the register
         */
        class iterator {
            private:
                JobRegister<T,SIZ> &    _jobRegister;
                uint8_t                 _pos = 0;
            public:
                iterator(JobRegister<T,SIZ> & jobRegister, uint8_t pos) : _jobRegister(jobRegister), _pos(pos) {
                }
                bool operator!= (const iterator& other) const {
			        return _pos != other._pos;
			    }
		        osjob_t & operator* () const {
		    	    return _jobRegister._jobs[_pos];
		        }
		        const iterator& operator++ () {
		    	    ++_pos;
		    	    return *this;
		        }
        };
        
        iterator begin() {
			return iterator(*this, 0);
		}

		iterator end() {
			return iterator(*this, SIZ);
		}

};

}
}