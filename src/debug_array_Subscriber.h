/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*!
 * @file debug_array_Subscriber.h
 * This header file contains the declaration of the subscriber functions.
 *
 * This file was adapted from the fastcdrgen tool.
 */


#ifndef _debug_array__SUBSCRIBER_H_
#define _debug_array__SUBSCRIBER_H_

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include "debug_arrayPubSubTypes.h"

#include <atomic>
#include <condition_variable>
#include <queue>

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

using debug_array_msg_t = debug_array;
using debug_array_msg_datatype = debug_arrayPubSubType;

class debug_array_Subscriber
{
public:
	debug_array_Subscriber();
	virtual ~debug_array_Subscriber();
	bool init(uint8_t topic_ID, std::condition_variable *t_send_queue_cv, std::mutex *t_send_queue_mutex,
		  std::queue<uint8_t> *t_send_queue, const std::string &ns);
	void run();
	bool hasMsg();
	debug_array_msg_t getMsg();
	void unlockMsg();

private:
	Participant *mp_participant;
	Subscriber *mp_subscriber;

	class SubListener : public SubscriberListener
	{
	public:
		SubListener() : n_matched(0), n_msg(0), has_msg(false) {};
		~SubListener() {};
		void onSubscriptionMatched(Subscriber *sub, MatchingInfo &info);
		void onNewDataMessage(Subscriber *sub);
		SampleInfo_t m_info;
		int n_matched;
		int n_msg;
		debug_array_msg_t msg;
		std::atomic_bool has_msg;
		uint8_t topic_ID;
		std::condition_variable *t_send_queue_cv;
		std::mutex *t_send_queue_mutex;
		std::queue<uint8_t> *t_send_queue;
		std::condition_variable has_msg_cv;
		std::mutex has_msg_mutex;

	} m_listener;
	debug_array_msg_datatype debug_arrayDataType;
};

#endif // _debug_array__SUBSCRIBER_H_
