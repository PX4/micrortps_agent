/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file iridiumsbd_status_Publisher.h
 * This header file contains the declaration of the publisher functions.
 *
 * This file was adapted from the fastcdrgen tool.
 */


#ifndef _iridiumsbd_status__PUBLISHER_H_
#define _iridiumsbd_status__PUBLISHER_H_

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/publisher/PublisherListener.h>

#include "iridiumsbd_statusPubSubTypes.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

using iridiumsbd_status_msg_t = iridiumsbd_status;
using iridiumsbd_status_msg_datatype = iridiumsbd_statusPubSubType;

class iridiumsbd_status_Publisher
{
public:
    iridiumsbd_status_Publisher();
    virtual ~iridiumsbd_status_Publisher();
    bool init();
    void run();
    void publish(iridiumsbd_status_msg_t* st);
private:
    Participant *mp_participant;
    Publisher *mp_publisher;

    class PubListener : public PublisherListener
    {
    public:
        PubListener() : n_matched(0){};
        ~PubListener(){};
        void onPublicationMatched(Publisher* pub, MatchingInfo& info);
        int n_matched;
    } m_listener;
    iridiumsbd_status_msg_datatype iridiumsbd_statusDataType;
};

#endif // _iridiumsbd_status__PUBLISHER_H_
