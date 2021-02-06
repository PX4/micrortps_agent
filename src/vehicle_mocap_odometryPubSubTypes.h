// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file vehicle_mocap_odometryPubSubTypes.h
 * This header file contains the declaration of the serialization functions.
 *
 * This file was generated by the tool fastcdrgen.
 */


#ifndef _VEHICLE_MOCAP_ODOMETRY_PUBSUBTYPES_H_
#define _VEHICLE_MOCAP_ODOMETRY_PUBSUBTYPES_H_

#include <fastrtps/config.h>
#include <fastrtps/TopicDataType.h>

#include "vehicle_mocap_odometry.h"

#if !defined(GEN_API_VER) || (GEN_API_VER != 1)
#error Generated vehicle_mocap_odometry is not compatible with current installed Fast-RTPS. Please, regenerate it with fastrtpsgen.
#endif

















typedef std::array<float, 21> vehicle_mocap_odometry__float_array_21;
typedef std::array<float, 4> vehicle_mocap_odometry__float_array_4;
/*!
 * @brief This class represents the TopicDataType of the type vehicle_mocap_odometry defined by the user in the IDL file.
 * @ingroup VEHICLE_MOCAP_ODOMETRY
 */
class vehicle_mocap_odometryPubSubType : public eprosima::fastrtps::TopicDataType {
public:
    typedef vehicle_mocap_odometry type;

    eProsima_user_DllExport vehicle_mocap_odometryPubSubType();

    eProsima_user_DllExport virtual ~vehicle_mocap_odometryPubSubType();
    eProsima_user_DllExport virtual bool serialize(void *data, eprosima::fastrtps::rtps::SerializedPayload_t *payload) override;
    eProsima_user_DllExport virtual bool deserialize(eprosima::fastrtps::rtps::SerializedPayload_t *payload, void *data) override;
    eProsima_user_DllExport virtual std::function<uint32_t()> getSerializedSizeProvider(void* data) override;
    eProsima_user_DllExport virtual bool getKey(void *data, eprosima::fastrtps::rtps::InstanceHandle_t *ihandle,
        bool force_md5 = false) override;
    eProsima_user_DllExport virtual void* createData() override;
    eProsima_user_DllExport virtual void deleteData(void * data) override;
    MD5 m_md5;
    unsigned char* m_keyBuffer;
};

#endif // _VEHICLE_MOCAP_ODOMETRY_PUBSUBTYPES_H_