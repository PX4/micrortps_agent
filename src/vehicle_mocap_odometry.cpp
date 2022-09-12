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
 * @file vehicle_mocap_odometry.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "vehicle_mocap_odometry.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>










vehicle_mocap_odometry::vehicle_mocap_odometry()
{
    // m_timestamp_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@2ea227af
    m_timestamp_ = 0;
    // m_timestamp_sample_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@4386f16
    m_timestamp_sample_ = 0;
    // m_pose_frame_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@363ee3a2
    m_pose_frame_ = 0;
    // m_position com.eprosima.idl.parser.typecode.AliasTypeCode@4690b489
    memset(&m_position, 0, (3) * 4);
    // m_q com.eprosima.idl.parser.typecode.AliasTypeCode@79b06cab
    memset(&m_q, 0, (4) * 4);
    // m_velocity_frame_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@3eb7fc54
    m_velocity_frame_ = 0;
    // m_velocity com.eprosima.idl.parser.typecode.AliasTypeCode@4690b489
    memset(&m_velocity, 0, (3) * 4);
    // m_angular_velocity com.eprosima.idl.parser.typecode.AliasTypeCode@4690b489
    memset(&m_angular_velocity, 0, (3) * 4);
    // m_position_variance com.eprosima.idl.parser.typecode.AliasTypeCode@4690b489
    memset(&m_position_variance, 0, (3) * 4);
    // m_orientation_variance com.eprosima.idl.parser.typecode.AliasTypeCode@4690b489
    memset(&m_orientation_variance, 0, (3) * 4);
    // m_velocity_variance com.eprosima.idl.parser.typecode.AliasTypeCode@4690b489
    memset(&m_velocity_variance, 0, (3) * 4);
    // m_reset_counter_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@7f552bd3
    m_reset_counter_ = 0;
    // m_quality_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@3c22fc4c
    m_quality_ = 0;

}

vehicle_mocap_odometry::~vehicle_mocap_odometry()
{













}

vehicle_mocap_odometry::vehicle_mocap_odometry(const vehicle_mocap_odometry &x)
{
    m_timestamp_ = x.m_timestamp_;
    m_timestamp_sample_ = x.m_timestamp_sample_;
    m_pose_frame_ = x.m_pose_frame_;
    m_position = x.m_position;
    m_q = x.m_q;
    m_velocity_frame_ = x.m_velocity_frame_;
    m_velocity = x.m_velocity;
    m_angular_velocity = x.m_angular_velocity;
    m_position_variance = x.m_position_variance;
    m_orientation_variance = x.m_orientation_variance;
    m_velocity_variance = x.m_velocity_variance;
    m_reset_counter_ = x.m_reset_counter_;
    m_quality_ = x.m_quality_;
}

vehicle_mocap_odometry::vehicle_mocap_odometry(vehicle_mocap_odometry &&x)
{
    m_timestamp_ = x.m_timestamp_;
    m_timestamp_sample_ = x.m_timestamp_sample_;
    m_pose_frame_ = x.m_pose_frame_;
    m_position = std::move(x.m_position);
    m_q = std::move(x.m_q);
    m_velocity_frame_ = x.m_velocity_frame_;
    m_velocity = std::move(x.m_velocity);
    m_angular_velocity = std::move(x.m_angular_velocity);
    m_position_variance = std::move(x.m_position_variance);
    m_orientation_variance = std::move(x.m_orientation_variance);
    m_velocity_variance = std::move(x.m_velocity_variance);
    m_reset_counter_ = x.m_reset_counter_;
    m_quality_ = x.m_quality_;
}

vehicle_mocap_odometry& vehicle_mocap_odometry::operator=(const vehicle_mocap_odometry &x)
{

    m_timestamp_ = x.m_timestamp_;
    m_timestamp_sample_ = x.m_timestamp_sample_;
    m_pose_frame_ = x.m_pose_frame_;
    m_position = x.m_position;
    m_q = x.m_q;
    m_velocity_frame_ = x.m_velocity_frame_;
    m_velocity = x.m_velocity;
    m_angular_velocity = x.m_angular_velocity;
    m_position_variance = x.m_position_variance;
    m_orientation_variance = x.m_orientation_variance;
    m_velocity_variance = x.m_velocity_variance;
    m_reset_counter_ = x.m_reset_counter_;
    m_quality_ = x.m_quality_;

    return *this;
}

vehicle_mocap_odometry& vehicle_mocap_odometry::operator=(vehicle_mocap_odometry &&x)
{

    m_timestamp_ = x.m_timestamp_;
    m_timestamp_sample_ = x.m_timestamp_sample_;
    m_pose_frame_ = x.m_pose_frame_;
    m_position = std::move(x.m_position);
    m_q = std::move(x.m_q);
    m_velocity_frame_ = x.m_velocity_frame_;
    m_velocity = std::move(x.m_velocity);
    m_angular_velocity = std::move(x.m_angular_velocity);
    m_position_variance = std::move(x.m_position_variance);
    m_orientation_variance = std::move(x.m_orientation_variance);
    m_velocity_variance = std::move(x.m_velocity_variance);
    m_reset_counter_ = x.m_reset_counter_;
    m_quality_ = x.m_quality_;

    return *this;
}

size_t vehicle_mocap_odometry::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += ((4) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    return current_alignment - initial_alignment;
}

size_t vehicle_mocap_odometry::getCdrSerializedSize(const vehicle_mocap_odometry& data, size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    if ((3) > 0)
    {
        current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    }

    if ((4) > 0)
    {
        current_alignment += ((4) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    }

    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    if ((3) > 0)
    {
        current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    }

    if ((3) > 0)
    {
        current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    }

    if ((3) > 0)
    {
        current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    }

    if ((3) > 0)
    {
        current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    }

    if ((3) > 0)
    {
        current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    }

    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    return current_alignment - initial_alignment;
}

void vehicle_mocap_odometry::serialize(eprosima::fastcdr::Cdr &scdr) const
{

    scdr << m_timestamp_;
    scdr << m_timestamp_sample_;
    scdr << m_pose_frame_;
    scdr << m_position;

    scdr << m_q;

    scdr << m_velocity_frame_;
    scdr << m_velocity;

    scdr << m_angular_velocity;

    scdr << m_position_variance;

    scdr << m_orientation_variance;

    scdr << m_velocity_variance;

    scdr << m_reset_counter_;
    scdr << m_quality_;
}

void vehicle_mocap_odometry::deserialize(eprosima::fastcdr::Cdr &dcdr)
{

    dcdr >> m_timestamp_;
    dcdr >> m_timestamp_sample_;
    dcdr >> m_pose_frame_;
    dcdr >> m_position;

    dcdr >> m_q;

    dcdr >> m_velocity_frame_;
    dcdr >> m_velocity;

    dcdr >> m_angular_velocity;

    dcdr >> m_position_variance;

    dcdr >> m_orientation_variance;

    dcdr >> m_velocity_variance;

    dcdr >> m_reset_counter_;
    dcdr >> m_quality_;
}

/*!
 * @brief This function sets a value in member timestamp_
 * @param _timestamp_ New value for member timestamp_
 */
void vehicle_mocap_odometry::timestamp_(uint64_t _timestamp_)
{
m_timestamp_ = _timestamp_;
}

/*!
 * @brief This function returns the value of member timestamp_
 * @return Value of member timestamp_
 */
uint64_t vehicle_mocap_odometry::timestamp_() const
{
    return m_timestamp_;
}

/*!
 * @brief This function returns a reference to member timestamp_
 * @return Reference to member timestamp_
 */
uint64_t& vehicle_mocap_odometry::timestamp_()
{
    return m_timestamp_;
}

/*!
 * @brief This function sets a value in member timestamp_sample_
 * @param _timestamp_sample_ New value for member timestamp_sample_
 */
void vehicle_mocap_odometry::timestamp_sample_(uint64_t _timestamp_sample_)
{
m_timestamp_sample_ = _timestamp_sample_;
}

/*!
 * @brief This function returns the value of member timestamp_sample_
 * @return Value of member timestamp_sample_
 */
uint64_t vehicle_mocap_odometry::timestamp_sample_() const
{
    return m_timestamp_sample_;
}

/*!
 * @brief This function returns a reference to member timestamp_sample_
 * @return Reference to member timestamp_sample_
 */
uint64_t& vehicle_mocap_odometry::timestamp_sample_()
{
    return m_timestamp_sample_;
}

/*!
 * @brief This function sets a value in member pose_frame_
 * @param _pose_frame_ New value for member pose_frame_
 */
void vehicle_mocap_odometry::pose_frame_(uint8_t _pose_frame_)
{
m_pose_frame_ = _pose_frame_;
}

/*!
 * @brief This function returns the value of member pose_frame_
 * @return Value of member pose_frame_
 */
uint8_t vehicle_mocap_odometry::pose_frame_() const
{
    return m_pose_frame_;
}

/*!
 * @brief This function returns a reference to member pose_frame_
 * @return Reference to member pose_frame_
 */
uint8_t& vehicle_mocap_odometry::pose_frame_()
{
    return m_pose_frame_;
}

/*!
 * @brief This function copies the value in member position
 * @param _position New value to be copied in member position
 */
void vehicle_mocap_odometry::position(const vehicle_mocap_odometry__float_array_3 &_position)
{
m_position = _position;
}

/*!
 * @brief This function moves the value in member position
 * @param _position New value to be moved in member position
 */
void vehicle_mocap_odometry::position(vehicle_mocap_odometry__float_array_3 &&_position)
{
m_position = std::move(_position);
}

/*!
 * @brief This function returns a constant reference to member position
 * @return Constant reference to member position
 */
const vehicle_mocap_odometry__float_array_3& vehicle_mocap_odometry::position() const
{
    return m_position;
}

/*!
 * @brief This function returns a reference to member position
 * @return Reference to member position
 */
vehicle_mocap_odometry__float_array_3& vehicle_mocap_odometry::position()
{
    return m_position;
}
/*!
 * @brief This function copies the value in member q
 * @param _q New value to be copied in member q
 */
void vehicle_mocap_odometry::q(const vehicle_mocap_odometry__float_array_4 &_q)
{
m_q = _q;
}

/*!
 * @brief This function moves the value in member q
 * @param _q New value to be moved in member q
 */
void vehicle_mocap_odometry::q(vehicle_mocap_odometry__float_array_4 &&_q)
{
m_q = std::move(_q);
}

/*!
 * @brief This function returns a constant reference to member q
 * @return Constant reference to member q
 */
const vehicle_mocap_odometry__float_array_4& vehicle_mocap_odometry::q() const
{
    return m_q;
}

/*!
 * @brief This function returns a reference to member q
 * @return Reference to member q
 */
vehicle_mocap_odometry__float_array_4& vehicle_mocap_odometry::q()
{
    return m_q;
}
/*!
 * @brief This function sets a value in member velocity_frame_
 * @param _velocity_frame_ New value for member velocity_frame_
 */
void vehicle_mocap_odometry::velocity_frame_(uint8_t _velocity_frame_)
{
m_velocity_frame_ = _velocity_frame_;
}

/*!
 * @brief This function returns the value of member velocity_frame_
 * @return Value of member velocity_frame_
 */
uint8_t vehicle_mocap_odometry::velocity_frame_() const
{
    return m_velocity_frame_;
}

/*!
 * @brief This function returns a reference to member velocity_frame_
 * @return Reference to member velocity_frame_
 */
uint8_t& vehicle_mocap_odometry::velocity_frame_()
{
    return m_velocity_frame_;
}

/*!
 * @brief This function copies the value in member velocity
 * @param _velocity New value to be copied in member velocity
 */
void vehicle_mocap_odometry::velocity(const vehicle_mocap_odometry__float_array_3 &_velocity)
{
m_velocity = _velocity;
}

/*!
 * @brief This function moves the value in member velocity
 * @param _velocity New value to be moved in member velocity
 */
void vehicle_mocap_odometry::velocity(vehicle_mocap_odometry__float_array_3 &&_velocity)
{
m_velocity = std::move(_velocity);
}

/*!
 * @brief This function returns a constant reference to member velocity
 * @return Constant reference to member velocity
 */
const vehicle_mocap_odometry__float_array_3& vehicle_mocap_odometry::velocity() const
{
    return m_velocity;
}

/*!
 * @brief This function returns a reference to member velocity
 * @return Reference to member velocity
 */
vehicle_mocap_odometry__float_array_3& vehicle_mocap_odometry::velocity()
{
    return m_velocity;
}
/*!
 * @brief This function copies the value in member angular_velocity
 * @param _angular_velocity New value to be copied in member angular_velocity
 */
void vehicle_mocap_odometry::angular_velocity(const vehicle_mocap_odometry__float_array_3 &_angular_velocity)
{
m_angular_velocity = _angular_velocity;
}

/*!
 * @brief This function moves the value in member angular_velocity
 * @param _angular_velocity New value to be moved in member angular_velocity
 */
void vehicle_mocap_odometry::angular_velocity(vehicle_mocap_odometry__float_array_3 &&_angular_velocity)
{
m_angular_velocity = std::move(_angular_velocity);
}

/*!
 * @brief This function returns a constant reference to member angular_velocity
 * @return Constant reference to member angular_velocity
 */
const vehicle_mocap_odometry__float_array_3& vehicle_mocap_odometry::angular_velocity() const
{
    return m_angular_velocity;
}

/*!
 * @brief This function returns a reference to member angular_velocity
 * @return Reference to member angular_velocity
 */
vehicle_mocap_odometry__float_array_3& vehicle_mocap_odometry::angular_velocity()
{
    return m_angular_velocity;
}
/*!
 * @brief This function copies the value in member position_variance
 * @param _position_variance New value to be copied in member position_variance
 */
void vehicle_mocap_odometry::position_variance(const vehicle_mocap_odometry__float_array_3 &_position_variance)
{
m_position_variance = _position_variance;
}

/*!
 * @brief This function moves the value in member position_variance
 * @param _position_variance New value to be moved in member position_variance
 */
void vehicle_mocap_odometry::position_variance(vehicle_mocap_odometry__float_array_3 &&_position_variance)
{
m_position_variance = std::move(_position_variance);
}

/*!
 * @brief This function returns a constant reference to member position_variance
 * @return Constant reference to member position_variance
 */
const vehicle_mocap_odometry__float_array_3& vehicle_mocap_odometry::position_variance() const
{
    return m_position_variance;
}

/*!
 * @brief This function returns a reference to member position_variance
 * @return Reference to member position_variance
 */
vehicle_mocap_odometry__float_array_3& vehicle_mocap_odometry::position_variance()
{
    return m_position_variance;
}
/*!
 * @brief This function copies the value in member orientation_variance
 * @param _orientation_variance New value to be copied in member orientation_variance
 */
void vehicle_mocap_odometry::orientation_variance(const vehicle_mocap_odometry__float_array_3 &_orientation_variance)
{
m_orientation_variance = _orientation_variance;
}

/*!
 * @brief This function moves the value in member orientation_variance
 * @param _orientation_variance New value to be moved in member orientation_variance
 */
void vehicle_mocap_odometry::orientation_variance(vehicle_mocap_odometry__float_array_3 &&_orientation_variance)
{
m_orientation_variance = std::move(_orientation_variance);
}

/*!
 * @brief This function returns a constant reference to member orientation_variance
 * @return Constant reference to member orientation_variance
 */
const vehicle_mocap_odometry__float_array_3& vehicle_mocap_odometry::orientation_variance() const
{
    return m_orientation_variance;
}

/*!
 * @brief This function returns a reference to member orientation_variance
 * @return Reference to member orientation_variance
 */
vehicle_mocap_odometry__float_array_3& vehicle_mocap_odometry::orientation_variance()
{
    return m_orientation_variance;
}
/*!
 * @brief This function copies the value in member velocity_variance
 * @param _velocity_variance New value to be copied in member velocity_variance
 */
void vehicle_mocap_odometry::velocity_variance(const vehicle_mocap_odometry__float_array_3 &_velocity_variance)
{
m_velocity_variance = _velocity_variance;
}

/*!
 * @brief This function moves the value in member velocity_variance
 * @param _velocity_variance New value to be moved in member velocity_variance
 */
void vehicle_mocap_odometry::velocity_variance(vehicle_mocap_odometry__float_array_3 &&_velocity_variance)
{
m_velocity_variance = std::move(_velocity_variance);
}

/*!
 * @brief This function returns a constant reference to member velocity_variance
 * @return Constant reference to member velocity_variance
 */
const vehicle_mocap_odometry__float_array_3& vehicle_mocap_odometry::velocity_variance() const
{
    return m_velocity_variance;
}

/*!
 * @brief This function returns a reference to member velocity_variance
 * @return Reference to member velocity_variance
 */
vehicle_mocap_odometry__float_array_3& vehicle_mocap_odometry::velocity_variance()
{
    return m_velocity_variance;
}
/*!
 * @brief This function sets a value in member reset_counter_
 * @param _reset_counter_ New value for member reset_counter_
 */
void vehicle_mocap_odometry::reset_counter_(uint8_t _reset_counter_)
{
m_reset_counter_ = _reset_counter_;
}

/*!
 * @brief This function returns the value of member reset_counter_
 * @return Value of member reset_counter_
 */
uint8_t vehicle_mocap_odometry::reset_counter_() const
{
    return m_reset_counter_;
}

/*!
 * @brief This function returns a reference to member reset_counter_
 * @return Reference to member reset_counter_
 */
uint8_t& vehicle_mocap_odometry::reset_counter_()
{
    return m_reset_counter_;
}

/*!
 * @brief This function sets a value in member quality_
 * @param _quality_ New value for member quality_
 */
void vehicle_mocap_odometry::quality_(uint8_t _quality_)
{
m_quality_ = _quality_;
}

/*!
 * @brief This function returns the value of member quality_
 * @return Value of member quality_
 */
uint8_t vehicle_mocap_odometry::quality_() const
{
    return m_quality_;
}

/*!
 * @brief This function returns a reference to member quality_
 * @return Reference to member quality_
 */
uint8_t& vehicle_mocap_odometry::quality_()
{
    return m_quality_;
}


size_t vehicle_mocap_odometry::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    size_t current_align = current_alignment;
















    return current_align;
}

bool vehicle_mocap_odometry::isKeyDefined()
{
   return false;
}

void vehicle_mocap_odometry::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
    (void) scdr;
     
     
     
     
     
     
     
     
     
     
     
     
     
}
