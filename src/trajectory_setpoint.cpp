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
 * @file trajectory_setpoint.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "trajectory_setpoint.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>


trajectory_setpoint::trajectory_setpoint()
{
    // m_timestamp_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@9597028
    m_timestamp_ = 0;
    // m_position com.eprosima.idl.parser.typecode.AliasTypeCode@6069db50
    memset(&m_position, 0, (3) * 4);
    // m_velocity com.eprosima.idl.parser.typecode.AliasTypeCode@6069db50
    memset(&m_velocity, 0, (3) * 4);
    // m_acceleration com.eprosima.idl.parser.typecode.AliasTypeCode@6069db50
    memset(&m_acceleration, 0, (3) * 4);
    // m_jerk com.eprosima.idl.parser.typecode.AliasTypeCode@6069db50
    memset(&m_jerk, 0, (3) * 4);
    // m_yaw_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@4efbca5a
    m_yaw_ = 0.0;
    // m_yawspeed_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@1b7cc17c
    m_yawspeed_ = 0.0;

}

trajectory_setpoint::~trajectory_setpoint()
{







}

trajectory_setpoint::trajectory_setpoint(const trajectory_setpoint &x)
{
    m_timestamp_ = x.m_timestamp_;
    m_position = x.m_position;
    m_velocity = x.m_velocity;
    m_acceleration = x.m_acceleration;
    m_jerk = x.m_jerk;
    m_yaw_ = x.m_yaw_;
    m_yawspeed_ = x.m_yawspeed_;
}

trajectory_setpoint::trajectory_setpoint(trajectory_setpoint &&x)
{
    m_timestamp_ = x.m_timestamp_;
    m_position = std::move(x.m_position);
    m_velocity = std::move(x.m_velocity);
    m_acceleration = std::move(x.m_acceleration);
    m_jerk = std::move(x.m_jerk);
    m_yaw_ = x.m_yaw_;
    m_yawspeed_ = x.m_yawspeed_;
}

trajectory_setpoint& trajectory_setpoint::operator=(const trajectory_setpoint &x)
{

    m_timestamp_ = x.m_timestamp_;
    m_position = x.m_position;
    m_velocity = x.m_velocity;
    m_acceleration = x.m_acceleration;
    m_jerk = x.m_jerk;
    m_yaw_ = x.m_yaw_;
    m_yawspeed_ = x.m_yawspeed_;

    return *this;
}

trajectory_setpoint& trajectory_setpoint::operator=(trajectory_setpoint &&x)
{

    m_timestamp_ = x.m_timestamp_;
    m_position = std::move(x.m_position);
    m_velocity = std::move(x.m_velocity);
    m_acceleration = std::move(x.m_acceleration);
    m_jerk = std::move(x.m_jerk);
    m_yaw_ = x.m_yaw_;
    m_yawspeed_ = x.m_yawspeed_;

    return *this;
}

size_t trajectory_setpoint::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

size_t trajectory_setpoint::getCdrSerializedSize(const trajectory_setpoint& data, size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


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

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

void trajectory_setpoint::serialize(eprosima::fastcdr::Cdr &scdr) const
{

    scdr << m_timestamp_;
    scdr << m_position;

    scdr << m_velocity;

    scdr << m_acceleration;

    scdr << m_jerk;

    scdr << m_yaw_;
    scdr << m_yawspeed_;
}

void trajectory_setpoint::deserialize(eprosima::fastcdr::Cdr &dcdr)
{

    dcdr >> m_timestamp_;
    dcdr >> m_position;

    dcdr >> m_velocity;

    dcdr >> m_acceleration;

    dcdr >> m_jerk;

    dcdr >> m_yaw_;
    dcdr >> m_yawspeed_;
}

/*!
 * @brief This function sets a value in member timestamp_
 * @param _timestamp_ New value for member timestamp_
 */
void trajectory_setpoint::timestamp_(uint64_t _timestamp_)
{
m_timestamp_ = _timestamp_;
}

/*!
 * @brief This function returns the value of member timestamp_
 * @return Value of member timestamp_
 */
uint64_t trajectory_setpoint::timestamp_() const
{
    return m_timestamp_;
}

/*!
 * @brief This function returns a reference to member timestamp_
 * @return Reference to member timestamp_
 */
uint64_t& trajectory_setpoint::timestamp_()
{
    return m_timestamp_;
}

/*!
 * @brief This function copies the value in member position
 * @param _position New value to be copied in member position
 */
void trajectory_setpoint::position(const trajectory_setpoint__float_array_3 &_position)
{
m_position = _position;
}

/*!
 * @brief This function moves the value in member position
 * @param _position New value to be moved in member position
 */
void trajectory_setpoint::position(trajectory_setpoint__float_array_3 &&_position)
{
m_position = std::move(_position);
}

/*!
 * @brief This function returns a constant reference to member position
 * @return Constant reference to member position
 */
const trajectory_setpoint__float_array_3& trajectory_setpoint::position() const
{
    return m_position;
}

/*!
 * @brief This function returns a reference to member position
 * @return Reference to member position
 */
trajectory_setpoint__float_array_3& trajectory_setpoint::position()
{
    return m_position;
}
/*!
 * @brief This function copies the value in member velocity
 * @param _velocity New value to be copied in member velocity
 */
void trajectory_setpoint::velocity(const trajectory_setpoint__float_array_3 &_velocity)
{
m_velocity = _velocity;
}

/*!
 * @brief This function moves the value in member velocity
 * @param _velocity New value to be moved in member velocity
 */
void trajectory_setpoint::velocity(trajectory_setpoint__float_array_3 &&_velocity)
{
m_velocity = std::move(_velocity);
}

/*!
 * @brief This function returns a constant reference to member velocity
 * @return Constant reference to member velocity
 */
const trajectory_setpoint__float_array_3& trajectory_setpoint::velocity() const
{
    return m_velocity;
}

/*!
 * @brief This function returns a reference to member velocity
 * @return Reference to member velocity
 */
trajectory_setpoint__float_array_3& trajectory_setpoint::velocity()
{
    return m_velocity;
}
/*!
 * @brief This function copies the value in member acceleration
 * @param _acceleration New value to be copied in member acceleration
 */
void trajectory_setpoint::acceleration(const trajectory_setpoint__float_array_3 &_acceleration)
{
m_acceleration = _acceleration;
}

/*!
 * @brief This function moves the value in member acceleration
 * @param _acceleration New value to be moved in member acceleration
 */
void trajectory_setpoint::acceleration(trajectory_setpoint__float_array_3 &&_acceleration)
{
m_acceleration = std::move(_acceleration);
}

/*!
 * @brief This function returns a constant reference to member acceleration
 * @return Constant reference to member acceleration
 */
const trajectory_setpoint__float_array_3& trajectory_setpoint::acceleration() const
{
    return m_acceleration;
}

/*!
 * @brief This function returns a reference to member acceleration
 * @return Reference to member acceleration
 */
trajectory_setpoint__float_array_3& trajectory_setpoint::acceleration()
{
    return m_acceleration;
}
/*!
 * @brief This function copies the value in member jerk
 * @param _jerk New value to be copied in member jerk
 */
void trajectory_setpoint::jerk(const trajectory_setpoint__float_array_3 &_jerk)
{
m_jerk = _jerk;
}

/*!
 * @brief This function moves the value in member jerk
 * @param _jerk New value to be moved in member jerk
 */
void trajectory_setpoint::jerk(trajectory_setpoint__float_array_3 &&_jerk)
{
m_jerk = std::move(_jerk);
}

/*!
 * @brief This function returns a constant reference to member jerk
 * @return Constant reference to member jerk
 */
const trajectory_setpoint__float_array_3& trajectory_setpoint::jerk() const
{
    return m_jerk;
}

/*!
 * @brief This function returns a reference to member jerk
 * @return Reference to member jerk
 */
trajectory_setpoint__float_array_3& trajectory_setpoint::jerk()
{
    return m_jerk;
}
/*!
 * @brief This function sets a value in member yaw_
 * @param _yaw_ New value for member yaw_
 */
void trajectory_setpoint::yaw_(float _yaw_)
{
m_yaw_ = _yaw_;
}

/*!
 * @brief This function returns the value of member yaw_
 * @return Value of member yaw_
 */
float trajectory_setpoint::yaw_() const
{
    return m_yaw_;
}

/*!
 * @brief This function returns a reference to member yaw_
 * @return Reference to member yaw_
 */
float& trajectory_setpoint::yaw_()
{
    return m_yaw_;
}

/*!
 * @brief This function sets a value in member yawspeed_
 * @param _yawspeed_ New value for member yawspeed_
 */
void trajectory_setpoint::yawspeed_(float _yawspeed_)
{
m_yawspeed_ = _yawspeed_;
}

/*!
 * @brief This function returns the value of member yawspeed_
 * @return Value of member yawspeed_
 */
float trajectory_setpoint::yawspeed_() const
{
    return m_yawspeed_;
}

/*!
 * @brief This function returns a reference to member yawspeed_
 * @return Reference to member yawspeed_
 */
float& trajectory_setpoint::yawspeed_()
{
    return m_yawspeed_;
}


size_t trajectory_setpoint::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    size_t current_align = current_alignment;










    return current_align;
}

bool trajectory_setpoint::isKeyDefined()
{
   return false;
}

void trajectory_setpoint::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
    (void) scdr;
     
     
     
     
     
     
     
}
