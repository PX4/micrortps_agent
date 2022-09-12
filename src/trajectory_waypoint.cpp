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
 * @file trajectory_waypoint.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "trajectory_waypoint.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>


trajectory_waypoint::trajectory_waypoint()
{
    // m_timestamp_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@11dc3715
    m_timestamp_ = 0;
    // m_position com.eprosima.idl.parser.typecode.AliasTypeCode@69930714
    memset(&m_position, 0, (3) * 4);
    // m_velocity com.eprosima.idl.parser.typecode.AliasTypeCode@69930714
    memset(&m_velocity, 0, (3) * 4);
    // m_acceleration com.eprosima.idl.parser.typecode.AliasTypeCode@69930714
    memset(&m_acceleration, 0, (3) * 4);
    // m_yaw_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@7a52f2a2
    m_yaw_ = 0.0;
    // m_yaw_speed_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@78047b92
    m_yaw_speed_ = 0.0;
    // m_point_valid_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@8909f18
    m_point_valid_ = false;
    // m_type_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@79ca92b9
    m_type_ = 0;

}

trajectory_waypoint::~trajectory_waypoint()
{








}

trajectory_waypoint::trajectory_waypoint(const trajectory_waypoint &x)
{
    m_timestamp_ = x.m_timestamp_;
    m_position = x.m_position;
    m_velocity = x.m_velocity;
    m_acceleration = x.m_acceleration;
    m_yaw_ = x.m_yaw_;
    m_yaw_speed_ = x.m_yaw_speed_;
    m_point_valid_ = x.m_point_valid_;
    m_type_ = x.m_type_;
}

trajectory_waypoint::trajectory_waypoint(trajectory_waypoint &&x)
{
    m_timestamp_ = x.m_timestamp_;
    m_position = std::move(x.m_position);
    m_velocity = std::move(x.m_velocity);
    m_acceleration = std::move(x.m_acceleration);
    m_yaw_ = x.m_yaw_;
    m_yaw_speed_ = x.m_yaw_speed_;
    m_point_valid_ = x.m_point_valid_;
    m_type_ = x.m_type_;
}

trajectory_waypoint& trajectory_waypoint::operator=(const trajectory_waypoint &x)
{

    m_timestamp_ = x.m_timestamp_;
    m_position = x.m_position;
    m_velocity = x.m_velocity;
    m_acceleration = x.m_acceleration;
    m_yaw_ = x.m_yaw_;
    m_yaw_speed_ = x.m_yaw_speed_;
    m_point_valid_ = x.m_point_valid_;
    m_type_ = x.m_type_;

    return *this;
}

trajectory_waypoint& trajectory_waypoint::operator=(trajectory_waypoint &&x)
{

    m_timestamp_ = x.m_timestamp_;
    m_position = std::move(x.m_position);
    m_velocity = std::move(x.m_velocity);
    m_acceleration = std::move(x.m_acceleration);
    m_yaw_ = x.m_yaw_;
    m_yaw_speed_ = x.m_yaw_speed_;
    m_point_valid_ = x.m_point_valid_;
    m_type_ = x.m_type_;

    return *this;
}

size_t trajectory_waypoint::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += ((3) * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    return current_alignment - initial_alignment;
}

size_t trajectory_waypoint::getCdrSerializedSize(const trajectory_waypoint& data, size_t current_alignment)
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

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    return current_alignment - initial_alignment;
}

void trajectory_waypoint::serialize(eprosima::fastcdr::Cdr &scdr) const
{

    scdr << m_timestamp_;
    scdr << m_position;

    scdr << m_velocity;

    scdr << m_acceleration;

    scdr << m_yaw_;
    scdr << m_yaw_speed_;
    scdr << m_point_valid_;
    scdr << m_type_;
}

void trajectory_waypoint::deserialize(eprosima::fastcdr::Cdr &dcdr)
{

    dcdr >> m_timestamp_;
    dcdr >> m_position;

    dcdr >> m_velocity;

    dcdr >> m_acceleration;

    dcdr >> m_yaw_;
    dcdr >> m_yaw_speed_;
    dcdr >> m_point_valid_;
    dcdr >> m_type_;
}

/*!
 * @brief This function sets a value in member timestamp_
 * @param _timestamp_ New value for member timestamp_
 */
void trajectory_waypoint::timestamp_(uint64_t _timestamp_)
{
m_timestamp_ = _timestamp_;
}

/*!
 * @brief This function returns the value of member timestamp_
 * @return Value of member timestamp_
 */
uint64_t trajectory_waypoint::timestamp_() const
{
    return m_timestamp_;
}

/*!
 * @brief This function returns a reference to member timestamp_
 * @return Reference to member timestamp_
 */
uint64_t& trajectory_waypoint::timestamp_()
{
    return m_timestamp_;
}

/*!
 * @brief This function copies the value in member position
 * @param _position New value to be copied in member position
 */
void trajectory_waypoint::position(const trajectory_waypoint__float_array_3 &_position)
{
m_position = _position;
}

/*!
 * @brief This function moves the value in member position
 * @param _position New value to be moved in member position
 */
void trajectory_waypoint::position(trajectory_waypoint__float_array_3 &&_position)
{
m_position = std::move(_position);
}

/*!
 * @brief This function returns a constant reference to member position
 * @return Constant reference to member position
 */
const trajectory_waypoint__float_array_3& trajectory_waypoint::position() const
{
    return m_position;
}

/*!
 * @brief This function returns a reference to member position
 * @return Reference to member position
 */
trajectory_waypoint__float_array_3& trajectory_waypoint::position()
{
    return m_position;
}
/*!
 * @brief This function copies the value in member velocity
 * @param _velocity New value to be copied in member velocity
 */
void trajectory_waypoint::velocity(const trajectory_waypoint__float_array_3 &_velocity)
{
m_velocity = _velocity;
}

/*!
 * @brief This function moves the value in member velocity
 * @param _velocity New value to be moved in member velocity
 */
void trajectory_waypoint::velocity(trajectory_waypoint__float_array_3 &&_velocity)
{
m_velocity = std::move(_velocity);
}

/*!
 * @brief This function returns a constant reference to member velocity
 * @return Constant reference to member velocity
 */
const trajectory_waypoint__float_array_3& trajectory_waypoint::velocity() const
{
    return m_velocity;
}

/*!
 * @brief This function returns a reference to member velocity
 * @return Reference to member velocity
 */
trajectory_waypoint__float_array_3& trajectory_waypoint::velocity()
{
    return m_velocity;
}
/*!
 * @brief This function copies the value in member acceleration
 * @param _acceleration New value to be copied in member acceleration
 */
void trajectory_waypoint::acceleration(const trajectory_waypoint__float_array_3 &_acceleration)
{
m_acceleration = _acceleration;
}

/*!
 * @brief This function moves the value in member acceleration
 * @param _acceleration New value to be moved in member acceleration
 */
void trajectory_waypoint::acceleration(trajectory_waypoint__float_array_3 &&_acceleration)
{
m_acceleration = std::move(_acceleration);
}

/*!
 * @brief This function returns a constant reference to member acceleration
 * @return Constant reference to member acceleration
 */
const trajectory_waypoint__float_array_3& trajectory_waypoint::acceleration() const
{
    return m_acceleration;
}

/*!
 * @brief This function returns a reference to member acceleration
 * @return Reference to member acceleration
 */
trajectory_waypoint__float_array_3& trajectory_waypoint::acceleration()
{
    return m_acceleration;
}
/*!
 * @brief This function sets a value in member yaw_
 * @param _yaw_ New value for member yaw_
 */
void trajectory_waypoint::yaw_(float _yaw_)
{
m_yaw_ = _yaw_;
}

/*!
 * @brief This function returns the value of member yaw_
 * @return Value of member yaw_
 */
float trajectory_waypoint::yaw_() const
{
    return m_yaw_;
}

/*!
 * @brief This function returns a reference to member yaw_
 * @return Reference to member yaw_
 */
float& trajectory_waypoint::yaw_()
{
    return m_yaw_;
}

/*!
 * @brief This function sets a value in member yaw_speed_
 * @param _yaw_speed_ New value for member yaw_speed_
 */
void trajectory_waypoint::yaw_speed_(float _yaw_speed_)
{
m_yaw_speed_ = _yaw_speed_;
}

/*!
 * @brief This function returns the value of member yaw_speed_
 * @return Value of member yaw_speed_
 */
float trajectory_waypoint::yaw_speed_() const
{
    return m_yaw_speed_;
}

/*!
 * @brief This function returns a reference to member yaw_speed_
 * @return Reference to member yaw_speed_
 */
float& trajectory_waypoint::yaw_speed_()
{
    return m_yaw_speed_;
}

/*!
 * @brief This function sets a value in member point_valid_
 * @param _point_valid_ New value for member point_valid_
 */
void trajectory_waypoint::point_valid_(bool _point_valid_)
{
m_point_valid_ = _point_valid_;
}

/*!
 * @brief This function returns the value of member point_valid_
 * @return Value of member point_valid_
 */
bool trajectory_waypoint::point_valid_() const
{
    return m_point_valid_;
}

/*!
 * @brief This function returns a reference to member point_valid_
 * @return Reference to member point_valid_
 */
bool& trajectory_waypoint::point_valid_()
{
    return m_point_valid_;
}

/*!
 * @brief This function sets a value in member type_
 * @param _type_ New value for member type_
 */
void trajectory_waypoint::type_(uint8_t _type_)
{
m_type_ = _type_;
}

/*!
 * @brief This function returns the value of member type_
 * @return Value of member type_
 */
uint8_t trajectory_waypoint::type_() const
{
    return m_type_;
}

/*!
 * @brief This function returns a reference to member type_
 * @return Reference to member type_
 */
uint8_t& trajectory_waypoint::type_()
{
    return m_type_;
}


size_t trajectory_waypoint::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    size_t current_align = current_alignment;











    return current_align;
}

bool trajectory_waypoint::isKeyDefined()
{
   return false;
}

void trajectory_waypoint::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
    (void) scdr;
     
     
     
     
     
     
     
     
}
