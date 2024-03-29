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
 * @file trajectory_setpoint.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _TRAJECTORY_SETPOINT_H_
#define _TRAJECTORY_SETPOINT_H_

// TODO Poner en el contexto.

#include <stdint.h>
#include <array>
#include <string>
#include <vector>
#include <map>
#include <bitset>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif
#else
#define eProsima_user_DllExport
#endif

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(trajectory_setpoint_SOURCE)
#define trajectory_setpoint_DllAPI __declspec( dllexport )
#else
#define trajectory_setpoint_DllAPI __declspec( dllimport )
#endif // trajectory_setpoint_SOURCE
#else
#define trajectory_setpoint_DllAPI
#endif
#else
#define trajectory_setpoint_DllAPI
#endif // _WIN32

namespace eprosima
{
    namespace fastcdr
    {
        class Cdr;
    }
}


typedef std::array<float, 3> trajectory_setpoint__float_array_3;
/*!
 * @brief This class represents the structure trajectory_setpoint defined by the user in the IDL file.
 * @ingroup TRAJECTORY_SETPOINT
 */
class trajectory_setpoint
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport trajectory_setpoint();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~trajectory_setpoint();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object trajectory_setpoint that will be copied.
     */
    eProsima_user_DllExport trajectory_setpoint(const trajectory_setpoint &x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object trajectory_setpoint that will be copied.
     */
    eProsima_user_DllExport trajectory_setpoint(trajectory_setpoint &&x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object trajectory_setpoint that will be copied.
     */
    eProsima_user_DllExport trajectory_setpoint& operator=(const trajectory_setpoint &x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object trajectory_setpoint that will be copied.
     */
    eProsima_user_DllExport trajectory_setpoint& operator=(trajectory_setpoint &&x);

    /*!
     * @brief This function sets a value in member timestamp_
     * @param _timestamp_ New value for member timestamp_
     */
    eProsima_user_DllExport void timestamp_(uint64_t _timestamp_);

    /*!
     * @brief This function returns the value of member timestamp_
     * @return Value of member timestamp_
     */
    eProsima_user_DllExport uint64_t timestamp_() const;

    /*!
     * @brief This function returns a reference to member timestamp_
     * @return Reference to member timestamp_
     */
    eProsima_user_DllExport uint64_t& timestamp_();

    /*!
     * @brief This function copies the value in member position
     * @param _position New value to be copied in member position
     */
    eProsima_user_DllExport void position(const trajectory_setpoint__float_array_3 &_position);

    /*!
     * @brief This function moves the value in member position
     * @param _position New value to be moved in member position
     */
    eProsima_user_DllExport void position(trajectory_setpoint__float_array_3 &&_position);

    /*!
     * @brief This function returns a constant reference to member position
     * @return Constant reference to member position
     */
    eProsima_user_DllExport const trajectory_setpoint__float_array_3& position() const;

    /*!
     * @brief This function returns a reference to member position
     * @return Reference to member position
     */
    eProsima_user_DllExport trajectory_setpoint__float_array_3& position();
    /*!
     * @brief This function copies the value in member velocity
     * @param _velocity New value to be copied in member velocity
     */
    eProsima_user_DllExport void velocity(const trajectory_setpoint__float_array_3 &_velocity);

    /*!
     * @brief This function moves the value in member velocity
     * @param _velocity New value to be moved in member velocity
     */
    eProsima_user_DllExport void velocity(trajectory_setpoint__float_array_3 &&_velocity);

    /*!
     * @brief This function returns a constant reference to member velocity
     * @return Constant reference to member velocity
     */
    eProsima_user_DllExport const trajectory_setpoint__float_array_3& velocity() const;

    /*!
     * @brief This function returns a reference to member velocity
     * @return Reference to member velocity
     */
    eProsima_user_DllExport trajectory_setpoint__float_array_3& velocity();
    /*!
     * @brief This function copies the value in member acceleration
     * @param _acceleration New value to be copied in member acceleration
     */
    eProsima_user_DllExport void acceleration(const trajectory_setpoint__float_array_3 &_acceleration);

    /*!
     * @brief This function moves the value in member acceleration
     * @param _acceleration New value to be moved in member acceleration
     */
    eProsima_user_DllExport void acceleration(trajectory_setpoint__float_array_3 &&_acceleration);

    /*!
     * @brief This function returns a constant reference to member acceleration
     * @return Constant reference to member acceleration
     */
    eProsima_user_DllExport const trajectory_setpoint__float_array_3& acceleration() const;

    /*!
     * @brief This function returns a reference to member acceleration
     * @return Reference to member acceleration
     */
    eProsima_user_DllExport trajectory_setpoint__float_array_3& acceleration();
    /*!
     * @brief This function copies the value in member jerk
     * @param _jerk New value to be copied in member jerk
     */
    eProsima_user_DllExport void jerk(const trajectory_setpoint__float_array_3 &_jerk);

    /*!
     * @brief This function moves the value in member jerk
     * @param _jerk New value to be moved in member jerk
     */
    eProsima_user_DllExport void jerk(trajectory_setpoint__float_array_3 &&_jerk);

    /*!
     * @brief This function returns a constant reference to member jerk
     * @return Constant reference to member jerk
     */
    eProsima_user_DllExport const trajectory_setpoint__float_array_3& jerk() const;

    /*!
     * @brief This function returns a reference to member jerk
     * @return Reference to member jerk
     */
    eProsima_user_DllExport trajectory_setpoint__float_array_3& jerk();
    /*!
     * @brief This function sets a value in member yaw_
     * @param _yaw_ New value for member yaw_
     */
    eProsima_user_DllExport void yaw_(float _yaw_);

    /*!
     * @brief This function returns the value of member yaw_
     * @return Value of member yaw_
     */
    eProsima_user_DllExport float yaw_() const;

    /*!
     * @brief This function returns a reference to member yaw_
     * @return Reference to member yaw_
     */
    eProsima_user_DllExport float& yaw_();

    /*!
     * @brief This function sets a value in member yawspeed_
     * @param _yawspeed_ New value for member yawspeed_
     */
    eProsima_user_DllExport void yawspeed_(float _yawspeed_);

    /*!
     * @brief This function returns the value of member yawspeed_
     * @return Value of member yawspeed_
     */
    eProsima_user_DllExport float yawspeed_() const;

    /*!
     * @brief This function returns a reference to member yawspeed_
     * @return Reference to member yawspeed_
     */
    eProsima_user_DllExport float& yawspeed_();


    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getMaxCdrSerializedSize(size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    eProsima_user_DllExport static size_t getCdrSerializedSize(const trajectory_setpoint& data, size_t current_alignment = 0);


    /*!
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serialize(eprosima::fastcdr::Cdr &cdr) const;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void deserialize(eprosima::fastcdr::Cdr &cdr);



    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    eProsima_user_DllExport static bool isKeyDefined();

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serializeKey(eprosima::fastcdr::Cdr &cdr) const;

private:
    uint64_t m_timestamp_;
    trajectory_setpoint__float_array_3 m_position;
    trajectory_setpoint__float_array_3 m_velocity;
    trajectory_setpoint__float_array_3 m_acceleration;
    trajectory_setpoint__float_array_3 m_jerk;
    float m_yaw_;
    float m_yawspeed_;
};

#endif // _TRAJECTORY_SETPOINT_H_