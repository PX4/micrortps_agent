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
 * @file vehicle_control_mode.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _VEHICLE_CONTROL_MODE_H_
#define _VEHICLE_CONTROL_MODE_H_

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
#if defined(vehicle_control_mode_SOURCE)
#define vehicle_control_mode_DllAPI __declspec( dllexport )
#else
#define vehicle_control_mode_DllAPI __declspec( dllimport )
#endif // vehicle_control_mode_SOURCE
#else
#define vehicle_control_mode_DllAPI
#endif
#else
#define vehicle_control_mode_DllAPI
#endif // _WIN32

namespace eprosima
{
    namespace fastcdr
    {
        class Cdr;
    }
}


/*!
 * @brief This class represents the structure vehicle_control_mode defined by the user in the IDL file.
 * @ingroup VEHICLE_CONTROL_MODE
 */
class vehicle_control_mode
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport vehicle_control_mode();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~vehicle_control_mode();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object vehicle_control_mode that will be copied.
     */
    eProsima_user_DllExport vehicle_control_mode(const vehicle_control_mode &x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object vehicle_control_mode that will be copied.
     */
    eProsima_user_DllExport vehicle_control_mode(vehicle_control_mode &&x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object vehicle_control_mode that will be copied.
     */
    eProsima_user_DllExport vehicle_control_mode& operator=(const vehicle_control_mode &x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object vehicle_control_mode that will be copied.
     */
    eProsima_user_DllExport vehicle_control_mode& operator=(vehicle_control_mode &&x);

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
     * @brief This function sets a value in member flag_armed_
     * @param _flag_armed_ New value for member flag_armed_
     */
    eProsima_user_DllExport void flag_armed_(bool _flag_armed_);

    /*!
     * @brief This function returns the value of member flag_armed_
     * @return Value of member flag_armed_
     */
    eProsima_user_DllExport bool flag_armed_() const;

    /*!
     * @brief This function returns a reference to member flag_armed_
     * @return Reference to member flag_armed_
     */
    eProsima_user_DllExport bool& flag_armed_();

    /*!
     * @brief This function sets a value in member flag_external_manual_override_ok_
     * @param _flag_external_manual_override_ok_ New value for member flag_external_manual_override_ok_
     */
    eProsima_user_DllExport void flag_external_manual_override_ok_(bool _flag_external_manual_override_ok_);

    /*!
     * @brief This function returns the value of member flag_external_manual_override_ok_
     * @return Value of member flag_external_manual_override_ok_
     */
    eProsima_user_DllExport bool flag_external_manual_override_ok_() const;

    /*!
     * @brief This function returns a reference to member flag_external_manual_override_ok_
     * @return Reference to member flag_external_manual_override_ok_
     */
    eProsima_user_DllExport bool& flag_external_manual_override_ok_();

    /*!
     * @brief This function sets a value in member flag_control_manual_enabled_
     * @param _flag_control_manual_enabled_ New value for member flag_control_manual_enabled_
     */
    eProsima_user_DllExport void flag_control_manual_enabled_(bool _flag_control_manual_enabled_);

    /*!
     * @brief This function returns the value of member flag_control_manual_enabled_
     * @return Value of member flag_control_manual_enabled_
     */
    eProsima_user_DllExport bool flag_control_manual_enabled_() const;

    /*!
     * @brief This function returns a reference to member flag_control_manual_enabled_
     * @return Reference to member flag_control_manual_enabled_
     */
    eProsima_user_DllExport bool& flag_control_manual_enabled_();

    /*!
     * @brief This function sets a value in member flag_control_auto_enabled_
     * @param _flag_control_auto_enabled_ New value for member flag_control_auto_enabled_
     */
    eProsima_user_DllExport void flag_control_auto_enabled_(bool _flag_control_auto_enabled_);

    /*!
     * @brief This function returns the value of member flag_control_auto_enabled_
     * @return Value of member flag_control_auto_enabled_
     */
    eProsima_user_DllExport bool flag_control_auto_enabled_() const;

    /*!
     * @brief This function returns a reference to member flag_control_auto_enabled_
     * @return Reference to member flag_control_auto_enabled_
     */
    eProsima_user_DllExport bool& flag_control_auto_enabled_();

    /*!
     * @brief This function sets a value in member flag_control_offboard_enabled_
     * @param _flag_control_offboard_enabled_ New value for member flag_control_offboard_enabled_
     */
    eProsima_user_DllExport void flag_control_offboard_enabled_(bool _flag_control_offboard_enabled_);

    /*!
     * @brief This function returns the value of member flag_control_offboard_enabled_
     * @return Value of member flag_control_offboard_enabled_
     */
    eProsima_user_DllExport bool flag_control_offboard_enabled_() const;

    /*!
     * @brief This function returns a reference to member flag_control_offboard_enabled_
     * @return Reference to member flag_control_offboard_enabled_
     */
    eProsima_user_DllExport bool& flag_control_offboard_enabled_();

    /*!
     * @brief This function sets a value in member flag_control_rates_enabled_
     * @param _flag_control_rates_enabled_ New value for member flag_control_rates_enabled_
     */
    eProsima_user_DllExport void flag_control_rates_enabled_(bool _flag_control_rates_enabled_);

    /*!
     * @brief This function returns the value of member flag_control_rates_enabled_
     * @return Value of member flag_control_rates_enabled_
     */
    eProsima_user_DllExport bool flag_control_rates_enabled_() const;

    /*!
     * @brief This function returns a reference to member flag_control_rates_enabled_
     * @return Reference to member flag_control_rates_enabled_
     */
    eProsima_user_DllExport bool& flag_control_rates_enabled_();

    /*!
     * @brief This function sets a value in member flag_control_attitude_enabled_
     * @param _flag_control_attitude_enabled_ New value for member flag_control_attitude_enabled_
     */
    eProsima_user_DllExport void flag_control_attitude_enabled_(bool _flag_control_attitude_enabled_);

    /*!
     * @brief This function returns the value of member flag_control_attitude_enabled_
     * @return Value of member flag_control_attitude_enabled_
     */
    eProsima_user_DllExport bool flag_control_attitude_enabled_() const;

    /*!
     * @brief This function returns a reference to member flag_control_attitude_enabled_
     * @return Reference to member flag_control_attitude_enabled_
     */
    eProsima_user_DllExport bool& flag_control_attitude_enabled_();

    /*!
     * @brief This function sets a value in member flag_control_yawrate_override_enabled_
     * @param _flag_control_yawrate_override_enabled_ New value for member flag_control_yawrate_override_enabled_
     */
    eProsima_user_DllExport void flag_control_yawrate_override_enabled_(bool _flag_control_yawrate_override_enabled_);

    /*!
     * @brief This function returns the value of member flag_control_yawrate_override_enabled_
     * @return Value of member flag_control_yawrate_override_enabled_
     */
    eProsima_user_DllExport bool flag_control_yawrate_override_enabled_() const;

    /*!
     * @brief This function returns a reference to member flag_control_yawrate_override_enabled_
     * @return Reference to member flag_control_yawrate_override_enabled_
     */
    eProsima_user_DllExport bool& flag_control_yawrate_override_enabled_();

    /*!
     * @brief This function sets a value in member flag_control_rattitude_enabled_
     * @param _flag_control_rattitude_enabled_ New value for member flag_control_rattitude_enabled_
     */
    eProsima_user_DllExport void flag_control_rattitude_enabled_(bool _flag_control_rattitude_enabled_);

    /*!
     * @brief This function returns the value of member flag_control_rattitude_enabled_
     * @return Value of member flag_control_rattitude_enabled_
     */
    eProsima_user_DllExport bool flag_control_rattitude_enabled_() const;

    /*!
     * @brief This function returns a reference to member flag_control_rattitude_enabled_
     * @return Reference to member flag_control_rattitude_enabled_
     */
    eProsima_user_DllExport bool& flag_control_rattitude_enabled_();

    /*!
     * @brief This function sets a value in member flag_control_force_enabled_
     * @param _flag_control_force_enabled_ New value for member flag_control_force_enabled_
     */
    eProsima_user_DllExport void flag_control_force_enabled_(bool _flag_control_force_enabled_);

    /*!
     * @brief This function returns the value of member flag_control_force_enabled_
     * @return Value of member flag_control_force_enabled_
     */
    eProsima_user_DllExport bool flag_control_force_enabled_() const;

    /*!
     * @brief This function returns a reference to member flag_control_force_enabled_
     * @return Reference to member flag_control_force_enabled_
     */
    eProsima_user_DllExport bool& flag_control_force_enabled_();

    /*!
     * @brief This function sets a value in member flag_control_acceleration_enabled_
     * @param _flag_control_acceleration_enabled_ New value for member flag_control_acceleration_enabled_
     */
    eProsima_user_DllExport void flag_control_acceleration_enabled_(bool _flag_control_acceleration_enabled_);

    /*!
     * @brief This function returns the value of member flag_control_acceleration_enabled_
     * @return Value of member flag_control_acceleration_enabled_
     */
    eProsima_user_DllExport bool flag_control_acceleration_enabled_() const;

    /*!
     * @brief This function returns a reference to member flag_control_acceleration_enabled_
     * @return Reference to member flag_control_acceleration_enabled_
     */
    eProsima_user_DllExport bool& flag_control_acceleration_enabled_();

    /*!
     * @brief This function sets a value in member flag_control_velocity_enabled_
     * @param _flag_control_velocity_enabled_ New value for member flag_control_velocity_enabled_
     */
    eProsima_user_DllExport void flag_control_velocity_enabled_(bool _flag_control_velocity_enabled_);

    /*!
     * @brief This function returns the value of member flag_control_velocity_enabled_
     * @return Value of member flag_control_velocity_enabled_
     */
    eProsima_user_DllExport bool flag_control_velocity_enabled_() const;

    /*!
     * @brief This function returns a reference to member flag_control_velocity_enabled_
     * @return Reference to member flag_control_velocity_enabled_
     */
    eProsima_user_DllExport bool& flag_control_velocity_enabled_();

    /*!
     * @brief This function sets a value in member flag_control_position_enabled_
     * @param _flag_control_position_enabled_ New value for member flag_control_position_enabled_
     */
    eProsima_user_DllExport void flag_control_position_enabled_(bool _flag_control_position_enabled_);

    /*!
     * @brief This function returns the value of member flag_control_position_enabled_
     * @return Value of member flag_control_position_enabled_
     */
    eProsima_user_DllExport bool flag_control_position_enabled_() const;

    /*!
     * @brief This function returns a reference to member flag_control_position_enabled_
     * @return Reference to member flag_control_position_enabled_
     */
    eProsima_user_DllExport bool& flag_control_position_enabled_();

    /*!
     * @brief This function sets a value in member flag_control_altitude_enabled_
     * @param _flag_control_altitude_enabled_ New value for member flag_control_altitude_enabled_
     */
    eProsima_user_DllExport void flag_control_altitude_enabled_(bool _flag_control_altitude_enabled_);

    /*!
     * @brief This function returns the value of member flag_control_altitude_enabled_
     * @return Value of member flag_control_altitude_enabled_
     */
    eProsima_user_DllExport bool flag_control_altitude_enabled_() const;

    /*!
     * @brief This function returns a reference to member flag_control_altitude_enabled_
     * @return Reference to member flag_control_altitude_enabled_
     */
    eProsima_user_DllExport bool& flag_control_altitude_enabled_();

    /*!
     * @brief This function sets a value in member flag_control_climb_rate_enabled_
     * @param _flag_control_climb_rate_enabled_ New value for member flag_control_climb_rate_enabled_
     */
    eProsima_user_DllExport void flag_control_climb_rate_enabled_(bool _flag_control_climb_rate_enabled_);

    /*!
     * @brief This function returns the value of member flag_control_climb_rate_enabled_
     * @return Value of member flag_control_climb_rate_enabled_
     */
    eProsima_user_DllExport bool flag_control_climb_rate_enabled_() const;

    /*!
     * @brief This function returns a reference to member flag_control_climb_rate_enabled_
     * @return Reference to member flag_control_climb_rate_enabled_
     */
    eProsima_user_DllExport bool& flag_control_climb_rate_enabled_();

    /*!
     * @brief This function sets a value in member flag_control_termination_enabled_
     * @param _flag_control_termination_enabled_ New value for member flag_control_termination_enabled_
     */
    eProsima_user_DllExport void flag_control_termination_enabled_(bool _flag_control_termination_enabled_);

    /*!
     * @brief This function returns the value of member flag_control_termination_enabled_
     * @return Value of member flag_control_termination_enabled_
     */
    eProsima_user_DllExport bool flag_control_termination_enabled_() const;

    /*!
     * @brief This function returns a reference to member flag_control_termination_enabled_
     * @return Reference to member flag_control_termination_enabled_
     */
    eProsima_user_DllExport bool& flag_control_termination_enabled_();

    /*!
     * @brief This function sets a value in member flag_control_fixed_hdg_enabled_
     * @param _flag_control_fixed_hdg_enabled_ New value for member flag_control_fixed_hdg_enabled_
     */
    eProsima_user_DllExport void flag_control_fixed_hdg_enabled_(bool _flag_control_fixed_hdg_enabled_);

    /*!
     * @brief This function returns the value of member flag_control_fixed_hdg_enabled_
     * @return Value of member flag_control_fixed_hdg_enabled_
     */
    eProsima_user_DllExport bool flag_control_fixed_hdg_enabled_() const;

    /*!
     * @brief This function returns a reference to member flag_control_fixed_hdg_enabled_
     * @return Reference to member flag_control_fixed_hdg_enabled_
     */
    eProsima_user_DllExport bool& flag_control_fixed_hdg_enabled_();


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
    eProsima_user_DllExport static size_t getCdrSerializedSize(const vehicle_control_mode& data, size_t current_alignment = 0);


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
    bool m_flag_armed_;
    bool m_flag_external_manual_override_ok_;
    bool m_flag_control_manual_enabled_;
    bool m_flag_control_auto_enabled_;
    bool m_flag_control_offboard_enabled_;
    bool m_flag_control_rates_enabled_;
    bool m_flag_control_attitude_enabled_;
    bool m_flag_control_yawrate_override_enabled_;
    bool m_flag_control_rattitude_enabled_;
    bool m_flag_control_force_enabled_;
    bool m_flag_control_acceleration_enabled_;
    bool m_flag_control_velocity_enabled_;
    bool m_flag_control_position_enabled_;
    bool m_flag_control_altitude_enabled_;
    bool m_flag_control_climb_rate_enabled_;
    bool m_flag_control_termination_enabled_;
    bool m_flag_control_fixed_hdg_enabled_;
};

#endif // _VEHICLE_CONTROL_MODE_H_