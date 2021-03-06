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
 * @file input_rc.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _INPUT_RC_H_
#define _INPUT_RC_H_

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
#if defined(input_rc_SOURCE)
#define input_rc_DllAPI __declspec( dllexport )
#else
#define input_rc_DllAPI __declspec( dllimport )
#endif // input_rc_SOURCE
#else
#define input_rc_DllAPI
#endif
#else
#define input_rc_DllAPI
#endif // _WIN32

namespace eprosima
{
    namespace fastcdr
    {
        class Cdr;
    }
}


const uint8_t input_rc__RC_INPUT_SOURCE_UNKNOWN = 0;
const uint8_t input_rc__RC_INPUT_SOURCE_PX4FMU_PPM = 1;
const uint8_t input_rc__RC_INPUT_SOURCE_PX4IO_PPM = 2;
const uint8_t input_rc__RC_INPUT_SOURCE_PX4IO_SPEKTRUM = 3;
const uint8_t input_rc__RC_INPUT_SOURCE_PX4IO_SBUS = 4;
const uint8_t input_rc__RC_INPUT_SOURCE_PX4IO_ST24 = 5;
const uint8_t input_rc__RC_INPUT_SOURCE_MAVLINK = 6;
const uint8_t input_rc__RC_INPUT_SOURCE_QURT = 7;
const uint8_t input_rc__RC_INPUT_SOURCE_PX4FMU_SPEKTRUM = 8;
const uint8_t input_rc__RC_INPUT_SOURCE_PX4FMU_SBUS = 9;
const uint8_t input_rc__RC_INPUT_SOURCE_PX4FMU_ST24 = 10;
const uint8_t input_rc__RC_INPUT_SOURCE_PX4FMU_SUMD = 11;
const uint8_t input_rc__RC_INPUT_SOURCE_PX4FMU_DSM = 12;
const uint8_t input_rc__RC_INPUT_SOURCE_PX4IO_SUMD = 13;
const uint8_t input_rc__RC_INPUT_SOURCE_PX4FMU_CRSF = 14;
const uint8_t input_rc__RC_INPUT_SOURCE_PX4FMU_GHST = 15;
const uint8_t input_rc__RC_INPUT_MAX_CHANNELS = 18;
typedef std::array<uint16_t, 18> input_rc__unsigned_short_array_18;
/*!
 * @brief This class represents the structure input_rc defined by the user in the IDL file.
 * @ingroup INPUT_RC
 */
class input_rc
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport input_rc();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~input_rc();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object input_rc that will be copied.
     */
    eProsima_user_DllExport input_rc(const input_rc &x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object input_rc that will be copied.
     */
    eProsima_user_DllExport input_rc(input_rc &&x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object input_rc that will be copied.
     */
    eProsima_user_DllExport input_rc& operator=(const input_rc &x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object input_rc that will be copied.
     */
    eProsima_user_DllExport input_rc& operator=(input_rc &&x);

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
     * @brief This function sets a value in member timestamp_last_signal_
     * @param _timestamp_last_signal_ New value for member timestamp_last_signal_
     */
    eProsima_user_DllExport void timestamp_last_signal_(uint64_t _timestamp_last_signal_);

    /*!
     * @brief This function returns the value of member timestamp_last_signal_
     * @return Value of member timestamp_last_signal_
     */
    eProsima_user_DllExport uint64_t timestamp_last_signal_() const;

    /*!
     * @brief This function returns a reference to member timestamp_last_signal_
     * @return Reference to member timestamp_last_signal_
     */
    eProsima_user_DllExport uint64_t& timestamp_last_signal_();

    /*!
     * @brief This function sets a value in member channel_count_
     * @param _channel_count_ New value for member channel_count_
     */
    eProsima_user_DllExport void channel_count_(uint8_t _channel_count_);

    /*!
     * @brief This function returns the value of member channel_count_
     * @return Value of member channel_count_
     */
    eProsima_user_DllExport uint8_t channel_count_() const;

    /*!
     * @brief This function returns a reference to member channel_count_
     * @return Reference to member channel_count_
     */
    eProsima_user_DllExport uint8_t& channel_count_();

    /*!
     * @brief This function sets a value in member rssi_
     * @param _rssi_ New value for member rssi_
     */
    eProsima_user_DllExport void rssi_(int32_t _rssi_);

    /*!
     * @brief This function returns the value of member rssi_
     * @return Value of member rssi_
     */
    eProsima_user_DllExport int32_t rssi_() const;

    /*!
     * @brief This function returns a reference to member rssi_
     * @return Reference to member rssi_
     */
    eProsima_user_DllExport int32_t& rssi_();

    /*!
     * @brief This function sets a value in member rc_failsafe_
     * @param _rc_failsafe_ New value for member rc_failsafe_
     */
    eProsima_user_DllExport void rc_failsafe_(bool _rc_failsafe_);

    /*!
     * @brief This function returns the value of member rc_failsafe_
     * @return Value of member rc_failsafe_
     */
    eProsima_user_DllExport bool rc_failsafe_() const;

    /*!
     * @brief This function returns a reference to member rc_failsafe_
     * @return Reference to member rc_failsafe_
     */
    eProsima_user_DllExport bool& rc_failsafe_();

    /*!
     * @brief This function sets a value in member rc_lost_
     * @param _rc_lost_ New value for member rc_lost_
     */
    eProsima_user_DllExport void rc_lost_(bool _rc_lost_);

    /*!
     * @brief This function returns the value of member rc_lost_
     * @return Value of member rc_lost_
     */
    eProsima_user_DllExport bool rc_lost_() const;

    /*!
     * @brief This function returns a reference to member rc_lost_
     * @return Reference to member rc_lost_
     */
    eProsima_user_DllExport bool& rc_lost_();

    /*!
     * @brief This function sets a value in member rc_lost_frame_count_
     * @param _rc_lost_frame_count_ New value for member rc_lost_frame_count_
     */
    eProsima_user_DllExport void rc_lost_frame_count_(uint16_t _rc_lost_frame_count_);

    /*!
     * @brief This function returns the value of member rc_lost_frame_count_
     * @return Value of member rc_lost_frame_count_
     */
    eProsima_user_DllExport uint16_t rc_lost_frame_count_() const;

    /*!
     * @brief This function returns a reference to member rc_lost_frame_count_
     * @return Reference to member rc_lost_frame_count_
     */
    eProsima_user_DllExport uint16_t& rc_lost_frame_count_();

    /*!
     * @brief This function sets a value in member rc_total_frame_count_
     * @param _rc_total_frame_count_ New value for member rc_total_frame_count_
     */
    eProsima_user_DllExport void rc_total_frame_count_(uint16_t _rc_total_frame_count_);

    /*!
     * @brief This function returns the value of member rc_total_frame_count_
     * @return Value of member rc_total_frame_count_
     */
    eProsima_user_DllExport uint16_t rc_total_frame_count_() const;

    /*!
     * @brief This function returns a reference to member rc_total_frame_count_
     * @return Reference to member rc_total_frame_count_
     */
    eProsima_user_DllExport uint16_t& rc_total_frame_count_();

    /*!
     * @brief This function sets a value in member rc_ppm_frame_length_
     * @param _rc_ppm_frame_length_ New value for member rc_ppm_frame_length_
     */
    eProsima_user_DllExport void rc_ppm_frame_length_(uint16_t _rc_ppm_frame_length_);

    /*!
     * @brief This function returns the value of member rc_ppm_frame_length_
     * @return Value of member rc_ppm_frame_length_
     */
    eProsima_user_DllExport uint16_t rc_ppm_frame_length_() const;

    /*!
     * @brief This function returns a reference to member rc_ppm_frame_length_
     * @return Reference to member rc_ppm_frame_length_
     */
    eProsima_user_DllExport uint16_t& rc_ppm_frame_length_();

    /*!
     * @brief This function sets a value in member input_source_
     * @param _input_source_ New value for member input_source_
     */
    eProsima_user_DllExport void input_source_(uint8_t _input_source_);

    /*!
     * @brief This function returns the value of member input_source_
     * @return Value of member input_source_
     */
    eProsima_user_DllExport uint8_t input_source_() const;

    /*!
     * @brief This function returns a reference to member input_source_
     * @return Reference to member input_source_
     */
    eProsima_user_DllExport uint8_t& input_source_();

    /*!
     * @brief This function copies the value in member values
     * @param _values New value to be copied in member values
     */
    eProsima_user_DllExport void values(const input_rc__unsigned_short_array_18 &_values);

    /*!
     * @brief This function moves the value in member values
     * @param _values New value to be moved in member values
     */
    eProsima_user_DllExport void values(input_rc__unsigned_short_array_18 &&_values);

    /*!
     * @brief This function returns a constant reference to member values
     * @return Constant reference to member values
     */
    eProsima_user_DllExport const input_rc__unsigned_short_array_18& values() const;

    /*!
     * @brief This function returns a reference to member values
     * @return Reference to member values
     */
    eProsima_user_DllExport input_rc__unsigned_short_array_18& values();

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
    eProsima_user_DllExport static size_t getCdrSerializedSize(const input_rc& data, size_t current_alignment = 0);


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
    uint64_t m_timestamp_last_signal_;
    uint8_t m_channel_count_;
    int32_t m_rssi_;
    bool m_rc_failsafe_;
    bool m_rc_lost_;
    uint16_t m_rc_lost_frame_count_;
    uint16_t m_rc_total_frame_count_;
    uint16_t m_rc_ppm_frame_length_;
    uint8_t m_input_source_;
    input_rc__unsigned_short_array_18 m_values;
};

#endif // _INPUT_RC_H_