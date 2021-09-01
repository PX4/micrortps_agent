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
 * @file onboard_computer_status.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _ONBOARD_COMPUTER_STATUS_H_
#define _ONBOARD_COMPUTER_STATUS_H_

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
#if defined(onboard_computer_status_SOURCE)
#define onboard_computer_status_DllAPI __declspec( dllexport )
#else
#define onboard_computer_status_DllAPI __declspec( dllimport )
#endif // onboard_computer_status_SOURCE
#else
#define onboard_computer_status_DllAPI
#endif
#else
#define onboard_computer_status_DllAPI
#endif // _WIN32

namespace eprosima
{
    namespace fastcdr
    {
        class Cdr;
    }
}


typedef std::array<uint8_t, 8> onboard_computer_status__octet_array_8;
typedef std::array<uint32_t, 6> onboard_computer_status__unsigned_long_array_6;
typedef std::array<uint32_t, 4> onboard_computer_status__unsigned_long_array_4;
typedef std::array<int16_t, 4> onboard_computer_status__short_array_4;
typedef std::array<uint8_t, 4> onboard_computer_status__octet_array_4;
typedef std::array<uint8_t, 10> onboard_computer_status__octet_array_10;
/*!
 * @brief This class represents the structure onboard_computer_status defined by the user in the IDL file.
 * @ingroup ONBOARD_COMPUTER_STATUS
 */
class onboard_computer_status
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport onboard_computer_status();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~onboard_computer_status();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object onboard_computer_status that will be copied.
     */
    eProsima_user_DllExport onboard_computer_status(const onboard_computer_status &x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object onboard_computer_status that will be copied.
     */
    eProsima_user_DllExport onboard_computer_status(onboard_computer_status &&x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object onboard_computer_status that will be copied.
     */
    eProsima_user_DllExport onboard_computer_status& operator=(const onboard_computer_status &x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object onboard_computer_status that will be copied.
     */
    eProsima_user_DllExport onboard_computer_status& operator=(onboard_computer_status &&x);

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
     * @brief This function sets a value in member uptime_
     * @param _uptime_ New value for member uptime_
     */
    eProsima_user_DllExport void uptime_(uint32_t _uptime_);

    /*!
     * @brief This function returns the value of member uptime_
     * @return Value of member uptime_
     */
    eProsima_user_DllExport uint32_t uptime_() const;

    /*!
     * @brief This function returns a reference to member uptime_
     * @return Reference to member uptime_
     */
    eProsima_user_DllExport uint32_t& uptime_();

    /*!
     * @brief This function sets a value in member type_
     * @param _type_ New value for member type_
     */
    eProsima_user_DllExport void type_(uint8_t _type_);

    /*!
     * @brief This function returns the value of member type_
     * @return Value of member type_
     */
    eProsima_user_DllExport uint8_t type_() const;

    /*!
     * @brief This function returns a reference to member type_
     * @return Reference to member type_
     */
    eProsima_user_DllExport uint8_t& type_();

    /*!
     * @brief This function copies the value in member cpu_cores
     * @param _cpu_cores New value to be copied in member cpu_cores
     */
    eProsima_user_DllExport void cpu_cores(const onboard_computer_status__octet_array_8 &_cpu_cores);

    /*!
     * @brief This function moves the value in member cpu_cores
     * @param _cpu_cores New value to be moved in member cpu_cores
     */
    eProsima_user_DllExport void cpu_cores(onboard_computer_status__octet_array_8 &&_cpu_cores);

    /*!
     * @brief This function returns a constant reference to member cpu_cores
     * @return Constant reference to member cpu_cores
     */
    eProsima_user_DllExport const onboard_computer_status__octet_array_8& cpu_cores() const;

    /*!
     * @brief This function returns a reference to member cpu_cores
     * @return Reference to member cpu_cores
     */
    eProsima_user_DllExport onboard_computer_status__octet_array_8& cpu_cores();
    /*!
     * @brief This function copies the value in member cpu_combined
     * @param _cpu_combined New value to be copied in member cpu_combined
     */
    eProsima_user_DllExport void cpu_combined(const onboard_computer_status__octet_array_10 &_cpu_combined);

    /*!
     * @brief This function moves the value in member cpu_combined
     * @param _cpu_combined New value to be moved in member cpu_combined
     */
    eProsima_user_DllExport void cpu_combined(onboard_computer_status__octet_array_10 &&_cpu_combined);

    /*!
     * @brief This function returns a constant reference to member cpu_combined
     * @return Constant reference to member cpu_combined
     */
    eProsima_user_DllExport const onboard_computer_status__octet_array_10& cpu_combined() const;

    /*!
     * @brief This function returns a reference to member cpu_combined
     * @return Reference to member cpu_combined
     */
    eProsima_user_DllExport onboard_computer_status__octet_array_10& cpu_combined();
    /*!
     * @brief This function copies the value in member gpu_cores
     * @param _gpu_cores New value to be copied in member gpu_cores
     */
    eProsima_user_DllExport void gpu_cores(const onboard_computer_status__octet_array_4 &_gpu_cores);

    /*!
     * @brief This function moves the value in member gpu_cores
     * @param _gpu_cores New value to be moved in member gpu_cores
     */
    eProsima_user_DllExport void gpu_cores(onboard_computer_status__octet_array_4 &&_gpu_cores);

    /*!
     * @brief This function returns a constant reference to member gpu_cores
     * @return Constant reference to member gpu_cores
     */
    eProsima_user_DllExport const onboard_computer_status__octet_array_4& gpu_cores() const;

    /*!
     * @brief This function returns a reference to member gpu_cores
     * @return Reference to member gpu_cores
     */
    eProsima_user_DllExport onboard_computer_status__octet_array_4& gpu_cores();
    /*!
     * @brief This function copies the value in member gpu_combined
     * @param _gpu_combined New value to be copied in member gpu_combined
     */
    eProsima_user_DllExport void gpu_combined(const onboard_computer_status__octet_array_10 &_gpu_combined);

    /*!
     * @brief This function moves the value in member gpu_combined
     * @param _gpu_combined New value to be moved in member gpu_combined
     */
    eProsima_user_DllExport void gpu_combined(onboard_computer_status__octet_array_10 &&_gpu_combined);

    /*!
     * @brief This function returns a constant reference to member gpu_combined
     * @return Constant reference to member gpu_combined
     */
    eProsima_user_DllExport const onboard_computer_status__octet_array_10& gpu_combined() const;

    /*!
     * @brief This function returns a reference to member gpu_combined
     * @return Reference to member gpu_combined
     */
    eProsima_user_DllExport onboard_computer_status__octet_array_10& gpu_combined();
    /*!
     * @brief This function sets a value in member temperature_board_
     * @param _temperature_board_ New value for member temperature_board_
     */
    eProsima_user_DllExport void temperature_board_(uint8_t _temperature_board_);

    /*!
     * @brief This function returns the value of member temperature_board_
     * @return Value of member temperature_board_
     */
    eProsima_user_DllExport uint8_t temperature_board_() const;

    /*!
     * @brief This function returns a reference to member temperature_board_
     * @return Reference to member temperature_board_
     */
    eProsima_user_DllExport uint8_t& temperature_board_();

    /*!
     * @brief This function copies the value in member temperature_core
     * @param _temperature_core New value to be copied in member temperature_core
     */
    eProsima_user_DllExport void temperature_core(const onboard_computer_status__octet_array_8 &_temperature_core);

    /*!
     * @brief This function moves the value in member temperature_core
     * @param _temperature_core New value to be moved in member temperature_core
     */
    eProsima_user_DllExport void temperature_core(onboard_computer_status__octet_array_8 &&_temperature_core);

    /*!
     * @brief This function returns a constant reference to member temperature_core
     * @return Constant reference to member temperature_core
     */
    eProsima_user_DllExport const onboard_computer_status__octet_array_8& temperature_core() const;

    /*!
     * @brief This function returns a reference to member temperature_core
     * @return Reference to member temperature_core
     */
    eProsima_user_DllExport onboard_computer_status__octet_array_8& temperature_core();
    /*!
     * @brief This function copies the value in member fan_speed
     * @param _fan_speed New value to be copied in member fan_speed
     */
    eProsima_user_DllExport void fan_speed(const onboard_computer_status__short_array_4 &_fan_speed);

    /*!
     * @brief This function moves the value in member fan_speed
     * @param _fan_speed New value to be moved in member fan_speed
     */
    eProsima_user_DllExport void fan_speed(onboard_computer_status__short_array_4 &&_fan_speed);

    /*!
     * @brief This function returns a constant reference to member fan_speed
     * @return Constant reference to member fan_speed
     */
    eProsima_user_DllExport const onboard_computer_status__short_array_4& fan_speed() const;

    /*!
     * @brief This function returns a reference to member fan_speed
     * @return Reference to member fan_speed
     */
    eProsima_user_DllExport onboard_computer_status__short_array_4& fan_speed();
    /*!
     * @brief This function sets a value in member ram_usage_
     * @param _ram_usage_ New value for member ram_usage_
     */
    eProsima_user_DllExport void ram_usage_(uint32_t _ram_usage_);

    /*!
     * @brief This function returns the value of member ram_usage_
     * @return Value of member ram_usage_
     */
    eProsima_user_DllExport uint32_t ram_usage_() const;

    /*!
     * @brief This function returns a reference to member ram_usage_
     * @return Reference to member ram_usage_
     */
    eProsima_user_DllExport uint32_t& ram_usage_();

    /*!
     * @brief This function sets a value in member ram_total_
     * @param _ram_total_ New value for member ram_total_
     */
    eProsima_user_DllExport void ram_total_(uint32_t _ram_total_);

    /*!
     * @brief This function returns the value of member ram_total_
     * @return Value of member ram_total_
     */
    eProsima_user_DllExport uint32_t ram_total_() const;

    /*!
     * @brief This function returns a reference to member ram_total_
     * @return Reference to member ram_total_
     */
    eProsima_user_DllExport uint32_t& ram_total_();

    /*!
     * @brief This function copies the value in member storage_type
     * @param _storage_type New value to be copied in member storage_type
     */
    eProsima_user_DllExport void storage_type(const onboard_computer_status__unsigned_long_array_4 &_storage_type);

    /*!
     * @brief This function moves the value in member storage_type
     * @param _storage_type New value to be moved in member storage_type
     */
    eProsima_user_DllExport void storage_type(onboard_computer_status__unsigned_long_array_4 &&_storage_type);

    /*!
     * @brief This function returns a constant reference to member storage_type
     * @return Constant reference to member storage_type
     */
    eProsima_user_DllExport const onboard_computer_status__unsigned_long_array_4& storage_type() const;

    /*!
     * @brief This function returns a reference to member storage_type
     * @return Reference to member storage_type
     */
    eProsima_user_DllExport onboard_computer_status__unsigned_long_array_4& storage_type();
    /*!
     * @brief This function copies the value in member storage_usage
     * @param _storage_usage New value to be copied in member storage_usage
     */
    eProsima_user_DllExport void storage_usage(const onboard_computer_status__unsigned_long_array_4 &_storage_usage);

    /*!
     * @brief This function moves the value in member storage_usage
     * @param _storage_usage New value to be moved in member storage_usage
     */
    eProsima_user_DllExport void storage_usage(onboard_computer_status__unsigned_long_array_4 &&_storage_usage);

    /*!
     * @brief This function returns a constant reference to member storage_usage
     * @return Constant reference to member storage_usage
     */
    eProsima_user_DllExport const onboard_computer_status__unsigned_long_array_4& storage_usage() const;

    /*!
     * @brief This function returns a reference to member storage_usage
     * @return Reference to member storage_usage
     */
    eProsima_user_DllExport onboard_computer_status__unsigned_long_array_4& storage_usage();
    /*!
     * @brief This function copies the value in member storage_total
     * @param _storage_total New value to be copied in member storage_total
     */
    eProsima_user_DllExport void storage_total(const onboard_computer_status__unsigned_long_array_4 &_storage_total);

    /*!
     * @brief This function moves the value in member storage_total
     * @param _storage_total New value to be moved in member storage_total
     */
    eProsima_user_DllExport void storage_total(onboard_computer_status__unsigned_long_array_4 &&_storage_total);

    /*!
     * @brief This function returns a constant reference to member storage_total
     * @return Constant reference to member storage_total
     */
    eProsima_user_DllExport const onboard_computer_status__unsigned_long_array_4& storage_total() const;

    /*!
     * @brief This function returns a reference to member storage_total
     * @return Reference to member storage_total
     */
    eProsima_user_DllExport onboard_computer_status__unsigned_long_array_4& storage_total();
    /*!
     * @brief This function copies the value in member link_type
     * @param _link_type New value to be copied in member link_type
     */
    eProsima_user_DllExport void link_type(const onboard_computer_status__unsigned_long_array_6 &_link_type);

    /*!
     * @brief This function moves the value in member link_type
     * @param _link_type New value to be moved in member link_type
     */
    eProsima_user_DllExport void link_type(onboard_computer_status__unsigned_long_array_6 &&_link_type);

    /*!
     * @brief This function returns a constant reference to member link_type
     * @return Constant reference to member link_type
     */
    eProsima_user_DllExport const onboard_computer_status__unsigned_long_array_6& link_type() const;

    /*!
     * @brief This function returns a reference to member link_type
     * @return Reference to member link_type
     */
    eProsima_user_DllExport onboard_computer_status__unsigned_long_array_6& link_type();
    /*!
     * @brief This function copies the value in member link_tx_rate
     * @param _link_tx_rate New value to be copied in member link_tx_rate
     */
    eProsima_user_DllExport void link_tx_rate(const onboard_computer_status__unsigned_long_array_6 &_link_tx_rate);

    /*!
     * @brief This function moves the value in member link_tx_rate
     * @param _link_tx_rate New value to be moved in member link_tx_rate
     */
    eProsima_user_DllExport void link_tx_rate(onboard_computer_status__unsigned_long_array_6 &&_link_tx_rate);

    /*!
     * @brief This function returns a constant reference to member link_tx_rate
     * @return Constant reference to member link_tx_rate
     */
    eProsima_user_DllExport const onboard_computer_status__unsigned_long_array_6& link_tx_rate() const;

    /*!
     * @brief This function returns a reference to member link_tx_rate
     * @return Reference to member link_tx_rate
     */
    eProsima_user_DllExport onboard_computer_status__unsigned_long_array_6& link_tx_rate();
    /*!
     * @brief This function copies the value in member link_rx_rate
     * @param _link_rx_rate New value to be copied in member link_rx_rate
     */
    eProsima_user_DllExport void link_rx_rate(const onboard_computer_status__unsigned_long_array_6 &_link_rx_rate);

    /*!
     * @brief This function moves the value in member link_rx_rate
     * @param _link_rx_rate New value to be moved in member link_rx_rate
     */
    eProsima_user_DllExport void link_rx_rate(onboard_computer_status__unsigned_long_array_6 &&_link_rx_rate);

    /*!
     * @brief This function returns a constant reference to member link_rx_rate
     * @return Constant reference to member link_rx_rate
     */
    eProsima_user_DllExport const onboard_computer_status__unsigned_long_array_6& link_rx_rate() const;

    /*!
     * @brief This function returns a reference to member link_rx_rate
     * @return Reference to member link_rx_rate
     */
    eProsima_user_DllExport onboard_computer_status__unsigned_long_array_6& link_rx_rate();
    /*!
     * @brief This function copies the value in member link_tx_max
     * @param _link_tx_max New value to be copied in member link_tx_max
     */
    eProsima_user_DllExport void link_tx_max(const onboard_computer_status__unsigned_long_array_6 &_link_tx_max);

    /*!
     * @brief This function moves the value in member link_tx_max
     * @param _link_tx_max New value to be moved in member link_tx_max
     */
    eProsima_user_DllExport void link_tx_max(onboard_computer_status__unsigned_long_array_6 &&_link_tx_max);

    /*!
     * @brief This function returns a constant reference to member link_tx_max
     * @return Constant reference to member link_tx_max
     */
    eProsima_user_DllExport const onboard_computer_status__unsigned_long_array_6& link_tx_max() const;

    /*!
     * @brief This function returns a reference to member link_tx_max
     * @return Reference to member link_tx_max
     */
    eProsima_user_DllExport onboard_computer_status__unsigned_long_array_6& link_tx_max();
    /*!
     * @brief This function copies the value in member link_rx_max
     * @param _link_rx_max New value to be copied in member link_rx_max
     */
    eProsima_user_DllExport void link_rx_max(const onboard_computer_status__unsigned_long_array_6 &_link_rx_max);

    /*!
     * @brief This function moves the value in member link_rx_max
     * @param _link_rx_max New value to be moved in member link_rx_max
     */
    eProsima_user_DllExport void link_rx_max(onboard_computer_status__unsigned_long_array_6 &&_link_rx_max);

    /*!
     * @brief This function returns a constant reference to member link_rx_max
     * @return Constant reference to member link_rx_max
     */
    eProsima_user_DllExport const onboard_computer_status__unsigned_long_array_6& link_rx_max() const;

    /*!
     * @brief This function returns a reference to member link_rx_max
     * @return Reference to member link_rx_max
     */
    eProsima_user_DllExport onboard_computer_status__unsigned_long_array_6& link_rx_max();

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
    eProsima_user_DllExport static size_t getCdrSerializedSize(const onboard_computer_status& data, size_t current_alignment = 0);


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
    uint32_t m_uptime_;
    uint8_t m_type_;
    onboard_computer_status__octet_array_8 m_cpu_cores;
    onboard_computer_status__octet_array_10 m_cpu_combined;
    onboard_computer_status__octet_array_4 m_gpu_cores;
    onboard_computer_status__octet_array_10 m_gpu_combined;
    uint8_t m_temperature_board_;
    onboard_computer_status__octet_array_8 m_temperature_core;
    onboard_computer_status__short_array_4 m_fan_speed;
    uint32_t m_ram_usage_;
    uint32_t m_ram_total_;
    onboard_computer_status__unsigned_long_array_4 m_storage_type;
    onboard_computer_status__unsigned_long_array_4 m_storage_usage;
    onboard_computer_status__unsigned_long_array_4 m_storage_total;
    onboard_computer_status__unsigned_long_array_6 m_link_type;
    onboard_computer_status__unsigned_long_array_6 m_link_tx_rate;
    onboard_computer_status__unsigned_long_array_6 m_link_rx_rate;
    onboard_computer_status__unsigned_long_array_6 m_link_tx_max;
    onboard_computer_status__unsigned_long_array_6 m_link_rx_max;
};

#endif // _ONBOARD_COMPUTER_STATUS_H_