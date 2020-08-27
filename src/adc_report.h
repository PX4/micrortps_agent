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
 * @file adc_report.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _ADC_REPORT_H_
#define _ADC_REPORT_H_

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
#if defined(adc_report_SOURCE)
#define adc_report_DllAPI __declspec( dllexport )
#else
#define adc_report_DllAPI __declspec( dllimport )
#endif // adc_report_SOURCE
#else
#define adc_report_DllAPI
#endif
#else
#define adc_report_DllAPI
#endif // _WIN32

namespace eprosima
{
    namespace fastcdr
    {
        class Cdr;
    }
}


typedef std::array<int16_t, 12> adc_report__short_array_12;
typedef std::array<int32_t, 12> adc_report__long_array_12;
/*!
 * @brief This class represents the structure adc_report defined by the user in the IDL file.
 * @ingroup ADC_REPORT
 */
class adc_report
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport adc_report();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~adc_report();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object adc_report that will be copied.
     */
    eProsima_user_DllExport adc_report(const adc_report &x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object adc_report that will be copied.
     */
    eProsima_user_DllExport adc_report(adc_report &&x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object adc_report that will be copied.
     */
    eProsima_user_DllExport adc_report& operator=(const adc_report &x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object adc_report that will be copied.
     */
    eProsima_user_DllExport adc_report& operator=(adc_report &&x);

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
     * @brief This function sets a value in member device_id_
     * @param _device_id_ New value for member device_id_
     */
    eProsima_user_DllExport void device_id_(uint32_t _device_id_);

    /*!
     * @brief This function returns the value of member device_id_
     * @return Value of member device_id_
     */
    eProsima_user_DllExport uint32_t device_id_() const;

    /*!
     * @brief This function returns a reference to member device_id_
     * @return Reference to member device_id_
     */
    eProsima_user_DllExport uint32_t& device_id_();

    /*!
     * @brief This function copies the value in member channel_id
     * @param _channel_id New value to be copied in member channel_id
     */
    eProsima_user_DllExport void channel_id(const adc_report__short_array_12 &_channel_id);

    /*!
     * @brief This function moves the value in member channel_id
     * @param _channel_id New value to be moved in member channel_id
     */
    eProsima_user_DllExport void channel_id(adc_report__short_array_12 &&_channel_id);

    /*!
     * @brief This function returns a constant reference to member channel_id
     * @return Constant reference to member channel_id
     */
    eProsima_user_DllExport const adc_report__short_array_12& channel_id() const;

    /*!
     * @brief This function returns a reference to member channel_id
     * @return Reference to member channel_id
     */
    eProsima_user_DllExport adc_report__short_array_12& channel_id();
    /*!
     * @brief This function copies the value in member raw_data
     * @param _raw_data New value to be copied in member raw_data
     */
    eProsima_user_DllExport void raw_data(const adc_report__long_array_12 &_raw_data);

    /*!
     * @brief This function moves the value in member raw_data
     * @param _raw_data New value to be moved in member raw_data
     */
    eProsima_user_DllExport void raw_data(adc_report__long_array_12 &&_raw_data);

    /*!
     * @brief This function returns a constant reference to member raw_data
     * @return Constant reference to member raw_data
     */
    eProsima_user_DllExport const adc_report__long_array_12& raw_data() const;

    /*!
     * @brief This function returns a reference to member raw_data
     * @return Reference to member raw_data
     */
    eProsima_user_DllExport adc_report__long_array_12& raw_data();
    /*!
     * @brief This function sets a value in member resolution_
     * @param _resolution_ New value for member resolution_
     */
    eProsima_user_DllExport void resolution_(uint32_t _resolution_);

    /*!
     * @brief This function returns the value of member resolution_
     * @return Value of member resolution_
     */
    eProsima_user_DllExport uint32_t resolution_() const;

    /*!
     * @brief This function returns a reference to member resolution_
     * @return Reference to member resolution_
     */
    eProsima_user_DllExport uint32_t& resolution_();

    /*!
     * @brief This function sets a value in member v_ref_
     * @param _v_ref_ New value for member v_ref_
     */
    eProsima_user_DllExport void v_ref_(float _v_ref_);

    /*!
     * @brief This function returns the value of member v_ref_
     * @return Value of member v_ref_
     */
    eProsima_user_DllExport float v_ref_() const;

    /*!
     * @brief This function returns a reference to member v_ref_
     * @return Reference to member v_ref_
     */
    eProsima_user_DllExport float& v_ref_();


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
    eProsima_user_DllExport static size_t getCdrSerializedSize(const adc_report& data, size_t current_alignment = 0);


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
    uint32_t m_device_id_;
    adc_report__short_array_12 m_channel_id;
    adc_report__long_array_12 m_raw_data;
    uint32_t m_resolution_;
    float m_v_ref_;
};

#endif // _ADC_REPORT_H_