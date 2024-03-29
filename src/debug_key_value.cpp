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
 * @file debug_key_value.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "debug_key_value.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>


debug_key_value::debug_key_value()
{
    // m_timestamp_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@7cbd213e
    m_timestamp_ = 0;
    // m_key com.eprosima.idl.parser.typecode.AliasTypeCode@192d3247
    memset(&m_key, 0, (10) * 1);
    // m_value_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@3ecd23d9
    m_value_ = 0.0;

}

debug_key_value::~debug_key_value()
{



}

debug_key_value::debug_key_value(const debug_key_value &x)
{
    m_timestamp_ = x.m_timestamp_;
    m_key = x.m_key;
    m_value_ = x.m_value_;
}

debug_key_value::debug_key_value(debug_key_value &&x)
{
    m_timestamp_ = x.m_timestamp_;
    m_key = std::move(x.m_key);
    m_value_ = x.m_value_;
}

debug_key_value& debug_key_value::operator=(const debug_key_value &x)
{

    m_timestamp_ = x.m_timestamp_;
    m_key = x.m_key;
    m_value_ = x.m_value_;

    return *this;
}

debug_key_value& debug_key_value::operator=(debug_key_value &&x)
{

    m_timestamp_ = x.m_timestamp_;
    m_key = std::move(x.m_key);
    m_value_ = x.m_value_;

    return *this;
}

size_t debug_key_value::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += ((10) * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

size_t debug_key_value::getCdrSerializedSize(const debug_key_value& data, size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    if ((10) > 0)
    {
        current_alignment += ((10) * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);
    }

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

void debug_key_value::serialize(eprosima::fastcdr::Cdr &scdr) const
{

    scdr << m_timestamp_;
    scdr << m_key;

    scdr << m_value_;
}

void debug_key_value::deserialize(eprosima::fastcdr::Cdr &dcdr)
{

    dcdr >> m_timestamp_;
    dcdr >> m_key;

    dcdr >> m_value_;
}

/*!
 * @brief This function sets a value in member timestamp_
 * @param _timestamp_ New value for member timestamp_
 */
void debug_key_value::timestamp_(uint64_t _timestamp_)
{
m_timestamp_ = _timestamp_;
}

/*!
 * @brief This function returns the value of member timestamp_
 * @return Value of member timestamp_
 */
uint64_t debug_key_value::timestamp_() const
{
    return m_timestamp_;
}

/*!
 * @brief This function returns a reference to member timestamp_
 * @return Reference to member timestamp_
 */
uint64_t& debug_key_value::timestamp_()
{
    return m_timestamp_;
}

/*!
 * @brief This function copies the value in member key
 * @param _key New value to be copied in member key
 */
void debug_key_value::key(const debug_key_value__char_array_10 &_key)
{
m_key = _key;
}

/*!
 * @brief This function moves the value in member key
 * @param _key New value to be moved in member key
 */
void debug_key_value::key(debug_key_value__char_array_10 &&_key)
{
m_key = std::move(_key);
}

/*!
 * @brief This function returns a constant reference to member key
 * @return Constant reference to member key
 */
const debug_key_value__char_array_10& debug_key_value::key() const
{
    return m_key;
}

/*!
 * @brief This function returns a reference to member key
 * @return Reference to member key
 */
debug_key_value__char_array_10& debug_key_value::key()
{
    return m_key;
}
/*!
 * @brief This function sets a value in member value_
 * @param _value_ New value for member value_
 */
void debug_key_value::value_(float _value_)
{
m_value_ = _value_;
}

/*!
 * @brief This function returns the value of member value_
 * @return Value of member value_
 */
float debug_key_value::value_() const
{
    return m_value_;
}

/*!
 * @brief This function returns a reference to member value_
 * @return Reference to member value_
 */
float& debug_key_value::value_()
{
    return m_value_;
}


size_t debug_key_value::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    size_t current_align = current_alignment;






    return current_align;
}

bool debug_key_value::isKeyDefined()
{
   return false;
}

void debug_key_value::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
    (void) scdr;
     
     
     
}
