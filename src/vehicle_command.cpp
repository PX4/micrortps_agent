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
 * @file vehicle_command.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "vehicle_command.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>







































































































































vehicle_command::vehicle_command()
{
    // m_timestamp_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@627551fb
    m_timestamp_ = 0;
    // m_param1_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@1f36e637
    m_param1_ = 0.0;
    // m_param2_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@578486a3
    m_param2_ = 0.0;
    // m_param3_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@551aa95a
    m_param3_ = 0.0;
    // m_param4_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@35d176f7
    m_param4_ = 0.0;
    // m_param5_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@1dfe2924
    m_param5_ = 0.0;
    // m_param6_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@6ebc05a6
    m_param6_ = 0.0;
    // m_param7_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@6e6c3152
    m_param7_ = 0.0;
    // m_command_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@50b494a6
    m_command_ = 0;
    // m_target_system_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@3cef309d
    m_target_system_ = 0;
    // m_target_component_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@32709393
    m_target_component_ = 0;
    // m_source_system_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@3d99d22e
    m_source_system_ = 0;
    // m_source_component_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@49fc609f
    m_source_component_ = 0;
    // m_confirmation_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@cd2dae5
    m_confirmation_ = 0;
    // m_from_external_ com.eprosima.idl.parser.typecode.PrimitiveTypeCode@3a883ce7
    m_from_external_ = false;

}

vehicle_command::~vehicle_command()
{















}

vehicle_command::vehicle_command(const vehicle_command &x)
{
    m_timestamp_ = x.m_timestamp_;
    m_param1_ = x.m_param1_;
    m_param2_ = x.m_param2_;
    m_param3_ = x.m_param3_;
    m_param4_ = x.m_param4_;
    m_param5_ = x.m_param5_;
    m_param6_ = x.m_param6_;
    m_param7_ = x.m_param7_;
    m_command_ = x.m_command_;
    m_target_system_ = x.m_target_system_;
    m_target_component_ = x.m_target_component_;
    m_source_system_ = x.m_source_system_;
    m_source_component_ = x.m_source_component_;
    m_confirmation_ = x.m_confirmation_;
    m_from_external_ = x.m_from_external_;
}

vehicle_command::vehicle_command(vehicle_command &&x)
{
    m_timestamp_ = x.m_timestamp_;
    m_param1_ = x.m_param1_;
    m_param2_ = x.m_param2_;
    m_param3_ = x.m_param3_;
    m_param4_ = x.m_param4_;
    m_param5_ = x.m_param5_;
    m_param6_ = x.m_param6_;
    m_param7_ = x.m_param7_;
    m_command_ = x.m_command_;
    m_target_system_ = x.m_target_system_;
    m_target_component_ = x.m_target_component_;
    m_source_system_ = x.m_source_system_;
    m_source_component_ = x.m_source_component_;
    m_confirmation_ = x.m_confirmation_;
    m_from_external_ = x.m_from_external_;
}

vehicle_command& vehicle_command::operator=(const vehicle_command &x)
{

    m_timestamp_ = x.m_timestamp_;
    m_param1_ = x.m_param1_;
    m_param2_ = x.m_param2_;
    m_param3_ = x.m_param3_;
    m_param4_ = x.m_param4_;
    m_param5_ = x.m_param5_;
    m_param6_ = x.m_param6_;
    m_param7_ = x.m_param7_;
    m_command_ = x.m_command_;
    m_target_system_ = x.m_target_system_;
    m_target_component_ = x.m_target_component_;
    m_source_system_ = x.m_source_system_;
    m_source_component_ = x.m_source_component_;
    m_confirmation_ = x.m_confirmation_;
    m_from_external_ = x.m_from_external_;

    return *this;
}

vehicle_command& vehicle_command::operator=(vehicle_command &&x)
{

    m_timestamp_ = x.m_timestamp_;
    m_param1_ = x.m_param1_;
    m_param2_ = x.m_param2_;
    m_param3_ = x.m_param3_;
    m_param4_ = x.m_param4_;
    m_param5_ = x.m_param5_;
    m_param6_ = x.m_param6_;
    m_param7_ = x.m_param7_;
    m_command_ = x.m_command_;
    m_target_system_ = x.m_target_system_;
    m_target_component_ = x.m_target_component_;
    m_source_system_ = x.m_source_system_;
    m_source_component_ = x.m_source_component_;
    m_confirmation_ = x.m_confirmation_;
    m_from_external_ = x.m_from_external_;

    return *this;
}

size_t vehicle_command::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    return current_alignment - initial_alignment;
}

size_t vehicle_command::getCdrSerializedSize(const vehicle_command& data, size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 1 + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);



    return current_alignment - initial_alignment;
}

void vehicle_command::serialize(eprosima::fastcdr::Cdr &scdr) const
{

    scdr << m_timestamp_;
    scdr << m_param1_;
    scdr << m_param2_;
    scdr << m_param3_;
    scdr << m_param4_;
    scdr << m_param5_;
    scdr << m_param6_;
    scdr << m_param7_;
    scdr << m_command_;
    scdr << m_target_system_;
    scdr << m_target_component_;
    scdr << m_source_system_;
    scdr << m_source_component_;
    scdr << m_confirmation_;
    scdr << m_from_external_;
}

void vehicle_command::deserialize(eprosima::fastcdr::Cdr &dcdr)
{

    dcdr >> m_timestamp_;
    dcdr >> m_param1_;
    dcdr >> m_param2_;
    dcdr >> m_param3_;
    dcdr >> m_param4_;
    dcdr >> m_param5_;
    dcdr >> m_param6_;
    dcdr >> m_param7_;
    dcdr >> m_command_;
    dcdr >> m_target_system_;
    dcdr >> m_target_component_;
    dcdr >> m_source_system_;
    dcdr >> m_source_component_;
    dcdr >> m_confirmation_;
    dcdr >> m_from_external_;
}

/*!
 * @brief This function sets a value in member timestamp_
 * @param _timestamp_ New value for member timestamp_
 */
void vehicle_command::timestamp_(uint64_t _timestamp_)
{
m_timestamp_ = _timestamp_;
}

/*!
 * @brief This function returns the value of member timestamp_
 * @return Value of member timestamp_
 */
uint64_t vehicle_command::timestamp_() const
{
    return m_timestamp_;
}

/*!
 * @brief This function returns a reference to member timestamp_
 * @return Reference to member timestamp_
 */
uint64_t& vehicle_command::timestamp_()
{
    return m_timestamp_;
}

/*!
 * @brief This function sets a value in member param1_
 * @param _param1_ New value for member param1_
 */
void vehicle_command::param1_(float _param1_)
{
m_param1_ = _param1_;
}

/*!
 * @brief This function returns the value of member param1_
 * @return Value of member param1_
 */
float vehicle_command::param1_() const
{
    return m_param1_;
}

/*!
 * @brief This function returns a reference to member param1_
 * @return Reference to member param1_
 */
float& vehicle_command::param1_()
{
    return m_param1_;
}

/*!
 * @brief This function sets a value in member param2_
 * @param _param2_ New value for member param2_
 */
void vehicle_command::param2_(float _param2_)
{
m_param2_ = _param2_;
}

/*!
 * @brief This function returns the value of member param2_
 * @return Value of member param2_
 */
float vehicle_command::param2_() const
{
    return m_param2_;
}

/*!
 * @brief This function returns a reference to member param2_
 * @return Reference to member param2_
 */
float& vehicle_command::param2_()
{
    return m_param2_;
}

/*!
 * @brief This function sets a value in member param3_
 * @param _param3_ New value for member param3_
 */
void vehicle_command::param3_(float _param3_)
{
m_param3_ = _param3_;
}

/*!
 * @brief This function returns the value of member param3_
 * @return Value of member param3_
 */
float vehicle_command::param3_() const
{
    return m_param3_;
}

/*!
 * @brief This function returns a reference to member param3_
 * @return Reference to member param3_
 */
float& vehicle_command::param3_()
{
    return m_param3_;
}

/*!
 * @brief This function sets a value in member param4_
 * @param _param4_ New value for member param4_
 */
void vehicle_command::param4_(float _param4_)
{
m_param4_ = _param4_;
}

/*!
 * @brief This function returns the value of member param4_
 * @return Value of member param4_
 */
float vehicle_command::param4_() const
{
    return m_param4_;
}

/*!
 * @brief This function returns a reference to member param4_
 * @return Reference to member param4_
 */
float& vehicle_command::param4_()
{
    return m_param4_;
}

/*!
 * @brief This function sets a value in member param5_
 * @param _param5_ New value for member param5_
 */
void vehicle_command::param5_(double _param5_)
{
m_param5_ = _param5_;
}

/*!
 * @brief This function returns the value of member param5_
 * @return Value of member param5_
 */
double vehicle_command::param5_() const
{
    return m_param5_;
}

/*!
 * @brief This function returns a reference to member param5_
 * @return Reference to member param5_
 */
double& vehicle_command::param5_()
{
    return m_param5_;
}

/*!
 * @brief This function sets a value in member param6_
 * @param _param6_ New value for member param6_
 */
void vehicle_command::param6_(double _param6_)
{
m_param6_ = _param6_;
}

/*!
 * @brief This function returns the value of member param6_
 * @return Value of member param6_
 */
double vehicle_command::param6_() const
{
    return m_param6_;
}

/*!
 * @brief This function returns a reference to member param6_
 * @return Reference to member param6_
 */
double& vehicle_command::param6_()
{
    return m_param6_;
}

/*!
 * @brief This function sets a value in member param7_
 * @param _param7_ New value for member param7_
 */
void vehicle_command::param7_(float _param7_)
{
m_param7_ = _param7_;
}

/*!
 * @brief This function returns the value of member param7_
 * @return Value of member param7_
 */
float vehicle_command::param7_() const
{
    return m_param7_;
}

/*!
 * @brief This function returns a reference to member param7_
 * @return Reference to member param7_
 */
float& vehicle_command::param7_()
{
    return m_param7_;
}

/*!
 * @brief This function sets a value in member command_
 * @param _command_ New value for member command_
 */
void vehicle_command::command_(uint32_t _command_)
{
m_command_ = _command_;
}

/*!
 * @brief This function returns the value of member command_
 * @return Value of member command_
 */
uint32_t vehicle_command::command_() const
{
    return m_command_;
}

/*!
 * @brief This function returns a reference to member command_
 * @return Reference to member command_
 */
uint32_t& vehicle_command::command_()
{
    return m_command_;
}

/*!
 * @brief This function sets a value in member target_system_
 * @param _target_system_ New value for member target_system_
 */
void vehicle_command::target_system_(uint8_t _target_system_)
{
m_target_system_ = _target_system_;
}

/*!
 * @brief This function returns the value of member target_system_
 * @return Value of member target_system_
 */
uint8_t vehicle_command::target_system_() const
{
    return m_target_system_;
}

/*!
 * @brief This function returns a reference to member target_system_
 * @return Reference to member target_system_
 */
uint8_t& vehicle_command::target_system_()
{
    return m_target_system_;
}

/*!
 * @brief This function sets a value in member target_component_
 * @param _target_component_ New value for member target_component_
 */
void vehicle_command::target_component_(uint8_t _target_component_)
{
m_target_component_ = _target_component_;
}

/*!
 * @brief This function returns the value of member target_component_
 * @return Value of member target_component_
 */
uint8_t vehicle_command::target_component_() const
{
    return m_target_component_;
}

/*!
 * @brief This function returns a reference to member target_component_
 * @return Reference to member target_component_
 */
uint8_t& vehicle_command::target_component_()
{
    return m_target_component_;
}

/*!
 * @brief This function sets a value in member source_system_
 * @param _source_system_ New value for member source_system_
 */
void vehicle_command::source_system_(uint8_t _source_system_)
{
m_source_system_ = _source_system_;
}

/*!
 * @brief This function returns the value of member source_system_
 * @return Value of member source_system_
 */
uint8_t vehicle_command::source_system_() const
{
    return m_source_system_;
}

/*!
 * @brief This function returns a reference to member source_system_
 * @return Reference to member source_system_
 */
uint8_t& vehicle_command::source_system_()
{
    return m_source_system_;
}

/*!
 * @brief This function sets a value in member source_component_
 * @param _source_component_ New value for member source_component_
 */
void vehicle_command::source_component_(uint8_t _source_component_)
{
m_source_component_ = _source_component_;
}

/*!
 * @brief This function returns the value of member source_component_
 * @return Value of member source_component_
 */
uint8_t vehicle_command::source_component_() const
{
    return m_source_component_;
}

/*!
 * @brief This function returns a reference to member source_component_
 * @return Reference to member source_component_
 */
uint8_t& vehicle_command::source_component_()
{
    return m_source_component_;
}

/*!
 * @brief This function sets a value in member confirmation_
 * @param _confirmation_ New value for member confirmation_
 */
void vehicle_command::confirmation_(uint8_t _confirmation_)
{
m_confirmation_ = _confirmation_;
}

/*!
 * @brief This function returns the value of member confirmation_
 * @return Value of member confirmation_
 */
uint8_t vehicle_command::confirmation_() const
{
    return m_confirmation_;
}

/*!
 * @brief This function returns a reference to member confirmation_
 * @return Reference to member confirmation_
 */
uint8_t& vehicle_command::confirmation_()
{
    return m_confirmation_;
}

/*!
 * @brief This function sets a value in member from_external_
 * @param _from_external_ New value for member from_external_
 */
void vehicle_command::from_external_(bool _from_external_)
{
m_from_external_ = _from_external_;
}

/*!
 * @brief This function returns the value of member from_external_
 * @return Value of member from_external_
 */
bool vehicle_command::from_external_() const
{
    return m_from_external_;
}

/*!
 * @brief This function returns a reference to member from_external_
 * @return Reference to member from_external_
 */
bool& vehicle_command::from_external_()
{
    return m_from_external_;
}


size_t vehicle_command::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
    size_t current_align = current_alignment;


















    return current_align;
}

bool vehicle_command::isKeyDefined()
{
   return false;
}

void vehicle_command::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
    (void) scdr;
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
}
