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
 * @file ecat_valve.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _FAST_DDS_GENERATED_ECAT_VALVE_H_
#define _FAST_DDS_GENERATED_ECAT_VALVE_H_


#include <fastrtps/utils/fixed_size_string.hpp>

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
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define eProsima_user_DllExport
#endif  // _WIN32

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(ECAT_VALVE_SOURCE)
#define ECAT_VALVE_DllAPI __declspec( dllexport )
#else
#define ECAT_VALVE_DllAPI __declspec( dllimport )
#endif // ECAT_VALVE_SOURCE
#else
#define ECAT_VALVE_DllAPI
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define ECAT_VALVE_DllAPI
#endif // _WIN32

namespace eprosima {
namespace fastcdr {
class Cdr;
} // namespace fastcdr
} // namespace eprosima


/*!
 * @brief This class represents the structure ECValveMsg defined by the user in the IDL file.
 * @ingroup ecat_valve
 */
class ECValveMsg
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport ECValveMsg();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~ECValveMsg();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object ECValveMsg that will be copied.
     */
    eProsima_user_DllExport ECValveMsg(
            const ECValveMsg& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object ECValveMsg that will be copied.
     */
    eProsima_user_DllExport ECValveMsg(
            ECValveMsg&& x) noexcept;

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object ECValveMsg that will be copied.
     */
    eProsima_user_DllExport ECValveMsg& operator =(
            const ECValveMsg& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object ECValveMsg that will be copied.
     */
    eProsima_user_DllExport ECValveMsg& operator =(
            ECValveMsg&& x) noexcept;

    /*!
     * @brief Comparison operator.
     * @param x ECValveMsg object to compare.
     */
    eProsima_user_DllExport bool operator ==(
            const ECValveMsg& x) const;

    /*!
     * @brief Comparison operator.
     * @param x ECValveMsg object to compare.
     */
    eProsima_user_DllExport bool operator !=(
            const ECValveMsg& x) const;

    /*!
     * @brief This function sets a value in member timestamp
     * @param _timestamp New value for member timestamp
     */
    eProsima_user_DllExport void timestamp(
            double _timestamp);

    /*!
     * @brief This function returns the value of member timestamp
     * @return Value of member timestamp
     */
    eProsima_user_DllExport double timestamp() const;

    /*!
     * @brief This function returns a reference to member timestamp
     * @return Reference to member timestamp
     */
    eProsima_user_DllExport double& timestamp();

    /*!
     * @brief This function sets a value in member index
     * @param _index New value for member index
     */
    eProsima_user_DllExport void index(
            uint32_t _index);

    /*!
     * @brief This function returns the value of member index
     * @return Value of member index
     */
    eProsima_user_DllExport uint32_t index() const;

    /*!
     * @brief This function returns a reference to member index
     * @return Reference to member index
     */
    eProsima_user_DllExport uint32_t& index();

    /*!
     * @brief This function sets a value in member id
     * @param _id New value for member id
     */
    eProsima_user_DllExport void id(
            uint32_t _id);

    /*!
     * @brief This function returns the value of member id
     * @return Value of member id
     */
    eProsima_user_DllExport uint32_t id() const;

    /*!
     * @brief This function returns a reference to member id
     * @return Reference to member id
     */
    eProsima_user_DllExport uint32_t& id();

    /*!
     * @brief This function sets a value in member encoderPos
     * @param _encoderPos New value for member encoderPos
     */
    eProsima_user_DllExport void encoderPos(
            double _encoderPos);

    /*!
     * @brief This function returns the value of member encoderPos
     * @return Value of member encoderPos
     */
    eProsima_user_DllExport double encoderPos() const;

    /*!
     * @brief This function returns a reference to member encoderPos
     * @return Reference to member encoderPos
     */
    eProsima_user_DllExport double& encoderPos();

    /*!
     * @brief This function sets a value in member torque
     * @param _torque New value for member torque
     */
    eProsima_user_DllExport void torque(
            double _torque);

    /*!
     * @brief This function returns the value of member torque
     * @return Value of member torque
     */
    eProsima_user_DllExport double torque() const;

    /*!
     * @brief This function returns a reference to member torque
     * @return Reference to member torque
     */
    eProsima_user_DllExport double& torque();

    /*!
     * @brief This function sets a value in member pressure1
     * @param _pressure1 New value for member pressure1
     */
    eProsima_user_DllExport void pressure1(
            double _pressure1);

    /*!
     * @brief This function returns the value of member pressure1
     * @return Value of member pressure1
     */
    eProsima_user_DllExport double pressure1() const;

    /*!
     * @brief This function returns a reference to member pressure1
     * @return Reference to member pressure1
     */
    eProsima_user_DllExport double& pressure1();

    /*!
     * @brief This function sets a value in member pressure2
     * @param _pressure2 New value for member pressure2
     */
    eProsima_user_DllExport void pressure2(
            double _pressure2);

    /*!
     * @brief This function returns the value of member pressure2
     * @return Value of member pressure2
     */
    eProsima_user_DllExport double pressure2() const;

    /*!
     * @brief This function returns a reference to member pressure2
     * @return Reference to member pressure2
     */
    eProsima_user_DllExport double& pressure2();

    /*!
     * @brief This function sets a value in member temp
     * @param _temp New value for member temp
     */
    eProsima_user_DllExport void temp(
            double _temp);

    /*!
     * @brief This function returns the value of member temp
     * @return Value of member temp
     */
    eProsima_user_DllExport double temp() const;

    /*!
     * @brief This function returns a reference to member temp
     * @return Reference to member temp
     */
    eProsima_user_DllExport double& temp();

    /*!
     * @brief This function sets a value in member kp
     * @param _kp New value for member kp
     */
    eProsima_user_DllExport void kp(
            double _kp);

    /*!
     * @brief This function returns the value of member kp
     * @return Value of member kp
     */
    eProsima_user_DllExport double kp() const;

    /*!
     * @brief This function returns a reference to member kp
     * @return Reference to member kp
     */
    eProsima_user_DllExport double& kp();

    /*!
     * @brief This function sets a value in member ki
     * @param _ki New value for member ki
     */
    eProsima_user_DllExport void ki(
            double _ki);

    /*!
     * @brief This function returns the value of member ki
     * @return Value of member ki
     */
    eProsima_user_DllExport double ki() const;

    /*!
     * @brief This function returns a reference to member ki
     * @return Reference to member ki
     */
    eProsima_user_DllExport double& ki();

    /*!
     * @brief This function sets a value in member kd
     * @param _kd New value for member kd
     */
    eProsima_user_DllExport void kd(
            double _kd);

    /*!
     * @brief This function returns the value of member kd
     * @return Value of member kd
     */
    eProsima_user_DllExport double kd() const;

    /*!
     * @brief This function returns a reference to member kd
     * @return Reference to member kd
     */
    eProsima_user_DllExport double& kd();

    /*!
     * @brief This function sets a value in member current_ref
     * @param _current_ref New value for member current_ref
     */
    eProsima_user_DllExport void current_ref(
            double _current_ref);

    /*!
     * @brief This function returns the value of member current_ref
     * @return Value of member current_ref
     */
    eProsima_user_DllExport double current_ref() const;

    /*!
     * @brief This function returns a reference to member current_ref
     * @return Reference to member current_ref
     */
    eProsima_user_DllExport double& current_ref();

    /*!
     * @brief This function sets a value in member position_ref
     * @param _position_ref New value for member position_ref
     */
    eProsima_user_DllExport void position_ref(
            double _position_ref);

    /*!
     * @brief This function returns the value of member position_ref
     * @return Value of member position_ref
     */
    eProsima_user_DllExport double position_ref() const;

    /*!
     * @brief This function returns a reference to member position_ref
     * @return Reference to member position_ref
     */
    eProsima_user_DllExport double& position_ref();

    /*!
     * @brief This function sets a value in member torque_ref
     * @param _torque_ref New value for member torque_ref
     */
    eProsima_user_DllExport void torque_ref(
            double _torque_ref);

    /*!
     * @brief This function returns the value of member torque_ref
     * @return Value of member torque_ref
     */
    eProsima_user_DllExport double torque_ref() const;

    /*!
     * @brief This function returns a reference to member torque_ref
     * @return Reference to member torque_ref
     */
    eProsima_user_DllExport double& torque_ref();

    /*!
     * @brief This function sets a value in member current_offset
     * @param _current_offset New value for member current_offset
     */
    eProsima_user_DllExport void current_offset(
            double _current_offset);

    /*!
     * @brief This function returns the value of member current_offset
     * @return Value of member current_offset
     */
    eProsima_user_DllExport double current_offset() const;

    /*!
     * @brief This function returns a reference to member current_offset
     * @return Reference to member current_offset
     */
    eProsima_user_DllExport double& current_offset();


    /*!
    * @brief This function returns the maximum serialized size of an object
    * depending on the buffer alignment.
    * @param current_alignment Buffer alignment.
    * @return Maximum serialized size.
    */
    eProsima_user_DllExport static size_t getMaxCdrSerializedSize(
            size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    eProsima_user_DllExport static size_t getCdrSerializedSize(
            const ECValveMsg& data,
            size_t current_alignment = 0);


    /*!
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serialize(
            eprosima::fastcdr::Cdr& cdr) const;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void deserialize(
            eprosima::fastcdr::Cdr& cdr);



    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(
            size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    eProsima_user_DllExport static bool isKeyDefined();

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serializeKey(
            eprosima::fastcdr::Cdr& cdr) const;

private:

    double m_timestamp;
    uint32_t m_index;
    uint32_t m_id;
    double m_encoderPos;
    double m_torque;
    double m_pressure1;
    double m_pressure2;
    double m_temp;
    double m_kp;
    double m_ki;
    double m_kd;
    double m_current_ref;
    double m_position_ref;
    double m_torque_ref;
    double m_current_offset;

};

#endif // _FAST_DDS_GENERATED_ECAT_VALVE_H_

