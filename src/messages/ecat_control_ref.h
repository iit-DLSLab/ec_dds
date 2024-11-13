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
 * @file ecat_control_ref.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _FAST_DDS_GENERATED_ECAT_CONTROL_REF_H_
#define _FAST_DDS_GENERATED_ECAT_CONTROL_REF_H_


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
#if defined(ECAT_CONTROL_REF_SOURCE)
#define ECAT_CONTROL_REF_DllAPI __declspec( dllexport )
#else
#define ECAT_CONTROL_REF_DllAPI __declspec( dllimport )
#endif // ECAT_CONTROL_REF_SOURCE
#else
#define ECAT_CONTROL_REF_DllAPI
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define ECAT_CONTROL_REF_DllAPI
#endif // _WIN32

namespace eprosima {
namespace fastcdr {
class Cdr;
} // namespace fastcdr
} // namespace eprosima


/*!
 * @brief This class represents the structure EcatControlRefMsg defined by the user in the IDL file.
 * @ingroup ecat_control_ref
 */
class EcatControlRefMsg
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport EcatControlRefMsg();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~EcatControlRefMsg();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object EcatControlRefMsg that will be copied.
     */
    eProsima_user_DllExport EcatControlRefMsg(
            const EcatControlRefMsg& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object EcatControlRefMsg that will be copied.
     */
    eProsima_user_DllExport EcatControlRefMsg(
            EcatControlRefMsg&& x) noexcept;

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object EcatControlRefMsg that will be copied.
     */
    eProsima_user_DllExport EcatControlRefMsg& operator =(
            const EcatControlRefMsg& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object EcatControlRefMsg that will be copied.
     */
    eProsima_user_DllExport EcatControlRefMsg& operator =(
            EcatControlRefMsg&& x) noexcept;

    /*!
     * @brief Comparison operator.
     * @param x EcatControlRefMsg object to compare.
     */
    eProsima_user_DllExport bool operator ==(
            const EcatControlRefMsg& x) const;

    /*!
     * @brief Comparison operator.
     * @param x EcatControlRefMsg object to compare.
     */
    eProsima_user_DllExport bool operator !=(
            const EcatControlRefMsg& x) const;

    /*!
     * @brief This function copies the value in member frame_id
     * @param _frame_id New value to be copied in member frame_id
     */
    eProsima_user_DllExport void frame_id(
            const std::string& _frame_id);

    /*!
     * @brief This function moves the value in member frame_id
     * @param _frame_id New value to be moved in member frame_id
     */
    eProsima_user_DllExport void frame_id(
            std::string&& _frame_id);

    /*!
     * @brief This function returns a constant reference to member frame_id
     * @return Constant reference to member frame_id
     */
    eProsima_user_DllExport const std::string& frame_id() const;

    /*!
     * @brief This function returns a reference to member frame_id
     * @return Reference to member frame_id
     */
    eProsima_user_DllExport std::string& frame_id();
    /*!
     * @brief This function sets a value in member sequence_id
     * @param _sequence_id New value for member sequence_id
     */
    eProsima_user_DllExport void sequence_id(
            uint32_t _sequence_id);

    /*!
     * @brief This function returns the value of member sequence_id
     * @return Value of member sequence_id
     */
    eProsima_user_DllExport uint32_t sequence_id() const;

    /*!
     * @brief This function returns a reference to member sequence_id
     * @return Reference to member sequence_id
     */
    eProsima_user_DllExport uint32_t& sequence_id();

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
     * @brief This function copies the value in member amplitude
     * @param _amplitude New value to be copied in member amplitude
     */
    eProsima_user_DllExport void amplitude(
            const std::array<double, 12>& _amplitude);

    /*!
     * @brief This function moves the value in member amplitude
     * @param _amplitude New value to be moved in member amplitude
     */
    eProsima_user_DllExport void amplitude(
            std::array<double, 12>&& _amplitude);

    /*!
     * @brief This function returns a constant reference to member amplitude
     * @return Constant reference to member amplitude
     */
    eProsima_user_DllExport const std::array<double, 12>& amplitude() const;

    /*!
     * @brief This function returns a reference to member amplitude
     * @return Reference to member amplitude
     */
    eProsima_user_DllExport std::array<double, 12>& amplitude();
    /*!
     * @brief This function copies the value in member frequency
     * @param _frequency New value to be copied in member frequency
     */
    eProsima_user_DllExport void frequency(
            const std::array<double, 12>& _frequency);

    /*!
     * @brief This function moves the value in member frequency
     * @param _frequency New value to be moved in member frequency
     */
    eProsima_user_DllExport void frequency(
            std::array<double, 12>&& _frequency);

    /*!
     * @brief This function returns a constant reference to member frequency
     * @return Constant reference to member frequency
     */
    eProsima_user_DllExport const std::array<double, 12>& frequency() const;

    /*!
     * @brief This function returns a reference to member frequency
     * @return Reference to member frequency
     */
    eProsima_user_DllExport std::array<double, 12>& frequency();
    /*!
     * @brief This function copies the value in member offset
     * @param _offset New value to be copied in member offset
     */
    eProsima_user_DllExport void offset(
            const std::array<double, 12>& _offset);

    /*!
     * @brief This function moves the value in member offset
     * @param _offset New value to be moved in member offset
     */
    eProsima_user_DllExport void offset(
            std::array<double, 12>&& _offset);

    /*!
     * @brief This function returns a constant reference to member offset
     * @return Constant reference to member offset
     */
    eProsima_user_DllExport const std::array<double, 12>& offset() const;

    /*!
     * @brief This function returns a reference to member offset
     * @return Reference to member offset
     */
    eProsima_user_DllExport std::array<double, 12>& offset();
    /*!
     * @brief This function copies the value in member traj_type
     * @param _traj_type New value to be copied in member traj_type
     */
    eProsima_user_DllExport void traj_type(
            const std::array<std::string, 12>& _traj_type);

    /*!
     * @brief This function moves the value in member traj_type
     * @param _traj_type New value to be moved in member traj_type
     */
    eProsima_user_DllExport void traj_type(
            std::array<std::string, 12>&& _traj_type);

    /*!
     * @brief This function returns a constant reference to member traj_type
     * @return Constant reference to member traj_type
     */
    eProsima_user_DllExport const std::array<std::string, 12>& traj_type() const;

    /*!
     * @brief This function returns a reference to member traj_type
     * @return Reference to member traj_type
     */
    eProsima_user_DllExport std::array<std::string, 12>& traj_type();
    /*!
     * @brief This function copies the value in member traj_mode
     * @param _traj_mode New value to be copied in member traj_mode
     */
    eProsima_user_DllExport void traj_mode(
            const std::array<std::string, 12>& _traj_mode);

    /*!
     * @brief This function moves the value in member traj_mode
     * @param _traj_mode New value to be moved in member traj_mode
     */
    eProsima_user_DllExport void traj_mode(
            std::array<std::string, 12>&& _traj_mode);

    /*!
     * @brief This function returns a constant reference to member traj_mode
     * @return Constant reference to member traj_mode
     */
    eProsima_user_DllExport const std::array<std::string, 12>& traj_mode() const;

    /*!
     * @brief This function returns a reference to member traj_mode
     * @return Reference to member traj_mode
     */
    eProsima_user_DllExport std::array<std::string, 12>& traj_mode();
    /*!
     * @brief This function copies the value in member start_traj
     * @param _start_traj New value to be copied in member start_traj
     */
    eProsima_user_DllExport void start_traj(
            const std::array<bool, 12>& _start_traj);

    /*!
     * @brief This function moves the value in member start_traj
     * @param _start_traj New value to be moved in member start_traj
     */
    eProsima_user_DllExport void start_traj(
            std::array<bool, 12>&& _start_traj);

    /*!
     * @brief This function returns a constant reference to member start_traj
     * @return Constant reference to member start_traj
     */
    eProsima_user_DllExport const std::array<bool, 12>& start_traj() const;

    /*!
     * @brief This function returns a reference to member start_traj
     * @return Reference to member start_traj
     */
    eProsima_user_DllExport std::array<bool, 12>& start_traj();
    /*!
     * @brief This function copies the value in member max_freq
     * @param _max_freq New value to be copied in member max_freq
     */
    eProsima_user_DllExport void max_freq(
            const std::array<double, 12>& _max_freq);

    /*!
     * @brief This function moves the value in member max_freq
     * @param _max_freq New value to be moved in member max_freq
     */
    eProsima_user_DllExport void max_freq(
            std::array<double, 12>&& _max_freq);

    /*!
     * @brief This function returns a constant reference to member max_freq
     * @return Constant reference to member max_freq
     */
    eProsima_user_DllExport const std::array<double, 12>& max_freq() const;

    /*!
     * @brief This function returns a reference to member max_freq
     * @return Reference to member max_freq
     */
    eProsima_user_DllExport std::array<double, 12>& max_freq();
    /*!
     * @brief This function copies the value in member torque_scale_factor
     * @param _torque_scale_factor New value to be copied in member torque_scale_factor
     */
    eProsima_user_DllExport void torque_scale_factor(
            const std::array<double, 12>& _torque_scale_factor);

    /*!
     * @brief This function moves the value in member torque_scale_factor
     * @param _torque_scale_factor New value to be moved in member torque_scale_factor
     */
    eProsima_user_DllExport void torque_scale_factor(
            std::array<double, 12>&& _torque_scale_factor);

    /*!
     * @brief This function returns a constant reference to member torque_scale_factor
     * @return Constant reference to member torque_scale_factor
     */
    eProsima_user_DllExport const std::array<double, 12>& torque_scale_factor() const;

    /*!
     * @brief This function returns a reference to member torque_scale_factor
     * @return Reference to member torque_scale_factor
     */
    eProsima_user_DllExport std::array<double, 12>& torque_scale_factor();
    /*!
     * @brief This function copies the value in member kp_torque
     * @param _kp_torque New value to be copied in member kp_torque
     */
    eProsima_user_DllExport void kp_torque(
            const std::array<double, 12>& _kp_torque);

    /*!
     * @brief This function moves the value in member kp_torque
     * @param _kp_torque New value to be moved in member kp_torque
     */
    eProsima_user_DllExport void kp_torque(
            std::array<double, 12>&& _kp_torque);

    /*!
     * @brief This function returns a constant reference to member kp_torque
     * @return Constant reference to member kp_torque
     */
    eProsima_user_DllExport const std::array<double, 12>& kp_torque() const;

    /*!
     * @brief This function returns a reference to member kp_torque
     * @return Reference to member kp_torque
     */
    eProsima_user_DllExport std::array<double, 12>& kp_torque();
    /*!
     * @brief This function copies the value in member ki_torque
     * @param _ki_torque New value to be copied in member ki_torque
     */
    eProsima_user_DllExport void ki_torque(
            const std::array<double, 12>& _ki_torque);

    /*!
     * @brief This function moves the value in member ki_torque
     * @param _ki_torque New value to be moved in member ki_torque
     */
    eProsima_user_DllExport void ki_torque(
            std::array<double, 12>&& _ki_torque);

    /*!
     * @brief This function returns a constant reference to member ki_torque
     * @return Constant reference to member ki_torque
     */
    eProsima_user_DllExport const std::array<double, 12>& ki_torque() const;

    /*!
     * @brief This function returns a reference to member ki_torque
     * @return Reference to member ki_torque
     */
    eProsima_user_DllExport std::array<double, 12>& ki_torque();
    /*!
     * @brief This function copies the value in member kd_torque
     * @param _kd_torque New value to be copied in member kd_torque
     */
    eProsima_user_DllExport void kd_torque(
            const std::array<double, 12>& _kd_torque);

    /*!
     * @brief This function moves the value in member kd_torque
     * @param _kd_torque New value to be moved in member kd_torque
     */
    eProsima_user_DllExport void kd_torque(
            std::array<double, 12>&& _kd_torque);

    /*!
     * @brief This function returns a constant reference to member kd_torque
     * @return Constant reference to member kd_torque
     */
    eProsima_user_DllExport const std::array<double, 12>& kd_torque() const;

    /*!
     * @brief This function returns a reference to member kd_torque
     * @return Reference to member kd_torque
     */
    eProsima_user_DllExport std::array<double, 12>& kd_torque();
    /*!
     * @brief This function copies the value in member kp_position
     * @param _kp_position New value to be copied in member kp_position
     */
    eProsima_user_DllExport void kp_position(
            const std::array<double, 12>& _kp_position);

    /*!
     * @brief This function moves the value in member kp_position
     * @param _kp_position New value to be moved in member kp_position
     */
    eProsima_user_DllExport void kp_position(
            std::array<double, 12>&& _kp_position);

    /*!
     * @brief This function returns a constant reference to member kp_position
     * @return Constant reference to member kp_position
     */
    eProsima_user_DllExport const std::array<double, 12>& kp_position() const;

    /*!
     * @brief This function returns a reference to member kp_position
     * @return Reference to member kp_position
     */
    eProsima_user_DllExport std::array<double, 12>& kp_position();
    /*!
     * @brief This function copies the value in member ki_position
     * @param _ki_position New value to be copied in member ki_position
     */
    eProsima_user_DllExport void ki_position(
            const std::array<double, 12>& _ki_position);

    /*!
     * @brief This function moves the value in member ki_position
     * @param _ki_position New value to be moved in member ki_position
     */
    eProsima_user_DllExport void ki_position(
            std::array<double, 12>&& _ki_position);

    /*!
     * @brief This function returns a constant reference to member ki_position
     * @return Constant reference to member ki_position
     */
    eProsima_user_DllExport const std::array<double, 12>& ki_position() const;

    /*!
     * @brief This function returns a reference to member ki_position
     * @return Reference to member ki_position
     */
    eProsima_user_DllExport std::array<double, 12>& ki_position();
    /*!
     * @brief This function copies the value in member kd_position
     * @param _kd_position New value to be copied in member kd_position
     */
    eProsima_user_DllExport void kd_position(
            const std::array<double, 12>& _kd_position);

    /*!
     * @brief This function moves the value in member kd_position
     * @param _kd_position New value to be moved in member kd_position
     */
    eProsima_user_DllExport void kd_position(
            std::array<double, 12>&& _kd_position);

    /*!
     * @brief This function returns a constant reference to member kd_position
     * @return Constant reference to member kd_position
     */
    eProsima_user_DllExport const std::array<double, 12>& kd_position() const;

    /*!
     * @brief This function returns a reference to member kd_position
     * @return Reference to member kd_position
     */
    eProsima_user_DllExport std::array<double, 12>& kd_position();
    /*!
     * @brief This function copies the value in member current_offset
     * @param _current_offset New value to be copied in member current_offset
     */
    eProsima_user_DllExport void current_offset(
            const std::array<double, 12>& _current_offset);

    /*!
     * @brief This function moves the value in member current_offset
     * @param _current_offset New value to be moved in member current_offset
     */
    eProsima_user_DllExport void current_offset(
            std::array<double, 12>&& _current_offset);

    /*!
     * @brief This function returns a constant reference to member current_offset
     * @return Constant reference to member current_offset
     */
    eProsima_user_DllExport const std::array<double, 12>& current_offset() const;

    /*!
     * @brief This function returns a reference to member current_offset
     * @return Reference to member current_offset
     */
    eProsima_user_DllExport std::array<double, 12>& current_offset();

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
            const EcatControlRefMsg& data,
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

    std::string m_frame_id;
    uint32_t m_sequence_id;
    double m_timestamp;
    std::array<double, 12> m_amplitude;
    std::array<double, 12> m_frequency;
    std::array<double, 12> m_offset;
    std::array<std::string, 12> m_traj_type;
    std::array<std::string, 12> m_traj_mode;
    std::array<bool, 12> m_start_traj;
    std::array<double, 12> m_max_freq;
    std::array<double, 12> m_torque_scale_factor;
    std::array<double, 12> m_kp_torque;
    std::array<double, 12> m_ki_torque;
    std::array<double, 12> m_kd_torque;
    std::array<double, 12> m_kp_position;
    std::array<double, 12> m_ki_position;
    std::array<double, 12> m_kd_position;
    std::array<double, 12> m_current_offset;

};

#endif // _FAST_DDS_GENERATED_ECAT_CONTROL_REF_H_

