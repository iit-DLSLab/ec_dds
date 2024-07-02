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
 * @file ecat_robot_stateTypeObject.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "ecat_robot_state.h"
#include "ecat_robot_stateTypeObject.h"
#include <mutex>
#include <utility>
#include <sstream>
#include <fastrtps/rtps/common/SerializedPayload.h>
#include <fastrtps/utils/md5.h>
#include <fastrtps/types/TypeObjectFactory.h>
#include <fastrtps/types/TypeNamesGenerator.h>
#include <fastrtps/types/AnnotationParameterValue.h>
#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>

using namespace eprosima::fastrtps::rtps;

void registerecat_robot_stateTypes()
{
    static std::once_flag once_flag;
    std::call_once(once_flag, []()
            {
                TypeObjectFactory *factory = TypeObjectFactory::get_instance();
                factory->add_type_object("EcatRobotStateMsg", GetEcatRobotStateMsgIdentifier(true),
                GetEcatRobotStateMsgObject(true));
                factory->add_type_object("EcatRobotStateMsg", GetEcatRobotStateMsgIdentifier(false),
                GetEcatRobotStateMsgObject(false));

            });
}

const TypeIdentifier* GetEcatRobotStateMsgIdentifier(bool complete)
{
    const TypeIdentifier * c_identifier = TypeObjectFactory::get_instance()->get_type_identifier("EcatRobotStateMsg", complete);
    if (c_identifier != nullptr && (!complete || c_identifier->_d() == EK_COMPLETE))
    {
        return c_identifier;
    }

    GetEcatRobotStateMsgObject(complete); // Generated inside
    return TypeObjectFactory::get_instance()->get_type_identifier("EcatRobotStateMsg", complete);
}

const TypeObject* GetEcatRobotStateMsgObject(bool complete)
{
    const TypeObject* c_type_object = TypeObjectFactory::get_instance()->get_type_object("EcatRobotStateMsg", complete);
    if (c_type_object != nullptr)
    {
        return c_type_object;
    }
    else if (complete)
    {
        return GetCompleteEcatRobotStateMsgObject();
    }
    //else
    return GetMinimalEcatRobotStateMsgObject();
}

const TypeObject* GetMinimalEcatRobotStateMsgObject()
{
    const TypeObject* c_type_object = TypeObjectFactory::get_instance()->get_type_object("EcatRobotStateMsg", false);
    if (c_type_object != nullptr)
    {
        return c_type_object;
    }

    TypeObject *type_object = new TypeObject();
    type_object->_d(EK_MINIMAL);
    type_object->minimal()._d(TK_STRUCTURE);

    type_object->minimal().struct_type().struct_flags().IS_FINAL(false);
    type_object->minimal().struct_type().struct_flags().IS_APPENDABLE(false);
    type_object->minimal().struct_type().struct_flags().IS_MUTABLE(false);
    type_object->minimal().struct_type().struct_flags().IS_NESTED(false);
    type_object->minimal().struct_type().struct_flags().IS_AUTOID_HASH(false); // Unsupported

    MemberId memberId = 0;
    MinimalStructMember mst_frame_id;
    mst_frame_id.common().member_id(memberId++);
    mst_frame_id.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_frame_id.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_frame_id.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_frame_id.common().member_flags().IS_OPTIONAL(false);
    mst_frame_id.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_frame_id.common().member_flags().IS_KEY(false);
    mst_frame_id.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_frame_id.common().member_type_id(*TypeObjectFactory::get_instance()->get_string_identifier(255, false));


    MD5 frame_id_hash("frame_id");
    for(int i = 0; i < 4; ++i)
    {
        mst_frame_id.detail().name_hash()[i] = frame_id_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_frame_id);

    MinimalStructMember mst_sequence_id;
    mst_sequence_id.common().member_id(memberId++);
    mst_sequence_id.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_sequence_id.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_sequence_id.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_sequence_id.common().member_flags().IS_OPTIONAL(false);
    mst_sequence_id.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_sequence_id.common().member_flags().IS_KEY(false);
    mst_sequence_id.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_sequence_id.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("uint32_t", false));

    MD5 sequence_id_hash("sequence_id");
    for(int i = 0; i < 4; ++i)
    {
        mst_sequence_id.detail().name_hash()[i] = sequence_id_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_sequence_id);

    MinimalStructMember mst_timestamp;
    mst_timestamp.common().member_id(memberId++);
    mst_timestamp.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_timestamp.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_timestamp.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_timestamp.common().member_flags().IS_OPTIONAL(false);
    mst_timestamp.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_timestamp.common().member_flags().IS_KEY(false);
    mst_timestamp.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_timestamp.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    MD5 timestamp_hash("timestamp");
    for(int i = 0; i < 4; ++i)
    {
        mst_timestamp.detail().name_hash()[i] = timestamp_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_timestamp);

    MinimalStructMember mst_robot_name;
    mst_robot_name.common().member_id(memberId++);
    mst_robot_name.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_robot_name.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_robot_name.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_robot_name.common().member_flags().IS_OPTIONAL(false);
    mst_robot_name.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_robot_name.common().member_flags().IS_KEY(false);
    mst_robot_name.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_robot_name.common().member_type_id(*TypeObjectFactory::get_instance()->get_string_identifier(255, false));


    MD5 robot_name_hash("robot_name");
    for(int i = 0; i < 4; ++i)
    {
        mst_robot_name.detail().name_hash()[i] = robot_name_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_robot_name);

    MinimalStructMember mst_joints_name;
    mst_joints_name.common().member_id(memberId++);
    mst_joints_name.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_joints_name.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_joints_name.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_joints_name.common().member_flags().IS_OPTIONAL(false);
    mst_joints_name.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_joints_name.common().member_flags().IS_KEY(false);
    mst_joints_name.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_joints_name.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier(TypeNamesGenerator::get_string_type_name(255, false), {12}, false));


    MD5 joints_name_hash("joints_name");
    for(int i = 0; i < 4; ++i)
    {
        mst_joints_name.detail().name_hash()[i] = joints_name_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_joints_name);

    MinimalStructMember mst_joints_temperature;
    mst_joints_temperature.common().member_id(memberId++);
    mst_joints_temperature.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_joints_temperature.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_joints_temperature.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_joints_temperature.common().member_flags().IS_OPTIONAL(false);
    mst_joints_temperature.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_joints_temperature.common().member_flags().IS_KEY(false);
    mst_joints_temperature.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_joints_temperature.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, false));


    MD5 joints_temperature_hash("joints_temperature");
    for(int i = 0; i < 4; ++i)
    {
        mst_joints_temperature.detail().name_hash()[i] = joints_temperature_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_joints_temperature);

    MinimalStructMember mst_joints_position;
    mst_joints_position.common().member_id(memberId++);
    mst_joints_position.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_joints_position.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_joints_position.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_joints_position.common().member_flags().IS_OPTIONAL(false);
    mst_joints_position.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_joints_position.common().member_flags().IS_KEY(false);
    mst_joints_position.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_joints_position.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, false));


    MD5 joints_position_hash("joints_position");
    for(int i = 0; i < 4; ++i)
    {
        mst_joints_position.detail().name_hash()[i] = joints_position_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_joints_position);

    MinimalStructMember mst_joints_velocity;
    mst_joints_velocity.common().member_id(memberId++);
    mst_joints_velocity.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_joints_velocity.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_joints_velocity.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_joints_velocity.common().member_flags().IS_OPTIONAL(false);
    mst_joints_velocity.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_joints_velocity.common().member_flags().IS_KEY(false);
    mst_joints_velocity.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_joints_velocity.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, false));


    MD5 joints_velocity_hash("joints_velocity");
    for(int i = 0; i < 4; ++i)
    {
        mst_joints_velocity.detail().name_hash()[i] = joints_velocity_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_joints_velocity);

    MinimalStructMember mst_joints_acceleration;
    mst_joints_acceleration.common().member_id(memberId++);
    mst_joints_acceleration.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_joints_acceleration.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_joints_acceleration.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_joints_acceleration.common().member_flags().IS_OPTIONAL(false);
    mst_joints_acceleration.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_joints_acceleration.common().member_flags().IS_KEY(false);
    mst_joints_acceleration.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_joints_acceleration.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, false));


    MD5 joints_acceleration_hash("joints_acceleration");
    for(int i = 0; i < 4; ++i)
    {
        mst_joints_acceleration.detail().name_hash()[i] = joints_acceleration_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_joints_acceleration);

    MinimalStructMember mst_joints_torques;
    mst_joints_torques.common().member_id(memberId++);
    mst_joints_torques.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_joints_torques.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_joints_torques.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_joints_torques.common().member_flags().IS_OPTIONAL(false);
    mst_joints_torques.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_joints_torques.common().member_flags().IS_KEY(false);
    mst_joints_torques.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_joints_torques.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, false));


    MD5 joints_torques_hash("joints_torques");
    for(int i = 0; i < 4; ++i)
    {
        mst_joints_torques.detail().name_hash()[i] = joints_torques_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_joints_torques);

    MinimalStructMember mst_feet_contact;
    mst_feet_contact.common().member_id(memberId++);
    mst_feet_contact.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_feet_contact.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_feet_contact.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_feet_contact.common().member_flags().IS_OPTIONAL(false);
    mst_feet_contact.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_feet_contact.common().member_flags().IS_KEY(false);
    mst_feet_contact.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_feet_contact.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {4}, false));


    MD5 feet_contact_hash("feet_contact");
    for(int i = 0; i < 4; ++i)
    {
        mst_feet_contact.detail().name_hash()[i] = feet_contact_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_feet_contact);

    MinimalStructMember mst_feet_position;
    mst_feet_position.common().member_id(memberId++);
    mst_feet_position.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_feet_position.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_feet_position.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_feet_position.common().member_flags().IS_OPTIONAL(false);
    mst_feet_position.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_feet_position.common().member_flags().IS_KEY(false);
    mst_feet_position.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_feet_position.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, false));


    MD5 feet_position_hash("feet_position");
    for(int i = 0; i < 4; ++i)
    {
        mst_feet_position.detail().name_hash()[i] = feet_position_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_feet_position);

    MinimalStructMember mst_current_ref;
    mst_current_ref.common().member_id(memberId++);
    mst_current_ref.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_current_ref.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_current_ref.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_current_ref.common().member_flags().IS_OPTIONAL(false);
    mst_current_ref.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_current_ref.common().member_flags().IS_KEY(false);
    mst_current_ref.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_current_ref.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, false));


    MD5 current_ref_hash("current_ref");
    for(int i = 0; i < 4; ++i)
    {
        mst_current_ref.detail().name_hash()[i] = current_ref_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_current_ref);

    MinimalStructMember mst_position_ref;
    mst_position_ref.common().member_id(memberId++);
    mst_position_ref.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_position_ref.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_position_ref.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_position_ref.common().member_flags().IS_OPTIONAL(false);
    mst_position_ref.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_position_ref.common().member_flags().IS_KEY(false);
    mst_position_ref.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_position_ref.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, false));


    MD5 position_ref_hash("position_ref");
    for(int i = 0; i < 4; ++i)
    {
        mst_position_ref.detail().name_hash()[i] = position_ref_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_position_ref);

    MinimalStructMember mst_torque_ref;
    mst_torque_ref.common().member_id(memberId++);
    mst_torque_ref.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_torque_ref.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_torque_ref.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_torque_ref.common().member_flags().IS_OPTIONAL(false);
    mst_torque_ref.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_torque_ref.common().member_flags().IS_KEY(false);
    mst_torque_ref.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_torque_ref.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, false));


    MD5 torque_ref_hash("torque_ref");
    for(int i = 0; i < 4; ++i)
    {
        mst_torque_ref.detail().name_hash()[i] = torque_ref_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_torque_ref);

    MinimalStructMember mst_current_offset;
    mst_current_offset.common().member_id(memberId++);
    mst_current_offset.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_current_offset.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_current_offset.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_current_offset.common().member_flags().IS_OPTIONAL(false);
    mst_current_offset.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_current_offset.common().member_flags().IS_KEY(false);
    mst_current_offset.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_current_offset.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, false));


    MD5 current_offset_hash("current_offset");
    for(int i = 0; i < 4; ++i)
    {
        mst_current_offset.detail().name_hash()[i] = current_offset_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_current_offset);


    // Header
    // TODO Inheritance
    //type_object->minimal().struct_type().header().base_type()._d(EK_MINIMAL);
    //type_object->minimal().struct_type().header().base_type().equivalence_hash()[0..13];

    TypeIdentifier identifier;
    identifier._d(EK_MINIMAL);

    SerializedPayload_t payload(static_cast<uint32_t>(
        MinimalStructType::getCdrSerializedSize(type_object->minimal().struct_type()) + 4));
    eprosima::fastcdr::FastBuffer fastbuffer((char*) payload.data, payload.max_size);
    // Fixed endian (Page 221, EquivalenceHash definition of Extensible and Dynamic Topic Types for DDS document)
    eprosima::fastcdr::Cdr ser(
        fastbuffer, eprosima::fastcdr::Cdr::LITTLE_ENDIANNESS,
        eprosima::fastcdr::Cdr::DDS_CDR); // Object that serializes the data.
    payload.encapsulation = CDR_LE;

    type_object->serialize(ser);
    payload.length = (uint32_t)ser.getSerializedDataLength(); //Get the serialized length
    MD5 objectHash;
    objectHash.update((char*)payload.data, payload.length);
    objectHash.finalize();
    for(int i = 0; i < 14; ++i)
    {
        identifier.equivalence_hash()[i] = objectHash.digest[i];
    }

    TypeObjectFactory::get_instance()->add_type_object("EcatRobotStateMsg", &identifier, type_object);
    delete type_object;
    return TypeObjectFactory::get_instance()->get_type_object("EcatRobotStateMsg", false);
}

const TypeObject* GetCompleteEcatRobotStateMsgObject()
{
    const TypeObject* c_type_object = TypeObjectFactory::get_instance()->get_type_object("EcatRobotStateMsg", true);
    if (c_type_object != nullptr && c_type_object->_d() == EK_COMPLETE)
    {
        return c_type_object;
    }

    TypeObject *type_object = new TypeObject();
    type_object->_d(EK_COMPLETE);
    type_object->complete()._d(TK_STRUCTURE);

    type_object->complete().struct_type().struct_flags().IS_FINAL(false);
    type_object->complete().struct_type().struct_flags().IS_APPENDABLE(false);
    type_object->complete().struct_type().struct_flags().IS_MUTABLE(false);
    type_object->complete().struct_type().struct_flags().IS_NESTED(false);
    type_object->complete().struct_type().struct_flags().IS_AUTOID_HASH(false); // Unsupported

    MemberId memberId = 0;
    CompleteStructMember cst_frame_id;
    cst_frame_id.common().member_id(memberId++);
    cst_frame_id.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_frame_id.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_frame_id.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_frame_id.common().member_flags().IS_OPTIONAL(false);
    cst_frame_id.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_frame_id.common().member_flags().IS_KEY(false);
    cst_frame_id.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_frame_id.common().member_type_id(*TypeObjectFactory::get_instance()->get_string_identifier(255, false));


    cst_frame_id.detail().name("frame_id");

    type_object->complete().struct_type().member_seq().emplace_back(cst_frame_id);

    CompleteStructMember cst_sequence_id;
    cst_sequence_id.common().member_id(memberId++);
    cst_sequence_id.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_sequence_id.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_sequence_id.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_sequence_id.common().member_flags().IS_OPTIONAL(false);
    cst_sequence_id.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_sequence_id.common().member_flags().IS_KEY(false);
    cst_sequence_id.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_sequence_id.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("uint32_t", false));

    cst_sequence_id.detail().name("sequence_id");

    type_object->complete().struct_type().member_seq().emplace_back(cst_sequence_id);

    CompleteStructMember cst_timestamp;
    cst_timestamp.common().member_id(memberId++);
    cst_timestamp.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_timestamp.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_timestamp.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_timestamp.common().member_flags().IS_OPTIONAL(false);
    cst_timestamp.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_timestamp.common().member_flags().IS_KEY(false);
    cst_timestamp.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_timestamp.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    cst_timestamp.detail().name("timestamp");

    type_object->complete().struct_type().member_seq().emplace_back(cst_timestamp);

    CompleteStructMember cst_robot_name;
    cst_robot_name.common().member_id(memberId++);
    cst_robot_name.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_robot_name.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_robot_name.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_robot_name.common().member_flags().IS_OPTIONAL(false);
    cst_robot_name.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_robot_name.common().member_flags().IS_KEY(false);
    cst_robot_name.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_robot_name.common().member_type_id(*TypeObjectFactory::get_instance()->get_string_identifier(255, false));


    cst_robot_name.detail().name("robot_name");

    type_object->complete().struct_type().member_seq().emplace_back(cst_robot_name);

    CompleteStructMember cst_joints_name;
    cst_joints_name.common().member_id(memberId++);
    cst_joints_name.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_joints_name.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_joints_name.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_joints_name.common().member_flags().IS_OPTIONAL(false);
    cst_joints_name.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_joints_name.common().member_flags().IS_KEY(false);
    cst_joints_name.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_joints_name.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier(TypeNamesGenerator::get_string_type_name(255, false), {12}, true));


    cst_joints_name.detail().name("joints_name");

    type_object->complete().struct_type().member_seq().emplace_back(cst_joints_name);

    CompleteStructMember cst_joints_temperature;
    cst_joints_temperature.common().member_id(memberId++);
    cst_joints_temperature.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_joints_temperature.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_joints_temperature.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_joints_temperature.common().member_flags().IS_OPTIONAL(false);
    cst_joints_temperature.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_joints_temperature.common().member_flags().IS_KEY(false);
    cst_joints_temperature.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_joints_temperature.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, true));


    cst_joints_temperature.detail().name("joints_temperature");

    type_object->complete().struct_type().member_seq().emplace_back(cst_joints_temperature);

    CompleteStructMember cst_joints_position;
    cst_joints_position.common().member_id(memberId++);
    cst_joints_position.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_joints_position.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_joints_position.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_joints_position.common().member_flags().IS_OPTIONAL(false);
    cst_joints_position.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_joints_position.common().member_flags().IS_KEY(false);
    cst_joints_position.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_joints_position.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, true));


    cst_joints_position.detail().name("joints_position");

    type_object->complete().struct_type().member_seq().emplace_back(cst_joints_position);

    CompleteStructMember cst_joints_velocity;
    cst_joints_velocity.common().member_id(memberId++);
    cst_joints_velocity.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_joints_velocity.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_joints_velocity.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_joints_velocity.common().member_flags().IS_OPTIONAL(false);
    cst_joints_velocity.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_joints_velocity.common().member_flags().IS_KEY(false);
    cst_joints_velocity.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_joints_velocity.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, true));


    cst_joints_velocity.detail().name("joints_velocity");

    type_object->complete().struct_type().member_seq().emplace_back(cst_joints_velocity);

    CompleteStructMember cst_joints_acceleration;
    cst_joints_acceleration.common().member_id(memberId++);
    cst_joints_acceleration.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_joints_acceleration.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_joints_acceleration.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_joints_acceleration.common().member_flags().IS_OPTIONAL(false);
    cst_joints_acceleration.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_joints_acceleration.common().member_flags().IS_KEY(false);
    cst_joints_acceleration.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_joints_acceleration.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, true));


    cst_joints_acceleration.detail().name("joints_acceleration");

    type_object->complete().struct_type().member_seq().emplace_back(cst_joints_acceleration);

    CompleteStructMember cst_joints_torques;
    cst_joints_torques.common().member_id(memberId++);
    cst_joints_torques.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_joints_torques.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_joints_torques.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_joints_torques.common().member_flags().IS_OPTIONAL(false);
    cst_joints_torques.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_joints_torques.common().member_flags().IS_KEY(false);
    cst_joints_torques.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_joints_torques.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, true));


    cst_joints_torques.detail().name("joints_torques");

    type_object->complete().struct_type().member_seq().emplace_back(cst_joints_torques);

    CompleteStructMember cst_feet_contact;
    cst_feet_contact.common().member_id(memberId++);
    cst_feet_contact.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_feet_contact.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_feet_contact.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_feet_contact.common().member_flags().IS_OPTIONAL(false);
    cst_feet_contact.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_feet_contact.common().member_flags().IS_KEY(false);
    cst_feet_contact.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_feet_contact.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {4}, true));


    cst_feet_contact.detail().name("feet_contact");

    type_object->complete().struct_type().member_seq().emplace_back(cst_feet_contact);

    CompleteStructMember cst_feet_position;
    cst_feet_position.common().member_id(memberId++);
    cst_feet_position.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_feet_position.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_feet_position.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_feet_position.common().member_flags().IS_OPTIONAL(false);
    cst_feet_position.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_feet_position.common().member_flags().IS_KEY(false);
    cst_feet_position.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_feet_position.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, true));


    cst_feet_position.detail().name("feet_position");

    type_object->complete().struct_type().member_seq().emplace_back(cst_feet_position);

    CompleteStructMember cst_current_ref;
    cst_current_ref.common().member_id(memberId++);
    cst_current_ref.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_current_ref.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_current_ref.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_current_ref.common().member_flags().IS_OPTIONAL(false);
    cst_current_ref.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_current_ref.common().member_flags().IS_KEY(false);
    cst_current_ref.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_current_ref.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, true));


    cst_current_ref.detail().name("current_ref");

    type_object->complete().struct_type().member_seq().emplace_back(cst_current_ref);

    CompleteStructMember cst_position_ref;
    cst_position_ref.common().member_id(memberId++);
    cst_position_ref.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_position_ref.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_position_ref.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_position_ref.common().member_flags().IS_OPTIONAL(false);
    cst_position_ref.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_position_ref.common().member_flags().IS_KEY(false);
    cst_position_ref.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_position_ref.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, true));


    cst_position_ref.detail().name("position_ref");

    type_object->complete().struct_type().member_seq().emplace_back(cst_position_ref);

    CompleteStructMember cst_torque_ref;
    cst_torque_ref.common().member_id(memberId++);
    cst_torque_ref.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_torque_ref.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_torque_ref.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_torque_ref.common().member_flags().IS_OPTIONAL(false);
    cst_torque_ref.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_torque_ref.common().member_flags().IS_KEY(false);
    cst_torque_ref.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_torque_ref.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, true));


    cst_torque_ref.detail().name("torque_ref");

    type_object->complete().struct_type().member_seq().emplace_back(cst_torque_ref);

    CompleteStructMember cst_current_offset;
    cst_current_offset.common().member_id(memberId++);
    cst_current_offset.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_current_offset.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_current_offset.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_current_offset.common().member_flags().IS_OPTIONAL(false);
    cst_current_offset.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_current_offset.common().member_flags().IS_KEY(false);
    cst_current_offset.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_current_offset.common().member_type_id(*TypeObjectFactory::get_instance()->get_array_identifier("double", {12}, true));


    cst_current_offset.detail().name("current_offset");

    type_object->complete().struct_type().member_seq().emplace_back(cst_current_offset);


    // Header
    type_object->complete().struct_type().header().detail().type_name("EcatRobotStateMsg");
    // TODO inheritance


    TypeIdentifier identifier;
    identifier._d(EK_COMPLETE);

    SerializedPayload_t payload(static_cast<uint32_t>(
        CompleteStructType::getCdrSerializedSize(type_object->complete().struct_type()) + 4));
    eprosima::fastcdr::FastBuffer fastbuffer((char*) payload.data, payload.max_size);
    // Fixed endian (Page 221, EquivalenceHash definition of Extensible and Dynamic Topic Types for DDS document)
    eprosima::fastcdr::Cdr ser(
        fastbuffer, eprosima::fastcdr::Cdr::LITTLE_ENDIANNESS,
        eprosima::fastcdr::Cdr::DDS_CDR); // Object that serializes the data.
    payload.encapsulation = CDR_LE;

    type_object->serialize(ser);
    payload.length = (uint32_t)ser.getSerializedDataLength(); //Get the serialized length
    MD5 objectHash;
    objectHash.update((char*)payload.data, payload.length);
    objectHash.finalize();
    for(int i = 0; i < 14; ++i)
    {
        identifier.equivalence_hash()[i] = objectHash.digest[i];
    }

    TypeObjectFactory::get_instance()->add_type_object("EcatRobotStateMsg", &identifier, type_object);
    delete type_object;
    return TypeObjectFactory::get_instance()->get_type_object("EcatRobotStateMsg", true);
}
