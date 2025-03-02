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
 * @file ecat_valveTypeObject.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "ecat_valve.h"
#include "ecat_valveTypeObject.h"
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

void registerecat_valveTypes()
{
    static std::once_flag once_flag;
    std::call_once(once_flag, []()
            {
                TypeObjectFactory *factory = TypeObjectFactory::get_instance();
                factory->add_type_object("ECValveMsg", GetECValveMsgIdentifier(true),
                GetECValveMsgObject(true));
                factory->add_type_object("ECValveMsg", GetECValveMsgIdentifier(false),
                GetECValveMsgObject(false));

            });
}

const TypeIdentifier* GetECValveMsgIdentifier(bool complete)
{
    const TypeIdentifier * c_identifier = TypeObjectFactory::get_instance()->get_type_identifier("ECValveMsg", complete);
    if (c_identifier != nullptr && (!complete || c_identifier->_d() == EK_COMPLETE))
    {
        return c_identifier;
    }

    GetECValveMsgObject(complete); // Generated inside
    return TypeObjectFactory::get_instance()->get_type_identifier("ECValveMsg", complete);
}

const TypeObject* GetECValveMsgObject(bool complete)
{
    const TypeObject* c_type_object = TypeObjectFactory::get_instance()->get_type_object("ECValveMsg", complete);
    if (c_type_object != nullptr)
    {
        return c_type_object;
    }
    else if (complete)
    {
        return GetCompleteECValveMsgObject();
    }
    //else
    return GetMinimalECValveMsgObject();
}

const TypeObject* GetMinimalECValveMsgObject()
{
    const TypeObject* c_type_object = TypeObjectFactory::get_instance()->get_type_object("ECValveMsg", false);
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

    MinimalStructMember mst_index;
    mst_index.common().member_id(memberId++);
    mst_index.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_index.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_index.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_index.common().member_flags().IS_OPTIONAL(false);
    mst_index.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_index.common().member_flags().IS_KEY(false);
    mst_index.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_index.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("uint32_t", false));

    MD5 index_hash("index");
    for(int i = 0; i < 4; ++i)
    {
        mst_index.detail().name_hash()[i] = index_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_index);

    MinimalStructMember mst_id;
    mst_id.common().member_id(memberId++);
    mst_id.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_id.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_id.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_id.common().member_flags().IS_OPTIONAL(false);
    mst_id.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_id.common().member_flags().IS_KEY(false);
    mst_id.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_id.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("uint32_t", false));

    MD5 id_hash("id");
    for(int i = 0; i < 4; ++i)
    {
        mst_id.detail().name_hash()[i] = id_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_id);

    MinimalStructMember mst_encoderPos;
    mst_encoderPos.common().member_id(memberId++);
    mst_encoderPos.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_encoderPos.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_encoderPos.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_encoderPos.common().member_flags().IS_OPTIONAL(false);
    mst_encoderPos.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_encoderPos.common().member_flags().IS_KEY(false);
    mst_encoderPos.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_encoderPos.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    MD5 encoderPos_hash("encoderPos");
    for(int i = 0; i < 4; ++i)
    {
        mst_encoderPos.detail().name_hash()[i] = encoderPos_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_encoderPos);

    MinimalStructMember mst_torque;
    mst_torque.common().member_id(memberId++);
    mst_torque.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_torque.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_torque.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_torque.common().member_flags().IS_OPTIONAL(false);
    mst_torque.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_torque.common().member_flags().IS_KEY(false);
    mst_torque.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_torque.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    MD5 torque_hash("torque");
    for(int i = 0; i < 4; ++i)
    {
        mst_torque.detail().name_hash()[i] = torque_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_torque);

    MinimalStructMember mst_pressure1;
    mst_pressure1.common().member_id(memberId++);
    mst_pressure1.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_pressure1.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_pressure1.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_pressure1.common().member_flags().IS_OPTIONAL(false);
    mst_pressure1.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_pressure1.common().member_flags().IS_KEY(false);
    mst_pressure1.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_pressure1.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    MD5 pressure1_hash("pressure1");
    for(int i = 0; i < 4; ++i)
    {
        mst_pressure1.detail().name_hash()[i] = pressure1_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_pressure1);

    MinimalStructMember mst_pressure2;
    mst_pressure2.common().member_id(memberId++);
    mst_pressure2.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_pressure2.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_pressure2.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_pressure2.common().member_flags().IS_OPTIONAL(false);
    mst_pressure2.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_pressure2.common().member_flags().IS_KEY(false);
    mst_pressure2.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_pressure2.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    MD5 pressure2_hash("pressure2");
    for(int i = 0; i < 4; ++i)
    {
        mst_pressure2.detail().name_hash()[i] = pressure2_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_pressure2);

    MinimalStructMember mst_temp;
    mst_temp.common().member_id(memberId++);
    mst_temp.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_temp.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_temp.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_temp.common().member_flags().IS_OPTIONAL(false);
    mst_temp.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_temp.common().member_flags().IS_KEY(false);
    mst_temp.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_temp.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    MD5 temp_hash("temp");
    for(int i = 0; i < 4; ++i)
    {
        mst_temp.detail().name_hash()[i] = temp_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_temp);

    MinimalStructMember mst_kp;
    mst_kp.common().member_id(memberId++);
    mst_kp.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_kp.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_kp.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_kp.common().member_flags().IS_OPTIONAL(false);
    mst_kp.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_kp.common().member_flags().IS_KEY(false);
    mst_kp.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_kp.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    MD5 kp_hash("kp");
    for(int i = 0; i < 4; ++i)
    {
        mst_kp.detail().name_hash()[i] = kp_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_kp);

    MinimalStructMember mst_ki;
    mst_ki.common().member_id(memberId++);
    mst_ki.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_ki.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_ki.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_ki.common().member_flags().IS_OPTIONAL(false);
    mst_ki.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_ki.common().member_flags().IS_KEY(false);
    mst_ki.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_ki.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    MD5 ki_hash("ki");
    for(int i = 0; i < 4; ++i)
    {
        mst_ki.detail().name_hash()[i] = ki_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_ki);

    MinimalStructMember mst_kd;
    mst_kd.common().member_id(memberId++);
    mst_kd.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_kd.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_kd.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_kd.common().member_flags().IS_OPTIONAL(false);
    mst_kd.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_kd.common().member_flags().IS_KEY(false);
    mst_kd.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_kd.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    MD5 kd_hash("kd");
    for(int i = 0; i < 4; ++i)
    {
        mst_kd.detail().name_hash()[i] = kd_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_kd);

    MinimalStructMember mst_current_ref;
    mst_current_ref.common().member_id(memberId++);
    mst_current_ref.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_current_ref.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_current_ref.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_current_ref.common().member_flags().IS_OPTIONAL(false);
    mst_current_ref.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_current_ref.common().member_flags().IS_KEY(false);
    mst_current_ref.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_current_ref.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

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
    mst_position_ref.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

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
    mst_torque_ref.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

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
    mst_current_offset.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

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

    TypeObjectFactory::get_instance()->add_type_object("ECValveMsg", &identifier, type_object);
    delete type_object;
    return TypeObjectFactory::get_instance()->get_type_object("ECValveMsg", false);
}

const TypeObject* GetCompleteECValveMsgObject()
{
    const TypeObject* c_type_object = TypeObjectFactory::get_instance()->get_type_object("ECValveMsg", true);
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

    CompleteStructMember cst_index;
    cst_index.common().member_id(memberId++);
    cst_index.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_index.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_index.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_index.common().member_flags().IS_OPTIONAL(false);
    cst_index.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_index.common().member_flags().IS_KEY(false);
    cst_index.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_index.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("uint32_t", false));

    cst_index.detail().name("index");

    type_object->complete().struct_type().member_seq().emplace_back(cst_index);

    CompleteStructMember cst_id;
    cst_id.common().member_id(memberId++);
    cst_id.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_id.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_id.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_id.common().member_flags().IS_OPTIONAL(false);
    cst_id.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_id.common().member_flags().IS_KEY(false);
    cst_id.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_id.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("uint32_t", false));

    cst_id.detail().name("id");

    type_object->complete().struct_type().member_seq().emplace_back(cst_id);

    CompleteStructMember cst_encoderPos;
    cst_encoderPos.common().member_id(memberId++);
    cst_encoderPos.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_encoderPos.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_encoderPos.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_encoderPos.common().member_flags().IS_OPTIONAL(false);
    cst_encoderPos.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_encoderPos.common().member_flags().IS_KEY(false);
    cst_encoderPos.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_encoderPos.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    cst_encoderPos.detail().name("encoderPos");

    type_object->complete().struct_type().member_seq().emplace_back(cst_encoderPos);

    CompleteStructMember cst_torque;
    cst_torque.common().member_id(memberId++);
    cst_torque.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_torque.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_torque.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_torque.common().member_flags().IS_OPTIONAL(false);
    cst_torque.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_torque.common().member_flags().IS_KEY(false);
    cst_torque.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_torque.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    cst_torque.detail().name("torque");

    type_object->complete().struct_type().member_seq().emplace_back(cst_torque);

    CompleteStructMember cst_pressure1;
    cst_pressure1.common().member_id(memberId++);
    cst_pressure1.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_pressure1.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_pressure1.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_pressure1.common().member_flags().IS_OPTIONAL(false);
    cst_pressure1.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_pressure1.common().member_flags().IS_KEY(false);
    cst_pressure1.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_pressure1.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    cst_pressure1.detail().name("pressure1");

    type_object->complete().struct_type().member_seq().emplace_back(cst_pressure1);

    CompleteStructMember cst_pressure2;
    cst_pressure2.common().member_id(memberId++);
    cst_pressure2.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_pressure2.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_pressure2.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_pressure2.common().member_flags().IS_OPTIONAL(false);
    cst_pressure2.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_pressure2.common().member_flags().IS_KEY(false);
    cst_pressure2.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_pressure2.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    cst_pressure2.detail().name("pressure2");

    type_object->complete().struct_type().member_seq().emplace_back(cst_pressure2);

    CompleteStructMember cst_temp;
    cst_temp.common().member_id(memberId++);
    cst_temp.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_temp.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_temp.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_temp.common().member_flags().IS_OPTIONAL(false);
    cst_temp.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_temp.common().member_flags().IS_KEY(false);
    cst_temp.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_temp.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    cst_temp.detail().name("temp");

    type_object->complete().struct_type().member_seq().emplace_back(cst_temp);

    CompleteStructMember cst_kp;
    cst_kp.common().member_id(memberId++);
    cst_kp.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_kp.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_kp.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_kp.common().member_flags().IS_OPTIONAL(false);
    cst_kp.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_kp.common().member_flags().IS_KEY(false);
    cst_kp.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_kp.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    cst_kp.detail().name("kp");

    type_object->complete().struct_type().member_seq().emplace_back(cst_kp);

    CompleteStructMember cst_ki;
    cst_ki.common().member_id(memberId++);
    cst_ki.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_ki.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_ki.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_ki.common().member_flags().IS_OPTIONAL(false);
    cst_ki.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_ki.common().member_flags().IS_KEY(false);
    cst_ki.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_ki.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    cst_ki.detail().name("ki");

    type_object->complete().struct_type().member_seq().emplace_back(cst_ki);

    CompleteStructMember cst_kd;
    cst_kd.common().member_id(memberId++);
    cst_kd.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_kd.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_kd.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_kd.common().member_flags().IS_OPTIONAL(false);
    cst_kd.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_kd.common().member_flags().IS_KEY(false);
    cst_kd.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_kd.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    cst_kd.detail().name("kd");

    type_object->complete().struct_type().member_seq().emplace_back(cst_kd);

    CompleteStructMember cst_current_ref;
    cst_current_ref.common().member_id(memberId++);
    cst_current_ref.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_current_ref.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_current_ref.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_current_ref.common().member_flags().IS_OPTIONAL(false);
    cst_current_ref.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_current_ref.common().member_flags().IS_KEY(false);
    cst_current_ref.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_current_ref.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

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
    cst_position_ref.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

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
    cst_torque_ref.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

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
    cst_current_offset.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    cst_current_offset.detail().name("current_offset");

    type_object->complete().struct_type().member_seq().emplace_back(cst_current_offset);


    // Header
    type_object->complete().struct_type().header().detail().type_name("ECValveMsg");
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

    TypeObjectFactory::get_instance()->add_type_object("ECValveMsg", &identifier, type_object);
    delete type_object;
    return TypeObjectFactory::get_instance()->get_type_object("ECValveMsg", true);
}
