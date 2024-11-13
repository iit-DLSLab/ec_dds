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
 * @file ecat_robot_net_statTypeObject.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "ecat_robot_net_stat.h"
#include "ecat_robot_net_statTypeObject.h"
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

void registerecat_robot_net_statTypes()
{
    static std::once_flag once_flag;
    std::call_once(once_flag, []()
            {
                TypeObjectFactory *factory = TypeObjectFactory::get_instance();
                factory->add_type_object("NetStatMsg", GetNetStatMsgIdentifier(true),
                GetNetStatMsgObject(true));
                factory->add_type_object("NetStatMsg", GetNetStatMsgIdentifier(false),
                GetNetStatMsgObject(false));

            });
}

const TypeIdentifier* GetNetStatMsgIdentifier(bool complete)
{
    const TypeIdentifier * c_identifier = TypeObjectFactory::get_instance()->get_type_identifier("NetStatMsg", complete);
    if (c_identifier != nullptr && (!complete || c_identifier->_d() == EK_COMPLETE))
    {
        return c_identifier;
    }

    GetNetStatMsgObject(complete); // Generated inside
    return TypeObjectFactory::get_instance()->get_type_identifier("NetStatMsg", complete);
}

const TypeObject* GetNetStatMsgObject(bool complete)
{
    const TypeObject* c_type_object = TypeObjectFactory::get_instance()->get_type_object("NetStatMsg", complete);
    if (c_type_object != nullptr)
    {
        return c_type_object;
    }
    else if (complete)
    {
        return GetCompleteNetStatMsgObject();
    }
    //else
    return GetMinimalNetStatMsgObject();
}

const TypeObject* GetMinimalNetStatMsgObject()
{
    const TypeObject* c_type_object = TypeObjectFactory::get_instance()->get_type_object("NetStatMsg", false);
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
    MinimalStructMember mst_delta_timestamp;
    mst_delta_timestamp.common().member_id(memberId++);
    mst_delta_timestamp.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_delta_timestamp.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_delta_timestamp.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_delta_timestamp.common().member_flags().IS_OPTIONAL(false);
    mst_delta_timestamp.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_delta_timestamp.common().member_flags().IS_KEY(false);
    mst_delta_timestamp.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_delta_timestamp.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    MD5 delta_timestamp_hash("delta_timestamp");
    for(int i = 0; i < 4; ++i)
    {
        mst_delta_timestamp.detail().name_hash()[i] = delta_timestamp_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_delta_timestamp);

    MinimalStructMember mst_missed_packages_tot;
    mst_missed_packages_tot.common().member_id(memberId++);
    mst_missed_packages_tot.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_missed_packages_tot.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_missed_packages_tot.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_missed_packages_tot.common().member_flags().IS_OPTIONAL(false);
    mst_missed_packages_tot.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_missed_packages_tot.common().member_flags().IS_KEY(false);
    mst_missed_packages_tot.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_missed_packages_tot.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("uint32_t", false));

    MD5 missed_packages_tot_hash("missed_packages_tot");
    for(int i = 0; i < 4; ++i)
    {
        mst_missed_packages_tot.detail().name_hash()[i] = missed_packages_tot_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_missed_packages_tot);

    MinimalStructMember mst_missed_packages_curr;
    mst_missed_packages_curr.common().member_id(memberId++);
    mst_missed_packages_curr.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_missed_packages_curr.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_missed_packages_curr.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_missed_packages_curr.common().member_flags().IS_OPTIONAL(false);
    mst_missed_packages_curr.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_missed_packages_curr.common().member_flags().IS_KEY(false);
    mst_missed_packages_curr.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_missed_packages_curr.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("uint32_t", false));

    MD5 missed_packages_curr_hash("missed_packages_curr");
    for(int i = 0; i < 4; ++i)
    {
        mst_missed_packages_curr.detail().name_hash()[i] = missed_packages_curr_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_missed_packages_curr);

    MinimalStructMember mst_sequence_id_curr;
    mst_sequence_id_curr.common().member_id(memberId++);
    mst_sequence_id_curr.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_sequence_id_curr.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_sequence_id_curr.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_sequence_id_curr.common().member_flags().IS_OPTIONAL(false);
    mst_sequence_id_curr.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_sequence_id_curr.common().member_flags().IS_KEY(false);
    mst_sequence_id_curr.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_sequence_id_curr.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("uint32_t", false));

    MD5 sequence_id_curr_hash("sequence_id_curr");
    for(int i = 0; i < 4; ++i)
    {
        mst_sequence_id_curr.detail().name_hash()[i] = sequence_id_curr_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_sequence_id_curr);

    MinimalStructMember mst_sequence_id_prec;
    mst_sequence_id_prec.common().member_id(memberId++);
    mst_sequence_id_prec.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    mst_sequence_id_prec.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    mst_sequence_id_prec.common().member_flags().IS_EXTERNAL(false); // Unsupported
    mst_sequence_id_prec.common().member_flags().IS_OPTIONAL(false);
    mst_sequence_id_prec.common().member_flags().IS_MUST_UNDERSTAND(false);
    mst_sequence_id_prec.common().member_flags().IS_KEY(false);
    mst_sequence_id_prec.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    mst_sequence_id_prec.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("uint32_t", false));

    MD5 sequence_id_prec_hash("sequence_id_prec");
    for(int i = 0; i < 4; ++i)
    {
        mst_sequence_id_prec.detail().name_hash()[i] = sequence_id_prec_hash.digest[i];
    }
    type_object->minimal().struct_type().member_seq().emplace_back(mst_sequence_id_prec);


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

    TypeObjectFactory::get_instance()->add_type_object("NetStatMsg", &identifier, type_object);
    delete type_object;
    return TypeObjectFactory::get_instance()->get_type_object("NetStatMsg", false);
}

const TypeObject* GetCompleteNetStatMsgObject()
{
    const TypeObject* c_type_object = TypeObjectFactory::get_instance()->get_type_object("NetStatMsg", true);
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
    CompleteStructMember cst_delta_timestamp;
    cst_delta_timestamp.common().member_id(memberId++);
    cst_delta_timestamp.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_delta_timestamp.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_delta_timestamp.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_delta_timestamp.common().member_flags().IS_OPTIONAL(false);
    cst_delta_timestamp.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_delta_timestamp.common().member_flags().IS_KEY(false);
    cst_delta_timestamp.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_delta_timestamp.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("double", false));

    cst_delta_timestamp.detail().name("delta_timestamp");

    type_object->complete().struct_type().member_seq().emplace_back(cst_delta_timestamp);

    CompleteStructMember cst_missed_packages_tot;
    cst_missed_packages_tot.common().member_id(memberId++);
    cst_missed_packages_tot.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_missed_packages_tot.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_missed_packages_tot.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_missed_packages_tot.common().member_flags().IS_OPTIONAL(false);
    cst_missed_packages_tot.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_missed_packages_tot.common().member_flags().IS_KEY(false);
    cst_missed_packages_tot.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_missed_packages_tot.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("uint32_t", false));

    cst_missed_packages_tot.detail().name("missed_packages_tot");

    type_object->complete().struct_type().member_seq().emplace_back(cst_missed_packages_tot);

    CompleteStructMember cst_missed_packages_curr;
    cst_missed_packages_curr.common().member_id(memberId++);
    cst_missed_packages_curr.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_missed_packages_curr.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_missed_packages_curr.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_missed_packages_curr.common().member_flags().IS_OPTIONAL(false);
    cst_missed_packages_curr.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_missed_packages_curr.common().member_flags().IS_KEY(false);
    cst_missed_packages_curr.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_missed_packages_curr.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("uint32_t", false));

    cst_missed_packages_curr.detail().name("missed_packages_curr");

    type_object->complete().struct_type().member_seq().emplace_back(cst_missed_packages_curr);

    CompleteStructMember cst_sequence_id_curr;
    cst_sequence_id_curr.common().member_id(memberId++);
    cst_sequence_id_curr.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_sequence_id_curr.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_sequence_id_curr.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_sequence_id_curr.common().member_flags().IS_OPTIONAL(false);
    cst_sequence_id_curr.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_sequence_id_curr.common().member_flags().IS_KEY(false);
    cst_sequence_id_curr.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_sequence_id_curr.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("uint32_t", false));

    cst_sequence_id_curr.detail().name("sequence_id_curr");

    type_object->complete().struct_type().member_seq().emplace_back(cst_sequence_id_curr);

    CompleteStructMember cst_sequence_id_prec;
    cst_sequence_id_prec.common().member_id(memberId++);
    cst_sequence_id_prec.common().member_flags().TRY_CONSTRUCT1(false); // Unsupported
    cst_sequence_id_prec.common().member_flags().TRY_CONSTRUCT2(false); // Unsupported
    cst_sequence_id_prec.common().member_flags().IS_EXTERNAL(false); // Unsupported
    cst_sequence_id_prec.common().member_flags().IS_OPTIONAL(false);
    cst_sequence_id_prec.common().member_flags().IS_MUST_UNDERSTAND(false);
    cst_sequence_id_prec.common().member_flags().IS_KEY(false);
    cst_sequence_id_prec.common().member_flags().IS_DEFAULT(false); // Doesn't apply
    cst_sequence_id_prec.common().member_type_id(*TypeObjectFactory::get_instance()->get_type_identifier("uint32_t", false));

    cst_sequence_id_prec.detail().name("sequence_id_prec");

    type_object->complete().struct_type().member_seq().emplace_back(cst_sequence_id_prec);


    // Header
    type_object->complete().struct_type().header().detail().type_name("NetStatMsg");
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

    TypeObjectFactory::get_instance()->add_type_object("NetStatMsg", &identifier, type_object);
    delete type_object;
    return TypeObjectFactory::get_instance()->get_type_object("NetStatMsg", true);
}
