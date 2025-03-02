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

/**
 * @file pub_valve.cpp
 *
 */

#include "pub_valve.hpp"
#include "utils.hpp"
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>

#include <thread>

using namespace eprosima::fastdds::dds;

PubValve::PubValve()
    : participant_(nullptr)
    , publisher_(nullptr)
    , topic_(nullptr)
    , writer_(nullptr)
    , type_(new ECValveMsgPubSubType())
{
}

bool PubValve::init(
        bool use_env)
{
    valve_msg.timestamp(0);
    valve_msg.index(0);
    valve_msg.encoderPos(0.0);
    valve_msg.temp(0.0);
    valve_msg.pressure1(0.0);
    valve_msg.pressure2(0.0);
    valve_msg.torque(0.0);
    valve_msg.current_ref(0.0);
    valve_msg.position_ref(0.0);
    valve_msg.torque_ref(0.0);

    DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;
    pqos.name("Participant_pub");
    auto factory = DomainParticipantFactory::get_instance();

    if (use_env)
    {
        factory->load_profiles();
        factory->get_default_participant_qos(pqos);
    }

    // Use discovery server
    configureParticipantAsClient(3, pqos); //3: signal domain
    // Allow dynamic types to be sent
    pqos.wire_protocol().builtin.typelookup_config.use_server = true;

    eprosima::fastdds::dds::StatusMask mask;
    participant_ = factory->create_participant(0, pqos);

    if (participant_ == nullptr)
    {
        return false;
    }

    //REGISTER THE TYPE
    // dynamic type discovery setting
    type_->auto_fill_type_information(false);
    type_->auto_fill_type_object(true);
    participant_->register_type(type_);
    // type_.register_type(participant_);

    //CREATE THE PUBLISHER
    PublisherQos pubqos = PUBLISHER_QOS_DEFAULT;

    if (use_env)
    {
        participant_->get_default_publisher_qos(pubqos);
    }

    publisher_ = participant_->create_publisher(
        pubqos,
        nullptr);

    if (publisher_ == nullptr)
    {
        return false;
    }

    //CREATE THE TOPIC
    TopicQos tqos = TOPIC_QOS_DEFAULT;

    if (use_env)
    {
        participant_->get_default_topic_qos(tqos);
    }

    topic_ = participant_->create_topic(
        "ec_valve",
        "ECValveMsg",
        tqos);

    if (topic_ == nullptr)
    {
        return false;
    }

    // CREATE THE WRITER
    DataWriterQos wqos = DATAWRITER_QOS_DEFAULT;

    if (use_env)
    {
        publisher_->get_default_datawriter_qos(wqos);
    }

    writer_ = publisher_->create_datawriter(
        topic_,
        wqos,
        &listener_);

    if (writer_ == nullptr)
    {
        return false;
    }

    return true;
}

PubValve::~PubValve()
{
    if (writer_ != nullptr)
    {
        publisher_->delete_datawriter(writer_);
    }
    if (publisher_ != nullptr)
    {
        participant_->delete_publisher(publisher_);
    }
    if (topic_ != nullptr)
    {
        participant_->delete_topic(topic_);
    }
    DomainParticipantFactory::get_instance()->delete_participant(participant_);
}

void PubValve::PubListener::on_publication_matched(
        eprosima::fastdds::dds::DataWriter*,
        const eprosima::fastdds::dds::PublicationMatchedStatus& info)
{
    if (info.current_count_change == 1)
    {
        matched_ = info.total_count;
        firstConnected_ = true;
        std::cout << "Publisher matched." << std::endl;
    }
    else if (info.current_count_change == -1)
    {
        matched_ = info.total_count;
        std::cout << "Publisher unmatched." << std::endl;
    }
    else
    {
        std::cout << info.current_count_change
                  << " is not a valid value for PublicationMatchedStatus current count change" << std::endl;
    }
}

void PubValve::runThread(
        uint32_t samples,
        uint32_t sleep)
{
    // if (samples == 0)
    // {
    //     while (!stop_)
    //     {
    //         if (publish(false))
    //         {
    //             std::cout << "Index: " << valve.index() << " ID:" << valve.id() << " Encoder Pos: " << valve.encoderPos()  << " Pressure1: " << valve.pressure1() << \
    //             " Pressure2: " << valve.pressure2() << " Temp: " << valve.temp() << " Torque: " << valve.torque() << " SENT" << std::endl;
    //         }
    //         std::this_thread::sleep_for(std::chrono::milliseconds(sleep));
    //     }
    // }
    // else
    // {
    //     for (uint32_t i = 0; i < samples; ++i)
    //     {
    //         if (!publish())
    //         {
    //             --i;
    //         }
    //         else
    //         {
    //             std::cout << "Index: " << valve.index() << " ID:" << valve.id() << " Encoder Pos: " << valve.encoderPos()  << " Pressure1: " << valve.pressure1() << \
    //             " Pressure2: " << valve.pressure2() << " Temp: " << valve.temp() << " Torque: " << valve.torque() << " SENT" << std::endl;
    //         }
    //         std::this_thread::sleep_for(std::chrono::milliseconds(sleep));
    //     }
    // }
}

void PubValve::run(
        uint32_t samples,
        uint32_t sleep)
{
    stop_ = false;
    std::thread thread(&PubValve::runThread, this, samples, sleep);
    if (samples == 0)
    {
        std::cout << "Publisher running. Please press enter to stop the Publisher at any time." << std::endl;
        std::cin.ignore();
        stop_ = true;
    }
    else
    {
        std::cout << "Publisher running " << samples << " samples." << std::endl;
    }
    thread.join();
}

bool PubValve::publish(
        bool waitForListener)
{
    if (listener_.firstConnected_ || !waitForListener || listener_.matched_ > 0)
    {
        writer_->write(&valve_msg);
        return true;
    }
    return false;
}