/**
 * @file pub_dds.cpp
 *
 */

#include "pub_dds.hpp"
#include "utils.hpp"
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>

#include <thread>

template<class MsgType, class Topic>
PubDDS<MsgType, Topic>::PubDDS(const std::string& topic_name)
    : participant_(nullptr)
    , publisher_(nullptr)
    , topic_(nullptr)
    , writer_(nullptr)
    , type_(new Topic())
    , topic_name_(topic_name)
{
}

template<class MsgType, class Topic>
bool PubDDS<MsgType, Topic>::init(
        bool use_env)
{
    eprosima::fastdds::dds::DomainParticipantQos pqos = eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
    pqos.name("Participant_pub");
    auto factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();

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
    type_->auto_fill_type_information(false);
    type_->auto_fill_type_object(true);
    participant_->register_type(type_);

    //CREATE THE PUBLISHER
    eprosima::fastdds::dds::PublisherQos pubqos = eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT;

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
    eprosima::fastdds::dds::TopicQos tqos = eprosima::fastdds::dds::TOPIC_QOS_DEFAULT;

    if (use_env)
    {
        participant_->get_default_topic_qos(tqos);
    }

    topic_ = participant_->create_topic(
        topic_name_,
        type_.get_type_name(),
        tqos);

    if (topic_ == nullptr)
    {
        return false;
    }

    // CREATE THE WRITER
    eprosima::fastdds::dds::DataWriterQos wqos = eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;

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

template<class MsgType, class Topic>
PubDDS<MsgType, Topic>::~PubDDS()
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
    eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
}

template<class MsgType, class Topic>
void PubDDS<MsgType, Topic>::PubListener::on_publication_matched(
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

template<class MsgType, class Topic>
bool PubDDS<MsgType, Topic>::publish(
        bool waitForListener)
{
    if (listener_.firstConnected_ || !waitForListener || listener_.matched_ > 0)
    {
        writer_->write(&msg);
        return true;
    }
    return false;
}