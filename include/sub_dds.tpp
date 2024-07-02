/**
 * @file sub_dds.cpp
 *
 */

#include "sub_dds.hpp"

#include "utils.hpp"
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

template<class MsgType, class Topic>
SubDDS<MsgType, Topic>::SubDDS(const std::string& topic_name)
    : participant_(nullptr)
    , subscriber_(nullptr)
    , topic_(nullptr)
    , reader_(nullptr)
    , type_(new Topic())
    , topic_name_(topic_name)
{}

template<class MsgType, class Topic>
bool SubDDS<MsgType, Topic>::init(
        bool use_env)
{
    eprosima::fastdds::dds::DomainParticipantQos pqos = eprosima::fastdds::dds::PARTICIPANT_QOS_DEFAULT;
    pqos.name("Participant_sub");
    auto factory = eprosima::fastdds::dds::DomainParticipantFactory::get_instance();

    if (use_env)
    {
        factory->load_profiles();
        factory->get_default_participant_qos(pqos);
    }

    configureParticipantAsClient(3, pqos); //3: signal domain
    pqos.wire_protocol().builtin.typelookup_config.use_server = true;	
    participant_ = factory->create_participant(0, pqos);

    if (participant_ == nullptr)
    {
        return false;
    }

    //REGISTER THE TYPE
    type_.get()->auto_fill_type_information(false);
    type_.get()->auto_fill_type_object(true);
    type_.register_type(participant_);

    //CREATE THE SUBSCRIBER
    eprosima::fastdds::dds::SubscriberQos sqos = eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT;

    if (use_env)
    {
        participant_->get_default_subscriber_qos(sqos);
    }

    subscriber_ = participant_->create_subscriber(sqos, nullptr);

    if (subscriber_ == nullptr)
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

    // CREATE THE READER
    eprosima::fastdds::dds::DataReaderQos rqos = eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
    rqos.reliability().kind = eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;

    if (use_env)
    {
        subscriber_->get_default_datareader_qos(rqos);
    }

    reader_ = subscriber_->create_datareader(topic_, rqos, &listener_);

    if (reader_ == nullptr)
    {
        return false;
    }

    return true;
}

template<class MsgType, class Topic>
SubDDS<MsgType, Topic>::~SubDDS()
{
    if (reader_ != nullptr)
    {
        subscriber_->delete_datareader(reader_);
    }
    if (topic_ != nullptr)
    {
        participant_->delete_topic(topic_);
    }
    if (subscriber_ != nullptr)
    {
        participant_->delete_subscriber(subscriber_);
    }
    eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->delete_participant(participant_);
}

template<class MsgType, class Topic>
void SubDDS<MsgType, Topic>::SubListener::on_subscription_matched(
        eprosima::fastdds::dds::DataReader*,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus& info)
{
    if (info.current_count_change == 1)
    {
        matched_ = info.total_count;
        std::cout << "Subscriber matched." << std::endl;
    }
    else if (info.current_count_change == -1)
    {
        matched_ = info.total_count;
        std::cout << "Subscriber unmatched." << std::endl;
    }
    else
    {
        std::cout << info.current_count_change
                  << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
    }
}

template<class MsgType, class Topic>
void SubDDS<MsgType, Topic>::SubListener::on_data_available(
        eprosima::fastdds::dds::DataReader* reader)
{
    eprosima::fastdds::dds::SampleInfo info;
    if (reader->take_next_sample(&msg, &info) == ReturnCode_t::RETCODE_OK)
    {
        if (info.instance_state == eprosima::fastdds::dds::ALIVE_INSTANCE_STATE)
        {
            samples_++;
        }
    }
}

template<class MsgType, class Topic>
MsgType SubDDS<MsgType, Topic>::getMsg() const{
    return listener_.msg;
}
        