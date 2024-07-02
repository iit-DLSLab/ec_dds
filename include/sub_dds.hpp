/**
 * @file sub_dds.hpp
 *
 */

#ifndef SUB_DDS_H
#define SUB_DDS_H

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastrtps/subscriber/SampleInfo.h>
#include <fastdds/dds/core/status/SubscriptionMatchedStatus.hpp>

/*!
* @brief Class implementing a generic Fast-DDS subscriber
* @tparam MsgType message type associated to the TopicDataType of the message
* @tparam Topic serialization and deserialization class. It must be of type TopicDataType, which is automatically generated from the idl file
*/
template<class MsgType, class Topic>
class SubDDS
{
public:

    SubDDS(const std::string& topic_name);

    virtual ~SubDDS();

    //!Initialize the subscriber
    bool init(
            bool use_env);

    class SubListener : public eprosima::fastdds::dds::DataReaderListener
    {
    public:

        SubListener()
            : matched_(0)
            , samples_(0)
        {
        }

        ~SubListener() override
        {
        }

        void on_data_available(
                eprosima::fastdds::dds::DataReader* reader) override;

        void on_subscription_matched(
                eprosima::fastdds::dds::DataReader* reader,
                const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) override;

        MsgType msg; 

        int matched_;

        uint32_t samples_;
    }
    listener_;

    MsgType getMsg() const;

private:

    eprosima::fastdds::dds::DomainParticipant* participant_;

    eprosima::fastdds::dds::Subscriber* subscriber_;

    eprosima::fastdds::dds::Topic* topic_;

    eprosima::fastdds::dds::DataReader* reader_;

    eprosima::fastdds::dds::TypeSupport type_;

    const std::string topic_name_;
};

#include "sub_dds.tpp"

#endif /* SUB_DDS_H */