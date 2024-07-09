/**
 * @file pub_dds.h
 *
 */
#ifndef PUB_DDS_H
#define PUB_DDS_H

#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>

/*!
* @brief Class implementing a generic Fast-DDS publisher
* @tparam MsgType message type associated to the TopicDataType of the message
* @tparam Topic serialization and deserialization class. It must be of type TopicDataType, which is automatically generated from the idl file
*/
template<class MsgType, class Topic>
class PubDDS
{
public:

    PubDDS(const std::string& topic_name);

    virtual ~PubDDS();

    //!Initialize
    bool init(
            bool use_env);

    //!Publish a sample
    bool publish(bool waitForListener = false);

    MsgType msg;

private:

    eprosima::fastdds::dds::DomainParticipant* participant_;

    eprosima::fastdds::dds::Publisher* publisher_;

    eprosima::fastdds::dds::Topic* topic_;

    eprosima::fastdds::dds::DataWriter* writer_;

    bool stop_;

    class PubListener : public eprosima::fastdds::dds::DataWriterListener
    {
    public:

        PubListener()
            : matched_(0)
            , firstConnected_(false)
        {
        }

        ~PubListener() override
        {
        }

        void on_publication_matched(
                eprosima::fastdds::dds::DataWriter* writer,
                const eprosima::fastdds::dds::PublicationMatchedStatus& info) override;

        int matched_;

        bool firstConnected_;
    }
    listener_;

    eprosima::fastdds::dds::TypeSupport type_;

    const std::string topic_name_;
};

#include "pub_dds.tpp"

#endif /* PUB_DDS_H */