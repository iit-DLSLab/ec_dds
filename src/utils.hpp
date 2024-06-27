#ifndef HYQREAL3_UTILS_HPP
#define HYQREAL3_UTILS_HPP

#include <fastdds/dds/domain/DomainParticipant.hpp>

eprosima::fastrtps::rtps::Locator_t createServerLocator(double domain);
std::string createServerGUIDPrefix(double domain);
void configureParticipantAsClient(double domain, eprosima::fastdds::dds::DomainParticipantQos& participant_qos);

#endif /* end of include guard: HYQREAL3_UTILS_HPP */