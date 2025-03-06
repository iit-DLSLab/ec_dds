#ifndef HYQREAL3_UTILS_HPP
#define HYQREAL3_UTILS_HPP

#include <fastdds/dds/domain/DomainParticipant.hpp>

eprosima::fastdds::rtps::Locator_t createServerLocator(double domain);
void configureParticipantAsClient(double domain, eprosima::fastdds::dds::DomainParticipantQos& participant_qos);
std::vector<std::vector<double>> read_csv_double(const std::string& file_name);

#endif /* end of include guard: HYQREAL3_UTILS_HPP */