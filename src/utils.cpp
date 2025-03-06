#include "utils.hpp"
#include <fstream>

eprosima::fastdds::rtps::Locator_t createServerLocator(double domain){
    const std::string server_ip = "192.168.2.3";//"10.240.22.66";
    const int server_port = 11811+domain;
    eprosima::fastdds::rtps::Locator_t server_locator;
    eprosima::fastdds::rtps::IPLocator::setIPv4(server_locator, server_ip);
    eprosima::fastdds::rtps::IPLocator::setPhysicalPort(server_locator, server_port);
    server_locator.kind = LOCATOR_KIND_UDPv4;
    return server_locator;
}

void configureParticipantAsClient(double domain, eprosima::fastdds::dds::DomainParticipantQos& participant_qos){
    // Configure participant as client
    participant_qos.wire_protocol().builtin.discovery_config.discoveryProtocol = eprosima::fastdds::rtps::DiscoveryProtocol::CLIENT;
    // Add the server locator in the metatraffic unicast locator list of the remote server attributes
    eprosima::fastdds::rtps::Locator_t server_locator = createServerLocator(domain);
    participant_qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(server_locator);
}

std::vector<std::vector<double>> read_csv_double(const std::string& file_name)
{
    // File pointer
    std::fstream fin;

    // Open an existing file
    fin.open(file_name, std::ios::in);
    std::string row;

    std::vector<std::vector<double>> data;
    std::getline(fin, row); // neglect the first row as it might contain either the columns'name or their indexes
    while (std::getline(fin, row)) {
        // getting numbers in each column of the row
        std::stringstream row_ss(row);
        std::string num;
        std::vector<double> row_data;
        while(std::getline(row_ss, num, ','))
        {
            row_data.push_back(std::stod(num));
        }
        data.push_back(row_data);
    }
    fin.close();

    return data;
}