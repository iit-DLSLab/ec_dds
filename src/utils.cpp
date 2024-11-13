#include "utils.hpp"
#include <fstream>

eprosima::fastrtps::rtps::Locator_t createServerLocator(double domain){
    const std::string server_ip = "192.168.2.3";//"10.240.22.66";
    const int server_port = 56540+domain;
    eprosima::fastrtps::rtps::Locator_t server_locator;
    eprosima::fastrtps::rtps::IPLocator::setIPv4(server_locator, server_ip);
    eprosima::fastrtps::rtps::IPLocator::setPhysicalPort(server_locator, server_port);
    server_locator.kind = LOCATOR_KIND_UDPv4;
    return server_locator;
}

std::string createServerGUIDPrefix(double domain){
    return "44.53.00.5f.45.50.52.4f.53.49.4d.4"+std::to_string(domain);
}

void configureParticipantAsClient(double domain, eprosima::fastdds::dds::DomainParticipantQos& participant_qos){
    // Configure participant as client
    participant_qos.wire_protocol().builtin.discovery_config.discoveryProtocol = eprosima::fastrtps::rtps::DiscoveryProtocol_t::CLIENT;
    // Add the server locator in the metatraffic unicast locator list of the remote server attributes
    eprosima::fastrtps::rtps::Locator_t server_locator = createServerLocator(domain);
    eprosima::fastrtps::rtps::RemoteServerAttributes remote_server_attr;
    remote_server_attr.metatrafficUnicastLocatorList.push_back(server_locator);
    // Set the GUID prefix to identify the server
    remote_server_attr.ReadguidPrefix(createServerGUIDPrefix(domain).c_str());
    // Connect to the remote server
    participant_qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_attr);
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