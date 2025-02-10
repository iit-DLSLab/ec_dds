#include "torque_compensation.hpp"

#include <iostream>
#include <yaml-cpp/yaml.h>

int main(int argc, char * const argv[])
{
    // -- Configuration files
    // YAML::Node torque_comp_config = YAML::LoadFile("/home/embedded/dls_ws/src/ec_dds/config/motor_config/hyqreal3/hyqreal3_torque_comp.yaml");

    // -- Initialize torque compensation module (hp: joint 0)
    // TorqueCompensation torque_comp = torque_comp_config[std::to_string(0)].as<std::vector<double>>();

    // std::cout << torque_comp.run(6, 2, 5) << std::endl;
    // std::cout << torque_comp.run(60, 50, 100) << std::endl;
    TorqueCompensation torque_comp;
    std::vector<double> tau_control = {1, 2, 3};
    std::vector<double> w = {4, 5, 6};
    std::vector<double> temperature = {7, 8, 9};

    torque_comp.init(tau_control, w, temperature);

    std::map<std::vector<double>, double> table = torque_comp.getTable();
    for (auto const &pair : table)
    {
        std::cout << "Key: {";
        for (auto const &key : pair.first)
        {
            std::cout << key << ", ";
        }
        std::cout << "}, Value: " << pair.second << std::endl;
    }
}