#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>

#include "utils/ec_common_step.h"
#include <test_common.h>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fstream>
#include "pub_dds.hpp"
#include "sub_dds.hpp"
#include "messages/ecat_valve_refPubSubTypes.h"
#include "messages/ecat_robot_statePubSubTypes.h"
#include "custom_pid.hpp"

#include <Eigen/Dense>

#define PUMP_PRE_OP 0x01
#define PUMP_OP 0x02

using namespace std::chrono;

static bool run = true;
// signal hander to handle CTRL+C
static void sig_handler(int sig) 
{
    printf("got signal %d\n", sig);
    run = false;
}

int main(int argc, char * const argv[])
{
    // Create ecat client
    EcUtils::EC_CONFIG ec_cfg;
    EcIface::Ptr client;
    EcCommonStep ec_common_step;
    
    try{
        ec_common_step.create_ec(client,ec_cfg);
    }catch(std::exception &ex){
        DPRINTF("%s\n",ex.what());
        return 1;
    }
    
    // Detect slaves and start motors
    bool sys_ctrl=true;
    try{
        ec_common_step.autodetection();
        // sys_ctrl=ec_common_step.start_ec_motors();
        // sys_ctrl &= ec_common_step.start_ec_valves();
    }catch(std::exception &ex){
        DPRINTF("%s\n",ex.what());
        return 1;
    }

    if(sys_ctrl)
    {        
        // MEMORY ALLOCATION
	    
        // -- CTRL+C handler
        struct sigaction sa;
        sa.sa_handler = sig_handler;
        sa.sa_flags = 0;  // receive will return EINTR on CTRL+C!
        sigaction(SIGINT,&sa, nullptr);

        // -- Time variables
        struct timespec ts= { 0, ec_cfg.period_ms*1000000}; //sample time
        uint64_t start_time_ns=0;
        uint64_t time_ns=0;
        float time_elapsed_ms;

        // -- Robot data
        struct RobotState{ // actual robot state
            std::array<double, 12> joints_position {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> joints_velocity{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> joints_acceleration{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> joints_torques{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	        std::array<double, 12> joints_temperature{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	        std::array<double, 4> preassure1{0.0,0.0,0.0,0.0};
	        std::array<double, 4> preassure2{0.0,0.0,0.0,0.0};
        } robot_state;
        struct RobotRef{ // desired robot state
            std::array<double, 12> kp_torque{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> ki_torque{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> kd_torque{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> kp_position{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> ki_position{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> kd_position{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> current_ref{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> position_ref{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> torque_ref{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> current_offset{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        } robot_ref;

        // -- Configuration files
        YAML::Node ecat_config = YAML::LoadFile(ec_common_step.get_ec_utils()->get_ec_cfg_file());
        YAML::Node id_map = YAML::LoadFile(ecat_config["control"]["id_map_path"].as<std::string>());

        // -- ID mapping: from ecat to hal and viceversa
        std::map<int,int> ecat_to_hal_id_map;
        std::map<int,int> hal_to_ecat_id_map;
        SSI slave_info;
        client->retrieve_slaves_info(slave_info);
        std::cout << "\n";
        for (const auto &[esc_id, type, pos] : slave_info){
            ecat_to_hal_id_map.insert({esc_id, id_map["ecat_to_hal_id_map"][std::to_string(esc_id)].as<int>()});
            hal_to_ecat_id_map.insert({ecat_to_hal_id_map[esc_id], esc_id});
        }
        
        for (auto const &[key, data] : ecat_to_hal_id_map){
            std::cout << std::to_string(key) << " " << std::to_string(data)<< "\n";
        }
        std::cout << "***********************\n";
        for (auto const &[key, data] : hal_to_ecat_id_map){
            std::cout << std::to_string(key) << " " << std::to_string(data)<< "\n";
        }
        // -- Electric motors
        MotorStatusMap motors_status_map;
        MotorReferenceMap motors_ref;

        // -- Hydraulic motors
        ValveStatusMap valve_status_map;
        ValveReferenceMap valves_ref;

        //valves references check
        for ( const auto &[esc_id, curr_ref] : valves_current_ref){
            valves_ref[esc_id]=std::make_tuple(current_ref(ecat_to_hal_id_map[esc_id]),0,0,0,0,0,0,0);
        }

        // -- PID
        CustomPID pid_valve_force;
        // dt, kp_limit, ki_limit, kd_limit, pid_limit, error_i_limit
        pid_valve_force.init(ec_cfg.period_ms*0.001, 5.0, 1.75, 5.0, 5.0, 2000.0);
        // pid_valve_force.setGains(ec_cfg.kp_valve, ec_cfg.ki_valve, ec_cfg.kd_valve);
        CustomPID pid_valve_position;
        pid_valve_position.init(ec_cfg.period_ms*0.001, 20.0, 0.1, 20.0, 20.0, 0.1);
        double max_current = 20.0;

        // -- DDS publisher and subscriber
        PubDDS<EcatRobotStateMsg, EcatRobotStateMsgPubSubType> pub_dds("ec_robot_state");
        pub_dds.init(false);
        SubDDS<ECValveRefMsg, ECValveRefMsgPubSubType> sub_valve_ref("ec_valve_ref");
        sub_valve_ref.init(false);
        ECValveRefMsg valve_ref_msg;

        // Test file
        std::ofstream test_file;
        std::string test_data("");

        // if(ec_cfg.protocol=="iddp"){
        //     DPRINTF("Real-time process....\n");
        //     // add SIGALRM
        //     main_common (&argc, (char*const**)&argv, 0);
        //     assert(set_main_sched_policy(10) >= 0);
        // }

        // start_time_ns= iit::ecat::get_time_ns();
        // time_ns=start_time_ns;
        
        // while (run && client->is_client_alive())
        // {
        //     time_elapsed_ms= (static_cast<float>((time_ns-start_time_ns))/1000000);
        //     //DPRINTF("Time [%f]\n",time_elapsed_ms);
            
        //     // Rx "SENSE"
            
        //     //******************* Valve Telemetry and References********
        //     // Read valve references
        //     valve_ref_msg = sub_valve_ref.getMsg();
        //     pid_valve_force.setGains(valve_ref_msg.kp_force(), valve_ref_msg.ki_force(), valve_ref_msg.kd_force());
        //     pid_valve_position.setGains(valve_ref_msg.kp_position(), valve_ref_msg.ki_position(), valve_ref_msg.kd_position());

        //     client->get_valve_status(valve_status_map);
        //     for ( const auto &[esc_id, valve_rx_pdo] : valve_status_map){
        //         // References
        //         // valves_current_ref[esc_id] = valve_ref_msg.current_ref();
        //         valves_position_ref[esc_id] = valve_ref_msg.position_ref();
        //         valves_torque_ref[esc_id] = valve_ref_msg.torque_ref();
        //         // smoothing valve force reference
        //         valves_torque_ref_filt[esc_id] = (1-filter_gain_force) * valves_torque_ref_filt[esc_id] + filter_gain_force*valves_torque_ref[esc_id];
                
        //         // Telemetry
        //         encoder_position =   std::get<0>(valve_rx_pdo)/1000000.0;// raw data is in micrometer (1000um = 1mm), here we convert to meter
        //         tor_valve =         std::get<1>(valve_rx_pdo)-22327.5;
        //         pressure1 =         std::get<2>(valve_rx_pdo);
        //         pressure2 =         std::get<3>(valve_rx_pdo);
        //         temperature =        std::get<4>(valve_rx_pdo);
        //     }
        //     valve_current_offset = valve_ref_msg.current_offset();
        //     //******************* Valve Telemetry ********
            
        //     // Valve control
        //     for ( const auto &[esc_id, valve_rx_pdo] : valve_status_map){
        //         valves_current_ref[esc_id] = pid_valve_position.run(valves_position_ref[esc_id], encoder_position) + pid_valve_force.run(valves_torque_ref_filt[esc_id], tor_valve) + valve_current_offset;

        //         // saturate output
        //         if (valves_current_ref[esc_id]>=max_current){
        //             valves_current_ref[esc_id] = max_current;
        //         }
        //         else if (valves_current_ref[esc_id]<=-max_current){
        //             valves_current_ref[esc_id] = -max_current;
        //         }
        //     }

        //     // ************************* SEND ALWAYS REFERENCES*********************************** //
        //     for ( const auto &[esc_id, curr_ref] : valves_current_ref){
        //         std::get<0>(valves_ref[esc_id]) = curr_ref;
        //     }
        //     client->set_valves_references(RefFlags::FLAG_MULTI_REF, valves_ref);
        //     // ************************* SEND ALWAYS REFERENCES*********************************** //
            
        //     // Send dds messages
        //     for ( const auto &[esc_id, valve_rx_pdo] : valve_status_map){
        //         // map data in message
        //         // -- telemetry
        //         pub_dds.msg.encoderPos(encoder_position);
        //         pub_dds.msg.torque(tor_valve);
        //         pub_dds.msg.pressure1(pressure1);
        //         pub_dds.msg.pressure2(pressure2);
        //         pub_dds.msg.temp(temperature);
        //         // -- references
        //         pub_dds.msg.current_ref(valves_current_ref[esc_id]);
        //         pub_dds.msg.position_ref(valve_ref_msg.position_ref());
        //         pub_dds.msg.torque_ref(valve_ref_msg.torque_ref());
        //         pub_dds.msg.current_offset(valve_ref_msg.current_offset());
        //         // publish data
        //         pub_dds.publish(false);

        //         // save test data
        //         test_data+=std::to_string(encoder_position)+","+std::to_string(tor_valve)+","+std::to_string(pressure1)+","+std::to_string(pressure2)+","+std::to_string(temperature)+","+std::to_string(valves_current_ref[esc_id])+","+std::to_string(valves_torque_ref_filt[esc_id])+","+std::to_string(valves_position_ref[esc_id])+"\n";//+","+std::to_string(pid_valve.kp)+","+std::to_string(pid_valve.ki)+","+std::to_string(pid_valve.kd);
        //     }

        //     clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL); 
        //     // get period ns
        //     time_ns = iit::ecat::get_time_ns();
        // }
    }
    
    ec_common_step.stop_ec_valves();
    ec_common_step.stop_ec();
    
    // save test data
    test_file.open ("test.csv");
    test_file << "encoderPos,force,preassure1,preassure2,temperature,current_ref,force_ref,position_ref\n";//,kp,ki,kd\n";
    test_file << test_data;
    test_file.close();
    return 0;
}
