#include <experimental/filesystem>

#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>

#include <algorithm>

#include "utils/ec_common_step.h"
#include <test_common.h>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fstream>
#include "pub_dds.hpp"
#include "sub_dds.hpp"
#include "messages/ecat_robot_refPubSubTypes.h"
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
    
    // Detect and start slaves
    bool sys_ctrl=true;
    try{
        ec_common_step.autodetection();
        sys_ctrl=ec_common_step.start_ec_motors();
        sys_ctrl &= ec_common_step.start_ec_valves();
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
            std::array<double, 12> joints_position {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // linear position for hydraulic actuation
            std::array<double, 12> joints_velocity{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> joints_acceleration{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> joints_torques{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // force for hydraulic actuation
	        std::array<double, 12> joints_temperature{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	        std::array<double, 12> preassure1{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // preassures are only for hydraulic actuators
	        std::array<double, 12> preassure2{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // preassures are only for hydraulic actuators
            std::array<uint32_t, 12> fault,rtt,op_idx_ack; // only for electric motors
            std::array<uint32_t, 12> cmd_aux_sts,brake_sts,led_sts; // only for electric motors
            std::array<double,12> link_pos; // only for electric motors
            std::array<double,12> link_vel; // only for electric motors
            std::array<double,12> board_temp; // only for electric motors
            std::array<double,12> aux; // only for electric motors
        } state;
        struct RobotRef{ // desired robot state
            std::array<double, 12> kp_torque{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> ki_torque{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> kd_torque{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> kp_position{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> ki_position{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> kd_position{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> current{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> position{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> torque{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, 12> current_offset{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        } ref;

        // -- Configuration files
        YAML::Node ecat_config = YAML::LoadFile(ec_common_step.get_ec_utils()->get_ec_cfg_file());
        YAML::Node id_map = YAML::LoadFile(ecat_config["control"]["id_map_path"].as<std::string>());

        // -- ID mapping: from ecat to hal and viceversa
        std::map<int,int> ecat_to_hal_id;
        std::map<int,int> hal_to_ecat_id;
        SSI slave_info;
        client->retrieve_slaves_info(slave_info);
        for (const auto &[ecat_id, type, pos] : slave_info){
            ecat_to_hal_id.insert({ecat_id, id_map["ecat_to_hal_id"][std::to_string(ecat_id)].as<int>()});
            hal_to_ecat_id.insert({ecat_to_hal_id[ecat_id], ecat_id});
        }

        // -- Electric motors
        MotorStatusMap motors_status_map;
        MotorReferenceMap motors_ref;
        client->get_motors_status(motors_status_map);

        // --- initialize motor references
        for ( const auto &[esc_id, rx_pdo] : motors_status_map){
            motors_ref[esc_id]=std::make_tuple(ec_cfg.motor_config_map[esc_id].control_mode_type, //ctrl_type
                                                0.0, //pos_ref
                                                0.0, //vel_ref
                                                0.0, //tor_ref
                                                ec_cfg.motor_config_map[esc_id].gains[0], //gain_1
                                                ec_cfg.motor_config_map[esc_id].gains[1], //gain_2
                                                ec_cfg.motor_config_map[esc_id].gains[2], //gain_3
                                                ec_cfg.motor_config_map[esc_id].gains[3], //gain_4
                                                ec_cfg.motor_config_map[esc_id].gains[4], //gain_5
                                                1, // op means NO_OP
                                                0, // idx
                                                0  // aux
                                                );
        }
        if(motors_ref.empty()){
            throw std::runtime_error("fatal error: motors reference is empty");
        }

        // -- Hydraulic motors
        ValveStatusMap valves_status_map;
        ValveReferenceMap valves_ref;
        client->get_valve_status(valves_status_map);
        // --- initialize valve references
        for ( const auto &[esc_id, rx_pdo] : valves_status_map){
            valves_ref[esc_id] = std::make_tuple(0,0,0,0,0,0,0,0);
        }
        if(valves_ref.empty()){
            throw std::runtime_error("fatal error: valves reference is empty");
        }
        
        // -- PID
        std::array<CustomPID, 12> pid_torque;  // these are forces in case of hydraulic actuation
        for(auto &pid : pid_torque){
            // dt, kp_limit, ki_limit, kd_limit, pid_limit, error_i_limit
            pid.init(ec_cfg.period_ms*0.001, 5.0, 1.75, 5.0, 5.0, 2000.0);
        }
        std::array<CustomPID, 12> pid_position;
        for(auto &pid : pid_position){
            // dt, kp_limit, ki_limit, kd_limit, pid_limit, error_i_limit
            pid.init(ec_cfg.period_ms*0.001, 20.0, 0.1, 20.0, 20.0, 0.1);
        }
        double current_max = ecat_config["control"]["current_max"].as<double>();
        std::array<double, 12> current_offset = ecat_config["control"]["current_offset"].as<std::array<double, 12>>();
        std::array<double, 12> slaves_curr_ref{0,0,0,0,0,0,0,0,0,0,0,0};

        // -- DDS publisher and subscriber
        PubDDS<EcatRobotStateMsg, EcatRobotStateMsgPubSubType> pub_dds("ec_robot_state");
        pub_dds.init(false);
        SubDDS<EcatRobotRefMsg, EcatRobotRefMsgPubSubType> sub_dds("ec_robot_ref");
        sub_dds.init(false);
        // --- messages
        EcatRobotStateMsg state_msg;
        EcatRobotRefMsg ref_msg;

        // Test file
        std::array<std::string, 12> test_file_names;
        std::array<std::ofstream, 12> test_files;
        std::array<std::string,12> test_data;
        test_data.fill("");
        for(int i=0; i< test_file_names.size(); i++){
            test_file_names[i] = id_map["id_to_name"][i].as<std::string>();
        }

        // Make this process REAL-TIME
        if(ec_cfg.protocol=="iddp"){
            DPRINTF("Real-time process....\n");
            // add SIGALRM
            main_common (&argc, (char*const**)&argv, 0);
            assert(set_main_sched_policy(10) >= 0);
        }

        start_time_ns= iit::ecat::get_time_ns();
        time_ns=start_time_ns;
        
        int hal_id = 0;
        while (run && client->is_client_alive())
        {
            time_elapsed_ms= (static_cast<float>((time_ns-start_time_ns))/1000000);
            //DPRINTF("Time [%f]\n",time_elapsed_ms);
            // Read robot state
            // -- motors
            client->get_motors_status(motors_status_map);
            for ( const auto &[ecat_id, rx_pdo] : motors_status_map){
                hal_id = ecat_to_hal_id[ecat_id];
                try {
                    std::tie(state.link_pos[hal_id],
                    state.joints_position[hal_id],
                    state.link_vel[hal_id],
                    state.joints_velocity[hal_id],
                    state.joints_torques[hal_id],
                    state.joints_temperature[hal_id],
                    state.board_temp[hal_id],
                    state.fault[hal_id],
                    state.rtt[hal_id],
                    state.op_idx_ack[hal_id],
                    state.aux[hal_id],
                    state.cmd_aux_sts[hal_id]) = rx_pdo;  
                    
                    // PRINT OUT Brakes and LED get_motors_status @ NOTE To be tested.         
                    state.brake_sts[hal_id] = state.cmd_aux_sts[hal_id] & 3; //00 unknown
                                                //01 release brake 
                                                //10 enganged brake  
                                                //11 error
                    state.led_sts[hal_id]= (state.cmd_aux_sts[hal_id] & 4)/4; // 1 or 0 LED  ON/OFF
                } catch (std::out_of_range oor) {}
            }
            // -- valves
            client->get_valve_status(valves_status_map);
            for ( const auto &[ecat_id, rx_pdo] : valves_status_map){
                // raw data is in micrometer (1000um = 1mm), here we convert to meter
                hal_id = ecat_to_hal_id[ecat_id];
                state.joints_position[hal_id] =      std::get<0>(rx_pdo)/1000000.0;
                state.joints_torques[hal_id] =       std::get<1>(rx_pdo)-105;//-22327.5;
                state.preassure1[hal_id] =           std::get<2>(rx_pdo);
                state.preassure2[hal_id] =           std::get<3>(rx_pdo);
                state.joints_temperature[hal_id] =   std::get<4>(rx_pdo);
            }

            // Get references
            ref_msg = sub_dds.getMsg();
            for ( int i=0; i<ref.position.size();i++){
                ref.position[i] = ref_msg.position_ref()[i];
                ref.torque[i] = ref_msg.torque_ref()[i];
                ref.current_offset[i] = ref_msg.current_offset()[i];
                // Set PID gains
                pid_torque[i].setGains(ref_msg.kp_torque()[i], ref_msg.ki_torque()[i], ref_msg.kd_torque()[i]);
                pid_position[i].setGains(ref_msg.kp_position()[i], ref_msg.ki_position()[i], ref_msg.kd_position()[i]);
            }

            // Joint PID control
            for(int i = 0; i< slaves_curr_ref.size(); i++){
                slaves_curr_ref[i] =     pid_position[i].run(ref.position[i], state.joints_position[i])
                                    +   pid_torque[i].run(ref.torque[i], state.joints_torques[i])
                                    +   current_offset[i];

                // saturate output
                if (slaves_curr_ref[i] >= current_max){
                    slaves_curr_ref[i]  = current_max;
                }
                else if (slaves_curr_ref[i] <= -current_max){
                    slaves_curr_ref[i]  = -current_max;
                }
            }

            // Send desired current to slaves
            // -- motors (for now controlling in position)
            for ( auto &[ecat_id, rx_pdo] : motors_ref){
                std::get<1>(rx_pdo) = ref.position[ecat_to_hal_id[ecat_id]];
            }
            client->set_motors_references(RefFlags::FLAG_MULTI_REF, motors_ref);
            // -- valves
            for ( auto &[ecat_id, rx_pdo] : valves_ref){
                std::get<0>(rx_pdo) = slaves_curr_ref[ecat_to_hal_id[ecat_id]];
            }
            client->set_valves_references(RefFlags::FLAG_MULTI_REF, valves_ref);
            
            // Publish dds messages
            for(int i=0; i<state.joints_position.size();i++){
                // state
                pub_dds.msg.joints_position()[i]      = state.joints_position[i];
                pub_dds.msg.joints_torques()[i]       = state.joints_torques[i];
                pub_dds.msg.preassure1()[i]           = state.preassure1[i];
                pub_dds.msg.preassure2()[i]           = state.preassure2[i];
                pub_dds.msg.joints_temperature()[i]   = state.joints_temperature[i];
                // references
                pub_dds.msg.position_ref()[i]         =  ref.position[i];
                pub_dds.msg.torque_ref()[i]           =  ref.torque[i];
                pub_dds.msg.current_offset()[i]       =  ref.current_offset[i];
                pub_dds.msg.current_ref()[i]          =  slaves_curr_ref[i];

                // save test data
                test_data[i]+=std::to_string(state.joints_position[i])+","+std::to_string(state.joints_torques[i])+","+std::to_string(state.preassure1[i])+","+std::to_string(state.preassure2[i])+","+std::to_string(state.joints_temperature[i])+","+std::to_string(slaves_curr_ref[i])+","+std::to_string(ref.torque[i])+","+std::to_string(ref.position[i])+"\n";
            }
            pub_dds.publish();

            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL); 
            // get period ns
            time_ns = iit::ecat::get_time_ns();
        }

        // save test data
        std::time_t t = std::time(nullptr);
        std::tm tm = *std::localtime(&t);
        std::ostringstream date_and_time_ss;
        date_and_time_ss << std::put_time(&tm, "%F_%T");
        for(int i=0; i<test_file_names.size(); i++){
            const std::string dirname = std::string(getenv("HOME"))+"/ec_dds_tests/"+ date_and_time_ss.str();
            std::experimental::filesystem::v1::create_directories(dirname);
            test_files[i].open (dirname+"/"+ test_file_names[i]+".csv");
            test_files[i] << "position,torque,preassure1,preassure2,temperature,current_ref,torque_ref,position_ref\n";//,kp,ki,kd\n";
            test_files[i] << test_data[i];
            test_files[i].close();
        }
    }
    
    // Stop slaves
    ec_common_step.stop_ec_valves();
    ec_common_step.stop_ec();
    
    return 0;
}
