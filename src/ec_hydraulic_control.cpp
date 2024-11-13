#include "pub_dds.hpp"
#include "sub_dds.hpp"
#include "messages/ecat_robot_refPubSubTypes.h"
#include "messages/ecat_robot_statePubSubTypes.h"
#include "messages/ecat_robot_net_statPubSubTypes.h"
#include "messages/ecat_control_refPubSubTypes.h"
#include "custom_pid.hpp"
#include "net_stat.hpp"
#include "trajectory_generator.hpp"
#include "torque_compensation.hpp"
#include "utils.hpp"

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

#include "utils/ec_common_step.h"
#include <test_common.h>

#include <fstream>
#include <experimental/filesystem>
#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <Eigen/Dense>

#define PUMP_PRE_OP 0x01
#define PUMP_OP 0x02
#define NUM_JOINTS 12

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
        int64_t sleep_ns=0, min_sleep_ns=10000;
        float time_elapsed_ms;

        // -- Robot data
        struct RobotState{ // actual robot state
            std::array<double, NUM_JOINTS> joints_position {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // linear position for hydraulic actuation
            std::array<double, NUM_JOINTS> joints_velocity{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, NUM_JOINTS> joints_acceleration{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, NUM_JOINTS> joints_torques{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // force for hydraulic actuation
            std::array<double, NUM_JOINTS> joints_torques_from_current{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // torque estimanted from current
	        std::array<double, NUM_JOINTS> joints_temperature{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double,NUM_JOINTS> motors_current{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // motor side
	        std::array<double, NUM_JOINTS> preassure1{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // preassures are only for hydraulic actuators
	        std::array<double, NUM_JOINTS> preassure2{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // preassures are only for hydraulic actuators
            std::array<uint32_t, NUM_JOINTS> fault,rtt,op_idx_ack; // only for electric motors
            std::array<uint32_t, NUM_JOINTS> cmd_aux_sts,brake_sts,led_sts; // only for electric motors
            std::array<double,NUM_JOINTS> link_pos; // only for electric motors
            std::array<double,NUM_JOINTS> link_vel; // only for electric motors
            std::array<double,NUM_JOINTS> board_temp; // only for electric motors
            std::array<double,NUM_JOINTS> aux; // only for electric motors
        } state;
        struct RobotRef{ // desired robot state
            std::array<double, NUM_JOINTS> kp_torque{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, NUM_JOINTS> ki_torque{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, NUM_JOINTS> kd_torque{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, NUM_JOINTS> kp_position{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, NUM_JOINTS> ki_position{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, NUM_JOINTS> kd_position{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, NUM_JOINTS> current{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; // motor side
            std::array<double, NUM_JOINTS> position{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, NUM_JOINTS> torque{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            std::array<double, NUM_JOINTS> current_offset{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
        } ref;

        // -- Configuration files
        YAML::Node ecat_config = YAML::LoadFile(ec_common_step.get_ec_utils()->get_ec_cfg_file());
        YAML::Node id_map = YAML::LoadFile(ecat_config["control"]["id_map_path"].as<std::string>());
        YAML::Node torque_comp_config = YAML::LoadFile(ecat_config["control"]["torque_compensation"].as<std::string>());

        // -- ID mapping: from ecat to hal and viceversa
        std::map<int,int> ecat_to_hal_id;
        std::map<int,int> hal_to_ecat_id;
        SSI slave_info;
        client->retrieve_slaves_info(slave_info);
        for (const auto &[ecat_id, type, pos] : slave_info){
            ecat_to_hal_id.insert({ecat_id, id_map["ecat_to_hal_id"][std::to_string(ecat_id)].as<int>()});
            hal_to_ecat_id.insert({ecat_to_hal_id[ecat_id], ecat_id});
        }
        // -- Initialize torque compensation module
        std::array<TorqueCompensation, NUM_JOINTS> torque_comp;
        for(int i=0; i<NUM_JOINTS; i++){
            torque_comp[i].init(torque_comp_config[std::to_string(i)].as<std::vector<double>>());
        }

        // -- Read
        client->read();

        // -- Electric motors
        MotorStatusMap motors_status_map;
        MotorReferenceMap motors_ref;
        client->get_motors_status(motors_status_map);

        // --- initialize motor references
        // --- position offset (is the angle when the hand-stop of the flywheel is at the middle)
        double position_offset = 0.0;
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
            position_offset = std::get<1>(rx_pdo);
            ref.position[ecat_to_hal_id[esc_id]] = position_offset;
        }

        // --- motory type
        const std::string motor_type="circulo";
        // ---- torque constant and reduction ratio
        double torque_constant = 1.0;
        if(motor_type=="circulo"){        
            torque_constant = 0.397394;
        }
        else if (motor_type=="advr"){
            torque_constant = 0.281;
        }
        const double reduction_ratio = 15;
        // --- torque filtering
        const double alpha = 1 - exp(-2.0 * M_PI * 50 / 1000.0);
        double torque_filtered = 0.0;

        // if(motors_ref.empty()){
        //     throw std::runtime_error("fatal error: motors reference is empty");
        // }

        // -- Hydraulic motors
        // ValveStatusMap valves_status_map;
        // ValveReferenceMap valves_ref;
        // client->get_valve_status(valves_status_map);
        // // --- initialize valve references
        // for ( const auto &[esc_id, rx_pdo] : valves_status_map){
        //     valves_ref[esc_id] = std::make_tuple(0,0,0,0,0,0,0,0);
        // }
        // if(valves_ref.empty()){
        //     throw std::runtime_error("fatal error: valves reference is empty");
        // }
        
        // -- PID
        std::array<CustomPID, NUM_JOINTS> pid_torque;  // these are forces in case of hydraulic actuation
        for(auto &pid : pid_torque){
            // dt, kp_limit, ki_limit, kd_limit, pid_limit, error_i_limit
            pid.init(ec_cfg.period_ms*0.001, 5.0, 1.75, 5.0, 5.0, 2000.0);
        }
        std::array<CustomPID, NUM_JOINTS> pid_position;
        for(auto &pid : pid_position){
            // dt, kp_limit, ki_limit, kd_limit, pid_limit, error_i_limit
            pid.init(ec_cfg.period_ms*0.001, 20.0, 0.1, 20.0, 20.0, 0.1);
        }
        double current_max = ecat_config["control"]["current_max"].as<double>();
        // std::array<double, NUM_JOINTS> current_offset = ecat_config["control"]["current_offset"].as<std::array<double, NUM_JOINTS>>();
        std::array<double, NUM_JOINTS> slaves_curr_ref{0,0,0,0,0,0,0,0,0,0,0,0};

        // -- Network statistics
        NetworkStatistics net_stat;

        // Define trajectories
        std::array<TrajectoryGenerator, NUM_JOINTS> traj_gen;
        for(auto &t : traj_gen){
            t.init(1,1,0,ec_cfg.period_ms*0.001);
        }
        std::array<std::string, NUM_JOINTS> traj_mode;
        std::array<bool, NUM_JOINTS> start_traj;
        for(int i=0; i<traj_mode.size();i++){
            traj_mode[i] = "torque";
            start_traj[i] = false;
        }        

        // -- DDS publisher and subscriber
        PubDDS<EcatRobotStateMsg, EcatRobotStateMsgPubSubType> pub_dds("ec_robot_state");
        pub_dds.init(false);
        // SubDDS<EcatRobotRefMsg, EcatRobotRefMsgPubSubType> sub_dds("ec_robot_ref");
        SubDDS<EcatControlRefMsg, EcatControlRefMsgPubSubType> sub_dds("ec_robot_ref");
        sub_dds.init(false);
        PubDDS<NetStatMsg, NetStatMsgPubSubType> pub_dds_net_stat("ec_robot_net_stat");
        pub_dds_net_stat.init(false);
        // --- messages
        EcatRobotStateMsg state_msg;
        // EcatRobotRefMsg ref_msg;
        EcatControlRefMsg ref_msg;

        // Test file
        std::array<std::string, NUM_JOINTS> test_file_names;
        std::array<std::ofstream, NUM_JOINTS> test_files;
        std::array<std::string,NUM_JOINTS> test_data;
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
        // if (ec_cfg.protocol == "iddp"){
        //     DPRINTF("Real-time process....\n");
        //     // add SIGALRM
        //     main_common(&argc, (char *const **)&argv, 0);
        //     int priority = SCHED_OTHER;
        //     #if defined(PREEMPT_RT) || defined(__COBALT__)
        //         priority = sched_get_priority_max ( SCHED_FIFO ) / 3;
        //     #endif
        //     int ret = set_main_sched_policy(priority);
        //     if (ret < 0){
        //         throw std::runtime_error("fatal error on set_main_sched_policy");
        //     }
        // }

        // start_time_ns= iit::ecat::get_time_ns();
        // time_ns=start_time_ns;
        // read desired torque profile

        std::vector<std::vector<double>> torque_profile = read_csv_double(ecat_config["control"]["torque_profile"].as<std::string>());
        std::vector<std::vector<double>> position_profile = read_csv_double(ecat_config["control"]["position_profile"].as<std::string>());
        const std::string position_profile_joint = "haa";
        // variable used to read torque profile data from csv file
        int profile_row = 0;
        int hal_id = 0;
        // time when the command are sent: used to save data in the csv files

        double sending_start_time = iit::ecat::get_time_ns(CLOCK_MONOTONIC);
        double sending_time = 0.0;
        
        while (run && client->is_client_alive())
        {
            start_time_ns = iit::ecat::get_time_ns(CLOCK_MONOTONIC);

            client->read();
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
                    if(motor_type=="circulo"){
                        state.joints_torques[hal_id] = -4.7 + state.joints_torques[hal_id];
                        state.joints_torques_from_current[hal_id] = state.aux[hal_id];//*torque_constant*reduction_ratio; //torque from current (in case of advr it is the current directly, so needs to be multiplied by the torque constant (0.281) and the reduction ratio (15));
                        state.motors_current[hal_id] = (1/(torque_constant*reduction_ratio))*state.joints_torques_from_current[hal_id]; //advr: state.aux[i], circulo: 1/(torque_constant*reduction_ratio)*state.aux[i]
                        // PRINT OUT Brakes and LED get_motors_status @ NOTE To be tested.         
                            
                    }
                    else if (motor_type=="advr"){
                        state.joints_torques[hal_id] = -3.6 + state.joints_torques[hal_id];
                        state.joints_torques_from_current[hal_id] = state.aux[hal_id]*torque_constant*reduction_ratio; //torque from current (in case of advr it is the current directly, so needs to be multiplied by the torque constant (0.281) and the reduction ratio (15));
                        state.motors_current[hal_id] = (1/(torque_constant*reduction_ratio))*state.joints_torques_from_current[hal_id];
                    }
                    // PRINT OUT Brakes and LED get_motors_status @ NOTE To be tested.         
                    state.brake_sts[hal_id] = state.cmd_aux_sts[hal_id] & 3; //00 unknown
                                                //01 release brake 
                                                //10 enganged brake  
                                                //11 error
                    state.led_sts[hal_id]= (state.cmd_aux_sts[hal_id] & 4)/4; // 1 or 0 LED  ON/OFF
                } catch (std::out_of_range oor) {}
            }
            // -- valves
            // client->get_valve_status(valves_status_map);
            // for ( const auto &[ecat_id, rx_pdo] : valves_status_map){
            //     // raw data is in micrometer (1000um = 1mm), here we convert to meter
            //     hal_id = ecat_to_hal_id[ecat_id];
            //     state.joints_position[hal_id] =      std::get<0>(rx_pdo)/1000000.0;
            //     state.joints_torques[hal_id] =       std::get<1>(rx_pdo)-105;//-22327.5;
            //     state.preassure1[hal_id] =           std::get<2>(rx_pdo);
            //     state.preassure2[hal_id] =           std::get<3>(rx_pdo);
            //     state.joints_temperature[hal_id] =   std::get<4>(rx_pdo);
            // }

            // Get references
            ref_msg = sub_dds.getMsg();
            

            // Update trajectory
            for(int i=0; i<traj_gen.size();i++){
                traj_gen[i].amplitude = ref_msg.amplitude()[i];
                traj_gen[i].offset = ref_msg.offset()[i];
                traj_gen[i].traj_type = ref_msg.traj_type()[i];
                traj_gen[i].max_freq = ref_msg.max_freq()[i];
                traj_mode[i] = ref_msg.traj_mode()[i];
                start_traj[i] = ref_msg.start_traj()[i];
                if(start_traj[i] == false){
                    traj_gen[i].reset();
                }
                // getting desired frequency from the message only if the trajectory is not of chirp type
                if(!(traj_gen[i].traj_type == "chirp")) 
                    traj_gen[i].frequency = ref_msg.frequency()[i];

                // generate trajectory
                if(start_traj[i] && traj_mode[i] == "position"){
                    ref.position[i] = position_offset + traj_gen[i].run();
                }
                else if (start_traj[i] && traj_mode[i] == "torque"){
                    ref.torque[i] = traj_gen[i].run();
                }
                if  (start_traj[i] &&
                    traj_gen[i].traj_type=="chirp" && 
                    (traj_gen[i].frequency==traj_gen[i].max_freq && fabs(1.0/traj_gen[i].frequency-traj_gen[i].chirp_time)<0.001))
                {
                    std::cout << "Chirp test finished." << std::endl;
                    start_traj[i] = false;

                }
            }

            for ( int i=0; i<ref.position.size();i++){
                // ref.position[i] = ref_msg.position_ref()[i];
                // ref.torque[i] = ref_msg.torque_ref()[i];
                ref.current_offset[i] = ref_msg.current_offset()[i];
                // Set PID gains
                pid_torque[i].setGains(ref_msg.kp_torque()[i], ref_msg.ki_torque()[i], ref_msg.kd_torque()[i]);
                pid_position[i].setGains(ref_msg.kp_position()[i], ref_msg.ki_position()[i], ref_msg.kd_position()[i]);
            }

            // Joint PID control
            // for(int i = 0; i< slaves_curr_ref.size(); i++){
            //     slaves_curr_ref[i] =     pid_position[i].run(ref.position[i], state.joints_position[i])
            //                         +   pid_torque[i].run(ref.torque[i], state.joints_torques[i])
            //                         +   ref.current_offset[i];

            //     // saturate output
            //     if (slaves_curr_ref[i] >= current_max){
            //         slaves_curr_ref[i]  = current_max;
            //     }
            //     else if (slaves_curr_ref[i] <= -current_max){
            //         slaves_curr_ref[i]  = -current_max;
            //     }
            // }

            // Send desired torques to slave (0.281Nm: torque constant of Circulo (default value))            
            for ( auto &[ecat_id, rx_pdo] : motors_ref){
                // for advr driver, setting tor_ref in control mode, let the low level understand that tor_ref is actually a current. So there is no need to multiply the current with the torque constant
                // double compensed_torque = torque_comp[ecat_to_hal_id[ecat_id]].run(state.joints_temperature[ecat_to_hal_id[ecat_id]], state.joints_velocity[ecat_to_hal_id], ref.torque[ecat_to_hal_id]);
                // TODO: use real temperature and motor velocity
                // double compensed_torque = torque_comp[ecat_to_hal_id[ecat_id]].run(20, 0, ref.torque[ecat_to_hal_id[ecat_id]]);
                // pub_dds.msg.preassure1()[ecat_to_hal_id[ecat_id]] = compensed_torque;// TEMP

                // torque_filtered = (1-alpha)*torque_filtered + alpha*compensed_torque;
                // std::get<3>(rx_pdo) = compensed_torque;
                // std::get<3>(rx_pdo) = torque_filtered;


                // desired torque from torque profile
                // if(profile_row<torque_profile.size()){
                //     ref.torque[ecat_to_hal_id[ecat_id]] = torque_profile[profile_row][1]; // 1: hfe des_torque, 2: haa des_torque, 
                // }
                // else{
                //     ref.torque[ecat_to_hal_id[ecat_id]] = 0.0;
                // }
                // std::get<3>(rx_pdo) = ref.torque[ecat_to_hal_id[ecat_id]];

                // torque compensation
                // double compensed_torque = torque_comp[ecat_to_hal_id[ecat_id]].run(20, 0, ref.torque[ecat_to_hal_id[ecat_id]]);
                // std::get<3>(rx_pdo) = compensed_torque;

                // torque filtered
                // torque_filtered = (1-alpha)*torque_filtered + alpha*compensed_torque;
                // std::get<3>(rx_pdo) = torque_filtered;

                // torque with offset
                std::get<3>(rx_pdo) = ref.torque[ecat_to_hal_id[ecat_id]] + ref.current_offset[ecat_to_hal_id[ecat_id]];
                
                // torque scale factor identification
                // std::get<3>(rx_pdo) = ref.torque[ecat_to_hal_id[ecat_id]]*ref_msg.torque_scale_factor()[ecat_to_hal_id[ecat_id]];

                // use simply the torque reference
                // std::get<3>(rx_pdo) = ref.torque[ecat_to_hal_id[ecat_id]];

                // **** POSITION CONTROL ****
                //  --- desired position from position profile ---
                // if(profile_row<position_profile.size()){
                //     if(position_profile_joint=="hfe"){ //-1 is the offset added to the position profile
                //         ref.position[ecat_to_hal_id[ecat_id]] = position_offset -1 + position_profile[profile_row][0]; // 0: hfe, 1: haa 
                //     }
                //     else if (position_profile_joint=="haa"){
                //         ref.position[ecat_to_hal_id[ecat_id]] = position_offset + position_profile[profile_row][1]; // 0: hfe, 1: haa 
                //     } 
                // }
                // std::get<1>(rx_pdo) = ref.position[ecat_to_hal_id[ecat_id]];
                
                // --- default position control ---
                // std::get<1>(rx_pdo) = ref.position[ecat_to_hal_id[ecat_id]];

                ref.current[ecat_to_hal_id[ecat_id]] = (1/(torque_constant*reduction_ratio)) * std::get<3>(rx_pdo);
            }
            profile_row++;
            client->set_motors_references(RefFlags::FLAG_MULTI_REF, motors_ref);
            
            // -- valves
            // for ( auto &[ecat_id, rx_pdo] : valves_ref){
            //     std::get<0>(rx_pdo) = slaves_curr_ref[ecat_to_hal_id[ecat_id]];
            // }
            // client->set_valves_references(RefFlags::FLAG_MULTI_REF, valves_ref);

            client->write();
            sending_time = iit::ecat::get_time_ns(CLOCK_MONOTONIC)-sending_start_time;

            client->log();

            // Publish dds messages
            for(int i=0; i<state.joints_position.size();i++){
                // state
                pub_dds.msg.joints_position()[i]      = state.joints_position[i];
                pub_dds.msg.joints_torques()[i]       = state.joints_torques[i];
                pub_dds.msg.joints_velocity()[i]      = state.joints_velocity[i];
                pub_dds.msg.preassure1()[i]           = state.preassure1[i];
                pub_dds.msg.preassure2()[i]           = state.preassure2[i];
                pub_dds.msg.joints_temperature()[i]   = state.joints_temperature[i];
                pub_dds.msg.joints_torques_from_current()[i] = state.joints_torques_from_current[i];
                pub_dds.msg.motors_current()[i] = state.motors_current[i];
                
                // references
                pub_dds.msg.position_ref()[i]         =  ref.position[i];
                pub_dds.msg.torque_ref()[i]           =  ref.torque[i];
                pub_dds.msg.current_offset()[i]       =  ref.current_offset[i];
                pub_dds.msg.current_ref()[i]          =  ref.current[i];
                pub_dds.msg.chirp_freq()[i] = traj_gen[i].frequency;
                pub_dds.msg.torque_scale_factor()[i] = ref_msg.torque_scale_factor()[i];

                // save test data
                test_data[i]+=std::to_string(sending_time/1000000000.0)+","+std::to_string(state.joints_position[i])+","+std::to_string(state.motors_current[i])+","+std::to_string(state.joints_torques[i])+","+std::to_string(state.joints_torques_from_current[i])+","+std::to_string(state.preassure1[i])+","+std::to_string(state.preassure2[i])+","+std::to_string(state.joints_temperature[i])+","+std::to_string(ref.current[i])+","+std::to_string(ref.torque[i])+","+std::to_string(ref.position[i])+"\n";
            }

            // compute network statistics

            pub_dds_net_stat.msg.sequence_id_curr() = ref_msg.sequence_id();
            pub_dds_net_stat.msg.sequence_id_prec() = net_stat.sequence_id_prec;
            net_stat.run(ref_msg.sequence_id(), ref_msg.timestamp());

            pub_dds_net_stat.msg.delta_timestamp() = net_stat.delta_timestamp;
            pub_dds_net_stat.msg.missed_packages_tot() = net_stat.missed_packages_tot;
            pub_dds_net_stat.msg.missed_packages_curr() = net_stat.missed_packages_curr;

            pub_dds.publish();
            pub_dds_net_stat.publish();

            sleep_ns = static_cast<uint64_t>(ec_cfg.period_ms*1'000'000-(iit::ecat::get_time_ns(CLOCK_MONOTONIC)-start_time_ns));

            sleep_ns = std::max(min_sleep_ns, sleep_ns);
            ts.tv_nsec=sleep_ns;
            while(clock_nanosleep(CLOCK_MONOTONIC, 0, &ts,NULL) == -1 && errno == EINTR)
            {} 
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
            test_files[i] << "time, position,motor_current,torque_sensor,torque_from_current,preassure1,preassure2,temperature,current_ref,torque_ref,position_ref\n";//,kp,ki,kd\n";
            test_files[i] << test_data[i];
            test_files[i].close();
        }
    }
    
    // Stop slaves
    ec_common_step.stop_ec_motors();
    ec_common_step.stop_ec_valves();
    ec_common_step.stop_ec();
    
    return 0;
}
