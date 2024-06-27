#include <stdlib.h>
#include <iostream>

#include <chrono>
#include <thread>

#include "utils/ec_common_step.h"
#include <test_common.h>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fstream>
#include "pub_valve.hpp"
#include "sub_valve_ref.hpp"
#include "custom_pid.hpp"

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
    EcUtils::EC_CONFIG ec_cfg;
    EcIface::Ptr client;
    EcCommonStep ec_common_step;
    
    try{
        ec_common_step.create_ec(client,ec_cfg);
    }catch(std::exception &ex){
        DPRINTF("%s\n",ex.what());
        return 1;
    }
    
    std::cout << ec_common_step.get_ec_utils()->get_ec_cfg_file() << std::endl;
    // DDS publisher publishing valve status
    PubValve pub_valve;
    pub_valve.init(false);
    // DDS reader getting desired valve references
    SubValveRef sub_valve_ref;
    sub_valve_ref.init(false);
    ECValveRefMsg valve_ref_msg;

    // Test file
    std::ofstream test_file;
    std::string test_data("");
    // memory allocation

    // std::vector<int> motor_id_vector;
    // for ( auto &[id, pos] : ec_cfg.homing_position ) {
    //     motor_id_vector.push_back(id);
    // }
    
    // if(motor_id_vector.empty()){
    //     DPRINTF("Got an homing position map\n");
    //     ec_common_step.stop_ec();
    //     return 1;
    // }
    
    // if(ec_cfg.trajectory.empty()){
    //     DPRINTF("Got an empty general trajectory map\n");
    //     ec_common_step.stop_ec();
    //     return 1;
    // }
    
    bool sys_ctrl=true;
    try{
        ec_common_step.autodetection();
        //sys_ctrl=ec_common_step.start_ec_motors(motor_id_vector);
        sys_ctrl &= ec_common_step.start_ec_valves(ec_cfg.valve_id);
    }catch(std::exception &ex){
        DPRINTF("%s\n",ex.what());
        return 1;
    }


#ifdef TEST_EXAMPLES
    sys_ctrl=true;
#endif 

    if(sys_ctrl)
    {        
        // MEMORY ALLOCATION
	    
        // set handler for CTRL+C
        struct sigaction sa;
        sa.sa_handler = sig_handler;
        sa.sa_flags = 0;  // receive will return EINTR on CTRL+C!
        sigaction(SIGINT,&sa, nullptr);

        struct timespec ts= { 0, ec_cfg.period_ms*1000000}; //sample time
        
        uint64_t start_time_ns=0;
        uint64_t time_ns=0;
        
        float time_elapsed_ms;
        // float hm_time_ms=ec_cfg.homing_time_sec*1000;
        // float trj_time_ms=ec_cfg.trajectory_time_sec*1000;
        // float pressure_time_ms=1000; // 2minutes.
        // float set_trj_time_ms=hm_time_ms;
        
        // std::string STM_sts;
        // bool run=true;
        // bool first_motor_RX=false;
        
        // std::map<int,double> q_set_trj=ec_cfg.homing_position;
        // std::map<int,double> q_ref,q_start,qdot;
        
        // // bool first_pump_RX=false;
        // uint8_t pump_pressure_ref=180; //bar
        // std::map<int,uint8_t> pumps_trj_1,pumps_set_trj;
        // std::map<int,uint8_t> pumps_set_ref,pumps_start,pumps_actual_read;
        // uint16_t pump_status_word;
        // int pump_req_op=PUMP_PRE_OP;
        // double error_pressure_set;
        // bool pump_req_sts,pump_in_pressure;
        
        // double valve_curr_ref=2.5; //mA
        // std::map<int,double> valves_trj_1,valves_trj_2,valves_set_zero,valves_set_trj;
        // std::map<int,double> valves_set_ref,valves_start;
        // int trajectory_counter=0;
       	// double tau, alpha;

        // Valve
        ValveStatusMap valve_status_map;
        float encoder_position,tor_valve;           
        float pressure1,pressure2,temperature;   
        ValveReferenceMap valves_ref;

        std::map<int,double> valves_current_ref;
        std::map<int,double> valves_current_limit;
        std::map<int,double> valves_position_ref;
        std::map<int,double> valves_torque_ref;
        std::map<int,double> valves_torque_ref_filt; //filtered force reference
        double valve_current_offset = 1.16;
        double filter_gain_force = 1;

        client->get_valve_status(valve_status_map);
    
        for ( const auto &[esc_id, valve_rx_pdo] : valve_status_map){
            valves_current_ref[esc_id] = 0.0;
            valves_position_ref[esc_id] = 0.0;
            valves_torque_ref[esc_id] = 0.0;
            valves_current_limit[esc_id] = 2.5;
        }

        //valves references check
        for ( const auto &[esc_id, curr_ref] : valves_current_ref){
            valves_ref[esc_id]=std::make_tuple(curr_ref,0,0,0,0,0,0,0);
            std::cout << "sending the valce referenes for valve: " << esc_id << std::endl;
        }

        // Valve PID
        CustomPID pid_valve_force;
        // dt, kp_limit, ki_limit, kd_limit, pid_limit, error_i_limit
        pid_valve_force.init(ec_cfg.period_ms*0.001, 5.0, 1.75, 5.0, 5.0, 2000.0);
        // pid_valve_force.setGains(ec_cfg.kp_valve, ec_cfg.ki_valve, ec_cfg.kd_valve);
        CustomPID pid_valve_position;
        pid_valve_position.init(ec_cfg.period_ms*0.001, 20.0, 0.1, 20.0, 20.0, 0.1);
        double max_current = 20.0;

        if(ec_cfg.protocol=="iddp"){
            DPRINTF("Real-time process....\n");
            // add SIGALRM
            main_common (&argc, (char*const**)&argv, 0);
            assert(set_main_sched_policy(10) >= 0);
        }

        start_time_ns= iit::ecat::get_time_ns();
        time_ns=start_time_ns;
        
        while (run && client->is_client_alive())
        {
            time_elapsed_ms= (static_cast<float>((time_ns-start_time_ns))/1000000);
            //DPRINTF("Time [%f]\n",time_elapsed_ms);
            
            // Rx "SENSE"
            
            //******************* Valve Telemetry and References********
            // Read valve references
            valve_ref_msg = sub_valve_ref.listener_.data;
            pid_valve_force.setGains(valve_ref_msg.kp_force(), valve_ref_msg.ki_force(), valve_ref_msg.kd_force());
            pid_valve_position.setGains(valve_ref_msg.kp_position(), valve_ref_msg.ki_position(), valve_ref_msg.kd_position());

            client->get_valve_status(valve_status_map);
            for ( const auto &[esc_id, valve_rx_pdo] : valve_status_map){
                // References
                // valves_current_ref[esc_id] = valve_ref_msg.current_ref();
                valves_position_ref[esc_id] = valve_ref_msg.position_ref();
                valves_torque_ref[esc_id] = valve_ref_msg.torque_ref();
                // smoothing valve force reference
                valves_torque_ref_filt[esc_id] = (1-filter_gain_force) * valves_torque_ref_filt[esc_id] + filter_gain_force*valves_torque_ref[esc_id];
                
                // Telemetry
                encoder_position =   std::get<0>(valve_rx_pdo)/1000000.0;// raw data is in micrometer (1000um = 1mm), here we convert to meter
                tor_valve =         std::get<1>(valve_rx_pdo)-22327.5;
                pressure1 =         std::get<2>(valve_rx_pdo);
                pressure2 =         std::get<3>(valve_rx_pdo);
                temperature =        std::get<4>(valve_rx_pdo);
            }
            valve_current_offset = valve_ref_msg.current_offset();
            //******************* Valve Telemetry ********
            
            // Valve control
            for ( const auto &[esc_id, valve_rx_pdo] : valve_status_map){
                // // Kp
                // valves_current_ref[esc_id] = (-1)*ec_cfg.kp_valve*(valves_torque_ref_filt[esc_id] - tor_valve);
                // // Ki
                
                // if (valves_current_ref[esc_id]>valves_current_limit[esc_id]){
                //     valves_current_ref[esc_id] = valves_current_limit[esc_id];
                // }
                // else if (valves_current_ref[esc_id]<-valves_current_limit[esc_id]){
                //     valves_current_ref[esc_id] = -valves_current_limit[esc_id];
                // }
                // valves_current_ref[esc_id] = pid_valve_force.run(valves_torque_ref_filt[esc_id], tor_valve) + valve_current_offset;
                valves_current_ref[esc_id] = pid_valve_position.run(valves_position_ref[esc_id], encoder_position) + pid_valve_force.run(valves_torque_ref_filt[esc_id], tor_valve) + valve_current_offset;

                // saturate output
                if (valves_current_ref[esc_id]>=max_current){
                    valves_current_ref[esc_id] = max_current;
                }
                else if (valves_current_ref[esc_id]<=-max_current){
                    valves_current_ref[esc_id] = -max_current;
                }
            }

            // ************************* SEND ALWAYS REFERENCES*********************************** //
            for ( const auto &[esc_id, curr_ref] : valves_current_ref){
                std::get<0>(valves_ref[esc_id]) = curr_ref;
            }
            client->set_valves_references(RefFlags::FLAG_MULTI_REF, valves_ref);
            // ************************* SEND ALWAYS REFERENCES*********************************** //
            
            // Send dds messages
            for ( const auto &[esc_id, valve_rx_pdo] : valve_status_map){
                // map data in message
                // -- telemetry
                pub_valve.valve_msg.encoderPos(encoder_position);
                pub_valve.valve_msg.torque(tor_valve);
                pub_valve.valve_msg.pressure1(pressure1);
                pub_valve.valve_msg.pressure2(pressure2);
                pub_valve.valve_msg.temp(temperature);
                // -- references
                // pub_valve.valve_msg.kp_force(valve_ref_msg.kp_force());
                // pub_valve.valve_msg.ki_force(valve_ref_msg.ki_force());
                // pub_valve.valve_msg.kd_force(valve_ref_msg.kd_force());
                // pub_valve.valve_msg.kp_position(valve_ref_msg.kp_position());
                // pub_valve.valve_msg.ki_position(valve_ref_msg.ki_position());
                // pub_valve.valve_msg.kd_position(valve_ref_msg.kd_position());
                pub_valve.valve_msg.current_ref(valves_current_ref[esc_id]);
                pub_valve.valve_msg.position_ref(valve_ref_msg.position_ref());
                pub_valve.valve_msg.torque_ref(valve_ref_msg.torque_ref());
                pub_valve.valve_msg.current_offset(valve_ref_msg.current_offset());
                // publish data
                pub_valve.publish(false);

                // save test data
                test_data+=std::to_string(encoder_position)+","+std::to_string(tor_valve)+","+std::to_string(pressure1)+","+std::to_string(pressure2)+","+std::to_string(temperature)+","+std::to_string(valves_current_ref[esc_id])+","+std::to_string(valves_torque_ref_filt[esc_id])+","+std::to_string(valves_position_ref[esc_id])+"\n";//+","+std::to_string(pid_valve.kp)+","+std::to_string(pid_valve.ki)+","+std::to_string(pid_valve.kd);
            }

            clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL); 
            // get period ns
            time_ns = iit::ecat::get_time_ns();
        }
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
