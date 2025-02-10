#include "pub_dds.hpp"
#include "sub_dds.hpp"

#include "utils.hpp"

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include "messages/motor_statePubSubTypes.h"

#include "utils/ec_wrapper.h"
#include <test_common.h>

#include <fstream>
#include <experimental/filesystem>
#include <stdlib.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>

#define PUMP_PRE_OP 0x01
#define PUMP_OP 0x02
#define NUM_JOINTS 12

using namespace std::chrono;

static bool run = true;

// Create ecat client
EcUtils::EC_CONFIG ec_cfg;
EcIface::Ptr client;
EcWrapper ec_wrapper;

// Define table
std::vector<std::vector<double>> compensation_table;
enum class TableEntries {TEMPERATURE, CURRENT, W, COMPENSATION_FACTOR};

// ***MOTOR CONSTANTS***
constexpr int32_t motor_id = 1; // torque control
constexpr int32_t auxiliar_motor_id = 2; // velocity control
constexpr double torque_constant = 0.397394;
constexpr double reduction_ratio = 15;

// ***TEST TIME PER TABLE ENTRY***
constexpr double test_time = 5; //seconds

// Create dds publisher
PubDDS<MotorStateMsg, MotorStateMsgPubSubType> pub_dds("motor_state");

// signal hander to handle CTRL+C
static void sig_handler(int sig) 
{
    printf("got signal %d\n", sig);
    run = false;
}

void initCompensationTable(const YAML::Node& torque_comp_config){
    std::vector<double> tau_control = torque_comp_config["tau_control"].as<std::vector<double>>();
    std::vector<double> w = torque_comp_config["w"].as<std::vector<double>>();
    std::vector<double> temperature = torque_comp_config["temperature"].as<std::vector<double>>();
    // Allocate table
    std::vector<std::vector<double>> compensation_table;
    for (int i=0; i<tau_control.size(); i++){
        for (int j=0; j<w.size(); j++){
            for (int k=0; k<temperature.size(); k++){
                // Table filled with temperature, tau, w, compensation factor. The tau value will be substituted by the measured mean current associated to that desired torque
                compensation_table.push_back(std::vector<double>{temperature[k], tau_control[i], w[j], 0.0}); 
            }
        }
    }
}

void checkSetup(){
    if (motor_reference_map.size() < 2){
        DPRINTF("ERROR: at least two motors are needed for this test\n");
        return 1;
    }
    else{
        // Check motor controller type
        if (std::get<0>(motor_reference_map[motor_id])!=0xCC)
        {
            DPRINTF("ERROR: motor %d is not controlled in torque\n", motor_id);
            return 1;
        }
        if (std::get<0>(motor_reference_map[auxiliar_motor_id])!=0x3B)
        {
            DPRINTF("ERROR: motor %d is not controlled in position/velocity\n", auxiliar_motor_id);
            return 1;
        }
    }
}

void makeProcessRT(){
    // Make this process REAL-TIME
    if (ec_cfg.protocol == "iddp"){
        // add SIGALRM
        signal ( SIGALRM, sig_handler );
        //avoid map swap
        main_common (&argc, (char*const**)&argv, sig_handler);
    }
    else{
        struct sigaction sa;
        sa.sa_handler = sig_handler;
        sa.sa_flags = 0;  // receive will return EINTR on CTRL+C!
        sigaction(SIGINT,&sa, nullptr);
    }
    // process scheduling
    try{
        ec_wrapper.ec_self_sched(argv[0]);
    }catch(std::exception& e){
        throw std::runtime_error(e.what());
    }
}

void publishMessage(){
    int count_motor = 0
    for ( const auto &[esc_id, rx_pdo] : motor_status_map){
        // state
        pub_dds.msg.id()[count_motor]                       = esc_id;
        pub_dds.msg.position()[count_motor]                 = std::get<2>(motor_status_map[esc_id]);
        pub_dds.msg.velocity()[count_motor]                 = std::get<4>(motor_status_map[esc_id]);
        pub_dds.msg.torque()[count_motor]                   = std::get<5>(motor_status_map[esc_id]);
        pub_dds.msg.estimated_torque()[count_motor]         = std::get<6>(motor_status_map[esc_id]);
        pub_dds.msg.current()[count_motor]                  = std::get<6>(motor_status_map[esc_id])/(torque_constant*reduction_ratio);
        pub_dds.msg.temp()[count_motor]                     = std::get<7>(motor_status_map[esc_id]);
        
        // references
        if(esc_id==motor_id){
            pub_dds.msg.des_torque()[count_motor] =  std::get<3>(motor_reference_map[motor_id]);
            pub_dds.msg.des_velocity()[count_motor] = 0.0;
        }
        else if(esc_id==auxiliar_motor_id){
            pub_dds.msg.des_velocity()[count_motor] = std::get<2>(motor_reference_map[auxiliar_motor_id]);
            pub_dds.msg.des_torque()[count_motor] = 0.0;
        }
        count_motor++;
    }
    pub_dds.publish();
}

void saveCompensationTable(){
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);
    std::ostringstream date_and_time_ss;
    date_and_time_ss << std::put_time(&tm, "%F_%T");
    std::ofstream file;
    const std::string dirname = std::string(getenv("HOME"))+"/ec_dds_tests/compensation_table/"+ date_and_time_ss.str();
    std::experimental::filesystem::v1::create_directories(dirname);
    file.open (dirname+"/current_compensation_table.csv");
    file << "temperature,current, angular_velocity,comp_factor\n";//,kp,ki,kd\n";
    for (int i=0; i<compensation_table.size(); i++){
        for (int j=0; j<compensation_table[i].size(); j++){
            file << std::to_string(compensation_table[i][j]);
            if (j<compensation_table[i].size()-1)
                file << ",";
        }
        file << "\n";
    }
    file.close();
}

int main(int argc, char * const argv[])
{
    // Create ecat client
    try{
        ec_wrapper.create_ec(client, ec_cfg);
    }catch(std::exception &ex){
        DPRINTF("%s\n",ex.what());
        return 1;
    }

    // Detect and start slaves
    bool ec_sys_started = true;
    try{
        ec_sys_started = ec_wrapper.start_ec_sys();
    }
    catch (std::exception &ex){
        DPRINTF("%s\n", ex.what());
        return 1;
    }

    if(ec_sys_started)
    {        
        // MEMORY ALLOCATION
        int overruns = 0;

        // Check setup: needs two motors with ID 1 and 2: motor 1 is controlled in torque, motor 2 is controlled in position/velocity
        checkSetup();

        // Read table inpus from yaml file
        // Initialize compensation table
        initCompensationTable(YAML::LoadFile("/home/embedded/dls_ws/src/ec_dds/config/motor_config/hyqreal3/hyqreal3_torque_comp.yaml"));
        
        // Initialize dds publisher
        pub_dds.init();

        // Make this process REAL-TIME
        makeProcessRT();
    
        const auto period = std::chrono::nanoseconds(ec_cfg.period_ms * 1000000);

        // iterate over the table and build the table
        for (int i=0; i<compensation_table.size();i++){
            // put motors in default configuration - TODO
            // force desired temperature - TODO

            // START TEST
            // Get motor reference (torque) from the compensation table
            double tau = compensation_table[i][TableEntries::CURRENT];
            // Get auxilar motor reference (velocity)  from the compensation table
            double w = compensation_table[i][TableEntries::W];
            // Set motor reference (torque)
            std::get<3>(motor_reference_map[motor_id]) = tau;
            // Set auxilar motor reference (velocity)
            std::get<2>(motor_reference_map[auxiliar_motor_id]) = w;

            // the control inputs are applied for test_time seconds to collect current measurements
            auto start = std::chrono::high_resolution_clock::now();
            auto time = std::chrono::high_resolution_clock::now();
            double enlapsed_time = 0.0;
            int count_current_measurements=0;
            double current_measurement=0.0;

            while (run && client->get_client_status().run_loop && enlapsed_time<=test_time)
            {
                // Read motor state
                client->read();
                client->get_motor_status(motor_status_map);
                
                // collect current measurements
                count_current_measurements++;
                current_measurement+=std::get<6>(motor_status_map[motor_id]);
                
                client->set_motor_reference(motor_reference_map);

                client->write();
                ec_wrapper.log_ec_sys();

                // Publish dds messages
                publishMessage();
                
                // Timing
                time = time + period;
                const auto now = std::chrono::high_resolution_clock::now();

    #if defined(PREEMPT_RT) || defined(__COBALT__)
                // if less than threshold, print warning (only on rt threads)
                if (now > time && ec_cfg.protocol == "iddp"){
                    ++overruns;
                    DPRINTF("main process overruns: %d\n", overruns);
                }
    #endif
                std::this_thread::sleep_until(time);
                // compute current test time
                enlapsed_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start).count();
            }
            // a mean current is computed after data collection
            current_measurement = current_measurement/count_current_measurements;
            // substitute the tau value in the table with the mean current
            compensation_table[i][TableEntries::CURRENT] = current_measurement;
            // the compensation factor is computed using tau_control, the mean current + torque constant and reduction ratio
            compensation_table[i][TableEntries::COMPENSATION_FACTOR] = tau/(current_measurement*torque_constant*reduction_ratio);
        }
    }
    
    // Save the compensation table in a file
    saveCompensationTable();

    // Stop slaves
    ec_wrapper.stop_ec_sys();
    
    return 0;
}