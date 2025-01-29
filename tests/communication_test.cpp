
#include "messages/ecat_robot_statePubSubTypes.h"
#include "pub_dds.hpp"

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <unistd.h>
int main(int argc, char * const argv[])
{
    PubDDS<EcatRobotStateMsg, EcatRobotStateMsgPubSubType> pub_dds2("ec_robot_state");
    pub_dds2.init(false);
    for(int i=0; i<12;i++){
        // state
        pub_dds2.msg.joints_position()[i]      = 1;
        pub_dds2.msg.joints_torques()[i]       =1;
        pub_dds2.msg.joints_velocity()[i]      =1;
        pub_dds2.msg.preassure1()[i]           =1;
        pub_dds2.msg.preassure2()[i]           =1;
        pub_dds2.msg.joints_temperature()[i]   =1;
        pub_dds2.msg.joints_torques_from_current()[i] = 1;
        pub_dds2.msg.motors_current()[i] = 1;
        
        // references
        pub_dds2.msg.position_ref()[i]         =  1;
        pub_dds2.msg.torque_ref()[i]           =  1;
        pub_dds2.msg.current_offset()[i]       =  1;
        pub_dds2.msg.current_ref()[i]          =  1;
        pub_dds2.msg.chirp_freq()[i] = 1;
        pub_dds2.msg.torque_scale_factor()[i] = 1;

    }

    while(true){
        for(int i=0; i<12;i++){
            // state
            pub_dds2.msg.joints_position()[i]++;
        }
        pub_dds2.publish();
        usleep(500000);
    }
}