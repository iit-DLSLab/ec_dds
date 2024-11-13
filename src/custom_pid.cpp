#include "custom_pid.hpp"
#include <cmath>

CustomPID::CustomPID()
:   kp(0.0),
    ki(0.0),
    kd(0.0),
    error_p(0.0),
    error_i(0.0),
    error_d(0.0),
    error_p_old(0.0),
    error_p_gain(1.0),
    error_i_limit(0.0),
    output_kp_limit(0.0),
    output_ki_limit(0.0),
    output_kd_limit(0.0),
    output_limit(0.0),
    output_kp(0.0),
    output_ki(0.0),
    output_kd(0.0),
    output(0.0),
    dt(0.0),
    negative_pid(false)
{}

void CustomPID::init(double dt, double kp_limit, double ki_limit, double kd_limit, double pid_limit, double error_i_limit){
    this->dt = dt;
    this->output_kp_limit = kp_limit;
    this->output_ki_limit = ki_limit;
    this->output_kd_limit = kd_limit;
    this->output_limit = pid_limit;
    this->error_i_limit = error_i_limit;
}

double CustomPID::run(double demand, double feedback, double demand_ff){
    // Update Errors
    error_p = demand - feedback;
    error_p = error_p * error_p_gain;
    error_d = (error_p - error_p_old) * (1.0 / dt);
    error_p_old = error_p;
    error_i = error_i + error_p * dt;
    limitOutput(error_i, error_i_limit);

    // Compute Outputs
    output_kp = kp * error_p;
    output_ki = ki * error_i;
    output_kd = kd * error_d;
 
    // Limit Outputs
    limitOutput(output_kp, output_kp_limit);
    limitOutput(output_ki, output_ki_limit);
    limitOutput(output_kd, output_kd_limit);
    
    // // Compute and Limit Final Output
    output = output_kp + output_ki + output_kd; 
    limitOutput(output, output_limit);
    
    // if (!negative_pid){
    //     output = demand_ff + cstm_pid->output; 
    // }
    // else {
    //     output = demand_ff - cstm_pid->output; 
    // }
    return -output; //-(output+1.16);
}

void CustomPID::limitOutput(double& value, double limit){
    if (abs(limit) >= PID_OUTPUT_LIMIT_TOL){
        if(value > limit){
            value = limit;
        }
        else if(value < -1.0 * (limit)){
            value = -1.0 * (limit);
        }
    }
}

void CustomPID::setGains(double kp, double ki, double kd){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

double CustomPID::getKp(){return kp;}
double CustomPID::getKi(){return ki;}
double CustomPID::getKd(){return kd;}