#ifndef CUSTOM_PID_HPP
#define CUSTOM_PID_HPP

#define PID_OUTPUT_LIMIT_TOL 0.00001F

class CustomPID{
public:
    CustomPID();

    void init(double dt, double kp_limit, double ki_limit, double kd_limit, double pid_limit, double error_i_limit);

    double run(double demand, double feedback, double demand_ff = 0.0);

    void setGains(double kp, double ki, double kd);
private:
    double kp;
    double ki;
    double kd;

    double error_p;
    double error_i;
    double error_d;
    double error_p_old;
    double error_p_gain;

    double error_i_limit;
    double output_kp_limit;
    double output_ki_limit;
    double output_kd_limit;
    double output_limit;

    double output_kp;
    double output_ki;
    double output_kd;
    double output;
    
    double dt;

    bool negative_pid;

    void limitOutput(double& value, double limit);
};

#endif