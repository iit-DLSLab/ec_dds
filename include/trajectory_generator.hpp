#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include <string>

class TrajectoryGenerator
{
public:
    TrajectoryGenerator();
    
    ~TrajectoryGenerator(){};

    void init(double amplitude, double frequency, double offset, double dt);
    void initSine();
    void initSquare();
    void initChirp();

    void reset();
    double run();

    double amplitude;
    double frequency;
    double offset;
    double angle;
    double dt; //seconds
    double max_freq;
    std::string traj_type;
    double chirp_time;
};
#endif