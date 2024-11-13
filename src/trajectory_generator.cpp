#include "trajectory_generator.hpp"
#include <math.h>
#include <iostream>


TrajectoryGenerator::TrajectoryGenerator()
: amplitude(0.0)
, frequency (0.0)
, offset(0.0)
, dt(0.0)
, angle(0.0)
, max_freq(100)
, traj_type("sine")
{
}

void TrajectoryGenerator::reset(){
    angle = 0.0;
    frequency = 1.0;
    chirp_time = 0.0;
}

double TrajectoryGenerator::run(){
    angle += 2*M_PI*frequency*dt;
    if(traj_type=="sine"){
        return offset + amplitude*sin(angle);
    }
    else if (traj_type=="square"){
        double out = 0.0;
        if (amplitude*sin(angle)>=0)
            out = amplitude;
        else
            out = -amplitude;
        return offset + out;
    }
    else if(traj_type=="chirp"){
        chirp_time += dt;
        if(frequency<max_freq && fabs(1.0/frequency-chirp_time)<0.001){
            frequency += 1;
            chirp_time = 0;
            std::cout << "changing frequency to "<<frequency<<"\n";
        }
        return offset + amplitude*sin(angle);
    }
    else{
        std::cerr << "Unknown trajectory type, returing 0." << std::endl;
        return 0.0;
    }
}

void TrajectoryGenerator::init(double amplitude, double frequency, double offset, double dt){
    this->amplitude = amplitude;
    this->frequency = frequency;
    this->offset = offset;
    this->dt = dt;
    this->angle = 0.0;
}

void TrajectoryGenerator::initSine(){
    traj_type = "sine";
    reset();
}

void TrajectoryGenerator::initSquare(){
    traj_type = "square";
    reset();
}

void TrajectoryGenerator::initChirp(){
    traj_type = "chirp";
    reset();
    frequency = 1.0;
}