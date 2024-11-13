#ifndef TORQUE_COMPENSATION_HPP
#define TORQUE_COMPENSATION_HPP

#include <array>
#include <vector>
#include <map>

class TorqueCompensation{
public:
    TorqueCompensation(const std::vector<double>& data);

    TorqueCompensation();

    void init(const std::vector<double>& data);

    double run(double temp, double w, double tau_des);

private:
    std::map<std::vector<double>,double> torque_comp_table; // in the map the data are sorted by its key in ascending order by default

    const int torque_comp_vars;

    std::vector<double> temp_keys;
    std::vector<double> w_keys;
    std::vector<double> tau_des_keys;
    
    void sortAndUniqueVec(std::vector<double>& vec);

    std::array<double,2> findLowerUpperBounds(double value, std::vector<double> data);


    double findCubeVertex(double temp_bound, double w_bound, double tau_des_bound);


    // Function for linear interpolation between two points
    double linearInterpolate(double xL, double xU, double yL, double yU, double x);

    // Function for trilinear interpolation
    double trilinearInterpolate(
        double x1L, double x1U, double x2L, double x2U, double x3L, double x3U,  // The boundaries of x1, x2, x3
        double yLLL, double yULL, double yLUL, double yUUL,  // Values at the corners for x3L
        double yLLU, double yULU, double yLUU, double yUUU,  // Values at the corners for x3U
        double x1, double x2, double x3);                    // The point where we want to interpolate
    };
#endif