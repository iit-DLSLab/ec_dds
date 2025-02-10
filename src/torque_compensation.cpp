#include "torque_compensation.hpp"
#include <math.h>
#include <iostream>
#include <algorithm>  // For std::sort, std::unique

// data expressing data collected in the format: row=sample, column=temp, w, tau_des, alpha
TorqueCompensation::TorqueCompensation()
{}

void TorqueCompensation::init(  const std::vector<double>& tau_control,
                const std::vector<double>& w,
                const std::vector<double>& temperature){
    // Save tau control
    this->tau_control = tau_control;
    // Save table keys
    this->w = w;
    this->temperature = temperature;
    this->current.resize(tau_control.size(),0.0); // the current keys are estimated using the computeCompensationTable function
    // Create empty table
    for (double tau : tau_control){
        for (double w : w){
            for (double  temp: temperature){
                torque_comp_table[std::vector<double>{0.0,w,temp}] = 0.0;
            }
        }
    }
}

std::map<std::vector<double>,double> TorqueCompensation::getTable(){
    return torque_comp_table;
}

void TorqueCompensation::sortAndUniqueVec(std::vector<double>& vec){
    std::sort(vec.begin(), vec.end());
    auto last = std::unique(vec.begin(), vec.end());
    vec.erase(last, vec.end());
}

// double TorqueCompensation::run(double temp, double w, double tau_des){     
//     // if tau_des lower than lowest registered tau_des value, return same tau_des
//     if(tau_des<tau_des_keys[0])
//         return tau_des;
//     // if tau_des higher that highest regitered tau_des value, return tau_des compensed with alpha belonging to highest values
//     // TODO: manage this depending on the other variables (temp, w)
//     else if(tau_des>tau_des_keys[tau_des_keys.size()-1]){
//         std::vector<double> key {temp_keys[temp_keys.size()-1], w_keys[w_keys.size()-1], tau_des_keys[tau_des_keys.size()-1]};
//         return tau_des * torque_comp_table[key];
//     }
    
//     // find lower and upper bounds
//     auto temp_bounds = findLowerUpperBounds(temp, temp_keys);
//     auto w_bounds = findLowerUpperBounds(w, w_keys);
//     auto tau_des_bounds = findLowerUpperBounds(tau_des, tau_des_keys);
    
//     // std::cout << temp_bounds[0] << " " <<temp_bounds[1] << std::endl;
//     // std::cout << w_bounds[0] << " " <<w_bounds[1] << std::endl;
//     // std::cout << tau_des_bounds[0] << " " <<tau_des_bounds[1] << std::endl;
    
//     double yLLL = findCubeVertex(temp_bounds[0], w_bounds[0], tau_des_bounds[0]);
//     double yULL = findCubeVertex(temp_bounds[1], w_bounds[0], tau_des_bounds[0]);
//     double yLUL = findCubeVertex(temp_bounds[0], w_bounds[1], tau_des_bounds[0]);
//     double yUUL = findCubeVertex(temp_bounds[1], w_bounds[1], tau_des_bounds[0]);
//     double yLLU = findCubeVertex(temp_bounds[0], w_bounds[0], tau_des_bounds[1]);
//     double yULU = findCubeVertex(temp_bounds[1], w_bounds[0], tau_des_bounds[1]);
//     double yLUU = findCubeVertex(temp_bounds[0], w_bounds[1], tau_des_bounds[1]);
//     double yUUU = findCubeVertex(temp_bounds[1], w_bounds[1], tau_des_bounds[1]);
//     // std::cout << yLLL << " " << yULL << " " << yLUL << " " << yUUL << " " << yLLU << " " << yULU << " " << yLUU << " " << yUUU << std::endl;
//     // compensate desired torque
//     double alpha = trilinearInterpolate(temp_bounds[0], temp_bounds[1], w_bounds[0], w_bounds[1], tau_des_bounds[0], tau_des_bounds[1], yLLL, yULL, yLUL, yUUL, yLLU, yULU, yLUU, yUUU, temp, w, tau_des);
//     // std::cout << "alpha: " << alpha << std::endl;
//     return tau_des*alpha;
// }


// std::array<double,2> TorqueCompensation::findLowerUpperBounds(double value, std::vector<double> data){
//     std::array<double,2> bounds;
//     // limit case: value lower than first value in data --> lowerBound=upperBound=data[0]
//     if(value<data[0]){
//         bounds[0] = data[0];
//         bounds[1] = data[0];
//     }
//     // limit case: value higher than last value in data --> lowerBound=upperBound=data[len(data)]
//     else if(value>data[data.size()-1]){
//         bounds[0] = data[data.size()-1];
//         bounds[1] = data[data.size()-1];
//     }
//     // find bounds 
//     else{
//         for (double data_value : data){
//             // set lower bound for temp
//             if(data_value==value){
//                 bounds[0] = data_value;
//                 bounds[1] = data_value;
//                 break;
//             }
//             else if (data_value<value){
//                 bounds[0] = data_value;
//             }
//             else{
//                 bounds[1] = data_value;
//                 break;
//             }
//         }
//     }

//     // find bounds 
//     return bounds;
// }

// double TorqueCompensation::findCubeVertex(double temp_bound, double w_bound, double tau_des_bound){
//     return torque_comp_table[std::vector<double>{temp_bound, w_bound, tau_des_bound}];
// }

// // Function for linear interpolation between two points
// double TorqueCompensation::linearInterpolate(double xL, double xU, double yL, double yU, double x) {
//     // Handle division by zero when xL == xU by directly returning yL (or yU, since they are equal)
//     if (std::abs(xU - xL) < 1e-9) {  // Small tolerance for floating-point comparison
//         return yL;  // Or yU, since yL == yU in this case
//     }
//     return yL + ((x - xL) / (xU - xL)) * (yU - yL);
// }

// // Function for trilinear interpolation
// double TorqueCompensation::trilinearInterpolate(
//     double x1L, double x1U, double x2L, double x2U, double x3L, double x3U,  // The boundaries of x1, x2, x3
//     double yLLL, double yULL, double yLUL, double yUUL,  // Values at the corners for x3L
//     double yLLU, double yULU, double yLUU, double yUUU,  // Values at the corners for x3U
//     double x1, double x2, double x3)                    // The point where we want to interpolate
// {
//     // 1. Interpolate along x1 for the four points at x3 = x3L (lower plane)
//     double y1 = linearInterpolate(x1L, x1U, yLLL, yULL, x1);
//     double y2 = linearInterpolate(x1L, x1U, yLUL, yUUL, x1);

//     // 2. Interpolate along x2 at x1 between the interpolated points y1 and y2 (lower plane)
//     double y3 = linearInterpolate(x2L, x2U, y1, y2, x2);

//     // 3. Interpolate along x1 for the four points at x3 = x3U (upper plane)
//     double y4 = linearInterpolate(x1L, x1U, yLLU, yULU, x1);
//     double y5 = linearInterpolate(x1L, x1U, yLUU, yUUU, x1);

//     // 4. Interpolate along x2 at x1 between the interpolated points y4 and y5 (upper plane)
//     double y6 = linearInterpolate(x2L, x2U, y4, y5, x2);

//     // 5. Finally, interpolate along x3 between the results from the lower and upper planes
//     return linearInterpolate(x3L, x3U, y3, y6, x3);
// }