#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace interceptor {

// Detection structure
struct Detection {
    int class_id;
    float confidence;
    int x, y, width, height;
    float depth;
};

// Target state
struct TargetState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    Eigen::Matrix3d position_cov;
    Eigen::Matrix3d velocity_cov;
    double quality;
    bool is_valid;
};

// Guidance output
struct GuidanceOutput {
    Eigen::Vector3d acceleration_cmd;
    double navigation_constant;
    double closing_velocity;
    double time_to_go;
    double miss_distance;
    bool is_feasible;
};

// Vector3d to string for debugging
std::string vec3dToString(const Eigen::Vector3d& v);

// Clamp value
inline double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(value, max_val));
}

} // namespace interceptor
