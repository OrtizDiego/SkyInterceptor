#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace interceptor {

class Parameters {
public:
    // Stereo camera parameters
    struct StereoParams {
        double baseline = 0.12;  // meters
        double fx = 535.4;
        double fy = 535.4;
        double cx = 320.5;
        double cy = 240.5;
    };
    
    // EKF parameters
    struct EKFParams {
        double process_noise_pos = 0.1;
        double process_noise_vel = 0.5;
        double process_noise_acc = 1.0;
        double measurement_noise_pos = 0.1;
    };
    
    // Guidance parameters
    struct GuidanceParams {
        double nav_constant_far = 4.0;
        double nav_constant_mid = 5.0;
        double max_acceleration = 20.0;  // m/s^2
        double terminal_range = 10.0;    // meters
        double mid_range = 50.0;         // meters
    };
    
    // Controller parameters
    struct ControllerParams {
        double kp_pos = 1.0;
        double ki_pos = 0.0;
        double kd_pos = 0.5;
        
        double kp_vel = 2.0;
        double ki_vel = 0.1;
        double kd_vel = 0.5;
        
        double max_velocity = 41.7;   // m/s (150 km/h)
        double max_altitude = 200.0;  // meters
        double min_altitude = 2.0;    // meters
    };
    
    // Load parameters from ROS node
    static void loadFromNode(rclcpp::Node* node);
    
    static StereoParams stereo;
    static EKFParams ekf;
    static GuidanceParams guidance;
    static ControllerParams controller;
};

} // namespace interceptor
