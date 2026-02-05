#include "common/parameters.hpp"

namespace interceptor {

Parameters::StereoParams Parameters::stereo;
Parameters::EKFParams Parameters::ekf;
Parameters::GuidanceParams Parameters::guidance;
Parameters::ControllerParams Parameters::controller;

void Parameters::loadFromNode(rclcpp::Node* node) {
    // Stereo parameters
    node->declare_parameter("stereo.baseline", 0.12);
    node->declare_parameter("stereo.fx", 535.4);
    node->declare_parameter("stereo.fy", 535.4);
    node->declare_parameter("stereo.cx", 320.5);
    node->declare_parameter("stereo.cy", 240.5);
    
    stereo.baseline = node->get_parameter("stereo.baseline").as_double();
    stereo.fx = node->get_parameter("stereo.fx").as_double();
    stereo.fy = node->get_parameter("stereo.fy").as_double();
    stereo.cx = node->get_parameter("stereo.cx").as_double();
    stereo.cy = node->get_parameter("stereo.cy").as_double();
    
    // EKF parameters
    node->declare_parameter("ekf.process_noise_pos", 0.1);
    node->declare_parameter("ekf.process_noise_vel", 0.5);
    node->declare_parameter("ekf.process_noise_acc", 1.0);
    node->declare_parameter("ekf.measurement_noise_pos", 0.1);
    
    ekf.process_noise_pos = node->get_parameter("ekf.process_noise_pos").as_double();
    ekf.process_noise_vel = node->get_parameter("ekf.process_noise_vel").as_double();
    ekf.process_noise_acc = node->get_parameter("ekf.process_noise_acc").as_double();
    ekf.measurement_noise_pos = node->get_parameter("ekf.measurement_noise_pos").as_double();
    
    // Guidance parameters
    node->declare_parameter("guidance.nav_constant_far", 4.0);
    node->declare_parameter("guidance.nav_constant_mid", 5.0);
    node->declare_parameter("guidance.max_acceleration", 20.0);
    node->declare_parameter("guidance.terminal_range", 10.0);
    node->declare_parameter("guidance.mid_range", 50.0);
    
    guidance.nav_constant_far = node->get_parameter("guidance.nav_constant_far").as_double();
    guidance.nav_constant_mid = node->get_parameter("guidance.nav_constant_mid").as_double();
    guidance.max_acceleration = node->get_parameter("guidance.max_acceleration").as_double();
    guidance.terminal_range = node->get_parameter("guidance.terminal_range").as_double();
    guidance.mid_range = node->get_parameter("guidance.mid_range").as_double();
    
    // Controller parameters
    node->declare_parameter("controller.kp_pos", 1.0);
    node->declare_parameter("controller.ki_pos", 0.0);
    node->declare_parameter("controller.kd_pos", 0.5);
    node->declare_parameter("controller.kp_vel", 2.0);
    node->declare_parameter("controller.ki_vel", 0.1);
    node->declare_parameter("controller.kd_vel", 0.5);
    node->declare_parameter("controller.max_velocity", 41.7);
    node->declare_parameter("controller.max_altitude", 200.0);
    node->declare_parameter("controller.min_altitude", 2.0);
    
    controller.kp_pos = node->get_parameter("controller.kp_pos").as_double();
    controller.ki_pos = node->get_parameter("controller.ki_pos").as_double();
    controller.kd_pos = node->get_parameter("controller.kd_pos").as_double();
    controller.kp_vel = node->get_parameter("controller.kp_vel").as_double();
    controller.ki_vel = node->get_parameter("controller.ki_vel").as_double();
    controller.kd_vel = node->get_parameter("controller.kd_vel").as_double();
    controller.max_velocity = node->get_parameter("controller.max_velocity").as_double();
    controller.max_altitude = node->get_parameter("controller.max_altitude").as_double();
    controller.min_altitude = node->get_parameter("controller.min_altitude").as_double();
}

} // namespace interceptor
