#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

namespace interceptor {

class StereoDepthProcessor : public rclcpp::Node {
public:
    StereoDepthProcessor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("stereo_depth_processor", options) {
        
        RCLCPP_INFO(get_logger(), "Stereo Depth Processor started");
        
        // Load parameters
        declare_parameter("stereo.baseline", 0.12);
        declare_parameter("stereo.fx", 535.4);
        declare_parameter("stereo.fy", 535.4);
        declare_parameter("stereo.cx", 320.5);
        declare_parameter("stereo.cy", 240.5);
        declare_parameter("num_disparities", 128);
        declare_parameter("block_size", 11);
    }
};

} // namespace interceptor

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<interceptor::StereoDepthProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
