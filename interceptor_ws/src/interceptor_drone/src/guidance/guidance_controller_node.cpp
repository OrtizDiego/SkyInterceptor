#include <rclcpp/rclcpp.hpp>

namespace interceptor {

class GuidanceControllerNode : public rclcpp::Node {
public:
    GuidanceControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("guidance_controller_node", options) {
        RCLCPP_INFO(get_logger(), "Guidance Controller Node started");
    }
};

} // namespace interceptor

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<interceptor::GuidanceControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
