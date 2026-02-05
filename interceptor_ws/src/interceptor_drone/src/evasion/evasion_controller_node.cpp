#include <rclcpp/rclcpp.hpp>

namespace interceptor {

class EvasionControllerNode : public rclcpp::Node {
public:
    EvasionControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("evasion_controller_node", options) {
        RCLCPP_INFO(get_logger(), "Evasion Controller Node started");
    }
};

} // namespace interceptor

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<interceptor::EvasionControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
