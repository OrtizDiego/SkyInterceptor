#include <rclcpp/rclcpp.hpp>

namespace interceptor {

class HectorInterfaceNode : public rclcpp::Node {
public:
    HectorInterfaceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("hector_interface_node", options) {
        RCLCPP_INFO(get_logger(), "Hector Interface Node started");
    }
};

} // namespace interceptor

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<interceptor::HectorInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
