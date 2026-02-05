#include <rclcpp/rclcpp.hpp>

namespace interceptor {

class TargetTrackerNode : public rclcpp::Node {
public:
    TargetTrackerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("target_tracker_node", options) {
        RCLCPP_INFO(get_logger(), "Target Tracker Node started");
    }
};

} // namespace interceptor

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<interceptor::TargetTrackerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
