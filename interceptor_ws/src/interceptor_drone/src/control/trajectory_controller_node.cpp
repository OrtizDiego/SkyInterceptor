#include <rclcpp/rclcpp.hpp>

namespace interceptor {

class TrajectoryControllerNode : public rclcpp::Node {
public:
    TrajectoryControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("trajectory_controller_node", options) {
        RCLCPP_INFO(get_logger(), "Trajectory Controller Node started");
    }
};

} // namespace interceptor

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<interceptor::TrajectoryControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
