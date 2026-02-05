#include <rclcpp/rclcpp.hpp>

namespace interceptor {

class Target3DLocalizer : public rclcpp::Node {
public:
    Target3DLocalizer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("target_3d_localizer", options) {
        RCLCPP_INFO(get_logger(), "Target 3D Localizer started");
    }
};

} // namespace interceptor

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<interceptor::Target3DLocalizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
