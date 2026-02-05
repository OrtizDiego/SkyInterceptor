#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>

namespace interceptor {

class StereoSyncNode : public rclcpp::Node {
public:
    StereoSyncNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("stereo_sync_node", options) {
        
        RCLCPP_INFO(get_logger(), "Stereo Sync Node started");
        
        // Declare parameters
        declare_parameter("sync_slop", 0.005);
        
        // Setup subscribers
        left_image_sub_.subscribe(this, "/stereo/left/image_raw");
        right_image_sub_.subscribe(this, "/stereo/right/image_raw");
        left_info_sub_.subscribe(this, "/stereo/left/camera_info");
        right_info_sub_.subscribe(this, "/stereo/right/camera_info");
        
        // Setup synchronizer
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), left_image_sub_, right_image_sub_, 
            left_info_sub_, right_info_sub_);
        sync_->registerCallback(std::bind(&StereoSyncNode::syncedCallback, this, 
            std::placeholders::_1, std::placeholders::_2, 
            std::placeholders::_3, std::placeholders::_4));
    }

private:
    using Image = sensor_msgs::msg::Image;
    using CameraInfo = sensor_msgs::msg::CameraInfo;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        Image, Image, CameraInfo, CameraInfo>;
    
    message_filters::Subscriber<Image> left_image_sub_;
    message_filters::Subscriber<Image> right_image_sub_;
    message_filters::Subscriber<CameraInfo> left_info_sub_;
    message_filters::Subscriber<CameraInfo> right_info_sub_;
    
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    void syncedCallback(
        const Image::ConstSharedPtr& left_img,
        const Image::ConstSharedPtr& right_img,
        const CameraInfo::ConstSharedPtr& left_info,
        const CameraInfo::ConstSharedPtr& right_info) {
        
        RCLCPP_DEBUG(get_logger(), "Received synchronized stereo pair");
        // For now, just pass through - depth processor will subscribe to these
    }
};

} // namespace interceptor

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<interceptor::StereoSyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
