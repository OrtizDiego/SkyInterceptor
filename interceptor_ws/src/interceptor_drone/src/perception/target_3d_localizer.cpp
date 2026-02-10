/**
 * @file target_3d_localizer.cpp
 * @brief Turn 2D detections + Depth map into 3D world coordinates
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <interceptor_interfaces/msg/target_detection.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace interceptor {

class Target3DLocalizer : public rclcpp::Node {
public:
    Target3DLocalizer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("target_3d_localizer", options) {
        
        RCLCPP_INFO(get_logger(), "============================================");
        RCLCPP_INFO(get_logger(), "Target 3D Localizer Starting...");
        RCLCPP_INFO(get_logger(), "============================================");

        // Parameters
        declare_parameter("target_frame", "map");
        declare_parameter("sync_slop", 0.1); // 100ms tolerance
        target_frame_ = get_parameter("target_frame").as_string();
        double sync_slop = get_parameter("sync_slop").as_double();

        // TF2 Setup
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscriptions
        sub_detection_2d_.subscribe(this, "/target/detection_2d");
        sub_depth_.subscribe(this, "/stereo/depth");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(20), sub_detection_2d_, sub_depth_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop));
        sync_->registerCallback(std::bind(&Target3DLocalizer::sync_callback, this, 
                                        std::placeholders::_1, std::placeholders::_2));

        // Global Camera Info subscriber (to get fx, fy, cx, cy)
        sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/stereo/left/camera_info", 10,
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                camera_info_ = msg;
            });

        // Publisher
        pub_detection_3d_ = create_publisher<interceptor_interfaces::msg::TargetDetection>(
            "/target/detection_3d", 10);

        RCLCPP_INFO(get_logger(), "Localizer initialized. Target Frame: %s", target_frame_.c_str());
    }

private:
    using TargetDetection = interceptor_interfaces::msg::TargetDetection;
    using Image = sensor_msgs::msg::Image;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<TargetDetection, Image>;

    void sync_callback(const TargetDetection::ConstSharedPtr& det_msg,
                       const Image::ConstSharedPtr& depth_msg) {
        
        if (!camera_info_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for camera info...");
            return;
        }

        // 1. Get depth at detection center
        float depth = get_depth_at_bbox(det_msg, depth_msg);
        
        if (depth <= 0.1f || depth > 50.0f) {
            RCLCPP_DEBUG(get_logger(), "Invalid depth (%.2f) for detection", depth);
            return;
        }

        // 2. Project to 3D Camera Frame
        // X = (u - cx) * Z / fx
        // Y = (v - cy) * Z / fy
        // Z = depth
        double u = det_msg->bbox_x + (det_msg->bbox_width / 2.0);
        double v = det_msg->bbox_y + (det_msg->bbox_height / 2.0);
        
        double fx = camera_info_->k[0];
        double cx = camera_info_->k[2];
        double fy = camera_info_->k[4];
        double cy = camera_info_->k[5];

        geometry_msgs::msg::PointStamped pt_camera;
        pt_detection_camera_ = det_msg; // Cache for use
        pt_camera.header = det_msg->header;
        pt_camera.point.z = depth;
        pt_camera.point.x = (u - cx) * depth / fx;
        pt_camera.point.y = (v - cy) * depth / fy;

        // 3. Transform to World Frame (Map)
        geometry_msgs::msg::PointStamped pt_world;
        try {
            // Transform point from camera frame to target frame (map)
            pt_world = tf_buffer_->transform(pt_camera, target_frame_, tf2::durationFromSec(0.1));
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(get_logger(), "TF2 Transform Error: %s", ex.what());
            return;
        }

        // 4. Publish 3D Detection
        TargetDetection det_3d = *det_msg;
        det_3d.position_camera = pt_camera.point;
        det_3d.position_world = pt_world.point;
        det_3d.world_position_valid = true;
        det_3d.depth_meters = depth;
        
        pub_detection_3d_->publish(det_3d);
        
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "Target Localized: [x: %.2f, y: %.2f, z: %.2f] in %s frame",
            det_3d.position_world.x, det_3d.position_world.y, det_3d.position_world.z,
            target_frame_.c_str());
    }

    float get_depth_at_bbox(const TargetDetection::ConstSharedPtr& det, 
                           const Image::ConstSharedPtr& depth_img) {
        
        auto cv_ptr = cv_bridge::toCvShare(depth_img, "32FC1");
        
        // Use a small 5x5 window around center to average depth (more robust than single pixel)
        int cx = det->bbox_x + (det->bbox_width / 2);
        int cy = det->bbox_y + (det->bbox_height / 2);
        
        int window = 2;
        float sum = 0;
        int count = 0;
        
        for (int i = -window; i <= window; ++i) {
            for (int j = -window; j <= window; ++j) {
                int py = cy + i;
                int px = cx + j;
                
                if (px >= 0 && px < cv_ptr->image.cols && py >= 0 && py < cv_ptr->image.rows) {
                    float d = cv_ptr->image.at<float>(py, px);
                    if (d > 0.1f) {
                        sum += d;
                        count++;
                    }
                }
            }
        }
        
        return (count > 0) ? (sum / count) : 0.0f;
    }

    // ROS
    message_filters::Subscriber<TargetDetection> sub_detection_2d_;
    message_filters::Subscriber<Image> sub_depth_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Publisher<TargetDetection>::SharedPtr pub_detection_3d_;

    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Data
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    std::string target_frame_;
    TargetDetection::ConstSharedPtr pt_detection_camera_;
};

} // namespace interceptor

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<interceptor::Target3DLocalizer>());
    rclcpp::shutdown();
    return 0;
}