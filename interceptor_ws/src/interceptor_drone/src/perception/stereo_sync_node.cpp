/**
 * @file stereo_sync_node.cpp
 * @brief Synchronizes stereo camera pair (left/right images + camera_info)
 * 
 * This node uses message_filters::Synchronizer with ApproximateTime policy
 * to ensure left and right camera frames are temporally aligned before
 * processing by downstream nodes (depth estimation, object detection).
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <interceptor_interfaces/msg/stereo_image_pair.hpp>

#include <chrono>
#include <deque>
#include <mutex>

namespace interceptor {

class StereoSyncNode : public rclcpp::Node {
public:
    StereoSyncNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("stereo_sync_node", options),
          sync_slop_(0.005),
          queue_size_(10),
          sync_count_(0),
          drop_count_(0) {
        
        RCLCPP_INFO(get_logger(), "============================================");
        RCLCPP_INFO(get_logger(), "Stereo Synchronization Node Starting...");
        RCLCPP_INFO(get_logger(), "============================================");
        
        // Declare and get parameters
        declare_parameters();
        get_parameters();
        
        // Log configuration
        RCLCPP_INFO(get_logger(), "Configuration:");
        RCLCPP_INFO(get_logger(), "  Sync slop (tolerance): %.3f seconds (%.1f ms)", 
                    sync_slop_, sync_slop_ * 1000.0);
        RCLCPP_INFO(get_logger(), "  Queue size: %d", queue_size_);
        RCLCPP_INFO(get_logger(), "  Left image topic: %s", left_image_topic_.c_str());
        RCLCPP_INFO(get_logger(), "  Right image topic: %s", right_image_topic_.c_str());
        
        // Setup subscribers with message_filters
        setup_subscribers();
        
        // Setup publisher for synchronized pair
        stereo_pub_ = create_publisher<interceptor_interfaces::msg::StereoImagePair>(
            "/stereo/synced", queue_size_);
        
        // Setup statistics timer (1 Hz)
        stats_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&StereoSyncNode::publish_statistics, this));
        
        RCLCPP_INFO(get_logger(), "Stereo Sync Node initialized successfully!");
        RCLCPP_INFO(get_logger(), "Waiting for stereo image pairs...");
    }

private:
    using Image = sensor_msgs::msg::Image;
    using CameraInfo = sensor_msgs::msg::CameraInfo;
    using StereoImagePair = interceptor_interfaces::msg::StereoImagePair;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        Image, Image, CameraInfo, CameraInfo>;
    
    // Subscribers
    std::shared_ptr<message_filters::Subscriber<Image>> left_image_sub_;
    std::shared_ptr<message_filters::Subscriber<Image>> right_image_sub_;
    std::shared_ptr<message_filters::Subscriber<CameraInfo>> left_info_sub_;
    std::shared_ptr<message_filters::Subscriber<CameraInfo>> right_info_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    
    // Publisher
    rclcpp::Publisher<StereoImagePair>::SharedPtr stereo_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr stats_timer_;
    
    // Parameters
    double sync_slop_;
    int queue_size_;
    std::string left_image_topic_;
    std::string right_image_topic_;
    std::string left_info_topic_;
    std::string right_info_topic_;
    
    // Statistics
    std::atomic<uint64_t> sync_count_;
    std::atomic<uint64_t> drop_count_;
    std::deque<double> latency_history_;
    std::mutex latency_mutex_;
    rclcpp::Time last_sync_time_;

    void declare_parameters() {
        declare_parameter("sync_slop", 0.005);  // 5ms default
        declare_parameter("queue_size", 10);
        declare_parameter("left_image_topic", "/stereo/left/image_raw");
        declare_parameter("right_image_topic", "/stereo/right/image_raw");
        declare_parameter("left_info_topic", "/stereo/left/camera_info");
        declare_parameter("right_info_topic", "/stereo/right/camera_info");
    }

    void get_parameters() {
        sync_slop_ = get_parameter("sync_slop").as_double();
        queue_size_ = get_parameter("queue_size").as_int();
        left_image_topic_ = get_parameter("left_image_topic").as_string();
        right_image_topic_ = get_parameter("right_image_topic").as_string();
        left_info_topic_ = get_parameter("left_info_topic").as_string();
        right_info_topic_ = get_parameter("right_info_topic").as_string();
    }

    void setup_subscribers() {
        // Create subscribers
        left_image_sub_ = std::make_shared<message_filters::Subscriber<Image>>(
            this, left_image_topic_);
        right_image_sub_ = std::make_shared<message_filters::Subscriber<Image>>(
            this, right_image_topic_);
        left_info_sub_ = std::make_shared<message_filters::Subscriber<CameraInfo>>(
            this, left_info_topic_);
        right_info_sub_ = std::make_shared<message_filters::Subscriber<CameraInfo>>(
            this, right_info_topic_);
        
        // Create synchronizer with custom slop
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(queue_size_), 
            *left_image_sub_, *right_image_sub_, 
            *left_info_sub_, *right_info_sub_);
        
        // Set sync tolerance (slop)
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_));
        
        // Register callback
        sync_->registerCallback(
            std::bind(&StereoSyncNode::synced_callback, this,
                      std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3, std::placeholders::_4));
    }

    void synced_callback(
        const Image::ConstSharedPtr& left_img,
        const Image::ConstSharedPtr& right_img,
        const CameraInfo::ConstSharedPtr& left_info,
        const CameraInfo::ConstSharedPtr& right_info) {
        
        auto now = this->now();
        
        // Calculate synchronization quality (time difference between left and right)
        rclcpp::Time left_time(left_img->header.stamp);
        rclcpp::Time right_time(right_img->header.stamp);
        double time_diff = std::abs((left_time - right_time).seconds());
        
        // Calculate latency (time from capture to sync)
        double latency = (now - left_time).seconds();
        
        // Store latency for statistics
        {
            std::lock_guard<std::mutex> lock(latency_mutex_);
            latency_history_.push_back(latency);
            if (latency_history_.size() > 100) {
                latency_history_.pop_front();
            }
        }
        
        // Check if synchronization is within tolerance
        bool is_well_synced = time_diff <= sync_slop_;
        
        if (!is_well_synced) {
            drop_count_++;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Poor synchronization: time diff = %.3f ms (threshold: %.3f ms)",
                time_diff * 1000.0, sync_slop_ * 1000.0);
            return;
        }
        
        // Create synchronized message
        StereoImagePair stereo_pair;
        stereo_pair.header.stamp = now;
        stereo_pair.header.frame_id = left_img->header.frame_id;
        
        // Copy images
        stereo_pair.left_image = *left_img;
        stereo_pair.right_image = *right_img;
        
        // Copy camera info
        stereo_pair.left_info = *left_info;
        stereo_pair.right_info = *right_info;
        
        // Metadata
        stereo_pair.sync_slop_used = sync_slop_;
        stereo_pair.is_synced = true;
        
        // Publish
        stereo_pub_->publish(stereo_pair);
        
        // Update statistics
        sync_count_++;
        last_sync_time_ = now;
        
        // Debug logging (throttled)
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
            "Published synced stereo pair: latency=%.2f ms, diff=%.2f ms",
            latency * 1000.0, time_diff * 1000.0);
    }

    void publish_statistics() {
        uint64_t synced = sync_count_.load();
        uint64_t dropped = drop_count_.load();
        uint64_t total = synced + dropped;
        
        if (total == 0) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                "Statistics: No messages received yet...");
            return;
        }
        
        double sync_rate = 100.0 * synced / total;
        
        // Calculate average latency
        double avg_latency = 0.0;
        {
            std::lock_guard<std::mutex> lock(latency_mutex_);
            if (!latency_history_.empty()) {
                avg_latency = std::accumulate(latency_history_.begin(), 
                                              latency_history_.end(), 0.0) 
                              / latency_history_.size();
            }
        }
        
        RCLCPP_INFO(get_logger(), 
            "Sync Statistics [1s]: %lu synced, %lu dropped (%.1f%% success), "
            "avg latency: %.2f ms",
            synced, dropped, sync_rate, avg_latency * 1000.0);
    }
};

} // namespace interceptor

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Set logging level to INFO by default
    rclcpp::Logger logger = rclcpp::get_logger("stereo_sync_node");
    auto node = std::make_shared<interceptor::StereoSyncNode>();
    
    RCLCPP_INFO(node->get_logger(), "Spinning...");
    rclcpp::spin(node);
    
    RCLCPP_INFO(node->get_logger(), "Shutting down...");
    rclcpp::shutdown();
    return 0;
}
