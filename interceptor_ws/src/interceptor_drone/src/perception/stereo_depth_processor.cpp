/**
 * @file stereo_depth_processor.cpp
 * @brief Computes depth from synchronized stereo image pairs using CPU (SGBM)
 * 
 * This node subscribes to synchronized stereo pairs and uses OpenCV's 
 * Semi-Global Block Matching (SGBM) and WLS filtering to compute 
 * high-quality depth maps.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/ximgproc.hpp>
#include <interceptor_interfaces/msg/stereo_image_pair.hpp>

#include <chrono>

namespace interceptor {

class StereoDepthProcessor : public rclcpp::Node {
public:
    StereoDepthProcessor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("stereo_depth_processor", options) {
        
        RCLCPP_INFO(get_logger(), "============================================");
        RCLCPP_INFO(get_logger(), "Stereo Depth Processor Starting (CPU-SGBM)...");
        RCLCPP_INFO(get_logger(), "============================================");
        
        // Declare and load parameters
        declare_parameters();
        load_parameters();
        
        // Initialize SGBM Matcher
        // P1, P2 are penalty parameters for disparity smoothness
        int P1 = 8 * 3 * block_size_ * block_size_;
        int P2 = 32 * 3 * block_size_ * block_size_;
        
        left_matcher_ = cv::StereoSGBM::create(
            0,                  // minDisparity
            num_disparities_,   // numDisparities
            block_size_,        // blockSize
            P1,                 // P1
            P2,                 // P2
            1,                  // disp12MaxDiff
            63,                 // preFilterCap
            10,                 // uniquenessRatio
            100,                // speckleWindowSize
            1,                  // speckleRange
            cv::StereoSGBM::MODE_SGBM_3WAY
        );
        
        // Initialize WLS Filter for post-processing
        wls_filter_ = cv::ximgproc::createDisparityWLSFilter(left_matcher_);
        right_matcher_ = cv::ximgproc::createRightMatcher(left_matcher_);
        
        // Setup subscriber
        sub_synced_ = create_subscription<interceptor_interfaces::msg::StereoImagePair>(
            "/stereo/synced", 10,
            std::bind(&StereoDepthProcessor::synced_callback, this, std::placeholders::_1));
            
        // Setup publishers
        pub_depth_ = create_publisher<sensor_msgs::msg::Image>("/stereo/depth", 10);
        pub_disparity_ = create_publisher<sensor_msgs::msg::Image>("/stereo/disparity", 10);
        
        RCLCPP_INFO(get_logger(), "Initialized SGBM + WLS Filter:");
        RCLCPP_INFO(get_logger(), "  Baseline: %.3f m", baseline_);
        RCLCPP_INFO(get_logger(), "  Focal length (fx): %.2f px", fx_);
        RCLCPP_INFO(get_logger(), "  Disparities: %d", num_disparities_);
        RCLCPP_INFO(get_logger(), "Waiting for synchronized stereo pairs...");
    }

private:
    void declare_parameters() {
        declare_parameter("stereo.baseline", 0.12);
        declare_parameter("stereo.fx", 535.4);
        declare_parameter("num_disparities", 128);
        declare_parameter("block_size", 11);
        declare_parameter("lambda", 8000.0);
        declare_parameter("sigma", 1.5);
    }

    void load_parameters() {
        baseline_ = get_parameter("stereo.baseline").as_double();
        fx_ = get_parameter("stereo.fx").as_double();
        num_disparities_ = get_parameter("num_disparities").as_int();
        block_size_ = get_parameter("block_size").as_int();
        lambda_ = get_parameter("lambda").as_double();
        sigma_ = get_parameter("sigma").as_double();
    }

    void synced_callback(const interceptor_interfaces::msg::StereoImagePair::SharedPtr msg) {
        RCLCPP_INFO_ONCE(get_logger(), "Received first synchronized stereo pair!");
        auto start_time = std::chrono::steady_clock::now();

        try {
            // 1. Convert ROS images to OpenCV Mat
            cv::Mat left = cv_bridge::toCvCopy(msg->left_image, "bgr8")->image;
            cv::Mat right = cv_bridge::toCvCopy(msg->right_image, "bgr8")->image;
            
            // 2. Convert to grayscale
            cv::Mat left_gray, right_gray;
            cv::cvtColor(left, left_gray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(right, right_gray, cv::COLOR_BGR2GRAY);
            
            // 3. Compute Disparity
            cv::Mat left_disp, right_disp;
            left_matcher_->compute(left_gray, right_gray, left_disp);
            right_matcher_->compute(right_gray, left_gray, right_disp);
            
            // 4. Apply WLS Filtering
            cv::Mat filtered_disp;
            wls_filter_->setLambda(lambda_);
            wls_filter_->setSigmaColor(sigma_);
            wls_filter_->filter(left_disp, left, filtered_disp, right_disp);
            
            // 5. Convert to Float32 Disparity (CV_16S -> CV_32F, scale by 1/16)
            cv::Mat disp_float;
            filtered_disp.convertTo(disp_float, CV_32F, 1.0/16.0);
            
            // 6. Compute Depth: Z = (f * B) / disparity
            cv::Mat depth_mat = cv::Mat::zeros(disp_float.size(), CV_32F);
            double focal_baseline = fx_ * baseline_;
            
            // Optimization: Vectorized depth computation
            // depth = focal_baseline / disparity
            // Only where disparity > 0
            for (int i = 0; i < disp_float.rows; i++) {
                float* d_ptr = disp_float.ptr<float>(i);
                float* z_ptr = depth_mat.ptr<float>(i);
                for (int j = 0; j < disp_float.cols; j++) {
                    if (d_ptr[j] > 0.1f) {
                        z_ptr[j] = static_cast<float>(focal_baseline / d_ptr[j]);
                    }
                }
            }
            
            // 7. Publish Results
            auto disp_msg = cv_bridge::CvImage(msg->header, "32FC1", disp_float).toImageMsg();
            pub_disparity_->publish(*disp_msg);
            
            auto depth_msg = cv_bridge::CvImage(msg->header, "32FC1", depth_mat).toImageMsg();
            pub_depth_->publish(*depth_msg);
            
            auto end_time = std::chrono::steady_clock::now();
                        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                        
                        RCLCPP_INFO(get_logger(), "Depth processed in %ld ms", duration.count());
            

        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Depth Processor Error: %s", e.what());
        }
    }

    // ROS Members
    rclcpp::Subscription<interceptor_interfaces::msg::StereoImagePair>::SharedPtr sub_synced_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_disparity_;

    // Matchers and Filter
    cv::Ptr<cv::StereoSGBM> left_matcher_;
    cv::Ptr<cv::StereoMatcher> right_matcher_;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter_;

    // Parameters
    double baseline_;
    double fx_;
    int num_disparities_;
    int block_size_;
    double lambda_;
    double sigma_;
};

} // namespace interceptor

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<interceptor::StereoDepthProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
