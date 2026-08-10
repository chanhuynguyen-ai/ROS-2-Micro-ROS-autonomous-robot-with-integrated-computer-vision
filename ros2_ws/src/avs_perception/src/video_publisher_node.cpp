#include <memory>
#include <string>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <unistd.h>
#include <limits.h>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

// Helper: check if path is a /dev/ device (camera)
static bool is_device_path(const std::string& path) {
    return (path.rfind("/dev/", 0) == 0);
}

// Helper: resolve symlink and extract camera index for V4L2
static bool try_parse_camera_index(const std::string& path, int& camera_index) {
    if (!is_device_path(path)) {
        return false;
    }

    // Resolve symlinks if present
    char resolved_path[PATH_MAX];
    if (realpath(path.c_str(), resolved_path) != nullptr) {
        std::string resolved(resolved_path);
        // Restrict only to "/dev/videoN" devices to avoid wrong-device selection
        if (resolved.rfind("/dev/video", 0) == 0) {
            size_t last_non_digit = resolved.find_last_not_of("0123456789");
            if (last_non_digit != std::string::npos && last_non_digit < resolved.length() - 1) {
                std::string digit_str = resolved.substr(last_non_digit + 1);
                try {
                    camera_index = std::stoi(digit_str);
                    return true;
                } catch (...) {
                    // Ignore parsing errors
                }
            }
        }
    }
    return false;
}

class VideoPublisherNode : public rclcpp::Node {
public:
    VideoPublisherNode() : Node("video_publisher_node") {
        // Declare parameters
        this->declare_parameter<std::string>("video_path", "/workspace/test/test_video/video_test1.mp4");
        this->declare_parameter<std::string>("publish_topic", "/camera/image_raw");
        this->declare_parameter<bool>("loop", true);
        this->declare_parameter<double>("fps_override", 0.0); // 0.0 means use source native FPS

        // Camera V4L2 capture parameters
        this->declare_parameter<int>("camera_width", 640);
        this->declare_parameter<int>("camera_height", 480);
        this->declare_parameter<int>("camera_fps", 30);

        // Retrieve parameters
        std::string video_path = this->get_parameter("video_path").as_string();
        std::string publish_topic = this->get_parameter("publish_topic").as_string();
        loop_ = this->get_parameter("loop").as_bool();
        fps_override_ = this->get_parameter("fps_override").as_double();
        cam_width_ = this->get_parameter("camera_width").as_int();
        cam_height_ = this->get_parameter("camera_height").as_int();
        cam_fps_ = this->get_parameter("camera_fps").as_int();

        // Open video source
        if (!open_source(video_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video source: %s", video_path.c_str());
            throw std::runtime_error("Could not open video source");
        }

        // Determine publishing FPS
        pub_fps_ = determine_fps();

        // Create image publishers
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(publish_topic, rclcpp::SensorDataQoS().keep_last(1));
        compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(publish_topic + "/compressed", rclcpp::SensorDataQoS().keep_last(1));

        // Create timer
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / pub_fps_.load()));
        timer_ = this->create_wall_timer(
            period,
            std::bind(&VideoPublisherNode::timer_callback, this)
        );

        // Setup dynamic parameter change callback
        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&VideoPublisherNode::on_set_parameters, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Publisher started. Publishing to topic: %s at %.1f FPS", publish_topic.c_str(), pub_fps_.load());

        // Start the background camera capture thread.
        // This thread reads frames as fast as the camera produces them and
        // stores only the latest in latest_frame_ (overwrite buffer, size=1).
        // Decouples cap_.read() (can block) from the publishing timer.
        capture_running_ = true;
        capture_thread_ = std::thread(&VideoPublisherNode::capture_loop, this);
    }

    ~VideoPublisherNode() {
        // Signal capture thread to stop and wait for it to finish
        capture_running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
    }

private:
    // ── Background capture thread ─────────────────────────────────────────────
    // Runs continuously and independently of the publish timer.
    // Always overwrites latest_frame_ with the newest camera frame (buffer size=1).
    // For video files: handles loop/end internally.
    void capture_loop() {
        while (capture_running_) {
            cv::Mat frame;
            bool ok = false;
            {
                std::lock_guard<std::mutex> lock(cap_mutex_);
                ok = cap_.read(frame);
                if (!ok) {
                    // Video file: handle end-of-file
                    if (!is_camera_source_ && loop_) {
                        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
                        ok = cap_.read(frame);
                    } else if (!is_camera_source_) {
                        RCLCPP_INFO(this->get_logger(), "Video end reached. Stopping capture thread.");
                        capture_running_ = false;
                        break;
                    }
                    // Camera frame drop: skip silently and try again
                }
            }

            if (ok && !frame.empty()) {
                // Overwrite buffer — always keep only the latest frame
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_frame_ = std::move(frame);
                new_frame_available_ = true;
            }
            
            // Producer-consumer synchronization for video files:
            // Block decoding of next frame until the current one is published/consumed by the timer callback.
            if (!is_camera_source_) {
                while (capture_running_ && new_frame_available_) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            } else {
                // Camera: small sleep to prevent CPU starvation
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    // ── Video source management ───────────────────────────────────────────────
    bool open_source(const std::string& path) {
        cv::VideoCapture new_cap;

        if (is_device_path(path)) {
            // --- Camera device ---
            // Resolve symlinks to get the real device path (e.g., /dev/video_source -> /dev/video0)
            std::string resolved = path;
            char resolved_buf[PATH_MAX];
            if (realpath(path.c_str(), resolved_buf) != nullptr) {
                resolved = std::string(resolved_buf);
            }
            int camera_index = 0;
            if (try_parse_camera_index(resolved, camera_index)) {
                RCLCPP_INFO(this->get_logger(), "Opening camera device with index %d (resolved from path: %s) with V4L2", camera_index, resolved.c_str());
                new_cap.open(camera_index, cv::CAP_V4L2);
            } else {
                RCLCPP_INFO(this->get_logger(), "Opening camera device: %s (resolved: %s) with V4L2", path.c_str(), resolved.c_str());
                new_cap.open(resolved, cv::CAP_V4L2);
            }

            if (!new_cap.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "V4L2 failed to open camera: %s", path.c_str());
                return false;
            }

            // Force MJPEG pixel format for hardware-accelerated decoding
            new_cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
            new_cap.set(cv::CAP_PROP_FRAME_WIDTH, cam_width_);
            new_cap.set(cv::CAP_PROP_FRAME_HEIGHT, cam_height_);
            new_cap.set(cv::CAP_PROP_FPS, cam_fps_);

            // Log actual negotiated values
            int actual_w = static_cast<int>(new_cap.get(cv::CAP_PROP_FRAME_WIDTH));
            int actual_h = static_cast<int>(new_cap.get(cv::CAP_PROP_FRAME_HEIGHT));
            double actual_fps = new_cap.get(cv::CAP_PROP_FPS);
            int fourcc = static_cast<int>(new_cap.get(cv::CAP_PROP_FOURCC));
            char fourcc_str[5] = {
                (char)(fourcc & 0xFF),
                (char)((fourcc >> 8) & 0xFF),
                (char)((fourcc >> 16) & 0xFF),
                (char)((fourcc >> 24) & 0xFF),
                '\0'
            };

            RCLCPP_INFO(this->get_logger(),
                "Camera configured: %dx%d @ %.1f FPS, Format: %s (requested: %dx%d @ %d FPS MJPG)",
                actual_w, actual_h, actual_fps, fourcc_str,
                cam_width_, cam_height_, cam_fps_);

            is_camera_source_ = true;
        } else {
            // --- Video file ---
            RCLCPP_INFO(this->get_logger(), "Loading video file: %s", path.c_str());
            new_cap.open(path);

            if (!new_cap.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open video file: %s", path.c_str());
                return false;
            }

            int w = static_cast<int>(new_cap.get(cv::CAP_PROP_FRAME_WIDTH));
            int h = static_cast<int>(new_cap.get(cv::CAP_PROP_FRAME_HEIGHT));
            double fps = new_cap.get(cv::CAP_PROP_FPS);
            RCLCPP_INFO(this->get_logger(), "Video loaded: %dx%d @ %.2f FPS", w, h, fps);

            is_camera_source_ = false;
        }

        // Swap capture object
        cap_.release();
        cap_ = std::move(new_cap);
        return true;
    }

    // Determine the FPS to use for the timer
    double determine_fps() {
        if (fps_override_ > 0.0) {
            return fps_override_;
        }
        if (is_camera_source_) {
            // Use the configured camera FPS (V4L2 FPS reporting can be unreliable)
            return static_cast<double>(cam_fps_);
        }
        double native_fps = cap_.get(cv::CAP_PROP_FPS);
        if (native_fps <= 0.0 || std::isnan(native_fps)) {
            native_fps = 30.0;
        }
        return native_fps;
    }

    // Reset the publishing timer to match a new FPS
    void reset_timer() {
        double fps = determine_fps();
        pub_fps_ = fps;
        timer_->cancel();
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / fps));
        timer_ = this->create_wall_timer(
            period,
            std::bind(&VideoPublisherNode::timer_callback, this)
        );
        RCLCPP_INFO(this->get_logger(), "Timer reset to %.1f FPS (period: %d ms)",
                     fps, static_cast<int>(1000.0 / fps));
    }

    rcl_interfaces::msg::SetParametersResult on_set_parameters(const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        std::lock_guard<std::mutex> lock(cap_mutex_);

        for (const auto &param : parameters) {
            if (param.get_name() == "video_path") {
                std::string new_path = param.as_string();

                if (open_source(new_path)) {
                    RCLCPP_INFO(this->get_logger(), "Dynamically switched source to: %s", new_path.c_str());
                    reset_timer();
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to switch source to: %s. Keeping previous source.", new_path.c_str());
                    result.successful = false;
                    result.reason = "Failed to open new source: " + new_path;
                }
            } else if (param.get_name() == "loop") {
                loop_ = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "Loop setting updated to: %s", loop_ ? "true" : "false");
            } else if (param.get_name() == "camera_width") {
                cam_width_ = param.as_int();
                if (is_camera_source_) {
                    cap_.set(cv::CAP_PROP_FRAME_WIDTH, cam_width_);
                    RCLCPP_INFO(this->get_logger(), "Applied camera width to device: %d", cam_width_);
                }
            } else if (param.get_name() == "camera_height") {
                cam_height_ = param.as_int();
                if (is_camera_source_) {
                    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cam_height_);
                    RCLCPP_INFO(this->get_logger(), "Applied camera height to device: %d", cam_height_);
                }
            } else if (param.get_name() == "camera_fps") {
                cam_fps_ = param.as_int();
                if (is_camera_source_) {
                    cap_.set(cv::CAP_PROP_FPS, cam_fps_);
                    RCLCPP_INFO(this->get_logger(), "Applied camera FPS to device: %d", cam_fps_);
                }
                reset_timer();
            } else if (param.get_name() == "fps_override") {
                fps_override_ = param.as_double();
                reset_timer();
            }
        }
        return result;
    }

    void timer_callback() {
        // Read the latest frame from the shared buffer (written by capture_loop).
        // If no new frame is available yet, skip this tick.
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (!new_frame_available_ || latest_frame_.empty()) {
                return;  // No fresh frame — skip publish tick
            }
            frame = latest_frame_.clone();  // cheap for 640x480 BGR
            new_frame_available_ = false;   // mark consumed
        }

        auto timestamp = this->now();

        // 1. Publish raw image (Image message)
        {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            msg->header.stamp = timestamp;
            msg->header.frame_id = "camera_frame";
            image_pub_->publish(*msg);
        }

        // 2. Publish compressed image (CompressedImage message - JPEG format)
        {
            sensor_msgs::msg::CompressedImage compressed_msg;
            compressed_msg.header.stamp = timestamp;
            compressed_msg.header.frame_id = "camera_frame";
            compressed_msg.format = "jpeg";

            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
            cv::imencode(".jpg", frame, compressed_msg.data, params);

            compressed_pub_->publish(compressed_msg);
        }
    }

    cv::VideoCapture cap_;
    bool loop_;
    bool is_camera_source_ = false;
    double fps_override_;
    std::atomic<double> pub_fps_{20.0};
    int cam_width_ = 640;
    int cam_height_ = 480;
    int cam_fps_ = 30;

    // ── Shared frame buffer (overwrite, size=1) ───────────────────────────────
    // Written by capture_loop(), read by timer_callback().
    cv::Mat             latest_frame_;
    std::mutex          frame_mutex_;                    // guards latest_frame_
    std::atomic<bool>   new_frame_available_{false};     // flag: fresh frame ready
    std::atomic<bool>   capture_running_{false};         // controls capture_loop
    std::thread         capture_thread_;                 // background reader

    std::mutex cap_mutex_;  // guards cv::VideoCapture cap_
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
