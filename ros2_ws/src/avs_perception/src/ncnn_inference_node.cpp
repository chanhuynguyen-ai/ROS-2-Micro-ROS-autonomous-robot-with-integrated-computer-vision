#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <deque>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

#include "avs_perception/yolo26_seg.hpp"
#include "avs_perception/label_mapping.hpp"

using namespace avs_perception;

class NCNNInferenceNode : public rclcpp::Node {
public:
    NCNNInferenceNode() : Node("ncnn_inference_node") {
        // Declare ROS2 parameters
        this->declare_parameter<std::string>("model_param_path", "/workspace/models/best_ncnn_model/model.ncnn.param");
        this->declare_parameter<std::string>("model_bin_path", "/workspace/models/best_ncnn_model/model.ncnn.bin");
        this->declare_parameter<float>("prob_threshold", 0.25f);
        this->declare_parameter<float>("nms_threshold", 0.45f);
        this->declare_parameter<std::string>("input_topic", "/camera/image_raw");
        this->declare_parameter<std::string>("output_topic", "/camera/segmented_image/compressed");
        this->declare_parameter<int>("num_threads", 3);
        
        this->declare_parameter<bool>("use_vulkan_compute", false);
        this->declare_parameter<bool>("use_fp16_packed", false);
        this->declare_parameter<bool>("use_fp16_storage", false);
        this->declare_parameter<bool>("use_fp16_arithmetic", false);
        this->declare_parameter<bool>("use_packing_layout", false);
        this->declare_parameter<bool>("use_int8_inference", false);
        this->declare_parameter<int>("target_size", 320);
        this->declare_parameter<bool>("decode_non_control_masks", false);
        this->declare_parameter<bool>("enable_nms", true);
        this->declare_parameter<int>("max_detections", 30);

        // Retrieve parameters
        std::string param_path = this->get_parameter("model_param_path").as_string();
        std::string bin_path = this->get_parameter("model_bin_path").as_string();
        prob_threshold_ = this->get_parameter("prob_threshold").as_double();
        nms_threshold_ = this->get_parameter("nms_threshold").as_double();
        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        int num_threads = this->get_parameter("num_threads").as_int();

        NcnnOptions options;
        options.use_vulkan_compute = this->get_parameter("use_vulkan_compute").as_bool();
        options.use_fp16_packed = this->get_parameter("use_fp16_packed").as_bool();
        options.use_fp16_storage = this->get_parameter("use_fp16_storage").as_bool();
        options.use_fp16_arithmetic = this->get_parameter("use_fp16_arithmetic").as_bool();
        options.use_packing_layout = this->get_parameter("use_packing_layout").as_bool();
        options.use_int8_inference = this->get_parameter("use_int8_inference").as_bool();
        options.num_threads = num_threads;
        options.decode_non_control_masks = this->get_parameter("decode_non_control_masks").as_bool();
        options.enable_nms = this->get_parameter("enable_nms").as_bool();
        options.max_detections = this->get_parameter("max_detections").as_int();
        options.target_size = this->get_parameter("target_size").as_int();
        if (options.target_size <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid target_size %d, falling back to 320", options.target_size);
            options.target_size = 320;
        }
        
        current_num_threads_ = num_threads;

        RCLCPP_INFO(this->get_logger(), "Loading NCNN model from: \n  Param: %s\n  Bin: %s (threads: %d)", param_path.c_str(), bin_path.c_str(), num_threads);
        
        // Initialize inference engine
        yolo_ = std::make_unique<YOLO26Seg>();
        yolo_->set_options(options);

        // Log the effective options after set_options applies fallbacks (like Vulkan unavailability)
        RCLCPP_INFO(this->get_logger(), "Applied NCNN Options: vulkan=%d, fp16_p=%d, fp16_s=%d, fp16_a=%d, pack=%d, int8=%d, target_size=%d, decode_non_control_masks=%d, enable_nms=%d, max_detections=%d",
                    options.use_vulkan_compute, options.use_fp16_packed, options.use_fp16_storage, 
                    options.use_fp16_arithmetic, options.use_packing_layout, options.use_int8_inference, options.target_size, options.decode_non_control_masks,
                    options.enable_nms, options.max_detections);
        
        if (yolo_->load(param_path, bin_path) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load model files!");
            throw std::runtime_error("Failed to load NCNN model");
        }
        RCLCPP_INFO(this->get_logger(), "Model loaded successfully.");

        // Create publisher for telemetry data
        telemetry_pub_ = this->create_publisher<std_msgs::msg::String>("/avs/telemetry", 10);

        // Create subscription to raw camera images
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&NCNNInferenceNode::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Subscribed to %s, publishing telemetry to /avs/telemetry", input_topic.c_str());
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "Received image frame! Timestamp: %u.%u", msg->header.stamp.sec, msg->header.stamp.nanosec);
        auto start_time = std::chrono::steady_clock::now();
        auto callback_start_ros = this->now();

        // 1. Calculate input_fps and input_age_ms
        double input_age_ms = 0.0;
        rclcpp::Time msg_stamp(msg->header.stamp);
        bool use_msg_stamp = (msg_stamp.nanoseconds() > 0);
        
        if (use_msg_stamp) {
            input_age_ms = (callback_start_ros - msg_stamp).seconds() * 1000.0;
        }

        double input_fps = 0.0;
        if (use_msg_stamp && has_previous_msg_stamp_) {
            double diff_sec = (msg_stamp - previous_msg_stamp_).seconds();
            if (diff_sec > 1e-6 && diff_sec < 10.0) {
                input_fps = 1.0 / diff_sec;
            } else {
                use_msg_stamp = false; // Fallback
            }
        }
        
        if (!use_msg_stamp && has_previous_callback_time_) {
            double diff_sec = std::chrono::duration<double>(start_time - previous_callback_time_).count();
            if (diff_sec > 1e-6) {
                input_fps = 1.0 / diff_sec;
            }
        }
        
        previous_msg_stamp_ = msg_stamp;
        has_previous_msg_stamp_ = (msg_stamp.nanoseconds() > 0);
        previous_callback_time_ = start_time;
        has_previous_callback_time_ = true;

        // Update parameters dynamically from standard parameter service
        prob_threshold_ = static_cast<float>(this->get_parameter("prob_threshold").as_double());
        nms_threshold_ = static_cast<float>(this->get_parameter("nms_threshold").as_double());
        bool enable_nms = this->get_parameter("enable_nms").as_bool();
        int max_detections = this->get_parameter("max_detections").as_int();
        yolo_->set_postprocess_options(enable_nms, max_detections);
        int num_threads = this->get_parameter("num_threads").as_int();
        if (num_threads != current_num_threads_) {
            yolo_->set_num_threads(num_threads);
            current_num_threads_ = num_threads;
        }

        // Convert ROS2 image to cv::Mat using zero-copy share if possible
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            if (!cv_ptr->image.isContinuous()) {
                // NCNN from_pixels_resize assumes tightly packed rows (no padding stride)
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        auto bridge_end = std::chrono::steady_clock::now();
        double bridge_latency_ms = std::chrono::duration<double, std::milli>(bridge_end - start_time).count();

        // Run inference
        std::vector<Object> objects;
        std::vector<double> timings;
        if (yolo_->detect(cv_ptr->image, objects, timings, prob_threshold_, nms_threshold_) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Inference detection failed");
            return;
        }
        auto inference_end = std::chrono::steady_clock::now();
        double inference_latency_ms = std::chrono::duration<double, std::milli>(inference_end - bridge_end).count();

        // Phase E: Profiling Timers Log
        if (timings.size() == 5) {
            RCLCPP_DEBUG(this->get_logger(), "[Phase E Profile] Preprocess: %.2f ms | Extractor: %.2f ms | Proposal: %.2f ms | NMS: %.2f ms | Mask Decode: %.2f ms", 
                         timings[0], timings[1], timings[2], timings[3], timings[4]);
        }

        // Measure post-processing (contour extraction & JSON serialization)
        auto post_start = std::chrono::steady_clock::now();
        double contour_time_ms = 0.0;

        // Greedy 2D IoU Tracking
        struct MatchPair {
            size_t det_idx;
            size_t track_idx;
            float iou;
        };
        std::vector<MatchPair> candidates;
        for (size_t i = 0; i < objects.size(); ++i) {
            for (size_t j = 0; j < active_tracks_.size(); ++j) {
                if (objects[i].label == active_tracks_[j].label) {
                    // Compute IoU
                    cv::Rect_<float> r1 = objects[i].rect;
                    cv::Rect_<float> r2 = active_tracks_[j].rect;
                    float intersection_area = (r1 & r2).area();
                    float union_area = r1.area() + r2.area() - intersection_area;
                    float iou = (union_area > 0.0f) ? (intersection_area / union_area) : 0.0f;
                    
                    if (iou >= 0.3f) {
                        candidates.push_back({i, j, iou});
                    }
                }
            }
        }

        // Sort candidates by IoU descending
        std::sort(candidates.begin(), candidates.end(), [](const MatchPair& a, const MatchPair& b) {
            return a.iou > b.iou;
        });

        std::vector<bool> det_matched(objects.size(), false);
        std::vector<bool> track_matched(active_tracks_.size(), false);
        std::vector<std::string> object_ids(objects.size());

        for (const auto& pair : candidates) {
            if (det_matched[pair.det_idx] || track_matched[pair.track_idx]) {
                continue;
            }
            det_matched[pair.det_idx] = true;
            track_matched[pair.track_idx] = true;
            
            active_tracks_[pair.track_idx].rect = objects[pair.det_idx].rect;
            active_tracks_[pair.track_idx].age += 1;
            active_tracks_[pair.track_idx].lost_count = 0;
            
            object_ids[pair.det_idx] = active_tracks_[pair.track_idx].id;
        }

        // Update unmatched tracks
        for (size_t j = 0; j < active_tracks_.size(); ++j) {
            if (!track_matched[j]) {
                active_tracks_[j].lost_count += 1;
            }
        }

        // Spawn new tracks for unmatched detections
        for (size_t i = 0; i < objects.size(); ++i) {
            if (!det_matched[i]) {
                Track new_track;
                std::string label_name = (objects[i].label >= 0 && objects[i].label < static_cast<int>(CLASS_NAMES.size())) 
                                         ? CLASS_NAMES[objects[i].label] : "unknown";
                std::string clean_label = label_name;
                std::replace(clean_label.begin(), clean_label.end(), '-', '_');
                
                new_track.id = clean_label + "_" + std::to_string(next_track_id_++);
                new_track.rect = objects[i].rect;
                new_track.label = objects[i].label;
                new_track.age = 1;
                new_track.lost_count = 0;
                
                active_tracks_.push_back(new_track);
                object_ids[i] = new_track.id;
            }
        }

        // Purge dead tracks (> 5 frames lost)
        active_tracks_.erase(
            std::remove_if(active_tracks_.begin(), active_tracks_.end(), [](const Track& t) {
                return t.lost_count > 5;
            }),
            active_tracks_.end()
        );

        // 1. Build detections part of the JSON
        std::string detections_json;
        detections_json.reserve(512);
        detections_json = "\"detections\":{";
        for (size_t i = 0; i < CLASS_NAMES.size(); i++) {
            int count = 0;
            for (const auto& obj : objects) {
                if (obj.label == static_cast<int>(i)) {
                    count++;
                }
            }
            detections_json += "\"" + CLASS_NAMES[i] + "\":" + std::to_string(count);
            if (i < CLASS_NAMES.size() - 1) {
                detections_json += ",";
            }
        }
        detections_json += "}";

        // 2. Build objects part of the JSON
        std::string objects_json;
        objects_json.reserve(4096);
        objects_json = "\"objects\":[";
        for (size_t i = 0; i < objects.size(); i++) {
            const auto& obj = objects[i];
            objects_json += "{";
            objects_json += "\"label\":" + std::to_string(obj.label) + ",";
            objects_json += "\"prob\":" + std::to_string(obj.prob) + ",";
            objects_json += "\"id\":\"" + object_ids[i] + "\",";
            objects_json += "\"track_id\":\"" + object_ids[i] + "\",";
            std::string label_name = (obj.label >= 0 && obj.label < static_cast<int>(CLASS_NAMES.size())) ? CLASS_NAMES[obj.label] : "unknown";
            objects_json += "\"class_name\":\"" + label_name + "\",";
            objects_json += "\"box\":[" + std::to_string(obj.rect.x) + "," 
                                       + std::to_string(obj.rect.y) + "," 
                                       + std::to_string(obj.rect.width) + "," 
                                       + std::to_string(obj.rect.height) + "],";
            
            // Extract contours of the mask for this object to represent polygons
            auto contour_start = std::chrono::steady_clock::now();
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            if (!obj.mask.empty()) {
                cv::findContours(obj.mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                // Apply approxPolyDP and offset
                for (size_t c = 0; c < contours.size(); c++) {
                    std::vector<cv::Point> approx;
                    cv::approxPolyDP(contours[c], approx, 1.5, true);
                    contours[c] = approx;
                    for (size_t p = 0; p < contours[c].size(); p++) {
                        contours[c][p].x += obj.mask_offset.x;
                        contours[c][p].y += obj.mask_offset.y;
                    }
                }
            }
            auto contour_end = std::chrono::steady_clock::now();
            contour_time_ms += std::chrono::duration<double, std::milli>(contour_end - contour_start).count();
            
            objects_json += "\"polygons\":[";
            for (size_t c = 0; c < contours.size(); c++) {
                objects_json += "[";
                for (size_t p = 0; p < contours[c].size(); p++) {
                    objects_json += "[" + std::to_string(contours[c][p].x) + "," + std::to_string(contours[c][p].y) + "]";
                    if (p < contours[c].size() - 1) objects_json += ",";
                }
                objects_json += "]";
                if (c < contours.size() - 1) objects_json += ",";
            }
            objects_json += "]";
            objects_json += "}";
            if (i < objects.size() - 1) objects_json += ",";
        }
        objects_json += "]";

        auto post_end = std::chrono::steady_clock::now();
        double post_processing_latency_ms = std::chrono::duration<double, std::milli>(post_end - post_start).count();

        // 3. Measure JSON Finalization
        auto json_start = post_end;

        // Calculate sliding-window processing_fps
        processing_times_.push_back(post_end);
        while (!processing_times_.empty() && 
               std::chrono::duration<double>(post_end - processing_times_.front()).count() > 1.0) {
            processing_times_.pop_front();
        }
        double processing_fps = static_cast<double>(processing_times_.size());

        // Prune publish_times_ to current time_point to ensure publish_fps is accurate even after a pause (P1)
        while (!publish_times_.empty() && 
               std::chrono::duration<double>(post_end - publish_times_.front()).count() > 1.0) {
            publish_times_.pop_front();
        }
        double publish_fps = static_cast<double>(publish_times_.size());
        
        double node_processing_latency_ms = std::chrono::duration<double, std::milli>(post_end - start_time).count();

        // Compile final JSON telemetry including correct latency & FPS values.
        // We report the actual fully measured values from the previous frame (P2) to avoid stale/inconsistent
        // Emitting actual previous-frame's metrics for physical consistency and accuracy (P2)
        std::string json_str;
        json_str.reserve(8192);
        json_str = "{";
        json_str += "\"input_fps\":" + std::to_string(input_fps) + ",";
        json_str += "\"processing_fps\":" + std::to_string(processing_fps) + ",";
        json_str += "\"publish_fps\":" + std::to_string(publish_fps) + ",";
        json_str += "\"fps\":" + std::to_string(processing_fps) + ","; // Legacy compatibility
        
        json_str += "\"bridge_latency_ms\":" + std::to_string(bridge_latency_ms) + ",";
        json_str += "\"inference_latency_ms\":" + std::to_string(inference_latency_ms) + ",";
        json_str += "\"post_processing_latency_ms\":" + std::to_string(post_processing_latency_ms) + ",";
        json_str += "\"contour_time_ms\":" + std::to_string(contour_time_ms) + ",";
        
        if (timings.size() == 5) {
            json_str += "\"profile_preprocess_ms\":" + std::to_string(timings[0]) + ",";
            json_str += "\"profile_extractor_ms\":" + std::to_string(timings[1]) + ",";
            json_str += "\"profile_proposal_ms\":" + std::to_string(timings[2]) + ",";
            json_str += "\"profile_nms_ms\":" + std::to_string(timings[3]) + ",";
            json_str += "\"profile_mask_decode_ms\":" + std::to_string(timings[4]) + ",";
        }

        
        // Emitting actual previous-frame's metrics for physical consistency and accuracy (P2)
        json_str += "\"json_finalize_latency_ms\":" + std::to_string(last_json_finalize_latency_ms_) + ",";
        json_str += "\"publish_latency_ms\":" + std::to_string(last_publish_latency_ms_) + ",";
        json_str += "\"node_total_latency_ms\":" + std::to_string(last_node_total_latency_ms_) + ",";
        json_str += "\"output_age_ms\":" + std::to_string(last_output_age_ms_) + ",";
        json_str += "\"last_input_age_ms\":" + std::to_string(last_input_age_ms_) + ",";
        
        json_str += "\"node_processing_latency_ms\":" + std::to_string(node_processing_latency_ms) + ",";
        json_str += "\"full_latency_ms\":" + std::to_string(node_processing_latency_ms) + ","; // Legacy compatibility
        
        json_str += "\"input_age_ms\":" + std::to_string(input_age_ms) + ",";
        json_str += "\"streaming\":true,";
        json_str += detections_json + ",";
        json_str += objects_json;
        json_str += "}";

        auto json_end = std::chrono::steady_clock::now();
        double current_json_finalize_latency_ms = std::chrono::duration<double, std::milli>(json_end - json_start).count();
        last_json_finalize_latency_ms_ = current_json_finalize_latency_ms;

        // 4. Publish telemetry
        auto publish_start = std::chrono::steady_clock::now();
        auto telemetry_msg = std_msgs::msg::String();
        telemetry_msg.data = json_str;
        telemetry_pub_->publish(telemetry_msg);
        auto publish_end = std::chrono::steady_clock::now();
        auto publish_end_ros = this->now();

        double current_publish_latency_ms = std::chrono::duration<double, std::milli>(publish_end - publish_start).count();
        last_publish_latency_ms_ = current_publish_latency_ms;

        // Update publish_fps sliding window with the actual completed publish time to handle DDS backpressure (P1)
        publish_times_.push_back(publish_end);
        while (!publish_times_.empty() && 
               std::chrono::duration<double>(publish_end - publish_times_.front()).count() > 1.0) {
            publish_times_.pop_front();
        }

        double current_node_total_latency_ms = std::chrono::duration<double, std::milli>(publish_end - start_time).count();
        last_node_total_latency_ms_ = current_node_total_latency_ms;

        double current_output_age_ms = 0.0;
        if (msg_stamp.nanoseconds() > 0) {
            current_output_age_ms = (publish_end_ros - msg_stamp).seconds() * 1000.0;
        }
        last_output_age_ms_ = current_output_age_ms;
        last_input_age_ms_ = input_age_ms;

        RCLCPP_DEBUG(this->get_logger(), 
            "Profiling [ms] - Total: %.2f (%.1f FPS) | cv_bridge: %.2f | Inference: %.2f | Post-proc: %.2f (Contours: %.2f) | JSON: %.2f | Publish: %.2f",
            current_node_total_latency_ms, processing_fps, bridge_latency_ms, inference_latency_ms, post_processing_latency_ms, contour_time_ms, current_json_finalize_latency_ms, current_publish_latency_ms);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[AVS Profile] Input FPS: %.1f | Proc FPS: %.1f | Pub FPS: %.1f | Total Latency: %.2f ms | Inference: %.2f ms | Output Age: %.2f ms",
            input_fps, processing_fps, publish_fps, current_node_total_latency_ms, inference_latency_ms, current_output_age_ms);
    }

    std::unique_ptr<YOLO26Seg> yolo_;
    float prob_threshold_;
    float nms_threshold_;
    int current_num_threads_ = 3;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;

    // 2D Tracking members
    struct Track {
        std::string id;
        cv::Rect_<float> rect;
        int label;
        int age;
        int lost_count;
    };
    std::vector<Track> active_tracks_;
    int next_track_id_ = 0;

    // FPS and Latency tracking members
    rclcpp::Time previous_msg_stamp_{0, 0, RCL_ROS_TIME};
    bool has_previous_msg_stamp_ = false;
    std::chrono::steady_clock::time_point previous_callback_time_;
    bool has_previous_callback_time_ = false;

    std::deque<std::chrono::steady_clock::time_point> processing_times_;
    std::deque<std::chrono::steady_clock::time_point> publish_times_;

    double last_json_finalize_latency_ms_ = 0.0;
    double last_publish_latency_ms_ = 0.0;
    double last_node_total_latency_ms_ = 0.0;
    double last_output_age_ms_ = 0.0;
    double last_input_age_ms_ = 0.0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NCNNInferenceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
