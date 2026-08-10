#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "avs_perception/yolo26_seg.hpp"

class VideoTestNode : public rclcpp::Node {
public:
    VideoTestNode() : Node("video_test_node") {
        // Declare parameters
        this->declare_parameter<std::string>("model_param_path", "/workspace/models/best_ncnn_model/model.ncnn.param");
        this->declare_parameter<std::string>("model_bin_path", "/workspace/models/best_ncnn_model/model.ncnn.bin");
        this->declare_parameter<std::string>("video_path", "/workspace/test/test_video/video_test1.mp4");
        this->declare_parameter<std::string>("output_path", "/workspace/test/test_video_output/output_video_test1.mp4");
        this->declare_parameter<float>("prob_threshold", 0.25f);
        this->declare_parameter<float>("nms_threshold", 0.45f);
        
        this->declare_parameter<bool>("use_vulkan_compute", false);
        this->declare_parameter<bool>("use_fp16_packed", false);
        this->declare_parameter<bool>("use_fp16_storage", false);
        this->declare_parameter<bool>("use_fp16_arithmetic", false);
        this->declare_parameter<bool>("use_packing_layout", false);
        this->declare_parameter<bool>("use_int8_inference", false);
        this->declare_parameter<int>("target_size", 320);
        this->declare_parameter<int>("num_threads", 3);
        this->declare_parameter<bool>("decode_non_control_masks", false);
        this->declare_parameter<bool>("enable_nms", true);
        this->declare_parameter<int>("max_detections", 30);

        // Retrieve parameters
        std::string param_path = this->get_parameter("model_param_path").as_string();
        std::string bin_path = this->get_parameter("model_bin_path").as_string();
        std::string video_path = this->get_parameter("video_path").as_string();
        std::string output_path = this->get_parameter("output_path").as_string();
        float prob_threshold = this->get_parameter("prob_threshold").as_double();
        float nms_threshold = this->get_parameter("nms_threshold").as_double();
        
        NcnnOptions options;
        options.use_vulkan_compute = this->get_parameter("use_vulkan_compute").as_bool();
        options.use_fp16_packed = this->get_parameter("use_fp16_packed").as_bool();
        options.use_fp16_storage = this->get_parameter("use_fp16_storage").as_bool();
        options.use_fp16_arithmetic = this->get_parameter("use_fp16_arithmetic").as_bool();
        options.use_packing_layout = this->get_parameter("use_packing_layout").as_bool();
        options.use_int8_inference = this->get_parameter("use_int8_inference").as_bool();
        options.num_threads = this->get_parameter("num_threads").as_int();
        options.decode_non_control_masks = this->get_parameter("decode_non_control_masks").as_bool();
        options.enable_nms = this->get_parameter("enable_nms").as_bool();
        options.max_detections = this->get_parameter("max_detections").as_int();
        options.target_size = this->get_parameter("target_size").as_int();
        if (options.target_size <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid target_size %d, falling back to 320", options.target_size);
            options.target_size = 320;
        }

        RCLCPP_INFO(this->get_logger(), "Initializing Video Profiler...");
        RCLCPP_INFO(this->get_logger(), "Input Video: %s", video_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Output Video: %s", output_path.c_str());

        // Initialize YOLO engine
        auto yolo = std::make_unique<YOLO26Seg>();
        yolo->set_options(options);
        
        RCLCPP_INFO(this->get_logger(), "Applied NCNN Options: vulkan=%d, fp16_p=%d, fp16_s=%d, fp16_a=%d, pack=%d, int8=%d, target_size=%d, decode_non_control_masks=%d, enable_nms=%d, max_detections=%d",
                    options.use_vulkan_compute, options.use_fp16_packed, options.use_fp16_storage, 
                    options.use_fp16_arithmetic, options.use_packing_layout, options.use_int8_inference, options.target_size,
                    options.decode_non_control_masks, options.enable_nms, options.max_detections);
                    
        if (yolo->load(param_path, bin_path) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load NCNN model!");
            return;
        }

        // Open input video
        cv::VideoCapture cap(video_path);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open input video file: %s", video_path.c_str());
            return;
        }

        int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        double fps = cap.get(cv::CAP_PROP_FPS);
        int total_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);

        RCLCPP_INFO(this->get_logger(), "Video Resolution: %dx%d, Native FPS: %.2f, Total Frames: %d", 
                    width, height, fps, total_frames);

        // Open output video writer
        cv::VideoWriter writer(
            output_path,
            cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
            fps,
            cv::Size(width, height)
        );

        if (!writer.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open output video file for writing: %s", output_path.c_str());
            return;
        }

        cv::Mat frame;
        int frame_count = 0;
        double total_latency = 0.0;
        double max_latency = 0.0;
        double min_latency = 1e9;
        std::vector<double> latencies;
        std::array<std::vector<double>, 5> timing_samples;
        std::vector<int> object_counts;

        RCLCPP_INFO(this->get_logger(), "Processing frames...");

        while (cap.read(frame)) {
            if (frame.empty()) break;

            auto start = std::chrono::high_resolution_clock::now();

            // Run detection & segmentation
            std::vector<Object> objects;
            std::vector<double> timings;
            yolo->detect(frame, objects, timings, prob_threshold, nms_threshold);

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end - start;
            double latency = duration.count();

            total_latency += latency;
            max_latency = std::max(max_latency, latency);
            min_latency = std::min(min_latency, latency);
            latencies.push_back(latency);
            object_counts.push_back(static_cast<int>(objects.size()));
            if (timings.size() == timing_samples.size()) {
                for (size_t i = 0; i < timing_samples.size(); ++i) {
                    timing_samples[i].push_back(timings[i]);
                }
            }
            
            if (frame_count < 5) {
                std::cout << "[CPP DIAGNOSTIC] Frame " << frame_count << " Detections: ";
                if (objects.empty()) {
                    std::cout << "None";
                } else {
                    for (const auto& obj : objects) {
                        // Class names: see config/label_mapping.json (0: dashed-white, 1: dashed-yellow, 2: double-solid-white, 6: main-lane, 7: other-lane, ...).
                        std::cout << obj.label << " (" << obj.prob << "), ";
                    }
                }
                std::cout << std::endl;
            }
            
            frame_count++;

            // Draw results
            yolo->draw(frame, objects);

            // Write drawn frame to output video
            writer.write(frame);

            if (frame_count % 30 == 0 || frame_count == total_frames) {
                RCLCPP_INFO(this->get_logger(), "Processed %d/%d frames (%.1f%%). Current Latency: %.2f ms", 
                            frame_count, total_frames, (float)frame_count / total_frames * 100.f, latency);
            }
        }

        cap.release();
        writer.release();

        if (frame_count > 0) {
            double avg_latency = total_latency / frame_count;
            double avg_fps = 1000.0 / avg_latency;
            auto mean = [](const std::vector<double>& values) {
                if (values.empty()) return 0.0;
                return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
            };
            auto percentile = [](std::vector<double> values, double pct) {
                if (values.empty()) return 0.0;
                std::sort(values.begin(), values.end());
                const double rank = (pct / 100.0) * static_cast<double>(values.size() - 1);
                const size_t idx = static_cast<size_t>(std::round(rank));
                return values[std::min(idx, values.size() - 1)];
            };
            double avg_objects = 0.0;
            int max_objects = 0;
            if (!object_counts.empty()) {
                avg_objects = static_cast<double>(std::accumulate(object_counts.begin(), object_counts.end(), 0)) / object_counts.size();
                max_objects = *std::max_element(object_counts.begin(), object_counts.end());
            }
            RCLCPP_INFO(this->get_logger(), "=========================================");
            RCLCPP_INFO(this->get_logger(), "             PROFILING REPORT            ");
            RCLCPP_INFO(this->get_logger(), "=========================================");
            RCLCPP_INFO(this->get_logger(), "Processed Frames: %d", frame_count);
            RCLCPP_INFO(this->get_logger(), "Detection Latency stats:");
            RCLCPP_INFO(this->get_logger(), "  - Average:  %.2f ms", avg_latency);
            RCLCPP_INFO(this->get_logger(), "  - P50:      %.2f ms", percentile(latencies, 50.0));
            RCLCPP_INFO(this->get_logger(), "  - P95:      %.2f ms", percentile(latencies, 95.0));
            RCLCPP_INFO(this->get_logger(), "  - Min:      %.2f ms", min_latency);
            RCLCPP_INFO(this->get_logger(), "  - Max:      %.2f ms", max_latency);
            RCLCPP_INFO(this->get_logger(), "Inference Performance: %.2f FPS", avg_fps);
            RCLCPP_INFO(this->get_logger(), "Object Count: avg=%.2f max=%d", avg_objects, max_objects);
            RCLCPP_INFO(this->get_logger(), "Timing breakdown mean / p95:");
            RCLCPP_INFO(this->get_logger(), "  - Preprocess: %.2f / %.2f ms", mean(timing_samples[0]), percentile(timing_samples[0], 95.0));
            RCLCPP_INFO(this->get_logger(), "  - Extractor:  %.2f / %.2f ms", mean(timing_samples[1]), percentile(timing_samples[1], 95.0));
            RCLCPP_INFO(this->get_logger(), "  - Proposal:   %.2f / %.2f ms", mean(timing_samples[2]), percentile(timing_samples[2], 95.0));
            RCLCPP_INFO(this->get_logger(), "  - NMS/select: %.2f / %.2f ms", mean(timing_samples[3]), percentile(timing_samples[3], 95.0));
            RCLCPP_INFO(this->get_logger(), "  - Mask:       %.2f / %.2f ms", mean(timing_samples[4]), percentile(timing_samples[4], 95.0));
            RCLCPP_INFO(this->get_logger(), "=========================================");
        } else {
            RCLCPP_ERROR(this->get_logger(), "No frames were processed!");
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoTestNode>();
    rclcpp::shutdown();
    return 0;
}
