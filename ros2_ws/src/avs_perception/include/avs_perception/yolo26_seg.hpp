#ifndef YOLO26_SEG_HPP
#define YOLO26_SEG_HPP

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ncnn/net.h>

struct Object {
    cv::Rect_<float> rect;
    int label;
    float prob;
    std::vector<float> mask_feats; // 32 mask coefficients
    cv::Mat mask;                  // CV_8UC1 binary mask at ROI size
    cv::Point mask_offset;         // Top-left offset of the mask relative to full image
};

struct NcnnOptions {
    bool use_vulkan_compute = false;
    bool use_fp16_packed = false;
    bool use_fp16_storage = false;
    bool use_fp16_arithmetic = false;
    bool use_packing_layout = false;
    bool use_int8_inference = false;
    int num_threads = 3;
    int target_size = 320;
    bool decode_non_control_masks = false;
    bool enable_nms = true;
    int max_detections = 30;
};

class YOLO26Seg {
public:
    YOLO26Seg();
    ~YOLO26Seg();

    int load(const std::string& param_path, const std::string& bin_path);
    int detect(const cv::Mat& bgr, std::vector<Object>& objects, std::vector<double>& timings, float prob_threshold = 0.25f, float nms_threshold = 0.45f);
    int detect(const cv::Mat& bgr, std::vector<Object>& objects, float prob_threshold = 0.25f, float nms_threshold = 0.45f);
    void draw(cv::Mat& image, const std::vector<Object>& objects);
    void set_options(NcnnOptions& options);
    void set_num_threads(int num_threads);
    void set_postprocess_options(bool enable_nms, int max_detections);

private:
    ncnn::Net net;
    int num_threads_ = 3;
    int target_size = 320;
    bool decode_non_control_masks_ = false;
    bool enable_nms_ = true;
    int max_detections_ = 30;
    std::vector<std::string> class_names = {
        "dashed-white", "dashed-yellow", "double-solid-white", "light_green",
        "light_red", "light_yellow", "main-lane", "other-lane",
        "parking-zone", "sign-no-left", "sign-no-parking", "sign-no-right",
        "sign-parking", "sign-stop", "sign-turn-left", "sign-turn-right",
        "solid-white", "solid-yellow", "start", "stop-line",
        "turn-lane", "vehicle"
    };

    // Color palette for segmentation overlay (BGR format)
    std::vector<cv::Scalar> class_colors = {
        cv::Scalar(255, 0, 0),      // dashed-white: Blue
        cv::Scalar(0, 165, 255),    // dashed-yellow: Orange
        cv::Scalar(255, 127, 0),    // double-solid-white: Light Blue
        cv::Scalar(120, 200, 0),    // light_green: Sea Green
        cv::Scalar(80, 80, 255),    // light_red: Salmon Red
        cv::Scalar(128, 255, 255),  // light_yellow: Pale Yellow
        cv::Scalar(0, 255, 0),      // main-lane: Green
        cv::Scalar(0, 0, 255),      // other-lane: Red
        cv::Scalar(128, 128, 128),  // parking-zone: Gray
        cv::Scalar(60, 20, 220),    // sign-no-left: Crimson
        cv::Scalar(0, 0, 180),      // sign-no-parking: Bright Crimson
        cv::Scalar(50, 50, 150),    // sign-no-right: Dark Red
        cv::Scalar(230, 100, 50),   // sign-parking: Royal Blue
        cv::Scalar(0, 0, 255),      // sign-stop: Stop Red
        cv::Scalar(235, 206, 135),  // sign-turn-left: Sky Blue
        cv::Scalar(180, 130, 70),   // sign-turn-right: Steel Blue
        cv::Scalar(255, 255, 0),    // solid-white: Cyan
        cv::Scalar(0, 255, 255),    // solid-yellow: Yellow
        cv::Scalar(0, 255, 127),    // start: Spring Green
        cv::Scalar(0, 0, 128),      // stop-line: Navy
        cv::Scalar(127, 0, 255),    // turn-lane: Purple
        cv::Scalar(255, 0, 255)     // vehicle: Magenta
    };

    // Masks only matter for lane/marking geometry. Traffic lights, signs and
    // vehicles are consumed as boxes, so their contours can be skipped. Keyed on
    // the class name, not the id, so inserting classes into the model cannot
    // silently start skipping a lane class.
    bool class_needs_mask(int label) const {
        if (label < 0 || label >= static_cast<int>(class_names.size())) return true;
        const std::string& name = class_names[label];
        return !(name.rfind("sign-", 0) == 0 || name.rfind("light_", 0) == 0 || name == "vehicle");
    }

    void qsort_descent_inplace(std::vector<Object>& objects, int left, int right);
    void qsort_descent_inplace(std::vector<Object>& objects);
    void nms_sorted_bboxes(const std::vector<Object>& objects, std::vector<int>& picked, float nms_threshold);
    float intersection_area(const Object& a, const Object& b);
    void decode_mask(const ncnn::Mat& proto, const std::vector<float>& mask_feats, const cv::Rect_<float>& lb_rect, const cv::Rect& out_rect, cv::Mat& dest_mask, cv::Point& mask_offset, int input_size);
};

#endif // YOLO26_SEG_HPP
