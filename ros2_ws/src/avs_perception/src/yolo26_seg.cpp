#include "avs_perception/yolo26_seg.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <chrono>

YOLO26Seg::YOLO26Seg() {
    // Options are now set via set_options()
}

YOLO26Seg::~YOLO26Seg() {
    net.clear();
}

void YOLO26Seg::set_options(NcnnOptions& options) {
#if NCNN_VULKAN
    net.opt.use_vulkan_compute = options.use_vulkan_compute;
#else
    if (options.use_vulkan_compute) {
        std::cerr << "[WARNING] NCNN was compiled without Vulkan support. Ignoring use_vulkan_compute=true" << std::endl;
        options.use_vulkan_compute = false;
    }
    net.opt.use_vulkan_compute = false;
#endif
    
    net.opt.use_fp16_packed = options.use_fp16_packed;
    net.opt.use_fp16_storage = options.use_fp16_storage;
    net.opt.use_fp16_arithmetic = options.use_fp16_arithmetic;
    net.opt.use_packing_layout = options.use_packing_layout;
    net.opt.use_int8_inference = options.use_int8_inference;
    net.opt.num_threads = options.num_threads;
    
    num_threads_ = options.num_threads;
    target_size = options.target_size;
    decode_non_control_masks_ = options.decode_non_control_masks;
    set_postprocess_options(options.enable_nms, options.max_detections);
}

void YOLO26Seg::set_num_threads(int num_threads) {
    num_threads_ = num_threads;
    net.opt.num_threads = num_threads;
}

void YOLO26Seg::set_postprocess_options(bool enable_nms, int max_detections) {
    enable_nms_ = enable_nms;
    max_detections_ = std::max(1, max_detections);
}

int YOLO26Seg::load(const std::string& param_path, const std::string& bin_path) {
    if (net.load_param(param_path.c_str()) != 0) {
        std::cerr << "Failed to load NCNN param file: " << param_path << std::endl;
        return -1;
    }
    if (net.load_model(bin_path.c_str()) != 0) {
        std::cerr << "Failed to load NCNN bin file: " << bin_path << std::endl;
        return -1;
    }
    return 0;
}

int YOLO26Seg::detect(const cv::Mat& bgr, std::vector<Object>& objects, float prob_threshold, float nms_threshold) {
    std::vector<double> timings;
    return detect(bgr, objects, timings, prob_threshold, nms_threshold);
}

int YOLO26Seg::detect(const cv::Mat& bgr, std::vector<Object>& objects, std::vector<double>& timings, float prob_threshold, float nms_threshold) {
    timings.clear();
    timings.resize(5, 0.0); // 0: preprocess, 1: extractor, 2: proposal, 3: nms, 4: mask
    auto t0 = std::chrono::steady_clock::now();

    int img_w = bgr.cols;
    int img_h = bgr.rows;

    // 1. Preprocessing: letterbox resize (preserve aspect ratio) to target_size, RGB, normalize [0,1].
    // Must match the Ultralytics LetterBox used at training / PC inference time: uniform scale plus
    // centered gray (114) padding. A plain stretch here distorts the aspect ratio of a non-square
    // input (e.g. 640x480) and measurably lowers accuracy/recall vs the PC pipeline.
    const float lb_scale = std::min((float)target_size / img_w, (float)target_size / img_h);
    const int resized_w = std::lround(img_w * lb_scale);
    const int resized_h = std::lround(img_h * lb_scale);
    const int pad_left = (target_size - resized_w) / 2;
    const int pad_top = (target_size - resized_h) / 2;
    const int pad_right = target_size - resized_w - pad_left;
    const int pad_bottom = target_size - resized_h - pad_top;

    ncnn::Mat in = ncnn::Mat::from_pixels_resize(bgr.data, ncnn::Mat::PIXEL_BGR2RGB, img_w, img_h, resized_w, resized_h);
    ncnn::Mat in_pad;
    ncnn::copy_make_border(in, in_pad, pad_top, pad_bottom, pad_left, pad_right, ncnn::BORDER_CONSTANT, 114.f);
    const float mean_vals[3] = {0.f, 0.f, 0.f};
    const float norm_vals[3] = {1/255.f, 1/255.f, 1/255.f};
    in_pad.substract_mean_normalize(mean_vals, norm_vals);

    auto t1 = std::chrono::steady_clock::now();
    timings[0] = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // 2. Run Inference
    ncnn::Extractor ex = net.create_extractor();
    ex.input("in0", in_pad);

    ncnn::Mat out0; // Detection & Mask Coefficients (44 x 2100)
    ncnn::Mat out1; // Prototype Masks (32 x 80 x 80)
    if (ex.extract("out0", out0) != 0 || ex.extract("out1", out1) != 0) {
        std::cerr << "Failed to extract output blobs from NCNN net" << std::endl;
        return -1;
    }

    auto t2 = std::chrono::steady_clock::now();
    timings[1] = std::chrono::duration<double, std::milli>(t2 - t1).count();

    int num_anchors = out0.w; // 2100
    int feat_channels = 32;
    int num_classes = out0.h - 4 - feat_channels; // Dynamically derived from NCNN output tensor shape (e.g. 58 - 4 - 32 = 22)
    if (num_classes != static_cast<int>(class_names.size())) {
        // A stale .param/.bin next to an updated class list (or vice versa) would
        // otherwise decode zero or mislabeled detections silently.
        std::cerr << "Class count mismatch: model output implies " << num_classes
                  << " classes, class list has " << class_names.size()
                  << ". Re-export the model or sync config/label_mapping.json." << std::endl;
        return -1;
    }

    std::vector<Object> proposals;

    // 3. Decode boxes, class scores, and mask coefficients
    for (int i = 0; i < num_anchors; i++) {
        // Find best class
        float max_score = 0.f;
        int class_id = -1;
        for (int c = 0; c < num_classes; c++) {
            float score = out0.row(4 + c)[i];
            if (score > max_score) {
                max_score = score;
                class_id = c;
            }
        }

        if (max_score > prob_threshold) {
            float cx = out0.row(0)[i];
            float cy = out0.row(1)[i];
            float w = out0.row(2)[i];
            float h = out0.row(3)[i];

            float x = cx - w / 2.f;
            float y = cy - h / 2.f;

            Object obj;
            obj.rect.x = x;
            obj.rect.y = y;
            obj.rect.width = w;
            obj.rect.height = h;
            obj.label = class_id;
            obj.prob = max_score;

            obj.mask_feats.resize(feat_channels);
            for (int j = 0; j < feat_channels; j++) {
                obj.mask_feats[j] = out0.row(4 + num_classes + j)[i];
            }

            proposals.push_back(obj);
        }
    }

    auto t3 = std::chrono::steady_clock::now();
    timings[2] = std::chrono::duration<double, std::milli>(t3 - t2).count();

    // Sort proposals by probability score
    qsort_descent_inplace(proposals);

    std::vector<int> picked;
    if (enable_nms_) {
        // Apply Non-Maximum Suppression (NMS)
        nms_sorted_bboxes(proposals, picked, nms_threshold);
    } else {
        const int limit = std::min(static_cast<int>(proposals.size()), max_detections_);
        picked.reserve(limit);
        for (int i = 0; i < limit; ++i) {
            picked.push_back(i);
        }
    }

    auto t4 = std::chrono::steady_clock::now();
    timings[3] = std::chrono::duration<double, std::milli>(t4 - t3).count();

    objects.clear();
    for (size_t i = 0; i < picked.size(); i++) {
        int idx = picked[i];
        Object obj = proposals[idx];

        // Box is in letterboxed input space. Keep a copy for prototype-mask sampling, then invert
        // the letterbox (remove centered padding, undo the uniform scale) to original image coords.
        cv::Rect_<float> lb_rect = obj.rect;
        obj.rect.x = (obj.rect.x - pad_left) / lb_scale;
        obj.rect.y = (obj.rect.y - pad_top) / lb_scale;
        obj.rect.width = obj.rect.width / lb_scale;
        obj.rect.height = obj.rect.height / lb_scale;

        // Clamp coordinates
        obj.rect.x = std::max(0.f, std::min(obj.rect.x, (float)(img_w - 1)));
        obj.rect.y = std::max(0.f, std::min(obj.rect.y, (float)(img_h - 1)));
        obj.rect.width = std::max(1.f, std::min(obj.rect.width, (float)(img_w - obj.rect.x)));
        obj.rect.height = std::max(1.f, std::min(obj.rect.height, (float)(img_h - obj.rect.y)));

        // Phase E: Selective mask decoding
        // Skip masks for traffic lights, signs and vehicles to save contour processing time.
        bool needs_mask = decode_non_control_masks_ ? true : class_needs_mask(obj.label);
        
        auto mask_t0 = std::chrono::steady_clock::now();
        if (needs_mask) {
            decode_mask(out1, obj.mask_feats, lb_rect, obj.rect, obj.mask, obj.mask_offset, target_size);
        }
        auto mask_t1 = std::chrono::steady_clock::now();
        timings[4] += std::chrono::duration<double, std::milli>(mask_t1 - mask_t0).count();

        objects.push_back(obj);
    }

    return 0;
}

void YOLO26Seg::decode_mask(const ncnn::Mat& proto, const std::vector<float>& mask_feats, const cv::Rect_<float>& lb_rect, const cv::Rect& out_rect, cv::Mat& dest_mask, cv::Point& mask_offset, int input_size) {
    int proto_w = proto.w;
    int proto_h = proto.h;
    int proto_c = proto.c;

    // The prototype grid corresponds to the (square) letterboxed input, so map the letterbox-space
    // box into prototype space with a single uniform scale, then output at the original box size.
    float proto_scale = (float)proto_w / input_size;

    int rx = std::round(lb_rect.x * proto_scale);
    int ry = std::round(lb_rect.y * proto_scale);
    int rw = std::round(lb_rect.width * proto_scale);
    int rh = std::round(lb_rect.height * proto_scale);

    // Clamp inside prototype dimensions
    rx = std::max(0, std::min(rx, proto_w - 1));
    ry = std::max(0, std::min(ry, proto_h - 1));
    rw = std::max(1, std::min(rw, proto_w - rx));
    rh = std::max(1, std::min(rh, proto_h - ry));

    // Allocate matrix only for the bounding box ROI in prototype space
    cv::Mat cropped_mask = cv::Mat::zeros(rh, rw, CV_32FC1);

    // Linear combination of prototype masks only within the mapped bounding box ROI
    for (int c = 0; c < proto_c; c++) {
        float coeff = mask_feats[c];
        const float* proto_ptr = proto.channel(c);

        for (int r = 0; r < rh; r++) {
            float* mask_ptr = cropped_mask.ptr<float>(r);
            const float* proto_row = proto_ptr + (ry + r) * proto_w;
            for (int col = 0; col < rw; col++) {
                mask_ptr[col] += coeff * proto_row[rx + col];
            }
        }
    }

    // Sigmoid function only within ROI
    for (int r = 0; r < rh; r++) {
        float* mask_ptr = cropped_mask.ptr<float>(r);
        for (int col = 0; col < rw; col++) {
            mask_ptr[col] = 1.0f / (1.0f + std::exp(-mask_ptr[col]));
        }
    }

    // Resize back to original bounding box size
    cv::Mat resized_mask;
    cv::resize(cropped_mask, resized_mask, out_rect.size(), 0, 0, cv::INTER_LINEAR);

    // Store mask only for the ROI
    dest_mask = cv::Mat::zeros(out_rect.size(), CV_8UC1);
    for (int r = 0; r < out_rect.height; r++) {
        const float* res_ptr = resized_mask.ptr<float>(r);
        uchar* dest_ptr = dest_mask.ptr<uchar>(r);
        for (int col = 0; col < out_rect.width; col++) {
            dest_ptr[col] = (res_ptr[col] > 0.5f) ? 255 : 0;
        }
    }
    mask_offset = out_rect.tl();
}

void YOLO26Seg::draw(cv::Mat& image, const std::vector<Object>& objects) {
    cv::Mat overlay = image.clone();

    for (size_t i = 0; i < objects.size(); i++) {
        const Object& obj = objects[i];
        cv::Scalar color = class_colors[obj.label % class_colors.size()];

        // Draw overlay transparent mask
        if (!obj.mask.empty()) {
            cv::Rect roi_rect(obj.mask_offset.x, obj.mask_offset.y, obj.mask.cols, obj.mask.rows);
            // Ensure ROI is strictly inside the image to prevent crash
            roi_rect &= cv::Rect(0, 0, image.cols, image.rows);
            if (roi_rect.area() > 0) {
                // Adjust mask in case ROI was clamped
                cv::Rect mask_roi(roi_rect.x - obj.mask_offset.x, roi_rect.y - obj.mask_offset.y, roi_rect.width, roi_rect.height);
                cv::Mat img_roi = overlay(roi_rect);
                img_roi.setTo(color, obj.mask(mask_roi));
            }
        }

        // Draw bounding box
        cv::rectangle(image, obj.rect, color, 2);

        // Draw label text
        char text[256];
        sprintf(text, "%s %.1f%%", class_names[obj.label].c_str(), obj.prob * 100);
        
        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        int x = obj.rect.x;
        int y = obj.rect.y - label_size.height - 2;
        if (y < 0) y = 0;
        if (x + label_size.width > image.cols) x = image.cols - label_size.width;

        cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)), color, -1);
        cv::putText(image, text, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }

    // Blend overlay into the original image
    cv::addWeighted(overlay, 0.4, image, 0.6, 0, image);
}

void YOLO26Seg::qsort_descent_inplace(std::vector<Object>& objects, int left, int right) {
    int i = left;
    int j = right;
    float p = objects[(left + right) / 2].prob;

    while (i <= j) {
        while (objects[i].prob > p) i++;
        while (objects[j].prob < p) j--;
        if (i <= j) {
            std::swap(objects[i], objects[j]);
            i++;
            j--;
        }
    }

    if (left < j) qsort_descent_inplace(objects, left, j);
    if (i < right) qsort_descent_inplace(objects, i, right);
}

void YOLO26Seg::qsort_descent_inplace(std::vector<Object>& objects) {
    if (objects.empty()) return;
    qsort_descent_inplace(objects, 0, objects.size() - 1);
}

void YOLO26Seg::nms_sorted_bboxes(const std::vector<Object>& objects, std::vector<int>& picked, float nms_threshold) {
    picked.clear();
    const int n = objects.size();
    std::vector<float> areas(n);
    for (int i = 0; i < n; i++) {
        areas[i] = objects[i].rect.area();
    }

    for (int i = 0; i < n; i++) {
        const Object& a = objects[i];
        bool keep = true;
        for (int j = 0; j < (int)picked.size(); j++) {
            const Object& b = objects[picked[j]];
            // Class-aware NMS: only suppress boxes of the same class, matching Ultralytics'
            // default (agnostic=False). Class-agnostic suppression drops valid overlapping
            // detections of different classes (e.g. other-lane under main-lane, parking-zone
            // over a lane), which systematically lowers recall vs the PC pipeline.
            if (a.label != b.label) continue;
            float inter = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter;
            if (inter / union_area > nms_threshold) {
                keep = false;
                break;
            }
        }
        if (keep) {
            picked.push_back(i);
        }
    }
}

float YOLO26Seg::intersection_area(const Object& a, const Object& b) {
    cv::Rect_<float> rect = a.rect & b.rect;
    return rect.area();
}
