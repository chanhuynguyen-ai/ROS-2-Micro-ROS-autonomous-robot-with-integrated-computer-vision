#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <cmath>
#include <algorithm>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include "avs_perception/label_mapping.hpp"
#include "avs_perception/bev_region.hpp"

using json = nlohmann::json;
using namespace avs_perception;

class IPMTransformNode : public rclcpp::Node {
public:
    IPMTransformNode() : Node("ipm_transform_node") {
        // Declare ROS2 parameters
        this->declare_parameter<std::string>("calibration_file_path", "/workspace/config/calibration.json");
        this->declare_parameter<bool>("publish_debug_centerline", false);
        this->declare_parameter<double>("lookahead_T_preview", 0.15);   // seconds
        this->declare_parameter<double>("lookahead_d_min_mm", 120.0);  // mm
        this->declare_parameter<double>("lookahead_d_max_mm", 450.0);  // mm
        // Plan E1: valid BEV region for backprojection (see bev_region.hpp)
        this->declare_parameter<double>("bev_horizon_margin_px", 10.0);
        this->declare_parameter<double>("bev_y_max_mm", 8000.0);
        this->declare_parameter<double>("bev_x_abs_max_mm", 4000.0);

        calibration_file_path_ = this->get_parameter("calibration_file_path").as_string();
        publish_debug_centerline_ = this->get_parameter("publish_debug_centerline").as_bool();
        T_preview_  = this->get_parameter("lookahead_T_preview").as_double();
        d_min_mm_   = this->get_parameter("lookahead_d_min_mm").as_double();
        d_max_mm_   = this->get_parameter("lookahead_d_max_mm").as_double();
        bev_region_.horizon_margin_px = this->get_parameter("bev_horizon_margin_px").as_double();
        bev_region_.y_max_mm          = this->get_parameter("bev_y_max_mm").as_double();
        bev_region_.x_abs_max_mm      = this->get_parameter("bev_x_abs_max_mm").as_double();

        RCLCPP_INFO(this->get_logger(), "Starting IPM Transform Node.");
        RCLCPP_INFO(this->get_logger(), "Calibration file path: %s", calibration_file_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Look-ahead: T_preview=%.2fs, d=[%.0f, %.0f]mm",
            T_preview_, d_min_mm_, d_max_mm_);
        RCLCPP_INFO(this->get_logger(), "BEV region: horizon_margin=%.0fpx, y=[%.0f, %.0f]mm, |x|<=%.0fmm",
            bev_region_.horizon_margin_px, bev_region_.y_min_mm, bev_region_.y_max_mm, bev_region_.x_abs_max_mm);

        // Attempt initial calibration load
        load_calibration();

        // Create publishers
        telemetry_realworld_pub_ = this->create_publisher<std_msgs::msg::String>("/avs/telemetry_realworld", 10);
        ipm_debug_pub_ = this->create_publisher<std_msgs::msg::String>("/avs/ipm_debug", 10);

        // Subscriber to raw telemetry (pixel coords)
        telemetry_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/avs/telemetry", 10,
            std::bind(&IPMTransformNode::telemetry_callback, this, std::placeholders::_1)
        );

        // Subscriber to odometry for dynamic look-ahead velocity
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_raw", 10,
            std::bind(&IPMTransformNode::odom_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Subscribed to /avs/telemetry and /odom_raw, publishing to /avs/telemetry_realworld");
    }

private:
    // Odometry callback — extract linear speed from /odom_raw
    // Note: linear.x is normalized (0-1), where 1.0 corresponds to 2.5 m/s (2500 mm/s)
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_speed_mms_ = std::abs(msg->twist.twist.linear.x) * 2500.0;
    }

    // Compute dynamic look-ahead distance based on current speed
    double compute_lookahead_d() const {
        // Re-read parameters in case they were changed at runtime
        double d = current_speed_mms_ * T_preview_;
        return std::clamp(d, d_min_mm_, d_max_mm_);
    }
    // Removed get_resolved_calibration_path() to avoid fail-open fallback behavior

    void load_calibration() {
        try {
            if (!std::filesystem::exists(calibration_file_path_)) {
                RCLCPP_WARN(this->get_logger(), "Calibration file does not exist at: %s. IPM transformation will be skipped.", calibration_file_path_.c_str());
                has_calibration_ = false;
                return;
            }

            std::ifstream f(calibration_file_path_);
            if (!f.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open calibration file: %s", calibration_file_path_.c_str());
                has_calibration_ = false;
                return;
            }

            json data;
            f >> data;
            f.close();

            if (data.contains("homography_matrix")) {
                auto h_mat = data["homography_matrix"];
                if (h_mat.size() == 3 && h_mat[0].size() == 3) {
                    for (int i = 0; i < 3; ++i) {
                        for (int j = 0; j < 3; ++j) {
                            H_[i][j] = h_mat[i][j].get<double>();
                        }
                    }
                    has_calibration_ = true;
                    last_write_time_ = std::filesystem::last_write_time(calibration_file_path_);
                    RCLCPP_INFO(this->get_logger(), "Successfully loaded homography matrix H from calibration file: %s", calibration_file_path_.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Invalid homography matrix dimensions in calibration file.");
                    has_calibration_ = false;
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Calibration file does not contain 'homography_matrix'.");
                has_calibration_ = false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception during load_calibration: %s", e.what());
            has_calibration_ = false;
        }
    }

    void check_calibration_update() {
        try {
            if (std::filesystem::exists(calibration_file_path_)) {
                auto current_time = std::filesystem::last_write_time(calibration_file_path_);
                if (!has_calibration_ || current_time != last_write_time_) {
                    RCLCPP_INFO(this->get_logger(), "Calibration file change detected. Reloading...");
                    load_calibration();
                }
            } else if (has_calibration_) {
                RCLCPP_WARN(this->get_logger(), "Calibration file was removed. Skipping IPM transformations.");
                has_calibration_ = false;
            }
        } catch (const std::filesystem::filesystem_error& e) {
            // Log warning but don't crash, since filesystem operations can occasionally block or fail during writes
            RCLCPP_WARN(this->get_logger(), "Filesystem error while checking calibration update: %s", e.what());
        }
    }

    struct Waypoint {
        double x;
        double y;
    };

    std::vector<Waypoint> extract_centerline_waypoints_y(const std::vector<std::vector<double>>& poly_real_world, double step_mm = 100.0, json* debug_out = nullptr) {
        std::vector<Waypoint> waypoints;
        if (poly_real_world.empty()) return waypoints;

        if (debug_out) {
            if (!debug_out->contains("slices")) (*debug_out)["slices"] = json::array();
            if (!debug_out->contains("raw_midpoints")) (*debug_out)["raw_midpoints"] = json::array();
            if (!debug_out->contains("filtered_midpoints")) (*debug_out)["filtered_midpoints"] = json::array();
        }

        double min_y = 1e9;
        double max_y = -1e9;
        for (const auto& pt : poly_real_world) {
            if (pt.size() >= 2) {
                double y = pt[1];
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
            }
        }

        if (min_y >= max_y) return waypoints;

        // Structure to hold slice data
        struct Slice {
            double y;
            double x_left;
            double x_right;
            double width;
            double x_mid;
            bool is_bloated = false;
        };
        std::vector<Slice> slices;

        double start_y = std::ceil(min_y / step_mm) * step_mm;
        for (double y = start_y; y <= max_y; y += step_mm) {
            std::vector<double> x_intersections;
            size_t n = poly_real_world.size();
            for (size_t i = 0; i < n; ++i) {
                const auto& p1 = poly_real_world[i];
                const auto& p2 = poly_real_world[(i + 1) % n];
                if (p1.size() < 2 || p2.size() < 2) continue;

                double x1 = p1[0], y1 = p1[1];
                double x2 = p2[0], y2 = p2[1];

                if ((y1 <= y && y2 >= y) || (y2 <= y && y1 >= y)) {
                    if (std::abs(y2 - y1) > 1e-5) {
                        double x_int = x1 + (y - y1) * (x2 - x1) / (y2 - y1);
                        x_intersections.push_back(x_int);
                    }
                }
            }

            if (!x_intersections.empty()) {
                auto minmax = std::minmax_element(x_intersections.begin(), x_intersections.end());
                double x_left = *minmax.first;
                double x_right = *minmax.second;
                double width = x_right - x_left;
                double x_mid = (x_left + x_right) / 2.0;
                slices.push_back({y, x_left, x_right, width, x_mid, false});
                if (debug_out) {
                    (*debug_out)["slices"].push_back({{"y", std::round(y*10.0)/10.0}, {"x_left", std::round(x_left*10.0)/10.0}, {"x_right", std::round(x_right*10.0)/10.0}});
                    (*debug_out)["raw_midpoints"].push_back({std::round(x_mid*10.0)/10.0, std::round(y*10.0)/10.0});
                }
            }
        }

        if (slices.empty()) return waypoints;

        // 1. Calculate median width to identify nominal lane width
        std::vector<double> widths;
        widths.reserve(slices.size());
        for (const auto& slice : slices) {
            widths.push_back(slice.width);
        }
        std::sort(widths.begin(), widths.end());
        double w_median = widths[widths.size() / 2];
        if (w_median < 10.0) w_median = 400.0; // Avoid division by zero or extremely small width

        // 2. Identify bloated slices (width > 1.3 * w_median)
        for (auto& slice : slices) {
            if (slice.width > 1.3 * w_median) {
                slice.is_bloated = true;
            }
        }

        // 3. Fit a robust global linear trend (x = m * y + c) on clean slices as a fallback
        double m_global = 0.0, c_global = 0.0;
        bool has_global_trend = false;
        int n_clean = 0;
        double sum_y = 0, sum_x = 0, sum_yy = 0, sum_yx = 0;
        for (const auto& slice : slices) {
            if (!slice.is_bloated) {
                sum_y += slice.y;
                sum_x += slice.x_mid;
                sum_yy += slice.y * slice.y;
                sum_yx += slice.y * slice.x_mid;
                n_clean++;
            }
        }
        if (n_clean >= 2) {
            double denom = n_clean * sum_yy - sum_y * sum_y;
            if (std::abs(denom) > 1e-5) {
                m_global = (n_clean * sum_yx - sum_y * sum_x) / denom;
                c_global = (sum_x - m_global * sum_y) / n_clean;
                has_global_trend = true;
            }
        }

        // 4. Local Sliding Window Filter with Global Fallback for bloated slices (preserving local curves!)
        int n_slices = slices.size();
        for (int i = 0; i < n_slices; ++i) {
            double final_x = slices[i].x_mid;
            
            if (slices[i].is_bloated) {
                // Look in a local window of +/- 3 slices for clean reference centers
                std::vector<double> local_clean_mids;
                for (int j = std::max(0, i - 3); j <= std::min(n_slices - 1, i + 3); ++j) {
                    if (!slices[j].is_bloated) {
                        local_clean_mids.push_back(slices[j].x_mid);
                    }
                }
                
                double local_center = 0.0;
                bool center_found = false;
                if (!local_clean_mids.empty()) {
                    // Use the median of local clean midpoints to represent the curve's center
                    std::sort(local_clean_mids.begin(), local_clean_mids.end());
                    local_center = local_clean_mids[local_clean_mids.size() / 2];
                    center_found = true;
                } else if (has_global_trend) {
                    // Fallback to global trend if local neighborhood is entirely bloated
                    local_center = m_global * slices[i].y + c_global;
                    center_found = true;
                }
                
                if (center_found) {
                    // Clip the bloated slice's boundaries to nominal width around the center
                    double left_dev = std::abs(slices[i].x_left - (local_center - w_median / 2.0));
                    double right_dev = std::abs(slices[i].x_right - (local_center + w_median / 2.0));
                    
                    double dev_diff = std::abs(left_dev - right_dev);
                    if (dev_diff < 50.0) {
                        // Symmetric bloat on both sides -> center it exactly at the local estimate
                        final_x = local_center;
                    } else if (left_dev > right_dev) {
                        // Left boundary leaked significantly more -> clip left boundary
                        final_x = (local_center - w_median / 2.0 + slices[i].x_right) / 2.0;
                    } else {
                        // Right boundary leaked significantly more -> clip right boundary
                        final_x = (slices[i].x_left + local_center + w_median / 2.0) / 2.0;
                    }
                }
            }
            
            waypoints.push_back({final_x, slices[i].y});
            if (debug_out) {
                (*debug_out)["filtered_midpoints"].push_back({std::round(final_x*10.0)/10.0, std::round(slices[i].y*10.0)/10.0});
            }
        }

        std::sort(waypoints.begin(), waypoints.end(), [](const Waypoint& a, const Waypoint& b) {
            return a.y < b.y;
        });

        return waypoints;
    }

    std::vector<Waypoint> extract_centerline_waypoints_x(const std::vector<std::vector<double>>& poly_real_world, double step_mm = 100.0, json* debug_out = nullptr) {
        std::vector<Waypoint> waypoints;
        if (poly_real_world.empty()) return waypoints;

        if (debug_out) {
            if (!debug_out->contains("slices")) (*debug_out)["slices"] = json::array();
            if (!debug_out->contains("raw_midpoints")) (*debug_out)["raw_midpoints"] = json::array();
            if (!debug_out->contains("filtered_midpoints")) (*debug_out)["filtered_midpoints"] = json::array();
        }

        double min_x = 1e9;
        double max_x = -1e9;
        for (const auto& pt : poly_real_world) {
            if (pt.size() >= 2) {
                double x = pt[0];
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
            }
        }

        if (min_x >= max_x) return waypoints;

        // Structure to hold slice data
        struct Slice {
            double x;
            double y_bottom;
            double y_top;
            double width;
            double y_mid;
            bool is_bloated = false;
        };
        std::vector<Slice> slices;

        double start_x = std::ceil(min_x / step_mm) * step_mm;
        for (double x = start_x; x <= max_x; x += step_mm) {
            std::vector<double> y_intersections;
            size_t n = poly_real_world.size();
            for (size_t i = 0; i < n; ++i) {
                const auto& p1 = poly_real_world[i];
                const auto& p2 = poly_real_world[(i + 1) % n];
                if (p1.size() < 2 || p2.size() < 2) continue;

                double x1 = p1[0], y1 = p1[1];
                double x2 = p2[0], y2 = p2[1];

                if ((x1 <= x && x2 >= x) || (x2 <= x && x1 >= x)) {
                    if (std::abs(x2 - x1) > 1e-5) {
                        double y_int = y1 + (x - x1) * (y2 - y1) / (x2 - x1);
                        y_intersections.push_back(y_int);
                    }
                }
            }

            if (!y_intersections.empty()) {
                auto minmax = std::minmax_element(y_intersections.begin(), y_intersections.end());
                double y_bottom = *minmax.first;
                double y_top = *minmax.second;
                double width = y_top - y_bottom;
                double y_mid = (y_bottom + y_top) / 2.0;
                slices.push_back({x, y_bottom, y_top, width, y_mid, false});
                if (debug_out) {
                    (*debug_out)["slices"].push_back({{"x", std::round(x*10.0)/10.0}, {"y_bottom", std::round(y_bottom*10.0)/10.0}, {"y_top", std::round(y_top*10.0)/10.0}});
                    (*debug_out)["raw_midpoints"].push_back({std::round(x*10.0)/10.0, std::round(y_mid*10.0)/10.0});
                }
            }
        }

        if (slices.empty()) return waypoints;

        // 1. Calculate median width to identify nominal lane width
        std::vector<double> widths;
        widths.reserve(slices.size());
        for (const auto& slice : slices) {
            widths.push_back(slice.width);
        }
        std::sort(widths.begin(), widths.end());
        double w_median = widths[widths.size() / 2];
        if (w_median < 10.0) w_median = 400.0; // Avoid division by zero or extremely small width

        // 2. Identify bloated slices (width > 1.3 * w_median)
        for (auto& slice : slices) {
            if (slice.width > 1.3 * w_median) {
                slice.is_bloated = true;
            }
        }

        // 3. Fit a robust global linear trend (y = m * x + c) on clean slices as a fallback
        double m_global = 0.0, c_global = 0.0;
        bool has_global_trend = false;
        int n_clean = 0;
        double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
        for (const auto& slice : slices) {
            if (!slice.is_bloated) {
                sum_x += slice.x;
                sum_y += slice.y_mid;
                sum_xx += slice.x * slice.x;
                sum_xy += slice.x * slice.y_mid;
                n_clean++;
            }
        }
        if (n_clean >= 2) {
            double denom = n_clean * sum_xx - sum_x * sum_x;
            if (std::abs(denom) > 1e-5) {
                m_global = (n_clean * sum_xy - sum_x * sum_y) / denom;
                c_global = (sum_y - m_global * sum_x) / n_clean;
                has_global_trend = true;
            }
        }

        // 4. Local Sliding Window Filter with Global Fallback for bloated slices (preserving local curves!)
        int n_slices = slices.size();
        for (int i = 0; i < n_slices; ++i) {
            double final_y = slices[i].y_mid;
            
            if (slices[i].is_bloated) {
                // Look in a local window of +/- 3 slices for clean reference centers
                std::vector<double> local_clean_mids;
                for (int j = std::max(0, i - 3); j <= std::min(n_slices - 1, i + 3); ++j) {
                    if (!slices[j].is_bloated) {
                        local_clean_mids.push_back(slices[j].y_mid);
                    }
                }
                
                double local_center = 0.0;
                bool center_found = false;
                if (!local_clean_mids.empty()) {
                    // Use the median of local clean midpoints to represent the curve's center
                    std::sort(local_clean_mids.begin(), local_clean_mids.end());
                    local_center = local_clean_mids[local_clean_mids.size() / 2];
                    center_found = true;
                } else if (has_global_trend) {
                    // Fallback to global trend if local neighborhood is entirely bloated
                    local_center = m_global * slices[i].x + c_global;
                    center_found = true;
                }
                
                if (center_found) {
                    // Clip the bloated slice's boundaries to nominal width around the center
                    double bottom_dev = std::abs(slices[i].y_bottom - (local_center - w_median / 2.0));
                    double top_dev = std::abs(slices[i].y_top - (local_center + w_median / 2.0));
                    
                    double dev_diff = std::abs(bottom_dev - top_dev);
                    if (dev_diff < 50.0) {
                        // Symmetric bloat on both sides -> center it exactly at the local estimate
                        final_y = local_center;
                    } else if (bottom_dev > top_dev) {
                        // Bottom boundary leaked significantly more -> clip bottom boundary
                        final_y = (local_center - w_median / 2.0 + slices[i].y_top) / 2.0;
                    } else {
                        // Top boundary leaked significantly more -> clip top boundary
                        final_y = (slices[i].y_bottom + local_center + w_median / 2.0) / 2.0;
                    }
                }
            }
            
            waypoints.push_back({slices[i].x, final_y});
            if (debug_out) {
                (*debug_out)["filtered_midpoints"].push_back({std::round(slices[i].x*10.0)/10.0, std::round(final_y*10.0)/10.0});
            }
        }

        std::sort(waypoints.begin(), waypoints.end(), [](const Waypoint& a, const Waypoint& b) {
            return a.x < b.x;
        });

        return waypoints;
    }

    std::vector<double> fit_polynomial_xy(const std::vector<Waypoint>& waypoints) {
        std::vector<double> coeffs(4, 0.0);
        size_t n = waypoints.size();
        if (n < 2) {
            return coeffs;
        }

        if (n >= 4) {
            cv::Mat A(n, 4, CV_64F);
            cv::Mat B(n, 1, CV_64F);
            for (size_t i = 0; i < n; ++i) {
                double y = waypoints[i].y;
                A.at<double>(i, 0) = y * y * y;
                A.at<double>(i, 1) = y * y;
                A.at<double>(i, 2) = y;
                A.at<double>(i, 3) = 1.0;
                B.at<double>(i, 0) = waypoints[i].x;
            }
            cv::Mat C;
            cv::solve(A, B, C, cv::DECOMP_SVD);
            coeffs[0] = C.at<double>(0, 0); // a3
            coeffs[1] = C.at<double>(1, 0); // a2
            coeffs[2] = C.at<double>(2, 0); // a1
            coeffs[3] = C.at<double>(3, 0); // a0
        } else {
            cv::Mat A(n, 2, CV_64F);
            cv::Mat B(n, 1, CV_64F);
            for (size_t i = 0; i < n; ++i) {
                double y = waypoints[i].y;
                A.at<double>(i, 0) = y;
                A.at<double>(i, 1) = 1.0;
                B.at<double>(i, 0) = waypoints[i].x;
            }
            cv::Mat C;
            cv::solve(A, B, C, cv::DECOMP_SVD);
            coeffs[0] = 0.0; // a3
            coeffs[1] = 0.0; // a2
            coeffs[2] = C.at<double>(0, 0); // a1
            coeffs[3] = C.at<double>(1, 0); // a0
        }

        return coeffs;
    }

    std::vector<double> fit_polynomial_yx(const std::vector<Waypoint>& waypoints) {
        std::vector<double> coeffs(4, 0.0);
        size_t n = waypoints.size();
        if (n < 2) {
            return coeffs;
        }

        if (n >= 4) {
            cv::Mat A(n, 4, CV_64F);
            cv::Mat B(n, 1, CV_64F);
            for (size_t i = 0; i < n; ++i) {
                double x = waypoints[i].x;
                A.at<double>(i, 0) = x * x * x;
                A.at<double>(i, 1) = x * x;
                A.at<double>(i, 2) = x;
                A.at<double>(i, 3) = 1.0;
                B.at<double>(i, 0) = waypoints[i].y;
            }
            cv::Mat C;
            cv::solve(A, B, C, cv::DECOMP_SVD);
            coeffs[0] = C.at<double>(0, 0); // a3
            coeffs[1] = C.at<double>(1, 0); // a2
            coeffs[2] = C.at<double>(2, 0); // a1
            coeffs[3] = C.at<double>(3, 0); // a0
        } else {
            cv::Mat A(n, 2, CV_64F);
            cv::Mat B(n, 1, CV_64F);
            for (size_t i = 0; i < n; ++i) {
                double x = waypoints[i].x;
                A.at<double>(i, 0) = x;
                A.at<double>(i, 1) = 1.0;
                B.at<double>(i, 0) = waypoints[i].y;
            }
            cv::Mat C;
            cv::solve(A, B, C, cv::DECOMP_SVD);
            coeffs[0] = 0.0; // a3
            coeffs[1] = 0.0; // a2
            coeffs[2] = C.at<double>(0, 0); // a1
            coeffs[3] = C.at<double>(1, 0); // a0
        }

        return coeffs;
    }

    void telemetry_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received telemetry message! Length: %zu bytes", msg->data.size());
        // Periodically check/reload calibration file
        check_calibration_update();
        
        // Dynamically update parameter so simulator can toggle it at runtime
        publish_debug_centerline_ = this->get_parameter("publish_debug_centerline").as_bool();

        if (!has_calibration_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "IPM calibration is missing or invalid. Telemetry publish blocked. Please calibrate via the Web Dashboard.");
            return;
        }

        frame_count_++;

        try {
            json telemetry = json::parse(msg->data);

            // If we have calibration, perform the planar homography mapping
            if (telemetry.contains("objects") && telemetry["objects"].is_array()) {
                for (auto& obj : telemetry["objects"]) {
                    obj["polygons_real_world"] = json::array();

                    if (obj.contains("polygons") && obj["polygons"].is_array()) {
                        for (const auto& poly : obj["polygons"]) {
                            json poly_real = json::array();

                            if (poly.is_array()) {
                                // Plan E1: clip the polygon against the valid BEV region and
                                // project it to world coordinates (mm). Edges crossing the
                                // horizon or the world-space box are interpolated at the
                                // boundary instead of dropping vertices, so a sparse
                                // corner-list polygon straddling the horizon keeps its valid
                                // part (see bev_region.hpp for the perspective equations).
                                std::vector<BevPoint> pixel_pts;
                                for (const auto& pt : poly) {
                                    if (pt.is_array() && pt.size() >= 2) {
                                        pixel_pts.push_back({pt[0].get<double>(), pt[1].get<double>()});
                                    }
                                }
                                for (const auto& wp : bev_region_.clip_and_project(H_, pixel_pts)) {
                                    // Rounding for clean representation in JSON (optional, but keeps strings readable)
                                    double X = std::round(wp.x * 10.0) / 10.0;
                                    double Y = std::round(wp.y * 10.0) / 10.0;
                                    poly_real.push_back({X, Y});
                                }
                            }
                            obj["polygons_real_world"].push_back(poly_real);
                        }
                    }

                    // Extract centerline waypoints and fit polynomial for lane objects
                    int label = obj.contains("label") ? obj["label"].get<int>() : -1;
                    bool is_lane = (label == LABEL_MAIN_LANE || label == LABEL_OTHER_LANE || label == LABEL_TURN_LANE);
                    if (is_lane) {
                        obj["waypoints"] = json::array();
                        obj["polynomial"] = {{"a3", 0.0}, {"a2", 0.0}, {"a1", 0.0}, {"a0", 0.0}};
                        obj["lateral_offset_mm"] = 0.0;
                        obj["longitudinal_offset_mm"] = 0.0;
                        obj["heading_angle_rad"] = 0.0;
                        obj["curvature_inv_mm"] = 0.0;

                        if (obj.contains("polygons_real_world") && obj["polygons_real_world"].is_array()) {
                            std::vector<Waypoint> all_waypoints;
                            
                            // Retrieve track/object ID
                            std::string track_id = "";
                            if (obj.contains("track_id") && obj["track_id"].is_string()) {
                                track_id = obj["track_id"].get<std::string>();
                            } else if (obj.contains("id") && obj["id"].is_string()) {
                                track_id = obj["id"].get<std::string>();
                            } else {
                                track_id = "label_" + std::to_string(label);
                            }

                            // Check if turn-lane -> sweep along X and fit y(x)
                            if (label == LABEL_TURN_LANE) {
                                if (publish_debug_centerline_) {
                                    obj["debug_centerline"] = json::object();
                                }
                                std::vector<json> sorted_polys;
                                for (const auto& poly_json : obj["polygons_real_world"]) {
                                    if (poly_json.is_array()) sorted_polys.push_back(poly_json);
                                }
                                std::sort(sorted_polys.begin(), sorted_polys.end(), [](const json& a, const json& b) {
                                    return a.size() > b.size();
                                });
                                
                                for (const auto& poly : sorted_polys) {
                                    std::vector<std::vector<double>> poly_pts;
                                    for (const auto& pt : poly) {
                                        if (pt.is_array() && pt.size() >= 2) {
                                            poly_pts.push_back({pt[0].get<double>(), pt[1].get<double>()});
                                        }
                                    }
                                    // Plan E1: a polygon clipped down to fewer than 3 world points is
                                    // too degenerate to slice a centerline from - skip it instead of
                                    // producing garbage waypoints.
                                    if (poly_pts.size() < 3) continue;
                                    auto wps = extract_centerline_waypoints_x(poly_pts, 100.0, publish_debug_centerline_ ? &obj["debug_centerline"] : nullptr);
                                    all_waypoints.insert(all_waypoints.end(), wps.begin(), wps.end());
                                }

                                if (all_waypoints.size() >= 2) {
                                    std::stable_sort(all_waypoints.begin(), all_waypoints.end(), [](const Waypoint& a, const Waypoint& b) {
                                        return a.x < b.x;
                                    });

                                    std::vector<Waypoint> unique_waypoints;
                                    for (const auto& wp : all_waypoints) {
                                        if (unique_waypoints.empty() || std::abs(unique_waypoints.back().x - wp.x) > 1e-3) {
                                            unique_waypoints.push_back(wp);
                                        } else {
                                            if (std::abs(unique_waypoints.back().y - wp.y) < 300.0) {
                                                unique_waypoints.back().y = (unique_waypoints.back().y + wp.y) / 2.0;
                                            } else {
                                                unique_waypoints.push_back(wp);
                                            }
                                        }
                                    }

                                    // 1. Spatial smoothing of raw waypoints
                                    if (unique_waypoints.size() >= 3) {
                                        std::vector<Waypoint> smoothed = unique_waypoints;
                                        for (size_t i = 1; i < unique_waypoints.size() - 1; ++i) {
                                            smoothed[i].y = (unique_waypoints[i-1].y + unique_waypoints[i].y + unique_waypoints[i+1].y) / 3.0;
                                        }
                                        unique_waypoints = smoothed;
                                    }

                                    // 2. Fit raw polynomial (before smoothing)
                                    std::vector<double> raw_coeffs = fit_polynomial_yx(unique_waypoints);

                                    // 3. Temporal smoothing of sampled waypoints
                                    double alpha = 0.25;
                                    auto& state = track_states_[track_id];
                                    state.last_seen_frame = frame_count_;

                                    double x_start = std::ceil(unique_waypoints.front().x / 100.0) * 100.0;
                                    double x_end = std::floor(unique_waypoints.back().x / 100.0) * 100.0;

                                    std::vector<Waypoint> smoothed_anchor_pts;
                                    for (double x_val = x_start; x_val <= x_end; x_val += 100.0) {
                                        double y_raw = raw_coeffs[0] * std::pow(x_val, 3)
                                                     + raw_coeffs[1] * std::pow(x_val, 2)
                                                     + raw_coeffs[2] * x_val
                                                     + raw_coeffs[3];
                                        
                                        double y_smooth = y_raw;
                                        if (state.has_prev && state.smoothed_val.count(x_val) > 0) {
                                            y_smooth = alpha * y_raw + (1.0 - alpha) * state.smoothed_val[x_val];
                                        }
                                        state.smoothed_val[x_val] = y_smooth;
                                        smoothed_anchor_pts.push_back({x_val, y_smooth});
                                    }

                                    std::vector<double> coeffs;
                                    if (smoothed_anchor_pts.size() >= 2) {
                                        coeffs = fit_polynomial_yx(smoothed_anchor_pts);
                                    } else {
                                        coeffs = raw_coeffs;
                                    }
                                    state.coeffs = coeffs;
                                    state.has_prev = true;

                                    // 4. Regenerate smooth waypoints from the smoothed polynomial
                                    double x_min = unique_waypoints.front().x;
                                    double x_max = unique_waypoints.back().x;
                                    std::vector<Waypoint> smooth_wps;
                                    // Plan E1: kMaxWaypointsPerObject is a safety net; the clipped
                                    // BEV range keeps counts far below it in normal operation.
                                    for (double x_val = x_min; x_val <= x_max && smooth_wps.size() < kMaxWaypointsPerObject; x_val += 100.0) {
                                        double y_val = coeffs[0] * std::pow(x_val, 3) + coeffs[1] * std::pow(x_val, 2) + coeffs[2] * x_val + coeffs[3];
                                        smooth_wps.push_back({x_val, y_val});
                                    }
                                    if ((smooth_wps.empty() || std::abs(smooth_wps.back().x - x_max) > 1e-3) &&
                                        smooth_wps.size() < kMaxWaypointsPerObject) {
                                        double y_val = coeffs[0] * std::pow(x_max, 3) + coeffs[1] * std::pow(x_max, 2) + coeffs[2] * x_max + coeffs[3];
                                        smooth_wps.push_back({x_max, y_val});
                                    }
                                    if (smooth_wps.size() >= kMaxWaypointsPerObject) {
                                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                            "Waypoint cap (%zu) hit for track %s - upstream x-range suspicious",
                                            kMaxWaypointsPerObject, track_id.c_str());
                                    }

                                    // Write smooth waypoints to JSON
                                    for (const auto& wp : smooth_wps) {
                                        double rx = std::round(wp.x * 10.0) / 10.0;
                                        double ry = std::round(wp.y * 10.0) / 10.0;
                                        obj["waypoints"].push_back({rx, ry});
                                    }

                                    // Write smooth polynomial coefficients to JSON
                                    obj["polynomial"]["a3"] = coeffs[0];
                                    obj["polynomial"]["a2"] = coeffs[1];
                                    obj["polynomial"]["a1"] = coeffs[2];
                                    obj["polynomial"]["a0"] = coeffs[3];

                                    // 5. Smooth and write control outputs
                                    double longitudinal_offset = coeffs[3];
                                    double heading_angle = std::atan(coeffs[2]);
                                    double curvature = 2.0 * coeffs[1];

                                    if (state.has_prev_metrics) {
                                        longitudinal_offset = alpha * longitudinal_offset + (1.0 - alpha) * state.longitudinal_offset;
                                        heading_angle = alpha * heading_angle + (1.0 - alpha) * state.heading_angle;
                                        curvature = alpha * curvature + (1.0 - alpha) * state.curvature;
                                    }
                                    state.longitudinal_offset = longitudinal_offset;
                                    state.heading_angle = heading_angle;
                                    state.curvature = curvature;
                                    state.has_prev_metrics = true;

                                    obj["lateral_offset_mm"] = 0.0;
                                    obj["longitudinal_offset_mm"] = std::round(longitudinal_offset * 10.0) / 10.0;
                                    obj["heading_angle_rad"] = std::round(heading_angle * 1000.0) / 1000.0;
                                    obj["curvature_inv_mm"] = std::round(curvature * 1e6) / 1e6;

                                    // 6. Look-ahead for turn-lane (y(x)): target is vehicle centerline x=0
                                    double d_la_turn = compute_lookahead_d();
                                    obj["lookahead_d_mm"]      = std::round(d_la_turn * 10.0) / 10.0;
                                    obj["lookahead_x_mm"]      = 0.0;
                                    obj["lookahead_theta_rad"] = std::round(heading_angle * 1000.0) / 1000.0;
                                }
                            } else {
                                // main-lane (label 3) or other-lane (label 4) -> sweep along Y and fit x(y)
                                if (publish_debug_centerline_) {
                                    obj["debug_centerline"] = json::object();
                                }
                                std::vector<json> sorted_polys;
                                for (const auto& poly_json : obj["polygons_real_world"]) {
                                    if (poly_json.is_array()) sorted_polys.push_back(poly_json);
                                }
                                std::sort(sorted_polys.begin(), sorted_polys.end(), [](const json& a, const json& b) {
                                    return a.size() > b.size();
                                });
                                
                                for (const auto& poly : sorted_polys) {
                                    std::vector<std::vector<double>> poly_pts;
                                    for (const auto& pt : poly) {
                                        if (pt.is_array() && pt.size() >= 2) {
                                            poly_pts.push_back({pt[0].get<double>(), pt[1].get<double>()});
                                        }
                                    }
                                    // Plan E1: same degenerate-polygon skip as the turn-lane branch.
                                    if (poly_pts.size() < 3) continue;
                                    auto wps = extract_centerline_waypoints_y(poly_pts, 100.0, publish_debug_centerline_ ? &obj["debug_centerline"] : nullptr);
                                    all_waypoints.insert(all_waypoints.end(), wps.begin(), wps.end());
                                }

                                if (all_waypoints.size() >= 2) {
                                    std::stable_sort(all_waypoints.begin(), all_waypoints.end(), [](const Waypoint& a, const Waypoint& b) {
                                        return a.y < b.y;
                                    });

                                    std::vector<Waypoint> unique_waypoints;
                                    for (const auto& wp : all_waypoints) {
                                        if (unique_waypoints.empty() || std::abs(unique_waypoints.back().y - wp.y) > 1e-3) {
                                            unique_waypoints.push_back(wp);
                                        } else {
                                            if (std::abs(unique_waypoints.back().x - wp.x) < 300.0) {
                                                unique_waypoints.back().x = (unique_waypoints.back().x + wp.x) / 2.0;
                                            } else {
                                                unique_waypoints.push_back(wp);
                                            }
                                        }
                                    }

                                    // 1. Spatial smoothing of raw waypoints
                                    if (unique_waypoints.size() >= 3) {
                                        std::vector<Waypoint> smoothed = unique_waypoints;
                                        for (size_t i = 1; i < unique_waypoints.size() - 1; ++i) {
                                            smoothed[i].x = (unique_waypoints[i-1].x + unique_waypoints[i].x + unique_waypoints[i+1].x) / 3.0;
                                        }
                                        unique_waypoints = smoothed;
                                    }

                                    // 2. Fit raw polynomial (before smoothing)
                                    std::vector<double> raw_coeffs = fit_polynomial_xy(unique_waypoints);

                                    // 3. Temporal smoothing of sampled waypoints
                                    double alpha = 0.25;
                                    auto& state = track_states_[track_id];
                                    state.last_seen_frame = frame_count_;

                                    double y_start = std::ceil(unique_waypoints.front().y / 100.0) * 100.0;
                                    double y_end = std::floor(unique_waypoints.back().y / 100.0) * 100.0;

                                    std::vector<Waypoint> smoothed_anchor_pts;
                                    for (double y_val = y_start; y_val <= y_end; y_val += 100.0) {
                                        double x_raw = raw_coeffs[0] * std::pow(y_val, 3)
                                                     + raw_coeffs[1] * std::pow(y_val, 2)
                                                     + raw_coeffs[2] * y_val
                                                     + raw_coeffs[3];
                                        
                                        double x_smooth = x_raw;
                                        if (state.has_prev && state.smoothed_val.count(y_val) > 0) {
                                            x_smooth = alpha * x_raw + (1.0 - alpha) * state.smoothed_val[y_val];
                                        }
                                        state.smoothed_val[y_val] = x_smooth;
                                        smoothed_anchor_pts.push_back({x_smooth, y_val});
                                    }

                                    std::vector<double> coeffs;
                                    if (smoothed_anchor_pts.size() >= 2) {
                                        coeffs = fit_polynomial_xy(smoothed_anchor_pts);
                                    } else {
                                        coeffs = raw_coeffs;
                                    }
                                    state.coeffs = coeffs;
                                    state.has_prev = true;

                                    // 4. Regenerate smooth waypoints from the smoothed polynomial
                                    double y_min = unique_waypoints.front().y;
                                    double y_max = unique_waypoints.back().y;
                                    std::vector<Waypoint> smooth_wps;
                                    // Plan E1: kMaxWaypointsPerObject is a safety net; the clipped
                                    // BEV range keeps counts far below it in normal operation.
                                    for (double y_val = y_min; y_val <= y_max && smooth_wps.size() < kMaxWaypointsPerObject; y_val += 100.0) {
                                        double x_val = coeffs[0] * std::pow(y_val, 3) + coeffs[1] * std::pow(y_val, 2) + coeffs[2] * y_val + coeffs[3];
                                        smooth_wps.push_back({x_val, y_val});
                                    }
                                    if ((smooth_wps.empty() || std::abs(smooth_wps.back().y - y_max) > 1e-3) &&
                                        smooth_wps.size() < kMaxWaypointsPerObject) {
                                        double x_val = coeffs[0] * std::pow(y_max, 3) + coeffs[1] * std::pow(y_max, 2) + coeffs[2] * y_max + coeffs[3];
                                        smooth_wps.push_back({x_val, y_max});
                                    }
                                    if (smooth_wps.size() >= kMaxWaypointsPerObject) {
                                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                            "Waypoint cap (%zu) hit for track %s - upstream y-range suspicious",
                                            kMaxWaypointsPerObject, track_id.c_str());
                                    }

                                    // Write smooth waypoints to JSON
                                    for (const auto& wp : smooth_wps) {
                                        double rx = std::round(wp.x * 10.0) / 10.0;
                                        double ry = std::round(wp.y * 10.0) / 10.0;
                                        obj["waypoints"].push_back({rx, ry});
                                    }

                                    // Write smooth polynomial coefficients to JSON
                                    obj["polynomial"]["a3"] = coeffs[0];
                                    obj["polynomial"]["a2"] = coeffs[1];
                                    obj["polynomial"]["a1"] = coeffs[2];
                                    obj["polynomial"]["a0"] = coeffs[3];

                                    // 5. Smooth and write control outputs
                                    double lateral_offset = coeffs[3];
                                    double heading_angle = std::atan(coeffs[2]);
                                    double curvature = 2.0 * coeffs[1];

                                    if (state.has_prev_metrics) {
                                        lateral_offset = alpha * lateral_offset + (1.0 - alpha) * state.lateral_offset;
                                        heading_angle = alpha * heading_angle + (1.0 - alpha) * state.heading_angle;
                                        curvature = alpha * curvature + (1.0 - alpha) * state.curvature;
                                    }
                                    state.lateral_offset = lateral_offset;
                                    state.heading_angle = heading_angle;
                                    state.curvature = curvature;
                                    state.has_prev_metrics = true;

                                    obj["lateral_offset_mm"] = std::round(lateral_offset * 10.0) / 10.0;
                                    obj["longitudinal_offset_mm"] = 0.0;
                                    obj["heading_angle_rad"] = std::round(heading_angle * 1000.0) / 1000.0;
                                    obj["curvature_inv_mm"] = std::round(curvature * 1e6) / 1e6;

                                    // 6. Dynamic look-ahead: evaluate x(y) polynomial at d_lookahead
                                    double d_la = compute_lookahead_d();
                                    double x_wp = coeffs[0] * std::pow(d_la, 3)
                                                + coeffs[1] * std::pow(d_la, 2)
                                                + coeffs[2] * d_la
                                                + coeffs[3];
                                    double theta_la = std::atan2(x_wp, d_la);

                                    obj["lookahead_d_mm"]      = std::round(d_la * 10.0) / 10.0;
                                    obj["lookahead_x_mm"]      = std::round(x_wp * 10.0) / 10.0;
                                    obj["lookahead_theta_rad"] = std::round(theta_la * 1000.0) / 1000.0;
                                }
                            }
                        }
                    }
                }
            }

            // A lane object whose clipped centerline yielded fewer than 2
            // waypoints is pure noise: its lateral/longitudinal offset fields
            // still hold their 0.0 initialization values, which downstream
            // consumers would mistake for a measured "right next to the
            // vehicle" (e.g. the turn proximity gate reading
            // longitudinal_offset_mm). Drop it instead of publishing it.
            if (telemetry.contains("objects") && telemetry["objects"].is_array()) {
                auto& objs = telemetry["objects"];
                objs.erase(
                    std::remove_if(objs.begin(), objs.end(), [](const json& obj) {
                        int label = obj.contains("label") ? obj["label"].get<int>() : -1;
                        bool is_lane = (label == LABEL_MAIN_LANE || label == LABEL_OTHER_LANE || label == LABEL_TURN_LANE);
                        if (!is_lane) return false;
                        return !obj.contains("waypoints") || !obj["waypoints"].is_array() || obj["waypoints"].size() < 2;
                    }),
                    objs.end());
            }

            // Memory Leak Prevention: Cleanup inactive tracks
            for (auto it = track_states_.begin(); it != track_states_.end(); ) {
                if (frame_count_ - it->second.last_seen_frame > 15) {
                    it = track_states_.erase(it);
                } else {
                    ++it;
                }
            }

            // Publish output
            std_msgs::msg::String out_msg;
            
            if (publish_debug_centerline_) {
                out_msg.data = telemetry.dump();
                ipm_debug_pub_->publish(out_msg);
                
                // Strip debug_centerline before publishing to production topic
                for (auto& obj : telemetry["objects"]) {
                    if (obj.contains("debug_centerline")) {
                        obj.erase("debug_centerline");
                    }
                }
            }
            
            out_msg.data = telemetry.dump();
            telemetry_realworld_pub_->publish(out_msg);
            RCLCPP_INFO(this->get_logger(), "Published real-world telemetry! Size: %zu bytes", out_msg.data.size());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing telemetry: %s", e.what());
            
            // Fallback: publish original telemetry message on error
            telemetry_realworld_pub_->publish(*msg);
        }
    }

    std::string calibration_file_path_;
    bool has_calibration_ = false;
    bool publish_debug_centerline_ = false;
    double H_[3][3] = {0};
    std::filesystem::file_time_type last_write_time_;

    // Plan E1: valid BEV region for backprojection (defaults in bev_region.hpp,
    // overridable via bev_* parameters)
    BevRegion bev_region_;

    // Look-ahead parameters
    double T_preview_       = 0.5;    // seconds
    double d_min_mm_        = 150.0;  // mm
    double d_max_mm_        = 600.0;  // mm
    double current_speed_mms_ = 0.0; // mm/s (from /odom_raw)

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_realworld_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ipm_debug_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telemetry_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Temporal smoothing memory using unified LaneSmoothingState
    struct LaneSmoothingState {
        std::vector<double> coeffs;
        double lateral_offset = 0.0;
        double longitudinal_offset = 0.0;
        double heading_angle = 0.0;
        double curvature = 0.0;
        bool has_prev = false;
        bool has_prev_metrics = false;
        uint64_t last_seen_frame = 0;
        std::map<double, double> smoothed_val; // key: sweep coord (y or x), val: smoothed dependent coord (x or y)
    };
    std::map<std::string, LaneSmoothingState> track_states_;
    uint64_t frame_count_ = 0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IPMTransformNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
