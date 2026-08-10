#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"
#include "avs_perception/label_mapping.hpp"
#include "avs_perception/decision_types.hpp"

using namespace avs_perception;

class PathObservationBuilder {
public:
    static PathObservationFrame build(const json& telemetry) {
        PathObservationFrame frame;
        frame.timestamp_ms = telemetry.value("timestamp_ms", static_cast<uint64_t>(0));

        if (!telemetry.contains("objects") || !telemetry["objects"].is_array()) {
            return frame;
        }

        for (const auto& obj : telemetry["objects"]) {
            int label = obj.value("label", -1);
            std::string class_name = obj.value("class_name", "");
            std::string id_str;
            if (obj.contains("id")) {
                const auto& id_val = obj["id"];
                if (id_val.is_string()) id_str = id_val.get<std::string>();
                else if (!id_val.is_null()) id_str = id_val.dump();
            }
            if (id_str.empty() && obj.contains("track_id")) {
                const auto& tid_val = obj["track_id"];
                if (tid_val.is_string()) id_str = tid_val.get<std::string>();
                else if (!tid_val.is_null()) id_str = tid_val.dump();
            }
            if (id_str.empty()) {
                id_str = "obj_" + std::to_string(label) + "_" + std::to_string(frame.lanes.size() + frame.markings.size());
            }

            if (label == LABEL_MAIN_LANE || label == LABEL_OTHER_LANE || label == LABEL_TURN_LANE) {
                LaneObservation lane;
                lane.lane_id = id_str;
                lane.class_name = class_name;
                lane.label = label;
                lane.raw_obj = obj;

                std::vector<Point2D> raw_points;
                if (obj.contains("waypoints") && obj["waypoints"].is_array()) {
                    for (const auto& pt : obj["waypoints"]) {
                        if (pt.is_array() && pt.size() >= 2) {
                            Point2D p;
                            p.x = pt[0].get<double>();
                            p.y = pt[1].get<double>();
                            raw_points.push_back(p);
                        }
                    }
                }

                // Sort points
                if (label != LABEL_TURN_LANE) {
                    std::sort(raw_points.begin(), raw_points.end(), [](const Point2D& a, const Point2D& b) {
                        return a.y < b.y;
                    });
                } else {
                    if (!raw_points.empty()) {
                        double dist_front = raw_points.front().x * raw_points.front().x + raw_points.front().y * raw_points.front().y;
                        double dist_back = raw_points.back().x * raw_points.back().x + raw_points.back().y * raw_points.back().y;
                        if (dist_back < dist_front) {
                            std::reverse(raw_points.begin(), raw_points.end());
                        }
                    }
                }

                // Remove duplicate/too-close points (>10mm)
                if (!raw_points.empty()) {
                    lane.points.push_back(raw_points.front());
                    for (size_t i = 1; i < raw_points.size(); ++i) {
                        double dist = std::sqrt(std::pow(raw_points[i].x - lane.points.back().x, 2) +
                                                std::pow(raw_points[i].y - lane.points.back().y, 2));
                        if (dist > 10.0) {
                            lane.points.push_back(raw_points[i]);
                        }
                    }
                }

                // Precomputed control fields compatibility
                if (lane.points.size() < 2 && label == LABEL_TURN_LANE && obj.contains("longitudinal_offset_mm")) {
                    lane.has_precomputed_control = true;
                    lane.precomputed_epsilon_x_mm = 0.0;
                    lane.precomputed_epsilon_y_mm = obj["longitudinal_offset_mm"].get<double>();
                    lane.precomputed_theta_rad = obj.value("lookahead_theta_rad", 0.0);
                    lane.precomputed_curvature_inv_mm = obj.value("curvature_inv_mm", 0.0);
                    lane.precomputed_lookahead_d_mm = obj.value("lookahead_d_mm", lane.precomputed_epsilon_y_mm);
                } else if (lane.points.size() < 2 && obj.contains("lookahead_x_mm") && obj.contains("lookahead_d_mm")) {
                    lane.has_precomputed_control = true;
                    lane.precomputed_epsilon_x_mm = obj["lookahead_x_mm"].get<double>();
                    lane.precomputed_epsilon_y_mm = obj["lookahead_d_mm"].get<double>();
                    lane.precomputed_theta_rad = obj.value(
                        "lookahead_theta_rad",
                        std::atan2(lane.precomputed_epsilon_x_mm, lane.precomputed_epsilon_y_mm)
                    );
                    lane.precomputed_curvature_inv_mm = obj.value("curvature_inv_mm", 0.0);
                    lane.precomputed_lookahead_d_mm = obj["lookahead_d_mm"].get<double>();
                }

                // Metadata & confidence
                if (lane.points.size() >= 2) {
                    lane.heading_hint = std::atan2(lane.points[1].x - lane.points[0].x, lane.points[1].y - lane.points[0].y);

                    double dx = lane.points.back().x - lane.points.front().x;
                    double dy = lane.points.back().y - lane.points.front().y;
                    double total_len = std::sqrt(dx * dx + dy * dy);

                    double len_factor = std::min(1.0, total_len / 5000.0);
                    double pts_factor = std::min(1.0, static_cast<double>(lane.points.size()) / 10.0);
                    lane.confidence = 0.5 * len_factor + 0.5 * pts_factor;
                } else if (lane.has_precomputed_control) {
                    lane.confidence = 1.0;
                    lane.heading_hint = lane.precomputed_theta_rad;
                } else {
                    lane.confidence = 0.0;
                }

                lane.confidence = obj.value("confidence", lane.confidence);
                frame.lanes.push_back(lane);

            } else if (label == LABEL_DASHED_WHITE || label == LABEL_DASHED_YELLOW || label == LABEL_DOUBLE_SOLID_WHITE || label == LABEL_SOLID_WHITE || label == LABEL_SOLID_YELLOW || label == LABEL_STOP_LINE) {
                MarkingObservation marking;
                marking.marking_id = id_str;
                marking.class_name = class_name;
                marking.label = label;
                marking.raw_obj = obj;

                std::vector<Point2D> raw_points;
                if (obj.contains("waypoints") && obj["waypoints"].is_array()) {
                    for (const auto& pt : obj["waypoints"]) {
                        if (pt.is_array() && pt.size() >= 2) {
                            Point2D p;
                            p.x = pt[0].get<double>();
                            p.y = pt[1].get<double>();
                            raw_points.push_back(p);
                        }
                    }
                }

                std::sort(raw_points.begin(), raw_points.end(), [](const Point2D& a, const Point2D& b) {
                    return a.y < b.y;
                });

                if (!raw_points.empty()) {
                    marking.points.push_back(raw_points.front());
                    for (size_t i = 1; i < raw_points.size(); ++i) {
                        double dist = std::sqrt(std::pow(raw_points[i].x - marking.points.back().x, 2) +
                                                std::pow(raw_points[i].y - marking.points.back().y, 2));
                        if (dist > 10.0) {
                            marking.points.push_back(raw_points[i]);
                        }
                    }
                }

                marking.confidence = obj.value("confidence", 1.0);
                frame.markings.push_back(marking);
            }
        }

        return frame;
    }
};
