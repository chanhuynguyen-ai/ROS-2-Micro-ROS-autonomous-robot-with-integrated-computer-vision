#!/usr/bin/env python3

import json
import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String



CLASS_NAMES = [
    "dashed-white",
    "dashed-yellow",
    "double-solid-white",
    "light_green",
    "light_red",
    "light_yellow",
    "main-lane",
    "other-lane",
    "parking-zone",
    "sign-no-left",
    "sign-no-parking",
    "sign-no-right",
    "sign-parking",
    "sign-stop",
    "sign-turn-left",
    "sign-turn-right",
    "solid-white",
    "solid-yellow",
    "start",
    "stop-line",
    "turn-lane",
    "vehicle",
]


LANE_CLASSES = {
    'dashed-white',
    'dashed-yellow',
    'double-solid-white',
    'main-lane',
    'other-lane',
    'parking-zone',
    'solid-white',
    'solid-yellow',
    'start',
    'stop-line',
    'turn-lane',
    'vehicle',
}


def clamp(value, low, high):
    return max(low, min(high, value))


class LaneTargetFromTelemetryNode(Node):
    def __init__(self):
        super().__init__('lane_target_from_telemetry_node')

        self.declare_parameter('telemetry_topic', '/avs/telemetry_realworld')
        self.declare_parameter('fallback_telemetry_topic', '/avs/telemetry')
        self.declare_parameter('lane_target_topic', '/avs/lane_target')
        self.declare_parameter('control_log_topic', '/avs/control_log')

        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        # Dùng khi telemetry còn là pixel, chưa phải meter.
        self.declare_parameter('pixel_to_meter_x', 0.0025)
        self.declare_parameter('pixel_to_meter_y', 0.0030)

        # Điểm nhìn trước xe để bám làn.
        self.declare_parameter('lookahead_m', 0.65)
        self.declare_parameter('lookahead_pixel_ratio', 0.55)

        self.declare_parameter('min_confidence', 0.20)
        self.declare_parameter('stop_line_distance_m', 0.35)
        self.declare_parameter('target_timeout_s', 0.50)

        self.telemetry_topic = str(self.get_parameter('telemetry_topic').value)
        self.fallback_telemetry_topic = str(self.get_parameter('fallback_telemetry_topic').value)
        self.lane_target_topic = str(self.get_parameter('lane_target_topic').value)
        self.control_log_topic = str(self.get_parameter('control_log_topic').value)

        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.pixel_to_meter_x = float(self.get_parameter('pixel_to_meter_x').value)
        self.pixel_to_meter_y = float(self.get_parameter('pixel_to_meter_y').value)
        self.lookahead_m = float(self.get_parameter('lookahead_m').value)
        self.lookahead_pixel_ratio = float(self.get_parameter('lookahead_pixel_ratio').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.stop_line_distance_m = float(self.get_parameter('stop_line_distance_m').value)
        self.target_timeout_s = float(self.get_parameter('target_timeout_s').value)

        self.lane_target_pub = self.create_publisher(String, self.lane_target_topic, 10)
        self.log_pub = self.create_publisher(String, self.control_log_topic, 10)

        self.sub_real = self.create_subscription(
            String,
            self.telemetry_topic,
            self.telemetry_callback,
            10
        )

        self.sub_raw = self.create_subscription(
            String,
            self.fallback_telemetry_topic,
            self.telemetry_callback,
            10
        )

        self.last_target_time = 0.0
        self.last_source_topic = ''

        self.timer = self.create_timer(0.10, self.timeout_loop)

        self.get_logger().info('lane_target_from_telemetry_node started')
        self.get_logger().info(f'Subscribe primary:  {self.telemetry_topic}')
        self.get_logger().info(f'Subscribe fallback: {self.fallback_telemetry_topic}')
        self.get_logger().info(f'Publish target:     {self.lane_target_topic}')

    def publish_log(self, level, message, extra=None):
        payload = {
            'time': time.time(),
            'node': 'lane_target_from_telemetry_node',
            'level': level,
            'message': message,
            'extra': extra or {},
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.log_pub.publish(msg)

    def get_label(self, obj):
        for key in ['class_name', 'class', 'label', 'name', 'type']:
            if key in obj:
                value = obj[key]

                if isinstance(value, str):
                    return value.strip()

                try:
                    idx = int(value)
                    if 0 <= idx < len(CLASS_NAMES):
                        return CLASS_NAMES[idx]
                except Exception:
                    pass

                return str(value).strip()

        for key in ['class_id', 'cls', 'id', 'category_id']:
            if key in obj:
                try:
                    idx = int(obj[key])
                    if 0 <= idx < len(CLASS_NAMES):
                        return CLASS_NAMES[idx]
                except Exception:
                    pass

        return ''

    def get_conf(self, obj):
        for key in ['confidence', 'conf', 'score', 'prob']:
            if key in obj:
                try:
                    return float(obj[key])
                except Exception:
                    pass
        return 1.0

    def recursively_collect_objects(self, data):
        objects = []

        if isinstance(data, dict):
            label = self.get_label(data)
            if label in LANE_CLASSES:
                objects.append(data)

            for key, value in data.items():
                if isinstance(value, (dict, list)):
                    objects.extend(self.recursively_collect_objects(value))

        elif isinstance(data, list):
            for item in data:
                objects.extend(self.recursively_collect_objects(item))

        return objects

    def parse_point_pair(self, p):
        if isinstance(p, dict):
            # Ưu tiên world/real x,y nếu có.
            if 'x_m' in p and 'y_m' in p:
                return float(p['x_m']), float(p['y_m']), True
            if 'world_x' in p and 'world_y' in p:
                return float(p['world_x']), float(p['world_y']), True
            if 'real_x' in p and 'real_y' in p:
                return float(p['real_x']), float(p['real_y']), True
            if 'x' in p and 'y' in p:
                return float(p['x']), float(p['y']), False

        if isinstance(p, (list, tuple)) and len(p) >= 2:
            return float(p[0]), float(p[1]), False

        return None

    def extract_points(self, obj):
        possible_keys = [
            'realworld_points',
            'world_points',
            'ipm_points',
            'centerline',
            'polyline',
            'points',
            'polygon',
            'contour',
            'mask_points',
            'segmentation',
            'segments',
            'line_points',
            'center_points',
        ]

        points = []
        is_world_any = False

        for key in possible_keys:
            if key not in obj:
                continue

            raw_points = obj[key]
            if not isinstance(raw_points, list):
                continue

            for p in raw_points:
                parsed = self.parse_point_pair(p)
                if parsed is None:
                    continue

                x, y, is_world = parsed
                points.append((x, y))
                is_world_any = is_world_any or is_world

        # Fallback bbox.
        if not points:
            bbox = None

            for key in ['bbox', 'box', 'rect']:
                if key in obj:
                    bbox = obj[key]
                    break

            if isinstance(bbox, (list, tuple)) and len(bbox) >= 4:
                x1, y1, x2, y2 = map(float, bbox[:4])
                # Nếu dạng x,y,w,h thì chuyển.
                if x2 < self.image_width and y2 < self.image_height and x2 > 0 and y2 > 0:
                    if x2 < x1 or y2 < y1:
                        pass
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                points.append((cx, cy))

        # Nếu points có giá trị lớn giống pixel thì coi là pixel.
        if points and not is_world_any:
            max_abs = max(max(abs(x), abs(y)) for x, y in points)
            if max_abs <= 20.0:
                # Có thể telemetry đã là meter dù không ghi rõ.
                is_world_any = True

        return points, is_world_any

    def pixel_to_robot_meter(self, x_px, y_px):
        # Ảnh: x phải dương, y xuống dưới.
        # Robot: x phải dương, y trước dương.
        x_m = (x_px - self.image_width / 2.0) * self.pixel_to_meter_x
        y_m = (self.image_height - y_px) * self.pixel_to_meter_y
        return x_m, y_m

    def choose_target_from_world_points(self, points):
        forward_points = [(x, y) for x, y in points if y > 0.03]

        if not forward_points:
            return None

        target = min(
            forward_points,
            key=lambda p: abs(p[1] - self.lookahead_m)
        )

        # Tính hướng làn từ 2 điểm gần target.
        sorted_points = sorted(forward_points, key=lambda p: p[1])
        if len(sorted_points) >= 2:
            p1 = sorted_points[0]
            p2 = sorted_points[-1]
            heading_error = math.atan2(p2[0] - p1[0], max(p2[1] - p1[1], 1e-6))
        else:
            heading_error = math.atan2(target[0], max(target[1], 1e-6))

        return {
            'target_x_m': float(target[0]),
            'target_y_m': float(target[1]),
            'heading_error_rad': float(heading_error),
        }

    def choose_target_from_pixel_points(self, points):
        if not points:
            return None

        target_y_px = self.image_height * self.lookahead_pixel_ratio
        band = max(20.0, self.image_height * 0.08)

        band_points = [
            (x, y) for x, y in points
            if abs(y - target_y_px) <= band
        ]

        if not band_points:
            band_points = sorted(points, key=lambda p: abs(p[1] - target_y_px))[:20]

        avg_x = sum(p[0] for p in band_points) / max(len(band_points), 1)
        avg_y = sum(p[1] for p in band_points) / max(len(band_points), 1)

        target_x_m, target_y_m = self.pixel_to_robot_meter(avg_x, avg_y)

        top_points = sorted(points, key=lambda p: p[1])[:20]
        bottom_points = sorted(points, key=lambda p: p[1], reverse=True)[:20]

        if top_points and bottom_points:
            top_x = sum(p[0] for p in top_points) / len(top_points)
            top_y = sum(p[1] for p in top_points) / len(top_points)
            bot_x = sum(p[0] for p in bottom_points) / len(bottom_points)
            bot_y = sum(p[1] for p in bottom_points) / len(bottom_points)

            top_m = self.pixel_to_robot_meter(top_x, top_y)
            bot_m = self.pixel_to_robot_meter(bot_x, bot_y)

            heading_error = math.atan2(
                top_m[0] - bot_m[0],
                max(top_m[1] - bot_m[1], 1e-6)
            )
        else:
            heading_error = math.atan2(target_x_m, max(target_y_m, 1e-6))

        return {
            'target_x_m': float(target_x_m),
            'target_y_m': float(target_y_m),
            'heading_error_rad': float(heading_error),
        }

    def classify_flags(self, objects):
        labels = [self.get_label(o) for o in objects]

        flags = {
            'main_lane_visible': 'main-lane' in labels,
            'other_lane_visible': 'other-lane' in labels,
            'solid_yellow_visible': 'solid-yellow' in labels,
            'solid_white_visible': 'solid-white' in labels or 'double-solid-white' in labels,
            'stop_line_visible': 'stop-line' in labels,
            'vehicle_visible': 'vehicle' in labels,
            'turn_lane_visible': 'turn-lane' in labels,
            'parking_zone_visible': 'parking-zone' in labels,
            'start_visible': 'start' in labels,
        }

        return flags

    def estimate_stop_line_close(self, stop_objects):
        if not stop_objects:
            return False

        min_y_m = None

        for obj in stop_objects:
            points, is_world = self.extract_points(obj)

            if not points:
                continue

            if is_world:
                ys = [p[1] for p in points if p[1] > 0.0]
            else:
                converted = [self.pixel_to_robot_meter(x, y) for x, y in points]
                ys = [p[1] for p in converted if p[1] > 0.0]

            if not ys:
                continue

            y = min(ys)
            if min_y_m is None or y < min_y_m:
                min_y_m = y

        if min_y_m is None:
            return False

        return min_y_m <= self.stop_line_distance_m

    def telemetry_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Invalid telemetry JSON: {e}')
            return

        objects = self.recursively_collect_objects(data)
        objects = [o for o in objects if self.get_conf(o) >= self.min_confidence]

        flags = self.classify_flags(objects)

        main_objects = [o for o in objects if self.get_label(o) == 'main-lane']
        stop_objects = [o for o in objects if self.get_label(o) == 'stop-line']

        target = None
        confidence = 0.0

        if main_objects:
            # Chọn main-lane có confidence cao nhất.
            main_obj = max(main_objects, key=self.get_conf)
            confidence = self.get_conf(main_obj)

            points, is_world = self.extract_points(main_obj)

            if points:
                if is_world:
                    target = self.choose_target_from_world_points(points)
                else:
                    target = self.choose_target_from_pixel_points(points)

        stop_line_close = self.estimate_stop_line_close(stop_objects)

        if target is None:
            payload = {
                'valid': False,
                'source_topic': 'unknown',
                'target_x_m': 0.0,
                'target_y_m': 0.0,
                'lateral_error_m': 0.0,
                'heading_error_rad': 0.0,
                'confidence': 0.0,
                'stop_line_close': stop_line_close,
                'mode_hint': 'SEARCH_LANE',
                **flags,
            }
        else:
            lateral_error_m = target['target_x_m']
            heading_error_rad = target['heading_error_rad']

            mode_hint = 'LANE_FOLLOW'
            if stop_line_close:
                mode_hint = 'STOP_LINE'

            payload = {
                'valid': True,
                'source_topic': 'telemetry',
                'target_x_m': float(target['target_x_m']),
                'target_y_m': float(target['target_y_m']),
                'lateral_error_m': float(lateral_error_m),
                'heading_error_rad': float(heading_error_rad),
                'confidence': float(confidence),
                'stop_line_close': bool(stop_line_close),
                'mode_hint': mode_hint,
                **flags,
            }

            self.last_target_time = time.time()

        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.lane_target_pub.publish(out)

    def timeout_loop(self):
        if self.last_target_time <= 0.0:
            return

        if time.time() - self.last_target_time > self.target_timeout_s:
            payload = {
                'valid': False,
                'target_x_m': 0.0,
                'target_y_m': 0.0,
                'lateral_error_m': 0.0,
                'heading_error_rad': 0.0,
                'confidence': 0.0,
                'main_lane_visible': False,
                'stop_line_visible': False,
                'stop_line_close': False,
                'solid_yellow_visible': False,
                'solid_white_visible': False,
                'other_lane_visible': False,
                'vehicle_visible': False,
                'mode_hint': 'SEARCH_LANE',
            }

            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.lane_target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneTargetFromTelemetryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
