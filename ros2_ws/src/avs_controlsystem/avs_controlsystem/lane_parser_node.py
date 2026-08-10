#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point


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


class LaneParserNode(Node):
    def __init__(self):
        super().__init__('lane_parser_node')

        self.declare_parameter('image_width', 640.0)
        self.declare_parameter('image_height', 480.0)
        self.declare_parameter('target_class', 'main-lane')
        self.declare_parameter('lookahead_ratio', 0.70)
        self.declare_parameter('near_ratio', 0.85)
        self.declare_parameter('min_points', 8)

        self.sub = self.create_subscription(
            String,
            '/avs/telemetry',
            self.telemetry_callback,
            10
        )

        self.pub = self.create_publisher(Point, '/lane_target', 10)

        self.get_logger().info('avs_controlsystem lane_parser_node started')

    def telemetry_callback(self, msg):
        out = Point()
        out.z = 0.0

        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Invalid JSON from /avs/telemetry: {e}')
            self.pub.publish(out)
            return

        objects = data.get('objects', [])

        image_width = float(self.get_parameter('image_width').value)
        image_height = float(self.get_parameter('image_height').value)
        target_class = str(self.get_parameter('target_class').value)
        lookahead_ratio = float(self.get_parameter('lookahead_ratio').value)
        near_ratio = float(self.get_parameter('near_ratio').value)
        min_points = int(self.get_parameter('min_points').value)

        if target_class not in CLASS_NAMES:
            self.get_logger().warn(f'Unknown target_class: {target_class}')
            self.pub.publish(out)
            return

        target_label = CLASS_NAMES.index(target_class)

        points = []

        for obj in objects:
            if int(obj.get('label', -1)) != target_label:
                continue

            polygons = obj.get('polygons', [])
            for poly in polygons:
                for p in poly:
                    if len(p) >= 2:
                        points.append((float(p[0]), float(p[1])))

        if len(points) < min_points:
            self.pub.publish(out)
            return

        lookahead_y = image_height * lookahead_ratio
        near_y = image_height * near_ratio

        look_band = image_height * 0.08
        near_band = image_height * 0.08

        look_pts = [(x, y) for x, y in points if abs(y - lookahead_y) < look_band]
        near_pts = [(x, y) for x, y in points if abs(y - near_y) < near_band]

        if len(look_pts) < 2:
            lower_pts = [(x, y) for x, y in points if y > image_height * 0.55]
            if len(lower_pts) < min_points:
                self.pub.publish(out)
                return

            target_x = sum(x for x, _ in lower_pts) / len(lower_pts)
            near_x = target_x
        else:
            target_x = sum(x for x, _ in look_pts) / len(look_pts)

            if len(near_pts) >= 2:
                near_x = sum(x for x, _ in near_pts) / len(near_pts)
            else:
                near_x = target_x

        image_center_x = image_width / 2.0

        e_y = (target_x - image_center_x) / image_width

        dy = max(near_y - lookahead_y, 1.0)
        e_theta = math.atan2(target_x - near_x, dy)

        out.x = float(e_y)
        out.y = float(e_theta)
        out.z = 1.0

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LaneParserNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
