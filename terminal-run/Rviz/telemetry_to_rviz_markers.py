#!/usr/bin/env python3
"""
telemetry_to_rviz_markers.py

Subscribe /avs/telemetry_realworld (std_msgs/String JSON).
Publish visualization_msgs/MarkerArray on /avs/lane_markers.

Markers render in base_footprint frame so they always appear
in front of the car model in RViz (bird's eye view).

Coordinate convention from IPM:
  X = lateral offset (mm), positive = left of vehicle
  Y = longitudinal distance ahead (mm)

RViz base_footprint:
  X = forward, Y = left, Z = up

So we map:
  rviz_x = telemetry_Y / 1000  (forward)
  rviz_y = telemetry_X / 1000  (left)
  rviz_z = ground level (0.005 for polygons, slightly higher for waypoints)
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


# ================================================================
# CLASS NAMES AND COLORS (mirror web dashboard / label_mapping.json)
# ================================================================

CLASS_NAMES = [
    "dashed-white",       # 0
    "dashed-yellow",      # 1
    "double-solid-white", # 2
    "light_green",        # 3
    "light_red",          # 4
    "light_yellow",       # 5
    "main-lane",          # 6
    "other-lane",         # 7
    "parking-zone",       # 8
    "sign-no-left",       # 9
    "sign-no-parking",    # 10
    "sign-no-right",      # 11
    "sign-parking",       # 12
    "sign-stop",          # 13
    "sign-turn-left",     # 14
    "sign-turn-right",    # 15
    "solid-white",        # 16
    "solid-yellow",       # 17
    "start",              # 18
    "stop-line",          # 19
    "turn-lane",          # 20
    "vehicle",            # 21
]

# RGBA colors matching the web dashboard (values 0-1)
CLASS_COLORS = [
    (51/255.0, 102/255.0, 255/255.0, 0.8),    # 0: dashed-white
    (255/255.0, 153/255.0, 0.0, 0.8),         # 1: dashed-yellow
    (0.0, 127/255.0, 255/255.0, 0.8),         # 2: double-solid-white
    (0.0, 200/255.0, 120/255.0, 0.8),         # 3: light_green
    (255/255.0, 80/255.0, 80/255.0, 0.8),     # 4: light_red
    (255/255.0, 255/255.0, 128/255.0, 0.8),   # 5: light_yellow
    (0.0, 255/255.0, 102/255.0, 0.8),         # 6: main-lane
    (255/255.0, 51/255.0, 51/255.0, 0.8),     # 7: other-lane
    (128/255.0, 128/255.0, 128/255.0, 0.8),   # 8: parking-zone
    (220/255.0, 20/255.0, 60/255.0, 0.8),     # 9: sign-no-left
    (180/255.0, 0.0, 0.0, 0.8),               # 10: sign-no-parking
    (150/255.0, 50/255.0, 50/255.0, 0.8),     # 11: sign-no-right
    (50/255.0, 100/255.0, 230/255.0, 0.8),    # 12: sign-parking
    (255/255.0, 0.0, 0.0, 0.8),               # 13: sign-stop
    (135/255.0, 206/255.0, 235/255.0, 0.8),   # 14: sign-turn-left
    (70/255.0, 130/255.0, 180/255.0, 0.8),    # 15: sign-turn-right
    (0.0, 242/255.0, 254/255.0, 0.8),         # 16: solid-white
    (255/255.0, 255/255.0, 0.0, 0.8),         # 17: solid-yellow
    (0.0, 255/255.0, 127/255.0, 0.8),         # 18: start
    (128/255.0, 0.0, 0.0, 0.8),               # 19: stop-line
    (170/255.0, 0.0, 255/255.0, 0.8),         # 20: turn-lane
    (255/255.0, 0.0, 255/255.0, 0.8),         # 21: vehicle
]

# Lane labels that should have waypoints/trajectory drawn
LANE_LABELS = {6, 7, 20}


def make_color(label_idx):
    """Return a ColorRGBA for the given label index."""
    r, g, b, a = CLASS_COLORS[label_idx % len(CLASS_COLORS)]
    c = ColorRGBA()
    c.r = r
    c.g = g
    c.b = b
    c.a = a
    return c


def telemetry_to_rviz(x_mm, y_mm, z=0.005):
    """
    Convert telemetry coordinates (mm, vehicle-centric BEV)
    to RViz base_footprint coordinates (meters).

    Telemetry: X = lateral (+ left), Y = forward distance
    RViz:      X = forward,          Y = left
    """
    p = Point()
    p.x = y_mm / 1000.0   # forward
    p.y = x_mm / 1000.0   # left
    p.z = z
    return p


def triangulate_polygon(points):
    """
    Simple fan triangulation of a convex-ish polygon.
    Returns list of Point triples for TRIANGLE_LIST.
    Adds triangles in both CCW and CW order to defeat RViz backface culling.
    """
    if len(points) < 3:
        return []
    triangles = []
    p0 = points[0]
    for i in range(1, len(points) - 1):
        p1 = points[i]
        p2 = points[i + 1]
        triangles.extend([p0, p1, p2])
        triangles.extend([p0, p2, p1])
    return triangles


class TelemetryToRvizMarkers(Node):

    def __init__(self):
        super().__init__('telemetry_to_rviz_markers')

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('marker_topic', '/avs/lane_markers')
        self.declare_parameter('polygon_z', 0.005)     # meters above ground
        self.declare_parameter('waypoint_z', 0.015)
        self.declare_parameter('trajectory_z', 0.025)
        self.declare_parameter('waypoint_radius', 0.012)
        self.declare_parameter('trajectory_width', 0.018)
        self.declare_parameter('polygon_line_width', 0.008)

        self.base_frame = str(
            self.get_parameter('base_frame').value
        )
        self.marker_topic = str(
            self.get_parameter('marker_topic').value
        )

        # ============================================================
        # Publisher
        # ============================================================

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10
        )

        # ============================================================
        # Subscriber
        # ============================================================

        self.telemetry_sub = self.create_subscription(
            String,
            '/avs/telemetry_realworld',
            self.telemetry_callback,
            10
        )

        # ============================================================
        # State
        # ============================================================

        self.prev_marker_count = 0

        self.get_logger().info(
            '========================================='
        )
        self.get_logger().info(
            'Telemetry -> RViz Markers node started'
        )
        self.get_logger().info(
            f'Subscribe: /avs/telemetry_realworld'
        )
        self.get_logger().info(
            f'Publish:   {self.marker_topic}'
        )
        self.get_logger().info(
            f'Frame:     {self.base_frame}'
        )
        self.get_logger().info(
            '========================================='
        )

    # ================================================================
    # TELEMETRY CALLBACK
    # ================================================================

    def telemetry_callback(self, msg):
        try:
            telemetry = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(
                f'JSON parse error: {e}'
            )
            return

        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        marker_id = 0

        poly_z = float(self.get_parameter('polygon_z').value)
        wp_z = float(self.get_parameter('waypoint_z').value)
        traj_z = float(self.get_parameter('trajectory_z').value)
        wp_radius = float(self.get_parameter('waypoint_radius').value)
        traj_width = float(self.get_parameter('trajectory_width').value)
        poly_line_w = float(self.get_parameter('polygon_line_width').value)

        # ============================================================
        # Draw object polygons (filled + outline)
        # ============================================================

        objects = telemetry.get('objects', [])

        for obj in objects:
            label = obj.get('label', -1)
            if label < 0 or label >= len(CLASS_COLORS):
                continue

            color = make_color(label)
            polygons_rw = obj.get('polygons_real_world', [])

            for poly in polygons_rw:
                if not poly or len(poly) < 3:
                    continue

                # Convert polygon points
                rviz_pts = [
                    telemetry_to_rviz(pt[0], pt[1], poly_z)
                    for pt in poly
                    if len(pt) >= 2
                ]

                if len(rviz_pts) < 3:
                    continue

                # --- Filled polygon (TRIANGLE_LIST) ---
                tri_pts = triangulate_polygon(rviz_pts)
                if tri_pts:
                    m = Marker()
                    m.header.stamp = stamp
                    m.header.frame_id = self.base_frame
                    m.ns = 'avs_lane_fill'
                    m.id = marker_id
                    marker_id += 1
                    m.type = Marker.TRIANGLE_LIST
                    m.action = Marker.ADD
                    m.pose.orientation.w = 1.0
                    m.scale.x = 1.0
                    m.scale.y = 1.0
                    m.scale.z = 1.0
                    m.points = tri_pts

                    # Apply global color
                    fill_color = ColorRGBA()
                    fill_color.r = color.r
                    fill_color.g = color.g
                    fill_color.b = color.b
                    fill_color.a = color.a * 0.4  # slightly transparent fill
                    m.color = fill_color

                    m.lifetime.sec = 0
                    m.lifetime.nanosec = 0  # infinite lifetime to prevent flickering

                    markers.markers.append(m)

                # --- Polygon outline (LINE_STRIP) ---
                m_line = Marker()
                m_line.header.stamp = stamp
                m_line.header.frame_id = self.base_frame
                m_line.ns = 'avs_lane_outline'
                m_line.id = marker_id
                marker_id += 1
                m_line.type = Marker.LINE_STRIP
                m_line.action = Marker.ADD
                m_line.pose.orientation.w = 1.0
                m_line.scale.x = poly_line_w

                outline_color = ColorRGBA()
                outline_color.r = color.r
                outline_color.g = color.g
                outline_color.b = color.b
                outline_color.a = min(1.0, color.a + 0.3)
                m_line.color = outline_color

                # Close the polygon
                closed_pts = list(rviz_pts) + [rviz_pts[0]]
                m_line.points = closed_pts

                m_line.lifetime.sec = 0
                m_line.lifetime.nanosec = 0

                markers.markers.append(m_line)

            # ============================================================
            # Draw waypoints (SPHERE_LIST) for lane objects
            # ============================================================

            waypoints = obj.get('waypoints', [])
            if waypoints and label in LANE_LABELS:
                m_wp = Marker()
                m_wp.header.stamp = stamp
                m_wp.header.frame_id = self.base_frame
                m_wp.ns = 'avs_lane_waypoints'
                m_wp.id = marker_id
                marker_id += 1
                m_wp.type = Marker.SPHERE_LIST
                m_wp.action = Marker.ADD
                m_wp.pose.orientation.w = 1.0
                m_wp.scale.x = wp_radius * 2
                m_wp.scale.y = wp_radius * 2
                m_wp.scale.z = wp_radius * 2

                wp_color = ColorRGBA()
                wp_color.r = 1.0
                wp_color.g = 1.0
                wp_color.b = 1.0
                wp_color.a = 0.9
                m_wp.color = wp_color

                for wp in waypoints:
                    if len(wp) >= 2:
                        m_wp.points.append(
                            telemetry_to_rviz(wp[0], wp[1], wp_z)
                        )

                m_wp.lifetime.sec = 0
                m_wp.lifetime.nanosec = 0

                markers.markers.append(m_wp)

            # ============================================================
            # Draw waypoint connecting line (LINE_STRIP)
            # ============================================================

            if waypoints and len(waypoints) > 1 and label in LANE_LABELS:
                m_wl = Marker()
                m_wl.header.stamp = stamp
                m_wl.header.frame_id = self.base_frame
                m_wl.ns = 'avs_lane_waypoint_line'
                m_wl.id = marker_id
                marker_id += 1
                m_wl.type = Marker.LINE_STRIP
                m_wl.action = Marker.ADD
                m_wl.pose.orientation.w = 1.0
                m_wl.scale.x = poly_line_w * 1.5

                wl_color = ColorRGBA()
                wl_color.r = color.r
                wl_color.g = color.g
                wl_color.b = color.b
                wl_color.a = 1.0
                m_wl.color = wl_color

                for wp in waypoints:
                    if len(wp) >= 2:
                        m_wl.points.append(
                            telemetry_to_rviz(wp[0], wp[1], wp_z + 0.002)
                        )

                m_wl.lifetime.sec = 0
                m_wl.lifetime.nanosec = 0

                markers.markers.append(m_wl)

        # ============================================================
        # Draw active trajectory (neon cyan glow)
        # ============================================================

        traj_pts = telemetry.get('active_trajectory_points', [])
        if traj_pts and len(traj_pts) > 1:
            # Outer glow line (wider, semi-transparent)
            m_glow = Marker()
            m_glow.header.stamp = stamp
            m_glow.header.frame_id = self.base_frame
            m_glow.ns = 'avs_active_trajectory_glow'
            m_glow.id = marker_id
            marker_id += 1
            m_glow.type = Marker.LINE_STRIP
            m_glow.action = Marker.ADD
            m_glow.pose.orientation.w = 1.0
            m_glow.scale.x = traj_width * 2.5

            glow_color = ColorRGBA()
            glow_color.r = 0.0
            glow_color.g = 1.0
            glow_color.b = 1.0
            glow_color.a = 0.3
            m_glow.color = glow_color

            for pt in traj_pts:
                if len(pt) >= 2:
                    m_glow.points.append(
                        telemetry_to_rviz(pt[0], pt[1], traj_z)
                    )

            m_glow.lifetime.sec = 0
            m_glow.lifetime.nanosec = 0
            markers.markers.append(m_glow)

            # Inner bright line
            m_traj = Marker()
            m_traj.header.stamp = stamp
            m_traj.header.frame_id = self.base_frame
            m_traj.ns = 'avs_active_trajectory'
            m_traj.id = marker_id
            marker_id += 1
            m_traj.type = Marker.LINE_STRIP
            m_traj.action = Marker.ADD
            m_traj.pose.orientation.w = 1.0
            m_traj.scale.x = traj_width

            traj_color = ColorRGBA()
            traj_color.r = 0.0
            traj_color.g = 1.0
            traj_color.b = 1.0
            traj_color.a = 1.0
            m_traj.color = traj_color

            for pt in traj_pts:
                if len(pt) >= 2:
                    m_traj.points.append(
                        telemetry_to_rviz(pt[0], pt[1], traj_z + 0.003)
                    )

            m_traj.lifetime.sec = 0
            m_traj.lifetime.nanosec = 0
            markers.markers.append(m_traj)

        # ============================================================
        # Draw target lookahead point
        # ============================================================

        eps_x = telemetry.get('epsilon_x_mm')
        eps_y = telemetry.get('epsilon_y_mm')
        if eps_x is not None and eps_y is not None:
            m_target = Marker()
            m_target.header.stamp = stamp
            m_target.header.frame_id = self.base_frame
            m_target.ns = 'avs_target_lookahead'
            m_target.id = marker_id
            marker_id += 1
            m_target.type = Marker.SPHERE
            m_target.action = Marker.ADD
            m_target.pose.orientation.w = 1.0
            m_target.pose.position = telemetry_to_rviz(
                eps_x, eps_y, traj_z + 0.01
            )
            m_target.scale.x = 0.035
            m_target.scale.y = 0.035
            m_target.scale.z = 0.035

            target_color = ColorRGBA()
            target_color.r = 1.0
            target_color.g = 0.0
            target_color.b = 0.5
            target_color.a = 1.0
            m_target.color = target_color

            m_target.lifetime.sec = 0
            m_target.lifetime.nanosec = 0
            markers.markers.append(m_target)

        # ============================================================
        # Delete old markers if count decreased
        # ============================================================

        current_count = marker_id
        if current_count < self.prev_marker_count:
            # Send DELETE for orphaned marker IDs
            for ns in [
                'avs_lane_fill',
                'avs_lane_outline',
                'avs_lane_waypoints',
                'avs_lane_waypoint_line',
                'avs_active_trajectory_glow',
                'avs_active_trajectory',
                'avs_target_lookahead',
            ]:
                for old_id in range(current_count, self.prev_marker_count + 50):
                    m_del = Marker()
                    m_del.header.stamp = stamp
                    m_del.header.frame_id = self.base_frame
                    m_del.ns = ns
                    m_del.id = old_id
                    m_del.action = Marker.DELETE
                    markers.markers.append(m_del)

        self.prev_marker_count = current_count

        # ============================================================
        # Publish
        # ============================================================

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryToRvizMarkers()

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
