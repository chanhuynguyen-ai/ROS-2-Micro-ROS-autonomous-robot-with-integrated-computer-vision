#!/usr/bin/env python3
"""Ghi lại một lượt chạy sa bàn, tập trung vào cú rẽ.

Khác `config/record_run.py` (bản cũ, bỏ hình học đi cho nhẹ): script này GIỮ
`active_trajectory_points` và `debug_trajectories[stage=candidate|committed]`.
Đó chính là đầu vào thật của `extend_to_turn_angle`, thứ duy nhất cho phép chạy
lại chuỗi guard offline — nếu không ghi, cú rẽ hỏng không tài nào truy lại được.

Ngoài file jsonl, script in ra sự kiện latch ngay lúc chạy (đóng/nhả latch kèm
số đo) để người đứng cạnh xe biết ngay cú rẽ vừa rồi tốt hay xấu, không phải
đợi phân tích.

Chạy trong container avs_perception trên Pi:
  ROS_DOMAIN_ID=20 python3 /workspace/config/turn_observe.py \
      /workspace/config/run19.jsonl 120
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

OUT = sys.argv[1] if len(sys.argv) > 1 else "/workspace/config/turn_run.jsonl"
DURATION_S = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0

# Bắt thêm telemetry thô (waypoint sau IPM + polygon ảnh) - MẶC ĐỊNH TẮT.
# Bật bằng tham số thứ 3 = "telemetry".
#
# ĐỪNG bật khi đang cho xe chạy tự động. Đo trên xe (run26): Pi 5 đã bão hoà
# sẵn (load 5.5/4 nhân, ncnn_inference một mình ~191% CPU), và hai topic này
# ~16KB ở 12Hz buộc rclpy giải tuần tự trong tiến trình recorder dù callback
# bỏ đi phần lớn - recorder ngốn 26% một nhân, pipeline thị giác tụt từ 12.0
# xuống 8.7 FPS, controller đói lệnh mới và XE CHẠY GIẬT (gia tốc trung vị
# 0.13 -> 0.40 m/s², 7 lần tụt tốc về 0 rồi chạy lại, run24 không có lần nào).
# Chỉ bật khi xe kê bánh hoặc khi phân tích offline cần hình học thô.
CAPTURE_TELEMETRY = len(sys.argv) > 3 and sys.argv[3] == "telemetry"

# Các stage cần cho việc chạy lại guard offline. "normalized" bỏ đi: nó suy ra
# được từ candidate + committed và là stage nặng nhất.
KEEP_STAGES = ("candidate", "committed")

# Label của turn-lane. Xem CLAUDE.md: 20, KHÔNG phải 17 (solid-yellow) hay 10
# (sign-no-parking) - repo từng có regression đúng chỗ này.
LABEL_TURN_LANE = 20
LABEL_MAIN_LANE = 6
KEEP_LABELS = (LABEL_TURN_LANE, LABEL_MAIN_LANE)


def yaw_deg_from_quat(q):
    """Yaw ROS (CCW dương quanh +z) theo độ. Rẽ TRÁI làm giá trị này TĂNG."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


def unwrap_deg(prev, cur):
    """Nối góc cho liên tục qua mốc ±180 để cộng dồn được góc xoay cả cú rẽ."""
    d = cur - prev
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


class TurnObserver(Node):
    def __init__(self):
        super().__init__("turn_observer")
        self.f = open(OUT, "w", buffering=1)
        self.t0 = time.time()
        self.counts = {}

        # Trạng thái để phát hiện cạnh lên/xuống của latch và cộng dồn yaw đo.
        self.latch_active = False
        self.yaw_deg = None
        self.yaw_unwrapped = 0.0
        self.yaw_seen = False
        self.yaw_at_latch = None
        self.latch_len_at_close = 0.0
        self.yaw_at_intent = None
        self.last_intent = "FOLLOW_LANE"
        self.latch_frames = 0
        self.latch_progress_max = 0.0
        self.prev_progress = 0.0
        self.skip_events = 0
        self.stub_frames = 0
        self.last_print = 0.0
        self.frames = 0

        self.create_subscription(String, "/avs/lane_state", self.on_lane_state, 10)
        self.create_subscription(String, "/avs/control_error", self.on_control_error, 10)
        self.create_subscription(String, "/avs/route_intent", self.on_intent, 10)
        self.create_subscription(String, "/avs/route_intent_ack", self.on_intent_ack, 10)
        self.create_subscription(Twist, "/avs/lane_ref_cmd", self.on_ref, 10)
        # Lệnh THẬT gửi xuống ESP32. Firmware chỉ nhận đúng topic này (xem
        # `ros2 node info /YB_Car_Node`): không có topic điều khiển từng bánh,
        # nên mọi hành vi bánh xe đều suy ra từ đây. Twist chỉ vài chục byte ở
        # ~50Hz - rẻ, không phải thứ gây nghẽn như CAPTURE_TELEMETRY.
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 20)
        self.create_subscription(Odometry, "/odom_raw", self.on_odom, 20)
        # Hai topic dưới đây để truy nguồn độ cong: waypoint thô sau IPM và
        # polygon trong ảnh trước IPM. Đo run23-25: độ cong đã thoải ~713mm ngay
        # ở candidate, tức mất trước khi vào control_node - muốn biết IPM có phải
        # thủ phạm thì phải tự chiếu lại polygon ảnh, nên cần cả hai.
        # Lọc còn turn-lane + main-lane, nếu không một lượt 420s ra ~80MB.
        if CAPTURE_TELEMETRY:
            self.create_subscription(String, "/avs/telemetry_realworld",
                                     lambda m: self.on_telemetry("telemetry_realworld", m), 5)
            self.create_subscription(String, "/avs/telemetry",
                                     lambda m: self.on_telemetry("telemetry_image", m), 5)

        self.log("ghi vào %s, %.0fs%s" % (
            OUT, DURATION_S,
            "  [BẮT TELEMETRY - làm chậm pipeline, đừng dùng khi xe tự chạy]"
            if CAPTURE_TELEMETRY else ""))

    # ── ghi ──────────────────────────────────────────────────────────────────
    def rec(self, topic, payload):
        row = {"t": round(time.time() - self.t0, 3), "topic": topic, "data": payload}
        self.f.write(json.dumps(row) + "\n")
        self.counts[topic] = self.counts.get(topic, 0) + 1

    def log(self, msg):
        line = "[%6.1fs] %s" % (time.time() - self.t0, msg)
        print(line, flush=True)
        self.rec("event", {"msg": msg})

    # ── callbacks ────────────────────────────────────────────────────────────
    def on_odom(self, msg):
        y = yaw_deg_from_quat(msg.pose.pose.orientation)
        if self.yaw_deg is None:
            self.yaw_unwrapped = 0.0
        else:
            self.yaw_unwrapped += unwrap_deg(self.yaw_deg, y)
        self.yaw_deg = y
        self.yaw_seen = True
        self.rec("odom", {
            "vx": msg.twist.twist.linear.x,
            "wz": msg.twist.twist.angular.z,
            "yaw_deg": round(y, 2),
            # Yaw đã nối liên tục: đây mới là thứ cộng dồn được qua cả cú rẽ.
            "yaw_unwrapped_deg": round(self.yaw_unwrapped, 2),
            "px": msg.pose.pose.position.x,
            "py": msg.pose.pose.position.y,
        })

    def on_ref(self, msg):
        self.rec("lane_ref_cmd", {"v": msg.linear.x, "omega": msg.angular.z})

    def on_cmd_vel(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        # Vi sai bánh theo mô hình cặp trái/cặp phải mà firmware dùng. Nửa
        # khoảng cách bánh 0.0675m lấy từ yahboomcar_description/urdf/MicroROS.urdf
        # (bánh ở y = +-0.0675). Bánh phía trong là bánh bị đòi chạy chậm nhất -
        # nếu nó rơi xuống dưới ngưỡng ma sát tĩnh thì bánh đó đứng im trong khi
        # bánh ngoài vẫn quay, đúng triệu chứng "1 bánh chạy, các bánh kia dừng".
        HALF_W = 0.0675
        self.rec("cmd_vel", {
            "v": v, "omega": w,
            "v_left": v - w * HALF_W,
            "v_right": v + w * HALF_W,
        })

    def on_control_error(self, msg):
        try:
            self.rec("control_error", json.loads(msg.data))
        except Exception:
            self.rec("control_error", {"raw": msg.data[:200]})

    def on_intent(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {"raw": msg.data[:200]}
        self.rec("route_intent_cmd", payload)
        self.log("INTENT ra lệnh: %s" % json.dumps(payload)[:120])

    def on_intent_ack(self, msg):
        try:
            self.rec("route_intent_ack", json.loads(msg.data))
        except Exception:
            self.rec("route_intent_ack", {"raw": msg.data[:200]})

    def on_telemetry(self, topic, msg):
        """Chỉ giữ lane liên quan tới cú rẽ, và chỉ khi đang có turn intent."""
        if not self.last_intent.lower().startswith("turn") and not self.latch_active:
            return
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        objs = payload.get("objects") or []
        keep = [o for o in objs if o.get("label") in KEEP_LABELS]
        if not keep:
            return
        self.rec(topic, {"timestamp_ms": payload.get("timestamp_ms"), "objects": keep})

    def on_lane_state(self, msg):
        try:
            s = json.loads(msg.data)
        except Exception:
            self.rec("lane_state", {"raw": msg.data[:200]})
            return
        self.frames += 1

        # Giữ hình học, chỉ lọc bớt stage thừa.
        trajs = s.get("debug_trajectories")
        if trajs:
            s["debug_trajectories"] = [t for t in trajs if t.get("stage") in KEEP_STAGES]

        # Yaw đo kèm mỗi frame, để phân tích khỏi phải nội suy nhiều nguồn.
        s["_yaw_unwrapped_deg"] = round(self.yaw_unwrapped, 2) if self.yaw_seen else None
        self.rec("lane_state", s)

        self.track_turn(s)

    # ── phát hiện sự kiện rẽ ─────────────────────────────────────────────────
    def track_turn(self, s):
        intent = s.get("route_intent", "?")
        active = bool(s.get("turn_latch_active"))
        progress = float(s.get("turn_latch_progress_mm") or 0.0)
        length = float(s.get("turn_latch_length_mm") or 0.0)
        turned = float(s.get("turn_latch_heading_turned_deg") or 0.0)
        yaw_now = self.yaw_unwrapped if self.yaw_seen else 0.0

        if intent != self.last_intent:
            self.log("intent %s -> %s (decision=%s)" %
                     (self.last_intent, intent, s.get("decision_state")))
            # `route_intent_name` phát ra CHỮ THƯỜNG ("turn_left"), không phải
            # "TURN_LEFT". So sánh sai hoa/thường không báo lỗi, chỉ lặng lẽ để
            # mốc yaw = None và in ra "nan".
            if intent.lower().startswith("turn"):
                # Mốc yaw lúc nhận intent: dùng để đối chiếu góc xoay THẬT của
                # cả cú rẽ với con số latch tự báo.
                self.yaw_at_intent = yaw_now
            self.last_intent = intent

        # Cạnh lên: latch đóng.
        if active and not self.latch_active:
            self.yaw_at_latch = yaw_now
            # Giữ lại chiều dài lúc ĐÓNG: ở frame nhả, `turn_latch_length_mm`
            # đã bị reset về 0 nên tính %% tiêu thụ ngay tại đó luôn ra 0.
            self.latch_len_at_close = length
            self.latch_frames = 0
            self.latch_progress_max = 0.0
            self.prev_progress = progress
            self.skip_events = 0
            self.log("LATCH ĐÓNG kind=%s obs_span=%.1f ext_span=%.1f len=%.0fmm "
                     "ext=%.0fmm deadline=%.1fs" % (
                         s.get("trajectory_kind"),
                         s.get("turn_latch_observed_span_deg") or 0.0,
                         s.get("turn_latch_extended_span_deg") or 0.0,
                         length,
                         s.get("turn_latch_extension_mm") or 0.0,
                         s.get("turn_latch_deadline_s") or 0.0))

        if active:
            self.latch_frames += 1
            self.latch_progress_max = max(self.latch_progress_max, progress)
            # Nhảy tiến bất thường = fix "skip-to-runout" vừa kích hoạt. Ở 14FPS
            # và tốc độ sa bàn, một frame đi được cỡ vài chục mm; 300mm là nhảy.
            if progress - self.prev_progress > 300.0:
                self.skip_events += 1
                self.log("SKIP-TO-RUNOUT: progress %.0f -> %.0f mm (turned=%.1f deg)" %
                         (self.prev_progress, progress, turned))
            self.prev_progress = progress

        # Cạnh xuống: latch nhả.
        if not active and self.latch_active:
            close_len = self.latch_len_at_close
            consumed = (self.latch_progress_max / close_len * 100.0) if close_len > 0 else 0.0
            measured = (yaw_now - self.yaw_at_latch) if self.yaw_at_latch is not None else float("nan")
            since_intent = (yaw_now - self.yaw_at_intent) if self.yaw_at_intent is not None else float("nan")
            self.log("LATCH NHẢ reason=%s turned=%.1f deg | yaw đo: từ lúc latch %.1f, "
                     "từ lúc có intent %.1f | tiêu thụ %.0f%% path (%.0f/%.0fmm) | "
                     "%d frame, %d lần skip" % (
                         s.get("turn_latch_release_reason"), turned,
                         measured, since_intent, consumed,
                         self.latch_progress_max, close_len,
                         self.latch_frames, self.skip_events))

        self.latch_active = active

        # Stub sau latch: đếm để biết perception có bắt được đường mới không.
        if s.get("normalization_mode") == "post_latch_stub":
            self.stub_frames += 1
            if self.stub_frames == 1:
                self.log("post_latch_stub bắt đầu (xe chạy thẳng theo stub, chưa bám lane)")
        elif self.stub_frames:
            self.log("post_latch_stub kết thúc sau %d frame -> %s" %
                     (self.stub_frames, s.get("normalization_mode")))
            self.stub_frames = 0

        # Nhịp trạng thái 2s để biết pipeline còn sống.
        now = time.time()
        if now - self.last_print > 2.0:
            self.last_print = now
            pts = s.get("active_trajectory_points") or []
            front_y = pts[0][1] if pts else float("nan")
            self.log("intent=%s state=%s kind=%s latch=%s turned=%.0f "
                     "path_front_y=%.0fmm npts=%d turn_lane=%s yaw=%.0f" % (
                         intent, s.get("decision_state"), s.get("trajectory_kind"),
                         "Y" if active else "n", turned, front_y, len(pts),
                         "Y" if s.get("turn_lane_detected") else "n", yaw_now))


def main():
    rclpy.init()
    node = TurnObserver()
    end = time.time() + DURATION_S
    try:
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.log("kết thúc: %s" % json.dumps(node.counts))
    node.f.close()
    print(json.dumps(node.counts))


if __name__ == "__main__":
    main()
