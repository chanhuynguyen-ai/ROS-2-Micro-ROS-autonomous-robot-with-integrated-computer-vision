#!/usr/bin/env python3
"""
Logger cho Hybrid Backstepping + Cascade PD Controller.
Subscribe: /avs/hybrid_controller_state
Output: hybrid_unified_<timestamp>/  raw_state.jsonl + CSV + PNG
"""
import argparse, csv, json, math, signal, sys, time
from collections import OrderedDict
from pathlib import Path

import matplotlib
if "--no-live" in sys.argv:
    matplotlib.use("Agg")
try:
    matplotlib.rcParams["figure.raise_window"] = False
except Exception:
    pass
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
                        qos_profile_sensor_data)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

STATE_TOPIC = "/avs/hybrid_controller_state"

def now(): return time.time()

def f(v, default=math.nan):
    try:
        x = float(v)
        return x if math.isfinite(x) else default
    except Exception:
        return default

def ff(*values, default=math.nan):
    for v in values:
        x = f(v)
        if math.isfinite(x):
            return x
    return default

def bi(v):
    if isinstance(v, bool): return int(v)
    if isinstance(v, str):
        return 0 if v.strip().lower() in {"","0","false","no","none","invalid","lost"} else 1
    return int(bool(v)) if v is not None else 0

def jload(text):
    try:
        d = json.loads(text)
        return d if isinstance(d, dict) else {}
    except Exception:
        return {}

def yaw(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y),
                      1.0 - 2.0*(q.y*q.y + q.z*q.z))

def csv_scalar(v):
    if v is None: return ""
    if isinstance(v, (str, int, float, bool)): return v
    try: return json.dumps(v, ensure_ascii=False, separators=(",",":"))
    except Exception: return str(v)


class HybridLogger(Node):
    def __init__(self, args):
        super().__init__("hybrid_controller_logger")
        self.a = args
        self.t0 = now()
        self.stop = self.window_closed = False
        self.last_log = self.last_plot = self.last_save = 0.0
        self.records, self.raw = [], []
        self.state, self.ce, self.cmd, self.odom, self.scan = {}, {}, {}, {}, {}

        q = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                       reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(String, STATE_TOPIC, self._state_cb, q)
        self.create_subscription(String, args.control_error_topic, self._ce_cb, 20)
        self.create_subscription(Twist, args.cmd_vel_topic, self._cmd_cb, q)
        self.create_subscription(Odometry, args.odom_topic, self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, args.scan_topic, self._scan_cb, qos_profile_sensor_data)

        stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.name = f"hybrid_unified_{stamp}"
        self.out = Path(args.output_dir).expanduser().resolve() / self.name
        self.out.mkdir(parents=True, exist_ok=True)
        self.csv_path  = self.out / f"{self.name}.csv"
        self.png_path  = self.out / f"{self.name}.png"
        self.raw_path  = self.out / "raw_state.jsonl"

        self.fig = self.axes = self.lines = self.status_txt = None
        if not args.no_live:
            plt.ion()
            self.fig, self.axes, self.lines, self.status_txt = self._make_figure()
            self.fig.canvas.mpl_connect("close_event", self._close_cb)
            plt.show(block=False)

        self.get_logger().info(f"Hybrid logger → {self.out}")

    # ---- callbacks ----
    def _state_cb(self, m):
        d = jload(m.data)
        if not d: return
        d["_rx"] = now()
        self.state = d
        self.raw.append({"time_wall_s": now(), "state":
                         {k: v for k, v in d.items() if not k.startswith("_")}})

    def _ce_cb(self, m):   self.ce   = jload(m.data); self.ce["_rx"] = now()
    def _cmd_cb(self, m):  self.cmd  = {"cmd_v": float(m.linear.x), "cmd_omega": float(m.angular.z), "_rx": now()}
    def _odom_cb(self, m): self.odom = {"odom_x": float(m.pose.pose.position.x),
                                         "odom_y": float(m.pose.pose.position.y),
                                         "odom_yaw": yaw(m.pose.pose.orientation),
                                         "odom_v": float(m.twist.twist.linear.x),
                                         "odom_omega": float(m.twist.twist.angular.z), "_rx": now()}
    def _scan_cb(self, m):
        half = math.radians(self.a.front_angle_deg)
        angle = m.angle_min; vals = []
        for r in m.ranges:
            if math.isfinite(r) and m.range_min <= r <= m.range_max and abs(angle) <= half:
                vals.append(float(r))
            angle += m.angle_increment
        self.scan = {"front_min_m": min(vals) if vals else math.nan, "_rx": now()}

    def _age(self, d):
        if not isinstance(d, dict): return math.inf
        t = f(d.get("_rx"))
        return now() - t if math.isfinite(t) else math.inf

    def _close_cb(self, _):
        self.window_closed = self.stop = True

    # ---- build one log row ----
    def _row(self):
        s, ce, cmd, od, sc = self.state, self.ce, self.cmd, self.odom, self.scan

        ex_mm   = ff(s.get("epsilon_x_mm"),    ce.get("epsilon_x_mm"))
        th      = ff(s.get("theta_rad"),        ce.get("theta_rad"))
        v_cmd   = ff(s.get("v_cmd"),            cmd.get("cmd_v"))
        om_cmd  = ff(s.get("omega_cmd"),        cmd.get("cmd_omega"))

        return {
            "t_s":              now() - self.t0,
            "version":          s.get("version", ""),
            # ---- errors ----
            "e_lat_raw_mm":     ex_mm,
            "e_lat_used_mm":    ff(s.get("e_used_mm"), ex_mm),
            "e_lat_f_mm":       ff(s.get("e_f_mm")),
            "theta_raw_rad":    th,
            "theta_used_rad":   ff(s.get("theta_used_rad"), th),
            # ---- bs internals ----
            "theta_virtual_rad": ff(s.get("theta_virtual_rad")),
            "e_bs_rad":          ff(s.get("e_bs_rad")),
            "omega_pd":          ff(s.get("omega_pd")),
            "omega_bs":          ff(s.get("omega_bs")),
            "bs_weight":         ff(s.get("bs_weight")),
            # ---- steering ----
            "omega_feedback":   ff(s.get("omega_feedback")),
            "omega_ff":         ff(s.get("omega_ff")),
            "omega_ref":        ff(s.get("omega_ref")),
            "omega_cmd":        om_cmd,
            "omega_limit":      ff(s.get("omega_limit"), s.get("omega_dynamic_limit")),
            # ---- speed ----
            "v_target":         ff(s.get("v_target"), s.get("v_des")),
            "v_ref":            ff(s.get("v_ref")),
            "v_cmd":            v_cmd,
            "odom_v":           ff(s.get("odom_v"), od.get("odom_v")),
            "odom_omega":       ff(s.get("odom_omega"), od.get("odom_omega")),
            # ---- curve ----
            "curve_severity":   ff(s.get("curve_severity")),
            "curve_zone":       str(s.get("curve_zone", "")),
            "kappa_m":          ff(s.get("kappa_m"), ce.get("curvature_m_inv")),
            # ---- wheel ----
            "v_left_ref":       ff(s.get("v_left_ref")),
            "v_right_ref":      ff(s.get("v_right_ref")),
            "v_left_cmd":       ff(s.get("v_left_cmd")),
            "v_right_cmd":      ff(s.get("v_right_cmd")),
            # ---- misc ----
            "fps_est":          ff(s.get("fps_est"), ce.get("fps")),
            "confidence":       ff(s.get("confidence"), ce.get("confidence")),
            "front_min_m":      ff(s.get("front_min_m"), sc.get("front_min_m")),
            "cmd_published":    bi(s.get("cmd_published")),
            "odom_x":           ff(s.get("odom_x"), od.get("odom_x")),
            "odom_y":           ff(s.get("odom_y"), od.get("odom_y")),
        }

    def log(self):
        self.records.append(self._row())

    # ---- figure ----
    @staticmethod
    def _ax(ax, title, ylabel):
        ax.set_title(title); ax.set_xlabel("t [s]"); ax.set_ylabel(ylabel); ax.grid(True)

    def _make_figure(self):
        fig, aa = plt.subplots(4, 3, figsize=(18, 12))
        fig.subplots_adjust(left=.05, right=.985, top=.94, bottom=.06, hspace=.45, wspace=.28)
        fig.suptitle("AVS Hybrid BS+Cascade PD Logger", fontsize=14)
        try: fig.canvas.manager.set_window_title("Hybrid Controller Logger")
        except Exception: pass

        panels = ["lat","head","steer","spd","ang","bs","whl","curve","misc","path","status"]
        axes = OrderedDict()
        flat = aa.flatten()
        for i, p in enumerate(panels):
            axes[p] = flat[i]
        axes["path"].set_aspect("equal", adjustable="datalim")

        lines = {p: OrderedDict() for p in panels}
        def add(p, key, label):
            l, = axes[p].plot([], [], label=label); lines[p][key] = l

        self._ax(axes["lat"],   "1. Lateral error", "mm")
        for k,l in [("e_lat_raw_mm","raw"),("e_lat_used_mm","used"),("e_lat_f_mm","filtered")]: add("lat",k,l)

        self._ax(axes["head"],  "2. Heading error", "rad")
        for k,l in [("theta_raw_rad","raw"),("theta_used_rad","used"),("theta_virtual_rad","theta_v(BS)")]: add("head",k,l)

        self._ax(axes["steer"], "3. Steering omega", "rad/s")
        for k,l in [("omega_feedback","feedback"),("omega_ff","FF"),("omega_ref","ref"),("omega_cmd","cmd"),("omega_limit","limit")]: add("steer",k,l)

        self._ax(axes["spd"],   "4. Linear speed", "m/s")
        for k,l in [("v_target","target"),("v_ref","ref"),("v_cmd","cmd"),("odom_v","odom")]: add("spd",k,l)

        self._ax(axes["ang"],   "5. Angular speed", "rad/s")
        for k,l in [("omega_cmd","cmd"),("odom_omega","odom")]: add("ang",k,l)

        self._ax(axes["bs"],    "6. Backstepping components", "rad/s")
        for k,l in [("omega_pd","omega_PD"),("omega_bs","omega_BS"),("e_bs_rad","e_bs")]: add("bs",k,l)

        self._ax(axes["whl"],   "7. Wheel refs/cmds", "m/s")
        for k,l in [("v_left_ref","L_ref"),("v_right_ref","R_ref"),("v_left_cmd","L_cmd"),("v_right_cmd","R_cmd")]: add("whl",k,l)

        self._ax(axes["curve"], "8. Curve / kappa", "value")
        for k,l in [("curve_severity","severity"),("kappa_m","kappa"),("fps_est","FPS"),("confidence","conf")]: add("curve",k,l)

        self._ax(axes["misc"],  "9. Safety / flags", "value")
        for k,l in [("front_min_m","front_min"),("cmd_published","cmd_pub")]: add("misc",k,l)

        axes["path"].set_title("10. Odom path"); axes["path"].set_xlabel("x [m]"); axes["path"].set_ylabel("y [m]"); axes["path"].grid(True)
        pl, = axes["path"].plot([], [], "b-", lw=1, label="odom"); lines["path"]["odom"] = pl

        axes["status"].set_title("11. Status"); axes["status"].axis("off")
        st = axes["status"].text(.01, .99, "Waiting…", transform=axes["status"].transAxes,
                                  va="top", ha="left", family="monospace", fontsize=8)

        for p, ax in axes.items():
            if p not in ("status","path") and lines[p]: ax.legend(fontsize=6.5, loc="best")
        axes["path"].legend(fontsize=6.5)
        return fig, axes, lines, st

    def _render(self, rows, fig=None, axes=None, lines=None, status_txt=None):
        if not rows: return
        t = [r["t_s"] for r in rows]
        last = rows[-1]
        
        fig = fig or self.fig
        axes = axes or self.axes
        lines = lines or self.lines
        status_txt = status_txt or self.status_txt

        for p in ["lat","head","steer","spd","ang","bs","whl","curve","misc"]:
            for k, line in lines[p].items():
                line.set_data(t, [f(r.get(k)) for r in rows])
            axes[p].relim(visible_only=True); axes[p].autoscale_view()

        lines["path"]["odom"].set_data([f(r.get("odom_x")) for r in rows],
                                             [f(r.get("odom_y")) for r in rows])
        axes["path"].relim(); axes["path"].autoscale_view()

        text = (
            f"version     : {last.get('version','')}\n"
            f"e_lat used  : {f(last.get('e_lat_used_mm')):8.2f} mm\n"
            f"theta used  : {f(last.get('theta_used_rad')):8.3f} rad\n"
            f"theta_v(BS) : {f(last.get('theta_virtual_rad')):8.3f} rad\n"
            f"e_bs        : {f(last.get('e_bs_rad')):8.3f} rad\n\n"
            f"omega_PD    : {f(last.get('omega_pd')):8.3f} rad/s\n"
            f"omega_BS    : {f(last.get('omega_bs')):8.3f} rad/s\n"
            f"bs_weight   : {f(last.get('bs_weight')):8.2f}\n"
            f"omega_cmd   : {f(last.get('omega_cmd')):8.3f} rad/s\n"
            f"omega_limit : {f(last.get('omega_limit')):8.3f} rad/s\n\n"
            f"v_target    : {f(last.get('v_target')):8.3f} m/s\n"
            f"v_cmd       : {f(last.get('v_cmd')):8.3f} m/s\n"
            f"odom_v      : {f(last.get('odom_v')):8.3f} m/s\n\n"
            f"severity    : {f(last.get('curve_severity')):8.3f}\n"
            f"curve_zone  : {last.get('curve_zone','')}\n"
            f"fps         : {f(last.get('fps_est')):8.1f}\n"
            f"confidence  : {f(last.get('confidence')):8.2f}\n"
            f"front_min   : {f(last.get('front_min_m')):8.3f} m\n"
        )
        status_txt.set_text(text)
        try: fig.canvas.draw_idle()
        except Exception: pass

    def live(self):
        if self.fig is None or self.window_closed: return
        rows = self.records[-300:] if len(self.records) > 300 else self.records
        if rows: self._render(rows)
        try: self.fig.canvas.flush_events()
        except Exception: pass

    # ---- save ----
    def save_csv(self):
        if not self.records: return
        cols = list(self.records[0].keys())
        with self.csv_path.open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=cols); w.writeheader()
            for r in self.records: w.writerow(r)

    def save_raw(self):
        with self.raw_path.open("w", encoding="utf-8") as fh:
            for x in self.raw:
                fh.write(json.dumps(x, ensure_ascii=False) + "\n")

    def save_png(self):
        if not self.records: return
        plt.ioff()
        fig, axes, lines, st = self._make_figure()
        try:
            self._render(self.records, fig=fig, axes=axes, lines=lines, status_txt=st)
            fig.savefig(self.png_path, dpi=self.a.dpi, bbox_inches="tight")
        finally:
            plt.close(fig)
            if not self.a.no_live: plt.ion()

    def autosave(self):
        self.save_csv(); self.save_raw()

    def save_all(self):
        self.autosave(); self.save_png()
        print(f"\n{'='*55}\nHYBRID LOG SAVED\nFolder : {self.out}\n{'='*55}\n")


# ---- main ----
def parser():
    p = argparse.ArgumentParser(description="Hybrid BS+Cascade PD logger")
    p.add_argument("--output-dir", default="/home/bluedstar/SimpleRobot/terminal-run/plot_logger")
    p.add_argument("--log-hz",      type=float, default=20.0)
    p.add_argument("--plot-hz",     type=float, default=4.0)
    p.add_argument("--autosave-s",  type=float, default=10.0)
    p.add_argument("--duration-s",  type=float, default=0.0)
    p.add_argument("--window-s",    type=float, default=120.0)
    p.add_argument("--dpi",         type=int,   default=150)
    p.add_argument("--front-angle-deg", type=float, default=35.0)
    p.add_argument("--no-live",     action="store_true")
    p.add_argument("--control-error-topic", default="/avs/control_error")
    p.add_argument("--cmd-vel-topic",       default="/cmd_vel")
    p.add_argument("--odom-topic",          default="/odom_raw")
    p.add_argument("--scan-topic",          default="/scan")
    return p


def main():
    a = parser().parse_args()
    rclpy.init()
    n = HybridLogger(a)
    signal.signal(signal.SIGINT,  lambda *_: setattr(n, "stop", True))
    signal.signal(signal.SIGTERM, lambda *_: setattr(n, "stop", True))

    lp = 1.0 / max(a.log_hz, 1.0)
    pp = 1.0 / max(a.plot_hz, 0.5)

    try:
        while rclpy.ok() and not n.stop:
            rclpy.spin_once(n, timeout_sec=0.005)
            t = now()
            if t - n.last_log >= lp:  n.log();  n.last_log = t
            if not a.no_live and not n.window_closed and t - n.last_plot >= pp:
                n.live(); n.last_plot = t
            if a.autosave_s > 0 and t - n.last_save >= a.autosave_s:
                n.autosave(); n.last_save = t
            if a.duration_s > 0 and t - n.t0 >= a.duration_s: break
            time.sleep(0.003)
    except KeyboardInterrupt:
        pass
    finally:
        try: n.save_all()
        finally: n.destroy_node()


if __name__ == "__main__":
    main()
