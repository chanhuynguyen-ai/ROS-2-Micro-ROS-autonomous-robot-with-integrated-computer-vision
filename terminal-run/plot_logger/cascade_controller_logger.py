#!/usr/bin/env python3
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
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

CONTROLLERS = OrderedDict([
    ("v1", ("/avs/cascade_controller_v1_state", "/avs/cascade_controller_v1_ref", "Cascade V1 - PP/PD + wheel mixing")),
    ("v2", ("/avs/cascade_controller_v2_state", "/avs/cascade_controller_v2_ref", "Cascade V2 - True cascade PD")),
    ("v3", ("/avs/cascade_controller_v3_state", "/avs/cascade_controller_v3_ref", "Cascade V3 - Adaptive cascade PD")),
    ("v4", ("/avs/cascade_controller_v4_state", "/avs/cascade_controller_v4_ref", "Cascade V4 - High Speed (v2 fork)"))
])

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

def fv(*values, default=None):
    for v in values:
        if v is not None and v != "":
            return v
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
    return math.atan2(
        2.0*(q.w*q.z + q.x*q.y),
        1.0 - 2.0*(q.y*q.y + q.z*q.z)
    )

def csv_scalar(v):
    if v is None: return ""
    if isinstance(v, (str,int,float,bool)): return v
    try: return json.dumps(v, ensure_ascii=False, separators=(",",":"))
    except Exception: return str(v)


class CascadeLogger(Node):
    def __init__(self, args):
        super().__init__("cascade_controller_logger")
        self.a = args
        self.t0 = now()
        self.stop = False
        self.window_closed = False
        self.last_log = self.last_plot = self.last_save = 0.0
        self.records, self.raw = [], []
        self.ce, self.cmd, self.odom, self.scan = {}, {}, {}, {}
        self.states = {k:{} for k in CONTROLLERS}
        self.refs = {k:{} for k in CONTROLLERS}
        self.last_active = ""
        self.last_multi_warn = 0.0

        self.mode_maps = {
            k: OrderedDict() for k in [
                "mode","outer","mix","lidar","curve","avoidance",
                "intent","trajectory","planner"
            ]
        }

        q = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.q = q

        self.create_subscription(String, args.control_error_topic, self.ce_cb, 20)
        self.create_subscription(Twist, args.cmd_vel_topic, self.cmd_cb, q)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, args.scan_topic, self.scan_cb, qos_profile_sensor_data)

        for k,(st,rt,_) in CONTROLLERS.items():
            self.create_subscription(String, st, lambda m,c=k:self.state_cb(c,m), q)
            self.create_subscription(Twist, rt, lambda m,c=k:self.ref_cb(c,m), q)

        stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        self.name = f"cascade_unified_{stamp}"
        self.out = Path(args.output_dir).expanduser().resolve()/self.name
        self.out.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.out/f"{self.name}.csv"
        self.png_path = self.out/f"{self.name}.png"
        self.map_path = self.out/"mode_map.json"
        self.raw_path = self.out/"raw_state.jsonl"

        self.fig = self.axes = self.lines = self.status = None
        if not args.no_live:
            plt.ion()
            self.fig,self.axes,self.lines,self.status = self.make_figure("Waiting for cascade controller...")
            self.fig.canvas.mpl_connect("close_event", self.close_cb)
            plt.show(block=False)
            self.no_focus()

        self.get_logger().info("Unified Cascade V1/V2/V3 logger started")
        self.get_logger().info("AUTO = freshest state stream; logger is subscribe-only")
        self.get_logger().info(f"Output: {self.out}")

    def mark(self,d):
        if isinstance(d,dict): d["_rx"]=now()
        return d

    def age(self,d):
        if not isinstance(d,dict): return math.inf
        t = f(d.get("_rx"))
        return now()-t if math.isfinite(t) else math.inf

    def ce_cb(self,m): self.ce=self.mark(jload(m.data))

    def state_cb(self,c,m):
        d=jload(m.data)
        if not d: return
        d["_rx"],d["_controller"]=now(),c
        self.states[c]=d
        self.raw.append({
            "time_wall_s":now(),"controller":c,
            "state":{k:v for k,v in d.items() if not k.startswith("_")}
        })

    def ref_cb(self,c,m):
        self.refs[c]={"v_ref":float(m.linear.x),"omega_ref":float(m.angular.z),"_rx":now()}

    def cmd_cb(self,m):
        self.cmd={"cmd_v":float(m.linear.x),"cmd_omega":float(m.angular.z),"_rx":now()}

    def odom_cb(self,m):
        self.odom={
            "odom_x":float(m.pose.pose.position.x),
            "odom_y":float(m.pose.pose.position.y),
            "odom_yaw":yaw(m.pose.pose.orientation),
            "odom_v":float(m.twist.twist.linear.x),
            "odom_omega":float(m.twist.twist.angular.z),
            "_rx":now()
        }

    def scan_cb(self,m):
        half=math.radians(self.a.front_angle_deg)
        angle=m.angle_min; vals=[]
        for r in m.ranges:
            if math.isfinite(r) and m.range_min<=r<=m.range_max and abs(angle)<=half:
                vals.append(float(r))
            angle += m.angle_increment
        self.scan={"front_min_m":min(vals) if vals else math.nan,"_rx":now()}

    def active(self):
        if self.a.controller!="auto":
            return self.a.controller
        fresh=[(d.get("_rx",0.0),k) for k,d in self.states.items() if self.age(d)<=self.a.active_timeout_s]
        if fresh:
            fresh.sort(reverse=True)
            self.last_active=fresh[0][1]
            if len(fresh)>1 and now()-self.last_multi_warn>3.0:
                self.get_logger().warn(
                    "Multiple fresh cascade controllers: "+
                    ", ".join(f"{k}:{self.age(self.states[k]):.2f}s" for _,k in fresh)+
                    f". Showing {self.last_active}."
                )
                self.last_multi_warn=now()
        return self.last_active

    def code(self,group,value):
        value="" if value is None else str(value)
        if not value: return 0
        mp=self.mode_maps[group]
        if value not in mp: mp[value]=len(mp)+1
        return mp[value]

    def row(self):
        c=self.active()
        s=self.states.get(c,{}) if c else {}
        r=self.refs.get(c,{}) if c else {}
        ce,cmd,od,sc=self.ce,self.cmd,self.odom,self.scan

        ex=ff(s.get("epsilon_x_mm"),ce.get("epsilon_x_mm"),ce.get("x_mm"))
        eraw=ex/1000.0 if math.isfinite(ex) else ff(ce.get("lateral_error_m"),ce.get("e_lat_m"),ce.get("e_y_m"))
        ef=ff(s.get("e_f_m"),s.get("e_lat_f_m"))
        eu=ff(s.get("e_used_m"),s.get("e_lat_used_m"))
        traw=ff(s.get("theta_rad"),ce.get("theta_rad"),ce.get("heading_error_rad"),ce.get("e_theta_rad"))
        tf=ff(s.get("theta_f_rad"),s.get("e_heading_f_rad"))
        tu=ff(s.get("theta_used_rad"),s.get("e_heading_used_rad"))

        kappa=ff(s.get("kappa_m"),s.get("curvature"),ce.get("curvature_m_inv"),ce.get("kappa"))
        if not math.isfinite(kappa):
            inv=ff(ce.get("curvature_inv_mm"))
            if math.isfinite(inv): kappa=inv*1000.0

        vlr=ff(s.get("v_left_ref"),s.get("v_left_des"))
        vrr=ff(s.get("v_right_ref"),s.get("v_right_des"))
        vlm=ff(s.get("v_left_measured"),s.get("v_left_meas"),s.get("v_left_odom"))
        vrm=ff(s.get("v_right_measured"),s.get("v_right_meas"),s.get("v_right_odom"))

        ov=ff(s.get("odom_v"),od.get("odom_v"))
        ow=ff(s.get("odom_omega"),od.get("odom_omega"))
        cv=ff(cmd.get("cmd_v")); cw=ff(cmd.get("cmd_omega"))

        mode=str(fv(s.get("mode"),default=""))
        outer=str(fv(s.get("outer_mode"),default=""))
        mix=str(fv(s.get("mix_mode"),default=""))
        lidar=str(fv(s.get("lidar_mode"),default=""))
        curve=str(fv(s.get("curve_zone"),default=""))
        avoid=str(fv(s.get("avoidance_mode"),default=""))
        intent=str(fv(s.get("intent_hint"),default=""))
        traj=str(fv(s.get("trajectory_hint"),default=""))
        planner=str(fv(s.get("planner_status_hint"),default=""))

        d={
            "time_wall_s":now(),"t_s":now()-self.t0,
            "active_controller":c,"controller_node":s.get("node",""),
            "controller_version":s.get("version",""),
            "control_error_age_s":self.age(ce),"state_age_s":self.age(s),"ref_age_s":self.age(r),
            "cmd_vel_age_s":self.age(cmd),"odom_age_local_s":self.age(od),"scan_age_s":self.age(sc),
            "controller_error_age_s":ff(s.get("error_age_s")),"controller_odom_age_s":ff(s.get("odom_age_s")),

            "epsilon_x_mm":ex,
            "e_lat_raw_m":eraw,"e_lat_raw_mm":eraw*1000 if math.isfinite(eraw) else math.nan,
            "e_lat_filtered_m":ef,"e_lat_filtered_mm":ef*1000 if math.isfinite(ef) else math.nan,
            "e_lat_used_m":eu,"e_lat_used_mm":eu*1000 if math.isfinite(eu) else math.nan,
            "theta_raw_rad":traw,"theta_filtered_rad":tf,"theta_used_rad":tu,
            "de_lat_mps":ff(s.get("de_f")),"de_heading_rps":ff(s.get("dtheta_f")),

            "p_lat":ff(s.get("p_lat")),"d_lat":ff(s.get("d_lat")),
            "p_heading":ff(s.get("p_heading")),"d_heading":ff(s.get("d_heading")),
            "outer_kp_lat":ff(s.get("outer_kp_lat")),"outer_kd_lat":ff(s.get("outer_kd_lat")),
            "outer_kp_heading":ff(s.get("outer_kp_heading")),"outer_kd_heading":ff(s.get("outer_kd_heading")),
            "outer_gain_multiplier":ff(s.get("outer_gain_multiplier"),s.get("gain_multiplier")),

            "curvature_from_error":ff(s.get("curvature_from_error")),
            "slow_factor":ff(s.get("slow_factor")),
            "k_pp_used":ff(s.get("k_pp_used")),"k_lat_used":ff(s.get("k_lat_used")),"k_theta_used":ff(s.get("k_theta_used")),
            "curve_confirmed":bi(s.get("curve_confirmed")),"curve_count":ff(s.get("curve_count")),
            "curve_sign":ff(s.get("curve_sign")),"center_zone":bi(s.get("center_zone")),
            "near_zone":bi(s.get("near_zone")),"large_error":bi(s.get("large_error")),"lane_change":bi(s.get("lane_change")),

            "curve_severity":ff(s.get("curve_severity")),"omega_ff":ff(s.get("omega_ff")),"omega_fb":ff(s.get("omega_fb")),
            "curve_steer_gain":ff(s.get("curve_steer_gain")),"omega_dynamic_limit":ff(s.get("omega_dynamic_limit"),s.get("omega_dynamic_max")),
            "brake_scale":ff(s.get("brake_scale")),

            "v_target":ff(s.get("v_target"),s.get("v_des")),"omega_target":ff(s.get("omega_target"),s.get("omega_des")),
            "omega_raw":ff(s.get("omega_raw")),"omega_limit":ff(s.get("omega_limit")),
            "v_ref":ff(s.get("v_ref"),r.get("v_ref")),"omega_ref":ff(s.get("omega_ref"),r.get("omega_ref")),
            "v_cmd_internal":ff(s.get("v_cmd")),"omega_cmd_internal":ff(s.get("omega_cmd")),
            "cmd_v":cv,"cmd_omega":cw,"odom_v":ov,"odom_omega":ow,
            "odom_v_filtered":ff(s.get("odom_v_filtered")),"odom_omega_filtered":ff(s.get("odom_omega_filtered")),
            "linear_tracking_error":cv-ov if math.isfinite(cv) and math.isfinite(ov) else math.nan,
            "angular_tracking_error":cw-ow if math.isfinite(cw) and math.isfinite(ow) else math.nan,

            "v_left_ref":vlr,"v_right_ref":vrr,"v_left_measured":vlm,"v_right_measured":vrm,
            "v_left_cmd":ff(s.get("v_left_cmd")),"v_right_cmd":ff(s.get("v_right_cmd")),
            "left_wheel_error":ff(s.get("left_wheel_error")),"right_wheel_error":ff(s.get("right_wheel_error")),
            "d_left_wheel_error":ff(s.get("d_left_wheel_error")),"d_right_wheel_error":ff(s.get("d_right_wheel_error")),
            "left_pd_correction":ff(s.get("left_pd_correction")),"right_pd_correction":ff(s.get("right_pd_correction")),

            "fps":ff(s.get("fps_est"),ce.get("fps"),ce.get("fps_est"),ce.get("vision_fps")),
            "confidence":ff(s.get("confidence"),ce.get("confidence")),
            "lookahead_m":ff(s.get("lookahead_m"),ce.get("lookahead_m")),"curvature":kappa,
            "speed_factor":ff(s.get("speed_factor")),

            "jump_hold":bi(s.get("jump_hold")),"soft_replan_active":bi(s.get("soft_replan_active")),
            "avoidance_active":bi(s.get("avoidance_active")),"avoidance_reverse":bi(s.get("avoidance_reverse")),
            "avoidance_offset_m":ff(s.get("avoidance_offset_m")),
            "avoidance_target_offset_m":ff(s.get("avoidance_target_offset_m")),

            "front_min_m":ff(s.get("front_min_m"),sc.get("front_min_m")),
            "lidar_ratio":ff(s.get("lidar_ratio")),"lidar_stop":bi(s.get("lidar_stop")),
            "raw_valid":bi(s.get("raw_valid",ce.get("valid",False))),"cmd_published":bi(s.get("cmd_published")),
            "emergency_stop":bi(s.get("emergency_stop")),"cmd_vel_conflict":bi(s.get("cmd_vel_conflict")),

            "odom_x":ff(s.get("odom_x"),od.get("odom_x")),"odom_y":ff(s.get("odom_y"),od.get("odom_y")),
            "odom_yaw":ff(s.get("odom_yaw"),od.get("odom_yaw")),

            "lane_state":str(fv(s.get("lane_state"),ce.get("lane_state"),default="")),
            "controller_mode":mode,"outer_mode":outer,"mix_mode":mix,"lidar_mode":lidar,
            "curve_zone":curve,"avoidance_mode":avoid,"intent_hint":intent,
            "trajectory_hint":traj,"planner_status_hint":planner,

            "controller_mode_code":self.code("mode",mode),"outer_mode_code":self.code("outer",outer),
            "mix_mode_code":self.code("mix",mix),"lidar_mode_code":self.code("lidar",lidar),
            "curve_zone_code":self.code("curve",curve),"avoidance_mode_code":self.code("avoidance",avoid),
            "intent_hint_code":self.code("intent",intent),"trajectory_hint_code":self.code("trajectory",traj),
            "planner_status_hint_code":self.code("planner",planner),
        }

        # Không làm mất bất kỳ field nào controller publish.
        for k,v in s.items():
            if not k.startswith("_"):
                d[f"state__{k}"]=csv_scalar(v)
        return d

    def log(self): self.records.append(self.row())

    def window(self):
        if not self.records or self.a.window_s<=0: return self.records
        cutoff=self.records[-1]["t_s"]-self.a.window_s
        return [r for r in self.records if r["t_s"]>=cutoff]

    @staticmethod
    def col(rows,key): return [f(r.get(key)) for r in rows]

    @staticmethod
    def axsetup(ax,title,ylabel):
        ax.set_title(title); ax.set_xlabel("time [s]"); ax.set_ylabel(ylabel); ax.grid(True)

    def make_figure(self,title):
        fig,aa=plt.subplots(5,3,figsize=(19,14))
        fig.subplots_adjust(left=.05,right=.985,top=.945,bottom=.055,hspace=.42,wspace=.25)
        fig.suptitle(title,fontsize=15)
        try: fig.canvas.manager.set_window_title("AVS Cascade Unified Logger")
        except Exception: pass
        names=["lat","head","deriv","lin","ang","wheels","werr","inner","percep","track","speca","specb","safe","path","status"]
        axes=OrderedDict(zip(names,aa.flatten())); lines={n:OrderedDict() for n in names}

        def add(p,key,label):
            line,=axes[p].plot([],[],label=label); lines[p][key]=line

        self.axsetup(axes["lat"],"1. Lateral error","mm")
        for k,l in [("e_lat_raw_mm","raw"),("e_lat_filtered_mm","filtered"),("e_lat_used_mm","used")]: add("lat",k,l)

        self.axsetup(axes["head"],"2. Heading error","rad")
        for k,l in [("theta_raw_rad","raw"),("theta_filtered_rad","filtered"),("theta_used_rad","used")]: add("head",k,l)

        self.axsetup(axes["deriv"],"3. Error derivatives","value/s")
        add("deriv","de_lat_mps","de_lat"); add("deriv","de_heading_rps","dtheta")

        self.axsetup(axes["lin"],"4. Linear velocity","m/s")
        for k,l in [("v_target","target"),("v_ref","reference"),("v_cmd_internal","controller"),("cmd_v","/cmd_vel"),("odom_v","odom")]: add("lin",k,l)

        self.axsetup(axes["ang"],"5. Angular velocity","rad/s")
        for k,l in [("omega_raw","raw"),("omega_target","target"),("omega_ref","reference"),("omega_cmd_internal","controller"),("cmd_omega","/cmd_vel"),("odom_omega","odom")]: add("ang",k,l)

        self.axsetup(axes["wheels"],"6. Left/right wheel groups","m/s")
        for k,l in [("v_left_ref","L ref"),("v_right_ref","R ref"),("v_left_measured","L meas"),("v_right_measured","R meas"),("v_left_cmd","L cmd"),("v_right_cmd","R cmd")]: add("wheels",k,l)

        self.axsetup(axes["werr"],"7. Inner wheel errors","m/s")
        for k,l in [("left_wheel_error","L err"),("right_wheel_error","R err"),("d_left_wheel_error","dL/dt"),("d_right_wheel_error","dR/dt")]: add("werr",k,l)

        self.axsetup(axes["inner"],"8. Inner correction / wheel cmd","m/s")
        for k,l in [("left_pd_correction","L corr"),("right_pd_correction","R corr"),("v_left_cmd","L cmd"),("v_right_cmd","R cmd")]: add("inner",k,l)

        self.axsetup(axes["percep"],"9. Perception / geometry","value")
        for k,l in [("fps","FPS"),("confidence","confidence"),("lookahead_m","lookahead"),("curvature","curvature"),("speed_factor","speed factor")]: add("percep",k,l)

        self.axsetup(axes["track"],"10. Command tracking error","error")
        add("track","linear_tracking_error","linear cmd-odom"); add("track","angular_tracking_error","angular cmd-odom")

        self.axsetup(axes["speca"],"11. Controller-specific A","value")
        for k,l in [("p_lat","P lat"),("d_lat","D lat"),("p_heading","P heading"),("d_heading","D heading"),
                    ("curvature_from_error","PP curve err"),("k_pp_used","k_pp"),("k_lat_used","k_lat"),("k_theta_used","k_theta")]: add("speca",k,l)

        self.axsetup(axes["specb"],"12. Controller-specific B","value")
        for k,l in [("slow_factor","slow factor"),("outer_gain_multiplier","gain mul"),("curve_severity","severity"),
                    ("omega_ff","omega FF"),("omega_fb","omega FB"),("curve_steer_gain","steer gain"),
                    ("omega_limit","omega limit"),("avoidance_offset_m","avoid offset"),("avoidance_target_offset_m","avoid target")]: add("specb",k,l)

        self.axsetup(axes["safe"],"13. Safety / modes","value/code")
        for k,l in [("front_min_m","front min"),("lidar_ratio","lidar ratio"),("raw_valid","valid"),("cmd_published","cmd pub"),
                    ("emergency_stop","e-stop"),("cmd_vel_conflict","conflict"),("controller_mode_code","mode"),
                    ("outer_mode_code","outer"),("curve_zone_code","curve zone"),("avoidance_mode_code","avoid")]: add("safe",k,l)

        axes["path"].set_title("14. Odom path"); axes["path"].set_xlabel("x [m]"); axes["path"].set_ylabel("y [m]"); axes["path"].grid(True)
        pl,=axes["path"].plot([],[],label="odom path"); lines["path"]["odom_path"]=pl
        axes["path"].set_aspect("equal",adjustable="datalim")

        axes["status"].set_title("15. Current status"); axes["status"].axis("off")
        status=axes["status"].text(.01,.99,"Waiting...",transform=axes["status"].transAxes,va="top",ha="left",family="monospace",fontsize=7.7)

        for n,ax in axes.items():
            if n!="status" and lines[n]: ax.legend(fontsize=6.5,loc="best")
        return fig,axes,lines,status

    @staticmethod
    def visible(lines,panel,keys):
        keys=set(keys)
        for k,line in lines[panel].items(): line.set_visible(k in keys)

    def specific(self,c,axes,lines):
        if c=="v1":
            axes["werr"].set_title("7. V1: no closed-loop wheel-speed PD"); self.visible(lines,"werr",[])
            axes["inner"].set_title("8. V1 wheel mixer command"); self.visible(lines,"inner",["v_left_cmd","v_right_cmd"])
            axes["speca"].set_title("11. V1 PP/PD scheduler gains")
            self.visible(lines,"speca",["curvature_from_error","k_pp_used","k_lat_used","k_theta_used"])
            axes["specb"].set_title("12. V1 slowdown / obstacle avoidance")
            self.visible(lines,"specb",["slow_factor","omega_limit","avoidance_offset_m","avoidance_target_offset_m"])
        elif c=="v2":
            axes["werr"].set_title("7. V2 inner wheel-speed PD error"); self.visible(lines,"werr",lines["werr"].keys())
            axes["inner"].set_title("8. V2 inner wheel-speed PD correction"); self.visible(lines,"inner",lines["inner"].keys())
            axes["speca"].set_title("11. V2 true outer PD components")
            self.visible(lines,"speca",["p_lat","d_lat","p_heading","d_heading"])
            axes["specb"].set_title("12. V2 profile / gain / omega limit")
            self.visible(lines,"specb",["outer_gain_multiplier","omega_limit"])
        elif c=="v3":
            axes["werr"].set_title("7. V3 inner wheel-speed PD error"); self.visible(lines,"werr",lines["werr"].keys())
            axes["inner"].set_title("8. V3 inner wheel-speed PD correction"); self.visible(lines,"inner",lines["inner"].keys())
            axes["speca"].set_title("11. V3 outer PD components")
            self.visible(lines,"speca",["p_lat","d_lat","p_heading","d_heading"])
            axes["specb"].set_title("12. V3 adaptive curve / steering")
            self.visible(lines,"specb",["curve_severity","omega_ff","omega_fb","curve_steer_gain","omega_limit"])
        else:
            axes["werr"].set_title("7. Inner wheel errors"); self.visible(lines,"werr",lines["werr"].keys())
            self.visible(lines,"inner",lines["inner"].keys()); self.visible(lines,"speca",lines["speca"].keys()); self.visible(lines,"specb",lines["specb"].keys())

        for p in ["werr","inner","speca","specb"]:
            old=axes[p].get_legend()
            if old: old.remove()
            vis=[x for x in lines[p].values() if x.get_visible()]
            if vis: axes[p].legend(fontsize=6.5,loc="best")

    def render(self,fig,axes,lines,status,rows):
        if not rows: return
        last=rows[-1]; c=last.get("active_controller","")
        self.specific(c,axes,lines)
        label=CONTROLLERS.get(c,("","", "Cascade controller"))[2]
        ver=last.get("controller_version","")
        fig.suptitle(f"AVS {label}"+(f" | {ver}" if ver else ""),fontsize=15)

        t=self.col(rows,"t_s")
        for p in ["lat","head","deriv","lin","ang","wheels","werr","inner","percep","track","speca","specb","safe"]:
            for k,line in lines[p].items(): line.set_data(t,self.col(rows,k))
            axes[p].relim(visible_only=True); axes[p].autoscale_view()

        lines["path"]["odom_path"].set_data(self.col(rows,"odom_x"),self.col(rows,"odom_y"))
        axes["path"].relim(); axes["path"].autoscale_view()

        axes["safe"].set_title(
            f"13. Safety / modes | mode={last.get('controller_mode','')} | outer={last.get('outer_mode','')} | mix={last.get('mix_mode','')}"
        )

        fresh=[k for k,d in self.states.items() if self.age(d)<=self.a.active_timeout_s]
        warn="  !!! MULTIPLE FRESH !!!" if len(fresh)>1 else ""
        text=(
            f"active      : {c}{warn}\n"
            f"node        : {last.get('controller_node','')}\n"
            f"version     : {ver}\n"
            f"state age   : {f(last.get('state_age_s')):7.3f} s\n\n"
            f"mode        : {last.get('controller_mode','')}\n"
            f"outer       : {last.get('outer_mode','')}\n"
            f"mix         : {last.get('mix_mode','')}\n"
            f"curve zone  : {last.get('curve_zone','')}\n"
            f"avoidance   : {last.get('avoidance_mode','')}\n"
            f"intent      : {last.get('intent_hint','')}\n"
            f"trajectory  : {last.get('trajectory_hint','')}\n"
            f"planner     : {last.get('planner_status_hint','')}\n"
            f"lane        : {last.get('lane_state','')}\n"
            f"valid/cmd   : {last.get('raw_valid',0)} / {last.get('cmd_published',0)}\n\n"
            f"e_lat       : {f(last.get('e_lat_used_mm')):8.2f} mm\n"
            f"theta       : {f(last.get('theta_used_rad')):8.3f} rad\n"
            f"de/dtheta   : {f(last.get('de_lat_mps')):7.3f} / {f(last.get('de_heading_rps')):7.3f}\n\n"
            f"P lat/th    : {f(last.get('p_lat')):7.3f} / {f(last.get('p_heading')):7.3f}\n"
            f"D lat/th    : {f(last.get('d_lat')):7.3f} / {f(last.get('d_heading')):7.3f}\n\n"
            f"v tgt/ref   : {f(last.get('v_target')):7.3f} / {f(last.get('v_ref')):7.3f}\n"
            f"v cmd/odom  : {f(last.get('cmd_v')):7.3f} / {f(last.get('odom_v')):7.3f}\n"
            f"w tgt/ref   : {f(last.get('omega_target')):7.3f} / {f(last.get('omega_ref')):7.3f}\n"
            f"w cmd/odom  : {f(last.get('cmd_omega')):7.3f} / {f(last.get('odom_omega')):7.3f}\n\n"
            f"L ref/meas  : {f(last.get('v_left_ref')):7.3f} / {f(last.get('v_left_measured')):7.3f}\n"
            f"R ref/meas  : {f(last.get('v_right_ref')):7.3f} / {f(last.get('v_right_measured')):7.3f}\n"
            f"L/R corr    : {f(last.get('left_pd_correction')):7.3f} / {f(last.get('right_pd_correction')):7.3f}\n\n"
            f"fps/conf    : {f(last.get('fps')):7.2f} / {f(last.get('confidence')):7.2f}\n"
            f"kappa/sev   : {f(last.get('curvature')):7.3f} / {f(last.get('curve_severity')):7.3f}\n"
            f"omega limit : {f(last.get('omega_limit')):7.3f}\n"
        )
        status.set_text(text)
        try: fig.canvas.draw_idle()
        except Exception: pass

    def live(self):
        if self.fig is None or self.window_closed: return
        rows=self.window()
        if not rows: return
        self.render(self.fig,self.axes,self.lines,self.status,rows)
        try: self.fig.canvas.flush_events()
        except Exception: pass

    def no_focus(self):
        try:
            w=getattr(self.fig.canvas.manager,"window",None)
            if w is None: return
            try:
                from matplotlib.backends.qt_compat import QtCore
                try: w.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint,False)
                except Exception: pass
                try: w.setAttribute(QtCore.Qt.WA_ShowWithoutActivating,True)
                except Exception: pass
            except Exception: pass
        except Exception: pass

    def close_cb(self,_):
        self.window_closed=True
        self.stop=True

    def save_csv(self):
        if not self.records: return
        preferred=[
            "time_wall_s","t_s","active_controller","controller_node","controller_version",
            "controller_mode","outer_mode","mix_mode","curve_zone","avoidance_mode","lane_state",
            "intent_hint","trajectory_hint","planner_status_hint","raw_valid","cmd_published",
            "epsilon_x_mm","e_lat_raw_mm","e_lat_filtered_mm","e_lat_used_mm",
            "theta_raw_rad","theta_filtered_rad","theta_used_rad","de_lat_mps","de_heading_rps",
            "p_lat","d_lat","p_heading","d_heading","outer_kp_lat","outer_kd_lat","outer_kp_heading","outer_kd_heading",
            "curvature_from_error","k_pp_used","k_lat_used","k_theta_used","slow_factor","outer_gain_multiplier",
            "curve_severity","omega_ff","omega_fb","curve_steer_gain",
            "v_target","v_ref","v_cmd_internal","cmd_v","odom_v",
            "omega_raw","omega_target","omega_limit","omega_ref","omega_cmd_internal","cmd_omega","odom_omega",
            "linear_tracking_error","angular_tracking_error",
            "v_left_ref","v_left_measured","v_left_cmd","v_right_ref","v_right_measured","v_right_cmd",
            "left_wheel_error","right_wheel_error","d_left_wheel_error","d_right_wheel_error","left_pd_correction","right_pd_correction",
            "fps","confidence","lookahead_m","curvature","speed_factor",
            "curve_confirmed","center_zone","near_zone","large_error","lane_change",
            "jump_hold","soft_replan_active","avoidance_active","avoidance_reverse","avoidance_offset_m","avoidance_target_offset_m",
            "front_min_m","lidar_ratio","lidar_stop","emergency_stop","cmd_vel_conflict",
            "odom_x","odom_y","odom_yaw","control_error_age_s","state_age_s","ref_age_s","cmd_vel_age_s",
            "odom_age_local_s","scan_age_s","controller_error_age_s","controller_odom_age_s"
        ]
        cols=[]; seen=set()
        for k in preferred:
            if k not in seen: cols.append(k); seen.add(k)
        for r in self.records:
            for k in r:
                if k not in seen: cols.append(k); seen.add(k)
        with self.csv_path.open("w",newline="",encoding="utf-8") as fh:
            w=csv.DictWriter(fh,fieldnames=cols); w.writeheader()
            for r in self.records: w.writerow(r)

    def save_maps(self):
        obj={g:{str(code):name for name,code in mp.items()} for g,mp in self.mode_maps.items()}
        self.map_path.write_text(json.dumps(obj,indent=2,ensure_ascii=False),encoding="utf-8")

    def save_raw(self):
        with self.raw_path.open("w",encoding="utf-8") as fh:
            for x in self.raw:
                fh.write(json.dumps(x,ensure_ascii=False)+"\n")

    def save_png(self):
        if not self.records: return
        interactive=plt.isinteractive()
        plt.ioff()

        fig,axes,lines,status=self.make_figure(
            "Final Cascade Dashboard"
        )

        try:
            self.render(
                fig,
                axes,
                lines,
                status,
                self.records
            )

            fig.savefig(
                self.png_path,
                dpi=self.a.dpi,
                bbox_inches="tight"
            )

        finally:
            plt.close(fig)

            if interactive and not self.a.no_live:
                plt.ion()

    def autosave(self):
        self.save_csv()
        self.save_maps()
        self.save_raw()

    def save_all(self):
        self.autosave()
        self.save_png()

        print("\n"+"="*60)
        print("CASCADE UNIFIED LOG SAVED")
        print(f"Folder    : {self.out}")
        print(f"CSV       : {self.csv_path}")
        print(f"PNG       : {self.png_path}")
        print(f"Mode map  : {self.map_path}")
        print(f"Raw state : {self.raw_path}")
        print("="*60+"\n")


def parser():
    p=argparse.ArgumentParser(
        description=(
            "Unified realtime logger for "
            "cascade_controller_v1/v2/v3"
        )
    )

    p.add_argument(
        "--output-dir",
        default=(
            "/home/bluedstar/SimpleRobot/"
            "terminal-run/plot_logger"
        )
    )

    p.add_argument(
        "--controller",
        choices=["auto","v1","v2","v3"],
        default="auto"
    )

    p.add_argument(
        "--active-timeout-s",
        type=float,
        default=1.2
    )

    p.add_argument(
        "--window-s",
        type=float,
        default=120.0
    )

    p.add_argument(
        "--log-hz",
        type=float,
        default=20.0
    )

    p.add_argument(
        "--plot-hz",
        type=float,
        default=4.0
    )

    p.add_argument(
        "--autosave-s",
        type=float,
        default=10.0
    )

    p.add_argument(
        "--duration-s",
        type=float,
        default=0.0
    )

    p.add_argument(
        "--dpi",
        type=int,
        default=160
    )

    p.add_argument(
        "--front-angle-deg",
        type=float,
        default=35.0
    )

    p.add_argument(
        "--no-live",
        action="store_true"
    )

    p.add_argument(
        "--control-error-topic",
        default="/avs/control_error"
    )

    p.add_argument(
        "--cmd-vel-topic",
        default="/cmd_vel"
    )

    p.add_argument(
        "--odom-topic",
        default="/odom_raw"
    )

    p.add_argument(
        "--scan-topic",
        default="/scan"
    )

    return p


def main():
    a=parser().parse_args()

    rclpy.init()

    n=CascadeLogger(a)

    def stop(*_):
        n.stop=True

    signal.signal(
        signal.SIGINT,
        stop
    )

    signal.signal(
        signal.SIGTERM,
        stop
    )

    lp=1.0/max(a.log_hz,1.0)
    pp=1.0/max(a.plot_hz,.5)

    try:
        while rclpy.ok() and not n.stop:

            rclpy.spin_once(
                n,
                timeout_sec=.005
            )

            t=now()

            if t-n.last_log>=lp:
                n.log()
                n.last_log=t

            if (
                not a.no_live
                and
                not n.window_closed
                and
                t-n.last_plot>=pp
            ):
                n.live()
                n.last_plot=t

            if (
                a.autosave_s>0
                and
                t-n.last_save>=a.autosave_s
            ):
                n.autosave()
                n.last_save=t

            if (
                a.duration_s>0
                and
                t-n.t0>=a.duration_s
            ):
                break

            if (
                not a.no_live
                and
                not n.window_closed
                and
                n.fig is not None
            ):
                try:
                    n.fig.canvas.flush_events()
                except Exception:
                    pass

            time.sleep(.003)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            n.save_all()

        finally:
            n.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()

            try:
                plt.close("all")
            except Exception:
                pass


if __name__=="__main__":
    main()
