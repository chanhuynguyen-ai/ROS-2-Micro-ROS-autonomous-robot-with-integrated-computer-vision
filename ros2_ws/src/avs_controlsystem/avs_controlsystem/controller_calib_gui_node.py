
#!/usr/bin/env python3



import os

import csv

import math

import time

import signal

from datetime import datetime



import tkinter as tk

from tkinter import ttk, messagebox



import rclpy

from rclpy.node import Node



from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from std_msgs.msg import Float32MultiArray





def clamp(value, low, high):

    return max(low, min(high, value))





def wrap_pi(angle):

    return math.atan2(math.sin(angle), math.cos(angle))





def yaw_from_quaternion(q):

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)

    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    return math.atan2(siny_cosp, cosy_cosp)





class ControllerCalibGuiNode(Node):

    def __init__(self):

        super().__init__('controller_calib_gui_node')



        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('odom_topic', '/odom_raw')

        self.declare_parameter('debug_topic', '/controller_calib_debug')

        self.declare_parameter('save_dir', '/root/yahboomcar_ws/demo_reports/controller_calibration')

        self.declare_parameter('control_rate_hz', 20.0)



        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.odom_topic = str(self.get_parameter('odom_topic').value)

        self.debug_topic = str(self.get_parameter('debug_topic').value)

        self.save_dir = str(self.get_parameter('save_dir').value)

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)



        os.makedirs(self.save_dir, exist_ok=True)



        self.summary_csv = os.path.join(self.save_dir, 'controller_calibration_summary.csv')

        self.params_csv = os.path.join(self.save_dir, 'controller_parameter_log.csv')



        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.debug_pub = self.create_publisher(Float32MultiArray, self.debug_topic, 10)



        self.odom_sub = self.create_subscription(

            Odometry,

            self.odom_topic,

            self.odom_callback,

            20

        )



        self.odom_ok = False

        self.x = 0.0

        self.y = 0.0

        self.yaw = 0.0



        self.active = False

        self.test_type = 'none'

        self.controller_type = 'PID'



        self.start_x = 0.0

        self.start_y = 0.0

        self.start_yaw = 0.0



        self.target_x = 0.0

        self.target_y = 0.0

        self.target_yaw = 0.0



        self.run_start_time = 0.0

        self.max_duration_s = 10.0



        self.integral_dist = 0.0

        self.integral_yaw = 0.0

        self.prev_dist_error = 0.0

        self.prev_yaw_error = 0.0

        self.prev_theta_control = 0.0

        self.prev_time = time.time()



        self.run_records = []

        self.last_run_dir = ''

        self.last_status = 'No run yet'



        signal.signal(signal.SIGINT, self.signal_stop_handler)

        signal.signal(signal.SIGTERM, self.signal_stop_handler)



        self.root = tk.Tk()

        self.root.title('SimpleRobot Controller Calibration')

        self.root.geometry('1180x820')

        self.root.protocol('WM_DELETE_WINDOW', self.on_close)



        self.build_gui()

        self.ensure_summary_header()

        self.ensure_params_header()



        self.get_logger().info('Controller calibration GUI started')

        self.get_logger().info(f'Publish cmd_vel: {self.cmd_vel_topic}')

        self.get_logger().info(f'Subscribe odom:  {self.odom_topic}')

        self.get_logger().info(f'Save dir:        {self.save_dir}')



    def build_gui(self):

        style = ttk.Style()

        style.theme_use('clam')



        style.configure('Title.TLabel', font=('Arial', 20, 'bold'))

        style.configure('Header.TLabel', font=('Arial', 12, 'bold'))

        style.configure('Big.TButton', font=('Arial', 12, 'bold'), padding=7)

        style.configure('Stop.TButton', font=('Arial', 14, 'bold'), padding=8)

        style.configure('Status.TLabel', font=('Arial', 11))



        main = ttk.Frame(self.root, padding=12)

        main.pack(fill='both', expand=True)



        title = ttk.Label(main, text='SimpleRobot Controller Calibration GUI', style='Title.TLabel')

        title.pack(anchor='center', pady=(0, 10))



        top = ttk.Frame(main)

        top.pack(fill='x')



        mode_box = ttk.LabelFrame(top, text='Controller / Test Mode', padding=10)

        mode_box.pack(side='left', fill='both', expand=True, padx=(0, 8))



        self.controller_var = tk.StringVar(value='PID')

        self.test_type_var = tk.StringVar(value='distance')



        ttk.Label(mode_box, text='Controller').grid(row=0, column=0, sticky='w', pady=4)

        ttk.Combobox(

            mode_box,

            textvariable=self.controller_var,

            values=['PID', 'PD_BACKSTEPPING'],

            state='readonly',

            width=20

        ).grid(row=0, column=1, sticky='w', pady=4)



        ttk.Label(mode_box, text='Test type').grid(row=1, column=0, sticky='w', pady=4)

        ttk.Combobox(

            mode_box,

            textvariable=self.test_type_var,

            values=['distance', 'rotation', 'waypoint'],

            state='readonly',

            width=20

        ).grid(row=1, column=1, sticky='w', pady=4)



        self.max_duration_var = tk.StringVar(value='10.0')

        self.dist_tol_cm_var = tk.StringVar(value='3.0')

        self.angle_tol_deg_var = tk.StringVar(value='3.0')



        ttk.Label(mode_box, text='Max duration, s').grid(row=2, column=0, sticky='w', pady=4)

        ttk.Entry(mode_box, textvariable=self.max_duration_var, width=12).grid(row=2, column=1, sticky='w', pady=4)



        ttk.Label(mode_box, text='Distance tolerance, cm').grid(row=3, column=0, sticky='w', pady=4)

        ttk.Entry(mode_box, textvariable=self.dist_tol_cm_var, width=12).grid(row=3, column=1, sticky='w', pady=4)



        ttk.Label(mode_box, text='Angle tolerance, deg').grid(row=4, column=0, sticky='w', pady=4)

        ttk.Entry(mode_box, textvariable=self.angle_tol_deg_var, width=12).grid(row=4, column=1, sticky='w', pady=4)



        target_box = ttk.LabelFrame(top, text='Target Input', padding=10)

        target_box.pack(side='right', fill='both', expand=True, padx=(8, 0))



        self.target_distance_cm_var = tk.StringVar(value='60.0')

        self.target_angle_deg_var = tk.StringVar(value='90.0')

        self.target_x_cm_var = tk.StringVar(value='60.0')

        self.target_y_cm_var = tk.StringVar(value='0.0')

        self.target_theta_deg_var = tk.StringVar(value='0.0')



        ttk.Label(target_box, text='Distance target, cm').grid(row=0, column=0, sticky='w', pady=4)

        ttk.Entry(target_box, textvariable=self.target_distance_cm_var, width=12).grid(row=0, column=1, sticky='w', pady=4)



        ttk.Label(target_box, text='Rotation target, deg').grid(row=1, column=0, sticky='w', pady=4)

        ttk.Entry(target_box, textvariable=self.target_angle_deg_var, width=12).grid(row=1, column=1, sticky='w', pady=4)



        ttk.Label(target_box, text='Waypoint x forward, cm').grid(row=2, column=0, sticky='w', pady=4)

        ttk.Entry(target_box, textvariable=self.target_x_cm_var, width=12).grid(row=2, column=1, sticky='w', pady=4)



        ttk.Label(target_box, text='Waypoint y left, cm').grid(row=3, column=0, sticky='w', pady=4)

        ttk.Entry(target_box, textvariable=self.target_y_cm_var, width=12).grid(row=3, column=1, sticky='w', pady=4)



        ttk.Label(target_box, text='Waypoint theta, deg').grid(row=4, column=0, sticky='w', pady=4)

        ttk.Entry(target_box, textvariable=self.target_theta_deg_var, width=12).grid(row=4, column=1, sticky='w', pady=4)



        gain_area = ttk.Frame(main)

        gain_area.pack(fill='x', pady=10)



        pid_box = ttk.LabelFrame(gain_area, text='PID Gains', padding=10)

        pid_box.pack(side='left', fill='both', expand=True, padx=(0, 6))



        self.kp_dist_var = tk.StringVar(value='0.45')

        self.ki_dist_var = tk.StringVar(value='0.00')

        self.kd_dist_var = tk.StringVar(value='0.05')

        self.kp_yaw_var = tk.StringVar(value='1.45')

        self.ki_yaw_var = tk.StringVar(value='0.00')

        self.kd_yaw_var = tk.StringVar(value='0.08')



        pid_fields = [

            ('Kp distance', self.kp_dist_var),

            ('Ki distance', self.ki_dist_var),

            ('Kd distance', self.kd_dist_var),

            ('Kp yaw', self.kp_yaw_var),

            ('Ki yaw', self.ki_yaw_var),

            ('Kd yaw', self.kd_yaw_var),

        ]



        for i, (label, var) in enumerate(pid_fields):

            ttk.Label(pid_box, text=label).grid(row=i // 2, column=(i % 2) * 2, sticky='w', pady=4, padx=(0, 4))

            ttk.Entry(pid_box, textvariable=var, width=10).grid(row=i // 2, column=(i % 2) * 2 + 1, sticky='w', pady=4)



        bs_box = ttk.LabelFrame(gain_area, text='PD-Backstepping Gains', padding=10)

        bs_box.pack(side='left', fill='both', expand=True, padx=(6, 0))



        self.kx_var = tk.StringVar(value='0.45')

        self.ky_var = tk.StringVar(value='1.20')

        self.ktheta_var = tk.StringVar(value='1.60')

        self.kdtheta_var = tk.StringVar(value='0.08')

        self.w_target_var = tk.StringVar(value='0.70')

        self.w_final_yaw_var = tk.StringVar(value='0.30')



        bs_fields = [

            ('kx', self.kx_var),

            ('ky', self.ky_var),

            ('ktheta', self.ktheta_var),

            ('kdtheta', self.kdtheta_var),

            ('w target heading', self.w_target_var),

            ('w final yaw', self.w_final_yaw_var),

        ]



        for i, (label, var) in enumerate(bs_fields):

            ttk.Label(bs_box, text=label).grid(row=i // 2, column=(i % 2) * 2, sticky='w', pady=4, padx=(0, 4))

            ttk.Entry(bs_box, textvariable=var, width=10).grid(row=i // 2, column=(i % 2) * 2 + 1, sticky='w', pady=4)



        limit_box = ttk.LabelFrame(main, text='Velocity Limits / Options', padding=10)

        limit_box.pack(fill='x', pady=(0, 10))



        self.v_max_var = tk.StringVar(value='0.16')

        self.v_min_var = tk.StringVar(value='0.02')

        self.omega_max_var = tk.StringVar(value='0.90')

        self.allow_reverse_var = tk.BooleanVar(value=False)



        ttk.Label(limit_box, text='v max, m/s').grid(row=0, column=0, sticky='w', padx=4)

        ttk.Entry(limit_box, textvariable=self.v_max_var, width=10).grid(row=0, column=1, sticky='w', padx=4)



        ttk.Label(limit_box, text='v min, m/s').grid(row=0, column=2, sticky='w', padx=4)

        ttk.Entry(limit_box, textvariable=self.v_min_var, width=10).grid(row=0, column=3, sticky='w', padx=4)



        ttk.Label(limit_box, text='omega max, rad/s').grid(row=0, column=4, sticky='w', padx=4)

        ttk.Entry(limit_box, textvariable=self.omega_max_var, width=10).grid(row=0, column=5, sticky='w', padx=4)



        ttk.Checkbutton(limit_box, text='Allow reverse', variable=self.allow_reverse_var).grid(row=0, column=6, sticky='w', padx=10)



        button_box = ttk.LabelFrame(main, text='Run / Save', padding=10)

        button_box.pack(fill='x', pady=(0, 10))



        ttk.Button(button_box, text='RUN SELECTED TEST', style='Big.TButton',

                   command=self.run_selected_test).grid(row=0, column=0, padx=5, pady=5, sticky='ew')



        ttk.Button(button_box, text='RUN ROTATE LEFT', style='Big.TButton',

                   command=lambda: self.run_rotation_direction('left')).grid(row=0, column=1, padx=5, pady=5, sticky='ew')



        ttk.Button(button_box, text='RUN ROTATE RIGHT', style='Big.TButton',

                   command=lambda: self.run_rotation_direction('right')).grid(row=0, column=2, padx=5, pady=5, sticky='ew')



        ttk.Button(button_box, text='STOP NOW', style='Stop.TButton',

                   command=self.stop_now).grid(row=0, column=3, padx=8, pady=5, sticky='ew')



        ttk.Button(button_box, text='SAVE PARAMETERS', style='Big.TButton',

                   command=self.save_parameters).grid(row=0, column=4, padx=5, pady=5, sticky='ew')



        ttk.Button(button_box, text='SAVE LAST RUN REPORT', style='Big.TButton',

                   command=self.save_last_run_report).grid(row=0, column=5, padx=5, pady=5, sticky='ew')



        for i in range(6):

            button_box.columnconfigure(i, weight=1)



        status_box = ttk.LabelFrame(main, text='Live Status', padding=10)

        status_box.pack(fill='x', pady=(0, 10))



        self.status_var = tk.StringVar(value='Ready')

        self.odom_var = tk.StringVar(value='Odom: waiting /odom_raw')

        self.error_var = tk.StringVar(value='Error: none')

        self.cmd_var = tk.StringVar(value='Cmd: v=0.000, omega=0.000')

        self.report_var = tk.StringVar(value='Report: none')



        ttk.Label(status_box, textvariable=self.status_var, style='Status.TLabel').pack(anchor='w')

        ttk.Label(status_box, textvariable=self.odom_var, style='Status.TLabel').pack(anchor='w')

        ttk.Label(status_box, textvariable=self.error_var, style='Status.TLabel').pack(anchor='w')

        ttk.Label(status_box, textvariable=self.cmd_var, style='Status.TLabel').pack(anchor='w')

        ttk.Label(status_box, textvariable=self.report_var, style='Status.TLabel').pack(anchor='w')



        table_box = ttk.LabelFrame(main, text='Last Run Samples', padding=10)

        table_box.pack(fill='both', expand=True)



        columns = ('time', 'x', 'y', 'yaw', 'ex', 'ey', 'eyaw', 'v', 'omega')

        self.tree = ttk.Treeview(table_box, columns=columns, show='headings', height=10)



        headings = {

            'time': 't',

            'x': 'x',

            'y': 'y',

            'yaw': 'yaw',

            'ex': 'e_x',

            'ey': 'e_y',

            'eyaw': 'e_yaw',

            'v': 'v',

            'omega': 'omega',

        }



        for col in columns:

            self.tree.heading(col, text=headings[col])

            self.tree.column(col, width=90, anchor='center')



        self.tree.pack(fill='both', expand=True)



    def ensure_summary_header(self):

        if os.path.exists(self.summary_csv):

            return



        with open(self.summary_csv, 'w', newline='') as f:

            writer = csv.writer(f)

            writer.writerow([

                'timestamp',

                'controller',

                'test_type',

                'target_distance_cm',

                'target_angle_deg',

                'target_x_cm',

                'target_y_cm',

                'target_theta_deg',

                'max_duration_s',

                'run_time_s',

                'final_x_m',

                'final_y_m',

                'final_yaw_rad',

                'final_distance_error_cm',

                'final_lateral_error_cm',

                'final_yaw_error_deg',

                'max_abs_v',

                'max_abs_omega',

                'mean_abs_distance_error_cm',

                'mean_abs_lateral_error_cm',

                'mean_abs_yaw_error_deg',

                'report_dir',

                'params'

            ])



    def ensure_params_header(self):

        if os.path.exists(self.params_csv):

            return



        with open(self.params_csv, 'w', newline='') as f:

            writer = csv.writer(f)

            writer.writerow([

                'timestamp',

                'controller',

                'kp_dist',

                'ki_dist',

                'kd_dist',

                'kp_yaw',

                'ki_yaw',

                'kd_yaw',

                'kx',

                'ky',

                'ktheta',

                'kdtheta',

                'w_target',

                'w_final_yaw',

                'v_min',

                'v_max',

                'omega_max',

                'allow_reverse'

            ])



    def read_float(self, var, default):

        try:

            return float(var.get().strip())

        except Exception:

            return default



    def get_pid_gains(self):

        return {

            'kp_dist': self.read_float(self.kp_dist_var, 0.45),

            'ki_dist': self.read_float(self.ki_dist_var, 0.0),

            'kd_dist': self.read_float(self.kd_dist_var, 0.05),

            'kp_yaw': self.read_float(self.kp_yaw_var, 1.45),

            'ki_yaw': self.read_float(self.ki_yaw_var, 0.0),

            'kd_yaw': self.read_float(self.kd_yaw_var, 0.08),

        }



    def get_backstepping_gains(self):

        return {

            'kx': self.read_float(self.kx_var, 0.45),

            'ky': self.read_float(self.ky_var, 1.20),

            'ktheta': self.read_float(self.ktheta_var, 1.60),

            'kdtheta': self.read_float(self.kdtheta_var, 0.08),

            'w_target': self.read_float(self.w_target_var, 0.70),

            'w_final_yaw': self.read_float(self.w_final_yaw_var, 0.30),

        }



    def get_limits(self):

        return {

            'v_max': abs(self.read_float(self.v_max_var, 0.16)),

            'v_min': abs(self.read_float(self.v_min_var, 0.02)),

            'omega_max': abs(self.read_float(self.omega_max_var, 0.90)),

            'allow_reverse': bool(self.allow_reverse_var.get()),

        }



    def odom_callback(self, msg):

        self.x = float(msg.pose.pose.position.x)

        self.y = float(msg.pose.pose.position.y)

        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        self.odom_ok = True



    def signal_stop_handler(self, signum, frame):

        self.get_logger().warn(f'Received signal {signum}. Stopping robot...')

        self.stop_now()

        if rclpy.ok():

            rclpy.shutdown()



    def publish_cmd(self, v, omega):

        cmd = Twist()

        cmd.linear.x = float(v)

        cmd.angular.z = float(omega)

        self.cmd_pub.publish(cmd)



    def stop_burst(self):

        for _ in range(35):

            self.publish_cmd(0.0, 0.0)

            self.root.update_idletasks()

            time.sleep(0.015)



    def stop_now(self):

        self.active = False

        self.publish_cmd(0.0, 0.0)

        self.stop_burst()

        self.status_var.set('STOPPED. Repeated zero /cmd_vel sent.')

        self.cmd_var.set('Cmd: v=0.000, omega=0.000')



    def setup_target_from_gui(self, direction=None):

        self.controller_type = self.controller_var.get()

        self.test_type = self.test_type_var.get()



        self.max_duration_s = clamp(self.read_float(self.max_duration_var, 10.0), 0.2, 120.0)



        self.start_x = self.x

        self.start_y = self.y

        self.start_yaw = self.yaw



        if self.test_type == 'distance':

            d = self.read_float(self.target_distance_cm_var, 60.0) / 100.0

            self.target_x = self.start_x + d * math.cos(self.start_yaw)

            self.target_y = self.start_y + d * math.sin(self.start_yaw)

            self.target_yaw = self.start_yaw



        elif self.test_type == 'rotation':

            angle_deg = self.read_float(self.target_angle_deg_var, 90.0)



            if direction == 'right':

                angle_deg = -abs(angle_deg)

            elif direction == 'left':

                angle_deg = abs(angle_deg)



            self.target_x = self.start_x

            self.target_y = self.start_y

            self.target_yaw = wrap_pi(self.start_yaw + math.radians(angle_deg))



        else:

            bx = self.read_float(self.target_x_cm_var, 60.0) / 100.0

            by = self.read_float(self.target_y_cm_var, 0.0) / 100.0

            btheta = math.radians(self.read_float(self.target_theta_deg_var, 0.0))



            self.target_x = self.start_x + bx * math.cos(self.start_yaw) - by * math.sin(self.start_yaw)

            self.target_y = self.start_y + bx * math.sin(self.start_yaw) + by * math.cos(self.start_yaw)

            self.target_yaw = wrap_pi(self.start_yaw + btheta)



    def reset_controller_state(self):

        self.integral_dist = 0.0

        self.integral_yaw = 0.0

        self.prev_dist_error = 0.0

        self.prev_yaw_error = 0.0

        self.prev_theta_control = 0.0

        self.prev_time = time.time()

        self.run_records = []



        for item in self.tree.get_children():

            self.tree.delete(item)



    def run_selected_test(self):

        if self.active:

            messagebox.showwarning('Running', 'Robot is already running. Press STOP first.')

            return



        if not self.odom_ok:

            ok = messagebox.askyesno(

                'No odom',

                'No /odom_raw received yet. Controller calibration needs odom feedback. Run anyway?'

            )

            if not ok:

                return



        self.setup_target_from_gui()

        self.reset_controller_state()



        self.run_start_time = time.time()

        self.active = True



        self.status_var.set(

            f'RUNNING {self.controller_type} | test={self.test_type} | target=({self.target_x:.2f}, {self.target_y:.2f}, {self.target_yaw:.2f})'

        )



    def run_rotation_direction(self, direction):

        self.test_type_var.set('rotation')

        self.setup_target_from_gui(direction=direction)

        self.reset_controller_state()



        self.run_start_time = time.time()

        self.active = True



        self.status_var.set(f'RUNNING {self.controller_var.get()} rotation {direction}')



    def compute_errors(self):

        dx = self.target_x - self.x

        dy = self.target_y - self.y



        ex = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy

        ey = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy



        dist_error = math.hypot(dx, dy)

        yaw_error = wrap_pi(self.target_yaw - self.yaw)



        return ex, ey, dist_error, yaw_error



    def check_finished(self, ex, ey, dist_error, yaw_error):

        dist_tol_m = self.read_float(self.dist_tol_cm_var, 3.0) / 100.0

        angle_tol_rad = math.radians(self.read_float(self.angle_tol_deg_var, 3.0))



        if self.test_type == 'distance':

            return dist_error < dist_tol_m



        if self.test_type == 'rotation':

            return abs(yaw_error) < angle_tol_rad



        return dist_error < dist_tol_m and abs(yaw_error) < angle_tol_rad



    def pid_control(self, ex, ey, dist_error, yaw_error, dt):

        gains = self.get_pid_gains()

        limits = self.get_limits()



        kp_dist = gains['kp_dist']

        ki_dist = gains['ki_dist']

        kd_dist = gains['kd_dist']

        kp_yaw = gains['kp_yaw']

        ki_yaw = gains['ki_yaw']

        kd_yaw = gains['kd_yaw']



        if self.test_type == 'rotation':

            dist_ctrl_error = 0.0

            heading_error = yaw_error

        elif self.test_type == 'distance':

            dist_ctrl_error = ex

            heading_error = yaw_error

        else:

            target_heading = math.atan2(ey, max(ex, 0.05))

            dist_ctrl_error = dist_error



            if dist_error > 0.12:

                heading_error = wrap_pi(target_heading + 0.35 * yaw_error)

            else:

                heading_error = yaw_error



        self.integral_dist += dist_ctrl_error * dt

        self.integral_yaw += heading_error * dt



        self.integral_dist = clamp(self.integral_dist, -1.0, 1.0)

        self.integral_yaw = clamp(self.integral_yaw, -2.0, 2.0)



        d_dist = (dist_ctrl_error - self.prev_dist_error) / dt

        d_yaw = wrap_pi(heading_error - self.prev_yaw_error) / dt



        self.prev_dist_error = dist_ctrl_error

        self.prev_yaw_error = heading_error



        v = kp_dist * dist_ctrl_error + ki_dist * self.integral_dist + kd_dist * d_dist

        omega = kp_yaw * heading_error + ki_yaw * self.integral_yaw + kd_yaw * d_yaw



        if self.test_type == 'rotation':

            v = 0.0



        if not limits['allow_reverse']:

            v = clamp(v, 0.0, limits['v_max'])

        else:

            v = clamp(v, -limits['v_max'], limits['v_max'])



        if abs(v) > 0.001:

            v = math.copysign(max(abs(v), limits['v_min']), v)



        omega = clamp(omega, -limits['omega_max'], limits['omega_max'])



        return v, omega, heading_error



    def backstepping_control(self, ex, ey, dist_error, yaw_error, dt):

        gains = self.get_backstepping_gains()

        limits = self.get_limits()



        kx = gains['kx']

        ky = gains['ky']

        ktheta = gains['ktheta']

        kdtheta = gains['kdtheta']

        w_target = gains['w_target']

        w_final_yaw = gains['w_final_yaw']



        if self.test_type == 'rotation':

            theta_control = yaw_error

            v = 0.0

        else:

            target_heading = math.atan2(ey, max(ex, 0.05))



            if self.test_type == 'distance':

                theta_control = yaw_error

            else:

                theta_control = wrap_pi(w_target * target_heading + w_final_yaw * yaw_error)



            v = kx * ex * math.cos(theta_control)



            lateral_slow = math.exp(-1.1 * abs(ey))

            heading_slow = math.exp(-0.7 * abs(theta_control))

            v = v * lateral_slow * heading_slow



        dtheta = wrap_pi(theta_control - self.prev_theta_control) / dt

        self.prev_theta_control = theta_control



        omega = ky * ey + ktheta * theta_control + kdtheta * dtheta



        if self.test_type == 'rotation':

            omega = ktheta * yaw_error + kdtheta * dtheta



        if not limits['allow_reverse']:

            v = clamp(v, 0.0, limits['v_max'])

        else:

            v = clamp(v, -limits['v_max'], limits['v_max'])



        if abs(v) > 0.001:

            v = math.copysign(max(abs(v), limits['v_min']), v)



        omega = clamp(omega, -limits['omega_max'], limits['omega_max'])



        return v, omega, theta_control



    def append_record(self, t, ex, ey, dist_error, yaw_error, control_heading, v, omega):

        row = {

            'time_s': t,

            'x_m': self.x,

            'y_m': self.y,

            'yaw_rad': self.yaw,

            'target_x_m': self.target_x,

            'target_y_m': self.target_y,

            'target_yaw_rad': self.target_yaw,

            'ex_m': ex,

            'ey_m': ey,

            'distance_error_m': dist_error,

            'yaw_error_rad': yaw_error,

            'control_heading_rad': control_heading,

            'cmd_v_mps': v,

            'cmd_omega_radps': omega,

            'controller': self.controller_type,

            'test_type': self.test_type,

        }



        self.run_records.append(row)



        if len(self.run_records) % 5 == 0:

            self.tree.insert(

                '',

                'end',

                values=(

                    f'{t:.2f}',

                    f'{self.x:.2f}',

                    f'{self.y:.2f}',

                    f'{self.yaw:.2f}',

                    f'{ex:.2f}',

                    f'{ey:.2f}',

                    f'{yaw_error:.2f}',

                    f'{v:.2f}',

                    f'{omega:.2f}',

                )

            )



            children = self.tree.get_children()

            if len(children) > 60:

                self.tree.delete(children[0])



    def publish_debug(self, ex, ey, dist_error, yaw_error, control_heading, v, omega):

        msg = Float32MultiArray()

        msg.data = [

            float(ex),

            float(ey),

            float(dist_error),

            float(yaw_error),

            float(control_heading),

            float(v),

            float(omega),

            float(self.target_x),

            float(self.target_y),

            float(self.target_yaw),

        ]

        self.debug_pub.publish(msg)



    def control_tick(self):

        if not self.active:

            return



        now = time.time()

        dt = max(now - self.prev_time, 1e-3)

        self.prev_time = now



        run_time = now - self.run_start_time



        ex, ey, dist_error, yaw_error = self.compute_errors()



        if self.check_finished(ex, ey, dist_error, yaw_error):

            self.stop_now()

            self.status_var.set('FINISHED. Target reached. You can save report.')

            self.last_status = 'finished'

            return



        if run_time > self.max_duration_s:

            self.stop_now()

            self.status_var.set('TIMEOUT. Max duration reached. You can save report.')

            self.last_status = 'timeout'

            return



        self.controller_type = self.controller_var.get()



        if self.controller_type == 'PID':

            v, omega, control_heading = self.pid_control(ex, ey, dist_error, yaw_error, dt)

        else:

            v, omega, control_heading = self.backstepping_control(ex, ey, dist_error, yaw_error, dt)



        self.publish_cmd(v, omega)

        self.publish_debug(ex, ey, dist_error, yaw_error, control_heading, v, omega)



        self.append_record(run_time, ex, ey, dist_error, yaw_error, control_heading, v, omega)



        self.error_var.set(

            f'Error: ex={ex:.3f}m, ey={ey:.3f}m, dist={dist_error:.3f}m, yaw={math.degrees(yaw_error):.2f}deg'

        )

        self.cmd_var.set(f'Cmd: v={v:.3f} m/s, omega={omega:.3f} rad/s')



    def save_parameters(self):

        now = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        pid = self.get_pid_gains()

        bs = self.get_backstepping_gains()

        limits = self.get_limits()



        row = [

            now,

            self.controller_var.get(),

            pid['kp_dist'],

            pid['ki_dist'],

            pid['kd_dist'],

            pid['kp_yaw'],

            pid['ki_yaw'],

            pid['kd_yaw'],

            bs['kx'],

            bs['ky'],

            bs['ktheta'],

            bs['kdtheta'],

            bs['w_target'],

            bs['w_final_yaw'],

            limits['v_min'],

            limits['v_max'],

            limits['omega_max'],

            int(limits['allow_reverse'])

        ]



        with open(self.params_csv, 'a', newline='') as f:

            writer = csv.writer(f)

            writer.writerow(row)



        messagebox.showinfo('Saved', f'Saved parameters to:\n{self.params_csv}')



    def save_last_run_report(self):

        if not self.run_records:

            messagebox.showwarning('No data', 'No run records to save.')

            return



        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        run_dir = os.path.join(self.save_dir, f'{self.controller_type}_{self.test_type}_{timestamp}')

        os.makedirs(run_dir, exist_ok=True)



        self.last_run_dir = run_dir



        csv_path = os.path.join(run_dir, 'controller_run_log.csv')

        png_path = os.path.join(run_dir, 'controller_run_report.png')



        with open(csv_path, 'w', newline='') as f:

            writer = csv.DictWriter(f, fieldnames=list(self.run_records[0].keys()))

            writer.writeheader()

            writer.writerows(self.run_records)



        self.save_plot(png_path)

        self.append_summary(run_dir)



        self.report_var.set(f'Report: {run_dir}')

        messagebox.showinfo('Saved', f'Saved report:\n{run_dir}')



    def save_plot(self, png_path):

        try:

            import matplotlib

            matplotlib.use('Agg')

            import matplotlib.pyplot as plt

        except Exception as e:

            self.get_logger().error(f'Cannot import matplotlib: {e}')

            return



        t = [r['time_s'] for r in self.run_records]

        x = [r['x_m'] for r in self.run_records]

        y = [r['y_m'] for r in self.run_records]

        ex = [r['ex_m'] for r in self.run_records]

        ey = [r['ey_m'] for r in self.run_records]

        yaw_e = [math.degrees(r['yaw_error_rad']) for r in self.run_records]

        v = [r['cmd_v_mps'] for r in self.run_records]

        omega = [r['cmd_omega_radps'] for r in self.run_records]

        control_heading = [math.degrees(r['control_heading_rad']) for r in self.run_records]



        fig = plt.figure(figsize=(15, 10))



        ax1 = fig.add_subplot(2, 2, 1)

        ax1.plot(x, y, linewidth=2.5, label='robot path')

        ax1.scatter([x[0]], [y[0]], marker='o', s=70, label='start')

        ax1.scatter([self.target_x], [self.target_y], marker='x', s=90, label='target')

        ax1.scatter([x[-1]], [y[-1]], marker='s', s=70, label='end')

        ax1.set_title('Robot path')

        ax1.set_xlabel('x (m)')

        ax1.set_ylabel('y (m)')

        ax1.axis('equal')

        ax1.grid(True)

        ax1.legend()



        ax2 = fig.add_subplot(2, 2, 2)

        ax2.plot(t, ex, label='e_x')

        ax2.plot(t, ey, label='e_y')

        ax2.plot(t, yaw_e, label='yaw_error_deg')

        ax2.set_title('Tracking errors')

        ax2.set_xlabel('time (s)')

        ax2.set_ylabel('error')

        ax2.grid(True)

        ax2.legend()



        ax3 = fig.add_subplot(2, 2, 3)

        ax3.plot(t, v, label='linear.x')

        ax3.plot(t, omega, label='angular.z')

        ax3.set_title('Command velocity')

        ax3.set_xlabel('time (s)')

        ax3.set_ylabel('velocity')

        ax3.grid(True)

        ax3.legend()



        ax4 = fig.add_subplot(2, 2, 4)

        ax4.plot(t, control_heading, label='control heading deg')

        ax4.set_title('Controller heading signal')

        ax4.set_xlabel('time (s)')

        ax4.set_ylabel('deg')

        ax4.grid(True)

        ax4.legend()



        fig.tight_layout()

        fig.savefig(png_path, dpi=150)

        plt.close(fig)



    def append_summary(self, run_dir):

        if not self.run_records:

            return



        final = self.run_records[-1]



        abs_dist_error_cm = [abs(r['distance_error_m']) * 100.0 for r in self.run_records]

        abs_lateral_error_cm = [abs(r['ey_m']) * 100.0 for r in self.run_records]

        abs_yaw_error_deg = [abs(math.degrees(r['yaw_error_rad'])) for r in self.run_records]

        abs_v = [abs(r['cmd_v_mps']) for r in self.run_records]

        abs_w = [abs(r['cmd_omega_radps']) for r in self.run_records]



        pid = self.get_pid_gains()

        bs = self.get_backstepping_gains()

        limits = self.get_limits()



        params = {

            **pid,

            **bs,

            **limits,

        }



        row = [

            datetime.now().strftime('%Y-%m-%d_%H-%M-%S'),

            self.controller_type,

            self.test_type,

            self.read_float(self.target_distance_cm_var, 0.0),

            self.read_float(self.target_angle_deg_var, 0.0),

            self.read_float(self.target_x_cm_var, 0.0),

            self.read_float(self.target_y_cm_var, 0.0),

            self.read_float(self.target_theta_deg_var, 0.0),

            self.max_duration_s,

            final['time_s'],

            final['x_m'],

            final['y_m'],

            final['yaw_rad'],

            final['distance_error_m'] * 100.0,

            final['ey_m'] * 100.0,

            math.degrees(final['yaw_error_rad']),

            max(abs_v),

            max(abs_w),

            sum(abs_dist_error_cm) / len(abs_dist_error_cm),

            sum(abs_lateral_error_cm) / len(abs_lateral_error_cm),

            sum(abs_yaw_error_deg) / len(abs_yaw_error_deg),

            run_dir,

            str(params)

        ]



        with open(self.summary_csv, 'a', newline='') as f:

            writer = csv.writer(f)

            writer.writerow(row)



    def tick(self):

        rclpy.spin_once(self, timeout_sec=0.001)



        if self.odom_ok:

            self.odom_var.set(

                f'Odom: x={self.x:.3f}m, y={self.y:.3f}m, yaw={math.degrees(self.yaw):.2f}deg'

            )

        else:

            self.odom_var.set('Odom: waiting /odom_raw')



        self.control_tick()



        self.root.after(int(1000.0 / self.control_rate_hz), self.tick)



    def on_close(self):

        self.stop_now()

        self.root.destroy()



        if rclpy.ok():

            rclpy.shutdown()



    def run_gui(self):

        self.root.after(50, self.tick)

        self.root.mainloop()





def main(args=None):

    rclpy.init(args=args)

    node = ControllerCalibGuiNode()



    try:

        node.run_gui()

    except KeyboardInterrupt:

        node.stop_now()

    finally:

        try:

            node.stop_now()

        except Exception:

            pass



        node.destroy_node()



        if rclpy.ok():

            rclpy.shutdown()





if __name__ == '__main__':

    main()

