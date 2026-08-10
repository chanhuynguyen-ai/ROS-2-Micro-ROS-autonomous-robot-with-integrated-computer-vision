
#!/usr/bin/env python3



import os

import csv

import math

import time

from datetime import datetime



import tkinter as tk

from tkinter import ttk, messagebox



import rclpy

from rclpy.node import Node



from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry





def clamp(value, low, high):

    return max(low, min(high, value))





def yaw_from_quaternion(q):

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)

    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    return math.atan2(siny_cosp, cosy_cosp)





def wrap_pi(angle):

    return math.atan2(math.sin(angle), math.cos(angle))





class MotorCalibGuiNode(Node):

    def __init__(self):

        super().__init__('motor_calib_gui_node')



        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('odom_topic', '/odom_raw')

        self.declare_parameter('save_dir', '/root/yahboomcar_ws/demo_reports/motor_calibration')

        self.declare_parameter('publish_rate_hz', 20.0)

        self.declare_parameter('use_odom', True)



        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.odom_topic = str(self.get_parameter('odom_topic').value)

        self.save_dir = str(self.get_parameter('save_dir').value)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.use_odom = bool(self.get_parameter('use_odom').value)



        os.makedirs(self.save_dir, exist_ok=True)



        self.csv_path = os.path.join(self.save_dir, 'motor_calibration_log.csv')

        self.summary_path = os.path.join(self.save_dir, 'motor_calibration_summary.csv')



        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)



        self.odom_sub = self.create_subscription(

            Odometry,

            self.odom_topic,

            self.odom_callback,

            20

        )



        self.odom_ok = False

        self.odom_x = 0.0

        self.odom_y = 0.0

        self.odom_yaw = 0.0



        self.odom_start_x = 0.0

        self.odom_start_y = 0.0

        self.odom_start_yaw = 0.0



        self.odom_end_x = 0.0

        self.odom_end_y = 0.0

        self.odom_end_yaw = 0.0



        self.active = False

        self.active_mode = 'none'

        self.active_linear = 0.0

        self.active_angular = 0.0

        self.run_start_time = 0.0

        self.run_end_time = 0.0



        self.last_mode = ''

        self.last_linear = 0.0

        self.last_angular = 0.0

        self.last_duration = 0.0

        self.last_direction = ''

        self.last_odom_distance_cm = 0.0

        self.last_odom_angle_deg = 0.0



        self.root = tk.Tk()

        self.root.title('SimpleRobot Motor Calibration')

        self.root.geometry('980x720')

        self.root.protocol('WM_DELETE_WINDOW', self.on_close)



        self.build_gui()

        self.ensure_csv_header()



        self.get_logger().info('Motor calibration GUI started')

        self.get_logger().info(f'Publish cmd_vel: {self.cmd_vel_topic}')

        self.get_logger().info(f'Subscribe odom:  {self.odom_topic}')

        self.get_logger().info(f'Save CSV:        {self.csv_path}')



    def build_gui(self):

        style = ttk.Style()

        style.theme_use('clam')



        style.configure('Title.TLabel', font=('Arial', 20, 'bold'))

        style.configure('Header.TLabel', font=('Arial', 13, 'bold'))

        style.configure('Big.TButton', font=('Arial', 13, 'bold'), padding=8)

        style.configure('Stop.TButton', font=('Arial', 15, 'bold'), padding=10)

        style.configure('Status.TLabel', font=('Arial', 12))



        main = ttk.Frame(self.root, padding=14)

        main.pack(fill='both', expand=True)



        title = ttk.Label(main, text='SimpleRobot Motor Calibration GUI', style='Title.TLabel')

        title.pack(anchor='center', pady=(0, 12))



        top = ttk.Frame(main)

        top.pack(fill='x')



        cmd_box = ttk.LabelFrame(top, text='Command Input', padding=10)

        cmd_box.pack(side='left', fill='both', expand=True, padx=(0, 8))



        self.linear_var = tk.StringVar(value='0.50')

        self.angular_var = tk.StringVar(value='0.80')

        self.duration_var = tk.StringVar(value='4.00')



        ttk.Label(cmd_box, text='Linear speed v, m/s, 0–1.0').grid(row=0, column=0, sticky='w', pady=4)

        ttk.Entry(cmd_box, textvariable=self.linear_var, width=14).grid(row=0, column=1, sticky='w', pady=4)



        ttk.Label(cmd_box, text='Angular speed omega, rad/s').grid(row=1, column=0, sticky='w', pady=4)

        ttk.Entry(cmd_box, textvariable=self.angular_var, width=14).grid(row=1, column=1, sticky='w', pady=4)



        ttk.Label(cmd_box, text='Run duration, seconds').grid(row=2, column=0, sticky='w', pady=4)

        ttk.Entry(cmd_box, textvariable=self.duration_var, width=14).grid(row=2, column=1, sticky='w', pady=4)



        self.actual_distance_cm_var = tk.StringVar(value='')

        self.actual_angle_deg_var = tk.StringVar(value='')

        self.notes_var = tk.StringVar(value='')



        result_box = ttk.LabelFrame(top, text='Measured Result', padding=10)

        result_box.pack(side='right', fill='both', expand=True, padx=(8, 0))



        ttk.Label(result_box, text='Actual distance, cm').grid(row=0, column=0, sticky='w', pady=4)

        ttk.Entry(result_box, textvariable=self.actual_distance_cm_var, width=14).grid(row=0, column=1, sticky='w', pady=4)



        ttk.Label(result_box, text='Actual angle, deg').grid(row=1, column=0, sticky='w', pady=4)

        ttk.Entry(result_box, textvariable=self.actual_angle_deg_var, width=14).grid(row=1, column=1, sticky='w', pady=4)



        ttk.Label(result_box, text='Notes').grid(row=2, column=0, sticky='w', pady=4)

        ttk.Entry(result_box, textvariable=self.notes_var, width=34).grid(row=2, column=1, sticky='w', pady=4)



        btn_box = ttk.LabelFrame(main, text='Run Control', padding=12)

        btn_box.pack(fill='x', pady=12)



        ttk.Button(btn_box, text='RUN STRAIGHT', style='Big.TButton',

                   command=self.run_straight).grid(row=0, column=0, padx=5, pady=5, sticky='ew')



        ttk.Button(btn_box, text='TURN LEFT', style='Big.TButton',

                   command=self.run_turn_left).grid(row=0, column=1, padx=5, pady=5, sticky='ew')



        ttk.Button(btn_box, text='TURN RIGHT', style='Big.TButton',

                   command=self.run_turn_right).grid(row=0, column=2, padx=5, pady=5, sticky='ew')



        ttk.Button(btn_box, text='CUSTOM v + omega', style='Big.TButton',

                   command=self.run_custom).grid(row=0, column=3, padx=5, pady=5, sticky='ew')



        ttk.Button(btn_box, text='STOP NOW', style='Stop.TButton',

                   command=self.stop_now).grid(row=0, column=4, padx=8, pady=5, sticky='ew')



        for i in range(5):

            btn_box.columnconfigure(i, weight=1)



        save_box = ttk.LabelFrame(main, text='Save Calibration Sample', padding=12)

        save_box.pack(fill='x', pady=(0, 12))



        ttk.Button(save_box, text='SAVE LINEAR SAMPLE', style='Big.TButton',

                   command=self.save_linear_sample).grid(row=0, column=0, padx=5, pady=5, sticky='ew')



        ttk.Button(save_box, text='SAVE ANGULAR SAMPLE', style='Big.TButton',

                   command=self.save_angular_sample).grid(row=0, column=1, padx=5, pady=5, sticky='ew')



        ttk.Button(save_box, text='SAVE CUSTOM SAMPLE', style='Big.TButton',

                   command=self.save_custom_sample).grid(row=0, column=2, padx=5, pady=5, sticky='ew')



        ttk.Button(save_box, text='CLEAR INPUT', style='Big.TButton',

                   command=self.clear_input).grid(row=0, column=3, padx=5, pady=5, sticky='ew')



        for i in range(4):

            save_box.columnconfigure(i, weight=1)



        info_box = ttk.LabelFrame(main, text='Live Status', padding=10)

        info_box.pack(fill='x')



        self.status_var = tk.StringVar(value='Ready')

        self.odom_var = tk.StringVar(value='Odom: waiting /odom_raw')

        self.last_run_var = tk.StringVar(value='Last run: none')

        self.scale_var = tk.StringVar(value='Scale: no saved samples yet')



        ttk.Label(info_box, textvariable=self.status_var, style='Status.TLabel').pack(anchor='w')

        ttk.Label(info_box, textvariable=self.odom_var, style='Status.TLabel').pack(anchor='w')

        ttk.Label(info_box, textvariable=self.last_run_var, style='Status.TLabel').pack(anchor='w')

        ttk.Label(info_box, textvariable=self.scale_var, style='Status.TLabel').pack(anchor='w')



        table_box = ttk.LabelFrame(main, text='Saved Calibration Samples', padding=10)

        table_box.pack(fill='both', expand=True, pady=(12, 0))



        columns = (

            'time',

            'type',

            'v',

            'omega',

            'duration',

            'actual',

            'odom',

            'scale',

            'notes'

        )



        self.tree = ttk.Treeview(table_box, columns=columns, show='headings', height=10)



        headings = {

            'time': 'Time',

            'type': 'Type',

            'v': 'v',

            'omega': 'omega',

            'duration': 't',

            'actual': 'Actual',

            'odom': 'Odom',

            'scale': 'Scale',

            'notes': 'Notes',

        }



        widths = {

            'time': 150,

            'type': 90,

            'v': 70,

            'omega': 80,

            'duration': 70,

            'actual': 90,

            'odom': 90,

            'scale': 90,

            'notes': 250,

        }



        for col in columns:

            self.tree.heading(col, text=headings[col])

            self.tree.column(col, width=widths[col], anchor='center')



        self.tree.pack(fill='both', expand=True)



        self.load_existing_samples_to_table()

        self.update_scale_status()



    def ensure_csv_header(self):

        if os.path.exists(self.csv_path):

            return



        with open(self.csv_path, 'w', newline='') as f:

            writer = csv.writer(f)

            writer.writerow([

                'timestamp',

                'sample_type',

                'direction',

                'cmd_linear_mps',

                'cmd_angular_radps',

                'duration_s',

                'theory_distance_cm',

                'theory_angle_deg',

                'actual_distance_cm',

                'actual_angle_deg',

                'odom_distance_cm',

                'odom_angle_deg',

                'linear_scale',

                'angular_scale',

                'notes'

            ])



    def odom_callback(self, msg):

        self.odom_x = float(msg.pose.pose.position.x)

        self.odom_y = float(msg.pose.pose.position.y)

        self.odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        self.odom_ok = True



    def read_float(self, var, default):

        try:

            return float(var.get().strip())

        except Exception:

            return default



    def get_inputs(self):

        linear = self.read_float(self.linear_var, 0.0)

        angular = self.read_float(self.angular_var, 0.0)

        duration = self.read_float(self.duration_var, 0.0)



        linear = clamp(linear, -1.0, 1.0)

        angular = clamp(angular, -5.0, 5.0)

        duration = clamp(duration, 0.05, 60.0)



        return linear, angular, duration



    def capture_odom_start(self):

        self.odom_start_x = self.odom_x

        self.odom_start_y = self.odom_y

        self.odom_start_yaw = self.odom_yaw



    def capture_odom_end(self):

        self.odom_end_x = self.odom_x

        self.odom_end_y = self.odom_y

        self.odom_end_yaw = self.odom_yaw



        dx = self.odom_end_x - self.odom_start_x

        dy = self.odom_end_y - self.odom_start_y



        self.last_odom_distance_cm = math.hypot(dx, dy) * 100.0

        self.last_odom_angle_deg = abs(wrap_pi(self.odom_end_yaw - self.odom_start_yaw)) * 180.0 / math.pi



    def start_run(self, mode, linear, angular, duration, direction):

        if self.active:

            messagebox.showwarning('Running', 'Robot is already running. Press STOP first.')

            return



        self.active = True

        self.active_mode = mode

        self.active_linear = linear

        self.active_angular = angular

        self.run_start_time = time.time()

        self.run_end_time = self.run_start_time + duration



        self.last_mode = mode

        self.last_linear = linear

        self.last_angular = angular

        self.last_duration = duration

        self.last_direction = direction



        self.last_odom_distance_cm = 0.0

        self.last_odom_angle_deg = 0.0



        self.capture_odom_start()



        self.status_var.set(

            f'RUNNING {mode}: v={linear:.3f} m/s, omega={angular:.3f} rad/s, t={duration:.2f}s'

        )



    def run_straight(self):

        linear, angular, duration = self.get_inputs()

        self.start_run('linear', abs(linear), 0.0, duration, 'straight')



    def run_turn_left(self):

        linear, angular, duration = self.get_inputs()

        self.start_run('angular', 0.0, abs(angular), duration, 'left')



    def run_turn_right(self):

        linear, angular, duration = self.get_inputs()

        self.start_run('angular', 0.0, -abs(angular), duration, 'right')



    def run_custom(self):

        linear, angular, duration = self.get_inputs()

        self.start_run('custom', linear, angular, duration, 'custom')



    def publish_cmd(self, linear, angular):

        cmd = Twist()

        cmd.linear.x = float(linear)

        cmd.angular.z = float(angular)

        self.cmd_pub.publish(cmd)



    def stop_burst(self):

        for _ in range(30):

            self.publish_cmd(0.0, 0.0)

            self.root.update_idletasks()

            time.sleep(0.015)



    def stop_now(self):

        self.active = False

        self.publish_cmd(0.0, 0.0)

        self.stop_burst()

        self.capture_odom_end()



        self.status_var.set('STOPPED. Zero /cmd_vel sent repeatedly.')

        self.last_run_var.set(

            f'Last run: {self.last_mode}, v={self.last_linear:.3f}, omega={self.last_angular:.3f}, '

            f't={self.last_duration:.2f}s, odom={self.last_odom_distance_cm:.1f}cm, '

            f'{self.last_odom_angle_deg:.1f}deg'

        )



    def tick(self):

        if self.active:

            now = time.time()



            if now < self.run_end_time:

                self.publish_cmd(self.active_linear, self.active_angular)

                remain = self.run_end_time - now

                self.status_var.set(

                    f'RUNNING {self.active_mode}: remaining {remain:.2f}s'

                )

            else:

                self.stop_now()



        if self.odom_ok:

            self.odom_var.set(

                f'Odom: x={self.odom_x:.3f} m, y={self.odom_y:.3f} m, yaw={self.odom_yaw:.3f} rad'

            )

        else:

            self.odom_var.set('Odom: waiting /odom_raw')



        rclpy.spin_once(self, timeout_sec=0.001)

        self.root.after(int(1000.0 / self.publish_rate_hz), self.tick)



    def compute_theory(self):

        theory_distance_cm = abs(self.last_linear) * self.last_duration * 100.0

        theory_angle_deg = abs(self.last_angular) * self.last_duration * 180.0 / math.pi

        return theory_distance_cm, theory_angle_deg



    def parse_actual_distance(self):

        value = self.actual_distance_cm_var.get().strip()

        if value == '':

            return 0.0

        return float(value)



    def parse_actual_angle(self):

        value = self.actual_angle_deg_var.get().strip()

        if value == '':

            return 0.0

        return float(value)



    def save_sample(self, sample_type):

        if self.active:

            messagebox.showwarning('Still running', 'Stop robot before saving sample.')

            return



        if self.last_mode == '':

            messagebox.showwarning('No run', 'Run the robot first, then save calibration sample.')

            return



        now = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        theory_distance_cm, theory_angle_deg = self.compute_theory()



        actual_distance_cm = 0.0

        actual_angle_deg = 0.0



        try:

            if sample_type in ['linear', 'custom']:

                actual_distance_cm = self.parse_actual_distance()

            if sample_type in ['angular', 'custom']:

                actual_angle_deg = self.parse_actual_angle()

        except Exception:

            messagebox.showerror('Invalid input', 'Actual distance/angle must be a number.')

            return



        linear_scale = 0.0

        angular_scale = 0.0



        if sample_type == 'linear':

            if actual_distance_cm <= 0.0:

                messagebox.showwarning('Missing distance', 'Enter actual distance in cm before SAVE LINEAR.')

                return



            if theory_distance_cm > 1e-6:

                linear_scale = actual_distance_cm / theory_distance_cm



        elif sample_type == 'angular':

            if actual_angle_deg <= 0.0:

                messagebox.showwarning('Missing angle', 'Enter actual angle in deg before SAVE ANGULAR.')

                return



            if theory_angle_deg > 1e-6:

                angular_scale = actual_angle_deg / theory_angle_deg



        elif sample_type == 'custom':

            if theory_distance_cm > 1e-6 and actual_distance_cm > 0.0:

                linear_scale = actual_distance_cm / theory_distance_cm



            if theory_angle_deg > 1e-6 and actual_angle_deg > 0.0:

                angular_scale = actual_angle_deg / theory_angle_deg



        notes = self.notes_var.get().strip()



        row = [

            now,

            sample_type,

            self.last_direction,

            f'{self.last_linear:.6f}',

            f'{self.last_angular:.6f}',

            f'{self.last_duration:.6f}',

            f'{theory_distance_cm:.6f}',

            f'{theory_angle_deg:.6f}',

            f'{actual_distance_cm:.6f}',

            f'{actual_angle_deg:.6f}',

            f'{self.last_odom_distance_cm:.6f}',

            f'{self.last_odom_angle_deg:.6f}',

            f'{linear_scale:.6f}',

            f'{angular_scale:.6f}',

            notes

        ]



        with open(self.csv_path, 'a', newline='') as f:

            writer = csv.writer(f)

            writer.writerow(row)



        self.add_row_to_table(row)

        self.update_summary()

        self.update_scale_status()



        messagebox.showinfo('Saved', f'Saved {sample_type} sample to:\n{self.csv_path}')



    def save_linear_sample(self):

        self.save_sample('linear')



    def save_angular_sample(self):

        self.save_sample('angular')



    def save_custom_sample(self):

        self.save_sample('custom')



    def add_row_to_table(self, row):

        timestamp = row[0]

        sample_type = row[1]

        v = row[3]

        omega = row[4]

        duration = row[5]



        if sample_type == 'linear':

            actual = f'{float(row[8]):.1f}cm'

            odom = f'{float(row[10]):.1f}cm'

            scale = row[12]

        elif sample_type == 'angular':

            actual = f'{float(row[9]):.1f}deg'

            odom = f'{float(row[11]):.1f}deg'

            scale = row[13]

        else:

            actual = f'{float(row[8]):.1f}cm/{float(row[9]):.1f}deg'

            odom = f'{float(row[10]):.1f}cm/{float(row[11]):.1f}deg'

            scale = f'L:{row[12]} A:{row[13]}'



        notes = row[14]



        self.tree.insert(

            '',

            'end',

            values=(timestamp, sample_type, v, omega, duration, actual, odom, scale, notes)

        )



    def load_existing_samples_to_table(self):

        if not os.path.exists(self.csv_path):

            return



        try:

            with open(self.csv_path, 'r') as f:

                reader = csv.reader(f)

                rows = list(reader)



            for row in rows[1:]:

                if len(row) >= 15:

                    self.add_row_to_table(row)

        except Exception:

            pass



    def read_saved_samples(self):

        linear_scales = []

        angular_scales = []



        if not os.path.exists(self.csv_path):

            return linear_scales, angular_scales



        with open(self.csv_path, 'r') as f:

            reader = csv.DictReader(f)



            for row in reader:

                try:

                    ls = float(row.get('linear_scale', 0.0))

                    av = float(row.get('angular_scale', 0.0))



                    if ls > 0.0:

                        linear_scales.append(ls)



                    if av > 0.0:

                        angular_scales.append(av)

                except Exception:

                    pass



        return linear_scales, angular_scales



    def update_summary(self):

        linear_scales, angular_scales = self.read_saved_samples()



        linear_avg = sum(linear_scales) / len(linear_scales) if linear_scales else 0.0

        angular_avg = sum(angular_scales) / len(angular_scales) if angular_scales else 0.0



        with open(self.summary_path, 'w', newline='') as f:

            writer = csv.writer(f)

            writer.writerow(['type', 'sample_count', 'average_scale', 'recommended_command_factor'])

            writer.writerow([

                'linear',

                len(linear_scales),

                f'{linear_avg:.6f}',

                f'{1.0 / linear_avg:.6f}' if linear_avg > 1e-6 else '0.000000'

            ])

            writer.writerow([

                'angular',

                len(angular_scales),

                f'{angular_avg:.6f}',

                f'{1.0 / angular_avg:.6f}' if angular_avg > 1e-6 else '0.000000'

            ])



    def update_scale_status(self):

        linear_scales, angular_scales = self.read_saved_samples()



        linear_avg = sum(linear_scales) / len(linear_scales) if linear_scales else 0.0

        angular_avg = sum(angular_scales) / len(angular_scales) if angular_scales else 0.0



        text = (

            f'Saved scales: linear n={len(linear_scales)}, avg={linear_avg:.4f}, '

            f'cmd_factor={1.0 / linear_avg if linear_avg > 1e-6 else 0.0:.4f} | '

            f'angular n={len(angular_scales)}, avg={angular_avg:.4f}, '

            f'cmd_factor={1.0 / angular_avg if angular_avg > 1e-6 else 0.0:.4f}'

        )



        self.scale_var.set(text)



    def clear_input(self):

        self.actual_distance_cm_var.set('')

        self.actual_angle_deg_var.set('')

        self.notes_var.set('')



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

    node = MotorCalibGuiNode()



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

