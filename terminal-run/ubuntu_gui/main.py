#!/usr/bin/env python3

import sys
import subprocess
import os
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGridLayout, QFrame
)
from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QFont, QColor, QPalette

# --- CẤU HÌNH GIAO DIỆN ---
COLOR_BG = "#0B0C10"
COLOR_PANEL = "#1F2833"
COLOR_NEON_CYAN = "#66FCF1"
COLOR_NEON_GREEN = "#45A29E"
COLOR_NEON_RED = "#FF0055"
COLOR_TEXT = "#C5C6C7"

class NeonButton(QPushButton):
    def __init__(self, text, color_type="cyan"):
        super().__init__(text)
        self.color_type = color_type
        self.setCursor(Qt.PointingHandCursor)
        self.setFont(QFont("Monospace", 11, QFont.Bold))
        self.update_style()

    def update_style(self):
        if self.color_type == "cyan":
            color = COLOR_NEON_CYAN
        elif self.color_type == "red":
            color = COLOR_NEON_RED
        else:
            color = COLOR_NEON_GREEN

        self.setStyleSheet(f"""
            QPushButton {{
                background-color: transparent;
                border: 2px solid {color};
                color: {color};
                border-radius: 8px;
                padding: 12px 15px;
            }}
            QPushButton:hover {{
                background-color: {color};
                color: {COLOR_BG};
            }}
            QPushButton:pressed {{
                background-color: {COLOR_TEXT};
                border: 2px solid {COLOR_TEXT};
            }}
        """)

class AVSControlPanel(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AVS Robot Control Center")
        self.setMinimumSize(800, 600)
        self.setStyleSheet(f"background-color: {COLOR_BG};")
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(30, 30, 30, 30)
        main_layout.setSpacing(20)

        # Title
        title = QLabel("AVS CONTROL CENTER")
        title.setFont(QFont("Monospace", 24, QFont.Bold))
        title.setStyleSheet(f"color: {COLOR_NEON_CYAN}; letter-spacing: 2px;")
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)
        
        subtitle = QLabel("ROBOTIC AUTONOMY & VISION SYSTEM")
        subtitle.setFont(QFont("Monospace", 10))
        subtitle.setStyleSheet(f"color: {COLOR_TEXT}; letter-spacing: 4px;")
        subtitle.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(subtitle)

        main_layout.addSpacing(20)

        # --- GRID LAYOUT CHO CÁC MODULE ---
        grid = QGridLayout()
        grid.setSpacing(20)
        main_layout.addLayout(grid)

        # 1. RVIZ CONTROL
        rviz_panel = self.create_panel("RVIZ VISUALIZATION")
        rviz_layout = rviz_panel.inner_layout
        btn_start_rviz = NeonButton("▶ START RVIZ", "cyan")
        btn_stop_rviz = NeonButton("■ STOP RVIZ", "red")
        
        btn_start_rviz.clicked.connect(self.start_rviz)
        btn_stop_rviz.clicked.connect(self.stop_rviz)
        
        rviz_layout.addWidget(btn_start_rviz)
        rviz_layout.addWidget(btn_stop_rviz)
        grid.addWidget(rviz_panel, 0, 0)

        # 2. TOOLS
        tools_panel = self.create_panel("SYSTEM TOOLS")
        tools_layout = tools_panel.inner_layout
        btn_rqt = NeonButton("⎈ OPEN RQT GRAPH", "green")
        btn_web = NeonButton("🌐 OPEN WEB DASHBOARD", "green")
        
        btn_rqt.clicked.connect(self.open_rqt)
        btn_web.clicked.connect(self.open_web)
        
        tools_layout.addWidget(btn_rqt)
        tools_layout.addWidget(btn_web)
        grid.addWidget(tools_panel, 0, 1)

        # 3. LOGGERS (GRID BÊN DƯỚI)
        logger_panel = self.create_panel("CONTROLLER PLOT LOGGERS")
        logger_grid = QGridLayout()
        logger_grid.setSpacing(15)
        logger_panel.inner_layout.addLayout(logger_grid)

        loggers = [
            ("PD Controller", "PD_controller_logger.py"),
            ("PD Backstepping", "PD_backstepping_logger.py"),
            ("Cascade Controller", "cascade_controller_logger.py"),
            ("Hybrid Controller", "hybrid_controller_logger.py")
        ]

        row = 0
        col = 0
        for name, script in loggers:
            lbl = QLabel(name)
            lbl.setFont(QFont("Monospace", 11, QFont.Bold))
            lbl.setStyleSheet(f"color: {COLOR_TEXT};")
            lbl.setAlignment(Qt.AlignCenter)
            
            h_layout = QHBoxLayout()
            btn_start = NeonButton("▶ START", "cyan")
            btn_stop = NeonButton("■ STOP", "red")
            
            # Use lambda with default arguments to capture the current script correctly
            btn_start.clicked.connect(lambda checked, s=script: self.start_logger(s))
            btn_stop.clicked.connect(lambda checked, s=script: self.stop_logger(s))
            
            h_layout.addWidget(btn_start)
            h_layout.addWidget(btn_stop)
            
            v_layout = QVBoxLayout()
            v_layout.addWidget(lbl)
            v_layout.addLayout(h_layout)
            
            logger_grid.addLayout(v_layout, row, col)
            col += 1
            if col > 1:
                col = 0
                row += 1

        grid.addWidget(logger_panel, 1, 0, 1, 2)
        main_layout.addStretch()

        # Footer
        footer = QLabel("Developed for SimpleSysIDV")
        footer.setFont(QFont("Monospace", 9))
        footer.setStyleSheet("color: #444;")
        footer.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(footer)

    def create_panel(self, title_text):
        panel = QFrame()
        panel.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_PANEL};
                border-radius: 12px;
                border: 1px solid #333;
            }}
        """)
        layout = QVBoxLayout(panel)
        title = QLabel(title_text)
        title.setFont(QFont("Monospace", 12, QFont.Bold))
        title.setStyleSheet(f"color: {COLOR_NEON_CYAN}; border: none;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        layout.addSpacing(10)
        
        # Inner container for actual content to keep padding nice
        inner = QWidget()
        inner.setStyleSheet("background-color: transparent; border: none;")
        layout.addWidget(inner)
        panel.inner_layout = QVBoxLayout(inner)
        panel.inner_layout.setContentsMargins(0,0,0,0)
        
        return panel

    # --- HÀM THỰC THI LỆNH ---
    
    def run_cmd(self, cmd):
        print(f"Executing: {cmd}")
        subprocess.Popen(cmd, shell=True, executable="/bin/bash")

    def start_rviz(self):
        cmd = 'cd ~/SimpleRobot/terminal-run/Rviz && bash start_rviz_car.sh'
        self.run_cmd(f'gnome-terminal --title="RViz Launch" -- bash -c "{cmd}; exec bash"')

    def stop_rviz(self):
        # Lệnh stop giống hệt trong start_rviz_car.sh
        cmd = (
            "pkill -f robot_state_publisher; "
            "pkill -f joint_state_publisher; "
            "pkill -f odom_raw_to_tf.py; "
            "pkill -f telemetry_to_rviz_markers.py; "
            "pkill -f rviz2"
        )
        self.run_cmd(cmd)

    def start_logger(self, script_name):
        cmd = (
            "source /opt/ros/humble/setup.bash && "
            "export ROS_DOMAIN_ID=20 && "
            "export ROS_LOCALHOST_ONLY=0 && "
            f"cd ~/SimpleRobot/terminal-run/plot_logger && "
            f"python3 {script_name}"
        )
        self.run_cmd(f'gnome-terminal --title="{script_name}" -- bash -c "{cmd}; exec bash"')

    def stop_logger(self, script_name):
        self.run_cmd(f'pkill -f {script_name}')

    def open_rqt(self):
        cmd = (
            "source /opt/ros/humble/setup.bash && "
            "export ROS_DOMAIN_ID=20 && "
            "export ROS_LOCALHOST_ONLY=0 && "
            "rqt_graph"
        )
        self.run_cmd(f'gnome-terminal --title="rqt_graph" -- bash -c "{cmd}; exec bash"')

    def open_web(self):
        # Open local dashboard on chrome
        self.run_cmd('google-chrome http://raspi5.local:8000/ || chromium-browser http://raspi5.local:8000/')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    # Force style for consistency
    app.setStyle("Fusion")
    
    window = AVSControlPanel()
    window.show()
    sys.exit(app.exec_())
