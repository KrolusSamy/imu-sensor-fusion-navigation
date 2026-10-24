import sys
import time
import csv
import json
from datetime import datetime
import serial
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QLabel, QCheckBox
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from collections import deque

def load_config(filename="config.json"):
    try:
        with open(filename, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Error loading config: {e}. Using defaults.")
        return {
            "COM_PORT": "COM7", "BAUD_RATE": 115200,
            "ACCEL_SENSITIVITY": 8192.0, "GYRO_SENSITIVITY": 65.5,
            "ACCEL_BIAS": [0.0, 0.0, 0.0], "ACCEL_SCALE": [1.0, 1.0, 1.0],
            "GYRO_BIAS": [0.0, 0.0, 0.0] 
        }

CONFIG = load_config()

class SerialReader(QThread):
    new_data = pyqtSignal(list)
    status_update = pyqtSignal(str, str)

    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.running = True

    def connect_serial(self):
        while self.running and self.serial_port is None:
            try:
                self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1)
                self.serial_port.reset_input_buffer()
                self.status_update.emit(f"Connected to {self.port}", "#55ff55")
            except serial.SerialException:
                self.status_update.emit(f"Waiting for {self.port}...", "#ff5555")
                time.sleep(1)

    def run(self):
        while self.running:
            if self.serial_port is None:
                self.connect_serial()
                continue

            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        parts = line.split(',')
                        
                        if len(parts) == 6:
                            try:
                                # ONLY parse raw 16-bit 2's complement values here
                                raw_a = [float(parts[0]), float(parts[1]), float(parts[2])]
                                raw_g = [float(parts[3]), float(parts[4]), float(parts[5])]
                                
                                # Emit raw LSBs to the GUI thread for processing
                                self.new_data.emit(raw_a + raw_g)
                            except ValueError:
                                pass 
            except (serial.SerialException, OSError):
                if self.serial_port:
                    self.serial_port.close()
                self.serial_port = None
            except Exception:
                pass 

    def stop(self):
        self.running = False
        if self.serial_port:
            self.serial_port.close()

class TelemetryDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("6DOF MPU6050 Telemetry")
        self.resize(1300, 800)
        
        self.is_light_mode = False
        self.is_paused = False
        
        # Calibration State Variables
        self.apply_calibration = True
        self.is_calibrating = False
        self.calibration_samples = []
        self.dynamic_gyro_bias = [0.0, 0.0, 0.0]
        
        self.setup_theme()

        self.central_widget = QWidget()
        self.main_layout = QVBoxLayout(self.central_widget)
        self.setCentralWidget(self.central_widget)

        # --- Control Panel UI ---
        self.control_layout = QHBoxLayout()
        self.status_label = QLabel("Initializing...")
        self.control_layout.addWidget(self.status_label)
        self.control_layout.addStretch()

        self.calib_btn = QPushButton("Calibrate Gyro (Hold Still)")
        self.calib_btn.clicked.connect(self.start_calibration)
        self.control_layout.addWidget(self.calib_btn)

        self.toggle_calib_btn = QPushButton("Viewing: Calibrated")
        self.toggle_calib_btn.clicked.connect(self.toggle_calibration_view)
        self.control_layout.addWidget(self.toggle_calib_btn)

        self.theme_btn = QPushButton("Light Mode")
        self.theme_btn.clicked.connect(self.toggle_theme)
        self.control_layout.addWidget(self.theme_btn)

        self.pause_btn = QPushButton("Pause Stream")
        self.pause_btn.clicked.connect(self.toggle_pause)
        self.control_layout.addWidget(self.pause_btn)

        self.record_btn = QPushButton("Start Recording")
        self.record_btn.clicked.connect(self.toggle_recording)
        self.control_layout.addWidget(self.record_btn)
        
        self.main_layout.addLayout(self.control_layout)

        # --- Data & Buffers ---
        self.plots = []
        self.curves = []
        self.data_buffers = []
        self.window_size = 300 
        self.colors = ['#ff5555', '#55ff55', '#5555ff']
        self.axis_labels = ['X', 'Y', 'Z']
        self.sensor_names = ['Accelerometer (g)', 'Gyroscope (°/s)']
        
        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None
        self.start_time = 0
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_timer_display)

        self.initialize_ui() 
        self.apply_theme_styles()

        self.serial_thread = SerialReader(CONFIG["COM_PORT"], CONFIG["BAUD_RATE"]) 
        self.serial_thread.new_data.connect(self.process_data)
        self.serial_thread.status_update.connect(self.update_status)
        self.serial_thread.start()

    def start_calibration(self):
        self.is_calibrating = True
        self.calibration_samples = []
        self.calib_btn.setText("Calibrating... (Gathering Data)")
        self.calib_btn.setStyleSheet("background-color: #ffaa00; color: black; font-weight: bold; padding: 8px 16px; border-radius: 4px;")

    def toggle_calibration_view(self):
        self.apply_calibration = not self.apply_calibration
        self.apply_theme_styles()

    def setup_theme(self):
        if self.is_light_mode:
            pg.setConfigOption('background', '#ffffff')
            pg.setConfigOption('foreground', '#000000')
        else:
            pg.setConfigOption('background', '#121212')
            pg.setConfigOption('foreground', '#d3d3d3')

    def apply_theme_styles(self):
        bg_color = "#ffffff" if self.is_light_mode else "#121212"
        fg_color = "#000000" if self.is_light_mode else "#ffffff"
        btn_bg = "#e0e0e0" if self.is_light_mode else "#333333"

        self.setStyleSheet(f"background-color: {bg_color}; color: {fg_color};")
        
        base_style = f"background-color: {btn_bg}; color: {fg_color}; padding: 8px 16px; font-weight: bold; border-radius: 4px;"
        
        if not self.is_calibrating:
            self.calib_btn.setStyleSheet(base_style)
            self.calib_btn.setText("Calibrate Gyro")
            
        if self.apply_calibration:
            self.toggle_calib_btn.setText("Viewing: Calibrated")
            self.toggle_calib_btn.setStyleSheet("background-color: #55ff55; color: black; padding: 8px 16px; font-weight: bold; border-radius: 4px;")
        else:
            self.toggle_calib_btn.setText("Viewing: Raw Drift")
            self.toggle_calib_btn.setStyleSheet("background-color: #ff5555; color: white; padding: 8px 16px; font-weight: bold; border-radius: 4px;")

        self.theme_btn.setStyleSheet(base_style)
        
        pause_bg = "#ffaa00" if self.is_paused else btn_bg
        self.pause_btn.setStyleSheet(f"background-color: {pause_bg}; color: {fg_color}; padding: 8px 16px; font-weight: bold; border-radius: 4px;")
        
        rec_bg = "#aa0000" if self.is_recording else btn_bg
        rec_fg = "#ffffff" if self.is_recording else fg_color
        self.record_btn.setStyleSheet(f"background-color: {rec_bg}; color: {rec_fg}; padding: 8px 16px; font-weight: bold; border-radius: 4px;")

    def toggle_theme(self):
        self.is_light_mode = not self.is_light_mode
        self.setup_theme()
        self.apply_theme_styles()
        
        bg = '#ffffff' if self.is_light_mode else '#121212'
        fg = '#000000' if self.is_light_mode else '#d3d3d3'
        
        for plot in self.plots:
            plot.setBackground(bg)
            plot.getAxis('left').setPen(fg)
            plot.getAxis('left').setTextPen(fg)
            plot.getAxis('bottom').setPen(fg)
            plot.getAxis('bottom').setTextPen(fg)

    def toggle_pause(self):
        self.is_paused = not self.is_paused
        self.pause_btn.setText("Resume Stream" if self.is_paused else "Pause Stream")
        self.apply_theme_styles()

    def update_status(self, message, color):
        self.status_label.setText(message)
        self.status_label.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 14px;")

    def toggle_recording(self):
        if not self.is_recording:
            filename = datetime.now().strftime("telemetry_%Y_%m_%d_%H_%M.csv")
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['Accel_X_g', 'Accel_Y_g', 'Accel_Z_g', 'Gyro_X_dps', 'Gyro_Y_dps', 'Gyro_Z_dps'])
            
            self.is_recording = True
            self.start_time = time.time()
            self.timer.start(100) 
        else:
            self.is_recording = False
            self.timer.stop()
            if self.csv_file:
                self.csv_file.close()
            self.record_btn.setText("Start Recording")
        
        self.apply_theme_styles()

    def update_timer_display(self):
        if self.is_recording:
            elapsed = int(time.time() - self.start_time)
            mins, secs = divmod(elapsed, 60)
            self.record_btn.setText(f"Stop Recording [{mins:02d}:{secs:02d}]")

    def initialize_ui(self):
        for i in range(2):
            sensor_container = QWidget()
            sensor_layout = QVBoxLayout(sensor_container)
            sensor_layout.setContentsMargins(0, 5, 0, 15)

            toggle_layout = QHBoxLayout()
            toggle_label = QLabel(f"Toggle {self.sensor_names[i]}:")
            toggle_layout.addWidget(toggle_label)

            plot = pg.PlotWidget()
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.addLegend(offset=(10, 10))
            
            if i == 0:
                plot.setYRange(-4.5, 4.5) 
            else:
                plot.setYRange(-500, 500)
            
            group_curves = []
            group_buffers = []
            
            for j in range(3):
                buffer = deque([0.0] * self.window_size, maxlen=self.window_size)
                curve = plot.plot(list(buffer), pen=pg.mkPen(self.colors[j], width=2), name=self.axis_labels[j])
                group_buffers.append(buffer)
                group_curves.append(curve)
                
                cb = QCheckBox(self.axis_labels[j])
                cb.setChecked(True)
                cb.setStyleSheet(f"color: {self.colors[j]}; font-weight: bold;")
                cb.toggled.connect(lambda checked, c=curve: c.setVisible(checked))
                toggle_layout.addWidget(cb)
            
            toggle_layout.addStretch()
            sensor_layout.addLayout(toggle_layout)
            sensor_layout.addWidget(plot)
            self.main_layout.addWidget(sensor_container)
            
            self.plots.append(plot)
            self.data_buffers.append(group_buffers)
            self.curves.append(group_curves)

    def process_data(self, data):
        # 1. Parse Raw LSBs sent from the SerialReader Thread
        raw_a = data[0:3]
        raw_g = data[3:6]

        # 2. Handle Dynamic Auto-Calibration 
        if self.is_calibrating:
            self.calibration_samples.append(raw_g)
            # Collect 100 samples (2 seconds at 50Hz) to find the average zero-rate offset
            if len(self.calibration_samples) >= 100:
                # Calculate average for X, Y, Z
                self.dynamic_gyro_bias = [
                    sum(col) / len(col) for col in zip(*self.calibration_samples)
                ]
                self.is_calibrating = False
                self.apply_theme_styles() 

        # 3. Toggle Calibration States for BOTH sensors
        if self.apply_calibration:
            active_accel_bias = CONFIG["ACCEL_BIAS"]
            active_accel_scale = CONFIG["ACCEL_SCALE"]
            active_gyro_bias = self.dynamic_gyro_bias
        else:
            active_accel_bias = [0.0, 0.0, 0.0]
            active_accel_scale = [1.0, 1.0, 1.0]
            active_gyro_bias = [0.0, 0.0, 0.0]

        # 4. Apply Final Math
        cal_a = [
            ((raw_a[i] - active_accel_bias[i]) / CONFIG["ACCEL_SENSITIVITY"]) * active_accel_scale[i]
            for i in range(3)
        ]

        cal_g = [
            (raw_g[i] - active_gyro_bias[i]) / CONFIG["GYRO_SENSITIVITY"]
            for i in range(3)
        ]

        final_plot_data = cal_a + cal_g

        if self.is_recording and self.csv_writer:
            self.csv_writer.writerow(final_plot_data)

        if self.is_paused:
            return

        text_color = "#000000" if self.is_light_mode else "#ffffff"
        idx = 0
        for i, plot in enumerate(self.plots):
            title_html = f"<b style='color:{text_color}'>{self.sensor_names[i]}</b> &nbsp;&nbsp;&nbsp;"
            for j in range(3):
                val = final_plot_data[idx]
                self.data_buffers[i][j].append(val)
                self.curves[i][j].setData(self.data_buffers[i][j])
                title_html += f"<span style='color:{self.colors[j]}'>{self.axis_labels[j]}: {val:6.2f}</span> &nbsp;&nbsp;"
                idx += 1
                
            plot.setTitle(title_html)

    def closeEvent(self, event):
        if self.is_recording and self.csv_file:
            self.csv_file.close()
        self.serial_thread.stop()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = TelemetryDashboard()
    window.show()
    sys.exit(app.exec_())