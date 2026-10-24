import sys
import time
import csv
from datetime import datetime
import serial
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QLabel, QCheckBox
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from collections import deque

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
                self.status_update.emit(f"Connected to {self.port}", "#55ff55")
            except serial.SerialException:
                self.status_update.emit(f"Disconnected. Waiting for {self.port}...", "#ff5555")
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
                        data = [float(val) for val in line.split(',')]
                        self.new_data.emit(data)
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
        self.setWindowTitle("Academy Embedded Telemetry")
        self.resize(1200, 800)
        
        pg.setConfigOption('background', '#121212')
        pg.setConfigOption('foreground', '#d3d3d3')

        self.central_widget = QWidget()
        self.main_layout = QVBoxLayout(self.central_widget)
        self.setCentralWidget(self.central_widget)

        # --- Control Bar UI ---
        self.control_layout = QHBoxLayout()
        
        self.status_label = QLabel("Initializing...")
        self.status_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.control_layout.addWidget(self.status_label)
        
        self.control_layout.addStretch()

        self.record_btn = QPushButton("Start Recording")
        self.record_btn.setStyleSheet("background-color: #333; color: white; padding: 8px 16px; font-weight: bold; border-radius: 4px;")
        self.record_btn.clicked.connect(self.toggle_recording)
        self.control_layout.addWidget(self.record_btn)
        
        self.main_layout.addLayout(self.control_layout)

        # --- Graphing Variables ---
        self.plots = []
        self.curves = []
        self.data_buffers = []
        self.window_size = 500 
        
        self.colors = ['#ff5555', '#55ff55', '#5555ff']
        self.axis_labels = ['X', 'Y', 'Z']
        self.sensor_names = ['Accelerometer', 'Gyroscope', 'Magnetometer', 'Barometer']

        # --- Data Logging Variables ---
        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None
        self.start_time = 0
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_timer_display)

        # Initialize Serial Thread
        self.serial_thread = SerialReader('COM7', 115200) 
        self.serial_thread.new_data.connect(self.process_data)
        self.serial_thread.status_update.connect(self.update_status)
        self.serial_thread.start()

        self.is_initialized = False

    def update_status(self, message, color):
        self.status_label.setText(message)
        self.status_label.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 14px;")

    def toggle_recording(self):
        if not self.is_recording:
            filename = datetime.now().strftime("telemetry_%Y_%m_%d_%H_%M.csv")
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            
            self.is_recording = True
            self.start_time = time.time()
            self.timer.start(100) 
            
            self.record_btn.setStyleSheet("background-color: #aa0000; color: white; padding: 8px 16px; font-weight: bold; border-radius: 4px;")
        else:
            self.is_recording = False
            self.timer.stop()
            if self.csv_file:
                self.csv_file.close()
                
            self.record_btn.setText("Start Recording")
            self.record_btn.setStyleSheet("background-color: #333; color: white; padding: 8px 16px; font-weight: bold; border-radius: 4px;")

    def update_timer_display(self):
        if self.is_recording:
            elapsed = int(time.time() - self.start_time)
            mins, secs = divmod(elapsed, 60)
            self.record_btn.setText(f"Stop Recording [{mins:02d}:{secs:02d}]")

    def initialize_ui(self, data_length):
        num_plots = (data_length + 2) // 3  
        
        for i in range(num_plots):
            # Container for the toggle layout and the plot itself
            sensor_container = QWidget()
            sensor_layout = QVBoxLayout(sensor_container)
            sensor_layout.setContentsMargins(0, 5, 0, 15)

            # Interactive Checkbox Row
            toggle_layout = QHBoxLayout()
            toggle_label = QLabel(f"Hide/Show {self.sensor_names[i]} Axes:")
            toggle_label.setStyleSheet("color: #000000; font-weight: bold; font-size: 12px;")
            toggle_layout.addWidget(toggle_label)

            plot = pg.PlotWidget()
            plot.showGrid(x=True, y=True, alpha=0.2)
            plot.addLegend(offset=(10, 10))
            
            group_curves = []
            group_buffers = []
            
            axes_in_group = min(3, data_length - (i * 3))
            for j in range(axes_in_group):
                buffer = deque([0.0] * self.window_size, maxlen=self.window_size)
                curve = plot.plot(list(buffer), pen=pg.mkPen(self.colors[j], width=2), name=self.axis_labels[j])
                group_buffers.append(buffer)
                group_curves.append(curve)
                
                # Checkbox generation
                cb = QCheckBox(self.axis_labels[j])
                cb.setChecked(True)
                cb.setStyleSheet(f"color: {self.colors[j]}; font-weight: bold; font-size: 12px; margin-left: 10px;")
                # Lambda captures the specific curve reference to toggle visibility
                cb.toggled.connect(lambda checked, c=curve: c.setVisible(checked))
                toggle_layout.addWidget(cb)
            
            toggle_layout.addStretch()
            
            sensor_layout.addLayout(toggle_layout)
            sensor_layout.addWidget(plot)
            self.main_layout.addWidget(sensor_container)
            
            self.plots.append(plot)
            self.data_buffers.append(group_buffers)
            self.curves.append(group_curves)
            
        self.is_initialized = True

    def process_data(self, data):
        if not self.is_initialized:
            self.initialize_ui(len(data))
            
        if len(data) != sum(len(group) for group in self.data_buffers):
            return 

        if self.is_recording and self.csv_writer:
            self.csv_writer.writerow(data)

        idx = 0
        for i, plot in enumerate(self.plots):
            title_html = f"<b>{self.sensor_names[i]}</b> &nbsp;&nbsp;&nbsp;"
            axes_in_group = len(self.data_buffers[i])
            
            for j in range(axes_in_group):
                val = data[idx]
                self.data_buffers[i][j].append(val)
                self.curves[i][j].setData(self.data_buffers[i][j])
                title_html += f"<span style='color:{self.colors[j]}'>{self.axis_labels[j]}: {val:7.2f}</span> &nbsp;&nbsp;"
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