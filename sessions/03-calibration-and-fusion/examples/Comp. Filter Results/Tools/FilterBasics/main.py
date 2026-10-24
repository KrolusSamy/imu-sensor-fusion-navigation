import sys
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QCheckBox, QPushButton
from PyQt5.QtCore import QTimer, Qt

# --- Synthetic Data Generation ---
FPS = 50
DURATION = 10 
t = np.linspace(0, DURATION, FPS * DURATION)

# 1. Low Frequency Component (e.g., intentional slow arm movement at 0.3 Hz)
low_freq_signal = 40.0 * np.sin(2 * np.pi * 0.3 * t)

# 2. High Frequency Component (e.g., engine vibration at 15 Hz + random noise)
high_freq_signal = 10.0 * np.sin(2 * np.pi * 15.0 * t) + np.random.normal(0, 2, len(t))

# 3. The Composite Signal (What the sensor actually sees)
composite_signal = low_freq_signal + high_freq_signal

class FrequencyLesson(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Signal Decomposition: High vs. Low Frequency")
        self.resize(1100, 750)
        
        self.is_light_mode = False
        self.is_paused = False
        self.setup_theme()

        self.central = QWidget()
        self.setCentralWidget(self.central)
        self.layout = QVBoxLayout(self.central)

        # --- Control Panel ---
        self.controls = QHBoxLayout()
        
        self.theme_btn = QPushButton("Light Mode")
        self.theme_btn.clicked.connect(self.toggle_theme)
        self.controls.addWidget(self.theme_btn)

        self.pause_btn = QPushButton("Pause Stream")
        self.pause_btn.clicked.connect(self.toggle_pause)
        self.controls.addWidget(self.pause_btn)

        self.show_comp = QCheckBox("1. Composite (Raw Sensor)")
        self.show_low = QCheckBox("2. Pure Low Freq (Target)")
        self.show_high = QCheckBox("3. Pure High Freq (Noise)")
        self.show_lpf = QCheckBox("4. Filtered: LPF Output")
        self.show_hpf = QCheckBox("5. Filtered: HPF Output")
        
        for cb in [self.show_comp, self.show_low, self.show_high, self.show_lpf, self.show_hpf]:
            cb.setStyleSheet("font-weight: bold; font-size: 13px;")
            self.controls.addWidget(cb)
            
        self.show_comp.setChecked(True)
        
        self.layout.addLayout(self.controls)

        # --- Plot Setup ---
        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setYRange(-60, 60)
        self.plot.addLegend(offset=(10, 10))
        self.layout.addWidget(self.plot)

        # Curves
        self.curve_comp = self.plot.plot(pen=pg.mkPen('#888888', width=2, alpha=150), name="Composite Signal")
        self.curve_low = self.plot.plot(pen=pg.mkPen('#ffaa00', width=2, style=Qt.DashLine), name="Pure Low Freq")
        self.curve_high = self.plot.plot(pen=pg.mkPen('#ff5555', width=1, alpha=200), name="Pure High Freq")
        self.curve_lpf = self.plot.plot(pen=pg.mkPen('#55ff55', width=3), name="LPF (Passes Slow, Blocks Fast)")
        self.curve_hpf = self.plot.plot(pen=pg.mkPen('#5555ff', width=2), name="HPF (Passes Fast, Blocks Slow)")

        self.idx = 0
        
        # Filter States
        self.lpf_val = 0.0
        self.hpf_val = 0.0
        self.prev_raw = 0.0
        
        # Buffers for drawing
        self.window = 250
        self.buf_comp = np.zeros(self.window)
        self.buf_low = np.zeros(self.window)
        self.buf_high = np.zeros(self.window)
        self.buf_lpf = np.zeros(self.window)
        self.buf_hpf = np.zeros(self.window)

        self.apply_theme_styles()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(20) # 50Hz

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
        
        btn_style = f"background-color: {btn_bg}; color: {fg_color}; padding: 6px 12px; font-weight: bold; border-radius: 4px;"
        
        self.theme_btn.setStyleSheet(btn_style)
        self.theme_btn.setText("Dark Mode" if self.is_light_mode else "Light Mode")
        
        if self.is_paused:
            self.pause_btn.setStyleSheet("background-color: #ffaa00; color: black; padding: 6px 12px; font-weight: bold; border-radius: 4px;")
        else:
            self.pause_btn.setStyleSheet(btn_style)

        # Update pyqtgraph elements dynamically
        bg = '#ffffff' if self.is_light_mode else '#121212'
        fg = '#000000' if self.is_light_mode else '#d3d3d3'
        
        self.plot.setBackground(bg)
        self.plot.getAxis('left').setPen(fg)
        self.plot.getAxis('left').setTextPen(fg)
        self.plot.getAxis('bottom').setPen(fg)
        self.plot.getAxis('bottom').setTextPen(fg)

    def toggle_theme(self):
        self.is_light_mode = not self.is_light_mode
        self.setup_theme()
        self.apply_theme_styles()

    def toggle_pause(self):
        self.is_paused = not self.is_paused
        self.pause_btn.setText("Resume Stream" if self.is_paused else "Pause Stream")
        self.apply_theme_styles()

    def update_plot(self):
        if self.is_paused:
            return

        if self.idx >= len(t):
            self.idx = 0
            self.lpf_val = 0.0
            self.hpf_val = 0.0
            self.prev_raw = composite_signal[0]
            
        raw = composite_signal[self.idx]
        pure_l = low_freq_signal[self.idx]
        pure_h = high_freq_signal[self.idx]

        # First-Order Low-Pass Filter
        alpha_lpf = 0.1 
        self.lpf_val = (alpha_lpf * raw) + ((1.0 - alpha_lpf) * self.lpf_val)

        # First-Order High-Pass Filter 
        alpha_hpf = 0.9 
        self.hpf_val = alpha_hpf * (self.hpf_val + raw - self.prev_raw)
        self.prev_raw = raw

        # Shift buffers
        for buf in [self.buf_comp, self.buf_low, self.buf_high, self.buf_lpf, self.buf_hpf]:
            buf[:-1] = buf[1:]

        self.buf_comp[-1] = raw
        self.buf_low[-1] = pure_l
        self.buf_high[-1] = pure_h
        self.buf_lpf[-1] = self.lpf_val
        self.buf_hpf[-1] = self.hpf_val

        # Update visuals
        self.curve_comp.setData(self.buf_comp)
        self.curve_low.setData(self.buf_low)
        self.curve_high.setData(self.buf_high)
        self.curve_lpf.setData(self.buf_lpf)
        self.curve_hpf.setData(self.buf_hpf)

        self.curve_comp.setVisible(self.show_comp.isChecked())
        self.curve_low.setVisible(self.show_low.isChecked())
        self.curve_high.setVisible(self.show_high.isChecked())
        self.curve_lpf.setVisible(self.show_lpf.isChecked())
        self.curve_hpf.setVisible(self.show_hpf.isChecked())

        self.idx += 1

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = FrequencyLesson()
    window.show()
    sys.exit(app.exec_())