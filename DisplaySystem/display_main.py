import sys
import struct
import time
import random
import math
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QTextEdit, QPushButton, QGridLayout,
                             QGroupBox, QTableWidget, QTableWidgetItem, QHeaderView)
from PyQt6.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt6.QtGui import QVector3D
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np

# Dark Mode Theme
DARK_STYLESHEET = """
QMainWindow, QWidget {
    background-color: #2b2b2b;
    color: #e0e0e0;
    font-family: 'Segoe UI', sans-serif;
}
QGroupBox {
    border: 2px solid #3e3e3e;
    border-radius: 6px;
    margin-top: 1.5em;
    font-weight: bold;
    color: #ffd700; /* Gold accent for titles */
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 5px;
}
QPushButton {
    background-color: #3e3e3e;
    border: 1px solid #555;
    border-radius: 4px;
    padding: 5px;
    color: white;
}
QPushButton:hover {
    background-color: #4e4e4e;
    border: 1px solid #777;
}
QPushButton:pressed {
    background-color: #222;
}
QTableWidget {
    background-color: #1e1e1e;
    gridline-color: #333;
    color: #e0e0e0;
    border: 1px solid #3e3e3e;
}
QHeaderView::section {
    background-color: #333;
    color: white;
    padding: 4px;
    border: 1px solid #444;
}
QTextEdit {
    background-color: #1e1e1e;
    color: #00ff00; /* Console like text */
    border: 1px solid #3e3e3e;
    font-family: 'Consolas', monospace;
}
QLabel {
    color: #e0e0e0;
}
"""

# Data Structure Definition
class TelemetryStruct:
    FMT = '<h I h 3h 3h h 3h h i i h H H I B 5x'
    SIZE = struct.calcsize(FMT)

    @staticmethod
    def pack(data):
        return struct.pack(TelemetryStruct.FMT, *data)

    @staticmethod
    def unpack(data):
        return struct.unpack(TelemetryStruct.FMT, data)

class DataGenerator(QThread):
    new_data = pyqtSignal(object)  # Emits the unpacked tuple
    log_message = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.running = True
        # Initial simulation state
        self.lat = 29.56
        self.lon = -95.09 # Clear Lake area approximate
        self.alt = 0.0 # Start at ground
        self.max_alt = 0.0
        
        # Random Walk State
        self.bmp_temp = 25.0
        self.pressure = 1013.0
        self.imu_temp = 30.0
        self.extra_temp = 20.0
        self.accel = [0, 0, 9.8]
        self.gyro = [0, 0, 0]
        self.mag = [0, 0, 0]
        self.gps_speed = 0.0
        self.gps_angle = 0.0
        
        # State Machine
        self.state = 0 # 0: Idle
        self.state_timer = 0
        self.flight_start_time = 0

    def run(self):
        start_time = time.time()
        print("Data Generator Started")
        
        while self.running:
            current_time = time.time()
            elapsed = current_time - start_time
            
            # State Machine Logic
            flight_time = 0
            
            if self.state == 0: # Idle
                self.alt = 50.0 # Pad elevation
                if elapsed > 5.0:
                    self.state = 1
                    self.flight_start_time = current_time
                    self.log_message.emit("LAUNCH DETECTED!")
            
            elif self.state == 1: # Flight (Ascent)
                flight_time = current_time - self.flight_start_time
                self.alt += random.uniform(10, 20) # Go up fast
                self.accel[2] = 20.0 + random.uniform(-2, 2)
                
                if self.alt > 1500:
                    self.state = 2
                    self.max_alt = self.alt
                    self.log_message.emit("APOGEE DETECTED!")
                    
            elif self.state == 2: # Apogee
                flight_time = current_time - self.flight_start_time
                self.alt += random.uniform(-2, 2) # Hover
                self.accel[2] = 0.0 + random.uniform(-1, 1) # Freefall start
                self.state_timer += 1
                if self.state_timer > 3: # Short apogee
                    self.state = 3
                    self.log_message.emit("DESCENT DETECTED!")
            
            elif self.state == 3: # Descent
                flight_time = current_time - self.flight_start_time
                self.alt -= random.uniform(10, 15)
                self.accel[2] = 9.8 + random.uniform(-5, 5) # Drogue/Main
                
                if self.alt <= 50.0:
                    self.alt = 50.0
                    self.state = 4
                    self.log_message.emit("LANDING DETECTED!")
            
            elif self.state == 4: # Landed
                flight_time = current_time - self.flight_start_time # Keep final time
                pass
            
            # Helper to walk
            def walk(val, delta, min_val, max_val):
                val += random.uniform(-delta, delta)
                return max(min_val, min(val, max_val))

            self.bmp_temp = walk(self.bmp_temp, 0.5, 10.0, 40.0)
            self.pressure = walk(self.pressure, 1.0, 900.0, 1100.0)
            
            # Simulate consistent Wind Drift (Arc) instead of random walk
            # ~1m/s drift in X, 0.5m/s in Y
            if self.state in [1, 2, 3]: # In flight
                self.lat += 0.000005 
                self.lon += 0.00001
            else:
                # Random jitter on pad/landed
                self.lat += random.uniform(-0.000001, 0.000001)
                self.lon += random.uniform(-0.000001, 0.000001)
            self.gps_speed = walk(self.gps_speed, 5.0, 0.0, 200.0)
            self.gps_angle = walk(self.gps_angle, 10.0, 0.0, 360.0)
            
            bmp_alt = int(self.alt * 10)
            
            self.accel[0] = walk(self.accel[0], 0.5, -5.0, 5.0)
            self.accel[1] = walk(self.accel[1], 0.5, -5.0, 5.0)
            
            self.gyro[0] = walk(self.gyro[0], 5.0, -300.0, 300.0)
            self.gyro[1] = walk(self.gyro[1], 5.0, -300.0, 300.0)
            self.gyro[2] = walk(self.gyro[2], 5.0, -300.0, 300.0)
            
            self.imu_temp = walk(self.imu_temp, 0.2, 20.0, 50.0)
            self.extra_temp = walk(self.extra_temp, 0.5, 0.0, 40.0)
            
            self.mag[0] = walk(self.mag[0], 1.0, -60.0, 60.0)
            self.mag[1] = walk(self.mag[1], 1.0, -60.0, 60.0)
            self.mag[2] = walk(self.mag[2], 1.0, -60.0, 60.0)
            
            timestamp = int(time.time())
            
            # Flatten tuple for pack
            packet_data = (
                int(self.bmp_temp * 100), int(self.pressure * 100), bmp_alt,
                int(self.accel[0]*100), int(self.accel[1]*100), int(self.accel[2]*100),
                int(self.gyro[0]*100), int(self.gyro[1]*100), int(self.gyro[2]*100),
                int(self.imu_temp * 100), 
                int(self.mag[0]*100), int(self.mag[1]*100), int(self.mag[2]*100), 
                int(self.extra_temp * 100),
                int(self.lat * 1e7), int(self.lon * 1e7), int(self.alt * 10), 
                int(self.gps_speed * 100), int(self.gps_angle * 100),
                timestamp, self.state
            )
            
            # Pack to binary (simulation of radio)
            binary_packet = TelemetryStruct.pack(packet_data)
            
            # Use unpack to verify and emit
            unpacked_data = TelemetryStruct.unpack(binary_packet)
            
            # Append flight_time manually for the GUI
            final_data = unpacked_data + (flight_time,)
            
            self.new_data.emit(final_data)
            
            time.sleep(1.0) 

    def stop(self):
        self.running = False
        self.wait()

class ScrollingPlot(QWidget):
    def __init__(self, title, xlabel, ylabel, y_range=None, threshold=None):
        super().__init__()
        self.layout = QVBoxLayout(self)
        self.plot_widget = pg.PlotWidget(title=title)
        self.plot_widget.setLabel('left', ylabel)
        self.plot_widget.setLabel('bottom', xlabel)
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.addLegend() # Add Legend
        if y_range:
            self.plot_widget.setYRange(*y_range)
        
        self.layout.addWidget(self.plot_widget)
        
        self.data_buffer_size = 10000 # Increased buffer
        self.curves = []
        self.data = []

        # Add infinite line for threshold
        if threshold is not None:
            line = pg.InfiniteLine(pos=threshold, angle=0, pen=pg.mkPen('r', width=1, style=Qt.PenStyle.DashLine), name='Threshold')
            self.plot_widget.addItem(line)
            
    def add_curve(self, name, color):
        # Increased line width to 2 as requested
        curve = self.plot_widget.plot(pen=pg.mkPen(color, width=2), name=name)
        self.curves.append(curve)
        self.data.append(np.zeros(self.data_buffer_size))
        
    def update_data(self, new_values):
        for i, val in enumerate(new_values):
            if i < len(self.data):
                self.data[i] = np.roll(self.data[i], -1)
                self.data[i][-1] = val
                self.curves[i].setData(self.data[i])

    def zoom_to_last(self, n_samples):
        if n_samples > 0:
            self.plot_widget.setXRange(self.data_buffer_size - n_samples, self.data_buffer_size)
        else:
             self.plot_widget.setXRange(self.data_buffer_size - 100, self.data_buffer_size)

    def reset(self):
        for i in range(len(self.data)):
            self.data[i] = np.zeros(self.data_buffer_size)
            self.curves[i].setData(self.data[i])

class GPSDisplayWidget(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setSpacing(5)
        
        title = QLabel("GPS COORDINATES")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("font-weight: bold; font-size: 14px; color: #e0e0e0;")
        layout.addWidget(title)
        
        self.val_label = QLabel("Lat: --\nLon: -")
        self.val_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.val_label.setStyleSheet("font-size: 24px; font-weight: bold; color: gold; border: 2px solid #555; border-radius: 5px; padding: 10px; background-color: #1e1e1e;")
        layout.addWidget(self.val_label)

    def update_gps(self, lat, lon, alt):
        self.val_label.setText(f"{lat:.6f}\n{lon:.6f}\nAlt: {alt:.1f}m")

class GroundTrackWidget(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        self.plot = pg.PlotWidget(title="Ground Track (2D)")
        self.plot.setLabel('left', "Latitude")
        self.plot.setLabel('bottom', "Longitude")
        self.plot.showGrid(x=True, y=True)
        self.plot.setAspectLocked(True)
        self.plot.addLegend()
        layout.addWidget(self.plot)
        
        self.path_curve = self.plot.plot(pen=pg.mkPen('c', width=3), name='Path')
        
        self.start_scatter = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(255, 0, 0, 255), name='Start')
        self.plot.addItem(self.start_scatter)
        
        self.current_scatter = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(0, 255, 0, 255), name='Current')
        self.plot.addItem(self.current_scatter)
        
        self.lats = []
        self.lons = []
        
    def update_pos(self, lat, lon):
        if not self.lats:
            self.start_scatter.setData([lon], [lat])
            
        self.lats.append(lat)
        self.lons.append(lon)
        self.path_curve.setData(self.lons, self.lats)
        self.current_scatter.setData([lon], [lat])

    def reset(self):
        self.lats = []
        self.lons = []
        self.path_curve.setData([], [])
        self.start_scatter.setData([], [])
        self.current_scatter.setData([], [])

# 3D Visualization Widgets
class FlightPathWidget(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0,0,0,0)
        layout.setSpacing(0)
        
        # Toolbar
        btn_container = QWidget()
        btn_container.setFixedHeight(85) 
        btn_layout = QHBoxLayout(btn_container)
        btn_layout.setContentsMargins(2, 0, 2, 2)
        btn_layout.setSpacing(2)
        
        # 1. Legend - Single Line
        legend = QLabel("Z:500m | Grn:Asc | Red:Dsc")
        legend.setStyleSheet("color: white; font-weight: bold; background-color: rgba(0,0,0,100); padding: 1px; font-size: 10px;")
        legend.setFixedHeight(15)
        btn_layout.addWidget(legend)
        
        # 2. Plus Shape Look Buttons
        pad_layout = QGridLayout()
        pad_layout.setSpacing(0)
        pad_layout.setContentsMargins(0,0,0,0)
        
        btn_style = "font-size: 9px; padding: 0px; margin: 0px;"
        
        btn_n = QPushButton("N")
        btn_n.setFixedSize(20, 20)
        btn_n.setStyleSheet(btn_style)
        btn_n.clicked.connect(lambda: self.set_camera(-90, 0))
        pad_layout.addWidget(btn_n, 0, 1)
        
        btn_w = QPushButton("W")
        btn_w.setFixedSize(20, 20)
        btn_w.setStyleSheet(btn_style)
        btn_w.clicked.connect(lambda: self.set_camera(0, 0))
        pad_layout.addWidget(btn_w, 1, 0)
        
        btn_e = QPushButton("E")
        btn_e.setFixedSize(20, 20)
        btn_e.setStyleSheet(btn_style)
        btn_e.clicked.connect(lambda: self.set_camera(180, 0))
        pad_layout.addWidget(btn_e, 1, 2)
        
        btn_s = QPushButton("S")
        btn_s.setFixedSize(20, 20)
        btn_s.setStyleSheet(btn_style)
        btn_s.clicked.connect(lambda: self.set_camera(90, 0))
        pad_layout.addWidget(btn_s, 2, 1)
        
        btn_layout.addLayout(pad_layout)
        
        # 3. Stacked Move Buttons
        move_layout = QVBoxLayout()
        move_layout.setSpacing(1)
        move_layout.setContentsMargins(0,0,0,0)
        
        btn_up = QPushButton("Up")
        btn_up.setFixedSize(30, 20)
        btn_up.setStyleSheet(btn_style)
        btn_up.clicked.connect(lambda: self.move_camera_z(200))
        move_layout.addWidget(btn_up)
        
        btn_down = QPushButton("Dn")
        btn_down.setFixedSize(30, 20)
        btn_down.setStyleSheet(btn_style)
        btn_down.clicked.connect(lambda: self.move_camera_z(-200))
        move_layout.addWidget(btn_down)
        
        btn_layout.addLayout(move_layout)
        
        btn_layout.addStretch()
        
        # 4. Reset Camera
        btn_reset = QPushButton("Rst")
        btn_reset.setFixedSize(25, 40)
        btn_reset.setStyleSheet("background-color: #444; color: white; font-size: 9px; padding: 0px;")
        btn_reset.clicked.connect(self.reset_camera)
        btn_layout.addWidget(btn_reset)
        
        layout.addWidget(btn_container)
        
        self.view = gl.GLViewWidget()
        self.reset_camera() 
        self.opts = self.view.opts 
        self.opts['distance'] = 2000
        layout.addWidget(self.view)
        
        # Grid
        gz = gl.GLGridItem()
        gz.setSize(5000, 5000, 5000)
        self.view.addItem(gz)
        
        # Axis
        axis = gl.GLAxisItem()
        axis.setSize(100, 100, 100)
        self.view.addItem(axis)
        
        # Labels for axes
        try:
            def add_lbl(pos, txt):
                t = gl.GLTextItem(pos=pos, text=txt, color=(1,1,1,1))
                self.view.addItem(t)

            add_lbl([0, 2500, 0], 'N')
            add_lbl([0, -2500, 0], 'S')
            add_lbl([2500, 0, 0], 'E')
            add_lbl([-2500, 0, 0], 'W')
            
            # Alt Markers on Z axis
            for z in range(0, 5001, 500):
                add_lbl([0, 0, z], f"{z}m")
                tick = gl.GLLinePlotItem(pos=np.array([[-100, 0, z], [100, 0, z]]), color=(0.5, 0.5, 0.5, 1), width=1)
                self.view.addItem(tick)
                tick2 = gl.GLLinePlotItem(pos=np.array([[0, -100, z], [0, 100, z]]), color=(0.5, 0.5, 0.5, 1), width=1)
                self.view.addItem(tick2)

        except:
            print("GLTextItem not available or error adding labels")

        # Path
        # Width 30 might be too big for some drivers, but requested by user.
        self.path_item = gl.GLLinePlotItem(pos=np.array([[0,0,0]]), color=np.array([[0,1,0,1]]), width=3, antialias=True)
        self.view.addItem(self.path_item)
        
        self.points = [] 
        self.colors = []
        self.base_lat = None
        self.base_lon = None
        self.base_alt = None

    def set_camera(self, az, el):
        self.view.setCameraPosition(distance=self.view.cameraParams()['distance'], elevation=el, azimuth=az)
        
    def move_camera_z(self, delta):
        current = self.view.opts['center']
        new_vec = QVector3D(current.x(), current.y(), current.z() + delta)
        self.view.opts['center'] = new_vec
        self.view.update()
        
    def reset_camera(self):
        self.view.opts['center'] = QVector3D(0, 0, 800)
        self.view.setCameraPosition(distance=2000, elevation=30, azimuth=-90)

    def update_path(self, lat, lon, alt, state):
        if self.base_lat is None:
            self.base_lat = lat
            self.base_lon = lon
            self.base_alt = alt
            
        d_lat = lat - self.base_lat
        d_lon = lon - self.base_lon
        
        y = d_lat * 111132.92 
        x = d_lon * 111412.84 * math.cos(math.radians(self.base_lat))
        z = alt - self.base_alt # Relative altitude (Start at 0)
        
        self.points.append([x, y, z])
        
        # Color Logic
        # Green for Ascent (<=2), Red for Descent (3)
        c = (0, 1, 0, 1) # Green
        if state == 3:
            c = (1, 0, 0, 1) # Red
        
        self.colors.append(c)
        
        if len(self.points) > 1:
            pts = np.array(self.points)
            cols = np.array(self.colors)
            self.path_item.setData(pos=pts, color=cols)

    def reset_path(self):
        self.points = []
        self.colors = []
        self.base_lat = None
        self.base_lon = None
        self.base_alt = None
        self.path_item.setData(pos=np.array([[0,0,0]]), color=np.array([[0,1,0,1]]))

class RocketOrientationWidget(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0,0,0,0)
        
        # Legend for Directions - Smaller
        legend = QLabel("Red=N | Grn=E | Blu=Up | Yel=W | Pur=S")
        legend.setStyleSheet("color: white; font-weight: bold; background-color: rgba(0,0,0,100); padding: 1px; font-size: 9px;") 
        legend.setFixedHeight(15) 
        layout.addWidget(legend)
        
        # Toolbar
        btn_layout = QHBoxLayout()
        # Look N (-90), Look S (90), Look E (180), Look W (0)
        buttons = [
            ("Look N", -90, 0),
            ("Look S", 90, 0),
            ("Look E", 180, 0),
            ("Look W", 0, 0)
        ]
        
        for label, az, el in buttons:
            btn = QPushButton(label)
            btn.clicked.connect(lambda _, a=az, e=el: self.set_camera(a, e))
            btn_layout.addWidget(btn)
        layout.addLayout(btn_layout)
        
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=15, elevation=0, azimuth=-90)
        layout.addWidget(self.view)
        
        # Grid
        gz = gl.GLGridItem()
        self.view.addItem(gz)
        
        # Colored Axes
        width = 4
        # N
        self.view.addItem(gl.GLLinePlotItem(pos=np.array([[0,0,0], [0,10,0]]), color=(1, 0, 0, 1), width=width, antialias=True))
        # S
        self.view.addItem(gl.GLLinePlotItem(pos=np.array([[0,0,0], [0,-10,0]]), color=(1, 0, 1, 1), width=width, antialias=True)) # Purple
        # E
        self.view.addItem(gl.GLLinePlotItem(pos=np.array([[0,0,0], [10,0,0]]), color=(0, 1, 0, 1), width=width, antialias=True))
        # W
        self.view.addItem(gl.GLLinePlotItem(pos=np.array([[0,0,0], [-10,0,0]]), color=(1, 1, 0, 1), width=width, antialias=True)) # Yellow
        # Up
        self.view.addItem(gl.GLLinePlotItem(pos=np.array([[0,0,0], [0,0,10]]), color=(0, 0, 1, 1), width=width, antialias=True))
 
        # Rocket Model
        md = gl.MeshData.cylinder(rows=10, cols=20, radius=[1.0, 1.0], length=5.0)
        self.body = gl.GLMeshItem(meshdata=md, smooth=True, color=(0.8, 0.8, 0.8, 1), shader='balloon')
        
        md_nose = gl.MeshData.cylinder(rows=10, cols=20, radius=[1.0, 0.0], length=2.0)
        self.nose = gl.GLMeshItem(meshdata=md_nose, smooth=True, color=(1, 0, 0, 1), shader='balloon')
        
        # Assemble
        self.body.translate(0, 0, -2.5)
        self.nose.translate(0, 0, 2.5)
        
        self.view.addItem(self.body)
        self.view.addItem(self.nose)
        
    def set_camera(self, az, el):
        self.view.opts['center'] = QVector3D(0, 0, 0)
        self.view.setCameraPosition(distance=15, elevation=el, azimuth=az)
        
    def set_orientation(self, pitch, roll, yaw):
        self.body.resetTransform()
        self.body.translate(0, 0, -2.5)
        self.nose.resetTransform()
        self.nose.translate(0, 0, 2.5)
        
        # Rotate
        for item in [self.body, self.nose]:
            item.rotate(yaw, 0, 0, 1)
            item.rotate(pitch, 1, 0, 0)
            item.rotate(roll, 0, 1, 0)

class TelemetryTable(QTableWidget):
    def __init__(self):
        super().__init__()
        self.setColumnCount(2)
        self.setHorizontalHeaderLabels(["Parameter", "Value"])
        self.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.verticalHeader().setVisible(False)
        
        self.fields = [
            "BMP Temp", "Pressure", "BMP Alt", 
            "Accel X", "Accel Y", "Accel Z", 
            "Gyro X", "Gyro Y", "Gyro Z", 
            "IMU Temp", 
            "Mag X", "Mag Y", "Mag Z", 
            "External Temp", # Updated Name
            "GPS Lat", "GPS Lon", "GPS Alt", 
            "GPS Speed", "GPS Angle", 
            "Timestamp"
        ]
        self.setRowCount(len(self.fields))
        
        for i, field in enumerate(self.fields):
            self.setItem(i, 0, QTableWidgetItem(field))
            self.setItem(i, 1, QTableWidgetItem("-"))
            
    def update_data(self, data_dict):
        for i, field in enumerate(self.fields):
            if field in data_dict:
                self.setItem(i, 1, QTableWidgetItem(str(data_dict[field])))

# Main Window
class GroundStationWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rocket Ground Station")
        self.resize(1600, 950)
        self.program_start_time = time.time()
        self.current_flight_time = 0.0
        
        # Main layout container
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        self.layout = QGridLayout(main_widget)
        
        # Configure Grid Layout Column Stretches
        self.layout.setColumnStretch(0, 0) # Fixed
        self.layout.setColumnStretch(1, 22) 
        self.layout.setColumnStretch(2, 23) 
        self.layout.setColumnStretch(3, 55) 

        # 1. Header (State + Timer + Abort)
        self.header_layout = QHBoxLayout()
        self.state_label = QLabel("STATE: IDLE")
        self.state_label.setStyleSheet("font-size: 24px; font-weight: bold; color: green;")
        
        self.total_time_label = QLabel("PROG: 0s")
        self.total_time_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #e0e0e0; background-color: #1e1e1e; padding: 5px;")
        
        self.timer_label = QLabel("FLIGHT: 0.0s")
        self.timer_label.setStyleSheet("font-size: 24px; font-weight: bold; color: white; background-color: #1e1e1e; padding: 5px;")
        
        self.scale_btn = QPushButton("SCALE: FLIGHT")
        self.scale_btn.setStyleSheet("background-color: #004488; color: white; font-weight: bold; font-size: 16px; padding: 10px;")
        self.scale_btn.clicked.connect(self.scale_graphs_to_flight)
        
        self.abort_btn = QPushButton("ABORT")
        self.abort_btn.setStyleSheet("background-color: red; color: white; font-weight: bold; font-size: 20px; padding: 10px;")
        self.abort_btn.clicked.connect(self.on_abort)
        
        self.header_layout.addWidget(self.state_label)
        self.header_layout.addSpacing(20)
        self.header_layout.addWidget(self.total_time_label) 
        self.header_layout.addSpacing(10)
        self.header_layout.addWidget(self.timer_label) 
        self.header_layout.addStretch()
        self.header_layout.addWidget(self.scale_btn)
        self.header_layout.addSpacing(10)
        self.header_layout.addWidget(self.abort_btn)
        
        self.layout.addLayout(self.header_layout, 0, 0, 1, 4)

        # 2. Left Column
        left_layout = QVBoxLayout()
        self.gps_widget = GPSDisplayWidget() 
        left_layout.addWidget(self.gps_widget)

        # Log
        log_group = QGroupBox("System Log")
        log_ly = QVBoxLayout()
        self.console_log = QTextEdit()
        self.console_log.setReadOnly(True)
        self.console_log.setMaximumHeight(400) 
        log_ly.addWidget(self.console_log)
        log_group.setLayout(log_ly)
        left_layout.addWidget(log_group)
        
        # Table
        table_group = QGroupBox("Raw Telemetry")
        table_ly = QVBoxLayout()
        self.telemetry_table = TelemetryTable()
        table_ly.addWidget(self.telemetry_table)
        table_group.setLayout(table_ly)
        left_layout.addWidget(table_group)
        
        left_widget = QWidget()
        left_widget.setLayout(left_layout)
        left_widget.setMaximumWidth(350)
        self.layout.addWidget(left_widget, 1, 0, 2, 1)

        # 3. Center Area (3D Views + 2D Ground Track)
        center_layout = QVBoxLayout()
        
        viz_layout = QHBoxLayout()
        
        orient_group = QGroupBox("Rocket Orientation")
        orient_ly = QVBoxLayout()
        self.rocket_view = RocketOrientationWidget()
        orient_ly.addWidget(self.rocket_view)
        orient_group.setLayout(orient_ly)
        viz_layout.addWidget(orient_group)
        
        flight_group = QGroupBox("Flight Path (3D)")
        flight_ly = QVBoxLayout()
        self.flight_view3d = FlightPathWidget()
        flight_ly.addWidget(self.flight_view3d)
        flight_group.setLayout(flight_ly)
        viz_layout.addWidget(flight_group)
        
        center_layout.addLayout(viz_layout, stretch=1)
        
        self.ground_track = GroundTrackWidget()
        center_layout.addWidget(self.ground_track, stretch=1)
        
        self.layout.addLayout(center_layout, 1, 1, 2, 2)

        # 4. Graphs (Right Column)
        self.graphs_layout = QVBoxLayout()
        
        self.accel_plot = ScrollingPlot("Accelerometer", "Time", "Accel (m/s²)", threshold=0)
        self.accel_plot.add_curve("X", 'r')
        self.accel_plot.add_curve("Y", 'g')
        self.accel_plot.add_curve("Z", 'b')
        self.graphs_layout.addWidget(self.accel_plot)
        
        self.alt_plot = ScrollingPlot("Altitude", "Time", "Alt (m)")
        self.alt_plot.add_curve("Baro Alt", 'c')
        self.alt_plot.add_curve("GPS Alt", 'm')
        self.graphs_layout.addWidget(self.alt_plot)
        
        self.temp_plot = ScrollingPlot("Temperature", "Time", "Temp (°C)", threshold=25)
        self.temp_plot.add_curve("BMP Temp", 'y')
        self.temp_plot.add_curve("External Temp", 'm') 
        self.graphs_layout.addWidget(self.temp_plot)
        
        # Add the graphs layout to the main grid (Row 1, Col 3, RowSpan 2, ColSpan 1)
        # MOVED TO CORRECT LOCATION (End of __init__)
        self.layout.addLayout(self.graphs_layout, 1, 3, 2, 1)

        # Data Generator Thread
        self.data_thread = DataGenerator()
        self.data_thread.new_data.connect(self.update_display)
        self.data_thread.log_message.connect(self.log)
        self.data_thread.start()

    def update_display(self, data):
        # Data tuple has one extra element now: flight_time
        flight_time = data[-1]
        packet_data = data[:-1]
        
        # Extract fields
        # structure: bmp_temp, pressure, bmp_alt, ax, ay, az, gx, gy, gz, imu_temp, mx, my, mz, ext_temp, 
        #            lat, lon, gps_alt, speed, angle, time, state
        
        # Scaling
        bmp_temp = packet_data[0] / 100.0
        pressure = packet_data[1] / 100.0
        bmp_alt = packet_data[2] / 10.0
        accel_x = packet_data[3] / 100.0
        accel_y = packet_data[4] / 100.0
        accel_z = packet_data[5] / 100.0
        gyro_x = packet_data[6] / 100.0
        gyro_y = packet_data[7] / 100.0
        gyro_z = packet_data[8] / 100.0
        imu_temp = packet_data[9] / 100.0
        mag_x = packet_data[10] / 100.0
        mag_y = packet_data[11] / 100.0
        mag_z = packet_data[12] / 100.0
        extra_temp = packet_data[13] / 100.0
        lat = packet_data[14] / 1e7
        lon = packet_data[15] / 1e7
        gps_alt = packet_data[16] / 10.0
        speed = packet_data[17] / 100.0
        angle = packet_data[18] / 100.0
        timestamp = packet_data[19]
        state = packet_data[20] # State is last in packer logic (before appending flight_time)

        # State Map
        state_map = {0: "IDLE", 1: "FLIGHT", 2: "APOGEE", 3: "DESCENDING", 4: "LANDED"}
        state_str = state_map.get(state, "UNKNOWN")
        self.state_label.setText(f"STATE: {state_str}")
        
        self.timer_label.setText(f"FLIGHT: {flight_time:.1f}s")
        self.current_flight_time = flight_time
        
        prog_elapsed = int(time.time() - self.program_start_time)
        self.total_time_label.setText(f"PROG: {prog_elapsed}s")

        # Update Plots
        self.accel_plot.update_data([accel_x, accel_y, accel_z])
        self.alt_plot.update_data([bmp_alt + 20, gps_alt]) 
        self.temp_plot.update_data([bmp_temp, extra_temp])
        
        # Update Table
        table_data = {
            "BMP Temp": f"{bmp_temp:.2f} C", "Pressure": f"{pressure:.2f} hPa", "BMP Alt": f"{bmp_alt:.1f} m",
            "Accel X": f"{accel_x:.2f}", "Accel Y": f"{accel_y:.2f}", "Accel Z": f"{accel_z:.2f}",
            "Gyro X": f"{gyro_x:.2f}", "Gyro Y": f"{gyro_y:.2f}", "Gyro Z": f"{gyro_z:.2f}",
            "IMU Temp": f"{imu_temp:.2f} C",
            "Mag X": f"{mag_x:.2f}", "Mag Y": f"{mag_y:.2f}", "Mag Z": f"{mag_z:.2f}",
            "External Temp": f"{extra_temp:.2f} C",
            "GPS Lat": f"{lat:.6f}", "GPS Lon": f"{lon:.6f}", "GPS Alt": f"{gps_alt:.1f} m",
            "GPS Speed": f"{speed:.2f}", "GPS Angle": f"{angle:.2f}",
            "Timestamp": str(timestamp)
        }
        self.telemetry_table.update_data(table_data)
        
        # Update GPS
        self.gps_widget.update_gps(lat, lon, gps_alt)
        
        # Update Ground Track
        self.ground_track.update_pos(lat, lon)
        
        # Update 3D Path
        self.flight_view3d.update_path(lat, lon, gps_alt, state) 
        
        # Update Orientation
        pitch = math.degrees(math.atan2(accel_x, math.sqrt(accel_y**2 + accel_z**2)))
        roll = math.degrees(math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)))
        yaw = angle 
        
        self.rocket_view.set_orientation(pitch, roll, -yaw) 

    def log(self, message):
        self.console_log.append(message)
        sb = self.console_log.verticalScrollBar()
        sb.setValue(sb.maximum())

    def on_abort(self):
        self.log("!!! ABORT COMMAND SENT !!!")

    def scale_graphs_to_flight(self):
        samples = int(self.current_flight_time) + 5 
        self.log(f"Scaling graphs to last {samples} seconds/samples...")
        self.accel_plot.zoom_to_last(samples)
        self.alt_plot.zoom_to_last(samples)
        self.temp_plot.zoom_to_last(samples)

    def closeEvent(self, event):
        self.data_thread.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion") # Fusion is good base for dark themes
    app.setStyleSheet(DARK_STYLESHEET)
    window = GroundStationWindow()
    window.show()
    sys.exit(app.exec())
