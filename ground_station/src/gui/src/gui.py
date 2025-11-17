import sys
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QProgressBar, QGroupBox, QGridLayout)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QPainter, QColor, QPen, QFont, QPolygon
from PyQt6.QtCore import QPoint
import math
import random
import gui_UDP_Server
import packet_handling

class AttitudeIndicator(QWidget):
    """Custom widget to display drone orientation (pitch/roll)"""
    def __init__(self):
        super().__init__()
        self.pitch = 0  
        self.roll = 0   
        self.setMinimumSize(200, 200)

        
    def set_attitude(self, pitch, roll):
        self.pitch = pitch
        self.roll = roll
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        w, h = self.width(), self.height()
        cx, cy = w // 2, h // 2
        radius = min(w, h) // 2 - 10
        
        
        painter.fillRect(0, 0, w, h, QColor(30, 30, 30))
        
    
        painter.translate(cx, cy)
        painter.rotate(-self.roll)
        
       
        sky_height = int(radius * 2 * (self.pitch / 90))
        painter.fillRect(-radius, -radius, radius * 2, radius + sky_height, 
                        QColor(0, 120, 215))
        painter.fillRect(-radius, sky_height, radius * 2, radius * 2, 
                        QColor(139, 90, 43))
        
       
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        for angle in range(-90, 91, 10):
            if angle == 0:
                continue
            y_offset = int(radius * 2 * (angle / 90))
            line_width = 40 if angle % 30 == 0 else 20
            painter.drawLine(-line_width, y_offset, line_width, y_offset)
            
        
        painter.resetTransform()
        painter.translate(cx, cy)
      
        painter.setPen(QPen(QColor(255, 255, 0), 3))
        painter.drawLine(-30, 0, -10, 0)
        painter.drawLine(10, 0, 30, 0)
        painter.drawEllipse(-5, -5, 10, 10)
        
        
        painter.setPen(QPen(QColor(255, 255, 255), 2))
        painter.drawEllipse(-radius, -radius, radius * 2, radius * 2)

class CompassWidget(QWidget):
    """Custom widget to display heading"""
    def __init__(self):
        super().__init__()
        self.heading = 0 
        self.setMinimumSize(200, 60)
        
    def set_heading(self, heading):
        self.heading = heading % 360
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        w, h = self.width(), self.height()
        
        
        painter.fillRect(0, 0, w, h, QColor(30, 30, 30))
        
        
        painter.setPen(QPen(QColor(255, 255, 255), 1))
        painter.setFont(QFont('Arial', 10))
        
        for deg in range(0, 360, 10):
            offset = ((deg - self.heading + 180) % 360 - 180) * w / 180
            x = w // 2 + offset
            
            if -w//2 < offset < w//2:
                if deg % 30 == 0:
                    painter.drawLine(int(x), h - 20, int(x), h - 5)
                    label = ['N', '', '', 'E', '', '', 'S', '', '', 'W', '', ''][deg // 30]
                    if label:
                        painter.drawText(int(x - 10), h - 25, 20, 20, 
                                       Qt.AlignmentFlag.AlignCenter, label)
                else:
                    painter.drawLine(int(x), h - 15, int(x), h - 5)
        
       
        painter.setPen(QPen(QColor(255, 255, 0), 2))
        triangle = QPolygon([
            QPoint(w // 2, h - 5),
            QPoint(w // 2 - 8, h - 15),
            QPoint(w // 2 + 8, h - 15)
        ])
        painter.drawPolygon(triangle)
        

        painter.setFont(QFont('Arial', 14, QFont.Weight.Bold))
        painter.drawText(0, 5, w, 25, Qt.AlignmentFlag.AlignCenter, 
                        f"{int(self.heading)}°")

class DroneGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Telemetry Monitor")
        self.setGeometry(100, 100, 1200, 800)
        self.setStyleSheet("background-color: #1e1e1e; color: white;")
        
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.battery = 100.0
        self.signal_strength = 100
        self.gps_satellites = 12
        self.flight_time = 0
        self.distance_traveled = 0.0
        self.altitude_agl = 0.0 

        self.que = packet_handling.telemetry_queue()
        udp = gui_UDP_Server.server(UDP_callback=self.que.insert)
        udp.start()
        
        self.init_ui()
        
    
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_telemetry)
        self.timer.start(100) 


    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        left_panel = QVBoxLayout()
        
        attitude_group = QGroupBox("Attitude Indicator")
        attitude_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 14px; }")
        attitude_layout = QVBoxLayout()
        self.attitude_indicator = AttitudeIndicator()
        attitude_layout.addWidget(self.attitude_indicator)
        attitude_group.setLayout(attitude_layout)
        left_panel.addWidget(attitude_group)
        
        compass_group = QGroupBox("Heading")
        compass_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 14px; }")
        compass_layout = QVBoxLayout()
        self.compass = CompassWidget()
        compass_layout.addWidget(self.compass)
        compass_group.setLayout(compass_layout)
        left_panel.addWidget(compass_group)
        
        left_panel.addStretch()
        

        right_panel = QVBoxLayout()
        
        pos_group = QGroupBox("Position")
        pos_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 14px; }")
        pos_layout = QGridLayout()
        
        self.pos_labels = {}
        coords = ['X', 'Y', 'Z (Alt)']
        for i, coord in enumerate(coords):
            label = QLabel(f"{coord}:")
            label.setStyleSheet("font-size: 12px;")
            value = QLabel("0.00 m")
            value.setStyleSheet("font-size: 12px; color: #00ff00;")
            pos_layout.addWidget(label, i, 0)
            pos_layout.addWidget(value, i, 1)
            self.pos_labels[coord.split()[0].lower()] = value
            
        pos_group.setLayout(pos_layout)
        right_panel.addWidget(pos_group)
        
    


        vel_group = QGroupBox("Velocity")
        vel_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 14px; }")
        vel_layout = QGridLayout()
        
        self.vel_labels = {}
        for i, coord in enumerate(['Vx', 'Vy', 'Vz']):
            label = QLabel(f"{coord}:")
            label.setStyleSheet("font-size: 12px;")
            value = QLabel("0.00 m/s")
            value.setStyleSheet("font-size: 12px; color: #00ffff;")
            vel_layout.addWidget(label, i, 0)
            vel_layout.addWidget(value, i, 1)
            self.vel_labels[coord.lower()] = value
            
        vel_group.setLayout(vel_layout)
        right_panel.addWidget(vel_group)
        
       
        orient_group = QGroupBox("Orientation")
        orient_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 14px; }")
        orient_layout = QGridLayout()
        
        self.orient_labels = {}
        for i, angle in enumerate(['Roll', 'Pitch', 'Yaw']):
            label = QLabel(f"{angle}:")
            label.setStyleSheet("font-size: 12px;")
            value = QLabel("0.0°")
            value.setStyleSheet("font-size: 12px; color: #ffff00;")
            orient_layout.addWidget(label, i, 0)
            orient_layout.addWidget(value, i, 1)
            self.orient_labels[angle.lower()] = value
            
        orient_group.setLayout(orient_layout)
        right_panel.addWidget(orient_group)
        
        
        battery_group = QGroupBox("Power System")
        battery_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 14px; }")
        battery_layout = QVBoxLayout()
        
        self.battery_bar = QProgressBar()
        self.battery_bar.setMaximum(100)
        self.battery_bar.setValue(100)
        self.battery_bar.setFormat("%p%")
        self.battery_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid grey;
                border-radius: 5px;
                text-align: center;
                font-size: 14px;
                font-weight: bold;
            }
            QProgressBar::chunk {
                background-color: #00ff00;
            }
        """)
        battery_layout.addWidget(self.battery_bar)
        
        self.battery_voltage = QLabel("Voltage: 12.6V")
        self.battery_voltage.setStyleSheet("font-size: 12px;")
        battery_layout.addWidget(self.battery_voltage)
        
        battery_group.setLayout(battery_layout)
        right_panel.addWidget(battery_group)
        
        
        stats_group = QGroupBox("Flight Statistics")
        stats_group.setStyleSheet("QGroupBox { font-weight: bold; font-size: 14px; }")
        stats_layout = QGridLayout()
        
        self.stats_labels = {
            'time': QLabel("00:00"),
            'distance': QLabel("0.00 m"),
            'altitude_agl': QLabel("0.00 m"),
            'gps': QLabel("12 sats"),
            'signal': QLabel("100%")
        }
        
        labels = [
            ('Flight Time:', 'time'),
            ('Distance:', 'distance'),
            ('AGL:', 'altitude_agl'),
            ('GPS:', 'gps'),
            ('Signal:', 'signal')
        ]
        
        for i, (text, key) in enumerate(labels):
            label = QLabel(text)
            label.setStyleSheet("font-size: 12px;")
            self.stats_labels[key].setStyleSheet("font-size: 12px; color: #ff9900;")
            stats_layout.addWidget(label, i, 0)
            stats_layout.addWidget(self.stats_labels[key], i, 1)
            
        stats_group.setLayout(stats_layout)
        right_panel.addWidget(stats_group)
        
        right_panel.addStretch()
        
    
        main_layout.addLayout(left_panel, 1)
        main_layout.addLayout(right_panel, 1)

    def battery(self):
        if self.battery > 30:
            color = "#00ff00"
        elif self.battery > 15:
            color = "#ffff00"
        else:
            color = "#ff0000"
        self.battery_bar.setStyleSheet(f"""
            QProgressBar {{
                border: 2px solid grey;
                border-radius: 5px;
                text-align: center;
                font-size: 14px;
                font-weight: bold;
            }}
            QProgressBar::chunk {{
                background-color: {color};
            }}""")  
          
    def update_telemetry(self):
        data = self.que.get()
        if not data:
            return
        print("has data")
    
        for axis in ['vx', 'vy', 'vz']:
            if axis in data:
                value = data[axis]
                self.vel_labels[axis].setText(f"{value:.2f} m/s")

        
        
    



if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = DroneGUI()
    window.show()
    sys.exit(app.exec())