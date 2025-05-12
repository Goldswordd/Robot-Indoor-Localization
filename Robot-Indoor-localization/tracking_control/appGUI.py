import sys
import serial
import time
import threading
import re
import numpy as np
from queue import Queue
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# Cấu hình Anchor (4 anchor), thay đổi tọa độ nếu cần
ANCHORS = [
    (0, 0),     # Anchor 1
    (0, 1280), # Anchor 2
    (1800, 1280),        # Anchor 3
    (1800, 0)  # Anchor 4
]

# Cấu hình cổng Serial và tốc độ baud
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200

class FPTTracker(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tracking FPT - Real-Time Graph")
        self.setGeometry(100, 100, 800, 600)
        self.data_queue = Queue()
        self.position_history = []
        self.trail_history = []  # Lịch sử quỹ đạo di chuyển của FPT
        self.line_count = 0
        self.start_time = time.time()
        self.fps_time = time.time()
        # Biểu thức chính quy tìm dữ liệu khoảng cách (giả sử dữ liệu có dạng: "0xXXXX:=<value>")
        self.serial_pattern = re.compile(r'0x[\da-f]{4}:\s*=(\d+)')
        self.init_serial()
        self.init_ui()
        self.start_serial_thread()

        # Timer cập nhật đồ thị mỗi 30ms (~33 FPS)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(15)

    def init_serial(self):
        try:
            self.ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUDRATE,
                timeout=0.1
            )
        except Exception as e:
            print("Không thể mở cổng serial:", e)
            sys.exit(1)

    def init_ui(self):
        # Khởi tạo widget đồ thị của PyQtGraph
        self.plot_widget = pg.PlotWidget()
        self.setCentralWidget(self.plot_widget)
        # Cấu hình giới hạn trục
        self.plot_widget.setXRange(-500, 2000)
        self.plot_widget.setYRange(-500, 2000)
        self.plot_widget.setTitle("Tracking Tag - Vị trí theo thời gian thực")

        # Vẽ một tam giác (có thể là khu vực hoạt động) – chỉnh sửa tọa độ nếu cần
        # triangle_coords = [(1990, 0), (1990, 2388), (-7164, 2388), (1990, 0)]
        # triangle_x = [pt[0] for pt in triangle_coords]
        # triangle_y = [pt[1] for pt in triangle_coords]
        # self.plot_widget.plot(triangle_x, triangle_y, pen=pg.mkPen('g', width=2))

        # Vẽ các anchor dưới dạng các điểm xanh
        anchor_x, anchor_y = zip(*ANCHORS)
        self.plot_widget.plot(anchor_x, anchor_y, pen=None, symbol='o', symbolSize=20, symbolBrush='b')

        # Khởi tạo điểm FPT (điểm đỏ)
        self.fpt_scatter = self.plot_widget.plot([], [], pen=None, symbol='o', symbolSize=20, symbolBrush='r')
        # Vẽ quỹ đạo (trail) của FPT
        self.trail_curve = self.plot_widget.plot([], [], pen=pg.mkPen('r', width=1))
        
        # Hiển thị nhãn "FPT" gần điểm đối tượng
        self.fpt_text = pg.TextItem(text="TAG", color='w', anchor=(0.5, -1.0))
        self.plot_widget.addItem(self.fpt_text)

    def start_serial_thread(self):
        threading.Thread(target=self.serial_reader, daemon=True).start()

    def serial_reader(self):
        """Đọc dữ liệu từ cổng serial và đếm số dòng mỗi giây"""
        while True:
            try:
                line = self.ser.readline().decode('utf-8', errors='replace')
                if line:
                    self.line_count += 1
                    if '=' in line:
                        self.data_queue.put(line)
                    # Kiểm tra nếu đã qua 1 giây thì in số dòng đã đọc
                    current_time = time.time()
                    if current_time - self.start_time >= 1:
                        # print(f"Lines per second: {self.line_count}")
                        self.line_count = 0
                        self.start_time = current_time
            except Exception as e:
                print("Serial error:", e)

    def lse_trilateration(self, distances):
        """Thuật toán Least Squares cho 4 anchor nhằm ước lượng vị trí đối tượng FPT"""
        if len(distances) < 4:
            return np.array([np.nan, np.nan])
        x1, y1 = ANCHORS[0]
        d1 = distances[0]
        A = []
        b = []
        for i in range(1, 4):
            xi, yi = ANCHORS[i]
            di = distances[i]
            A.append([xi - x1, yi - y1])
            b.append(0.5 * (xi**2 + yi**2 - di**2 - (x1**2 + y1**2 - d1**2)))
        try:
            A = np.array(A)
            b = np.array(b)
            return np.linalg.lstsq(A, b, rcond=None)[0]
        except Exception as e:
            print("LSE error:", e)
            return np.array([np.nan, np.nan])

    def update_plot(self):
        current_time = time.time()
        fps = 1.0 / (current_time - self.fps_time)
        self.fps_time = current_time
        updated = False
        # Xử lý dữ liệu mới từ serial
        while not self.data_queue.empty():
            raw_data = self.data_queue.get()
            distances = list(map(int, self.serial_pattern.findall(raw_data)))
            if len(distances) >= 4:
                pos = self.lse_trilateration(distances[:4])
                if not np.isnan(pos).any():
                    self.position_history.append(pos)
                    if len(self.position_history) > 40:
                        self.position_history = self.position_history[-100:]
                    filtered_pos = np.mean(self.position_history, axis=0)

                    # THÊM DÒNG NÀY ĐỂ IN TỌA ĐỘ RA TERMINAL
                    print(f"X: {filtered_pos[0]:.2f}, Y: {filtered_pos[1]:.2f}")

                    self.trail_history.append(filtered_pos)
                    if len(self.trail_history) > 10000:
                        self.trail_history = self.trail_history[-50:]
                    # Cập nhật vị trí của FPT
                    self.fpt_scatter.setData([filtered_pos[0]], [filtered_pos[1]])
                    # Cập nhật nhãn "FPT" di chuyển theo đối tượng
                    self.fpt_text.setPos(filtered_pos[0], filtered_pos[1])
                    updated = True
        # Cập nhật quỹ đạo nếu có dữ liệu mới
        if updated and self.trail_history:
            trail = np.array(self.trail_history)
            self.trail_curve.setData(trail[:, 0], trail[:, 1])
        self.setWindowTitle(f"Real-Time UWB Tracking (FPS: {fps:.2f})")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    tracker = FPTTracker()
    tracker.show()
    sys.exit(app.exec_())
