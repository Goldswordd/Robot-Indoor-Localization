import sys
import serial
import time
import threading
import re
import numpy as np
from flask import Flask, render_template_string
from flask_socketio import SocketIO

ANCHORS = [
    (0, 0),     # Anchor 1
    (0, 1280),  # Anchor 2
    (1800, 1280), # Anchor 3
    (1800, 0)   # Anchor 4
]

SERIAL_PORT = 'COM5'  
BAUDRATE = 115200

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')
data_queue = []
position_history = []
ser = None

serial_pattern = re.compile(r'0x[\da-f]{4}:\s*=(\d+)')

def init_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        print("Đã kết nối serial thành công!")
    except Exception as e:
        print("Lỗi serial:", e)
        sys.exit(1)

def lse_trilateration(distances):
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
    except:
        return np.array([np.nan, np.nan])

def serial_reader():
    while True:
        if ser and ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='replace')
            if '=' in line:
                distances = list(map(int, serial_pattern.findall(line)))
                if len(distances) >= 4:
                    pos = lse_trilateration(distances[:4])
                    if not np.isnan(pos).any():
                        position_history.append(pos)
                        if len(position_history) > 40:
                            position_history.pop(0)
                        filtered_pos = np.mean(position_history, axis=0)
                        socketio.emit('position', {
                            'x': float(filtered_pos[0]),
                            'y': float(filtered_pos[1])
                        })

@app.route('/')
def index():
    return render_template_string('''
    <!DOCTYPE html>
    <html>
    <head>
        <title>Real-Time Tracking</title>
        <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
        <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    </head>
    <body>
        <h1>Location tracking</h1>
        <div id="plot" style="width:800px;height:600px;"></div>
        <div id="coordinates"></div>

        <script>
            const socket = io();
            let data = { x: [], y: [] };
            const layout = {
                xaxis: { range: [-500, 2000], title: 'X (mm)' },
                yaxis: { range: [-500, 2000], title: 'Y (mm)' },
                showlegend: false
            };
            Plotly.newPlot('plot', [data], layout);

            socket.on('position', (pos) => {
                data.x.push(pos.x);
                data.y.push(pos.y);
                if (data.x.length > 100) {
                    data.x.shift();
                    data.y.shift();
                }
                Plotly.update('plot', { x: [data.x], y: [data.y] }, layout);
                document.getElementById('coordinates').innerHTML = 
                    `X: ${pos.x.toFixed(2)} mm | Y: ${pos.y.toFixed(2)} mm`;
            });
        </script>
    </body>
    </html>
    ''')

if __name__ == '__main__':
    init_serial()
    threading.Thread(target=serial_reader, daemon=True).start()
    socketio.run(app, host='0.0.0.0', port=5555, debug=False)