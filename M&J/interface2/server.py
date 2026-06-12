from flask import Flask, render_template, Response
from flask_socketio import SocketIO
import serial
import threading #To read PID values from serial
import time
import json
#import cv2
#from picamera2 import Picamera2

app = Flask(__name__)
#picam2 = Picamera2()
socketio = SocketIO(app)

# Open serial port
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def serial_reader():
    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("{") and line.endswith("}"):
                    # Parse telemetry data packet from ESP32
                    data = json.loads(line)
                    socketio.emit('telemetry_sync', data)
                else:
                    # Print normal debug/serial messages from ESP32
                    print(f"[ESP32 Log]: {line}")
            except Exception as e:
                print(f"Serial parsing error: {e}")
        else:
            time.sleep(0.01) # 10ms rest
threading.Thread(target=serial_reader, daemon=True).start()

'''
# Optimize resolution for low latency over Wi-Fi
picam2.configure(picam2.create_preview_configuration(main={"format": "XRGB8888", "size": (640, 480)}))
picam2.start()

def generate_frames():
    while True:
        frame = picam2.capture_array()
        # Compress to JPEG to save network bandwidth
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ret:
            continue
        yield (b'--frame\r\n'b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')'''

@app.route('/')
def index():
    return render_template('index.html')

# Video feed for the app
@app.route('/video_feed')
def video_feed():
    return Response(status=204)
    #return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Initial handshake with ESP32 to get initial gains
@socketio.on('request_initial_values')
def get_values():
    print("Requesting configurations from ESP32...")
    ser.write(b'GET_SETUP\n')

@socketio.on('command')
def handle_command(cmd):
    # Sends movement keys: F, B, L, R, S, T1, T2, FL, FR, CAL, STOP
    print(f"Sending command: {cmd}")
    ser.write(f"CMD:{cmd}\n".encode('utf-8'))

@socketio.on('change_mode')
def handle_mode(mode):
    # mode is either 'MANUAL' or 'LINE'
    print(f"Setting Bot Mode: {mode}")
    ser.write(f"MODE:{mode}\n".encode('utf-8'))

@socketio.on('update_pid')
def handle_pid_update(data):
    # Turn incoming dictionary into a single clear string string 
    packet = f"CFG:p={data['p']},d={data['d']},t={data['t']},tkp={data['tkp']},tkd={data['tkd']},vp={data['vp']},vi={data['vi']},kt={data['kt']},tilt={data['tilt']},lff={data['lff']},lft={data['lft']}\n"
    ser.write(packet.encode('utf-8'))

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=80)
