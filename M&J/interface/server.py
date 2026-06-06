from flask import Flask, render_template
from flask_socketio import SocketIO
import serial
import threading #To read PID values from serial
import time

app = Flask(__name__)
socketio = SocketIO(app)

# Open serial port
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def serial_reader():
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("SYNC,"):
                ps = line.split(',') #ps means parts
                data = {'p':ps[1], 'i':ps[2], 'd':ps[3], 't':ps[4], 'tkp':ps[5], 'tkd':ps[6], 'vp':ps[7], 'vi':ps[8]}
                socketio.emit('sync_values', data)
            else:
                socketio.emit('telemetry', line)
        else:
            time.sleep(0.01) #10ms
threading.Thread(target=serial_reader, daemon=True).start()

@app.route('/')
def index():
    return render_template('index.html')

# Initial handshake with ESP32 to get initial gains
@socketio.on('request_initial_values')
def get_values():
    print("Requesting initial PID values from ESP")
    ser.write(b'GET_PID\n')

@socketio.on('command')
def handle_command(cmd):
    # If it's a simple movement, send it directly
    if cmd in ['F', 'B', 'L', 'R', 'S', 'T1', 'T2', 'CAL']:
        print(f"Sending to ESP32: {cmd}")
        ser.write(cmd.encode('utf-8')+b'\n')

@socketio.on('update_pid')
def handle_pid_update(data):
    msg = f"P{data['p']},I{data['i']},D{data['d']},T{data['t']},TKP{data['tkp']},TKD{data['tkd']},VP{data['vp']},VI{data['vi']}\n"
    ser.write(msg.encode('utf-8'))

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=80)
