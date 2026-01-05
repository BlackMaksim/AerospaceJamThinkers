# These two modules allow us to run a web server.
from flask import Flask, render_template
from flask_socketio import SocketIO
import random
import time

# Import sensor libraries provided by SDK
from bmp180 import BMP180
from mpu6050 import mpu6050
from tfluna import TFLuna
# Import GPIO library for motor control
import RPi.GPIO as GPIO

app = Flask(__name__)
socketio = SocketIO(app)

# --- MOTOR CONFIGURATION ---
# Define our motor controller pins as per documentation
IN1 = 12
IN2 = 13

# Set the GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# Set up the GPIO pins as output
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# --- SENSOR INITIALIZATION ---
print("INFO: Initializing hardware...")

# 1. Barometer
bmp = BMP180()

# 2. Accelerometer/Gyroscope
mpu = mpu6050(0x68)

# 3. LiDAR (TF-Luna)
tfluna = TFLuna()
tfluna.open()
tfluna.set_samp_rate(5)

@app.route('/')
def index():
    return render_template('index.html')

def background_thread():
    while True:
        socketio.sleep(1)
        try:
            # --- READ SENSORS ---
            pressure = bmp.get_pressure()
            accel_data = mpu.get_accel_data()
            gyro_data = mpu.get_gyro_data()
            distance, strength, temp = tfluna.read()

            # --- SEND DATA ---
            socketio.emit(
                'update_data',
                {
                    'randomNumber': random.randint(1, 100),
                    'barometricPressure': pressure,
                    # MPU Data
                    'accelX': round(accel_data['x'], 2),
                    'accelY': round(accel_data['y'], 2),
                    'accelZ': round(accel_data['z'], 2),
                    'gyroX': round(gyro_data['x'], 2),
                    'gyroY': round(gyro_data['y'], 2),
                    'gyroZ': round(gyro_data['z'], 2),
                    # LiDAR Data
                    'lidarDistance': round(distance * 100.0, 2)
                }
            )
        except Exception as e:
            print(f"Sensor Error: {e}")

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    socketio.start_background_task(target=background_thread)

# --- MOTOR CONTROL LOGIC ---
@socketio.on('control_motor')
def handle_motor_control(message):
    """Handles motor control events from the client."""
    action = message.get('action')
    print(f"EVENT: Received 'control_motor' with action: '{action}'")
    
    if action == 'forward':
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    elif action == 'backward':
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    elif action == 'stop':
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
    else:
        print(f"WARNING: Unknown motor action '{action}'")

def main():
    socketio.run(app, host='0.0.0.0', port=80, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    try:
        main()
    finally:
        # Clean up GPIO on exit
        GPIO.cleanup()
        print("INFO: Server stopped and GPIO cleaned up.")