# These two modules allow us to run a web server.
from flask import Flask, render_template
from flask_socketio import SocketIO
import random
import time
import csv                      # Library for CSV file handling
from datetime import datetime   # Library for timestamps

# Import sensor libraries provided by SDK
from bmp180 import BMP180
from mpu6050 import mpu6050
from tfluna import TFLuna
import RPi.GPIO as GPIO

app = Flask(__name__)
app.config['SECRET_KEY'] = 'aerospace-jam-secret'
socketio = SocketIO(app)

# --- SAFE HARDWARE INITIALIZATION ---
print("INFO: Initializing hardware with safety checks...")

# 1. Barometer (BMP180)
try:
    bmp = BMP180()
    print("SUCCESS: BMP180 initialized.")
except Exception as e:
    bmp = None
    print(f"WARNING: BMP180 not found ({e}). Using mock data.")

# 2. Accelerometer/Gyroscope (MPU-6050)
try:
    mpu = mpu6050(0x68)
    # Try reading once to confirm connection
    mpu.get_accel_data()
    print("SUCCESS: MPU-6050 initialized.")
except Exception as e:
    mpu = None
    print(f"WARNING: MPU-6050 not found ({e}). Using mock data.")

# 3. LiDAR (TF-Luna)
try:
    tfluna = TFLuna()
    tfluna.open()
    tfluna.set_samp_rate(5)
    print("SUCCESS: TF-Luna initialized.")
except Exception as e:
    tfluna = None
    print(f"WARNING: TF-Luna not found ({e}). Using mock data.")

# 4. Motors (GPIO)
IN1 = 12
IN2 = 13
gpio_active = False

try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    gpio_active = True
    print("SUCCESS: GPIO initialized for motors.")
except Exception as e:
    print(f"ERROR: GPIO initialization failed ({e}). Motors disabled.")

@app.route('/')
def index():
    return render_template('index.html')

def background_thread():
    # --- LEVEL 2: LOGGING SETUP ---
    # Create a unique filename with timestamp to prevent overwriting
    filename = f"flight_log_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
    log_file = None
    log_writer = None

    try:
        # Hybrid Method: Open file ONCE for performance
        log_file = open(filename, mode='w', newline='')
        log_writer = csv.writer(log_file)
        # Write CSV Header
        log_writer.writerow(['Timestamp', 'AccelX', 'AccelY', 'AccelZ', 'GyroX', 'GyroY', 'GyroZ'])
        print(f"INFO: Logging started to file {filename}")
    except Exception as e:
        print(f"ERROR: Could not open log file: {e}")

    while True:
        socketio.sleep(1) # Loop every second
        try:
            # --- READ SENSORS (Hybrid Mode: Real or Mock) ---
            
            # 1. Pressure
            if bmp:
                pressure = bmp.get_pressure()
            else:
                pressure = round(random.uniform(990.0, 1020.0), 2)

            # 2. IMU (Accel/Gyro)
            if mpu:
                accel = mpu.get_accel_data()
                gyro = mpu.get_gyro_data()
                ax, ay, az = accel['x'], accel['y'], accel['z']
                gx, gy, gz = gyro['x'], gyro['y'], gyro['z']
            else:
                ax, ay, az = 0.0, 0.0, 9.8
                gx, gy, gz = 0.0, 0.0, 0.0

            # Get current time for both logging and UI
            current_time = datetime.now().strftime("%H:%M:%S")

            # Write data row and flush to disk immediately
            if log_writer:
                log_writer.writerow([
                    current_time, 
                    round(ax, 2), round(ay, 2), round(az, 2),
                    round(gx, 2), round(gy, 2), round(gz, 2)
                ])
                log_file.flush() # Ensures data is saved even if power is lost

            # 3. LiDAR
            lidar_val = 0
            if tfluna:
                try:
                    dist, strength, temp = tfluna.read()
                    lidar_val = round(dist * 100.0, 2)
                except:
                    lidar_val = -1 
            else:
                lidar_val = 0 

            # --- SEND DATA TO CLIENT ---
            socketio.emit('update_data', {
                'time': current_time,          # <--- LEVEL 3: Time for Chart X-Axis
                'randomNumber': random.randint(1, 100),
                'barometricPressure': pressure,
                'accelX': round(ax, 2), 'accelY': round(ay, 2), 'accelZ': round(az, 2),
                'gyroX': round(gx, 2), 'gyroY': round(gy, 2), 'gyroZ': round(gz, 2),
                'lidarDistance': lidar_val
            })
        except Exception as e:
            print(f"Loop Error: {e}")

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    socketio.start_background_task(target=background_thread)

@socketio.on('control_motor')
def handle_motor_control(message):
    if not gpio_active:
        print("WARNING: Motor command ignored (GPIO inactive)")
        return

    action = message.get('action')
    print(f"Motor Command: {action}")

    if action == 'forward':
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    elif action == 'backward':
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    elif action == 'stop':
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)

def main():
    socketio.run(app, host='0.0.0.0', port=80, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    try:
        main()
    finally:
        if gpio_active:
            GPIO.cleanup()
            print("INFO: GPIO cleaned up.")