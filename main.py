from flask import Flask, render_template
from flask_socketio import SocketIO
import random
import time
import csv
from datetime import datetime

# Import sensor libraries provided by SDK
from bmp180 import BMP180
from mpu6050 import mpu6050
from tfluna import TFLuna
import RPi.GPIO as GPIO

app = Flask(__name__)
app.config['SECRET_KEY'] = 'aerospace-jam-secret'
# async_mode='threading' is critical for fast loops on Pi Zero
socketio = SocketIO(app, async_mode='threading')

# --- HARDWARE INIT ---
print("INFO: Initializing hardware...")

# 1. Barometer
try:
    bmp = BMP180()
    print("SUCCESS: BMP180 initialized.")
except:
    bmp = None
    print("WARNING: BMP180 not found. Using mock.")

# 2. IMU
try:
    mpu = mpu6050(0x68)
    mpu.get_accel_data()
    print("SUCCESS: MPU-6050 initialized.")
except:
    mpu = None
    print("WARNING: MPU-6050 not found. Using mock.")

# 3. LiDAR
try:
    tfluna = TFLuna()
    tfluna.open()
    tfluna.set_samp_rate(10) 
    print("SUCCESS: TF-Luna initialized.")
except:
    tfluna = None
    print("WARNING: TF-Luna not found. Using mock.")

# 4. Motors
IN1, IN2 = 12, 13
gpio_active = False
try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IN1, GPIO.OUT)
    GPIO.setup(IN2, GPIO.OUT)
    gpio_active = True
    print("SUCCESS: GPIO initialized.")
except:
    print("ERROR: GPIO initialization failed.")

@app.route('/')
def index():
    return render_template('index.html')

# --- PHYSICS STATE VARIABLES (LEVEL 4) ---
velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
rotation = {'x': 0.0, 'y': 0.0, 'z': 0.0}
# We need to store the previous readings for Velocity Verlet / Trapezoidal integration
prev_accel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
prev_gyro = {'x': 0.0, 'y': 0.0, 'z': 0.0}
last_time = time.time()

def background_thread():
    global velocity, position, rotation, prev_accel, prev_gyro, last_time

    # --- LEVEL 4 LOGGING ---
    filename = f"flight_log_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
    log_file = None
    log_writer = None
    
    try:
        log_file = open(filename, mode='w', newline='')
        log_writer = csv.writer(log_file)
        headers = ['Timestamp', 'AccelX', 'AccelY', 'AccelZ', 
                   'VelX', 'VelY', 'VelZ', 
                   'PosX', 'PosY', 'PosZ', 
                   'GyroX', 'GyroY', 'GyroZ',
                   'RotX', 'RotY', 'RotZ'] 
        log_writer.writerow(headers)
        print(f"INFO: High-Accuracy Logging started: {filename}")
    except:
        print("ERROR: Could not create log file.")

    while True:
        # 20Hz loop for stable integration
        socketio.sleep(0.05) 
        
        current_sys_time = time.time()
        dt = current_sys_time - last_time
        last_time = current_sys_time

        try:
            # 1. READ SENSORS
            pressure = bmp.get_pressure() if bmp else 1013.25

            if mpu:
                accel = mpu.get_accel_data()
                gyro = mpu.get_gyro_data()
                ax, ay, az = accel['x'], accel['y'], accel['z']
                gx, gy, gz = gyro['x'], gyro['y'], gyro['z']
            else:
                # Mock Data with slight noise
                ax, ay, az = random.uniform(-0.05, 0.05), random.uniform(-0.05, 0.05), 9.81 + random.uniform(-0.05, 0.05)
                gx, gy, gz = random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5)

            # --- PHYSICS ENGINE (LEVEL 4 - VELOCITY VERLET) ---
            
            # A. Gravity Compensation (Z-axis)
            az_real = az - 9.81
            
            # B. Noise Deadzone Filter
            accel_thresh = 0.2
            ax = ax if abs(ax) > accel_thresh else 0
            ay = ay if abs(ay) > accel_thresh else 0
            az_real = az_real if abs(az_real) > accel_thresh else 0

            gyro_thresh = 1.0
            gx = gx if abs(gx) > gyro_thresh else 0
            gy = gy if abs(gy) > gyro_thresh else 0
            gz = gz if abs(gz) > gyro_thresh else 0

            # C. VELOCITY VERLET INTEGRATION (More accurate than Euler)
            # 1. Update Position using current velocity and previous acceleration
            position['x'] += velocity['x'] * dt + 0.5 * prev_accel['x'] * (dt**2)
            position['y'] += velocity['y'] * dt + 0.5 * prev_accel['y'] * (dt**2)
            position['z'] += velocity['z'] * dt + 0.5 * prev_accel['z'] * (dt**2)

            # 2. Update Velocity using average of previous and current acceleration (Trapezoidal)
            velocity['x'] += 0.5 * (prev_accel['x'] + ax) * dt
            velocity['y'] += 0.5 * (prev_accel['y'] + ay) * dt
            velocity['z'] += 0.5 * (prev_accel['z'] + az_real) * dt
            
            # 3. Update Rotation using Trapezoidal rule for angles
            rotation['x'] += 0.5 * (prev_gyro['x'] + gx) * dt
            rotation['y'] += 0.5 * (prev_gyro['y'] + gy) * dt
            rotation['z'] += 0.5 * (prev_gyro['z'] + gz) * dt

            # Store current values for the next loop
            prev_accel = {'x': ax, 'y': ay, 'z': az_real}
            prev_gyro = {'x': gx, 'y': gy, 'z': gz}

            # 3. LiDAR
            lidar_val = round(tfluna.read()[0] * 100.0, 2) if tfluna else 0

            # 4. LOGGING
            t_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            if log_writer:
                log_writer.writerow([t_str, round(ax,2), round(ay,2), round(az,2),
                                    round(velocity['x'],2), round(velocity['y'],2), round(velocity['z'],2),
                                    round(position['x'],2), round(position['y'],2), round(position['z'],2),
                                    round(gx,2), round(gy,2), round(gz,2),
                                    round(rotation['x'],2), round(rotation['y'],2), round(rotation['z'],2)])
                log_file.flush()

            # 5. SEND TO UI
            socketio.emit('update_data', {
                'time': datetime.now().strftime("%H:%M:%S"),
                'accelX': round(ax, 2), 'accelY': round(ay, 2), 'accelZ': round(az, 2),
                'posX': round(position['x'], 2), 'posY': round(position['y'], 2), 'posZ': round(position['z'], 2),
                'rotX': round(rotation['x'], 2), 'rotY': round(rotation['y'], 2), 'rotZ': round(rotation['z'], 2),
                'lidarDistance': lidar_val, 'barometricPressure': pressure
            })

        except Exception as e:
            print(f"Loop Error: {e}")

@socketio.on('connect')
def handle_connect():
    global last_time
    print('Client connected')
    last_time = time.time()
    socketio.start_background_task(target=background_thread)

@socketio.on('control_motor')
def handle_motor_control(message):
    if not gpio_active: return
    action = message.get('action')
    if action == 'forward':
        GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
    elif action == 'backward':
        GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.HIGH)
    elif action == 'stop':
        GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.LOW)

def main():
    socketio.run(app, host='0.0.0.0', port=80, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    try:
        main()
    finally:
        if gpio_active: GPIO.cleanup()