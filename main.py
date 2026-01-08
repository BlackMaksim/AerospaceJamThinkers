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
    tfluna.set_samp_rate(10) # 10Hz sampling for integration
    print("SUCCESS: TF-Luna initialized.")
except:
    tfluna = None
    print("WARNING: TF-Luna not found. Using mock.")

# 4. Motors
IN1 = 12
IN2 = 13
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

# Global variables to store state between loop iterations
velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
rotation = {'x': 0.0, 'y': 0.0, 'z': 0.0} # Roll, Pitch, Yaw
last_time = time.time()

def background_thread():
    global velocity, position, rotation, last_time

    # Create unique log file
    filename = f"flight_log_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
    log_file = None
    log_writer = None
    
    try:
        log_file = open(filename, mode='w', newline='')
        log_writer = csv.writer(log_file)
        # Extended Header for Physics Data including Rotation
        headers = ['Timestamp', 'AccelX', 'AccelY', 'AccelZ', 
                   'VelX', 'VelY', 'VelZ', 
                   'PosX', 'PosY', 'PosZ', 
                   'GyroX', 'GyroY', 'GyroZ',
                   'RotX', 'RotY', 'RotZ'] 
        log_writer.writerow(headers)
        print(f"INFO: Level 4 Logging started: {filename}")
    except:
        print("ERROR: Could not create log file.")

    while True:
        #Faster loop = better integration accuracy.
        socketio.sleep(0.05) 
        
        current_sys_time = time.time()
        dt = current_sys_time - last_time # Delta Time
        last_time = current_sys_time

        try:
            # 1. READ SENSORS
            if bmp:
                pressure = bmp.get_pressure()
            else:
                pressure = 1013.25

            if mpu:
                accel = mpu.get_accel_data()
                gyro = mpu.get_gyro_data()
                ax, ay, az = accel['x'], accel['y'], accel['z']
                gx, gy, gz = gyro['x'], gyro['y'], gyro['z']
            else:
                # Mock Data
                ax = random.uniform(-0.1, 0.1)
                ay = random.uniform(-0.1, 0.1)
                az = 9.81 + random.uniform(-0.1, 0.1)
                gx, gy, gz = random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)

            # 2. PROCESS IMU DATA

            # A. Gravity Compensation (Z-axis only)
            az_real = az - 9.81
            
            # B. Noise Filter 
            # Ignore small vibrations to prevent drift when stationary
            threshold = 0.2
            if abs(ax) < threshold: ax = 0
            if abs(ay) < threshold: ay = 0
            if abs(az_real) < threshold: az_real = 0
            
            gyro_threshold = 1.0 # Degrees per second deadzone
            if abs(gx) < gyro_threshold: gx = 0
            if abs(gy) < gyro_threshold: gy = 0
            if abs(gz) < gyro_threshold: gz = 0

            # C. Integration 1: Accel -> Velocity (v = v0 + a*t)
            velocity['x'] += ax * dt
            velocity['y'] += ay * dt
            velocity['z'] += az_real * dt

            # D. Integration 2: Velocity -> Position (p = p0 + v*t)
            position['x'] += velocity['x'] * dt
            position['y'] += velocity['y'] * dt
            position['z'] += velocity['z'] * dt
            
            # E. Integration 3: Gyro Rate -> Rotation Angle (angle = angle0 + rate*t)
            rotation['x'] += gx * dt # Roll
            rotation['y'] += gy * dt # Pitch
            rotation['z'] += gz * dt # Yaw

            # 3. LiDAR
            lidar_val = 0
            if tfluna:
                try:
                    dist, _, _ = tfluna.read()
                    lidar_val = round(dist * 100.0, 2)
                except: lidar_val = -1
            else: lidar_val = 0

            # 4. LOGGING TO FILE
            t_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            
            if log_writer:
                log_writer.writerow([
                    t_str, 
                    round(ax, 2), round(ay, 2), round(az, 2),
                    round(velocity['x'], 2), round(velocity['y'], 2), round(velocity['z'], 2),
                    round(position['x'], 2), round(position['y'], 2), round(position['z'], 2),
                    round(gx, 2), round(gy, 2), round(gz, 2),
                    round(rotation['x'], 2), round(rotation['y'], 2), round(rotation['z'], 2)
                ])
                log_file.flush() # Force save to disk

            # 5. SEND TO WEB UI
            socketio.emit('update_data', {
                'time': datetime.now().strftime("%H:%M:%S"),
                'accelX': round(ax, 2), 'accelY': round(ay, 2), 'accelZ': round(az, 2),
                # Position Data
                'posX': round(position['x'], 2), 
                'posY': round(position['y'], 2), 
                'posZ': round(position['z'], 2),
                # Rotation Data
                'rotX': round(rotation['x'], 2),
                'rotY': round(rotation['y'], 2),
                'rotZ': round(rotation['z'], 2),
                'lidarDistance': lidar_val,
                'barometricPressure': pressure
            })

        except Exception as e:
            print(f"Loop Error: {e}")

@socketio.on('connect')
def handle_connect():
    global last_time
    print('Client connected')
    last_time = time.time() # Reset timer to avoid huge DT jump
    socketio.start_background_task(target=background_thread)

@socketio.on('control_motor')
def handle_motor_control(message):
    if not gpio_active: return
    action = message.get('action')
    print(f"Motor Command: {action}")
    
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