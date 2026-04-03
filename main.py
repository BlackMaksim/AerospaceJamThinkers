from flask import Flask, render_template
from flask_socketio import SocketIO
import time
import csv
from datetime import datetime

# Import Sensors
from bmp180 import BMP180
from mpu6050 import mpu6050
from tfluna import TFLuna

import RPi.GPIO as GPIO
from picamera2 import Picamera2
import io
import base64

app = Flask(__name__)
app.config['SECRET_KEY'] = 'nasa-cant-crack-this-hehe'
# Allow multiple clients to connect (threading mode)
socketio = SocketIO(app, async_mode='threading')

print("INFO: Initializing hardware...")

# Barometer check
try:
    bmp = BMP180()
    print("SUCCESS: BMP180 initialized.")
except:
    bmp = None
    print("WARNING: BMP180 not found. Using mock.")

# MPU check
try:
    mpu = mpu6050(0x68)
    mpu.get_accel_data()
    print("SUCCESS: MPU-6050 initialized.")
except:
    mpu = None
    print("WARNING: MPU-6050 not found. Using mock.")

# LiDAR check
try:
    tfluna = TFLuna()
    tfluna.open()
    tfluna.set_samp_rate(10) 
    print("SUCCESS: TF-Luna initialized.")
except:
    tfluna = None
    print("WARNING: TF-Luna not found. Using mock.")

# Motors check
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
    
# Camera check
try:
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(camera_config)  
    picam2.start()                   
    ready = True
    print("SUCCESS: Camera initialized.")
except Exception as e:
    ready = False
    print(f"WARNING: Camera not found. {e}")

@app.route('/')
def index():
    return render_template('index.html')

# Global variables initialization
velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
rotation = {'x': 0.0, 'y': 0.0, 'z': 0.0}
prev_accel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
prev_gyro = {'x': 0.0, 'y': 0.0, 'z': 0.0}
gravity = 0.0
last_time = time.time()
thread_started = False 
video_streaming = False
alt = 0.0 # altitude
dist1 = dist2 = None # distance to target1 and target2
scanning = False
scan_data = []
lidar_val = None
pressure = None

def background_thread():
    global velocity, position, rotation, prev_accel, prev_gyro, last_time, gravity, lidar_val, pressure

    # Gravitation calibration
    print("Info: Drone Calibration... Do not touch the drone for 1 second!")
    
    if mpu:
        total = 0
        for i in range(30):  # Collect 30 samples for better accuracy
            a = mpu.get_accel_data()
            total += a['z']
            socketio.sleep(0.02)
        gravity = total / 30
        print(f"SUCCESS: Gravity(avg) = {gravity:.2f} m/s²")
    else:
        gravity = 9.81
        print("WARNING: MPU not found. Use standart fravity 9.81 m/s².")

    # Creating a log file each second
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
                   'RotX', 'RotY', 'RotZ', 'Altitude'] 
        log_writer.writerow(headers)
        print(f"INFO: High-Accuracy Logging started: {filename}")
    except:
        print("ERROR: Could not create log file.")

    #main loop
    while True:
        socketio.sleep(0.05) 
        
        current_sys_time = time.time()
        dt = current_sys_time - last_time
        last_time = current_sys_time

        try:
            # Sensors reading
            if bmp:
                pressure = bmp.get_pressure()/100
                alt = bmp.get_altitude()
            else:
                pressure = 1013.25
                alt = 0.0
                
            if mpu:
                accel = mpu.get_accel_data()
                gyro = mpu.get_gyro_data()
                ax, ay, az = accel['x'], accel['y'], accel['z']
                gx, gy, gz = gyro['x'], gyro['y'], gyro['z']
            else:
                ax, ay, az = 0.0, 0.0, 9.81
                gx, gy, gz = 0.0, 0.0, 0.0
                
            # Gravity Compensation (Z-axis)
            az_real = az - gravity
            
            # Filtering noises from the sensor
            accel_thresh = 0.2
            if abs(ax) < accel_thresh:
                ax = 0
            if abs(ay) < accel_thresh:
                ay = 0
            if abs(az_real) < accel_thresh:
                az_real = 0
            
            gyro_thresh = 1.0
            if abs(gx) < gyro_thresh:
                gx = 0
            if abs(gy) < gyro_thresh:
                gy = 0
            if abs(gz) < gyro_thresh:
                gz = 0
        	
            # Velocity Verlet integration 
            # Position update (physics formula: s = v₀t + ½at²)
            position['x'] += velocity['x'] * dt + 0.5 * prev_accel['x'] * (dt**2)
            position['y'] += velocity['y'] * dt + 0.5 * prev_accel['y'] * (dt**2)
            position['z'] += velocity['z'] * dt + 0.5 * prev_accel['z'] * (dt**2)

            # Update Velocity using average of previous and current acceleration (trapezoid integration)
            velocity['x'] += 0.5 * (prev_accel['x'] + ax) * dt
            velocity['y'] += 0.5 * (prev_accel['y'] + ay) * dt
            velocity['z'] += 0.5 * (prev_accel['z'] + az_real) * dt
            
            # Update Rotation using Trapezoidal rule for angles 
            rotation['x'] += 0.5 * (prev_gyro['x'] + gx) * dt
            rotation['y'] += 0.5 * (prev_gyro['y'] + gy) * dt
            rotation['z'] += 0.5 * (prev_gyro['z'] + gz) * dt

            # Store current values for the next loop
            prev_accel = {'x': ax, 'y': ay, 'z': az_real}
            prev_gyro = {'x': gx, 'y': gy, 'z': gz}

            # LiDAR reading
            if tfluna:
                lidar_val = round(tfluna.read()[0] * 100.0, 2)
                if scanning:
                    scan_data.append(lidar_val) 
            else:
                lidar_val = 0

            # logging
            t_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            if log_writer:
                log_writer.writerow([t_str, round(ax,2), round(ay,2), round(az_real,2),
                                    round(velocity['x'],2), round(velocity['y'],2), round(velocity['z'],2),
                                    round(position['x'],2), round(position['y'],2), round(position['z'],2),
                                    round(gx,2), round(gy,2), round(gz,2),
                                    round(rotation['x'],2), round(rotation['y'],2), round(rotation['z'],2), round(alt, 2)])
                log_file.flush() #Ctrl+S for drone :)
                
            # send all to UI
            socketio.emit('update_data', {
                'time': datetime.now().strftime("%H:%M:%S"),
                'accelX': round(ax, 2), 'accelY': round(ay, 2), 'accelZ': round(az_real, 2),
                'posX': round(position['x'], 2), 'posY': round(position['y'], 2), 'posZ': round(position['z'], 2),
                'rotX': round(rotation['x'], 2), 'rotY': round(rotation['y'], 2), 'rotZ': round(rotation['z'], 2),
                'lidarDistance': lidar_val,
                'barometricPressure': pressure,
                'altitude': round(alt, 2)
            })

        except Exception as e:
            print(f"Loop Error: {e}")

def camera_stream_thread():
    global video_streaming
    while True:
        socketio.sleep(0.5) # 2FPS
        
        if video_streaming and ready:
            try:
                stream = io.BytesIO()
                picam2.capture_file(stream, format='jpeg')
                stream.seek(0)
                image_data = stream.read()
                b64_image = base64.b64encode(image_data).decode('utf-8')
                socketio.emit('new_image', {'image_data': b64_image})
            except Exception as e:
                print(f"Camera stream error: {e}")

@socketio.on('toggle_video')
def handle_toggle_video():
    global video_streaming
    video_streaming = not video_streaming 
    print(f"Video streaming is now: {video_streaming}")

@socketio.on('connect')
def handle_connect():
    global last_time, thread_started
    print('Client connected')
    if not thread_started:
        last_time = time.time() # reset the counter
        socketio.start_background_task(target=background_thread)
        socketio.start_background_task(target=camera_stream_thread)
        thread_started = True

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

@socketio.on('request_image')
def handle_image_request():    
    if not ready:
        print("ERROR: Camera not available.")
        return
    try:
        stream = io.BytesIO()
        picam2.capture_file(stream, format='jpeg')
        stream.seek(0) # Ctrl+Home (go to the beginning for reading)
        image_data = stream.read()
        b64_image = base64.b64encode(image_data).decode('utf-8')
        socketio.emit('new_image', {'image_data': b64_image})
        print("Image sent to client.")
    except Exception as e:
        print(f"ERROR: Could not capture image. {e}")

@socketio.on('mark_target')
def handle_mark_target():
    global pressure
    if bmp and pressure is not None:
        print(f"Target marked! Pressure at target: {pressure:.2f} hPa")
        socketio.emit('target_marked', {'pressure': round(pressure, 2)})
    else:
        print("ERROR: BMP not available")

@socketio.on('read_lidar_tgt1')
def handle_read_lidar1():
    global dist1, lidar_val
    if tfluna:
        dist1 = lidar_val # distance to target1 in cm
        print(f"Distance to target1: {dist1:.2f} сm")
        socketio.emit('targetData', {'lidarDistance1': round(dist1, 2)})
    else:
        print("ERROR: TF-Luna not available")

@socketio.on('read_lidar_tgt2')
def handle_read_lidar2():
    global dist1, dist2 , lidar_val
    if tfluna:
        dist2 = lidar_val # distance to target2 in cm
        if dist1 is not None:
            total = dist1 + dist2
        else:
            total = dist2
        socketio.emit('targetData', {
            'lidarDistance2': round(dist2, 2),
            'lidarTotal': round(total, 2)
        })
        print(f"Target 2 marked {dist2:.2f} cm. Total distance: {total:.2f} cm")
    else:
        print("ERROR: TF-Luna not available")

@socketio.on('toggle_scan')
def handle_toggle_scan():
    global scanning, scan_data
    
    if not scanning:
        scan_data = [] # clear previous data
        scanning = True
        print("INFO: LiDAR scan started!")
    else:
        scanning = False
        print(f"INFO: LiDAR scan stopped!")
        socketio.emit('scan_results', {'data': scan_data})

def main():
    socketio.run(app, host='0.0.0.0', port=80, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    try:
        main()
    finally:
        if gpio_active: GPIO.cleanup()
