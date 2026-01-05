# These two modules allow us to run a web server.
from flask import Flask, render_template
from flask_socketio import SocketIO
import random
import time

# Import sensor libraries provided by SDK
from bmp180 import BMP180
from mpu6050 import mpu6050
# Add TF-Luna library import:
from tfluna import TFLuna

app = Flask(__name__)
socketio = SocketIO(app)

# --- SENSOR INITIALIZATION ---

print("INFO: Initializing hardware...")

# 1. Barometer
bmp = BMP180()

# 2. Accelerometer/Gyroscope
mpu = mpu6050(0x68)

# 3. LiDAR (TF-Luna) - Setup according to docs
tfluna = TFLuna()        # Create object
tfluna.open()            # Open connection
tfluna.set_samp_rate(5)  # Set to 5Hz as recommended in docs

@app.route('/')
def index():
    return render_template('index.html')

def background_thread():
    while True:
        socketio.sleep(1) 

        try:
            # --- READ SENSORS ---
            
            # BMP180
            pressure = bmp.get_pressure()
            
            # MPU-6050
            accel_data = mpu.get_accel_data()
            gyro_data = mpu.get_gyro_data()
            
            # TF-Luna
            # This uses special syntax called a "tuple" to set all three variables
            distance, strength, temperature = tfluna.read()

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
                    
                    # LiDAR Data (Convert to cm for display)
                    'lidarDistance': round(distance * 100.0, 2)
                }
            )
        except Exception as e:
            print(f"Sensor Error: {e}")

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    socketio.start_background_task(target=background_thread)

# Interaction Event
@socketio.on('do_a_thing')
def do_a_thing(msg):
    print(msg['hello'])

def main():
    socketio.run(app, host='0.0.0.0', port=80, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    main()