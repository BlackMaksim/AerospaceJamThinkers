# These two modules allow us to run a web server.
from flask import Flask, render_template
from flask_socketio import SocketIO
import random
# Import sensor libraries
from bmp180 import BMP180
from mpu6050 import mpu6050

app = Flask(__name__)
socketio = SocketIO(app)

# --- SENSOR INITIALIZATION ---

# Create an object to represent our BMP180 sensor
bmp = BMP180()

# Create an object for MPU-6050. 0x68 is the default I2C address.
mpu = mpu6050(0x68)

@app.route('/')
def index():
    return render_template('index.html')

def background_thread():
    while True:
        socketio.sleep(1) # Send data every 1 second

        # 1. Read Barometric Pressure
        barometricPressure = bmp.get_pressure()

        # 2. Read Accelerometer and Gyroscope Data
        # Returns a dictionary with 'x', 'y', and 'z' keys
        accel_data = mpu.get_accel_data()
        gyro_data = mpu.get_gyro_data()

        # Emit all data to the client
        socketio.emit(
            'update_data',
            {
                'randomNumber': random.randint(1, 100),
                'barometricPressure': barometricPressure,
                # Accelerometer data (m/s^2)
                'accelX': round(accel_data['x'], 2),
                'accelY': round(accel_data['y'], 2),
                'accelZ': round(accel_data['z'], 2),
                # Gyroscope data (degrees)
                'gyroX': round(gyro_data['x'], 2),
                'gyroY': round(gyro_data['y'], 2),
                'gyroZ': round(gyro_data['z'], 2)
            }
        )

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    socketio.start_background_task(target=background_thread)

# Interaction Event (from previous step)
@socketio.on('do_a_thing')
def do_a_thing(msg):
    print(msg['hello'])

def main():
    socketio.run(app, host='0.0.0.0', port=80, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    main()