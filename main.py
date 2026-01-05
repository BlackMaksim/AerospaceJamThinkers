# These two modules allow us to run a web server.
from flask import Flask, render_template
from flask_socketio import SocketIO
# This module lets us pick random numbers.
import random
# Add this line to import the BMP180 library:
from bmp180 import BMP180

app = Flask(__name__)
# Security key (added for safety, though not explicitly in basic tutorial, it's good practice)
app.config['SECRET_KEY'] = 'aerospace-jam-secret'
socketio = SocketIO(app)

# Create an object to represent our BMP180 sensor
bmp = BMP180()

@app.route('/')
def index():
    return render_template('index.html')

# This function runs in the background...
def background_thread():
    while True:
        socketio.sleep(1)

        # First, we ask our sensor object for the current pressure
        barometricPressure = bmp.get_pressure()

        # Now, we add it to the message we're sending
        socketio.emit(
            'update_data',
            {
                'randomNumber': random.randint(1, 100),
                # Add a new pair for our pressure data
                'barometricPressure': barometricPressure
            }
        )

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    socketio.start_background_task(target=background_thread)

def main():
    socketio.run(app, host='0.0.0.0', port=80, allow_unsafe_werkzeug=True)

if __name__ == '__main__':
    main()