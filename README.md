# Aerospace Jam: The Thinkers Ground Control
This is the official code for our NASA Aerospace Jam drone project. We built this Ground Control Station to talk to the hardware on the drone and show us what's happening in the air.

Everything is built with Python, Flask, and Socket.IO to get real-time data without the UI lagging.

What this thing actually does:
Position Tracking: We using the MPU-6050 and some math to calculate where the drone is and how it’s rotating.

Environment Data: It reads barometric pressure from the BMP180 to find our altitude and uses TF-Luna LiDAR for measuring distances.

Mission Control UI: A live web dashboard that shows graphs of all flight data, lets us control the payload drop motors, and streams video from the Picamera2.

Logging: It automatically saves every single MPU and BMP reading into a CSV file so we can analyze the flight later.

How to Run:
Turn on the Pi: Make sure it’s in Development Mode (for testing) or Competition Mode (for the actual flight).

Open the code: Go to the teamCode folder in Thonny on the Raspberry Pi.

Run it: Just hit run on main.py.

Connect your laptop: Join the Pi’s network, open your browser, and go to:

Development: http://172.20.10.6 (Or if you using another pi you can have different IP)

Competition: http://10.42.0.1/
