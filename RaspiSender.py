import numpy as np
import socket
import pickle
import struct
import time
import picamera2

# Edit on each drone
ID = "10" # num1 = drone #; num2 = cam # in drone
ServerIp = "192.168.0.5"

if ID[1] == "0":
    from adxl345 import ADXL345

last_flag = None

if ID[1] == "0":
    adxl345 = ADXL345()

camera = picamera2.Picamera2()
camera.resolution = (720, 720)
camera.start()
time.sleep(2)

Socket = socket.socket()
Socket.connect((ServerIp, 8000))
Socket.recv(4096)

while True:
    # Get data
    if ID[1] == "0":
        axes = adxl345.getAxes(True)
        aX, aY, aZ = axes['x'], axes['y'], axes['z']
        Roll = np.arctan2(aY, aZ)
        Pitch = np.arctan2(-aX, np.sqrt(aY**2 + aZ**2))
    else:
        Roll, Pitch = None, None
    
    image = camera.capture_array("main")
    
    # Send Data
    data = pickle.dumps([image, [Roll, Pitch], ID])
    Socket.send(struct.pack('<l', len(data)) + data)
    
    Socket.recv(4096)