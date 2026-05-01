#This code has the fixes for the camera

# central_script_altV1.1.py

import cv2
import numpy as np
import paho.mqtt.client as mqtt
import threading
import time
import json
import math
import sys
import base64
import pynmea2
import os
from datetime import datetime
import serial
import csv

#.py files
import IMU  # Importing the IMU module
import GUIV2

from flask import Flask, Response, jsonify, request, render_template_string, send_from_directory
import logging
from multiprocessing import Process, Queue, Event
from face_tracking import face_tracking_process
from auto_navigation import auto_navigation_process

# MQTT Configuration
MQTT_SERVER = "192.168.1.145"  # Update if different
MQTT_PORT = 1883

MQTT_TOPIC_COMMAND = "robot/control"
MQTT_RAIL_TOPIC_COMMAND = "robot/rail"
MQTT_TOPIC_DETECTIONS = "robot/detections"
MQTT_TOPIC_CAMERA = "robot/camera"
MQTT_TOPIC_PUMP = "robot/pump"
MQTT_TOPIC_REMOTE_PUMP = "robot/remotepump"
MQTT_TOPIC_IMU = "imu/data"
MQTT_TOPIC_DATA = "moisture/data"

# Used to return data back to the server
MQTT_TOPIC_GPS_OUT = "robot/telemetry/gps"
MQTT_TOPIC_IMU_OUT = "robot/telemetry/imu"

# ─────────────────────────────────────────────────────────────────────────────
# CLOUD MQTT Configuration
# ─────────────────────────────────────────────────────────────────────────────
CLOUD_MQTT_SERVER   = "100.119.46.15"   # Replace with real IP/hostname
CLOUD_MQTT_PORT     = 1883
CLOUD_MQTT_USERNAME = None              # Set to your username
CLOUD_MQTT_PASSWORD = None              # Set to your password

# Cloud topics the laptop will publish to — Pi will subscribe and relay them
CLOUD_TOPIC_CONTROL = "cloud/robot/control"
CLOUD_TOPIC_RAIL    = "cloud/robot/rail"
CLOUD_TOPIC_PUMP    = "cloud/robot/pump"

# Cloud telemetry topics — Pi publishes robot data back to cloud client
CLOUD_TOPIC_STATUS   = "cloud/robot/status"
CLOUD_TOPIC_GPS_OUT  = "cloud/robot/gps"
CLOUD_TOPIC_CAM_OUT  = "cloud/robot/camera"
CLOUD_TOPIC_IMU_OUT  = "cloud/robot/imu"

app = Flask(__name__, static_folder='static')

# Global variables
latest_detection = None
latest_camera_frame = None  # Always holds the most recent USB camera frame
camera_frame_lock = threading.Lock()  # Protects latest_camera_frame
output_frame = None
lock = threading.Lock()
e_stop_active = False  # E-Stop state
moisture_threshold = 100  # For pump control with moisture
cloud_client_global = None  # Reference to cloud MQTT client for telemetry forwarding

# GPS and heading data
current_lat, current_lon = None, None
robot_heading = 0.0
gps_data = []
gps_data_lock = threading.Lock()

# PID Controller Parameters
w, h = 640, 480  # Frame dimensions for visualization (can be adjusted)
center = w // 2

# Configuration Flags
ENABLE_FRAME_FLIP = True  # Set to False to disable frame flipping
INVERT_YAW_CONTROL = False  # Set to True if robot moves opposite to desired direction

# Mode Control
current_mode = 'basic_movement'  # Default mode

# Queues for inter-process communication
command_queue = Queue()
detection_queue = Queue()
gps_data_queue = Queue()
imu_queue = Queue()

# Events to control processes
stop_event = Event()

# CSV section
filename = 'moisture_data.csv'
file_exists = os.path.isfile(filename)

csv_file = open(filename, mode='a', newline='')
writer = csv.writer(csv_file)
if not file_exists:
    writer.writerow(['Timestamp', 'Mac Address', 'Data'])


# Mainly for the Remote Pump Controls
zone_A_macs = {"C0:49:EF:69:BF:DC", "BB:BB:BB:BB:BB:BB"}
zone_B_macs = {"AA:AA:AA:AA:AA:AA", "BB:BB:BB:BB:BB:BB"}
zone_C_macs = {"AA:AA:AA:AA:AA:AA", "BB:BB:BB:BB:BB:BB"}

# Independent Thresholds for moisture
zone_threshold = {
    "A": 10,
    "B": 10,
    "C": 10
}
zone_macs = {
    "A": zone_A_macs,
    "B": zone_B_macs,
    "C": zone_C_macs
}

pump_states = {
    "A": False,
    "B": False,
    "C": False
}


def on_message(client, userdata, msg):
    try:
        if msg.topic == MQTT_TOPIC_DETECTIONS:
            detection_data = json.loads(msg.payload.decode())
            detection_queue.put(detection_data)
        elif msg.topic == MQTT_TOPIC_CAMERA:
            camera_data = json.loads(msg.payload.decode())
            image_b64 = camera_data.get('image', '')
            if image_b64:
                image_bytes = base64.b64decode(image_b64)
                np_arr = np.frombuffer(image_bytes, np.uint8)
                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if image is not None:
                    with camera_frame_lock:
                        latest_camera_frame = image
            
        elif msg.topic == MQTT_TOPIC_DATA:
            sensor_data = json.loads(msg.payload.decode())
            mac = sensor_data.get("mac")
            value = sensor_data.get("value")

            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            writer.writerow([timestamp, mac, value])
            csv_file.flush()

            cmd_value = int(value)

            zone = None
            for z, macs in zone_macs.items():
                if mac in macs:
                    zone = z
                    break

            if zone:
                threshold = zone_threshold[zone]
                if value < threshold:
                    pump_cmd = f"{zone} 1"
                    client.publish(MQTT_TOPIC_REMOTE_PUMP, pump_cmd)
                    pump_states[zone] = True
                elif value >= threshold:
                    pump_cmd = f"{zone} 0"
                    client.publish(MQTT_TOPIC_REMOTE_PUMP, pump_cmd)
                    pump_states[zone] = False

    except Exception as e:
        print(f"Error handling message on topic {msg.topic}: {e}")


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        topics = [
            MQTT_TOPIC_DETECTIONS,
            MQTT_TOPIC_CAMERA,
            MQTT_TOPIC_DATA,
            MQTT_TOPIC_IMU,
        ]
        for topic in topics:
            client.subscribe(topic)
    else:
        pass


# ─────────────────────────────────────────────────────────────────────────────
# CLOUD MQTT — Bridge functions
# ─────────────────────────────────────────────────────────────────────────────
def cloud_on_connect(cloud_client, userdata, flags, rc):
    if rc == 0:
        print("[CLOUD MQTT] Connected to cloud broker successfully.")
        cloud_client.subscribe(CLOUD_TOPIC_CONTROL)
        cloud_client.subscribe(CLOUD_TOPIC_RAIL)
        cloud_client.subscribe(CLOUD_TOPIC_PUMP)
        print(f"[CLOUD MQTT] Subscribed to: {CLOUD_TOPIC_CONTROL}, "
              f"{CLOUD_TOPIC_RAIL}, {CLOUD_TOPIC_PUMP}")
    else:
        print(f"[CLOUD MQTT] Failed to connect. Return code: {rc}")


def cloud_on_disconnect(cloud_client, userdata, rc):
    print(f"[CLOUD MQTT] Disconnected (rc={rc}). Will attempt reconnect...")


def cloud_on_message(cloud_client, userdata, msg):
    """
    Receives commands published by the laptop on the cloud broker and
    forwards them to the local broker so the robot acts on them.

    Cloud topic          ->  Local topic
    ─────────────────────────────────────
    cloud/robot/control  ->  robot/control
    cloud/robot/rail     ->  robot/rail
    cloud/robot/pump     ->  robot/pump
    """
    try:
        payload = msg.payload.decode()
        print(f"[CLOUD MQTT] Received on '{msg.topic}': {payload}")

        if msg.topic == CLOUD_TOPIC_CONTROL:
            client.publish(MQTT_TOPIC_COMMAND, payload)
            print(f"[CLOUD MQTT] Relayed to local '{MQTT_TOPIC_COMMAND}': {payload}")

        elif msg.topic == CLOUD_TOPIC_RAIL:
            client.publish(MQTT_RAIL_TOPIC_COMMAND, payload)
            print(f"[CLOUD MQTT] Relayed to local '{MQTT_RAIL_TOPIC_COMMAND}': {payload}")

        elif msg.topic == CLOUD_TOPIC_PUMP:
            client.publish(MQTT_TOPIC_PUMP, payload)
            print(f"[CLOUD MQTT] Relayed to local '{MQTT_TOPIC_PUMP}': {payload}")

    except Exception as e:
        print(f"[CLOUD MQTT] Error handling cloud message: {e}")


def cloud_bridge_thread():
    """
    Runs forever in a daemon thread.
    Connects to the cloud broker and keeps the connection alive with automatic
    reconnect logic so the robot is always reachable from the internet.
    """
    global cloud_client_global

    cloud_client = mqtt.Client(client_id="rpi_cloud_bridge")

    if CLOUD_MQTT_USERNAME and CLOUD_MQTT_PASSWORD:
        cloud_client.username_pw_set(CLOUD_MQTT_USERNAME, CLOUD_MQTT_PASSWORD)

    cloud_client.on_connect    = cloud_on_connect
    cloud_client.on_disconnect = cloud_on_disconnect
    cloud_client.on_message    = cloud_on_message

    while True:
        try:
            print(f"[CLOUD MQTT] Connecting to {CLOUD_MQTT_SERVER}:{CLOUD_MQTT_PORT} ...")
            cloud_client.connect(CLOUD_MQTT_SERVER, CLOUD_MQTT_PORT, keepalive=60)
            cloud_client_global = cloud_client
            cloud_client.loop_forever()

        except Exception as e:
            cloud_client_global = None
            print(f"[CLOUD MQTT] Connection error: {e}. Retrying in 10 seconds...")
            time.sleep(10)


# Initialize MQTT Client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_SERVER, MQTT_PORT, 60)
client.loop_start()

# Initialize IMU
IMU.detectIMU()
if IMU.BerryIMUversion == 99:
    print("No BerryIMU found... exiting")
    sys.exit()
IMU.initIMU()


def calculate_heading():
    """
    Calculate the robot's heading using IMU readings.
    Returns the heading in degrees.
    """
    ACCx = IMU.readACCx()
    ACCy = IMU.readACCy()
    ACCz = IMU.readACCz()
    MAGx = IMU.readMAGx()
    MAGy = IMU.readMAGy()
    MAGz = IMU.readMAGz()

    acc_magnitude = math.sqrt(ACCx ** 2 + ACCy ** 2 + ACCz ** 2)
    if acc_magnitude == 0:
        return 0
    accXnorm = ACCx / acc_magnitude
    accYnorm = ACCy / acc_magnitude

    pitch = math.asin(accXnorm)
    if math.cos(pitch) == 0:
        roll = 0
    else:
        roll = -math.asin(accYnorm / math.cos(pitch))

    magXcomp = MAGx * math.cos(pitch) + MAGz * math.sin(pitch)
    magYcomp = (MAGx * math.sin(roll) * math.sin(pitch) +
               MAGy * math.cos(roll) -
               MAGz * math.sin(roll) * math.cos(pitch))

    heading = math.degrees(math.atan2(magYcomp, magXcomp))
    if heading < 0:
        heading += 360

    return heading, math.degrees(pitch), math.degrees(roll)

# ─────────────────────────────────────────────────────────────────────────────
# VFan USB GPS — Reads NMEA sentences directly from the serial port.
# Removes the gpsd dependency entirely. Auto-detects the port by trying
# ttyACM0 first (most common for VFan), then falling back to ttyUSB0/1.
# ─────────────────────────────────────────────────────────────────────────────
GPS_PORT_CANDIDATES = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
GPS_BAUD = 9600

def gps_serial_thread():
    """
    Reads NMEA sentences from the VFan USB GPS receiver continuously.
    Parses GPRMC and GNRMC sentences to extract lat/lon and updates
    current_lat / current_lon globals for the rest of the app.
    """
    global current_lat, current_lon

    port = None
    for candidate in GPS_PORT_CANDIDATES:
        try:
            ser = serial.Serial(candidate, baudrate=GPS_BAUD, timeout=1)
            port = candidate
            print(f"[GPS] Opened VFan GPS on {port} at {GPS_BAUD} baud")
            break
        except Exception:
            continue

    if port is None:
        print(f"[GPS] ERROR: Could not open GPS on any of {GPS_PORT_CANDIDATES}")
        return

    while True:
        try:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if not line:
                continue
            # Parse any RMC sentence (GPRMC or GNRMC both contain position)
            if 'RMC' in line or 'GGA' in line:
                try:
                    msg = pynmea2.parse(line)
                    # RMC gives lat/lon when status == 'A' (active/valid fix)
                    if hasattr(msg, 'status') and msg.status == 'A':
                        current_lat = msg.latitude
                        current_lon = msg.longitude
                    # GGA gives lat/lon when gps_qual > 0
                    elif hasattr(msg, 'gps_qual') and msg.gps_qual and int(msg.gps_qual) > 0:
                        current_lat = msg.latitude
                        current_lon = msg.longitude
                except pynmea2.ParseError:
                    pass
        except Exception as e:
            print(f"[GPS] Read error: {e}. Retrying...")
            time.sleep(1)

def receive_gps_data():
    """Returns the most recent lat/lon read by gps_serial_thread."""
    return current_lat, current_lon

# ─────────────────────────────────────────────────────────────────────────────
# USB CAMERA — Writes the latest frame directly to latest_camera_frame.
# Using a single shared variable instead of a queue eliminates backlog buildup
# which was the main cause of lag. CAP_PROP_BUFFERSIZE=1 stops OpenCV from
# internally queuing stale frames too.
# ─────────────────────────────────────────────────────────────────────────────
def camera_capture_thread():
    global latest_camera_frame
    cap = cv2.VideoCapture(0)  # 0 = first USB camera; change if needed
    if not cap.isOpened():
        return
    # FIX: Minimize internal OpenCV buffer so we always read the newest frame
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    while True:
        ret, frame = cap.read()
        if ret:
            with camera_frame_lock:
                latest_camera_frame = frame

def main_loop():
        global latest_detection, latest_camera_frame
        global output_frame, lock, e_stop_active
        global current_lat, current_lon, robot_heading, gps_data
        global current_mode
        check = False

        # Initialize last_img with a black image
        last_img = np.zeros((h, w, 3), dtype=np.uint8)

        try:
            while True:
                # FIX: Read latest frame directly from shared variable — no queue backlog
                with camera_frame_lock:
                    if latest_camera_frame is not None:
                        last_img = latest_camera_frame.copy()

                # Use the last available frame
                img = last_img.copy()

                # Optionally flip the frame
                if ENABLE_FRAME_FLIP:
                    img = cv2.flip(img, 1)

                # Display mode on the frame
                cv2.putText(img, f"Mode: {current_mode}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Optionally display IMU heading
                imu_heading, imu_pitch, imu_roll = calculate_heading()
                cv2.putText(img, f"IMU Heading: {imu_heading:.2f}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # Optionally display E-Stop status
                if e_stop_active:
                    cv2.putText(img, "E-STOP ACTIVE!", (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                # FIX: Update output_frame once at the end of the loop, not twice
                with lock:
                    output_frame = img.copy()

                # Update GPS data
                current_lat, current_lon = receive_gps_data()
                if current_lat is not None and current_lon is not None:
                    with gps_data_lock:
                        gps_data.append({
                            'GPS_Lat': current_lat,
                            'GPS_Lon': current_lon,
                            'Heading': imu_heading
                        })
                        # Keep only the last 500 points to prevent memory leak
                        if len(gps_data) > 500:
                            gps_data = gps_data[-500:]

                # ─────────────────────────────────────────────────────────────
                # Publish telemetry to local broker and forward to cloud client
                if current_lat is not None:
                    # Formatted as strings to match React frontend requirements
                    gps_payload = json.dumps({
                        "lat": str(current_lat),
                        "lon": str(current_lon),
                        "alt": "0.0" # Hardcoded unless your GPS reads altitude
                    })
                    client.publish(MQTT_TOPIC_GPS_OUT, gps_payload)
                    if cloud_client_global and cloud_client_global.is_connected():
                        cloud_client_global.publish(CLOUD_TOPIC_GPS_OUT, gps_payload)

                # Safely check if your IMU.py has Gyroscope reading functions
                try:
                    gx, gy, gz = str(IMU.readGYRx()), str(IMU.readGYRy()), str(IMU.readGYRz())
                except AttributeError:
                    gx = gy = gz = "0.00"

                # Send nested JSON payload exactly as the frontend expects
                imu_payload = json.dumps({
                    "acc": {"x": str(IMU.readACCx()), "y": str(IMU.readACCy()), "z": str(IMU.readACCz())},
                    "gyro": {"x": gx, "y": gy, "z": gz},
                    "roll": str(round(imu_roll, 2)),
                    "pitch": str(round(imu_pitch, 2)),
                    "yaw": str(round(imu_heading, 2))
                })
                
                client.publish(MQTT_TOPIC_IMU_OUT, imu_payload)
                if cloud_client_global and cloud_client_global.is_connected():
                    cloud_client_global.publish(CLOUD_TOPIC_IMU_OUT, imu_payload)
                
                # NEW: Publish a plaintext status message for your dashboard UI
                status_msg = f"Mode: {current_mode} | E-Stop: {'ACTIVE' if e_stop_active else 'Off'}"
                #client.publish(MQTT_TOPIC_STATUS, status_msg)
                if cloud_client_global and cloud_client_global.is_connected():
                    cloud_client_global.publish(CLOUD_TOPIC_STATUS, status_msg)
                # ─────────────────────────────────────────────────────────────

                # Check for e-stop activation
                if e_stop_active:
                    front_back_command = 64  # Stop
                    side_side_command = 64   # Neutral steering
                    command_string = f"{front_back_command} {side_side_command}"
                    client.publish(MQTT_TOPIC_COMMAND, command_string)
                    with lock:
                        output_frame = img.copy()
                    time.sleep(0.1)
                    continue

                # Handle different modes
                if current_mode == 'face_tracking':
                    check = False
                    if not detection_queue.empty():
                        detection = detection_queue.get()
                        command_queue.put(('face_tracking', detection))
                    else:
                        front_back_command = 64
                        side_side_command = 64
                        command_string = f"{front_back_command} {side_side_command}"
                        client.publish(MQTT_TOPIC_COMMAND, command_string)
                elif current_mode == 'auto_navigation':
                    check = False
                    pass
                elif current_mode == 'basic_movement':
                    check = True
                else:
                    front_back_command = 64
                    side_side_command = 64
                    check = False
                    command_string = f"{front_back_command} {side_side_command}"
                    client.publish(MQTT_TOPIC_COMMAND, command_string)

                with lock:
                    output_frame = img.copy()

                time.sleep(0.03)

        except Exception as e:
            print(f"An error occurred: {e}")

        finally:
            front_back_command = 64
            side_side_command = 64
            command_string = f"{front_back_command} {side_side_command}"
            client.publish(MQTT_TOPIC_COMMAND, command_string)
            client.loop_stop()
            client.disconnect()

def generate():
    # FIX: Read directly from latest_camera_frame so the stream is not
    # bottlenecked by main_loop processing time (GPS, IMU, MQTT etc.)
    while True:
        with camera_frame_lock:
            frame = latest_camera_frame
        if frame is None:
            time.sleep(0.01)
            continue
        if ENABLE_FRAME_FLIP:
            frame = cv2.flip(frame, 1)
        (flag, encoded_image) = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not flag:
            continue
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
              bytearray(encoded_image) + b'\r\n')


html_content = GUIV2.html_content

@app.route("/")
def index():
    return render_template_string(html_content)

@app.route("/video_feed")
def video_feed():
    return Response(generate(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/estop", methods=['POST'])
def estop():
    global e_stop_active
    e_stop_active = True
    front_back_command = 64
    side_side_command = 64
    command_string = f"{front_back_command} {side_side_command}"
    client.publish(MQTT_TOPIC_COMMAND, command_string)
    return jsonify({"status": "E-Stop activated"})

@app.route("/undo_estop", methods=['POST'])
def undo_estop():
    global e_stop_active
    e_stop_active = False
    return jsonify({"status": "E-Stop deactivated"})

@app.route("/increase_face_area", methods=['POST'])
def increase_face_area():
    command_queue.put(('increase_face_area', None))
    return jsonify({"status": "Face area increased"})

@app.route("/decrease_face_area", methods=['POST'])
def decrease_face_area():
    command_queue.put(('decrease_face_area', None))
    return jsonify({"status": "Face area decreased"})

@app.route("/move_center_left", methods=['POST'])
def move_center_left():
    command_queue.put(('move_center_left', None))
    return jsonify({"status": "Center moved left"})

@app.route("/move_center_right", methods=['POST'])
def move_center_right():
    command_queue.put(('move_center_right', None))
    return jsonify({"status": "Center moved right"})

@app.route('/get_gps_data', methods=['GET'])
def get_gps_data_route():
    with gps_data_lock:
        if gps_data:
            sanitized_gps_data = [{
                'GPS_Lat': float(item.get('GPS_Lat', 0)),
                'GPS_Lon': float(item.get('GPS_Lon', 0)),
                'Heading': float(item.get('Heading', 0)),
                'Estimated_Lat': float(item.get('Estimated_Lat', 0)),
                'Estimated_Lon': float(item.get('Estimated_Lon', 0)),
                'Estimated_Theta': float(item.get('Estimated_Theta', 0))
            } for item in gps_data]
        else:
            sanitized_gps_data = []
    return jsonify(sanitized_gps_data)

@app.route('/initial_gps', methods=['GET'])
def initial_gps():
    lat, lon = receive_gps_data()
    if lat is not None and lon is not None:
        return jsonify({"lat": lat, "lon": lon})
    else:
        return jsonify({"lat": 0.0, "lon": 0.0})

@app.route("/set_mode", methods=['POST'])
def set_mode():
    global current_mode, stop_event
    data = request.get_json()
    mode = data.get('mode', 'basic_movement')
    if mode in ['basic_movement', 'auto_navigation', 'face_tracking']:
        current_mode = mode
        if current_mode == 'auto_navigation':
            stop_event.clear()
            auto_nav_proc = Process(target=auto_navigation_process, args=(command_queue, client, stop_event))
            auto_nav_proc.start()
        else:
            stop_event.set()
        return jsonify({"status": f"Mode set to {current_mode}"})
    else:
        return jsonify({"status": "Invalid mode selected"}), 400

@app.route("/move_forward", methods=['POST'])
def move_forward():
    global current_mode
    if current_mode == 'basic_movement':
        front_back_command = 126
        side_side_command = 64
        command_string = f"{front_back_command} {side_side_command}"
        client.publish(MQTT_TOPIC_COMMAND, command_string)
        return jsonify({"status": "Moving forward"})
    else:
        return jsonify({"status": "Cannot move in current mode"}), 400

@app.route("/move_rail_forward", methods=['POST'])
def move_rail_forward():
    global current_mode
    if current_mode == 'basic_movement':
        move_command = 0
        command_string = f"{move_command}"
        client.publish(MQTT_RAIL_TOPIC_COMMAND, command_string)
        return jsonify({"status": "Moving rail forward"})
    else:
        return jsonify({"status": "Cannot move in current mode"}), 400

@app.route("/move_backward", methods=['POST'])
def move_backward():
    global current_mode
    if current_mode == 'basic_movement':
        front_back_command = 0
        side_side_command = 64
        command_string = f"{front_back_command} {side_side_command}"
        client.publish(MQTT_TOPIC_COMMAND, command_string)
        return jsonify({"status": "Moving backward"})
    else:
        return jsonify({"status": "Cannot move in current mode"}), 400

@app.route("/move_rail_backward", methods=['POST'])
def move_rail_backward():
    global current_mode
    if current_mode == 'basic_movement':
        move_command = 126
        command_string = f"{move_command}"
        client.publish(MQTT_RAIL_TOPIC_COMMAND, command_string)
        return jsonify({"status": "Moving rail backward"})
    else:
        return jsonify({"status": "Cannot move in current mode"}), 400

@app.route("/move_left", methods=['POST'])
def move_left():
    global current_mode
    if current_mode == 'basic_movement':
        front_back_command = 64
        side_side_command = 126
        command_string = f"{front_back_command} {side_side_command}"
        client.publish(MQTT_TOPIC_COMMAND, command_string)
        return jsonify({"status": "Turning left"})
    else:
        return jsonify({"status": "Cannot move in current mode"}), 400

@app.route("/move_right", methods=['POST'])
def move_right():
    global current_mode
    if current_mode == 'basic_movement':
        front_back_command = 64
        side_side_command = 0
        command_string = f"{front_back_command} {side_side_command}"
        client.publish(MQTT_TOPIC_COMMAND, command_string)
        return jsonify({"status": "Turning right"})
    else:
        return jsonify({"status": "Cannot move in current mode"}), 400

@app.route("/stop_robot", methods=['POST'])
def stop_robot():
    global current_mode
    if current_mode == 'basic_movement':
        front_back_command = 64
        side_side_command = 64
        command_string = f"{front_back_command} {side_side_command}"
        client.publish(MQTT_TOPIC_COMMAND, command_string)
        return jsonify({"status": "Robot stopped"})
    else:
        return jsonify({"status": "Cannot stop in current mode"}), 400

@app.route("/pump_on", methods=['POST'])
def pump_on():
    global current_mode
    if current_mode == 'basic_movement':
        control_command = 1
        command_string = f"{control_command}"
        client.publish(MQTT_TOPIC_PUMP, command_string)
        return jsonify({"status": "Pump ON"})
    else:
        return jsonify({"status": "Cannot stop in current mode"}), 400

@app.route("/pump_off", methods=['POST'])
def pump_off():
    global current_mode
    if current_mode == 'basic_movement':
        control_command = 0
        command_string = f"{control_command}"
        client.publish(MQTT_TOPIC_PUMP, command_string)
        return jsonify({"status": "Pump OFF"})
    else:
        return jsonify({"status": "Cannot stop in current mode"}), 400

@app.route("/stop_rail", methods=['POST'])
def stop_rail():
    global current_mode
    if current_mode == 'basic_movement':
        move_command = 64
        command_string = f"{move_command}"
        client.publish(MQTT_RAIL_TOPIC_COMMAND, command_string)
        return jsonify({"status": "Rail stopped"})
    else:
        return jsonify({"status": "Cannot stop in current mode"}), 400

@app.route("/update_pid", methods=['POST'])
def update_pid():
    data = request.get_json()
    kp = data.get('kp')
    ki = data.get('ki')
    kd = data.get('kd')

    if kp is not None and ki is not None and kd is not None:
        command_queue.put(('update_pid', (kp, ki, kd)))
        return jsonify({"status": "PID parameters updated"})
    else:
        return jsonify({"status": "Invalid PID parameters"}), 400

@app.route('/send_coordinates', methods=['POST'])
def receive_coordinates():
    data = request.get_json()
    coordinates = data.get('coordinates', [])
    if coordinates:
        command_queue.put(('set_waypoints', coordinates))
        return jsonify({"status": "Coordinates received"})
    else:
        return jsonify({"status": "No coordinates received"}), 400

# FIX: Moved above app.run() so Flask registers this route correctly
@app.route('/get_moisture', methods=['GET'])
def get_moisture():
    try:
        with open('moisture_data.csv', 'r') as f:
            lines = f.readlines()
        if len(lines) > 1:
            latest = lines[-1].strip().split(',')
            return jsonify({"value": latest[2]})
        else:
            return jsonify({"value": "No data"})
    except Exception as e:
        return jsonify({"value": "Error"})

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR, format='%(asctime)s - %(levelname)s - %(message)s')

    # Start the cloud MQTT bridge — completely independent daemon thread
    cloud_thread = threading.Thread(target=cloud_bridge_thread, name="CloudMQTTBridge")
    cloud_thread.daemon = True
    cloud_thread.start()

    # Start the USB camera capture thread
    cam_thread = threading.Thread(target=camera_capture_thread, name="USBCamera")
    cam_thread.daemon = True
    cam_thread.start()

    # Start the VFan GPS serial thread
    gps_thread = threading.Thread(target=gps_serial_thread, name="VFanGPS")
    gps_thread.daemon = True
    gps_thread.start()

    # Start the main loop thread
    t = threading.Thread(target=main_loop)
    t.daemon = True
    t.start()

    # Start face tracking process
    face_track_proc = Process(target=face_tracking_process, args=(command_queue, client))
    face_track_proc.start()

    # Run the Flask app
    app.run(host='0.0.0.0', port=5000)
