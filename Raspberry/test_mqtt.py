import paho.mqtt.client as mqtt
import time
import json
import threading
import math
import sys
import cv2
import serial
import pynmea2
import IMU
import base64

# ─────────────────────────────────────────────────────────────────────────────
# CLOUD MQTT Configuration
# ─────────────────────────────────────────────────────────────────────────────
CLOUD_MQTT_SERVER   = "100.119.46.15"   
CLOUD_MQTT_PORT     = 1883
CLOUD_MQTT_USERNAME = None              
CLOUD_MQTT_PASSWORD = None              

CLOUD_TOPIC_STATUS  = "cloud/robot/status"
CLOUD_TOPIC_GPS     = "cloud/robot/gps"
CLOUD_TOPIC_CAMERA  = "cloud/robot/camera"
CLOUD_TOPIC_IMU     = "cloud/robot/imu"
CLOUD_TOPIC_CMD     = "cloud/robot/cmd"

# ─────────────────────────────────────────────────────────────────────────────
# HARDWARE GLOBALS & THREADS
# ─────────────────────────────────────────────────────────────────────────────
current_lat, current_lon = None, None
latest_camera_frame = None
camera_frame_lock = threading.Lock()

# 1. GPS Thread
GPS_PORT_CANDIDATES = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
GPS_BAUD = 9600

def gps_serial_thread():
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

            if 'RMC' in line or 'GGA' in line:
                try:
                    msg = pynmea2.parse(line)
                    if hasattr(msg, 'status') and msg.status == 'A':
                        current_lat = msg.latitude
                        current_lon = msg.longitude
                    elif hasattr(msg, 'gps_qual') and msg.gps_qual and int(msg.gps_qual) > 0:
                        current_lat = msg.latitude
                        current_lon = msg.longitude
                except pynmea2.ParseError:
                    pass
        except Exception as e:
            time.sleep(1)

# 2. Camera Thread
def camera_capture_thread():
    global latest_camera_frame
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("[CAMERA] ERROR: Could not open USB camera.")
        return
    
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 24)
    
    # Try to force Auto-Exposure ON (For V4L2, '3' usually means Auto, '1' means Manual)
    #cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3) 
    # Force a higher default brightness (usually a scale from 0 to 255)
    #cap.set(cv2.CAP_PROP_BRIGHTNESS, 150)

    print("[CAMERA] Active and reading frames.")
    while True:
        ret, frame = cap.read()
        if ret:
            with camera_frame_lock:
                latest_camera_frame = frame

# 3. IMU Helper
def calculate_heading():
    ACCx, ACCy, ACCz = IMU.readACCx(), IMU.readACCy(), IMU.readACCz()
    MAGx, MAGy, MAGz = IMU.readMAGx(), IMU.readMAGy(), IMU.readMAGz()

    acc_magnitude = math.sqrt(ACCx ** 2 + ACCy ** 2 + ACCz ** 2)
    if acc_magnitude == 0:
        return 0, 0, 0
    
    accXnorm = ACCx / acc_magnitude
    accYnorm = ACCy / acc_magnitude

    pitch = math.asin(accXnorm)
    roll = 0 if math.cos(pitch) == 0 else -math.asin(accYnorm / math.cos(pitch))

    magXcomp = MAGx * math.cos(pitch) + MAGz * math.sin(pitch)
    magYcomp = (MAGx * math.sin(roll) * math.sin(pitch) +
               MAGy * math.cos(roll) - MAGz * math.sin(roll) * math.cos(pitch))

    heading = math.degrees(math.atan2(magYcomp, magXcomp))
    if heading < 0:
        heading += 360

    return heading, math.degrees(pitch), math.degrees(roll)

# ─────────────────────────────────────────────────────────────────────────────
# MQTT CALLBACKS
# ─────────────────────────────────────────────────────────────────────────────
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[SUCCESS] Connected to Cloud MQTT Broker!")
        # Subscribe to the GamePad commands
        client.subscribe(CLOUD_TOPIC_CMD)
        print(f"[*] Listening for GamePad commands on {CLOUD_TOPIC_CMD}...")
    else:
        print(f"[ERROR] Failed to connect. Return code: {rc}")

def on_message(client, userdata, msg):
    # Decode the GamePad text (e.g., "forward", "stop")
    command = msg.payload.decode('utf-8')
    print(f"\n[GAMEPAD] Received: {command}")
    
    # Translate to your robot's specific front_back/side_side values
    if command == "forward":
        motor_cmd = "126 64"
    elif command == "back":
        motor_cmd = "0 64"
    elif command == "left":
        motor_cmd = "64 126"
    elif command == "right":
        motor_cmd = "64 0"
    elif command == "stop":
        motor_cmd = "64 64"
    else:
        return # Ignore unknown commands

    print(f"[*] Translated to Motor Command: {motor_cmd}")
    
    # -----------------------------------------------------------------
    # NOTE: Send `motor_cmd` to your hardware here!
    # If your motors listen on a local MQTT broker like the old script, 
    # you would initialize a local client and run:
    # local_client.publish("robot/control", motor_cmd)
    # -----------------------------------------------------------------



# ─────────────────────────────────────────────────────────────────────────────
# MAIN EXECUTION
# ─────────────────────────────────────────────────────────────────────────────
def main():
    print("[*] Starting Hardware-to-Cloud MQTT Script...")
    
    # Init IMU
    IMU.detectIMU()
    if IMU.BerryIMUversion == 99:
        print("[IMU] No BerryIMU found... exiting")
        sys.exit()
    IMU.initIMU()
    print("[IMU] Initialized successfully.")

    # Start Hardware Threads
    threading.Thread(target=gps_serial_thread, daemon=True).start()
    threading.Thread(target=camera_capture_thread, daemon=True).start()

    # Init MQTT
    client = mqtt.Client(client_id="rpi_hardware_telemetry")
    if CLOUD_MQTT_USERNAME and CLOUD_MQTT_PASSWORD:
        client.username_pw_set(CLOUD_MQTT_USERNAME, CLOUD_MQTT_PASSWORD)

    client.on_connect = on_connect
    client.on_message = on_message
    
    client.will_set(CLOUD_TOPIC_STATUS, "Status: INACTIVE | Connection Lost", retain=True)

    try:
        client.connect(CLOUD_MQTT_SERVER, CLOUD_MQTT_PORT, 60)
        client.loop_start()
        print(f"[*] Connected to Cloud MQTT at {CLOUD_MQTT_SERVER}")
    except Exception as e:
        print(f"[ERROR] Could not connect: {e}")
        return

    time.sleep(2)
    print("[*] Beginning telemetry stream...")

    try:
        while True:
            # 1. Real GPS Data
            if current_lat is not None and current_lon is not None:
                gps_payload = json.dumps({
                    "lat": str(current_lat),
                    "lon": str(current_lon),
                    "alt": "0.0"
                })
                client.publish(CLOUD_TOPIC_GPS, gps_payload)

            # 2. Real IMU Data
            h, p, r = calculate_heading()
            try:
                gx, gy, gz = str(IMU.readGYRx()), str(IMU.readGYRy()), str(IMU.readGYRz())
            except AttributeError:
                gx = gy = gz = "0.00"

            imu_payload = json.dumps({
                "acc": {"x": str(IMU.readACCx()), "y": str(IMU.readACCy()), "z": str(IMU.readACCz())},
                "gyro": {"x": gx, "y": gy, "z": gz},
                "roll": str(round(r, 2)),
                "pitch": str(round(p, 2)),
                "yaw": str(round(h, 2))
            })
            client.publish(CLOUD_TOPIC_IMU, imu_payload)

            # 3. Real Camera Live Feed (Base64 Encoded)
            with camera_frame_lock:
                frame_to_send = latest_camera_frame.copy() if latest_camera_frame is not None else None

            if frame_to_send is not None:
                # Resize to make the MQTT payload smaller and faster
                small_frame = cv2.resize(frame_to_send, (320, 240))
                # Encode as JPEG with 50% quality
                _, buffer = cv2.imencode('.jpg', small_frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
                # Convert the bytes to a Base64 string
                jpg_as_text = base64.b64encode(buffer).decode('utf-8')
                
                # Format exactly how HTML expects an image source
                cam_payload = f"data:image/jpeg;base64,{jpg_as_text}"
            else:
                cam_payload = "Camera Offline: Awaiting frame..."
                
            client.publish(CLOUD_TOPIC_CAMERA, cam_payload)

            # 4. Status
            client.publish(CLOUD_TOPIC_STATUS, "Status: ACTIVE | Telemetry Streaming")

            time.sleep(0.05) # Send at ~20 updates per second

    except KeyboardInterrupt:
        print("\n[*] Stopping Script...")
    finally:
        client.publish(CLOUD_TOPIC_STATUS, "Status: INACTIVE | Script Stopped", retain=True)
        time.sleep(0.5) # Give it half a second to actually send the message before cutting the connection

        client.loop_stop()
        client.disconnect()
        print("[*] Disconnected.")

if __name__ == "__main__":
    main()