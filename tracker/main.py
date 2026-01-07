import cv2
import numpy as np
import socket
import struct
import websockets
import asyncio
import json
import time
import threading
from websockets.exceptions import ConnectionClosed
from threading import Thread

import base64

# ----------------------------------------------------------------------------
# CONFIGURATION
# ----------------------------------------------------------------------------
UDP_IP = "0.0.0.0"
UDP_PORT = 44444
CAMERA_INDEX = 0
WEBSOCKET_PORT = 8765

# Kalman Filter Setup
def create_kalman_filter():
    # 4 state variables: x, y, vx, vy
    # 2 measurement variables: x, y
    # 2 control variables: ax, ay
    kf = cv2.KalmanFilter(4, 2, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                     [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0],
                                    [0, 1, 0, 1],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]], np.float32)
    
    # Process Noise Covariance (Q) - Model uncertainty
    # High -> trust measurement more. Low -> trust model more.
    kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
    
    # Measurement Noise Covariance (R) - Sensor noise
    kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
    
    return kf

class Tracker:
    def __init__(self):
        self.latest_pos = {
            'x': 0, 'y': 0, 
            'detected': False, 
            'udp_count': 0, 
            'image': '',
            'frame_w': 640, # Default, updated dynamically
            'frame_h': 480
        }
        self.running = True
        self.use_imu = True
        self.camera_mode = 'left'  # 'left', 'right', 'full'
        self.mirror = False # Mirror toggle
        self.kf = create_kalman_filter()
        self.last_time = time.time()
        self.udp_packet_count = 0
        self.gravity = None # For High-Pass Filter
        
    def start_udp_listener(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
        print(f"UDP Listening on {UDP_PORT}...")
        
        while self.running:
            try:
                data, addr = sock.recvfrom(1024)
                if len(data) == 28:
                    self.udp_packet_count += 1
                    self.latest_pos['udp_count'] = self.udp_packet_count
                    
                    # Structure: ax, ay, az, gx, gy, gz, timestamp
                    ax, ay, az, gx, gy, gz, ts = struct.unpack('ffffffI', data)
                    
                    # Debug print occasionally
                    if self.udp_packet_count % 100 == 0:
                        print(f"IMU Data: Ax={ax:.2f} Ay={ay:.2f}")

                    # Kalman Prediction Step using IMU
                    dt = 0.004 
                    self.kf.controlMatrix = np.array([[0.5*dt**2, 0],
                                                      [0, 0.5*dt**2],
                                                      [dt, 0],
                                                      [0, dt]], np.float32)
                    
                    # High-Pass Filter to remove Gravity
                    # alpha = 0.9 means we trust gravity changes slowly.
                    # user_accel = raw - gravity
                    # This removes the "tilt = acceleration" drift.
                    alpha = 0.98 
                    raw_accel = np.array([ax, ay, az])
                    
                    if self.gravity is None:
                        self.gravity = raw_accel
                    else:
                        self.gravity = alpha * self.gravity + (1 - alpha) * raw_accel
                        
                    user_accel = raw_accel - self.gravity
                    
                    # Use filtered accel for control
                    # Use filtered accel for control
                    if self.use_imu:
                        # Scale factor - converting 'g' units to 'pixel acceleration'
                        control = np.array([[user_accel[0] * 2000], [user_accel[1] * 2000]], np.float32)
                        
                        # Only predict/update high-speed if we are actually using IMU
                        prediction = self.kf.predict(control)
                        self.latest_pos['x'] = float(prediction[0])
                        self.latest_pos['y'] = float(prediction[1])
                    else:
                        # If IMU is off, do NOT predict or update position here.
                        # Rely solely on the Camera loop (30Hz) for stable, non-bouncy position.
                        pass
            except Exception as e:
                print(f"UDP Error: {e}")

    def start_camera(self):
        cap = cv2.VideoCapture(CAMERA_INDEX)
        # Check if camera opened successfully
        if not cap.isOpened():
            print(f"Error: Could not open camera with index {CAMERA_INDEX}")
            return

        print("Camera started.")
        frame_count = 0
        
        while self.running and cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                time.sleep(0.1)
                continue
            
            frame_count += 1
            # Flip camera (180 degrees) as requested
            frame = cv2.flip(frame, -1) 

            h, w, _ = frame.shape
            
            # Crop based on mode
            if self.camera_mode == 'left':
                process_frame = frame[:, :w//2]
            elif self.camera_mode == 'right':
                process_frame = frame[:, w//2:]
            else: # full
                process_frame = frame

            # Mirror if requested (Horizontal Flip) - APPLY BEFORE DETECTION
            if self.mirror:
                process_frame = cv2.flip(process_frame, 1)

            gray = cv2.cvtColor(process_frame, cv2.COLOR_BGR2GRAY)
            # Threshold - assume bright LED
            _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
            
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            detected = False
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 5: # Noise filter
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        detected = True
                        measurement = np.array([[np.float32(cx)], [np.float32(cy)]])
                        self.kf.correct(measurement)
                        
                        # Update position immediately from corrected state
                        # This ensures we have a valid position even if UDP/IMU is silent
                        self.latest_pos['x'] = float(self.kf.statePost[0][0])
                        self.latest_pos['y'] = float(self.kf.statePost[1][0])
                        
                        # Draw debug info
                        cv2.drawContours(process_frame, [c], -1, (0, 255, 0), 2)
                        cv2.circle(process_frame, (cx, cy), 5, (0, 0, 255), -1)

                        if frame_count % 60 == 0:
                           print(f"Camera Detected LED at: {cx}, {cy}")
            
            self.latest_pos['detected'] = detected
            
            # Update frame dims for frontend scaling
            ph, pw = process_frame.shape[:2]
            self.latest_pos['frame_w'] = pw
            self.latest_pos['frame_h'] = ph
            
            # (Mirror logic moved up)

            # Encode frame for network
            # Resize while maintaining aspect ratio
            ph, pw = process_frame.shape[:2]
            target_width = 320
            scale = target_width / pw
            target_height = int(ph * scale)
            
            small_frame = cv2.resize(process_frame, (target_width, target_height))
            _, buffer = cv2.imencode('.jpg', small_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            img_b64 = base64.b64encode(buffer).decode('utf-8')
            self.latest_pos['image'] = "data:image/jpeg;base64," + img_b64
        
        cap.release()
        cv2.destroyAllWindows()

    async def ws_handler(self, websocket):
        async def sender():
            while self.running:
                try:
                    await websocket.send(json.dumps(self.latest_pos))
                    await asyncio.sleep(0.033) # ~30Hz
                except websockets.exceptions.ConnectionClosed:
                    break

        async def receiver():
            try:
                async for message in websocket:
                    try:
                        data = json.loads(message)
                        if data.get('type') == 'config':
                            if 'use_imu' in data:
                                self.use_imu = data['use_imu']
                                print(f"IMU usage toggled: {self.use_imu}")
                            if 'camera_mode' in data:
                                self.camera_mode = data['camera_mode']
                                print(f"Camera mode set to: {self.camera_mode}")
                            if 'mirror' in data:
                                self.mirror = data['mirror']
                                print(f"Mirror set to: {self.mirror}")
                    except json.JSONDecodeError:
                        pass
            except websockets.exceptions.ConnectionClosed:
                pass

        await asyncio.gather(sender(), receiver())

    async def main(self):
        # Start Threads
        t_udp = threading.Thread(target=self.start_udp_listener)
        t_udp.daemon = True
        t_udp.start()
        
        t_cam = threading.Thread(target=self.start_camera)
        t_cam.daemon = True
        t_cam.start()
        
        # Start WebSocket Server (Main Async Loop)
        async with websockets.serve(self.ws_handler, "localhost", WEBSOCKET_PORT):
            print(f"WebSocket Server running on ws://localhost:{WEBSOCKET_PORT}")
            await asyncio.Future() # Run forever

if __name__ == "__main__":
    tracker = Tracker()
    try:
        asyncio.run(tracker.main())
    except KeyboardInterrupt:
        tracker.running = False
