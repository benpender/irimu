import cv2
import asyncio
import websockets
import json
import time

# =========================
# Configuration
# =========================

CAMERA_INDEX = 0
TARGET_FPS = 120
FRAME_WIDTH = 2560
FRAME_HEIGHT = 720

# Brightness threshold for "beacon locked"
THRESHOLD = 40   # Adjust if needed after filter is added


# =========================
# IR Tracker Class
# =========================

class IRTracker:
    def __init__(self):
        self.camera_index = CAMERA_INDEX
        self.cap = None
        self.running = True
        self.new_camera_requested = None
        self.init_camera()

    def init_camera(self):
        if self.cap:
            self.cap.release()

        print(f"Opening camera index: {self.camera_index}")
        self.cap = cv2.VideoCapture(self.camera_index)

        # Force high-speed capture mode
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        print(f"Camera initialized: {w}x{h} @ {actual_fps} FPS")

    async def handle_client(self, websocket, path):
        print("Client connected")

        if self.cap is None or not self.cap.isOpened():
            self.init_camera()

        last_frame_time = time.perf_counter()
        frame_counter = 0

        try:
            while self.running:

                # -------------------
                # Handle camera switch commands
                # -------------------
                try:
                    message = await asyncio.wait_for(websocket.recv(), timeout=0.001)
                    data = json.loads(message)
                    if data.get("command") == "set_camera":
                        self.new_camera_requested = int(data.get("index"))
                        print(f"Switching to camera {self.new_camera_requested}")
                except asyncio.TimeoutError:
                    pass
                except:
                    break

                if self.new_camera_requested is not None:
                    self.camera_index = self.new_camera_requested
                    self.new_camera_requested = None
                    self.init_camera()

                # -------------------
                # Grab frame
                # -------------------
                ret, frame = self.cap.read()
                if not ret:
                    print("Frame capture failed")
                    await asyncio.sleep(0.01)
                    continue

                # Stereo split — take LEFT half
                h, w, _ = frame.shape
                left_frame = frame[:, :w // 2]

                # -------------------
                # Detection
                # -------------------
                gray = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)

                # Find brightest pixel (IR beacon)
                (_, raw_max_val, _, raw_max_loc) = cv2.minMaxLoc(gray)

                beacon_x = int(raw_max_loc[0])
                beacon_y = int(raw_max_loc[1])
                beacon_val = float(raw_max_val)

                raw_x, raw_y = beacon_x, beacon_y

                # -------------------
                # FPS calculation
                # -------------------
                t_now = time.perf_counter()
                dt = t_now - last_frame_time
                last_frame_time = t_now
                current_fps = 1.0 / dt if dt > 0 else 0.0

                # -------------------
                # Lock decision
                # -------------------
                is_locked = beacon_val >= THRESHOLD

                frame_counter += 1
                if frame_counter % 60 == 0:
                    print(f"RAW_MAX: {beacon_val:.1f}  FPS: {current_fps:.1f}", flush=True)

                # -------------------
                # Send data
                # -------------------
                out = {
                    "found": is_locked,
                    "locked": is_locked,
                    "x": beacon_x,
                    "y": beacon_y,
                    "raw_x": raw_x,
                    "raw_y": raw_y,
                    "max_val": beacon_val,
                    "raw_max_val": raw_max_val,
                    "res_w": int(left_frame.shape[1]),
                    "res_h": int(left_frame.shape[0]),
                    "fps": current_fps,
                    "camera_index": self.camera_index
                }

                try:
                    await websocket.send(json.dumps(out))
                except websockets.exceptions.ConnectionClosed:
                    print("Client disconnected")
                    break

                await asyncio.sleep(0)

        finally:
            print("Client handler finished")

    def stop(self):
        self.running = False
        if self.cap:
            self.cap.release()


# =========================
# Main WebSocket Server
# =========================

async def main():
    tracker = IRTracker()

    async def handler(websocket, path):
        await tracker.handle_client(websocket, path)

    server = await websockets.serve(handler, "localhost", 8765)
    print("WebSocket Server started on ws://localhost:8765")
    await server.wait_closed()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Server stopped")
