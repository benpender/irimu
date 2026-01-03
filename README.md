# IRIMU Tracking System

This project implements a high-speed tracking system using an ESP32-S3 Matrix IMU and a Stereoscopic Webcam.

## Structure
- `/firmware`: PlatformIO project for ESP32-S3.
- `/tracker`: Python script for Computer Vision + Sensor Fusion.
- `/web`: HTML frontend for visualization.

## Setup Instructions

### 1. Firmware (ESP32-S3)
1. Navigate to `firmware/src/main.cpp`.
2. **IMPORTANT**: Update `YOUR_SSID`, `YOUR_PASSWORD`, and `udpAddress` (Your PC's IP) at the top of the file.
3. Connect your ESP32-S3 Matrix via USB.
4. Build and Upload:
   ```bash
   cd firmware
   pio run -t upload
   ```
   (Or use your PlatformIO IDE extension).

### 2. Tracker (Python)
1. Install dependencies:
   ```bash
   cd tracker
   pip install -r requirements.txt
   ```
2. Connect your USB Webcam.
3. Run the tracker:
   ```bash
   python main.py
   ```
   *Note: If the camera doesn't open, change `CAMERA_INDEX` in `main.py`.*

### 3. Visualization
1. Open `web/index.html` in your browser.
2. Ensure the Python tracker is running.
3. You should see the white dot move as you move the ESP32 Matrix!

## Tuning
- **IMU Sensitivity**: Adjust `control` matrix multiplication in `tracker/main.py` if the IMU movement is too strong/weak.
- **LED Detection**: Adjust the threshold `200` in `tracker/main.py` if the LED isn't detected.
