# IRIMU Tracking System

High-speed IR beacon tracking using an ESP32-S3, a stereo USB camera, and a browser visualization.

## Structure
- `/firmware`: PlatformIO project for ESP32-S3.
- `/tracker`: Python CV tracker + beacon detection + WebSocket server.
- `/web`: HTML frontend for visualization.

## Quick Start

### 1. Hardware (ESP32-S3)
- IR LED on GPIO 4 and GND.
- Firmware `ir_beacon` (8kHz carrier + 40Hz envelope) should already be flashed.
- To re-flash:
  ```bash
  cd firmware
  pio run -e ir_beacon -t upload
  ```

### 2. Run the Tracker (Back-end)
Start the tracker from the project root:
```bash
python3 tracker/ir_tracker.py
```
Expected output:
- `Camera initialized: 2560x720 @ 120 FPS`
- `WebSocket Server started on ws://localhost:8765`

### 3. Open the Visualization (Front-end)
```bash
open web/tracking_view.html
```
What you'll see:
- Red Cross: raw brightest point (sunlight/noise).
- Green Dot: appears only when the IR beacon is detected (pulsing signal).
- Graph: real-time brightness waveform.

## Key Files
- `tracker/ir_tracker.py`: Main CV loop, camera setup, max-brightness tracking, JSON via WebSocket.
- `tracker/beacon_detect.py`: Beacon detection (variance + edge transition checks across ~30 frames).
- `web/tracking_view.html`: Frontend render (video plane, red cross, green dot, graph).

## Troubleshooting
- WebSocket already in use:
  ```bash
  lsof -ti:8765 | xargs kill -9
  ```
- "SEARCHING..." but LED is close: camera may be clipping at 255. Move LED back or dim room.
- Green dot jumping: tracker holds last known position during OFF pulses; jumps mean lock lost.
