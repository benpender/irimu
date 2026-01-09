# Project Status Snapshot: Multi-Matrix Debugging
**Date:** 2026-01-08

## Current State
- **Web App (`matrix_mapper.html`):**
  - Updated to `921600` baud.
  - Implemented `0xAA 0xBB` Sync Header for TX packets.
  - Unthrottled (60+ FPS enabled).
- **Bridge Firmware (`bridge_dongle`):**
  - Updated RX Buffer to `4096` bytes.
  - Updated to `921600` baud.
  - Implemented Robust Parser (Require `0xAA 0xBB` header).
- **Dancer Firmware (`dancer_matrix`):**
  - Currently flashed with **DEBUG FIRMWARE** (Visual Colors) on at least one unit.
  - Logic: Red (Idle), Green (Success), Blue (Wrong MAC), Yellow (Wrong Size).

## The Issue
- User reports "One Single Green LED" stuck on matrices.
- This persists despite buffer/sync fixes.
- Debug firmware flashed to help diagnose (Red/Green/Blue codes), but testing paused to avoid USB flashing fatigue.

## Next Steps (When Resuming)
1.  **Verify Debug Colors:** See if the flashed unit turns Red/Blue/Green.
2.  **OTA Implementation:** Prioritize implementing OTA updates (via ESP-NOW or WiFi) to avoid plugging in devices to flash.
3.  **Fix Addressing:** If LEDs turn Blue, fix the discovery/MAC mapping logic.
