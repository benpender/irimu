import serial
import serial.tools.list_ports
import time
import struct
import math
import random
import threading

# ----------------------------------------------------------------------------
# CONFIGURATION
# ----------------------------------------------------------------------------
BAUD_RATE = 115200
MATRIX_W = 8
MATRIX_H = 8
FPS = 60

# ----------------------------------------------------------------------------
# GLOBAL STATE
# ----------------------------------------------------------------------------
imu_data = {'ax': 0, 'ay': 0, 'az': 0}
running = True

# Rain Drops: List of [x, y, vx, vy]
drops = []

# ----------------------------------------------------------------------------
# SERIAL MANAGEMENT
# ----------------------------------------------------------------------------
def find_esp32_ports():
    ports = list(serial.tools.list_ports.comports())
    esp_ports = []
    print("Scanning ports...")
    for p in ports:
        # Debug info
        # print(f"Found {p.device}: VID={p.vid}, PID={p.pid}, Desc={p.description}")
        
        # Match CH340/CH343 (0x1A86) or Native S3 (0x303A)
        if p.vid in [0x1A86, 0x303A, 0x10C4]: # Added SiLabs/CP210x as more safety
            esp_ports.append(p.device)
    return esp_ports

def setup_bridge(port, mode):
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
        time.sleep(2) # Wait for boot
        cmd = f"MODE:{mode}\n".encode()
        ser.write(cmd)
        print(f"Configured {port} as {mode}")
        return ser
    except Exception as e:
        print(f"Failed to open {port}: {e}")
        return None

# ----------------------------------------------------------------------------
# THREAD: READER (RX BRIDGE)
# ----------------------------------------------------------------------------
def rx_thread_func(ser):
    global imu_data
    print("RX Thread Started (Binary Mode)")
    buffer = bytearray()
    
    while running:
        if ser.in_waiting:
            try:
                # Read all available bytes directly
                chunk = ser.read(ser.in_waiting)
                if chunk:
                    buffer.extend(chunk)
                
                # Process Buffer
                while len(buffer) > 0:
                    # 1. Look for Binary Header 0xAA 0xBB
                    if len(buffer) >= 32 and buffer[0] == 0xAA and buffer[1] == 0xBB:
                        length = buffer[2]
                        if len(buffer) >= 3 + length + 2: # Header + Len + Pay + Foot
                            payload = buffer[3:3+length]
                            # Unpack IMU
                            if length == 28:
                                try:
                                    data = struct.unpack('ffffffI', payload)
                                    imu_data['ax'] = data[0]
                                    imu_data['ay'] = data[1]
                                    imu_data['az'] = data[2]
                                    # print(f"IMU: {imu_data['ax']:.2f}")
                                except:
                                    pass
                            # Remove processed packet
                            del buffer[:3+length+2]
                            continue
                        else:
                            # Incomplete packet, wait for more data
                            break

                    # 2. If not header, check if it's a text line (Debug message)
                    # We only look at the start range if it doesn't match 0xAA 0xBB
                    # Attempt to find a newline
                    try:
                        newline_idx = buffer.find(b'\n')
                        if newline_idx != -1 and newline_idx < 32: # Limit text line search
                            line = buffer[:newline_idx+1].decode('utf-8', errors='ignore').strip()
                            if line and "RAW RX" not in line: 
                                print(f"BRIDGE MSG: {line}")
                            del buffer[:newline_idx+1]
                            continue
                    except:
                        pass
                    
                    # 3. If neither, slide window by 1 byte to find next header
                    # (But be careful not to discard valid partial packet waiting)
                    if len(buffer) > 0 and (buffer[0] != 0xAA or (len(buffer) > 1 and buffer[1] != 0xBB)):
                         del buffer[0]
                    else:
                         # We have 0xAA or 0xAA+0xBB but packet incomplete
                         break

            except Exception as e:
                print(f"RX Error: {e}")
                time.sleep(0.1)
        else:
             time.sleep(0.001)

# ----------------------------------------------------------------------------
# GRAVITY ARROW SIMULATION
# ----------------------------------------------------------------------------
def update_physics():
    pass # No physics, just stateless rendering

def draw_line(frame, x0, y0, x1, y1, color):
    # Bresenham's Line Algorithm (Simplified)
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    
    while True:
        idx = (int(y0) * 8 + int(x0)) * 3
        if 0 <= idx < 192:
            frame[idx] = color[0]
            frame[idx+1] = color[1]
            frame[idx+2] = color[2]
            
        if abs(x0 - x1) < 0.5 and abs(y0 - y1) < 0.5: break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

# RAINBOW STATE
import colorsys
rainbow_offset = 0.0

def render_frame():
    global rainbow_offset
    
    # 8x8 Grid
    frame = bytearray(64 * 3)
    
    # Scroll speed (Hue cycle per frame)
    rainbow_offset += 0.02
    if rainbow_offset > 1.0:
        rainbow_offset -= 1.0
        
    # Get Gravity (Normalized approximate)
    ax = imu_data['ax']
    ay = imu_data['ay']
    
    # Avoid full zero
    if abs(ax) < 0.01 and abs(ay) < 0.01:
        ay = 1.0 # Default down
        
    for y in range(8):
        for x in range(8):
            # Calculate position projected onto gravity vector
            # This makes the gradient align with "Down"
            # We center coordinates around 3.5, 3.5
            px = x - 3.5
            py = y - 3.5
            
            # Projection: (px * ax + py * ay) / scale
            # This gives us a "height" along the gravity axis
            height = (px * ax + py * ay) * 0.15 # Scale for gradient density
            
            # Calculate hue
            hue = height + rainbow_offset
            # Wrap hue 0-1
            hue = hue - int(hue)
            if hue < 0: hue += 1.0
            
            # Convert HSV to RGB
            r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
            
            idx = (y * 8 + x) * 3
            frame[idx] = int(r * 255)
            frame[idx+1] = int(g * 255)
            frame[idx+2] = int(b * 255)

    return frame

# ----------------------------------------------------------------------------
# MAIN LOOP
# ----------------------------------------------------------------------------
if __name__ == "__main__":
    ports = find_esp32_ports()
    if len(ports) < 2:
        print("Error: Need 2 ESP32 Dongles connected (Found {len(ports)})")
        # exit() 
        # For testing, we might proceed if user only has 1 and we simulate?
        # User said "I now have all three boards connected", so we expect 2 dongles.
    
    print(f"Found ports: {ports}")
    
    print(f"Assigning roles to ports: {ports}")
    
    tx_port_name = None
    rx_port_name = None
    
    for p in ports:
        try:
            ser = serial.Serial(p, BAUD_RATE, timeout=0.5)
            time.sleep(1) # Wait for some data to trickle in
            sample = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            ser.close()
            
            if "BRIDGE_READY" in sample or "ESP_NOW_OK" in sample:
                print(f"Found BRIDGE on {p}")
                if not rx_port_name: rx_port_name = p
                elif not tx_port_name: tx_port_name = p
            elif "AX:" in sample:
                print(f"Found DANCER on {p}")
                # A Dancer can't be an RX bridge for ESP-NOW, but we can read it directly
                if not rx_port_name: rx_port_name = p 
            else:
                # Default fallback
                if not tx_port_name: tx_port_name = p
        except:
            continue

    if not tx_port_name: tx_port_name = ports[0]
    if not rx_port_name: rx_port_name = ports[1] if len(ports) > 1 else ports[0]
    
    print(f"Final Roles -> TX: {tx_port_name}, RX: {rx_port_name}")
    
    tx_ser = setup_bridge(tx_port_name, "TX")
    rx_ser = None
    
    if rx_port_name:
        rx_ser = setup_bridge(rx_port_name, "RX")
        t = threading.Thread(target=rx_thread_func, args=(rx_ser,))
        t.daemon = True
        t.start()
    
    print("Starting Gravity Arrow Demo...")
    frame_count = 0
    try:
        while True:
            start_time = time.time()
            
            # 1. Update
            update_physics()
            
            # 2. Render
            pixels = render_frame()
            
            # 3. Send (TX)
            # Protocol: Magic Byte 0xFA + 192 bytes
            header = bytes([0xFA])
            tx_ser.write(header)
            tx_ser.write(pixels)
            
            # Debug Log every 60 frames (~1 sec)
            frame_count += 1
            if frame_count % 60 == 0:
                ax = imu_data['ax']
                ay = imu_data['ay']
                print(f"[{frame_count}] Gravity: ax={ax:.2f}, ay={ay:.2f} (RX Active: {rx_ser is not None})")
            
            # FPS Lock
            elapsed = time.time() - start_time
            sleep_time = (1.0/FPS) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        print("Stopping...")
        running = False
        if tx_ser: tx_ser.close()
        if rx_ser: rx_ser.close()
