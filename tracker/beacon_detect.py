import numpy as np
from collections import deque

# Must match ESP32 code
CODE = np.array([1, 0, 1, 1, 0, 1, 0, 0], dtype=np.uint8)
CODE_LEN = len(CODE)

ENVELOPE_HZ = 40
DEFAULT_FPS = 120
DEFAULT_FRAMES_PER_BIT = max(1, int(round(DEFAULT_FPS / ENVELOPE_HZ)))

HISTORY_CYCLES = 3
MIN_RANGE = 40
MATCH_THRESHOLD = 0.7

def _get_state(frames_per_bit):
    if not hasattr(detect_beacon, "state") or detect_beacon.state["frames_per_bit"] != frames_per_bit:
        detect_beacon.state = {
            "frames_per_bit": frames_per_bit,
            "code_frames": np.repeat(CODE, frames_per_bit),
            "history": deque(maxlen=CODE_LEN * frames_per_bit * HISTORY_CYCLES),
        }
    return detect_beacon.state

def detect_beacon(brightness_value, frames_per_bit=None):
    """
    brightness_value = mean pixel intensity around the candidate beacon for this frame.
    returns True if the code pattern is detected.
    """
    if frames_per_bit is None:
        frames_per_bit = DEFAULT_FRAMES_PER_BIT

    state = _get_state(frames_per_bit)
    history = state["history"]

    history.append(float(brightness_value))
    if len(history) < history.maxlen:
        return False

    hist_array = np.array(history, dtype=np.float32)
    mn = float(np.nanmin(hist_array))
    mx = float(np.nanmax(hist_array))
    if mx - mn < MIN_RANGE:
        return False

    mid = (mx + mn) / 2.0
    raw_bits = (hist_array > mid).astype(np.uint8)

    code_frames = state["code_frames"]
    window = len(code_frames)
    if len(raw_bits) < window:
        return False

    best_match = 0.0
    for start in range(len(raw_bits) - window + 1):
        segment = raw_bits[start:start + window]
        match = float(np.mean(segment == code_frames))
        if match > best_match:
            best_match = match
            if best_match >= MATCH_THRESHOLD:
                break

    return best_match >= MATCH_THRESHOLD
