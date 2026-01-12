from tracker.beacon_detect import detect_beacon, CODE

print("Running Detector Test...")

FRAMES_PER_BIT = 3

def feed_signal(signal):
    result = False
    for val in signal:
        result = detect_beacon(val, frames_per_bit=FRAMES_PER_BIT)
    return result

# 1. Test Silence (0)
print("\nTest 1: Silence")
silent = [0] * (len(CODE) * FRAMES_PER_BIT * 3)
print(f"Result: {feed_signal(silent)} (Expected False)")

# 2. Test Beacon Code Pattern (40Hz envelope at 120FPS => 3 frames per bit)
print("\nTest 2: Beacon Code Pattern")
signal = []
for _ in range(3):
    for bit in CODE:
        signal.extend([250 if bit else 0] * FRAMES_PER_BIT)
print(f"Result: {feed_signal(signal)} (Expected True)")
