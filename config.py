"""Configuration constants for the BOZGUN control software.

All tunable parameters live here so the vision and serial behaviour can be
adjusted without touching the main application logic.
"""

# --- Serial / hardware -------------------------------------------------------
ARDUINO_PORT = "COM5"        # Serial port the Arduino is connected to
BAUDRATE = 9600
CAMERA_INDEX = 0             # OpenCV camera index (0 = default webcam)

# --- Frame -------------------------------------------------------------------
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# --- Servo limits (degrees) --------------------------------------------------
# The turret is mechanically constrained to a safe sub-range instead of 0-180.
PAN_MIN_ANGLE = 45
PAN_MAX_ANGLE = 135
TILT_MIN_ANGLE = 60
TILT_MAX_ANGLE = 120

# --- Targeting / lock-on -----------------------------------------------------
LOCK_ON_DURATION = 1.5       # seconds a target must stay locked to "fire"
HIT_COOLDOWN = 5.0           # seconds a hit target is ignored afterwards
MIN_TARGET_RADIUS = 30       # px, ignore contours smaller than this

# --- Motion smoothing --------------------------------------------------------
# Frame-rate-independent exponential smoothing time constant, in seconds.
# alpha = 1 - exp(-dt / tau). Smaller = snappier, larger = smoother/slower.
MOTOR_RESPONSE_TC = 0.12

# --- Centroid tracker --------------------------------------------------------
TRACK_MAX_MISSES = 0.6       # seconds before an unseen track is dropped
TRACK_MATCH_DIST = 60.0      # px, max distance to match a detection to a track

# --- Deadband ----------------------------------------------------------------
# If the target is within this many px of the frame centre, hold the current
# aim instead of issuing new angles. Removes servo jitter when the target is
# essentially centred.
DEADBAND_PX = 25

# --- Laser safety ------------------------------------------------------------
# Hard cap on continuous laser-on time (seconds). Software safety guard so the
# laser cannot stay on indefinitely if the loop stalls. Normal lock-on is
# LOCK_ON_DURATION (well below this), so it never trips in normal operation.
LASER_MAX_ON_TIME = 4.0

# --- HSV calibration ---------------------------------------------------------
DEFAULT_HSV = {
    "low_h": 94, "low_s": 80, "low_v": 2,
    "high_h": 126, "high_s": 255, "high_v": 255,
}
CALIB_FILE = "hsv_calib.json"

# --- UI / assets -------------------------------------------------------------
LOOP_FPS_SMOOTH = 0.9        # HUD FPS smoothing factor (0..1)
WINDOW_NAME = "BOZGUN - Control"
CALIB_WINDOW = "HSV Calibration"
SHOT_SOUND_FILE = "laser_shot.mp3"   # optional; the app runs fine if missing
