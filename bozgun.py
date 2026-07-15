"""BOZGUN - real-time colour-based target tracking and laser marking (control side).

This is the PC-side control and computer-vision software of the BOZGUN turret.
Each frame it:

    1. grabs a webcam frame,
    2. isolates a target colour in HSV space (live-calibratable),
    3. tracks detected blobs with a small memory-based centroid tracker,
    4. maps the chosen target's pixel position to pan/tilt servo angles,
    5. smooths the motion (frame-rate-independent) and streams the command
       to an Arduino over serial as "pan,tilt,laser\\n",
    6. after a target stays locked for LOCK_ON_DURATION seconds, triggers a
       *simulated* hit (sound + white flash + cooldown). No projectile.

If no serial device is available (or --sim is passed) the program keeps running
and prints the commands it would have sent, so it can be demoed without the
turret hardware. With --demo it also renders a synthetic moving target, so the
whole pipeline runs with no webcam and no Arduino at all.

Extras: a near-centre deadband removes servo jitter, and a laser safety timeout
caps continuous laser-on time.

Controls:  S = save current HSV calibration   Q = quit

Education / simulation project. See README for scope and laser-safety notes.
"""

import argparse
import json
import math
import os
import time

import cv2
import numpy as np

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover - serial is optional for sim runs
    serial = None

try:
    import pygame
except ImportError:  # pragma: no cover - sound is optional
    pygame = None

import config


# ==============================================================================
# Calibration helpers (trackbar window + JSON persistence)
# ==============================================================================
def load_calibration(path=config.CALIB_FILE):
    """Load saved HSV thresholds, falling back to the defaults."""
    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                data = json.load(f)
            print(f"[Calibration] Loaded {path}.")
            return data
        except Exception as e:
            print(f"[Calibration] Could not load {path}: {e}")
    return config.DEFAULT_HSV.copy()


def save_calibration(cfg, path=config.CALIB_FILE):
    """Persist the current HSV thresholds to disk."""
    try:
        with open(path, "w") as f:
            json.dump(cfg, f, indent=2)
        print(f"[Calibration] Saved -> {path}")
    except Exception as e:
        print(f"[Calibration] Save error: {e}")


def create_calibration_window(initial_cfg):
    cv2.namedWindow(config.CALIB_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(config.CALIB_WINDOW, 400, 320)
    cv2.createTrackbar("LowH", config.CALIB_WINDOW, initial_cfg["low_h"], 179, lambda v: None)
    cv2.createTrackbar("HighH", config.CALIB_WINDOW, initial_cfg["high_h"], 179, lambda v: None)
    cv2.createTrackbar("LowS", config.CALIB_WINDOW, initial_cfg["low_s"], 255, lambda v: None)
    cv2.createTrackbar("HighS", config.CALIB_WINDOW, initial_cfg["high_s"], 255, lambda v: None)
    cv2.createTrackbar("LowV", config.CALIB_WINDOW, initial_cfg["low_v"], 255, lambda v: None)
    cv2.createTrackbar("HighV", config.CALIB_WINDOW, initial_cfg["high_v"], 255, lambda v: None)


def read_calibration_from_window():
    return {
        "low_h": cv2.getTrackbarPos("LowH", config.CALIB_WINDOW),
        "high_h": cv2.getTrackbarPos("HighH", config.CALIB_WINDOW),
        "low_s": cv2.getTrackbarPos("LowS", config.CALIB_WINDOW),
        "high_s": cv2.getTrackbarPos("HighS", config.CALIB_WINDOW),
        "low_v": cv2.getTrackbarPos("LowV", config.CALIB_WINDOW),
        "high_v": cv2.getTrackbarPos("HighV", config.CALIB_WINDOW),
    }


# ==============================================================================
# Memory-based centroid tracker
# ==============================================================================
class Track:
    """A single tracked target with a stable id."""

    def __init__(self, track_id, centroid, timestamp):
        self.id = track_id
        self.centroid = centroid  # (x, y)
        self.last_seen = timestamp
        self.first_seen = timestamp
        self.missed_time = 0.0

    def update(self, centroid, timestamp):
        self.centroid = centroid
        self.last_seen = timestamp
        self.missed_time = 0.0


class SimpleTracker:
    """Nearest-neighbour centroid tracker with time-based track expiry."""

    def __init__(self, max_miss_seconds=config.TRACK_MAX_MISSES,
                 match_dist=config.TRACK_MATCH_DIST):
        self.tracks = {}          # id -> Track
        self._next_id = 1
        self.max_miss = max_miss_seconds
        self.match_dist = match_dist

    def step(self, detections, timestamp):
        """Advance the tracker one frame.

        detections: list of (x, y) centroids
        returns:     list of (track_id, centroid) for currently live tracks
        """
        used_det = set()

        # 1) Match existing tracks to the nearest detection within match_dist.
        for tid, tr in list(self.tracks.items()):
            best_det = None
            best_dist = None
            for i, d in enumerate(detections):
                if i in used_det:
                    continue
                dist = math.hypot(tr.centroid[0] - d[0], tr.centroid[1] - d[1])
                if best_det is None or dist < best_dist:
                    best_det = i
                    best_dist = dist
            if best_det is not None and best_dist <= self.match_dist:
                tr.update(detections[best_det], timestamp)
                used_det.add(best_det)

        # 2) Spawn new tracks for unmatched detections.
        for i, d in enumerate(detections):
            if i in used_det:
                continue
            tid = self._next_id
            self._next_id += 1
            self.tracks[tid] = Track(tid, d, timestamp)

        # 3) Drop tracks that have not been seen for too long.
        for tid, tr in list(self.tracks.items()):
            if timestamp - tr.last_seen > self.max_miss:
                del self.tracks[tid]

        return [(tid, tuple(tr.centroid)) for tid, tr in self.tracks.items()]

    def get_track_by_id(self, tid):
        return self.tracks.get(tid, None)


# ==============================================================================
# Synthetic camera (for --demo: run with no webcam and no Arduino)
# ==============================================================================
class SyntheticCamera:
    """Drop-in stand-in for cv2.VideoCapture that renders a moving blue target.

    Blue matches the default HSV range, so the full detect -> track -> aim ->
    lock -> fire pipeline runs against a generated frame with zero hardware.
    """

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self._t0 = time.time()
        self._opened = True

    def isOpened(self):
        return self._opened

    def set(self, *args, **kwargs):
        return True

    def read(self):
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        t = time.time() - self._t0
        # path that repeatedly sweeps across (and through) the centre
        cx = int(self.width / 2 + (self.width * 0.32) * math.sin(t * 0.9))
        cy = int(self.height / 2 + (self.height * 0.30) * math.sin(t * 1.4))
        cv2.circle(frame, (cx, cy), 42, (255, 0, 0), -1)  # blue target (BGR)
        time.sleep(0.01)  # keep the frame-rate reasonable
        return True, frame

    def release(self):
        self._opened = False


# ==============================================================================
# HUD
# ==============================================================================
def draw_hud(frame, status_text, target_info_list=None, fps=None):
    h, w, _ = frame.shape
    color = (0, 255, 0)
    thickness = 2
    corner = 30

    # corner reticle
    cv2.line(frame, (0, 0), (corner, 0), color, thickness)
    cv2.line(frame, (0, 0), (0, corner), color, thickness)
    cv2.line(frame, (w, 0), (w - corner, 0), color, thickness)
    cv2.line(frame, (w, 0), (w, corner), color, thickness)
    cv2.line(frame, (0, h), (corner, h), color, thickness)
    cv2.line(frame, (0, h), (0, h - corner), color, thickness)
    cv2.line(frame, (w, h), (w - corner, h), color, thickness)
    cv2.line(frame, (w, h), (w, h - corner), color, thickness)

    cv2.putText(frame, f"STATUS: {status_text}", (16, h - 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    if fps is not None:
        cv2.putText(frame, f"FPS: {fps:.1f}", (w - 140, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    if target_info_list:
        x0 = w - 300
        y0 = h - 90
        for i, (tid, (x, y, r)) in enumerate(target_info_list):
            cv2.putText(frame, f"ID:{tid} [{int(x)},{int(y)}] R:{int(r)}",
                        (x0, y0 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)


# ==============================================================================
# Main loop
# ==============================================================================
def main(force_sim=False, demo_mode=False):
    if demo_mode:
        force_sim = True  # no serial in demo

    # 1) Calibration (demo uses the default blue range so it detects the
    #    synthetic target regardless of any saved calibration)
    cfg = config.DEFAULT_HSV.copy() if demo_mode else load_calibration()
    create_calibration_window(cfg)

    # 2) Sound (optional)
    shot_sound = None
    if pygame is not None:
        try:
            pygame.mixer.init()
            shot_sound = pygame.mixer.Sound(config.SHOT_SOUND_FILE)
        except Exception:
            print(f"!!! Warning: '{config.SHOT_SOUND_FILE}' not found or audio "
                  f"unavailable. Continuing without sound.")
    else:
        print("!!! Warning: pygame not installed. Continuing without sound.")

    # 3) Arduino (fall back to simulation on any failure or when --sim is set)
    arduino = None
    arduino_connected = False
    if force_sim:
        print("[Arduino] --sim set: running without serial (simulation mode).")
    elif serial is None:
        print("[Arduino] pyserial not installed. Running in simulation mode.")
    else:
        try:
            arduino = serial.Serial(port=config.ARDUINO_PORT,
                                    baudrate=config.BAUDRATE, timeout=0.1)
            time.sleep(2)  # let the board reset
            arduino_connected = True
            print("[Arduino] Connected.")
        except Exception as e:
            print(f"[Arduino] Not connected ({e}). Running in simulation mode.")

    # 4) Camera (real webcam, or a synthetic source in demo mode)
    if demo_mode:
        print("[Demo] Synthetic target mode: no webcam or Arduino needed.")
        cap = SyntheticCamera(config.FRAME_WIDTH, config.FRAME_HEIGHT)
    else:
        cap = cv2.VideoCapture(config.CAMERA_INDEX)
    if not cap.isOpened():
        print("Could not open camera. Exiting.")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)

    cv2.namedWindow(config.WINDOW_NAME, cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty(config.WINDOW_NAME, cv2.WND_PROP_FULLSCREEN,
                          cv2.WINDOW_FULLSCREEN)

    tracker = SimpleTracker()
    hit_targets = {}                 # track_id -> time of hit
    locked_target_id = None
    lock_start_time = 0.0

    current_pan_angle = 90.0
    current_tilt_angle = 90.0
    laser_on_start = None            # for the laser safety timeout

    prev_time = time.time()
    smoothed_fps = None
    status_text = "Started - calibrate the colour or wait..."

    print("System started. Adjust the colour range in the calibration window. "
          "(S: save, Q: quit)")

    while True:
        loop_start = time.time()
        dt = loop_start - prev_time if prev_time is not None else 0.03
        prev_time = loop_start

        # read live HSV thresholds
        calib = read_calibration_from_window()
        lower = np.array([calib["low_h"], calib["low_s"], calib["low_v"]])
        upper = np.array([calib["high_h"], calib["high_s"], calib["high_v"]])

        ret, frame = cap.read()
        if not ret:
            print("Could not read frame, exiting.")
            break

        # --- vision pipeline ---
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        detections = []        # centroids
        detections_info = []   # (x, y, r)
        for c in contours:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius >= config.MIN_TARGET_RADIUS:
                detections.append((x, y))
                detections_info.append((x, y, radius))

        tracked = tracker.step(detections, loop_start)

        # map each tracked id to the nearest detection radius (for annotation)
        det_map = {}
        for x, y, r in detections_info:
            best_id = None
            best_dist = None
            for tid, ctr in tracked:
                dist = math.hypot(ctr[0] - x, ctr[1] - y)
                if best_id is None or dist < best_dist:
                    best_id = tid
                    best_dist = dist
            if best_id is not None and best_dist <= config.TRACK_MATCH_DIST:
                det_map[best_id] = (x, y, r)

        # target selection: largest radius that is not on hit-cooldown
        chosen = None
        if detections_info:
            candidates = sorted(detections_info, key=lambda t: t[2], reverse=True)
            for x, y, r in candidates:
                matched_id = None
                for tid, ctr in tracked:
                    if math.hypot(ctr[0] - x, ctr[1] - y) <= config.TRACK_MATCH_DIST:
                        matched_id = tid
                        break
                if matched_id is not None and matched_id in hit_targets:
                    if loop_start - hit_targets[matched_id] < config.HIT_COOLDOWN:
                        continue
                chosen = (matched_id, x, y, r)
                break

        laser_state = 0
        target_pan_angle = 90
        target_tilt_angle = 90
        target_info_list = []

        if chosen:
            matched_id, x, y, radius = chosen
            target_pan_angle = np.interp(x, [0, config.FRAME_WIDTH],
                                         [config.PAN_MAX_ANGLE, config.PAN_MIN_ANGLE])
            target_tilt_angle = np.interp(y, [0, config.FRAME_HEIGHT],
                                          [config.TILT_MIN_ANGLE, config.TILT_MAX_ANGLE])

            # deadband: if the target is already centred, hold the aim so the
            # servos don't jitter back and forth around the centre
            cx, cy = config.FRAME_WIDTH / 2, config.FRAME_HEIGHT / 2
            if abs(x - cx) <= config.DEADBAND_PX and abs(y - cy) <= config.DEADBAND_PX:
                target_pan_angle = current_pan_angle
                target_tilt_angle = current_tilt_angle

            for tid, ctr in tracked:
                r = det_map.get(tid, (ctr[0], ctr[1], 0))[2]
                target_info_list.append((tid, (ctr[0], ctr[1], r)))
                cv2.circle(frame, (int(ctr[0]), int(ctr[1])), 4, (0, 255, 0), -1)
                cv2.putText(frame, f"ID:{tid}", (int(ctr[0]) + 6, int(ctr[1]) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

            # lock-on logic (track based)
            if locked_target_id == matched_id and matched_id is not None:
                laser_state = 1
                elapsed = loop_start - lock_start_time
                status_text = f"LOCKING ID:{matched_id} [{elapsed:.2f}s]"
                if elapsed >= config.LOCK_ON_DURATION:
                    key = matched_id if matched_id is not None else f"raw_{int(x)}_{int(y)}"
                    hit_targets[key] = loop_start
                    if shot_sound:
                        shot_sound.play()
                    print(f"!!! TARGET HIT -> ID:{matched_id} (r={radius:.1f})")
                    laser_state = 0
                    locked_target_id = None
                    white = np.ones_like(frame, dtype=np.uint8) * 255
                    cv2.imshow(config.WINDOW_NAME, white)
                    cv2.waitKey(80)
            else:
                locked_target_id = matched_id
                lock_start_time = loop_start
                laser_state = 1
                status_text = f"NEW TARGET - ID:{matched_id}"

            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        else:
            status_text = "NO VALID TARGET - STANDBY"
            locked_target_id = None
            laser_state = 0

        # expire old hit records
        to_delete = [tid for tid, ttime in hit_targets.items()
                     if loop_start - ttime > config.HIT_COOLDOWN]
        for tid in to_delete:
            del hit_targets[tid]

        # --- motion smoothing: time-constant based, frame-rate independent ---
        if config.MOTOR_RESPONSE_TC <= 0:
            alpha = 1.0
        else:
            alpha = 1 - math.exp(-dt / config.MOTOR_RESPONSE_TC)
        current_pan_angle += (target_pan_angle - current_pan_angle) * alpha
        current_tilt_angle += (target_tilt_angle - current_tilt_angle) * alpha

        # --- laser safety: cap continuous on-time ---
        if laser_state == 1:
            if laser_on_start is None:
                laser_on_start = loop_start
            elif loop_start - laser_on_start > config.LASER_MAX_ON_TIME:
                laser_state = 0  # force off as a safety guard
        else:
            laser_on_start = None

        # --- send command / simulate ---
        command = f"{int(current_pan_angle)},{int(current_tilt_angle)},{int(laser_state)}\n"
        if arduino_connected:
            try:
                arduino.write(command.encode())
            except Exception as e:
                print(f"[Arduino] Write error: {e}")
                arduino_connected = False
        else:
            if int(loop_start * 2) % 2 == 0:  # throttle sim prints
                print(f"[SimCmd] {command.strip()}")

        # --- FPS ---
        if dt > 0:
            fps = 1.0 / dt
            smoothed_fps = (fps if smoothed_fps is None
                            else smoothed_fps * config.LOOP_FPS_SMOOTH + fps * (1 - config.LOOP_FPS_SMOOTH))

        draw_hud(frame, status_text,
                 [(tid, (int(x), int(y), int(r))) for tid, (x, y, r) in target_info_list],
                 fps=smoothed_fps)
        cv2.imshow(config.WINDOW_NAME, frame)
        cv2.imshow(config.CALIB_WINDOW, mask)  # show mask to ease calibration

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            save_calibration({
                "low_h": calib["low_h"], "low_s": calib["low_s"], "low_v": calib["low_v"],
                "high_h": calib["high_h"], "high_s": calib["high_s"], "high_v": calib["high_v"],
            })

    # --- shutdown ---
    print("Shutting down...")
    try:
        if arduino_connected:
            arduino.write(b"90,90,0\n")  # centre and laser off
            time.sleep(0.3)
            arduino.close()
    except Exception:
        pass
    cap.release()
    cv2.destroyAllWindows()
    print("System stopped.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="BOZGUN turret control software.")
    parser.add_argument("--sim", action="store_true",
                        help="Force simulation mode: never open the serial port.")
    parser.add_argument("--demo", action="store_true",
                        help="Hardware-free demo: synthetic target, no webcam or Arduino.")
    args = parser.parse_args()
    main(force_sim=args.sim, demo_mode=args.demo)
