import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import bozgun  # noqa: E402
import config  # noqa: E402


def test_new_detections_get_increasing_ids():
    tracker = bozgun.SimpleTracker()
    live = tracker.step([(10, 10), (300, 300)], timestamp=0.0)
    assert [tid for tid, _ in live] == [1, 2]


def test_track_keeps_its_id_while_moving_within_match_distance():
    tracker = bozgun.SimpleTracker(match_dist=60)
    tracker.step([(100, 100)], timestamp=0.0)
    live = tracker.step([(130, 120)], timestamp=0.1)
    assert live == [(1, (130, 120))]


def test_jump_beyond_match_distance_starts_a_new_track():
    tracker = bozgun.SimpleTracker(match_dist=60)
    tracker.step([(100, 100)], timestamp=0.0)
    live = dict(tracker.step([(400, 400)], timestamp=0.1))
    assert live[2] == (400, 400)
    assert 1 in live


def test_unseen_track_expires_after_max_miss_time():
    tracker = bozgun.SimpleTracker(max_miss_seconds=0.5)
    tracker.step([(100, 100)], timestamp=0.0)
    assert tracker.step([], timestamp=0.4) == [(1, (100, 100))]
    assert tracker.step([], timestamp=0.6) == []
    assert tracker.get_track_by_id(1) is None


def test_two_tracks_do_not_steal_the_same_detection():
    tracker = bozgun.SimpleTracker(match_dist=60)
    tracker.step([(100, 100), (140, 100)], timestamp=0.0)
    live = dict(tracker.step([(120, 100)], timestamp=0.1))
    assert sum(1 for c in live.values() if c == (120, 100)) == 1


def test_calibration_round_trip(tmp_path):
    path = tmp_path / "calib.json"
    cfg = {"low_h": 1, "low_s": 2, "low_v": 3, "high_h": 4, "high_s": 5, "high_v": 6}
    bozgun.save_calibration(cfg, path=str(path))
    assert bozgun.load_calibration(path=str(path)) == cfg


def test_missing_or_broken_calibration_falls_back_to_defaults(tmp_path):
    assert bozgun.load_calibration(path=str(tmp_path / "none.json")) == config.DEFAULT_HSV
    broken = tmp_path / "broken.json"
    broken.write_text("{not json")
    assert bozgun.load_calibration(path=str(broken)) == config.DEFAULT_HSV


def test_defaults_are_returned_as_a_copy(tmp_path):
    cfg = bozgun.load_calibration(path=str(tmp_path / "none.json"))
    cfg["low_h"] = 0
    assert config.DEFAULT_HSV["low_h"] != 0


def test_synthetic_target_is_detected_by_default_hsv_range():
    cam = bozgun.SyntheticCamera(config.FRAME_WIDTH, config.FRAME_HEIGHT)
    ok, frame = cam.read()
    assert ok and frame.shape == (config.FRAME_HEIGHT, config.FRAME_WIDTH, 3)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    d = config.DEFAULT_HSV
    mask = cv2.inRange(hsv, np.array([d["low_h"], d["low_s"], d["low_v"]]),
                       np.array([d["high_h"], d["high_s"], d["high_v"]]))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    (_, _), radius = cv2.minEnclosingCircle(max(contours, key=cv2.contourArea))
    assert radius >= config.MIN_TARGET_RADIUS
    cam.release()
    assert not cam.isOpened()


def test_safety_limits_are_consistent():
    assert 0 <= config.PAN_MIN_ANGLE < config.PAN_MAX_ANGLE <= 180
    assert 0 <= config.TILT_MIN_ANGLE < config.TILT_MAX_ANGLE <= 180
    assert config.LOCK_ON_DURATION < config.LASER_MAX_ON_TIME
