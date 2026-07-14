# BOZGUN — Autonomous Colour-Tracking Turret

**Real-time, colour-based target tracking and laser marking on low-cost hardware.**
A standard webcam finds a colour-marked target (a **blue LED**, matching the
default HSV range), a Python/OpenCV pipeline converts its pixel position into
pan/tilt servo angles, and a battery-powered Arduino aims the turret's
**red laser** at it. When a target stays centred long enough, the system
performs a **simulated** "hit" — the laser marks it and a fire sound plays.
No projectile is involved.

**Status:** Completed — course prototype (Robotics, 2025).
**Stack:** Python · OpenCV · NumPy · pySerial · Arduino (Servo)

> ⚠️ **Scope & safety.** This is an educational / simulation project, **not** a
> weapon or defence system. The "fire" action is purely a visual + audio cue.
> Use only a **low-power** laser module and follow standard laser eye-safety
> rules; the demo is designed so the laser is easy to disable in firmware.

---

## What it does

- **Real-time colour detection** — BGR→HSV, `inRange` thresholding, morphological
  cleanup (erode/dilate), contour extraction and `minEnclosingCircle`, with a
  minimum-radius filter to reject noise.
- **Live HSV calibration** — a trackbar window lets you tune the six HSV bounds
  on the fly; the mask is shown alongside the feed. Press **S** to save the
  thresholds to `hsv_calib.json`, which is auto-loaded on the next run.
- **Memory-based centroid tracker** — a small custom tracker assigns stable IDs
  to detected blobs using nearest-neighbour matching, and drops a track only
  after it has been unseen for a short grace period.
- **Target selection with cooldown** — the largest valid target is chosen; a
  target that was just "hit" is ignored for a cooldown window so the turret
  moves on to others.
- **Pixel → servo-angle mapping** — target coordinates are linearly mapped to
  pan/tilt angles (`np.interp`) and clamped to the turret's safe mechanical
  range.
- **Frame-rate-independent motion smoothing** — servo commands are eased with a
  time-constant filter (`alpha = 1 − e^(−dt/τ)`) so the aim stays smooth
  regardless of the current frame rate.
- **Centre deadband** — when the target is already within a small pixel radius
  of the frame centre, the aim is held instead of re-issued, so the servos stop
  jittering around the centre.
- **Lock-on → simulated hit** — once a target is held for the lock-on duration,
  the laser marks it, a fire sound plays, the screen flashes, and the target
  enters cooldown.
- **Laser safety timeout** — a software guard caps continuous laser-on time, so
  the laser can never stay on indefinitely if the loop stalls.
- **Hardware-free demo mode** — `--demo` renders a synthetic moving blue target
  so the entire pipeline runs with no webcam and no Arduino.
- **Serial link with simulation fallback** — commands are streamed to the
  Arduino as `pan,tilt,laser\n`. If no board is connected (or `--sim` is used),
  the program keeps running and prints the commands it would have sent.
- **HUD overlay** — corner reticle, live status, smoothed FPS, and per-target
  ID / position / radius readouts.

## How it works

```
Webcam ─▶ HSV threshold ─▶ morphology ─▶ contours ─▶ centroid tracker
                                                           │
                                          largest target (not on cooldown)
                                                           │
                          pixel → pan/tilt angle  ─▶  motion smoothing
                                                           │
                              "pan,tilt,laser\n"  ─▶  Arduino ─▶ servos + laser
                                                           │
                                     held for LOCK_ON_DURATION ─▶ simulated hit
```

## Project layout

| File | Purpose |
|------|---------|
| `bozgun.py` | Main control application: vision pipeline, tracker, aiming, serial. |
| `config.py` | All tunable parameters (ports, HSV defaults, servo limits, timings). |
| `firmware/bozgun_firmware.ino` | Reference Arduino firmware implementing the serial protocol. |
| `requirements.txt` | Python dependencies. |

## Hardware

- Standard USB webcam
- Arduino (Uno or similar)
- 2× hobby servos in a pan/tilt mount
- Low-power **red** laser module (turret aiming / fire indicator)
- **Blue LED** used to mark the target object (tracked via the blue HSV range)
- Battery pack powering the servos and electronics

## Serial protocol

One line per command, sent from the PC to the Arduino at 9600 baud:

```
pan,tilt,laser\n      e.g.  90,90,0
```

- `pan`, `tilt` — servo angles in degrees
- `laser` — `0` (off) or `1` (on)

## Quick start

Clone the repository and install the dependencies:

```bash
git clone https://github.com/nisamaasoglu/BOZGUN.git
cd BOZGUN
pip install -r requirements.txt
```

Run **with no hardware at all** (synthetic target — no webcam, no Arduino):

```bash
python bozgun.py --demo
```

Run against a **real webcam but no Arduino** (prints the commands it would send):

```bash
python bozgun.py --sim
```

Run with the turret connected (set `ARDUINO_PORT` and `CAMERA_INDEX` in
`config.py` to match your setup):

```bash
python bozgun.py
```

For the Arduino side, open `firmware/bozgun_firmware.ino` in the Arduino IDE,
set the pin numbers to match your wiring, and upload it.

### Controls

| Key | Action |
|-----|--------|
| `S` | Save the current HSV calibration to `hsv_calib.json` |
| `Q` | Quit |

### Calibration

Start the program, aim the camera at your target object, and adjust the six HSV
trackbars until only the target shows up white in the mask window. Press **S**
to save; the values are reloaded automatically next time.

### Optional sound

Drop a `laser_shot.mp3` next to `bozgun.py` to play a sound on each simulated
hit. The program runs fine without it.

## Notes & limitations

- Detection is colour-based, so lighting changes and same-coloured background
  objects can cause false positives — calibration and the minimum-radius filter
  mitigate this.
- Aiming accuracy is bounded by servo backlash and the linear pixel→angle model.

## Authors

Two-person robotics course project by **Nisa Maaşoğlu**
([@nisamaasoglu](https://github.com/nisamaasoglu)) and **Özge Bilici**
(Aksaray University, Software Engineering, 2025).
This repository contains the PC-side control and computer-vision software.

## License

Released under the [MIT License](LICENSE).
