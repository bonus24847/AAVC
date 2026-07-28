# AAVC ArUco Scanner

Detects the **AAVC touch-and-go pad markers** — ArUco **DICT_4X4_50, IDs 1–6** —
from a USB camera (e.g. the **WSD-9781-V12**) and reports each marker's ID, pixel
centre, and normalised offset from the frame centre (for visual centring during
landing).

Built to run **smoothly headless on the Raspberry Pi CM4** (the Pixhawk companion):
- a **threaded grabber** always holds the freshest frame, so detection never lags
  behind the camera or piles up a buffer (the key to a low-latency feed);
- **MJPG** transport + a **1-frame** driver buffer + a 30 fps camera cap;
- a **`--fps` detection cap** (default 20) so CPU stays low and steady;
- the USB camera's `/dev/videoN` node is **auto-detected** — it isn't stable across
  reboots on the Pi, so the scanner finds the live capture node itself.

## Run
```bash
pip install -r requirements.txt          # or, on the Pi:  sudo apt install python3-opencv

# Pi, headless — auto-detects the camera node, prints detections:
python aruco_scan.py

# force a specific camera node, or tune the CPU budget:
python aruco_scan.py --device /dev/video1
python aruco_scan.py --fps 15 --width 640 --height 480
```

## Output
Each change prints, e.g. `[aruco] ID 3 @(childx,y) dx+0.12 dy-0.34`:
- **`id`** — the marker ID (`1–6` = a real AAVC pad; other IDs are flagged `(not pad)`).
- **`dx, dy`** — offset from frame centre, normalised to `[-1, 1]` (right / down positive).
  Drive these toward `0` to centre the drone over the pad.

## For whoever continues this (mission integration)
- `Camera` (the threaded grabber) and `detect()` are importable. Call
  `detect(detector, gray, w, h)` to get a list of `{id, cx, cy, dx, dy, pad}` per frame
  and feed it to the flight controller's centring loop.
- **Pose (distance / angle)** needs camera calibration. Once you have the
  WSD-9781-V12 intrinsics (`cameraMatrix`, `distCoeffs`) and the 400 mm marker size,
  add `cv2.aruco.estimatePoseSingleMarkers` (or `solvePnP`) to get metric pose.
- Depends only on **OpenCV (with the `aruco` module)** + the Python standard library.

## Requirements
Python 3.8+, OpenCV ≥ 4.7 with `cv2.aruco` (`opencv-contrib-python`, or the distro's
`python3-opencv`). Tested on OpenCV 4.10 (Pi CM4 / Debian 13) and 4.13.
