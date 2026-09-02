#!/usr/bin/env python3
"""Measure how the nadir camera is BOLTED — the rotation nobody wrote down.

    .venv/bin/python tools/measure_mount_yaw.py                  # marker/pad in frame
    .venv/bin/python tools/measure_mount_yaw.py --pixel 812,240  # any object, by pixel
    .venv/bin/python tools/measure_mount_yaw.py --grid /tmp/g.jpg  # annotated copy

``cameras.nadir.mount_yaw_deg`` is the rotation of the camera about its optical
axis relative to the airframe: 0 = the image's "up" (row 0) points at the NOSE
and image-right at the right wing. It has NO symptom of its own — every
pixel->lat/lon answer simply comes out rotated by it about the aircraft. At the
12 m sweep a frame-edge pad sits ~9 m out, so 90 deg of unrecorded mount
rotation reports that pad ~13 m from where it is: the aircraft flies to empty
grass, never re-acquires, and the tracker's cluster never confirms. It also
sets the heading the sweep holds (``search_pattern.sweep_yaw_deg``, derived
as leg bearing - this and then flipped 180 if that would fly the leg
backwards), i.e. whether the WIDE image axis lands across track.

MEASURED on this airframe 2026-08-23: **180** — the camera is bolted upside
down. Re-run this after ANY camera re-mount; the four bench placements that
pin the current number are kept in ``tests/test_measure_mount_yaw.py``.

⚠ SITL cannot check it. The gz camera's pose is roll 0 yaw 0 — the very
assumption ``CameraModel`` makes — so sim agrees with the code no matter what
the real aircraft does. It has to be read off the airframe.

FIELD PROCEDURE (5 minutes, motors off, props on or off — nothing spins)
 1. The parked aircraft's lens sits ~3.5 cm off the ground: it can see nothing
    useful and cannot focus that close. ELEVATE it — a table, two chairs, a
    crate — so the lens looks down at the floor from **0.8-1.5 m**. Keep it
    LEVEL; a tilt just shifts the object, it does not rotate it, but a big
    tilt costs accuracy.
 2. Power the FC + CM4 and start the camera so frames are being written
    (``bash cm4/start_infra.sh`` on the CM4). INDOORS run the grabber with
    ``CAM_EXPOSURE=0``: the 2 ms daylight default makes an indoor frame black.
 3. Put a distinctive object on the floor **straight out from the NOSE**,
    20-40 cm from the point directly under the lens. A printed ArUco pad is
    ideal (this tool finds it for you); a roll of tape, a phone, or a coin on
    white paper works with ``--pixel``.
 4. Run this tool. It prints the angle, snaps it to the nearest 90 deg, and
    gives you the exact config line. It REFUSES a frame file older than 10 s
    (``--max-age-s``): a grabber can keep running on a dead camera descriptor
    and the stale picture would measure a confident wrong angle.
 5. Do it a second time with the object somewhere else — the right wing, say,
    with ``--object-bearing-deg 90``. One placement cannot tell a correct
    reading from a formula that is wrong everywhere except where you looked;
    that is exactly how a sign error in THIS function survived until
    2026-08-23.

WHY IT ASKS FOR "TOWARD THE NOSE": that direction is the one you can place by
eye without a compass, and it is the only one the answer depends on. If it is
easier to put the object somewhere else, measure that direction relative to the
nose (clockwise seen from above) and pass ``--object-bearing-deg``.

Exit: 0 measured · 2 no usable frame (missing, unreadable, or STALE)
      · 3 nothing found in the frame.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2  # noqa: E402

from vision.detectors.aruco import find_landing_pads  # noqa: E402

DEFAULT_FRAME = Path("/tmp/aavc_nadir.jpg")
# A frame file older than this is not a picture of the aircraft as it is posed
# now. It matters here more than almost anywhere: this tool's answer gets typed
# into a config and shapes every pad fix afterwards, and a stale frame yields a
# confident WRONG angle with no other symptom. It has already happened — on
# 2026-08-23 the OV9281 browned out and re-enumerated (video0 -> video1) while
# the grabber kept running on the dead descriptor, and three "no marker found"
# results in a row were really one 14-minute-old picture of the floor. Fail
# closed, like tools/verify_flight.py: refuse the frame rather than measure it.
_MAX_FRAME_AGE_S = 10.0
# Below this the object is too near the centre for the angle to mean anything:
# at 60 px a 5 px centroid wobble is already 5 deg.
_MIN_OFFSET_PX = 60.0


def _frame_age_s(path: Path, *, now: float | None = None) -> float | None:
    """Seconds since the frame file was last written, or None if absent.

    Wall clock on purpose: the question is "is the grabber still feeding this
    file", which is about the host process, not the aircraft's flight clock.
    """
    try:
        mtime = path.stat().st_mtime
    except OSError:
        return None
    return max(0.0, (time.time() if now is None else now) - mtime)


def mount_yaw_deg(u: float, v: float, w: int, h: int,
                  object_bearing_deg: float = 0.0) -> float:
    """Mount rotation from where a known-direction object lands in the frame.

    ``vision/projection.py`` maps image-right to +rx and image-UP to +ry, then
    un-rotates the pair by psi before reading body forward/right off it. Write
    ``theta`` for the object's CLOCKWISE angle from image-up,

        theta = atan2(u - cx, cy - v)

    and that un-rotation makes the body bearing come out as ``b = theta + psi``,
    so the mount is read back as ``psi = b - theta``. Since this function's
    ``ang`` is ``-theta`` (it takes ``cx - u``, not ``u - cx``), that is

        psi = ang + b

    — a PLUS. It was a minus until 2026-08-23, which is invisible at the
    documented ``b = 0`` (nose) placement and wrong by ``2b`` anywhere else: the
    confirmation placement out at the right wing read 7 deg where the aircraft's
    own projection says 187. Four placements now pin it (see
    ``tests/test_measure_mount_yaw.py``).

    (``cy - v`` because image rows grow DOWNWARD — that flip is the other place
    this measurement usually goes wrong, which is why this function exists
    instead of a note in a runbook.)"""
    ang = math.degrees(math.atan2(w / 2.0 - u, h / 2.0 - v))
    return (ang + object_bearing_deg) % 360.0


def _annotate(img, out: Path, *, u: float | None = None,
              v: float | None = None, psi: float | None = None) -> None:
    """Frame + a labelled 100 px ruler, so the pixel can be READ OFF by eye.

    The ruler is the point when the detector finds nothing: open this file,
    look at the object, read roughly (810, 240) off the grid, re-run with
    ``--pixel 810,240``. Ten pixels of eyeball error at a 250 px offset is
    ~2 deg — far below the 12 deg the snap-to-90 check tolerates."""
    h, w = img.shape[:2]
    for x in range(0, w, 100):
        cv2.line(img, (x, 0), (x, h), (90, 90, 90), 1)
        cv2.putText(img, str(x), (x + 3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (200, 200, 200), 1)
    for y in range(0, h, 100):
        cv2.line(img, (0, y), (w, y), (90, 90, 90), 1)
        cv2.putText(img, str(y), (3, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (200, 200, 200), 1)
    cx, cy = int(w / 2), int(h / 2)
    cv2.line(img, (cx - 30, cy), (cx + 30, cy), (0, 0, 255), 2)
    cv2.line(img, (cx, cy - 30), (cx, cy + 30), (0, 0, 255), 2)
    if u is not None and v is not None:
        cv2.circle(img, (int(u), int(v)), 18, (0, 200, 255), 3)
        cv2.line(img, (cx, cy), (int(u), int(v)), (0, 200, 255), 2)
    if psi is not None:
        cv2.putText(img, f"mount_yaw = {psi:.1f} deg", (20, 46),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 2)
    cv2.putText(img, "red cross = image centre; grid = 100 px",
                (20, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)
    cv2.imwrite(str(out), img)
    print(f"[mount] ภาพพร้อมเส้นกริด → {out}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--frame", type=Path, default=DEFAULT_FRAME,
                    help=f"nadir frame to read (default {DEFAULT_FRAME})")
    ap.add_argument("--pixel", help="object centre as U,V — skips the detector "
                                    "(use for a non-marker object)")
    ap.add_argument("--object-bearing-deg", type=float, default=0.0,
                    help="where the object actually lies relative to the NOSE, "
                         "clockwise seen from above (default 0 = straight ahead)")
    ap.add_argument("--grid", type=Path,
                    help="write a copy with a labelled 100 px ruler (+ the "
                         "object and angle when found) — use it to read the "
                         "pixel off by eye when there is no marker to detect")
    ap.add_argument("--max-age-s", type=float, default=_MAX_FRAME_AGE_S,
                    help=f"refuse a frame file older than this many seconds "
                         f"(default {_MAX_FRAME_AGE_S:g}; 0 disables — only for "
                         "measuring from a saved frame)")
    args = ap.parse_args()

    age = _frame_age_s(args.frame)
    if age is None:
        print(f"ไม่พบไฟล์เฟรม: {args.frame}")
        return 2
    if args.max_age_s > 0 and age > args.max_age_s:
        print(f"เฟรมเก่า {age:.0f} วินาที (เกิน {args.max_age_s:g}) — "
              f"{args.frame} ไม่ได้ถูกเขียนใหม่")
        print("  กล้องน่าจะหลุด/ค้าง: grabber ยังรันอยู่ได้ทั้งที่เฟรมไม่ขยับ")
        print("  เช็ค: pgrep -af 'camera_grabber.p[y]' และ dmesg | tail")
        print("  ถ้าจงใจวัดจากภาพที่เซฟไว้ ให้ใส่ --max-age-s 0")
        return 2

    img = cv2.imread(str(args.frame))
    if img is None:
        print(f"อ่านเฟรมไม่ได้: {args.frame} — กล้อง/grabber ทำงานอยู่ไหม "
              "(ในร่มต้องตั้ง CAM_EXPOSURE=0 ไม่งั้นภาพดำ)")
        return 2
    h, w = img.shape[:2]

    if args.pixel:
        try:
            u_s, v_s = args.pixel.split(",")
            u, v, src = float(u_s), float(v_s), "พิกัดที่ระบุ"
        except ValueError:
            print("--pixel ต้องเป็นรูปแบบ U,V เช่น 812,240")
            return 3
    else:
        hits = find_landing_pads(img)
        if not hits:
            print("ไม่เจอ pad/marker ในเฟรมนี้ — วาง ArUco ที่พิมพ์ไว้ให้เห็น "
                  "หรือใช้ --pixel U,V กับวัตถุอะไรก็ได้")
            if args.grid:
                _annotate(img, args.grid)
                print("เปิดไฟล์นั้น อ่านพิกัดวัตถุจากเส้นกริด แล้วรันใหม่ด้วย "
                      "--pixel U,V")
            else:
                print("ใส่ --grid /tmp/g.jpg เพื่อได้ภาพพร้อมเส้นกริดไว้อ่านพิกัดเอง")
            return 3
        best = hits[0]
        u, v = float(best.cx), float(best.cy)
        what = (f"ArUco id {best.marker_id}" if best.marker_id is not None
                else "pad ขาว (ยังไม่ถอดรหัส)")
        src = f"ตรวจพบ {what}"

    off = math.hypot(u - w / 2.0, v - h / 2.0)
    psi = mount_yaw_deg(u, v, w, h, args.object_bearing_deg)
    snapped = round(psi / 90.0) * 90 % 360
    residual = abs((psi - snapped + 180) % 360 - 180)

    print(f"[mount] เฟรม {args.frame}  {w}x{h}")
    print(f"[mount] {src}: พิกเซล ({u:.0f}, {v:.0f}) "
          f"ห่างจากกลางภาพ {off:.0f} px")
    if args.object_bearing_deg:
        print(f"[mount] วัตถุอยู่ที่ทิศ {args.object_bearing_deg:g} deg จากหัวโดรน")
    print()
    print(f"    mount_yaw_deg = {psi:.1f}   →  ปัดเป็น {snapped:g} "
          f"(ห่าง {residual:.1f} deg)")
    print()

    if off < _MIN_OFFSET_PX:
        print(f"⚠ วัตถุอยู่ใกล้กลางภาพเกินไป ({off:.0f} px < {_MIN_OFFSET_PX:.0f}) "
              "— มุมที่ได้เป็นสัญญาณรบกวน ขยับวัตถุออกไปอีก หรือยกโดรนให้สูงขึ้น")
    elif residual > 12.0:
        print(f"⚠ ห่างจากมุมฉากถึง {residual:.1f} deg. แท่นกล้องที่ยึดด้วยน็อต"
              "มักลงตัวที่ 0/90/180/270 — เช็กว่าวางวัตถุตรงแนวหัวจริงไหม "
              "และโดรนวางระนาบไหม แล้ววัดซ้ำ ถ้าซ้ำแล้วยังได้เท่าเดิม "
              "แปลว่าแท่นเอียงจริง — ใช้ค่าดิบ ไม่ใช่ค่าที่ปัด")
    else:
        print("ใส่ค่านี้ในทั้งสอง config (sitl/aavc_config.yaml, "
              "sitl/kmitl_config.yaml):")
        print(f"      mount_yaw_deg: {snapped:g}")
        print("แล้วแก้ tests/test_cameras.py::test_config_nadir_matches_ov9281_profile "
              "ให้ล็อกค่าใหม่ (เทสจะ fail จนกว่าจะแก้ — ตั้งใจให้เป็นแบบนั้น)")
        if snapped in (0, 180):
            print("ถ้าเป็น 0 หรือ 180: แกนกว้างของภาพขวางแนวบินอยู่แล้ว → "
                  "ถอย overlap_frac กลับเป็น 0.30 ได้ (คืนเวลาให้ KMITL 158 วิ)")
        else:
            print("ถ้าเป็น 90 หรือ 270: แกนกว้างวางตามแนวลำ → "
                  "คง overlap_frac 0.44 ไว้ และ sweep จะถือหัวขวางขากวาดเอง")

    if args.grid:
        _annotate(img, args.grid, u=u, v=v, psi=psi)
    return 0


if __name__ == "__main__":
    sys.exit(main())
