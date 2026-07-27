"""Download the newest PX4 .ulg off the FMU SD card over MAVLink (MAVFTP).

Used by gcs_server's "⬇️ ดึง log ล่าสุด" button: it borrows the GCS's already-open
mavutil connection (the GCS pauses its reader/heartbeat/poll threads first so this
is the SOLE consumer of the link), finds the newest log under /fs/microsd/log, and
saves it under LOG_DIR on the CM4 -- the browser then downloads it to the laptop.

MAVFTP drives its own recv loop on `master` (process_ftp_reply / cmd_list call
master.recv_match internally), which is exactly why the caller must pause every
other thread that touches the link before calling pull_newest_ulog().
"""
from __future__ import annotations

import os
import time

from pymavlink import mavftp

LOG_ROOT = "/fs/microsd/log"


def _list(ftp, path):
    """List one remote directory -> [DirectoryEntry(name, is_dir, size_b)]."""
    ftp.cmd_list([path])
    return list(getattr(ftp, "list_result", []) or [])


def find_newest_ulog(ftp, note=print):
    """Return (remote_path, size_bytes) of the newest .ulg, or None.

    PX4 lays logs out as /fs/microsd/log/<dated-or-sess-dir>/<name>.ulg (and
    occasionally directly under log/). Both dated (YYYY-MM-DD/HH_MM_SS) and
    sessNNN/logNNN names sort chronologically as strings, so 'max by path' = newest.
    """
    cands = []  # (fullpath, size)
    for e in _list(ftp, LOG_ROOT):
        if e.is_dir:
            sub = LOG_ROOT + "/" + e.name
            for f in _list(ftp, sub):
                if (not f.is_dir) and f.name.endswith(".ulg"):
                    cands.append((sub + "/" + f.name, f.size_b))
        elif (not e.is_dir) and e.name.endswith(".ulg"):
            cands.append((LOG_ROOT + "/" + e.name, e.size_b))
    if not cands:
        return None
    cands.sort(key=lambda c: c[0])
    return cands[-1]


def pull_newest_ulog(master, dest_dir, note=print, timeout=900):
    """Find + download the newest .ulg to dest_dir. Returns the local path or None.

    `master` is a connected mavutil link with target_system/component set; the
    caller MUST have paused all other users of this link first.
    """
    dest_dir = os.path.expanduser(dest_dir)
    os.makedirs(dest_dir, exist_ok=True)
    ts = int(getattr(master, "target_system", 0) or 1)
    tc = int(getattr(master, "target_component", 0) or 1)
    ftp = mavftp.MAVFTP(master, ts, tc)

    note("[log] หาไฟล์ใหม่สุดบน SD card …")
    newest = find_newest_ulog(ftp, note)
    if not newest:
        note("[log] ไม่พบ .ulg บน SD card")
        return None
    remote, size = newest

    # local name = <parent-dir>_<file> so logs from different sessions don't collide
    base = os.path.basename(remote)
    parent = os.path.basename(os.path.dirname(remote))
    local_name = f"{parent}_{base}" if parent and parent != "log" else base
    local = os.path.join(dest_dir, local_name)

    kb = (size or 0) / 1024.0
    note(f"[log] ดาวน์โหลด {remote} ({kb:.0f} KB) …")
    t0 = time.time()
    ftp.cmd_get([remote, local])
    ret = ftp.process_ftp_reply("OpenFileRO", timeout=timeout)
    dt = time.time() - t0

    if not os.path.exists(local) or os.path.getsize(local) == 0:
        note(f"[log] ดาวน์โหลดไม่สำเร็จ ({ret})")
        return None
    note(f"[log] เสร็จ: {local_name} ({os.path.getsize(local)/1024:.0f} KB, {dt:.0f}s)")
    return local
