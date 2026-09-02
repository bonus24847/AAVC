#!/usr/bin/env python3
"""Is the telemetry radio SAYING anything, or is the wire dead?

    .venv/bin/python tools/serial_sniff.py                  # auto-find the radio
    .venv/bin/python tools/serial_sniff.py --bauds 460800
    .venv/bin/python tools/serial_sniff.py --dev /dev/ttyUSB0 --seconds 15

The console can only report "no signal". That one phrase covers three very
different faults with three different fixes:

  * the wire is electrically silent (cable, connector, module not sending),
  * bytes arrive but are not MAVLink (the radio is in the wrong protocol mode),
  * bytes arrive as MAVLink but at another baud (a config mismatch).

This tool separates them by reading RAW BYTES and nothing else, at each baud in
turn, and printing what actually landed.

Why it exists (2026-08-25): the NOMAD ground module enumerated fine as a CP2102
and the console still showed "no signal" for 20 minutes. The thing that finally
told the truth was ``stat /dev/ttyUSB0``: the kernel bumps a tty's **mtime** on
every write and its **atime** only when a read RETURNS BYTES, so an atime frozen
at the moment of plug-in — while mtime tracked the console's heartbeats — proved
zero bytes had ever arrived. That reading is one command away and nobody knows
it; this tool makes the same measurement obvious, and adds the baud sweep, so a
wrong baud (which WOULD deliver garbage bytes) can never be confused with a
silent line again.

⚠ The port is exclusive in practice: stop the console first (its icon →
"⏹ ปิดทุกอย่าง"), or this tool will refuse and name the process holding it —
two readers on one tty split the byte stream and both then see nonsense.

Exit: 0 MAVLink frames decoded · 1 bytes arrived but no MAVLink at any baud ·
2 the line was silent at EVERY baud · 3 could not open the port.
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
import time

# MAVLink start-of-frame bytes. Counting these is a cheap "is this even the
# right protocol" test that needs no dialect and no pymavlink.
MAGIC_V1 = 0xFE
MAGIC_V2 = 0xFD


def find_device() -> str | None:
    """The radio, chosen exactly the way cm4/pick_telemetry_link.sh chooses it —
    so this tool and the console can never disagree about which port is 'the
    radio'."""
    for pattern in ("/dev/serial/by-id/*CP2102*", "/dev/serial/by-id/*Silicon_Labs*"):
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    hits = sorted(glob.glob("/dev/ttyUSB*"))
    return hits[0] if hits else None


def holders(dev: str) -> list[tuple[int, str]]:
    """Every process holding an fd on this device — by resolved path, so the
    by-id symlink and /dev/ttyUSB0 count as the same port."""
    target = os.path.realpath(dev)
    found: list[tuple[int, str]] = []
    for proc in glob.glob("/proc/[0-9]*"):
        fd_dir = os.path.join(proc, "fd")
        try:
            names = os.listdir(fd_dir)
        except OSError:
            continue                      # gone, or not ours to look at
        for name in names:
            try:
                if os.path.realpath(os.path.join(fd_dir, name)) != target:
                    continue
                with open(os.path.join(proc, "cmdline"), "rb") as fh:
                    cmd = fh.read().replace(b"\0", b" ").decode(errors="replace")
            except OSError:
                continue
            found.append((int(os.path.basename(proc)), cmd.strip()))
            break
    return found


def port_times(dev: str) -> str:
    """The atime/mtime reading described in the module docstring."""
    try:
        st = os.stat(os.path.realpath(dev))
    except OSError as exc:
        return f"(cannot stat: {exc})"
    fmt = "%H:%M:%S"
    return (f"atime {time.strftime(fmt, time.localtime(st.st_atime))} "
            f"(last read that RETURNED bytes) · "
            f"mtime {time.strftime(fmt, time.localtime(st.st_mtime))} "
            f"(last write)")


def parse_mavlink(buf: bytes) -> tuple[int, list[str]]:
    """(frame count, first few message names) using pymavlink if it is here."""
    try:
        from pymavlink.dialects.v20 import common as dialect
    except Exception:                                     # noqa: BLE001
        return -1, []
    mav = dialect.MAVLink(None)
    mav.robust_parsing = True
    names: list[str] = []
    count = 0
    try:
        msgs = mav.parse_buffer(buf) or []
    except Exception:                                     # noqa: BLE001
        return 0, []
    for msg in msgs:
        if msg.get_type() == "BAD_DATA":
            continue
        count += 1
        label = f"{msg.get_type()}(sys={msg.get_srcSystem()},comp={msg.get_srcComponent()})"
        if label not in names and len(names) < 6:
            names.append(label)
    return count, names


def sniff(dev: str, baud: int, seconds: float) -> bytes:
    import serial

    # rtscts/dsrdtr off: this is a 3-wire link, and letting pyserial wait on a
    # handshake line the module never drives would read as "silent" for the
    # wrong reason.
    with serial.Serial(dev, baud, timeout=0.2, rtscts=False, dsrdtr=False) as port:
        try:
            print(f"    modem lines: cts={port.cts} dsr={port.dsr} "
                  f"cd={port.cd} ri={port.ri}")
        except Exception:                                 # noqa: BLE001
            pass
        port.reset_input_buffer()
        buf = bytearray()
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            chunk = port.read(4096)
            if chunk:
                buf += chunk
        return bytes(buf)


def report(baud: int, buf: bytes, seconds: float) -> tuple[int, int]:
    """Print one baud's result. Returns (bytes, mavlink frames)."""
    n = len(buf)
    if not n:
        print(f"  {baud:>7} : 0 bytes — SILENT")
        return 0, 0
    magic = buf.count(bytes([MAGIC_V2])) + buf.count(bytes([MAGIC_V1]))
    frames, names = parse_mavlink(buf)
    head = " ".join(f"{b:02x}" for b in buf[:32])
    print(f"  {baud:>7} : {n} bytes ({n / seconds:.0f} B/s) · "
          f"magic 0xFD/0xFE x{magic} · "
          f"MAVLink frames {'(pymavlink missing)' if frames < 0 else frames}")
    print(f"            first 32: {head}")
    if names:
        print(f"            saw: {', '.join(names)}")
    return n, max(frames, 0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dev", default=None,
                    help="serial device (default: the radio, found the way "
                         "cm4/pick_telemetry_link.sh finds it)")
    ap.add_argument("--bauds", default="460800,115200,57600",
                    help="comma-separated bauds to try (default 460800,115200,57600)")
    ap.add_argument("--seconds", type=float, default=8.0,
                    help="seconds to listen at each baud (default 8)")
    ap.add_argument("--force", action="store_true",
                    help="sniff even while another process holds the port "
                         "(the byte stream is then SPLIT between the two — "
                         "expect nonsense from both)")
    args = ap.parse_args()

    dev = args.dev or find_device()
    if not dev:
        print("[sniff] no serial device found (/dev/serial/by-id/*CP2102*, "
              "/dev/ttyUSB*) — is the radio plugged in?")
        return 3
    print(f"[sniff] device: {dev}")
    if os.path.realpath(dev) != dev:
        print(f"[sniff]         -> {os.path.realpath(dev)}")
    print(f"[sniff] {port_times(dev)}")

    held = holders(dev)
    if held and not args.force:
        for pid, cmd in held:
            print(f"[sniff] port is HELD by pid {pid}: {cmd[:110]}")
        print("[sniff] stop the console first (icon → '⏹ ปิดทุกอย่าง'), or pass "
              "--force to read anyway. Two readers split the stream.")
        return 3

    bauds = [int(b) for b in args.bauds.split(",") if b.strip()]
    total_bytes = 0
    total_frames = 0
    for baud in bauds:
        print(f"[sniff] listening {args.seconds:.0f}s @ {baud} …")
        try:
            buf = sniff(dev, baud, args.seconds)
        except Exception as exc:                          # noqa: BLE001
            print(f"  {baud:>7} : cannot open — {exc}")
            continue
        n, frames = report(baud, buf, args.seconds)
        total_bytes += n
        total_frames += frames

    print(f"[sniff] {port_times(dev)}")
    if total_frames:
        print("[sniff] OK: MAVLink is arriving. If the console still says "
              "'no signal', the console's baud is the thing that disagrees.")
        return 0
    if total_bytes:
        print("[sniff] PROBLEM: bytes arrive but none of them are MAVLink — the "
              "radio is not in MAVLink mode (check the ELRS protocol setting on "
              "the TX side; the RX side is proven by the FC seeing RADIO_STATUS).")
        return 1
    print("[sniff] PROBLEM: the line is SILENT at every baud — nothing is being "
          "transmitted into the USB bridge at all. This is NOT a baud or a "
          "protocol fault (either would still deliver bytes). Look at the "
          "physical link: USB cable, connector, and whether the module's serial "
          "output is alive (power-cycle the handset with the cable in place).")
    return 2


if __name__ == "__main__":
    sys.exit(main())
