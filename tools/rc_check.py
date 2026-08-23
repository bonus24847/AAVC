#!/usr/bin/env python3
"""Is the RC link down, or is PX4 just refusing it? They look identical.

    .venv/bin/python tools/rc_check.py                     # via the CM4 router
    .venv/bin/python tools/rc_check.py --endpoint udpout:127.0.0.1:14550

Every RC indicator this project had — MAVSDK ``telemetry.rc_status()``, QGC's
RC bar, the drill script — reads PX4's RC **health** bit. That bit does not
mean "frames are arriving". ``SYS_STATUS.hpp::fillOutComponent`` sets it only
when the ``remote_control`` component has NO arming-check error:

    if (((arming_check_error_flags | ... ) & health_component) == 0) {
        msg.onboard_control_sensors_health |= mav_sensor;
    }

and ``manualControlCheck.cpp`` raises exactly such an error for an engaged kill
switch, an engaged RTL switch, or landing gear up. So a switch in the wrong
position clears the RC health bit while the radio is working perfectly, and
every tool then says "RC unavailable" — indistinguishable from a dead link.

That cost this project a 🔴 flight blocker for two days (2026-08-21 → 08-23,
"RC does not re-acquire after a TX power-cycle"). The link had never failed:
the kill switch was up. A TX power-cycle re-reads the physical switch
positions, which is why the symptom appeared exactly then.

This tool separates the two by looking at what the health bit does not:

  * ``RC_CHANNELS`` arriving at all, and whether the values MOVE.
  * the RC_RECEIVER present / enabled / health triplet, read raw.
  * whatever the FC is saying about it in STATUSTEXT.

Exit: 0 healthy · 1 frames arrive but PX4 cleared the health bit (look at the
switches, not the radio) · 2 no frames at all (link, wiring, or port) · 3 could
not reach the vehicle.
"""
from __future__ import annotations

import argparse
import sys
import time

RC_RECEIVER_BIT = 1 << 16          # MAV_SYS_STATUS_SENSOR_RC_RECEIVER

# Channel values are 11-bit-ish; anything under this is sensor noise, not a
# stick. 5 us is well below the smallest deliberate move and well above the
# jitter a resting gimbal produces.
_MOVE_EPS_US = 5


class RcObservation:
    """What was actually seen on the wire, with no interpretation applied."""

    def __init__(self) -> None:
        self.frames = 0
        self.rssi: int | None = None
        self.chancount: int | None = None
        self.extremes: dict[int, tuple[int, int]] = {}
        self.present: bool | None = None
        self.enabled: bool | None = None
        self.healthy: bool | None = None
        self.statustexts: list[tuple[int, str]] = []

    def note_channels(self, values: list[int], rssi: int, chancount: int) -> None:
        self.frames += 1
        self.rssi = rssi
        self.chancount = chancount
        for i, v in enumerate(values):
            lo, hi = self.extremes.get(i, (v, v))
            self.extremes[i] = (min(lo, v), max(hi, v))

    @property
    def moved_channels(self) -> list[int]:
        """1-based channels that changed by more than noise while watching."""
        return [i + 1 for i, (lo, hi) in sorted(self.extremes.items())
                if hi - lo > _MOVE_EPS_US]

    def value(self, channel: int) -> int | None:
        """Last-known span midpoint for a 1-based channel, if it was seen."""
        span = self.extremes.get(channel - 1)
        return None if span is None else span[1]


def classify(obs: RcObservation) -> tuple[int, str]:
    """Turn an observation into an exit code and the one-line verdict.

    Kept pure and separate from the MAVLink plumbing so the decision can be
    tested without a vehicle — which is the whole point, since the bug this
    exists for is a decision error, not a transport error.
    """
    if obs.frames == 0:
        return 2, ("NO RC FRAMES reach the FC — this is the link: transmitter "
                   "off, receiver unbound, or the RX->FC wiring/port")
    if obs.healthy is False:
        return 1, ("FRAMES ARE ARRIVING and PX4 still calls RC unhealthy — a "
                   "SWITCH or an arming check cleared the health bit, not the "
                   "radio. Check the kill / RTL / gear switches first")
    if obs.healthy is None:
        return 1, ("frames arrive but no SYS_STATUS was seen — cannot say "
                   "whether PX4 accepts the RC")
    return 0, "RC is healthy and frames are arriving"


def _collect(endpoint: str, seconds: float) -> RcObservation:
    from pymavlink import mavutil  # local: optional at import

    obs = RcObservation()
    link = mavutil.mavlink_connection(endpoint, source_system=203)
    link.mav.heartbeat_send(6, 8, 0, 0, 0)
    deadline = time.time() + seconds
    while time.time() < deadline:
        try:
            # Guarded: pymavlink 2.4.49 raises TypeError from its instanced-
            # message bookkeeping on some PX4 1.17 messages and drops whatever
            # arrived with it. Never let that end the watch.
            msg = link.recv_match(blocking=True, timeout=2)
        except Exception:                            # noqa: BLE001
            continue
        if msg is None:
            link.mav.heartbeat_send(6, 8, 0, 0, 0)
            continue
        kind = msg.get_type()
        if kind == "RC_CHANNELS":
            obs.note_channels(
                [getattr(msg, f"chan{i}_raw") for i in range(1, 17)],
                int(msg.rssi), int(msg.chancount))
        elif kind == "SYS_STATUS":
            obs.present = bool(msg.onboard_control_sensors_present & RC_RECEIVER_BIT)
            obs.enabled = bool(msg.onboard_control_sensors_enabled & RC_RECEIVER_BIT)
            obs.healthy = bool(msg.onboard_control_sensors_health & RC_RECEIVER_BIT)
        elif kind == "STATUSTEXT":
            text = msg.text.strip() if isinstance(msg.text, str) else str(msg.text)
            entry = (int(msg.severity), text)
            if text and not text.startswith("AAVC") and entry not in obs.statustexts:
                obs.statustexts.append(entry)
    return obs


def _report(obs: RcObservation, args: argparse.Namespace) -> None:
    print(f"[rc-check] {obs.frames} RC_CHANNELS frame(s) in {args.seconds:g}s"
          + (f", rssi={obs.rssi}, {obs.chancount} channels" if obs.frames else ""))
    if obs.present is None:
        print("[rc-check] no SYS_STATUS seen — is this endpoint carrying the FC?")
    else:
        print(f"[rc-check] RC_RECEIVER present={obs.present} "
              f"enabled={obs.enabled} healthy={obs.healthy}")
    if obs.frames:
        moved = obs.moved_channels
        print(f"[rc-check] channels that MOVED while watching: "
              f"{moved if moved else 'none (sticks untouched, or frozen)'}")
        for label, ch in (("kill", args.kill_ch), ("arm", args.arm_ch)):
            if ch:
                v = obs.value(ch)
                print(f"[rc-check] {label} switch on ch{ch}: "
                      f"{v if v is not None else 'not in frame'}")
    for severity, text in obs.statustexts[:8]:
        print(f"[rc-check] FC says [sev {severity}]: {text}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--endpoint", default="udpout:127.0.0.1:14550",
                    help="MAVLink endpoint to listen on (default: the router's "
                         "QGC port on the CM4)")
    ap.add_argument("--seconds", type=float, default=12.0,
                    help="how long to watch (default 12)")
    ap.add_argument("--kill-ch", type=int, default=8,
                    help="channel RC_MAP_KILL_SW points at (default 8)")
    ap.add_argument("--arm-ch", type=int, default=7,
                    help="channel RC_MAP_ARM_SW points at (default 7)")
    args = ap.parse_args()

    try:
        obs = _collect(args.endpoint, args.seconds)
    except Exception as exc:                         # noqa: BLE001
        print(f"[rc-check] cannot reach the vehicle on {args.endpoint}: {exc}")
        return 3

    _report(obs, args)
    code, verdict = classify(obs)
    print(f"[rc-check] {'OK' if code == 0 else 'PROBLEM'}: {verdict}")
    if code == 1 and obs.frames:
        print("[rc-check] the radio is NOT the thing to debug — PX4 is "
              "refusing an RC it can hear. `Preflight Fail: Kill switch "
              "engaged` is the message to look for above.")
    return code


if __name__ == "__main__":
    sys.exit(main())
