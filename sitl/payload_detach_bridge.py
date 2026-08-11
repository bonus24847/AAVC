#!/usr/bin/python3
"""SITL payload-detach bridge — sheds the matching Gazebo cargo box on each
egg release. Two trigger sources, usable separately or together:

AUDIT MODE (positional `audit` arg — the original mode): tails a mission
run's audit trail. The flight core writes one line per release to
runs/<mission_id>/audit.jsonl (orchestrator/tactical_align.py):

    t=<t>s DELIVERY <k> RELEASE pad=<marker_id or None> payload=<payload_id>
    lat=<lat> lon=<lon>

This bridge tails audit.jsonl FROM EOF (a re-used/appended audit file must
not replay stale releases — this repo's audit file genuinely appends across
runs; see AuditLog.record in orchestrator/audit.py) and, on each NEW RELEASE
line, publishes Empty on `/model/<model>/detach_payload_<payload_id>` exactly
once per payload index. The flight core needs no change — it never imports or
knows about this file.

SERVO MODE (`--servo`, KMUTNB 2026-08-11): watches the REAL command path.
The airframe (sitl/px4_patches/22000_gz_eft_x6100) now maps the four
payload-latch channels into gz — SIM_GZ_SV_FUNC1..4 = output function
"Peripheral via Actuator Set 1..4" (301..304), driven by
MAV_CMD_DO_SET_ACTUATOR from the orchestrator's drop_payload AND from the
AAVC GCS "ปล่อย servo" buttons. PX4's GZMixingInterfaceServo publishes each
channel as gz.msgs.Double (servo ANGLE, radians) on
`/model/<model>/servo_<n>`; hold = -0.63 rad (output 100), release = +0.63
rad (output 900) with the airframe's default ±45° angle map. This mode
subscribes to servo_0..3 and sheds payload n the first time its angle
crosses +0.5 rad — so a GCS button press physically drops the box, same
wire semantics as the real aircraft's latch servo. (The old header note
that "no servo output is published into gz" described the pre-KMUTNB
airframe and is obsolete.)

Both modes de-dupe per payload index against the SAME fired-set, so audit +
servo running together can never shed one box twice.

Task 10 wired four `cargo_payload_0..3` models into the `eft_x6100` aircraft
via gz DetachableJoint plugins (sitl/models/eft_x6100/model.sdf): each box
rides as dead weight through its joint while attached (Tier 1 — loads the
flight dynamics for free, no extra code) and drops free the instant an Empty
message is published on its own `/model/<model>/detach_payload_<N>` topic
(Tier 2).

INVOKE WITH /usr/bin/python3 (gz-transport is the apt-installed Debian
package python3-gz-transport13 at /usr/lib/python3/dist-packages/gz/, NOT
visible from the project's pip .venv) — same convention as
sitl/gz_camera_bridge.py; see that file's header for the house style this
one follows.

Degrades to a clean no-op (exit 0, no traceback) if gz-transport isn't
importable: Tier 1 (the belly mass) is real gz physics that needs no Python
at all, so a missing/broken bridge only costs the visible drop + mass-shed,
never the mission itself.
"""

from __future__ import annotations

import argparse
import re
import sys
import threading
import time
from collections.abc import Container, Iterable, Iterator
from pathlib import Path

# The discriminator is the literal RELEASE keyword between the delivery index
# and pad=, NOT payload='s presence: DELIVERY ... START lines (mission.py)
# carry payload= too. pad= is intentionally NOT constrained to digits — it
# prints `marker_id: int | None` (tactical_align.py's AlignParams), which is
# literally the text "None" at an id-unverified touchdown release (audited,
# not skipped) — see the RELEASE emitter at tactical_align.py:557-560.
_RELEASE = re.compile(
    r"DELIVERY (?P<k>\d+) RELEASE pad=\S+ payload=(?P<payload>\d+)")

_N_PAYLOADS = 4  # cargo_payload_0..3 (Task 10)

# Servo-mode release threshold (rad). With the airframe's default ±45° angle
# map over outputs 0..1000: hold (-0.8 norm -> output 100) reads -0.63 rad,
# release (+0.8 norm -> output 900) reads +0.63 rad — 0.5 sits comfortably
# between with margin for the angle-map defaults changing slightly.
_RELEASE_ANGLE_RAD = 0.5


def parse_release(line: str) -> tuple[int, int] | None:
    """``(delivery_k, payload_id)`` for a RELEASE line, else ``None``.

    Pure — imports nothing gz-related, so this is unit-testable without a
    simulator. Matches by substring search (``re.search``, not ``match``),
    so it works identically whether ``line`` is the bare f-string
    tactical_align.py emits or a whole on-disk audit.jsonl row: AuditLog.
    record (orchestrator/audit.py) wraps every entry as
    ``{"ts": ..., "entry": "<line>"}`` before appending, and the entry text
    here needs no JSON escaping, so the pattern still appears verbatim
    inside the JSON-quoted string either way.
    """
    m = _RELEASE.search(line)
    return (int(m.group("k")), int(m.group("payload"))) if m else None


def _iter_new_releases(
    audit: Path, poll_s: float, *, max_idle_polls: int | None = None,
) -> Iterator[tuple[int, int]]:
    """Tail ``audit`` from EOF, yielding ``(delivery_index, payload_id)`` for
    every NEW ``DELIVERY ... RELEASE`` line appended after this generator
    starts reading. Pure — no gz import, no publishing — so the two
    guarantees that matter most here are unit-testable without a simulator:

      * EOF seek: this repo's audit.jsonl genuinely appends across runs (a
        re-used mission_id keeps writing to the same file — see
        ``AuditLog.record``, ``orchestrator/audit.py``), so starting at
        offset 0 would replay every past mission's releases against
        whatever boxes happen to be attached to the aircraft right now.
        Lines already in the file when this generator starts must never be
        yielded.
      * live tail: a line appended AFTER this generator starts IS yielded.

    ``max_idle_polls`` bounds how many consecutive empty polls (waiting for
    the file to appear, or for a new line) this sits through before giving
    up and returning. Production (``_run``) leaves it at ``None`` and tails
    forever, exactly as this loop always has. Tests pass a small integer
    instead: a `for`/`list()` over the generator then terminates on its own
    once nothing more is forthcoming, rather than requiring the test to
    count exactly how many items are safe to pull with bare ``next()``
    calls — get that count wrong against an unbounded generator and the
    test would hang the whole suite, with no timeout. This makes that
    failure mode structurally impossible, in production as well as tests.
    """
    idle = 0
    print(f"[detach] waiting for {audit} to appear …")
    while not audit.exists():
        time.sleep(poll_s)
        idle += 1
        if max_idle_polls is not None and idle >= max_idle_polls:
            return

    with audit.open() as fh:
        # Seek to EOF *before* reading a single line — see the EOF-seek
        # guarantee above.
        fh.seek(0, 2)
        print(f"[detach] tailing {audit} from EOF")
        idle = 0
        while True:
            line = fh.readline()
            if not line:
                time.sleep(poll_s)
                idle += 1
                if max_idle_polls is not None and idle >= max_idle_polls:
                    return
                continue
            idle = 0
            release = parse_release(line)
            if release is not None:
                yield release


def _dedupe(
    releases: Iterable[tuple[int, int]],
    known_payloads: Container[int],
    fired: set[int],
) -> Iterator[tuple[int, int]]:
    """Filter ``releases`` down to the FIRST sighting of each payload index
    that has a matching publisher — the bridge's idempotence guard, so a
    duplicated or replayed RELEASE line can never shed the same cargo box
    twice. Pure — no gz import — so this guarantee is unit-testable without
    a simulator; ``_run`` supplies the real gz publishers (``pubs``) as
    ``known_payloads`` and starts ``fired`` empty. ``fired`` is mutated in
    place (mirroring ``_run``'s own bookkeeping) so a caller can inspect it
    once iteration ends.
    """
    for delivery_k, payload in releases:
        if payload in fired:
            continue
        if payload not in known_payloads:
            print(f"[detach] DELIVERY {delivery_k} RELEASE payload={payload} "
                  "has no matching cargo_payload model — ignoring")
            continue
        fired.add(payload)
        yield delivery_k, payload


def _run(audit: Path | None, model: str, poll_s: float, *,
         servo: bool = False) -> int:
    try:
        from gz.msgs10.empty_pb2 import Empty
        from gz.transport13 import Node
    except Exception as e:  # gz-transport not installed/importable here
        print(f"[detach] gz-transport unavailable ({e}) — bridge disabled; "
              "Tier-1 belly mass is unaffected, only the visible drop is skipped")
        return 0

    node = Node()
    pubs = {i: node.advertise(f"/model/{model}/detach_payload_{i}", Empty)
            for i in range(_N_PAYLOADS)}
    fired: set[int] = set()
    fired_lock = threading.Lock()

    def _shed(payload: int, why: str) -> None:
        """Idempotent shed shared by both trigger sources (thread-safe: servo
        callbacks fire on gz-transport's own threads)."""
        with fired_lock:
            if payload in fired:
                return
            fired.add(payload)
            n_shed = len(fired)
        pubs[payload].publish(Empty())
        print(f"[detach] {why}: shed payload {payload} "
              f"(detach_payload_{payload}, {n_shed}/{_N_PAYLOADS} shed)")

    if servo:
        from gz.msgs10.double_pb2 import Double

        def _make_servo_cb(i: int):
            def cb(msg: "Double") -> None:
                if msg.data >= _RELEASE_ANGLE_RAD:
                    _shed(i, f"SERVO {i + 1} angle {msg.data:+.2f} rad")
            return cb

        for i in range(_N_PAYLOADS):
            topic = f"/model/{model}/servo_{i}"
            if not node.subscribe(Double, topic, _make_servo_cb(i)):
                print(f"[detach] WARNING: subscribe failed for {topic}")
            else:
                print(f"[detach] watching {topic} "
                      f"(release at >= {_RELEASE_ANGLE_RAD} rad)")

    if audit is not None:
        print(f"[detach] bridging {audit} → model={model} "
              f"(payloads 0..{_N_PAYLOADS - 1})")
        # _dedupe gets its own bookkeeping set: the CROSS-SOURCE guard (audit
        # + servo racing on the same release event) is _shed's lock-guarded
        # `fired`, the single publish authority for both trigger paths.
        for delivery_k, payload in _dedupe(
                _iter_new_releases(audit, poll_s), pubs, set()):
            _shed(payload, f"DELIVERY {delivery_k}")
    elif servo:
        # Servo-only mode: callbacks do all the work; park the main thread.
        while True:
            time.sleep(1.0)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description="SITL cargo-detach bridge: audit-tail and/or gz servo "
                    "watch -> gz Empty on release")
    ap.add_argument("audit", type=Path, nargs="?", default=None,
                     help="runs/<mission_id>/audit.jsonl to tail (from EOF); "
                          "optional when --servo is given")
    ap.add_argument("--servo", action="store_true",
                     help="also watch /model/<model>/servo_0..3 (gz Double, "
                          "rad) — the DO_SET_ACTUATOR path used by the "
                          "orchestrator drop AND the AAVC GCS buttons")
    ap.add_argument("--model", default="eft_x6100",
                     help="gz model name carrying the payloads (default: eft_x6100)")
    ap.add_argument("--poll-s", type=float, default=0.2,
                     help="audit-file poll interval in seconds (default: 0.2)")
    args = ap.parse_args()
    if args.audit is None and not args.servo:
        ap.error("nothing to do: pass an audit file, --servo, or both")
    try:
        return _run(args.audit, args.model, args.poll_s, servo=args.servo)
    except KeyboardInterrupt:
        print("[detach] stopped")
        return 0


if __name__ == "__main__":
    sys.exit(main())
