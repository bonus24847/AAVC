"""Regression test for the payload-latch keep-alive (ACTUATOR_TEST) lifecycle.

Bug (2026-08-19, reported from the field): pressing the payload open/close
buttons rapidly leaves latches opening/closing BY THEMSELVES after 5-10 min,
and only killing the whole GCS process stops it.

Root cause: the disarmed bench hold keeps a latch open by re-sending
ACTUATOR_TEST (a watchdogged override) from a per-call background thread. Rapid
clicks spawn overlapping holds, and the pop/replace of the per-channel stop
Event in ``_servo_test_hold`` races with thread start, so a concurrent hold for
the same channel overwrites another's stop Event WITHOUT setting it -> that
keep-alive loop is orphaned and can never be stopped except by process exit.

Invariant this pins: after ``_servo_test_stop(idx)`` settles, NO keep-alive may
still be sending ACTUATOR_TEST(hold) frames for that channel.
"""
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import aavc_gcs  # noqa: E402

Link = aavc_gcs.Link
ACTUATOR_TEST = 310


def _make_link():
    """A Link with no real MAVLink connection; _cmd just records the frames."""
    link = Link.__new__(Link)          # bypass __init__ (no serial/UDP link)
    link.demo = False
    link.lock = threading.Lock()
    link.send_lock = threading.Lock()
    link.s = {"armed": False}
    # attrs used by the current impl AND the single-supervisor fix:
    link._servo_hold = {}
    link._servo_desired = {}
    link._servo_lock = threading.Lock()
    link._servo_sup_started = False
    calls = []
    lock = threading.Lock()

    def rec_cmd(cmd, *params):
        with lock:
            calls.append((cmd, tuple(params)))

    link._cmd = rec_cmd
    return link, calls, lock


def _hold_frames(calls, lock):
    # ACTUATOR_TEST hold = param2 (timeout) > 0; release = param2 == 0
    with lock:
        return [c for c in calls if c[0] == ACTUATOR_TEST and c[1][1] > 0]


class _EventGate:
    """Force the pop->store race deterministically instead of hoping the GIL
    schedules it. Each thread's FIRST ``threading.Event()`` (the per-channel stop
    Event created inside ``_servo_test_hold`` *after* it has popped the old one)
    blocks on a barrier, so all N holds pop the empty slot before any stores ->
    N-1 keep-alive loops get orphaned. Later Event()s (thread internals) and the
    single-supervisor fix (which creates no per-call Event) pass straight through,
    so the same test is honest for both implementations."""

    def __init__(self, n):
        self._real = aavc_gcs.threading.Event
        self._n = n
        self._count = 0
        self._clock = threading.Lock()
        self._open = self._real()      # a REAL Event, created before the patch
        self._tls = threading.local()

    def __enter__(self):
        gate = self

        def factory(*a, **k):
            if not getattr(gate._tls, "tripped", False):
                gate._tls.tripped = True          # only the FIRST Event per thread
                with gate._clock:
                    gate._count += 1
                    if gate._count >= gate._n:
                        gate._open.set()          # release once all N have popped
                gate._open.wait(timeout=5)
            return gate._real(*a, **k)

        aavc_gcs.threading.Event = factory
        return self

    def __exit__(self, *exc):
        aavc_gcs.threading.Event = self._real
        return False


def test_stop_terminates_all_keepalive_under_rapid_holds():
    link, calls, lock = _make_link()
    idx = 1
    N = 20

    # Start the supervisor (if the impl has one) BEFORE the burst, so its one-time
    # thread creation doesn't consume a gate slot; a no-op on the current impl.
    if hasattr(link, "_ensure_servo_sup"):
        link._ensure_servo_sup()

    # Reproduce "รัวๆ": N holds for the SAME channel fired simultaneously, with
    # the interleaving forced so the race actually happens.
    start = threading.Barrier(N)

    def worker():
        start.wait()
        link._servo_test_hold(idx, 0.8)

    with _EventGate(N):
        threads = [threading.Thread(target=worker, daemon=True) for _ in range(N)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        time.sleep(0.6)                # let the keep-alive(s) run a few ticks
        link._servo_test_stop(idx)
        time.sleep(0.6)                # let stop settle (period + release sends)

    baseline = len(_hold_frames(calls, lock))
    time.sleep(1.6)                    # > 3 keep-alive periods (0.4s each)
    after = len(_hold_frames(calls, lock))

    extra = after - baseline
    assert extra == 0, (
        f"{extra} ACTUATOR_TEST(hold) frames sent AFTER stop settled — a keep-alive "
        f"thread survived _servo_test_stop(); the latch is driving itself (only a "
        f"process kill would end it)."
    )


def test_stop_releases_control():
    """_servo_test_stop must hand the output back to PX4 (RELEASE_CONTROL, param2=0)."""
    link, calls, lock = _make_link()
    link._servo_test_hold(2, 0.8)
    time.sleep(0.5)
    link._servo_test_stop(2)
    time.sleep(0.6)
    with lock:
        releases = [c for c in calls if c[0] == ACTUATOR_TEST and c[1][1] == 0]
    assert releases, "no RELEASE_CONTROL (ACTUATOR_TEST timeout=0) sent on stop"
