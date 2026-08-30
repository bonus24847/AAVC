"""Every FC-state write on the page must be locked by what the AIRCRAFT says,
not by the liveness of this console's own ssh child (review 2026-08-30).

`mission_running()` is False whenever that child is gone — a console restarted
mid-flight (it happened at KMITL on 28 Aug 14:46), a mission started on the CM4
by hand, or the GO ssh dying when the aircraft's WiFi drops at takeoff. Until
this change that unlocked the four payload-latch buttons plus "ปล่อยทั้งหมด"
(one confirm from releasing eggs in flight), and the origin / geofence / fence
buttons were never locked at all: `SET_GPS_GLOBAL_ORIGIN` shifts the EKF's local
reference under a running mission, and `fence/clear` removes the only FC-level
fence the aircraft has (the radius fence was retired 2026-08-17).
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import aavc_gcs  # noqa: E402


class _FakeLink:
    demo = False

    def __init__(self, **snap):
        self._snap = {"armed": False, "in_air": False}
        self._snap.update(snap)

    def snapshot(self):
        return dict(self._snap)


def _with_link(link):
    aavc_gcs.LINK = link
    return aavc_gcs._flying()


def test_on_the_ground_and_disarmed_nothing_is_locked() -> None:
    assert _with_link(_FakeLink()) is False


def test_armed_on_the_pad_is_locked_for_fc_writes() -> None:
    # armed but ON THE GROUND: origin/geofence writes are refused…
    assert _with_link(_FakeLink(armed=True)) is True


def test_airborne_is_locked() -> None:
    assert _with_link(_FakeLink(in_air=True)) is True
    assert _with_link(_FakeLink(armed=True, in_air=True)) is True


def test_a_demo_console_is_never_locked() -> None:
    link = _FakeLink(armed=True, in_air=True)
    link.demo = True
    assert _with_link(link) is False


def test_a_broken_snapshot_fails_open_rather_than_locking_the_crew_out() -> None:
    class _Broken(_FakeLink):
        def snapshot(self):
            raise RuntimeError("no link")
    assert _with_link(_Broken()) is False
    aavc_gcs.LINK = None
    assert aavc_gcs._flying() is False


def test_the_manual_servo_route_refuses_while_airborne_but_not_on_the_pad() -> None:
    """The servo guard is `in_air`, NOT `armed`: a manual release while armed on
    the pad is the deliberate fallback ("วางไม่ตรง ดีกว่าไม่วาง")."""
    src = open(os.path.join(os.path.dirname(__file__), "..", "src",
                            "aavc_gcs.py"), encoding="utf-8").read()
    i = src.index('if action.startswith("servo/")')
    guard = src[i:i + 900]
    assert 'LINK.snapshot().get("in_air")' in guard, guard[:400]
    assert "ห้ามสั่ง servo มือ" in guard


def test_origin_geofence_and_fence_clear_are_gated_on_flying() -> None:
    src = open(os.path.join(os.path.dirname(__file__), "..", "src",
                            "aavc_gcs.py"), encoding="utf-8").read()
    for route in ('elif action == "origin/set":',
                  'elif action.startswith("geofence/set"):',
                  'elif action == "fence/clear":'):
        i = src.index(route)
        assert "_flying()" in src[i:i + 500], route


def test_the_console_never_writes_the_radius_fence_again() -> None:
    """GF_MAX_HOR_DIST killed a mission mid-transit on 2026-08-17 and the panel's
    boxes defaulted to 50 m — a 50 m radius around home against a search area
    that runs to ENU E 266 m means GF_ACTION=3 RTLs on the first sweep leg."""
    src = open(os.path.join(os.path.dirname(__file__), "..", "src",
                            "aavc_gcs.py"), encoding="utf-8").read()
    assert 'set_float("GF_MAX_HOR_DIST"' not in src
