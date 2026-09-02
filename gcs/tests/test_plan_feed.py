"""The map's route, on a console whose only live link is a radio.

Review 3.9 (2026-08-22): the mission plan polyline shipped 2026-08-21 could
never draw on a REAL flight. The route is written on the CM4 and a STATUSTEXT
packet is 50 characters, so nothing carried it to the laptop — the feature
worked in SITL and was invisible exactly where the operator needed it (the G7
takeover came early because the screen could not answer "where is it going
next?").

The fix pulls the plan over ssh while the aircraft is still at L&R in WiFi
range, keeps it when the link goes, and marks it stale rather than pretending
it is live. These tests pin the three ways that can go wrong: a bad read
wiping a good route, a stale route posing as live, and the throttle failing so
the console ssh-storms the CM4.
"""
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import aavc_gcs  # noqa: E402

MISSION_CMD = ("ssh -o StrictHostKeyChecking=accept-new -i /home/op/.ssh/cm4_key "
               "drone@10.42.0.1 '~/mission/sitl/run_mission.sh {ids}'")

_BODY = json.dumps({
    "phase": "search", "updated": 1.0, "run": "20260822_101500",
    "plan_ptr": 3,
    "plan": [[13.82, 100.51, "transit_ingress", 1],
             [13.83, 100.52, "search", 2],
             ["bad"],                       # short row — must be dropped
             [13.84, 100.53, "land", 3]],
})


def _wait_idle(timeout=5.0):
    end = time.time() + timeout
    while aavc_gcs._PLAN_INFLIGHT and time.time() < end:
        time.sleep(0.01)
    # the pull thread writes _PLAN then clears the flag; give it the handoff
    for _ in range(200):
        if not aavc_gcs._PLAN_INFLIGHT:
            return
        time.sleep(0.01)


def _reset_plan_state():
    aavc_gcs._PLAN = {"plan": [], "ptr": 0, "run": None, "t": 0.0}
    aavc_gcs._PLAN_PULLED_AT = 0.0
    aavc_gcs._PLAN_INFLIGHT = False


# ── parsing: a bad read must never clear a good route ──────────────────────

def test_parses_only_the_plan_fields_and_drops_short_rows():
    got = aavc_gcs._plan_from_status(_BODY)
    assert [r[3] for r in got["plan"]] == [1, 2, 3]
    assert got["ptr"] == 3 and got["run"] == "20260822_101500"
    assert "phase" not in got      # the radio owns phase — never the WiFi copy


def test_unparseable_or_planless_bodies_return_none():
    # ssh noise, a truncated read, a CM4 whose file predates the plan field:
    # each must be a "keep what you have", not a "wipe the map".
    for body in ("", "ssh: connect to host 10.42.0.1 port 22: No route to host",
                 '{"phase": "search", "updated": 1.0}', "[1,2,3]",
                 '{"plan": [[13.8, 100.5, "search", 1]]'):
        assert aavc_gcs._plan_from_status(body) is None


# ── which feed the map draws, and how old it says the route is ─────────────

def test_wifi_mission_wins_including_when_its_plan_is_empty():
    cache = {"plan": [[1, 2, "search", 1]], "ptr": 1, "run": "old", "t": 100.0}
    live = {"plan": [], "plan_ptr": 0, "run": "new", "age_s": 1.0}
    snap = aavc_gcs._plan_snapshot(live, cache, 200.0)
    # An empty plan on the live feed is a real statement ("nothing to fly") and
    # must not fall through to a cached route from a finished mission.
    assert snap["plan"] == [] and snap["run"] == "new"


def test_radio_mission_has_no_plan_key_so_the_ssh_pull_supplies_it():
    radio = {"phase": "search", "src": "radio", "age_s": 2.0}   # no "plan" key
    cache = {"plan": [[1, 2, "search", 1]], "ptr": 1, "run": "r1", "t": 100.0}
    snap = aavc_gcs._plan_snapshot(radio, cache, 130.0)
    assert snap["plan"] == cache["plan"] and snap["run"] == "r1"
    assert snap["age"] == 30.0      # age of the PULL, not of the beacon packet


def test_no_feed_at_all_draws_nothing():
    empty = {"plan": [], "ptr": 0, "run": None, "t": 0.0}
    assert aavc_gcs._plan_snapshot(None, empty, 10.0) is None


# ── the pull itself: right credentials, and throttled ──────────────────────

def test_pull_uses_the_go_commands_own_credentials_and_updates_once():
    _reset_plan_state()
    old_cmd, aavc_gcs.MISSION_CMD = aavc_gcs.MISSION_CMD, MISSION_CMD
    runs = []
    lock = threading.Lock()

    class _R:
        returncode = 0
        stdout = _BODY
        stderr = ""

    def fake_run(argv, **kw):
        with lock:
            runs.append(argv)
        return _R()

    old_run, aavc_gcs.subprocess.run = aavc_gcs.subprocess.run, fake_run
    try:
        aavc_gcs._maybe_pull_plan("10.42.0.1")
        _wait_idle()
        assert len(runs) == 1
        argv = runs[0]
        # the CM4 key is NOT id_rsa and the local username is not drone@ — the
        # auto-infra ssh lost a week to exactly that (commit efbab6e)
        assert "-i" in argv and "/home/op/.ssh/cm4_key" in argv
        assert "drone@10.42.0.1" in argv
        assert argv[-1] == "cat ~/mission/captures/mission_status.json"
        assert aavc_gcs._PLAN["run"] == "20260822_101500"

        # throttled: the probe loop ticks every 4 s, the pull may not
        aavc_gcs._maybe_pull_plan("10.42.0.1")
        _wait_idle()
        assert len(runs) == 1
    finally:
        aavc_gcs.subprocess.run = old_run
        aavc_gcs.MISSION_CMD = old_cmd
        _reset_plan_state()


def test_a_failed_pull_keeps_the_last_good_plan():
    _reset_plan_state()
    old_cmd, aavc_gcs.MISSION_CMD = aavc_gcs.MISSION_CMD, MISSION_CMD
    good = {"plan": [[1, 2, "search", 1]], "ptr": 1, "run": "r1", "t": 100.0}
    aavc_gcs._PLAN = dict(good)

    class _R:
        returncode = 255
        stdout = ""
        stderr = "ssh: connect to host: No route to host"

    old_run, aavc_gcs.subprocess.run = aavc_gcs.subprocess.run, (
        lambda argv, **kw: _R())
    try:
        aavc_gcs._maybe_pull_plan("10.42.0.1")
        _wait_idle()
        assert aavc_gcs._PLAN == good      # out of range != mission cancelled
    finally:
        aavc_gcs.subprocess.run = old_run
        aavc_gcs.MISSION_CMD = old_cmd
        _reset_plan_state()


def test_no_ssh_target_means_no_pull():
    """A SITL console runs the mission locally: nothing to ssh to, and its
    plan already arrives through the local status file."""
    _reset_plan_state()
    old_cmd, aavc_gcs.MISSION_CMD = aavc_gcs.MISSION_CMD, "bash sitl/run_mission.sh {ids}"
    called = []
    old_run, aavc_gcs.subprocess.run = aavc_gcs.subprocess.run, (
        lambda argv, **kw: called.append(argv))
    try:
        aavc_gcs._maybe_pull_plan(None)
        _wait_idle()
        assert called == []
    finally:
        aavc_gcs.subprocess.run = old_run
        aavc_gcs.MISSION_CMD = old_cmd
        _reset_plan_state()
