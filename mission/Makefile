.PHONY: help install lock sitl stack stack-stop spawn-targets camera-bridge camera-view payload-bridge payload-bridge-servo aavc-gcs camera-real run hitl hitl-params hitl-camera run-hitl web-build test lint clean preflight param-audit type-audit fence-probe alt-watch field-check verify replay

# NOTE: invoke tools via `python -m <module>` (location-independent — survives a
# directory move, unlike the venv's pinned shebangs) and clear PYTHONPATH so a
# sourced ROS environment cannot leak its pytest/launch plugins into the venv.
PY := env -u PYTHONPATH .venv/bin/python

# KMUTNB sky-field repo: default every run to the 5 m-ceiling practice profile
# (mission_brain/profile.py kmutnb_skyfield; orchestrator --profile overrides).
export AAVC_PROFILE ?= kmutnb_skyfield

help:
	@echo "AAVC — KMUTNB sky-field practice map — common tasks"
	@echo ""
	@echo "  make install        Create .venv + install -e .[dev]"
	@echo "  make sitl           Launch PX4 SITL + Gazebo with the KMUTNB sky-field"
	@echo "  make stack          DEBUG harness: SITL + bridges + GCS console + pads (GUI=1, SEED=n)"
	@echo "  make stack-stop     Tear the whole stack down (self-match-safe pkills)"
	@echo "  make spawn-targets  Spawn the 6 ArUco landing pads (SEED=n re-rolls ids+positions)"
	@echo "  make camera-bridge  gz camera -> /tmp/aavc_nadir.png (+ frame mirror)"
	@echo "  make payload-bridge SITL: shed cargo boxes on release, optional (RUN=runs/<id>/audit.jsonl)"
	@echo "  make payload-bridge-servo  SITL: shed cargo on gz servo release (GCS/DO_SET_ACTUATOR path)"
	@echo "  make aavc-gcs       AAVC GCS console (telemetry map + manual servo release)"
	@echo "  make camera-real    REAL CM4 cameras -> /tmp/aavc_*.png (BACKEND=v4l2|picamera2)"
	@echo "  make run            Run the orchestrator: blind search-and-serve (TRUTH=path to audit)"
	@echo "  make hitl           HITL: jMAVSim against a real Pixhawk 6X (see docs/HITL.md)"
	@echo "  make hitl-params    HITL: one-time FC config via nsh (SYS_HITL, airframe, RC)"
	@echo "  make hitl-camera    HITL: synthetic nadir frames from FC telemetry"
	@echo "  make run-hitl       HITL: run the mission vs the real 6X (CONNECT=endpoint, TRUTH=path)"
	@echo "  make web-build      Build the Svelte dashboard"
	@echo "  make test           Run pytest"
	@echo "  make lint           Run ruff + mypy"
	@echo "  make lock           Refresh requirements.lock (pinned deps; reproducible offline build)"
	@echo "  make clean          Remove caches + build artifacts"
	@echo ""
	@echo "  Field checks (skill PX4MASTER — .claude/skills/PX4MASTER/):"
	@echo "  make preflight      BOARD/PINNED param check (CONNECT=…, STRICT=1 after staging)"
	@echo "  make fence-probe    FC mission/dataman aliveness + SD flags (the 2026-08-20 wedge)"
	@echo "  make alt-watch      GPS/altitude stability gate — after every FC reboot"
	@echo "  make field-check    preflight -> fence-probe -> alt-watch (stops at first failure)"
	@echo "  make param-audit    Bench sweep: is every px4_tuning value holding on the board?"
	@echo "  make type-audit     Static: every pushed param uses the setter PX4 accepts"
	@echo "  make verify         Post-flight verifier (RUN=runs/<id>/audit.jsonl [TRUTH=…])"
	@echo "  make replay         ArUco decode over recorded frames (DIR=… [ARGS=…])"

# For a byte-reproducible competition build (the AAVC site bans internet, so the
# venv is built once beforehand), pin every transitive dep to requirements.lock:
#   .venv/bin/pip install -e ".[dev]" -c requirements.lock
# `make lock` regenerates that file from the current venv.
install:
	python3.12 -m venv .venv
	.venv/bin/pip install --upgrade pip
	.venv/bin/pip install -e ".[dev]"

lock:
	.venv/bin/pip freeze --exclude-editable > requirements.lock
	@echo "[lock] wrote requirements.lock (pinned deps for reproducible installs)"

# GUI=1 shows the gz viewer (default is headless to cut dGPU render load);
# SKIP_GPU_CHECK=1 bypasses the dGPU-health preflight (non-NVIDIA / other render path).
sitl:
	bash sitl/launch_sitl.sh

# One-command debug bring-up (everything except the mission): SITL (+GUI=1
# viewer), camera + payload bridges (live logs), AAVC GCS console on this
# repo's field/captures, seeded pads. Then `make run TRUTH=…` flies on top.
stack:
	bash sitl/launch_stack.sh

stack-stop:
	bash sitl/launch_stack.sh stop

# SEED=<n> re-jitters the target layout so blind-search runs can't memorise it,
# e.g. `make spawn-targets SEED=123`.
spawn-targets:
	$(PY) sitl/spawn_targets.py --config sitl/aavc_config.yaml $(if $(SEED),--seed $(SEED))

# Bridges need BOTH worlds: gz-transport13 lives in the apt dist-packages
# (invisible to the pip venv) while cv2 lives in the venv (this host's system
# python3 has no python3-opencv). Run them on the venv python with the system
# dist-packages APPENDED via PYTHONPATH — venv packages still win on conflict,
# and the scope is just these bridge targets (make test strips PYTHONPATH).
BRIDGE_PY := env PYTHONPATH=/usr/lib/python3/dist-packages .venv/bin/python
camera-bridge:
	$(BRIDGE_PY) sitl/gz_camera_bridge.py

# Live nadir-camera window for SITL test sessions (auto-reloading viewer on
# /tmp/aavc_nadir.png) — the operator watches what the detector actually sees.
camera-view:
	bash sitl/camera_view.sh

# OPTIONAL: tails a mission run's audit.jsonl (SYSTEM python3, same reason as
# camera-bridge above) and publishes gz Empty on /model/<model>/detach_payload_N
# for each DELIVERY RELEASE, shedding that cargo box (Task 10) onto the pad.
# Tier-1 belly mass loads the flight dynamics without this running at all; the
# bridge only adds the visible drop + mass-shed. RUN=runs/<id>/audit.jsonl
# (required — the mission's own run directory), e.g.
#   make payload-bridge RUN=runs/aavc_delivery_mission/audit.jsonl
# CHANNELS mirrors connection.drop_servo_channels in sitl/aavc_config.yaml (the
# as-wired payload_id -> AUX map) so the audit path sheds the box the real latch
# would open; tests/test_payload_detach_bridge.py fails if the two drift apart.
CHANNELS ?= 4,1,2,3
payload-bridge:
	$(BRIDGE_PY) sitl/payload_detach_bridge.py $(RUN) --model eft_x6100 \
		--channels $(CHANNELS)

# Servo-path variant (KMUTNB): watches /model/eft_x6100[_0]/servo_0..3 — the
# gz side of MAV_CMD_DO_SET_ACTUATOR (orchestrator drop_payload AND the AAVC
# GCS "ปล่อย servo" buttons) — and sheds the matching cargo box. Combine with
# the audit tail in one process: RUN=runs/<id>/audit.jsonl make payload-bridge-servo
payload-bridge-servo:
	$(BRIDGE_PY) sitl/payload_detach_bridge.py $(RUN) --servo --model eft_x6100 \
		--channels $(CHANNELS)

# AAVC GCS console (../gcs) against this repo's field file: live
# telemetry map (leaflet) + geofence/search/transit overlay + the manual
# payload-release buttons (DO_SET_ACTUATOR 1..4). Binds udp 14550 = PX4's GCS
# broadcast — which is why aavc_config.yaml sets raw_telemetry_port: 0 (the
# port has ONE listener; the orchestrator's optional raw widgets cede it).
# GCS_URL overrides the endpoint, GCS_ARGS passes extras (--port, --baud, …).
# --captures pins the console to THIS repo's captures/ so its map pads come
# from OUR orchestrator's live mission_status.json (orchestrator/gcs_status.py
# — pads appear as the drone scans them). Without it the console auto-shares
# a SIBLING project's captures dir and renders that project's stale pads.
# gcs/ sits beside mission/ in this repo (was a separate ~/Desktop checkout
# until 2026-09-02, which only ever resolved on one laptop).
AAVC_GCS ?= $(CURDIR)/../gcs/src/aavc_gcs.py
aavc-gcs:
	/usr/bin/python3 $(AAVC_GCS) --field gcs/kmutnb_field.yaml --captures captures --url $(if $(GCS_URL),$(GCS_URL),udpin:0.0.0.0:14550) --mission-cmd "'$(CURDIR)/sitl/run_mission.sh' {ids}" --reset-cmd "env KEEP_CONSOLE=1 GUI=$(if $(GUI),$(GUI),0) bash '$(CURDIR)/sitl/launch_stack.sh'" $(GCS_ARGS)

# REAL cameras on the CM4 (G5+) -> the SAME /tmp/aavc_*.png frames as the gz
# bridge. BACKEND=v4l2 (USB/UVC, runs in .venv) | picamera2 (CSI/libcamera, needs
# SYSTEM python3 — picamera2 is apt-level). GRAB_ARGS passes extra flags, e.g.
#   make camera-real BACKEND=v4l2 GRAB_ARGS="--nadir-device 0 --fourcc GREY --fps 50"
camera-real:
	$(if $(filter picamera2,$(BACKEND)),/usr/bin/python3,$(PY)) sitl/camera_grabber.py --backend $(if $(BACKEND),$(BACKEND),v4l2) $(GRAB_ARGS)

# TRUTH=<path> enables the post-flight discovered-vs-truth audit (SITL only),
# e.g. `make run TRUTH=/tmp/aavc_targets.json`. Never used for planning.
run:
	$(PY) -m orchestrator.main --config sitl/aavc_config.yaml $(if $(TRUTH),--truth-json $(TRUTH))

# ── HITL (real Pixhawk 6X + CM4 + ELRS RC) — full runbook in docs/HITL.md ─────
# PX4 HITL can't use gz (only jMAVSim / Gazebo Classic), so there are no sim
# cameras — `hitl-camera` feeds the vision pipeline a position-driven stand-in.
# HITL_SERIAL=<dev> overrides the 6X serial port (default /dev/ttyACM0).
hitl:
	bash sitl/launch_hitl.sh

# One-time FC config for HITL via the nsh shell (NOT PARAM_SET — byte-wise gotcha):
# airframe 1001 + SYS_HITL=1 + the RC/failsafe block. SERIAL=/BAUD= override the link;
# CONNECT=<endpoint> to go through the router instead. --dry-run prints the plan.
hitl-params:
	$(PY) sitl/hitl_param_config.py $(if $(SERIAL),--serial $(SERIAL)) $(if $(BAUD),--baud $(BAUD)) $(if $(CONNECT),--connect $(CONNECT))

# MAVLINK=<endpoint> overrides the telemetry feed; TARGETS=<json> the target set.
hitl-camera:
	$(PY) sitl/hitl_synthetic_camera.py --mavlink $(if $(MAVLINK),$(MAVLINK),udpin:0.0.0.0:14541) $(if $(TARGETS),--targets $(TARGETS))

# The real mission vs the real 6X. CONNECT=<endpoint> overrides the offboard link
# (default udpin://0.0.0.0:14540, fed by mavlink-router). TRUTH=path → audit.
run-hitl:
	$(PY) -m orchestrator.main --config sitl/aavc_config.yaml --connect $(if $(CONNECT),$(CONNECT),udpin://0.0.0.0:14540) $(if $(TRUTH),--truth-json $(TRUTH))

web-build:
	cd dashboard/web && npm i && npm run build

test:
	$(PY) -m pytest

lint:
	$(PY) -m ruff check .
	$(PY) -m mypy mission_brain orchestrator mavlink_adapter vision dashboard

clean:
	rm -rf __pycache__ .pytest_cache .mypy_cache .ruff_cache *.egg-info
	find . -name "__pycache__" -type d -not -path "./.venv/*" -exec rm -rf {} +
	find . -name "*.pyc" -not -path "./.venv/*" -delete

# ── Field checks (skill PX4MASTER — .claude/skills/PX4MASTER/) ──────────────
# CONNECT defaults to the router's offboard port; pass
# CONNECT=serial:///dev/ttyAMA0:921600 on the CM4 when the router is down.
CONNECT ?= udpin://0.0.0.0:14540

preflight:
	$(PY) tools/preflight_params.py --connect "$(CONNECT)" $(if $(STRICT),--strict)

param-audit:
	$(PY) tools/param_audit.py --connect "$(CONNECT)"

type-audit:
	$(PY) tools/px4_type_audit.py

fence-probe:
	$(PY) tools/fence_probe.py --connect "$(CONNECT)"

alt-watch:
	$(PY) tools/alt_watch.py --connect "$(CONNECT)"

lidar-check:
	$(PY) tools/lidar_check.py --connect "$(CONNECT)"

# The laptop-side pre-staging aggregate: stops at the first failure, in the
# order the field-day procedure runs them. ⚠ run BEFORE pressing GO — every
# probe binds the orchestrator's udpin port and exits when done.
# lidar-check runs LAST because it is the only one that proves hardware
# rather than stored values: preflight ticks SENS_TFMINI_CFG green off a
# param read, which stayed green through 5 of 11 flights flown with the
# rangefinder dead (2026-08-25).
field-check: preflight fence-probe alt-watch lidar-check
	@echo "✔ field-check: BOARD params + mission path + altitude frame + rangefinder all good — stage away"

# Post-flight verifier: make verify RUN=runs/<id>/audit.jsonl [TRUTH=/tmp/aavc_targets.json]
verify:
	$(PY) tools/verify_flight.py "$(RUN)" $(if $(TRUTH),--truth "$(TRUTH)") --config sitl/aavc_config.yaml

# Offline ArUco decode over recorded frames: make replay DIR=runs/<id>/frames [ARGS="--annotate /tmp/annot"]
replay:
	$(PY) tools/replay_frames.py "$(DIR)" $(ARGS)
