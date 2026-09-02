"""AAVC orchestrator — the mission state machine + flight execution.

Lightweight competition build. Layers:
  state.py           shared mutable state (phase, telemetry, drop tracking)
  safety.py          background watchdog (battery / GPS / geofence / time)
  vision_worker.py   dual-camera landing-pad detection feed
  tactical_align.py  visual-servo align + descend-gate + land-ON + touchdown release
  frame_recorder.py  low-rate JPEG mission recorder (runs/<id>/frames/)
  mission.py         the multi-sortie land-ON-and-release delivery loop
  main.py            entry point + wiring
"""
