"""Layer 3 — pymavlink/MAVSDK adapter.

Translates high-level commands (goto, drop, hover, takeoff, land) to MAVLink.
Subscribes telemetry and publishes to orchestrator.
"""
