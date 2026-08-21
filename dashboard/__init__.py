"""AAVC live flight-monitoring web dashboard (lightweight competition GCS).

In-process FastAPI server hosted alongside the orchestrator. Exposes:
- `/api/plan` — the active MissionPlan
- `/api/config` — geofence + search-area polygons
- `/api/health` — orchestrator running state
- `/api/camera/frame.png` + `/api/camera/stream` — nadir camera (/tmp/aavc_frame.jpg)
- `/api/cmd/*` — guarded operator command channel
- `/ws/realtime` — 5 Hz telemetry + event-driven push (vision matches,
  detected objects, MAVLink commands, anomalies, drop predictions)

The orchestrator wires the dashboard through `dashboard.integration.start_dashboard`.

Complementary to (not a replacement for) QGroundControl — manual safety-pilot
RC + calibration.
"""

from .integration import DashboardHandle, start_dashboard
from .realtime import RealtimeBroadcaster
from .server import DashboardServer, make_app

__all__ = [
    "DashboardHandle",
    "DashboardServer",
    "RealtimeBroadcaster",
    "make_app",
    "start_dashboard",
]
