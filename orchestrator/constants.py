"""Shared flight-envelope constants (L3).

Small numeric thresholds that were independently hardcoded in the in-flight
watchdog, the manual-drop guard, and the post-flight verifier — collected here
so those three can't silently drift apart.
"""

from __future__ import annotations

# Ceiling watchdog offsets, ABOVE the profile altitude ceiling (rules: transit
# flies strictly at 20 m, so warn needs headroom). A transient poke past
# ceiling+WARN is an anomaly; a sustained hold past ceiling+BREACH triggers RTH.
# Mirrored by tools/verify_flight.py so the post-flight audit uses the same lines.
CEILING_WARN_M = 0.5
# KMUTNB: 2.0 -> 1.5. A 5 m ceiling with a 2 m breach band tolerated a
# sustained hold at 7 m — 40% over the rule; 1.5 keeps the RTH trigger at
# 6.5 m, proportionate to the small band while still clearing the ±0.7 m
# altitude-frame wander plus climb overshoot.
CEILING_BREACH_M = 1.5

# Release/land altitude interlock: the egg is committed only at/below this
# relative altitude — AlignParams.land_alt_threshold_m (1.5 m) + a 1.0 m margin.
# Used by the autonomous touchdown gate, the manual-drop guard (S3), and the
# verifier's "released near the ground?" check.
TOUCHDOWN_ALT_GUARD_M = 2.5
