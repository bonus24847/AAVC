"""Pre-flight tuning aid (System-ID + Autotune) — NOT part of the scored sortie.

A SITL/bench tool that identifies the multicopter rate-loop plant from a
frequency-sweep flight (``tuning.sysid``, numpy-only) and designs PID gains from
it (``tuning.engine``/``synthesis``/``plant``), to be compared against PX4's
built-in autotune and baked into the deterministic competition mission. See
CLAUDE.md §2/§4 — this is the explicitly-scoped exception to "no tuning".
"""
