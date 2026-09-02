"""Shell-script trap lint: pgrep/pkill -f patterns must not match themselves.

The trap (docs/RESUME_2026-08-19.md §5, hit AGAIN live on 2026-08-20 mid
field-debug): ``pkill -f mavlink-routerd`` inside an ssh one-liner matches
the ssh'd shell's OWN command line — the shell kills itself before the next
statement runs, and the caller sees exit 255 with no output. The repo-wide
idiom is a character class that breaks self-matching (``mavlink-route[r]d``,
``orchestrator.mai[n]``) or an escaped dot (``camera_grabber\\.py`` — a
literal-dot regex does not appear verbatim in the killing command line).

This test walks every tracked ``*.sh`` and flags a ``pgrep -f``/``pkill -f``
whose pattern carries neither guard. Heuristic on purpose: a flagged line is
a line a human must look at, and the fix (add ``[x]``) is always cheap.
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]

_CALL = re.compile(r"""\b(?:pgrep|pkill)\s+(?:-\w+\s+)*-f\s+(?P<pat>"[^"]+"|'[^']+'|\S+)""")


def _tracked_shell_scripts() -> list[Path]:
    out = subprocess.run(
        ["git", "ls-files", "*.sh"], cwd=REPO,
        capture_output=True, text=True, check=True)
    return [REPO / line for line in out.stdout.splitlines() if line]


def _is_guarded(pattern: str) -> bool:
    body = pattern.strip("\"'")
    # [x] class breaks self-match; an escaped char means the regex text differs
    # from the process cmdline; $-expansion means the pattern is not a literal
    # present in this script's own text.
    return ("[" in body) or ("\\" in body) or ("$" in body)


def test_every_pgrep_pkill_f_pattern_is_self_match_safe() -> None:
    offenders: list[str] = []
    for script in _tracked_shell_scripts():
        for lineno, line in enumerate(script.read_text().splitlines(), 1):
            stripped = line.strip()
            if stripped.startswith("#"):
                continue
            for m in _CALL.finditer(stripped):
                if not _is_guarded(m.group("pat")):
                    offenders.append(
                        f"{script.relative_to(REPO)}:{lineno}: {stripped}")
    assert not offenders, (
        "unguarded pgrep/pkill -f pattern(s) — add the [x] bracket idiom "
        "(self-match kills the calling shell):\n" + "\n".join(offenders))
