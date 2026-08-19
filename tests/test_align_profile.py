"""The terminal-approach accept radius must follow the FIELD (its profile), not
the altitude ceiling.

Regression: commit a3e3b0f raised the KMUTNB practice ceiling 5 -> 10 m, which
silently killed the ``ceil <= 6`` guard in ``_align_for`` and let the tight
sky-field fall back to the wide 15 m accept radius — wider than the 14.5 m pad
spacing, so the align layer could lock onto a NEIGHBOURING pad. The accept
radius now rides on ``MissionProfile.terminal_accept_radius_m`` so it can never
drift with the ceiling again.
"""

from mission_brain.profile import COMPETITION, KMUTNB_SKYFIELD
from orchestrator.main import _align_for


def test_kmutnb_accept_radius_stays_tight_despite_the_raised_ceiling() -> None:
    # KMUTNB pads sit 14.5 m apart; the accept radius must stay well under that,
    # even though the ceiling is now 10 m (> the old <=6 tight-field guard)
    assert _align_for(KMUTNB_SKYFIELD, 2.0).accept_radius_m == 5.0


def test_competition_keeps_the_validated_wide_accept_radius() -> None:
    assert _align_for(COMPETITION, 2.0).accept_radius_m == 15.0
