"""``run_delivery_mission``'s ``max_pads`` default must match the field's pad
count.

The sweep only early-stops once ``max_pads`` distinct ids are confirmed
(``mission.py`` ``_sweep_for._done``). The field carries SIX pads, and
``main.py`` passes config's ``search.max_pads`` (default 6). The function
default sat at 4 — a footgun: any caller that omits the kwarg would truncate the
sweep after 4 of 6, the exact regression the ``_done()`` comment warns about.
"""

import inspect

from orchestrator.mission import run_delivery_mission


def test_max_pads_default_matches_the_six_pad_field() -> None:
    default = inspect.signature(run_delivery_mission).parameters["max_pads"].default
    assert default == 6
