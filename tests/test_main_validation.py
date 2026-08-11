"""Committee-assigned marker-id validation (L2).

--assigned-ids (CLI) and mission.assigned_marker_ids (config) fed straight into
int() with no range check: an out-of-range id (0, 7, 9) was silently accepted
and then never decoded, so the sortie burned window time deferring. Validate up
front against the competition id set with a clear error.
"""

from __future__ import annotations

import pytest

from orchestrator.main import _parse_assigned_ids


def test_valid_ids_parse_in_order() -> None:
    assert _parse_assigned_ids("3,1,4,6") == [3, 1, 4, 6]


def test_empty_is_empty() -> None:
    assert _parse_assigned_ids("") == []
    assert _parse_assigned_ids([]) == []


def test_list_input_from_config() -> None:
    assert _parse_assigned_ids([3, 1]) == [3, 1]


@pytest.mark.parametrize("bad", ["0", "7", "9", "3,7", "0,1"])
def test_out_of_range_rejected(bad: str) -> None:
    with pytest.raises(ValueError, match="valid"):
        _parse_assigned_ids(bad)


@pytest.mark.parametrize("bad", ["x", "3,x", "1.5"])
def test_non_integer_rejected(bad: str) -> None:
    with pytest.raises(ValueError):
        _parse_assigned_ids(bad)


def test_out_of_range_in_list_rejected() -> None:
    with pytest.raises(ValueError):
        _parse_assigned_ids([9])
