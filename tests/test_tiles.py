"""Offline map tiles (2026-08-27).

The console bundles Leaflet but the map tiles came straight from
tile.openstreetmap.org, so at a field with no internet (the rules ban SIM
cards; the KMITL console was opened offline and showed a blank map) the
zones drew on black. The practice field only ever "worked" on the browser's
own cache from earlier online sessions. These pin the fix: tiles are served
from a repo-local cache the console fills while online and
scripts/prefetch_tiles.py fills ahead of a field day.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import aavc_gcs  # noqa: E402


def test_tile_xy_matches_the_slippy_map_convention():
    # KMITL P1 (rules Table 1) — checked by hand against the OSM formula.
    assert aavc_gcs.tile_xy(13.730322, 100.787446, 17) == (102231, 60488)
    assert aavc_gcs.tile_xy(13.730322, 100.787446, 19) == (408926, 241953)


def test_tile_cache_path_accepts_only_slippy_paths(tmp_path, monkeypatch):
    monkeypatch.setattr(aavc_gcs, "TILE_DIR", str(tmp_path))
    p = aavc_gcs.tile_cache_path("/tiles/17/102231/60488.png")
    assert p == os.path.join(str(tmp_path), "17", "102231", "60488.png")
    for bad in ("/tiles/17/../../etc/passwd", "/tiles/17/102231/60488.jpg",
                "/tiles/25/1/1.png", "/tiles/17/999999999/1.png",
                "/tiles/-1/1/1.png", "/leaflet.js"):
        assert aavc_gcs.tile_cache_path(bad) is None, bad


def test_tiles_covering_the_kmitl_field_stays_a_few_hundred():
    tiles = aavc_gcs.tiles_covering(13.729806, 100.787175, 13.731359,
                                    100.789916, 19, margin_m=300.0)
    assert 100 <= len(tiles) <= 200            # a field, not a city
    assert all(0 <= x < 2 ** 19 and 0 <= y < 2 ** 19 for x, y in tiles)
    assert (408926, 241953) in tiles           # P1 itself is covered


def test_served_tile_route_reads_the_cache(tmp_path, monkeypatch):
    """A cached tile is served as image/png from the local route — the path
    the page must use instead of the OSM host."""
    monkeypatch.setattr(aavc_gcs, "TILE_DIR", str(tmp_path))
    d = os.path.join(str(tmp_path), "17", "102231")
    os.makedirs(d)
    with open(os.path.join(d, "60488.png"), "wb") as fh:
        fh.write(b"\x89PNG-fake")
    data, ctype = aavc_gcs.serve_tile("/tiles/17/102231/60488.png", online=False)
    assert data == b"\x89PNG-fake" and ctype == "image/png"
    assert aavc_gcs.serve_tile("/tiles/17/102231/60489.png", online=False) is None


def test_page_uses_the_local_tile_route():
    assert "tile.openstreetmap.org" not in aavc_gcs.PAGE
    assert "/tiles/{z}/{x}/{y}.png" in aavc_gcs.PAGE
