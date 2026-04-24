"""
test_avoidance.py — Pure pytest for avoidance geometry logic.

No ROS process needed. Tests:
  - rotation_direction() for all three bump sides
  - AvoidanceManeuver instantiation (geometry checks only)
  - Phase parameter sanity checks (backup distance/speed/duration, rotate angle)
"""

import math
import pytest

from patrol_fsm.avoidance import (
    AvoidanceManeuver,
    rotation_direction,
    BUMP_LEFT,
    BUMP_RIGHT,
    BUMP_CENTER,
    _BACKUP_DISTANCE,
    _BACKUP_SPEED,
    _BACKUP_DURATION,
    _ROTATE_ANGLE,
    _ROTATE_SPEED,
    _ROTATE_DURATION,
)


# ── rotation_direction() ───────────────────────────────────────────────────────

class TestRotationDirection:

    def test_left_bump_rotates_right(self):
        assert rotation_direction(BUMP_LEFT) == -1.0

    def test_right_bump_rotates_left(self):
        assert rotation_direction(BUMP_RIGHT) == 1.0

    def test_center_bump_rotates_right(self):
        assert rotation_direction(BUMP_CENTER) == -1.0

    def test_unknown_side_defaults_to_right(self):
        assert rotation_direction('UNKNOWN') == -1.0

    def test_rotation_is_unit_magnitude(self):
        for side in (BUMP_LEFT, BUMP_RIGHT, BUMP_CENTER):
            assert abs(rotation_direction(side)) == 1.0


# ── Phase parameter sanity ────────────────────────────────────────────────────

class TestManeuverParameters:

    def test_backup_duration_matches_physics(self):
        assert math.isclose(_BACKUP_DURATION, _BACKUP_DISTANCE / _BACKUP_SPEED, rel_tol=1e-6)

    def test_rotate_duration_matches_physics(self):
        assert math.isclose(_ROTATE_DURATION, _ROTATE_ANGLE / _ROTATE_SPEED, rel_tol=1e-6)

    def test_backup_distance_is_positive(self):
        assert _BACKUP_DISTANCE > 0

    def test_rotate_angle_is_90_degrees(self):
        assert math.isclose(_ROTATE_ANGLE, math.pi / 2, rel_tol=1e-6)

    def test_speeds_are_positive(self):
        assert _BACKUP_SPEED > 0
        assert _ROTATE_SPEED > 0

    def test_durations_are_positive(self):
        assert _BACKUP_DURATION > 0
        assert _ROTATE_DURATION > 0


# ── AvoidanceManeuver instantiation ──────────────────────────────────────────

class TestAvoidanceManeuverInit:

    class _MockNode:
        def get_logger(self):
            class _L:
                def info(self, *a, **kw): pass
                def warn(self, *a, **kw): pass
            return _L()

    def test_stores_bump_side_left(self):
        m = AvoidanceManeuver(self._MockNode(), BUMP_LEFT, lambda: None)
        assert m._bump_side == BUMP_LEFT

    def test_stores_bump_side_right(self):
        m = AvoidanceManeuver(self._MockNode(), BUMP_RIGHT, lambda: None)
        assert m._bump_side == BUMP_RIGHT

    def test_stores_bump_side_center(self):
        m = AvoidanceManeuver(self._MockNode(), BUMP_CENTER, lambda: None)
        assert m._bump_side == BUMP_CENTER

    def test_done_callback_is_stored(self):
        cb = lambda: None
        m = AvoidanceManeuver(self._MockNode(), BUMP_LEFT, cb)
        assert m._done_callback is cb

    def test_avoid_dir_left(self):
        m = AvoidanceManeuver(self._MockNode(), BUMP_LEFT, lambda: None)
        assert m._avoid_dir == -1.0

    def test_avoid_dir_right(self):
        m = AvoidanceManeuver(self._MockNode(), BUMP_RIGHT, lambda: None)
        assert m._avoid_dir == 1.0
