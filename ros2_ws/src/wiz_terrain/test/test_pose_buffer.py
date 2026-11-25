"""Offline tests for PoseBuffer (timestamp-matched poses, no ROS)."""

import math

import pytest

from wiz_terrain.pose import Pose2D, PoseBuffer


def _buffer(*entries, **kwargs):
    buf = PoseBuffer(**kwargs)
    for t, x, y, yaw in entries:
        assert buf.add(t, Pose2D(x, y, yaw))
    return buf


def test_empty_buffer_has_no_pose():
    assert PoseBuffer().pose_at(1.0) is None


def test_exact_and_interpolated_poses():
    buf = _buffer((1.0, 0.0, 0.0, 0.0), (2.0, 4.0, -2.0, 1.0))
    assert buf.pose_at(1.0) == Pose2D(0.0, 0.0, 0.0)
    p = buf.pose_at(1.25)  # a quarter of the way
    assert (p.x, p.y, p.yaw) == pytest.approx((1.0, -0.5, 0.25))


def test_heading_interpolates_the_short_way_round():
    a, b = math.radians(170), math.radians(-170)  # 20 deg apart across +-180
    p = _buffer((0.0, 0.0, 0.0, a), (1.0, 0.0, 0.0, b)).pose_at(0.5)
    assert abs(abs(p.yaw) - math.pi) < 1e-9  # halfway is 180 deg, not 0


def test_time_before_history_is_unknown():
    assert _buffer((1.0, 0.0, 0.0, 0.0), (2.0, 1.0, 0.0, 0.0)).pose_at(0.9) is None


def test_newest_pose_is_held_briefly_then_unknown():
    buf = _buffer((1.0, 0.0, 0.0, 0.0), (2.0, 1.0, 0.0, 0.0), max_hold_s=0.2)
    assert buf.pose_at(2.1) == Pose2D(1.0, 0.0, 0.0)
    assert buf.pose_at(2.5) is None


def test_out_of_order_pose_is_ignored():
    buf = _buffer((2.0, 1.0, 0.0, 0.0))
    assert not buf.add(1.5, Pose2D(9.0, 9.0, 0.0))
    assert not buf.add(2.0, Pose2D(9.0, 9.0, 0.0))
    assert len(buf) == 1


def test_old_poses_are_dropped():
    buf = _buffer(*[(float(t), float(t), 0.0, 0.0) for t in range(10)], max_history_s=2.0)
    assert len(buf) == 3            # t = 7, 8, 9 remain
    assert buf.pose_at(6.5) is None
    assert buf.pose_at(7.5).x == pytest.approx(7.5)


@pytest.mark.parametrize('kwargs', [
    dict(max_history_s=0.0),
    dict(max_history_s=float('nan')),
    dict(max_hold_s=-0.1),
])
def test_invalid_buffer_settings_rejected(kwargs):
    with pytest.raises(ValueError):
        PoseBuffer(**kwargs)


def test_non_finite_time_rejected():
    with pytest.raises(ValueError):
        PoseBuffer().add(float('inf'), Pose2D(0.0, 0.0, 0.0))
