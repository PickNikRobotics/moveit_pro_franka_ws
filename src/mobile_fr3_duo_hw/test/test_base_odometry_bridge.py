# Copyright 2026 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Tests for the pure parts of script/base_odometry_bridge.py and
script/spine_joint_state_publisher.py. No ROS."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script"))

from base_odometry_bridge import relative_pose, twist_in_base, wrap  # noqa: E402
from spine_joint_state_publisher import SpineHeight  # noqa: E402


def test_first_pose_is_the_origin():
    origin = (3.0, -2.0, 1.2)
    assert relative_pose(origin, origin) == pytest.approx((0.0, 0.0, 0.0))


def test_relative_pose_is_in_the_origin_frame():
    # Origin faces +y: a step along world +y is straight ahead.
    origin = (1.0, 1.0, math.pi / 2.0)
    assert relative_pose(origin, (1.0, 3.0, math.pi / 2.0)) == pytest.approx(
        (2.0, 0.0, 0.0)
    )
    # A step along world +x is to the right, which is base -y.
    assert relative_pose(origin, (2.0, 1.0, math.pi)) == pytest.approx(
        (0.0, -1.0, math.pi / 2.0)
    )


def test_relative_yaw_wraps():
    x, y, yaw = relative_pose((0.0, 0.0, 3.0), (0.0, 0.0, -3.0))
    assert yaw == pytest.approx(wrap(-6.0))
    assert -math.pi <= yaw < math.pi


def test_twist_in_base_rotates_by_the_heading():
    # Heading +y: moving along world +y is forward in the base frame.
    assert twist_in_base(0.0, 0.3, math.pi / 2.0) == pytest.approx((0.3, 0.0))
    # Heading zero: odometry axes and base axes agree.
    assert twist_in_base(0.2, -0.1, 0.0) == pytest.approx((0.2, -0.1))


def test_spine_height_unknown_until_first_valid_sample():
    height = SpineHeight()
    height.update(0.4, False)
    assert height.position is None
    height.update(0.25, True)
    assert height.position == 0.25


def test_spine_height_holds_through_invalid_samples():
    height = SpineHeight()
    height.update(0.3, True)
    height.update(0.0, False)
    assert height.position == 0.3
