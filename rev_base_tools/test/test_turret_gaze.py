import math

from rev_base_tools.turret_gaze import aim, slew, sweep

LIMIT, HYST = math.radians(135), math.radians(10)


def test_aim_follows_travel_past_the_hysteresis():
    assert aim(math.pi / 2, 0.0, LIMIT, HYST) == math.pi / 2
    assert aim(0.1, 0.0, LIMIT, HYST) == 0.0
    assert aim(-0.5, 0.0, LIMIT, HYST) == -0.5


def test_aim_holds_its_side_straight_back():
    # Travel flickering across +-180 deg must not swing the turret over.
    assert aim(math.pi - 0.01, 0.0, LIMIT, HYST) == LIMIT
    assert aim(-math.pi + 0.01, LIMIT, LIMIT, HYST) == LIMIT
    assert aim(-math.pi + 0.01, -1.0, LIMIT, HYST) == -LIMIT


def test_sweep_turns_at_the_ends():
    target, heading = sweep(1.5, 1.0, 1.6, 0.2)
    assert (target, heading) == (1.6, -1.0)
    target, heading = sweep(target, heading, 1.6, 0.2)
    assert math.isclose(target, 1.4) and heading == -1.0


def test_slew_is_rate_limited():
    assert slew(0.0, 1.0, 0.06) == 0.06
    assert slew(0.0, -1.0, 0.06) == -0.06
    assert slew(0.98, 1.0, 0.06) == 1.0
