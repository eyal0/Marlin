"""G2/G3 arcs.

The head approaches (50, 50) moving in +X, and arcs are taken at 10 mm/s with
20 mm/s of jerk, so no corner or arc segment forces a slowdown.
"""

import math

import pytest

START = "M205 X20 Y20\nG1 X40 Y50 F600\nG1 X50 Y50\n"


def test_full_circle_takes_its_circumference_at_feedrate(calc):
    leave = "G1 X80 Y50\n"
    with_circle = calc(START + "G2 I10 J0\n" + leave)
    without = calc(START + leave)
    assert with_circle.time - without.time == pytest.approx(2 * math.pi * 10 / 10, rel=0.01)


@pytest.mark.parametrize("command, bulge_y", [("G2", 60), ("G3", 40)])
def test_arc_direction(calc, command, bulge_y):
    # From (50, 50) around (60, 50) to (70, 50): clockwise passes the top of
    # the circle, counter-clockwise the bottom.
    area = calc(START + f"{command} X70 Y50 I10 J0 E5\nG1 X80 Y50\n").analysis["printingArea"]
    assert sorted([area["minY"], area["maxY"]]) == pytest.approx(sorted([50, bulge_y]), abs=0.05)


def test_mirrored_arcs_take_equal_time(calc):
    clockwise = calc(START + "G2 X70 Y50 I10 J0\nG1 X80 Y50\n")
    counter_clockwise = calc(START + "G3 X70 Y50 I10 J0\nG1 X80 Y50\n")
    # A half circle is where Marlin rounds its segment count up, so the last
    # bit of atan2f() decides between 36 and 37 segments. macOS's atan2f gives
    # G2 37, which takes 0.05 ms less.
    assert clockwise.time == pytest.approx(counter_clockwise.time, abs=1e-4)


def test_radius_form_matches_center_form(calc):
    center = calc(START + "G2 X60 Y60 I10 J0 E5\nG1 X80 Y60\n")
    radius = calc(START + "G2 X60 Y60 R10 E5\nG1 X80 Y60\n")
    assert radius.time == pytest.approx(center.time, rel=1e-6)
    assert radius.analysis["printingArea"] == pytest.approx(
        center.analysis["printingArea"], abs=1e-3)


def test_arc_extrusion_is_counted(calc):
    assert calc(START + "G2 I10 J0 E5\nG1 X80 Y50\n").filament() == pytest.approx(5)
