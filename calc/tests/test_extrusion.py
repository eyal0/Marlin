"""Filament accounting, positioning modes, and the printed area."""

import math

import pytest

# Three extruding moves; absolute E ends at 3 mm, relative E sums to 6 mm.
EXTRUDE = "G1 X10 E1 F1200\nG1 X20 E2\nG1 X30 E3\n"


def cross_section_mm2(diameter):
    return math.pi * diameter ** 2 / 4


def test_extrusion_is_absolute_by_default(calc):
    assert calc(EXTRUDE).filament() == pytest.approx(3)


def test_m83_makes_extrusion_relative(calc):
    assert calc("M83\n" + EXTRUDE).filament() == pytest.approx(6)
    assert calc("M83\nM82\n" + EXTRUDE).filament() == pytest.approx(3)


def test_g91_makes_all_axes_relative(calc):
    relative = calc("G91\n" + EXTRUDE)
    assert relative.filament() == pytest.approx(6)
    assert relative.analysis["printingArea"]["maxX"] == pytest.approx(60)
    absolute = calc("G91\nG90\n" + EXTRUDE)
    assert absolute.filament() == pytest.approx(3)
    assert absolute.analysis["printingArea"]["maxX"] == pytest.approx(30)


def test_g92_resets_the_extruder_position(calc):
    result = calc("G1 X10 E1 F1200\nG1 X20 E2\nG92 E0\nG1 X30 E1\nG1 X40 E2\n")
    assert result.filament() == pytest.approx(4)


def test_g92_moves_the_origin(calc):
    # After G92 X0 at X100, "X100" is another 100 mm forward, not a no-op.
    shifted = calc("G1 X100 F6000\nG92 X0\nG1 X100\nG1 X0\n")
    plain = calc("G1 X100 F6000\nG1 X200\nG1 X100\n")
    assert shifted.time == pytest.approx(plain.time, rel=1e-9)


def test_retract_and_prime_cancel_out(calc):
    result = calc("G1 X10 E1 F1200\nG1 E-1\nG1 E1\nG1 X20 E2\nG1 X30 E3\n")
    assert result.filament() == pytest.approx(3)


def test_filament_is_tracked_per_tool(calc):
    assert calc("T1\n" + EXTRUDE).analysis["filament"].keys() == {"tool1"}
    both = calc(EXTRUDE + "T1\nG92 E0\nG1 X40 E2\nG1 X50 E4\nG1 X60 E6\n")
    assert both.filament(0) == pytest.approx(3)
    assert both.filament(1) == pytest.approx(6)


def test_m221_scales_filament_for_one_tool(calc):
    assert calc("M221 S50\n" + EXTRUDE).filament() == pytest.approx(1.5)
    assert calc("M221 T1 S200\n" + EXTRUDE).filament(0) == pytest.approx(3)
    assert calc("M221 T1 S200\nT1\n" + EXTRUDE).filament(1) == pytest.approx(6)


def test_volume_uses_nominal_filament_diameter(calc):
    result = calc(EXTRUDE)
    assert result.volume() == pytest.approx(3 * cross_section_mm2(1.75) / 1000)


def test_m200_enables_volumetric_extrusion(calc):
    # In volumetric mode E is in mm^3 of plastic. Filament is counted in
    # whole extruder steps.
    result = calc("M200 D2.85\n" + EXTRUDE)
    assert result.filament() == pytest.approx(3 / cross_section_mm2(2.85), abs=1 / 500)
    assert calc("M200 D2.85\nM200 S0\n" + EXTRUDE).filament() == pytest.approx(3)


def test_volumetric_volume_matches_the_gcode(calc):
    assert calc("M200 D2.85\n" + EXTRUDE).volume() == pytest.approx(3 / 1000, rel=1e-3)


def test_printing_area_covers_only_extruding_moves(calc):
    result = calc(
        "G1 X10 Y10 Z0.2 F3000\n"
        "G1 X50 Y10 E1\nG1 X50 Y40 E2\nG1 X10 Y40 E3\nG1 X10 Y10 E4\n"
        "G1 Z0.4\nG1 X50 Y40 E5\n"
        "G1 X200 Y200 Z10\n")
    assert result.analysis["printingArea"] == pytest.approx(
        {"minX": 10, "maxX": 50, "minY": 10, "maxY": 40, "minZ": 0.2, "maxZ": 0.4})
    assert result.analysis["dimensions"] == pytest.approx(
        {"width": 40, "depth": 30, "height": 0.2})
