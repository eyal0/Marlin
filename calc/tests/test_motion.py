"""Timing of linear moves, dwells, homing, and the commands that tune motion.

Expected times come from the physics of trapezoidal velocity profiles, with
the parameters under test set in the G-code rather than taken from
Configuration.h.
"""

import math

import pytest

from marlin_calc import back_and_forth

NO_JERK = "M205 X0 Y0 Z0 E0\n"


def polygon_laps(extrude: bool) -> str:
    """Three laps around a 16-gon at 100 mm/s: many shallow corners."""
    lines = ["G1 X120 Y100 F6000"]
    for i in range(1, 3 * 16 + 1):
        angle = 2 * math.pi * i / 16
        move = f"G1 X{100 + 20 * math.cos(angle):.3f} Y{100 + 20 * math.sin(angle):.3f}"
        lines.append(move + (f" E{0.3 * i:.3f}" if extrude else ""))
    return "\n".join(lines) + "\n"


PRINTING_LAPS = polygon_laps(extrude=True)
TRAVEL_LAPS = polygon_laps(extrude=False)

TRAVEL = back_and_forth(3, 100, 6000)
PRINTING = "G1 F6000\nG1 X100 E5\nG1 X0 E10\nG1 X100 E15\n"
RETRACTING = "G1 F1500\nG1 E-2\nG1 E0\nG1 E-2\nG1 E0\n"


@pytest.mark.parametrize("feedrate", [10, 50, 150])
def test_cruise_time_matches_feedrate(calc, feedrate):
    # Lengthening each move by 100 mm only extends its constant-speed part.
    short = calc(NO_JERK + back_and_forth(3, 100, feedrate * 60))
    long = calc(NO_JERK + back_and_forth(3, 200, feedrate * 60))
    assert long.time - short.time == pytest.approx(3 * 100 / feedrate, rel=1e-6)


@pytest.mark.parametrize("distance, feedrate, accel", [
    (100, 100, 1000),
    (50, 20, 500),
    (200, 150, 3000),
])
def test_move_from_rest_follows_trapezoid(calc, distance, feedrate, accel):
    # With zero jerk every reversal stops, so each move takes d/v + v/a.
    result = calc(NO_JERK + f"M204 T{accel}\n" +
                  back_and_forth(3, distance, feedrate * 60))
    expected = 3 * (distance / feedrate + feedrate / accel)
    assert result.time == pytest.approx(expected, rel=0.01)


def test_short_move_follows_triangle_profile(calc):
    # 10 mm is too short to reach 200 mm/s at 500 mm/s^2, so each move
    # accelerates for half its length and decelerates for the rest. The high
    # steps/mm makes Marlin's minimum step rate negligible.
    result = calc(NO_JERK + "M92 X1600\nM204 T500\n" +
                  back_and_forth(3, 10, 200 * 60))
    assert result.time == pytest.approx(3 * 2 * math.sqrt(10 / 500), rel=0.01)


@pytest.mark.parametrize("moves", [1, 2])
def test_files_with_few_moves_are_timed(calc, moves):
    result = calc(NO_JERK + "M204 T1000\n" + back_and_forth(moves, 100, 6000))
    assert result.time == pytest.approx(moves * (100 / 100 + 100 / 1000), rel=0.01)


def test_g0_is_timed_like_g1(calc):
    assert calc(TRAVEL.replace("G1", "G0")).time == calc(TRAVEL).time


def test_feedrate_is_modal(calc):
    once = calc("G1 X100 F6000\nG1 X0\nG1 X100\n")
    every_line = calc("G1 X100 F6000\nG1 X0 F6000\nG1 X100 F6000\n")
    assert once.time == every_line.time


@pytest.mark.parametrize("axis, limit", [("X", 50), ("Y", 50), ("Z", 2)])
def test_max_feedrate_caps_speed(calc, axis, limit):
    def moves(feedrate_mm_s):
        return (f"M203 {axis}{limit}\nG1 F{feedrate_mm_s * 60}\n"
                f"G1 {axis}10\nG1 {axis}0\nG1 {axis}10\n")
    assert calc(moves(4 * limit)).time == pytest.approx(
        calc(moves(limit)).time, rel=1e-9)


def test_absurd_feedrate_is_capped(calc):
    # A slicer bug produced this line and an infinite estimate:
    # https://github.com/eyal0/OctoPrint-PrintTimeGenius/issues/128
    def time(z_move):
        return calc(f"M203 Z5\nG1 X10 F3000\nG1 X20\n{z_move}\nG1 X30 F3000\nG1 X40\n").time
    assert time("G1 F17987547480000 Z0.9") == pytest.approx(time("G1 F300 Z0.9"), rel=1e-3)


@pytest.mark.parametrize("m204, slowed", [
    ("M204 T100", {"travel"}),
    ("M204 P100", {"printing"}),
    ("M204 S100", {"travel", "printing"}),
    ("M204 R100", {"retracting"}),
])
def test_m204_acceleration_applies_to_its_move_type(calc, m204, slowed):
    moves = {"travel": TRAVEL, "printing": PRINTING, "retracting": RETRACTING}
    for kind, gcode in moves.items():
        default = calc(NO_JERK + gcode).time
        changed = calc(NO_JERK + m204 + "\n" + gcode).time
        if kind in slowed:
            assert changed > default, kind
        else:
            assert changed == default, kind


def test_m201_limits_acceleration_per_axis(calc):
    x_moves = back_and_forth(3, 100, 3000)
    assert calc(NO_JERK + "M201 X100\n" + x_moves).time == pytest.approx(
        calc(NO_JERK + "M204 T100\n" + x_moves).time, rel=1e-9)
    assert calc(NO_JERK + "M201 Y100\n" + x_moves).time == calc(NO_JERK + x_moves).time


@pytest.mark.parametrize("letter, slow_moves", [
    ("T", "G1 F60\nG1 X20\nG1 X0\nG1 X20\n"),
    ("S", "G1 F60\nG1 X20 E1\nG1 X0 E2\nG1 X20 E3\n"),
])
def test_m205_minimum_feedrate(calc, letter, slow_moves):
    # M205 T is the minimum travel feedrate, M205 S the minimum when extruding.
    raised = calc(NO_JERK + f"M205 {letter}5\n" + slow_moves)
    at_minimum = calc(NO_JERK + slow_moves.replace("F60", "F300"))
    assert raised.time == pytest.approx(at_minimum.time, rel=1e-9)


def test_m220_scales_feedrate(calc):
    half_speed = calc("M220 S50\n" + back_and_forth(3, 100, 3000))
    assert half_speed.time == pytest.approx(
        calc(back_and_forth(3, 100, 1500)).time, rel=1e-9)


def test_higher_jerk_takes_corners_faster(calc):
    times = [calc(f"M205 X{jerk} Y{jerk}\n" + PRINTING_LAPS).time
             for jerk in (0, 5, 10, 20)]
    assert times[0] > times[1] > times[2] > times[3]


@pytest.mark.parametrize("jerk, seconds_per_corner", [(5, 0.05), (11, 0)])
def test_corner_speed_is_limited_by_jerk(calc, jerk, seconds_per_corner):
    # At 10 mm/s each corner of the square changes the X and Y speeds by
    # 10 mm/s. A 5 mm/s jerk limit slows each corner to 5 mm/s, which at
    # 50 mm/s^2 costs 2(v - vc)/a - (v^2 - vc^2)/(a v) = 0.05 s; 11 mm/s
    # allows the corner at full speed.
    settings = f"M204 T50\nM205 X{jerk} Y{jerk}\n"
    square = calc(settings + "G1 F600\nG1 X20\nG1 X20 Y20\nG1 X0 Y20\nG1 X0 Y0\n")
    straight = calc(settings + "G1 F600\nG1 X20\nG1 X40\nG1 X60\nG1 X80\n")
    assert square.time - straight.time == pytest.approx(3 * seconds_per_corner, rel=0.02, abs=1e-4)


def test_printer_settings_choose_junction_deviation(calc):
    # A firmware using Junction Deviation reports J in M503's M205 line.
    classic = calc(PRINTING_LAPS).time
    small_deviation = calc(PRINTING_LAPS, mcodes="M205 B20000 S0 T0 J0.02").time
    large_deviation = calc(PRINTING_LAPS, mcodes="M205 B20000 S0 T0 J0.2").time
    assert small_deviation != classic
    assert large_deviation < small_deviation
    reported_classic = calc(PRINTING_LAPS, mcodes="M205 B20000 S0 T0 X10 Y10 Z0.3 E5").time
    assert reported_classic == classic


def test_m205_in_gcode_keeps_the_jerk_mode(calc):
    junction_deviation = "M205 J0.02"
    assert calc("M205 J0.02\n" + PRINTING_LAPS).time == calc(PRINTING_LAPS).time
    assert (calc("M205 X10 Y10\n" + PRINTING_LAPS, mcodes=junction_deviation).time
            == calc(PRINTING_LAPS, mcodes=junction_deviation).time)
    # It still changes the values the mode uses.
    assert (calc("M205 J0.2\n" + PRINTING_LAPS, mcodes=junction_deviation).time
            == calc(PRINTING_LAPS, mcodes="M205 J0.2").time)


def test_out_of_range_junction_deviation_is_ignored(calc):
    junction_deviation = "M205 J0.02"
    assert (calc("M205 J5\n" + PRINTING_LAPS, mcodes=junction_deviation).time
            == calc(PRINTING_LAPS, mcodes=junction_deviation).time)


def test_linear_advance_slows_extruding_moves(calc):
    without = calc(PRINTING_LAPS).time
    assert calc("M900 K0\n" + PRINTING_LAPS).time == without
    assert (without < calc("M900 K0.05\n" + PRINTING_LAPS).time
            < calc("M900 K0.5\n" + PRINTING_LAPS).time)


def test_linear_advance_does_not_affect_travel(calc):
    assert calc("M900 K0.5\n" + TRAVEL_LAPS).time == calc(TRAVEL_LAPS).time


def test_out_of_range_linear_advance_is_ignored(calc):
    assert calc("M900 K20\n" + PRINTING_LAPS).time == calc(PRINTING_LAPS).time


def test_z_moves_use_z_limits(calc):
    result = calc(NO_JERK + "M92 Z4000\nM203 Z5\nM201 Z100\n"
                  "G1 F600\nG1 Z5\nG1 Z0\nG1 Z5\n")
    assert result.time == pytest.approx(3 * (5 / 5 + 5 / 100), rel=0.01)


@pytest.mark.parametrize("gcode, seconds", [
    ("G4 P500", 0.5),
    ("G4 S2", 2),
    ("G4 S1.5", 1.5),
    ("G4 P0", 0),
    ("G4 P250\nG4 S1", 1.25),
])
def test_dwell(calc, gcode, seconds):
    assert calc(gcode + "\n").time == pytest.approx(seconds, abs=1e-12)


def test_dwell_adds_to_motion_time(calc):
    # Like M400, G4 first waits for the moves before it to finish.
    first, second = "G1 F6000\nG1 X100\nG1 X0\n", "G1 X100\nG1 X0\n"
    assert calc(first + "G4 S3\n" + second).time == pytest.approx(
        calc(first + "M400\n" + second).time + 3, rel=1e-12)


@pytest.mark.parametrize("gcode", ["G28", "G28 X Y"])
def test_homing_takes_five_seconds(calc, gcode):
    assert calc(gcode + "\n").time == 5
