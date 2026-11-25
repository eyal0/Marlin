"""Firmware retraction: G10/G11 with M207/M208 settings."""

import pytest

PRINT = "G1 X10 E1 F1200\nG1 X20 E2\n"
RESUME = "G1 X30 E3\nG1 X40 E4\n"


def retract_and_recover(settings="", retract="G10"):
    return settings + PRINT + retract + "\nG11\n" + RESUME


def test_retraction_length_and_feedrates(calc):
    def time(length):
        return calc(retract_and_recover(f"M203 E50\nM207 S{length} F1200\nM208 F600\n")).time
    # 4 mm more retraction is 4 mm more at 20 mm/s out and 4 mm more at 10 mm/s back.
    assert time(8) - time(4) == pytest.approx(4 / 20 + 4 / 10, rel=1e-6)
    assert time(4) > calc(PRINT + RESUME).time


def test_retraction_defaults_are_marlins(calc):
    # RETRACT_LENGTH 3 mm at RETRACT_FEEDRATE 45 mm/s, RETRACT_RECOVER_FEEDRATE 8 mm/s.
    assert calc(retract_and_recover()).time == calc(
        retract_and_recover("M207 S3 F2700\nM208 F480\n")).time


def test_retraction_is_limited_by_max_e_feedrate(calc):
    # A 40 mm/s retraction capped at 10 mm/s takes as long as a 10 mm/s one.
    capped = calc(retract_and_recover("M203 E10\nM207 S4 F2400\nM208 F480\n"))
    at_cap = calc(retract_and_recover("M203 E50\nM207 S4 F600\nM208 F480\n"))
    assert capped.time == pytest.approx(at_cap.time, rel=1e-9)


def test_repeated_g10_and_g11_are_ignored(calc):
    once = calc(PRINT + "G10\nG11\n" + RESUME)
    twice = calc(PRINT + "G10\nG10\nG11\nG11\n" + RESUME)
    assert twice.time == once.time
    assert twice.filament() == once.filament()


def test_z_hop(calc):
    def time(hop):
        return calc(retract_and_recover(f"M203 Z5\nM207 Z{hop}\n")).time
    # 1 mm more hop is 1 mm more up and 1 mm more down at 5 mm/s.
    assert time(2) - time(1) == pytest.approx(2 * 1 / 5, rel=1e-6)
    assert time(1) > time(0)


def test_retraction_does_not_use_filament(calc):
    assert calc(retract_and_recover()).filament() == pytest.approx(4)


def test_recover_extra_adds_filament(calc):
    assert calc(retract_and_recover("M208 S1\n")).filament() == pytest.approx(5)


def test_swap_retraction_uses_swap_length(calc):
    # G10 S1 retracts by the M207 W swap length instead of M207 S.
    normal = calc(retract_and_recover("M207 S3 W3\n"))
    swap = calc(retract_and_recover("M207 S3 W3\n", retract="G10 S1"))
    long_swap = calc(retract_and_recover("M207 S3 W13\n", retract="G10 S1"))
    assert swap.time == pytest.approx(normal.time, rel=1e-9)
    assert long_swap.time > swap.time
