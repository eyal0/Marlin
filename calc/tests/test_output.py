"""The output contract with OctoPrint-PrintTimeGenius.

Every test here also goes through marlin_calc.parse_output, which rejects any
stdout line the plugin wouldn't understand.
"""

import pytest

from marlin_calc import back_and_forth

# Ends with a travel move, so the last Progress line has all filament counted.
PRINT = (
    "G1 X10 Y10 F3000\n"
    "G1 X50 Y10 E2\n"
    "G1 X50 Y50 E4\n"
    "G1 X10 Y50 E6\n"
    "G1 X10 Y10 E8\n"
    "G1 X0 Y0\n"
)

# Settings as the plugin passes them: the printer's M503 report, one command
# per line, including commands marlin-calc doesn't use.
M503_SETTINGS = "\n".join([
    "M92 X80.00 Y80.00 Z400.00 E93.00",
    "M203 X500.00 Y500.00 Z12.00 E120.00",
    "M201 X500 Y500 Z100 E5000",
    "M204 P500.00 R1000.00 T500.00",
    "M205 B20000.00 S0.00 T0.00 X8.00 Y8.00 Z0.40 E5.00",
    "M200 S0 D1.75",
    "M301 P22.20 I1.08 D114.00",
    "M900 K0.00",
])


def test_analysis_fields(calc):
    analysis = calc(PRINT).analysis
    assert set(analysis) == {"estimatedPrintTime", "printingArea", "dimensions", "filament"}
    assert set(analysis["printingArea"]) == {"minX", "maxX", "minY", "maxY", "minZ", "maxZ"}
    assert set(analysis["dimensions"]) == {"width", "depth", "height"}
    assert set(analysis["filament"]) == {"tool0"}
    assert set(analysis["filament"]["tool0"]) == {"length", "volume"}


def test_progress_is_monotonic_and_ends_at_the_total(calc):
    result = calc(PRINT)
    filepos = [p.filepos for p in result.progress]
    times = [p.time for p in result.progress]
    assert len(result.progress) == 6
    assert filepos == sorted(filepos)
    assert 0 < filepos[0] and filepos[-1] == 1
    assert times == sorted(times)
    assert times[-1] == result.time


def test_progress_reports_filament_extruded_so_far(calc):
    result = calc(PRINT)
    filament = [p.filament for p in result.progress]
    assert filament == sorted(filament)
    assert filament[0] == 0
    assert filament[-1] == result.filament() == 8


def test_settings_argument_applies_before_the_file(calc):
    with_settings = calc(PRINT, mcodes=M503_SETTINGS)
    inline = calc(M503_SETTINGS + "\n" + PRINT)
    without = calc(PRINT)
    assert with_settings.time == inline.time != without.time
    # The settings aren't part of the file, so they don't shift its progress.
    assert ([p.filepos for p in with_settings.progress]
            == [p.filepos for p in without.progress])


def test_printing_area_without_extrusion_is_null(calc):
    # marlin-calc prints inf here, which the plugin turns into null.
    analysis = calc(back_and_forth(3, 100, 6000)).analysis
    assert set(analysis["printingArea"].values()) == {None}
    assert set(analysis["dimensions"].values()) == {None}
    assert analysis["filament"] == {}


@pytest.mark.xfail(reason="the last line's filepos is -1/filesize when the file "
                          "doesn't end with a newline: tellg() fails at EOF")
def test_last_progress_reaches_end_of_file_without_trailing_newline(calc):
    assert calc(PRINT.rstrip("\n")).progress[-1].filepos == 1


@pytest.mark.xfail(reason="a missing input file is analyzed as an empty one, "
                          "exiting with status 0")
def test_missing_input_file_is_an_error(marlin_calc, tmp_path):
    assert marlin_calc.invoke(str(tmp_path / "missing.gcode")).returncode != 0
