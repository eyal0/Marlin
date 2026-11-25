"""Whole-file regression tests against recorded results.

golden.json records marlin-calc's results for each file in gcode/. A failure
means the estimates changed. If the change is intended, re-record with

    pytest calc_tests --update-golden

and review the diff of golden.json.
"""

import json
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
GOLDEN = HERE / "golden.json"
FIXTURES = sorted((HERE / "gcode").glob("*.gcode"))
CHECKPOINTS = [i / 10 for i in range(1, 10)]


def summarize(result):
    """Flatten a result into named numbers."""
    summary = {"estimatedPrintTime": result.time, "blocks": len(result.progress)}
    for tool, usage in result.analysis["filament"].items():
        summary[f"filament.{tool}"] = usage["length"]
    for bound, value in result.analysis["printingArea"].items():
        summary[f"printingArea.{bound}"] = value
    # When the print reaches each tenth of the file: what the plugin uses to
    # estimate the time remaining.
    for fraction in CHECKPOINTS:
        summary[f"time@{fraction:.0%}"] = next(
            (p.time for p in result.progress if p.filepos >= fraction), None)
    return summary


@pytest.fixture(scope="module")
def golden(pytestconfig):
    recorded = json.loads(GOLDEN.read_text()) if GOLDEN.exists() else {}
    yield recorded
    if pytestconfig.getoption("update_golden"):
        current = {name: recorded[name] for name in sorted(recorded)
                   if (HERE / "gcode" / name).exists()}
        GOLDEN.write_text(json.dumps(current, indent=2) + "\n")


@pytest.mark.parametrize("path", FIXTURES, ids=lambda path: path.name)
def test_matches_golden(marlin_calc, golden, pytestconfig, path):
    actual = summarize(marlin_calc.run(path))
    if pytestconfig.getoption("update_golden"):
        golden[path.name] = actual
        return
    assert path.name in golden, "no recorded results: run with --update-golden"
    # Allows for math libraries rounding sin/cos differently in arcs.
    assert actual == pytest.approx(golden[path.name], rel=1e-5)
