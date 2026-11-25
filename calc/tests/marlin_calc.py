"""Run marlin-calc and parse its output.

The output format is a contract with OctoPrint-PrintTimeGenius
(octoprint_PrintTimeGenius/analyzers/analyze_progress.py), so parsing here is
strict: every line on stdout must be a "Progress:" line or the final
"Analysis:" line, and the Analysis JSON is decoded the same way the plugin
decodes it.
"""

from __future__ import annotations

import dataclasses
import json
import re
import subprocess
from pathlib import Path

# The plugin rewrites these non-JSON tokens before calling json.loads().
_INF_RE = re.compile(r":\s*-?inf\b", re.IGNORECASE)
_NAN_RE = re.compile(r":\s*nan\b", re.IGNORECASE)


@dataclasses.dataclass(frozen=True)
class Progress:
    filepos: float  # Fraction of the input file read when the block was queued.
    filament: float  # Filament extruded before the block, in mm.
    time: float  # Total print time when the block finishes, in seconds.


@dataclasses.dataclass(frozen=True)
class Result:
    progress: list[Progress]
    analysis: dict
    stderr: str

    @property
    def time(self) -> float:
        return self.analysis["estimatedPrintTime"]

    def filament(self, tool: int = 0) -> float:
        """Filament length used by `tool`, in mm."""
        return self.analysis["filament"].get(f"tool{tool}", {}).get("length", 0.0)

    def volume(self, tool: int = 0) -> float:
        """Filament volume used by `tool`, in cm^3."""
        return self.analysis["filament"].get(f"tool{tool}", {}).get("volume", 0.0)


def parse_output(stdout: str, stderr: str = "") -> Result:
    progress = []
    analysis = None
    for line in stdout.splitlines():
        if analysis is not None:
            raise ValueError(f"output after the Analysis line: {line!r}")
        if line.startswith("Progress:"):
            fields = line[len("Progress:"):].split(",")
            if len(fields) != 3:
                raise ValueError(f"malformed Progress line: {line!r}")
            progress.append(Progress(*map(float, fields)))
        elif line.startswith("Analysis:"):
            text = _INF_RE.sub(": null", line[len("Analysis:"):])
            analysis = json.loads(_NAN_RE.sub(": null", text))
        else:
            raise ValueError(f"unexpected output line: {line!r}")
    if analysis is None:
        raise ValueError("no Analysis line in output")
    return Result(progress, analysis, stderr)


class MarlinCalc:
    def __init__(self, command: list[str], timeout: float = 300):
        self.command = command
        self.timeout = timeout

    def invoke(self, *args: str) -> subprocess.CompletedProcess:
        return subprocess.run([*self.command, *args], capture_output=True,
                              text=True, timeout=self.timeout)

    def run(self, gcode_path: Path, mcodes: str | None = None) -> Result:
        """Analyze a G-code file, optionally with settings that apply first.

        `mcodes` is passed as the second argument, the way the plugin passes
        the printer's M503 settings: newline-separated commands.
        """
        args = [str(gcode_path)] + ([mcodes] if mcodes is not None else [])
        proc = self.invoke(*args)
        if proc.returncode != 0:
            raise RuntimeError(
                f"marlin-calc exited with {proc.returncode}:\n{proc.stderr}")
        return parse_output(proc.stdout, proc.stderr)


def back_and_forth(moves: int, distance: float, feedrate_mm_min: float) -> str:
    """G-code for `moves` X moves of `distance` mm, alternating direction.

    Each move reverses the previous one, so with zero jerk every move starts
    and ends at rest. Use at least 3 moves: marlin-calc currently reports no
    time at all for files with fewer than 3 moves.
    """
    lines = [f"G1 F{feedrate_mm_min:.10g}"]
    lines += [f"G1 X{distance if i % 2 == 0 else 0:.10g}" for i in range(moves)]
    return "\n".join(lines) + "\n"
