import itertools
import os
import platform
import shlex
from pathlib import Path

import pytest

from marlin_calc import MarlinCalc

REPO_ROOT = Path(__file__).resolve().parent.parent


def host_target() -> str:
    """The SConstruct target for this machine.

    Matches how OctoPrint-PrintTimeGenius picks which binary to run.
    """
    machine = platform.machine()
    if platform.system() == "Darwin":
        return f"darwin-{machine}"
    if platform.system() == "Windows":
        return f"windows-{machine}.exe"
    return machine


def pytest_addoption(parser):
    group = parser.getgroup("marlin-calc")
    group.addoption(
        "--marlin-calc", default=os.environ.get("MARLIN_CALC"),
        help="marlin-calc binary to test (default: $MARLIN_CALC, else this "
             "machine's build under Marlin/out)")
    group.addoption(
        "--emulator", default=os.environ.get("MARLIN_CALC_EMULATOR"),
        help="command that runs the binary, e.g. qemu-arm-static "
             "(default: $MARLIN_CALC_EMULATOR)")
    group.addoption(
        "--update-golden", action="store_true",
        help="rewrite golden.json from the current results instead of "
             "checking them")


@pytest.fixture(scope="session")
def marlin_calc(pytestconfig) -> MarlinCalc:
    binary = pytestconfig.getoption("marlin_calc")
    if binary is None:
        target = host_target()
        binary = REPO_ROOT / "Marlin" / "out" / target / f"marlin-calc.{target}"
    binary = Path(binary).resolve()
    if not binary.is_file():
        pytest.exit(f"marlin-calc not found at {binary}; build it with scons "
                    "or pass --marlin-calc", returncode=pytest.ExitCode.USAGE_ERROR)
    emulator = shlex.split(pytestconfig.getoption("emulator") or "")
    return MarlinCalc(emulator + [str(binary)])


@pytest.fixture
def calc(marlin_calc, tmp_path):
    """Analyze a G-code string (or bytes) and return the parsed Result."""
    counter = itertools.count()

    def run(gcode, mcodes=None):
        path = tmp_path / f"input{next(counter)}.gcode"
        path.write_bytes(gcode.encode() if isinstance(gcode, str) else gcode)
        return marlin_calc.run(path, mcodes)

    return run
