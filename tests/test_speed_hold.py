# SPDX-FileCopyrightText: Generative Bionics S.R.L.
# SPDX-License-Identifier: BSD-3-Clause

"""Hardware-free checks of the PVT speed reference and PDO range validation."""

import os
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest


class SpeedHoldTests(unittest.TestCase):
    def test_reference(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory(prefix="speed-hold-test-") as directory:
            binary = Path(directory) / "speed-hold-probe"
            compiler = shlex.split(os.environ.get("CXX", "c++"))
            subprocess.run(
                compiler
                + [
                    "-std=c++20",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-fsanitize=undefined",
                    "-fno-sanitize-recover=undefined",
                    "-I",
                    str(root / "include"),
                    str(root / "tests/speed_hold_probe.cpp"),
                    "-o",
                    str(binary),
                ],
                check=True,
            )
            subprocess.run([str(binary)], check=True, timeout=10)


if __name__ == "__main__":
    unittest.main()
