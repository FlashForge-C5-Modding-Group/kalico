# Creator 5 mainBoardGD motor-control native tests
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import pathlib
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
COMMON_FLAGS = [
    "gcc", "-std=gnu11", "-O2", "-Wall", "-Wextra", "-Werror",
    "-ffp-contract=off", "-fno-math-errno", "-I", "src",
]


class C5MCLibTest(unittest.TestCase):
    def _compile_and_run(self, name, sources):
        with tempfile.TemporaryDirectory() as temp:
            executable = pathlib.Path(temp) / name
            compile_result = subprocess.run(
                COMMON_FLAGS + list(sources)
                + ["-lm", "-o", str(executable)],
                cwd=ROOT, text=True, capture_output=True)
            self.assertEqual(compile_result.returncode, 0,
                             compile_result.stdout + compile_result.stderr)
            run_result = subprocess.run(
                [str(executable)], cwd=ROOT, text=True, capture_output=True)
            self.assertEqual(run_result.returncode, 0,
                             run_result.stdout + run_result.stderr)

    def test_production_motor_vectors(self):
        self._compile_and_run(
            "c5_mclib",
            ["src/c5_mclib.c", "test/c5_mclib.c"])

if __name__ == "__main__":
    unittest.main()
