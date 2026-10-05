# Creator 5 eBoard pressure-advance scorer native test
#
# Copyright (C) 2026
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import pathlib
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class C5EBoardPAScorerTest(unittest.TestCase):
    def compile_and_run(self, sources, name, optimization="-O0"):
        with tempfile.TemporaryDirectory() as temp:
            executable = pathlib.Path(temp) / name
            compile_result = subprocess.run(
                ["gcc", "-std=gnu11", optimization, "-Wall", "-Wextra",
                 "-Werror", "-I", "test/c5_eboard_stubs", "-I",
                 "test/c5_levelboard_stubs", "-I", "src", *sources,
                 "-o", str(executable)],
                cwd=ROOT, text=True, capture_output=True)
            self.assertEqual(compile_result.returncode, 0,
                             compile_result.stdout + compile_result.stderr)
            run_result = subprocess.run(
                [str(executable)], cwd=ROOT, text=True, capture_output=True)
            self.assertEqual(run_result.returncode, 0,
                             run_result.stdout + run_result.stderr)

    def test_production_scorer_vectors(self):
        self.compile_and_run(
            ["src/c5_eboard_pa.c", "test/c5_eboard_pa.c"],
            "c5_eboard_pa", "-O2")

    def test_production_logic_safety(self):
        self.compile_and_run(
            ["test/c5_eboard_safety.c", "src/c5_eboard_pa.c"],
            "c5_eboard_safety")

    def test_low_level_quiesce_and_rearm_registers(self):
        self.compile_and_run(
            ["test/c5_eboard_registers.c"], "c5_eboard_registers")


if __name__ == "__main__":
    unittest.main()
