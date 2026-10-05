# Creator 5 levelBoard production-source safety regressions
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import pathlib
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
COMMON = ["gcc", "-std=gnu11", "-O0", "-Wall", "-Wextra", "-Werror"]


class C5LevelBoardSafetyTest(unittest.TestCase):
    def compile_and_run(self, source, name, include_dirs, extra_flags=()):
        with tempfile.TemporaryDirectory() as temp:
            executable = pathlib.Path(temp) / name
            command = COMMON[:]
            for include_dir in include_dirs:
                command.extend(["-I", include_dir])
            command.extend(extra_flags)
            command.extend([source, "-o", str(executable)])
            compile_result = subprocess.run(
                command, cwd=ROOT, text=True, capture_output=True)
            self.assertEqual(compile_result.returncode, 0,
                             compile_result.stdout + compile_result.stderr)
            try:
                run_result = subprocess.run(
                    [str(executable)], cwd=ROOT, text=True,
                    capture_output=True, timeout=2)
            except subprocess.TimeoutExpired as exc:
                self.fail("%s timed out: %s%s" %
                          (name, exc.stdout or "", exc.stderr or ""))
            self.assertEqual(run_result.returncode, 0,
                             run_result.stdout + run_result.stderr)

    def test_calibration_uses_wrap_safe_raw_ticks(self):
        self.compile_and_run(
            "test/c5_levelboard_logic.c", "c5_levelboard_logic",
            ["test/c5_levelboard_stubs", "src"])

    def test_warm_dma_and_clock_startup_register_model(self):
        self.compile_and_run(
            "test/n32g430_register_model.c", "n32g430_register_model",
            ["test", "test/c5_levelboard_stubs", "src", "src/stm32"])

    def test_virtual_endstop_does_not_pull_up_boot0(self):
        self.compile_and_run(
            "test/c5_virtual_endstop.c", "c5_levelboard_virtual_endstop",
            ["test/c5_virtual_endstop_stubs", "src"],
            ["-DTEST_LEVELBOARD=1"])


if __name__ == "__main__":
    unittest.main()
