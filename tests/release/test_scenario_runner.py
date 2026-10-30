"""Release harness configuration and wall-clock cleanup, without real simulations."""
import contextlib
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from tools.run_artery import positive_timeout, run_simulation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'mcm'))
import scenario_runner


class ScenarioRunnerTest(unittest.TestCase):
    def test_build_path_override_and_timeout_diagnostics(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch.dict(os.environ, ARTERY_BUILD_DIR='build/Release', ARTERY_TEST_TIMEOUT='600'), \
             patch.object(scenario_runner.tempfile, 'mkdtemp', return_value=directory), \
             patch.object(scenario_runner, 'run_simulation',
                          side_effect=subprocess.TimeoutExpired('simulation', 600)) as run:
            with self.assertRaisesRegex(AssertionError, 'fixture exceeded wall-clock timeout of 600s') as error:
                scenario_runner.run_scenario('fixture')
            self.assertIn('ARTERY_TEST_TIMEOUT', str(error.exception))
            self.assertTrue((Path(directory) / 'simulation.log').exists())
            self.assertEqual(run.call_args.args[0][2], str(scenario_runner.REPO_ROOT / 'build/Release'))
            self.assertEqual(run.call_args.kwargs['timeout'], 600)

    def test_capture_timeout_preserves_log_and_returns_nonzero(self):
        from tools.animation import capture_scenario
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(capture_scenario, 'run_simulation',
                          side_effect=subprocess.TimeoutExpired('capture', 600)) as run:
            errors = io.StringIO()
            with contextlib.redirect_stderr(errors):
                code = capture_scenario.run_capture(
                    'fixture', '0', '30s', Path(directory),
                    build_dir='/tmp/custom-build', wall_clock_timeout=600,
                )
            self.assertEqual(code, 124)
            self.assertIn('exceeded wall-clock timeout of 600s', errors.getvalue())
            self.assertIn('--wall-clock-timeout', errors.getvalue())
            self.assertTrue((Path(directory) / 'simulation.log').exists())
            self.assertEqual(run.call_args.args[0][2], '/tmp/custom-build')

    def test_rejects_invalid_timeout(self):
        for value in ('0', '-1', 'nan', 'inf'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                positive_timeout(value)

    def test_successful_process_exit_code(self):
        self.assertEqual(run_simulation([sys.executable, '-c', 'raise SystemExit(7)'], timeout=10), 7)

    @unittest.skipUnless(sys.platform.startswith('linux'), 'descendant check uses Linux /proc')
    def test_timeout_kills_simulation_descendants(self):
        with tempfile.TemporaryDirectory() as directory:
            pid_file = Path(directory) / 'child.pid'
            inner_code = '''
import pathlib, subprocess, sys, time
child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
pathlib.Path(sys.argv[1]).write_text(str(child.pid))
print('simulation started', flush=True)
time.sleep(60)
'''
            # Match the MCM harness: timed wrapper -> untimed launcher -> simulator/SUMO.
            code = ("import sys; from tools.run_artery import run_simulation; "
                    f"raise SystemExit(run_simulation([sys.executable, '-c', {inner_code!r}, sys.argv[1]]))")
            log_path = Path(directory) / 'simulation.log'
            with log_path.open('w') as log:
                with self.assertRaises(subprocess.TimeoutExpired):
                    run_simulation([sys.executable, '-c', code, str(pid_file)],
                                   timeout=5, stdout=log, stderr=subprocess.STDOUT)
            self.assertIn('simulation started', log_path.read_text())
            child_pid = int(pid_file.read_text())
            # A killed orphan may briefly remain a zombie until reaped by init.
            stat = Path(f'/proc/{child_pid}/stat')
            deadline = time.monotonic() + 5
            while True:
                try:
                    state = stat.read_text().split(') ')[1].split()[0]
                except FileNotFoundError:
                    break
                if state == 'Z':
                    break
                self.assertLess(time.monotonic(), deadline, 'simulation descendant survived timeout')
                time.sleep(0.01)
