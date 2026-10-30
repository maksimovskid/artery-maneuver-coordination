"""Exercise the real CI entry point with small, injected unittest suites."""
import subprocess
import sys
import unittest


class CiExitStatusTest(unittest.TestCase):
    def test_result_file_enum_remains_string_compatible(self):
        from tools.ci.sim_results import SimRecordedData
        self.assertEqual(SimRecordedData.File.SCALAR, 'scalar')
        self.assertEqual(str(SimRecordedData.File.VECTOR), 'vector')

    def run_case(self, behavior):
        code = '''
import runpy
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import tools.ci.artery_test_case
import tools.ci.test_options_loader

class Fixture(unittest.TestCase):
    def test_fixture(self):
        print("fixture output preserved", flush=True)
        if BEHAVIOR == "failure":
            self.fail("intentional assertion failure")
        if BEHAVIOR == "error":
            raise RuntimeError("intentional test error")

with patch("tools.ci.test_options_loader.ArteryTestConfigLoader") as loader, \\
     patch("tools.ci.artery_test_case.ArteryTestFactory.make", return_value=Fixture):
    loader.return_value.scenarios.return_value = ["artery"]
    loader.return_value.test_config.return_value = SimpleNamespace(
        test_runner=SimpleNamespace(tested_configurations=["fixture"]))
    if BEHAVIOR == "exception":
        loader.return_value.scenarios.side_effect = RuntimeError("unexpected loader exception")
    sys.argv = ["tools/ci/run.py", "-s", "scenarios", "-l", "build"]
    runpy.run_module("tools.ci.run", run_name="__main__")
'''.replace('BEHAVIOR', repr(behavior))
        return subprocess.run([sys.executable, '-c', code], capture_output=True,
                              text=True, timeout=30)

    def test_success_exits_zero_and_preserves_output(self):
        result = self.run_case('success')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('fixture output preserved', result.stdout)
        self.assertIn('OK', result.stderr)

    def test_assertion_failure_exits_nonzero_and_preserves_output(self):
        result = self.run_case('failure')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('intentional assertion failure', result.stderr)
        self.assertIn('FAILED', result.stderr)
        self.assertIn('fixture output preserved', result.stdout)

    def test_test_error_exits_nonzero(self):
        result = self.run_case('error')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('intentional test error', result.stderr)
        self.assertIn('FAILED', result.stderr)

    def test_unexpected_exception_exits_nonzero(self):
        result = self.run_case('exception')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('unexpected loader exception', result.stderr)
