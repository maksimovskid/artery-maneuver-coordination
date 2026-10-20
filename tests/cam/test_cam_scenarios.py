"""Opt-in CAM+MCM integration; reuses the full merging protocol assertions."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'mcm'))
from scenario_runner import run_scenario
from test_mcm_scenarios import CoordinatedMergingTest as _MergingAssertions


class AdaptiveCamCoexistenceTest(_MergingAssertions):
    def run_checked_scenario(self, config):
        run = run_scenario(config, extra_args=(
            '--**.CaService.newGenCamRulesMyNegotiation=true',
            '--**.CaService.logAdaptiveGeneration=true',
        ) + (('--**.CaService.withDccRestriction=false',)
             if getattr(self, 'disable_cam_dcc', False) else ()))
        log = run.parsed_log
        try:
            self.check_trajectory_timing(log)
            configs = log.find('CAM-ADAPT-CONFIG')
            self.assertTrue(configs)
            self.assertTrue(all(e.fields['channel'] == '180' for e in configs))
            self.assertTrue(all(e.fields['cbrSource'] != 'unavailable' for e in configs))
            changes = log.find('CAM-ADAPT')
            self.assertTrue(any(e.fields['reasons'] == '1' for e in changes))
            self.assertTrue(any(e.fields['reasons'] == '0' for e in changes))
            generated = log.find('CAM-GENERATION')
            self.assertTrue(generated)
            for event in generated:
                if event.fields['reasons'] != '0':
                    self.assertGreaterEqual(float(event.fields['elapsed']), 1.0)
            # A sub-second own-negotiation window suppresses normal generation
            # until at least one second since the previous CAM or mode exit.
            starts = [e for e in changes if e.fields['reasons'] == '1']
            suppressed = False
            for start in starts:
                station = start.fields['station']; t = float(start.fields['time'])
                stops = [float(e.fields['time']) for e in changes
                         if e.fields['station'] == station and e.fields['reasons'] == '0'
                         and float(e.fields['time']) > t]
                self.assertTrue(stops, f'negotiation mode must clear for station {station}')
                stop = min(stops)
                before = [float(e.fields['time']) for e in generated
                          if e.fields['station'] == station and float(e.fields['time']) < t]
                during = [e for e in generated if e.fields['station'] == station
                          and t <= float(e.fields['time']) < stop]
                if before and stop - t >= .19 and not during:
                    suppressed = True
            self.assertTrue(suppressed, 'own negotiation must actually suppress CAM generation')
        except Exception:
            print(f'CAM integration log preserved at {run.log_path}', flush=True)
            raise
        return run

    def test_own_negotiation_without_facilities_dcc(self):
        self.disable_cam_dcc = True
        self.test_coordinated_merging_protocol_and_completion()

# Avoid unittest also discovering the imported assertion base as a second run.
del _MergingAssertions



class CamBaselineCompatibilityTest(unittest.TestCase):
    def test_fixed_rate_and_configured_periodic_policies(self):
        cases = (
            ('SameAsCAM', 'true', 0.1),
            ('PeriodicFixed1Hz', 'true', 1.0),
            ('PeriodicFixed2Hz', 'false', 0.5),
        )
        for policy, fixed, expected in cases:
            with self.subTest(policy=policy):
                with run_scenario('envmod-19CAVs-merging', time_limit='8s', extra_args=(
                    f'--**.CaService.triggeringCondition="{policy}"',
                    f'--**.CaService.fixedRate={fixed}',
                    '--**.CaService.withDccRestriction=false',
                    '--**.CaService.logAdaptiveGeneration=true',
                )) as run:
                    events = run.parsed_log.find('CAM-GENERATION')
                    self.assertGreaterEqual(len(events), 2)
                    self.assertFalse(run.parsed_log.find('CAM-ADAPT'))
                    seen = set()
                    checked = 0
                    for event in events:
                        station = event.fields['station']
                        if station in seen:
                            self.assertAlmostEqual(float(event.fields['elapsed']), expected, places=8)
                            checked += 1
                        seen.add(station)
                    self.assertGreater(checked, 0)


if __name__ == '__main__':
    unittest.main()
