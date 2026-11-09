"""Calibration pre-pass selects the impact metric from moving cross-axis noise."""
import math
import unittest

from harness import CommandError
from test_beta_calibration import BetaRig
from test_workflows import GCmd


class NoiseRig(BetaRig):
    """BetaRig whose central-stroke rows carry controlled per-axis noise."""
    def __init__(self, noise=(.2, 0.), options=None):
        super().__init__(options)
        self.noise = noise
        self.sessions = []
        self.on_home = lambda axis: self.sessions.append(self.workflow.session)

    def stop_motion(self, token, check):
        check()
        start, end = token["capture_cutoff_time_s"], token["motion_end_time_s"]
        x, y = self.noise
        rows = [(start + number / 1000., (x if number % 2 else -x), (y if number % 2 else -y), 1.)
                for number in range(1, math.ceil((end - start) * 1000.) + 2)]
        return {"samples": rows, "baseline_g": (0., 0., 1.), "sample_rate_hz": 1000.,
                "capture_cutoff_time_s": start, "sensor_context": self.sensor_context(),
                "raw_samples_validated": True, "dropped": 0, "fault": None}

    def central_moves(self):
        return [event for event in self.beta_events if event[0] == "central_move"]


class SelectedAxisCalibrationTests(unittest.TestCase):
    def test_cross_axis_noise_selects_selected_axis_and_seeds_from_axis_peak(self):
        rig = NoiseRig(noise=(3.1, 1.4))
        rig.home_results.extend([{"impact_metric": "selected_axis"}] * 11)
        rig.workflow.cmd_CALIBRATE(GCmd(AXIS="Y", CENTERED=1, SAVE_CONFIG=0))
        session = rig.sessions[0]
        self.assertEqual(session["impact_metric"], "selected_axis")
        self.assertEqual(session["metric_selection"]["source"], "moving_noise")
        self.assertAlmostEqual(session["metric_selection"]["axis_peak_g"], 1.4)
        self.assertIsNone(session["metric_selection"]["reason"])
        self.assertEqual(session["start_threshold_g"], 1.75)
        self.assertEqual(rig.home_calls[0]["threshold"], 1.75)
        self.assertTrue(rig.central_moves())

    def test_moving_seed_replaces_higher_stationary_seed(self):
        rig = NoiseRig(noise=(3.1, 1.4))
        rig.amplitudes = (1.5, .005, .002)
        rig.home_results.extend([{"impact_metric": "selected_axis"}] * 11)
        rig.workflow.cmd_CALIBRATE(GCmd(AXIS="Y", CENTERED=1, SAVE_CONFIG=0))
        session = rig.sessions[0]
        self.assertGreater(session["noise_seed"]["start_threshold_g"], 1.75)
        self.assertEqual(session["impact_metric"], "selected_axis")
        self.assertEqual(session["start_threshold_g"], 1.75)
        self.assertEqual(rig.home_calls[0]["threshold"], 1.75)

    def test_axis_dominated_noise_keeps_magnitude(self):
        rig = NoiseRig(noise=(3.1, .4))
        rig.workflow.cmd_CALIBRATE(GCmd(AXIS="X", CENTERED=1, SAVE_CONFIG=0))
        session = rig.sessions[0]
        self.assertEqual(session["impact_metric"], "xyz_magnitude")
        self.assertEqual(session["metric_selection"]["source"], "moving_noise")
        self.assertAlmostEqual(session["metric_selection"]["magnitude_peak_g"], math.hypot(3.1, .4))
        self.assertGreater(session["metric_selection"]["ratio"], .7)
        self.assertEqual(session["start_threshold_g"], 4.0)

    def test_prepass_unavailable_falls_back_with_warning(self):
        rig = NoiseRig()
        del rig.objects["motion_report"]
        rig.workflow.cmd_CALIBRATE(GCmd(AXIS="X", CENTERED=1, SAVE_CONFIG=0))
        session = rig.sessions[0]
        self.assertEqual(session["impact_metric"], "xyz_magnitude")
        self.assertEqual(session["metric_selection"]["source"], "skipped")
        self.assertIn("trapq extractor", session["metric_selection"]["reason"])
        self.assertTrue(any("cross-axis" in line for line in rig.output))
        self.assertEqual(rig.central_moves(), [])
        self.assertTrue(rig.home_calls)

    def test_cancel_during_prepass_aborts_and_restores(self):
        rig = NoiseRig(noise=(3.1, 1.4))
        original = rig.toolhead.manual_move
        seen = []
        def moving(coordinates, speed):
            value = original(coordinates, speed)
            session = rig.workflow.session
            if session and session.get("beta_phase") == "motion_noise" and not seen:
                seen.append(session)
                rig.workflow.cmd_CANCEL(GCmd())
            return value
        rig.toolhead.manual_move = moving
        with self.assertRaises(CommandError):
            rig.workflow.cmd_CALIBRATE(GCmd(AXIS="Y", CENTERED=1, SAVE_CONFIG=0))
        self.assertTrue(seen)
        self.assertNotIn("impact_metric", seen[0])
        self.assertEqual(rig.home_calls, [])
        self.assertEqual(rig.toolhead.accel, 50000.)
        self.assertEqual(rig.toolhead.kin.limits[:2], [(1., -1.), (1., -1.)])
        self.assertIsNone(rig.workflow.session)

    def test_resume_keeps_metric_without_rerunning_prepass(self):
        rig = NoiseRig(noise=(3.1, 1.4))
        rig.on_home = None
        rig.wait_calibration("Y", save=0)
        session = rig.workflow.session
        session.update(impact_metric="selected_axis", metric_selection={
            "source": "moving_noise", "axis_peak_g": 1.4, "magnitude_peak_g": 3.4,
            "ratio": 1.4 / 3.4, "reason": None})
        rig.home_results.extend([{"impact_metric": "selected_axis"}] * 11)
        seen = []
        rig.on_home = lambda axis: seen.append(rig.workflow.session.get("impact_metric"))
        rig.workflow.cmd_CALIBRATE(GCmd(AXIS="Y", CENTERED=1, SAVE_CONFIG=0))
        self.assertEqual(rig.central_moves(), [])
        self.assertTrue(seen)
        self.assertEqual(set(seen), {"selected_axis"})

    def test_selected_axis_calibration_homes_use_selected_detector(self):
        rig = NoiseRig(noise=(3.1, 1.4))
        rig.home_results.extend([{"impact_metric": "selected_axis"}] * 11)
        checks = []
        def observe(axis):
            if checks:
                return
            _center, expected = rig.workflow._geometry(axis)
            base = {"result": "impact", "travel_mm_exact": expected, "travel_mm": expected}
            checks.append((rig.workflow.simple_detection(axis),
                           rig.workflow._quality(axis, dict(base, impact_metric="selected_axis", moving=True)),
                           rig.workflow._quality(axis, dict(base, impact_metric="selected_axis")),
                           rig.workflow._quality(axis, dict(base, impact_metric="xyz_magnitude"))))
        rig.on_home = observe
        rig.workflow.cmd_CALIBRATE(GCmd(AXIS="Y", CENTERED=1, SAVE_CONFIG=0))
        simple, moving, unproven, other = checks[0]
        self.assertFalse(simple)
        self.assertEqual(moving[0], "good")
        self.assertEqual(unproven, ("fatal", "accelerometer did not establish motion before impact"))
        self.assertEqual(other[0], "fatal")


if __name__ == "__main__":
    unittest.main()
