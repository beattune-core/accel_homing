"""Calibration pre-pass selects the impact metric from moving cross-axis noise."""
import json
import math
import unittest

from harness import CommandError
from test_beta_calibration import BetaRig
from test_workflows import GCmd, Rig, workflow_module

MODEL = "accel_homing_model default"


class NoiseRig(BetaRig):
    """BetaRig whose central-stroke rows carry controlled per-axis noise."""
    def __init__(self, noise=(.2, 0.), options=None):
        super().__init__(options)
        self.noise = noise
        self.sessions = []
        self.on_home = lambda axis: self.sessions.append(self.workflow.session)
        # (pre-impact noise peak, impact peak) on the homing axis, cycled per capture.
        self.axis_peaks = [(1., 3.)]

    def stop_capture(self):
        capture = super().stop_capture()
        detector = capture.get("detector")
        if not detector:
            return capture
        axis = detector["axis"]
        noise, impact = self.axis_peaks[(self.capture_stops - 1) % len(self.axis_peaks)]
        t0 = capture["samples"][0][0]
        rows = []
        for number in range(41):
            value = [0., 0., 1.]
            if number == 3:
                value["xy".index(axis)] = noise
                value["yx".index(axis)] = 3.1
            elif number == 12:
                value["xy".index(axis)] = impact
            rows.append((t0 + number / 1000.,) + tuple(value))
        capture["samples"] = rows
        detector.update(arm_time_s=t0 + .001, onset_time_s=t0 + .010)
        return capture

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



def selected_rig(peaks):
    rig = NoiseRig(noise=(3.1, 1.4))
    rig.axis_peaks = peaks
    rig.home_results.extend([{"impact_metric": "selected_axis"}] * 11)
    return rig


def model_options(rig):
    return {key: value for (section, key), value in rig.configfile.saved.items() if section == MODEL}


def reload(options, manual=None):
    return Rig(manual, sections={"printer": {"kinematics": "corexy"}, MODEL: options})


class SelectedAxisModelTests(unittest.TestCase):
    def calibrated_y(self):
        rig = selected_rig([(1.43, 3.), (1., 2.21)])
        rig.workflow.cmd_CALIBRATE(GCmd(AXIS="Y", CENTERED=1, SAVE_CONFIG=0))
        return rig

    def test_selected_axis_model_saves_window_midpoint(self):
        rig = self.calibrated_y()
        md = json.loads(rig.configfile.saved[(MODEL, "y_metadata")])
        self.assertEqual(md["impact_metric"], "selected_axis")
        self.assertEqual(md["polarity_policy"], workflow_module.POLARITY_POLICY)
        self.assertAlmostEqual(float(rig.configfile.saved[(MODEL, "y_threshold")]), 1.82)
        self.assertEqual(md["threshold_rule"], "window_midpoint")
        self.assertIsNone(md["threshold_margin_g"])
        self.assertEqual(md["metric_selection"]["source"], "moving_noise")
        self.assertAlmostEqual(md["selected_axis_window"]["noise_max_g"], 1.43)
        self.assertAlmostEqual(md["selected_axis_window"]["impact_min_g"], 2.21)
        self.assertAlmostEqual(md["selected_axis_window"]["window_g"], .78)
        self.assertAlmostEqual(rig.workflow.last_result["saved_threshold_g"], 1.82)
        self.assertTrue(any("window midpoint" in line for line in rig.output))

    def test_narrow_window_refuses_to_save(self):
        rig = selected_rig([(1.8, 3.), (1., 2.1)])
        with self.assertRaises(CommandError) as raised:
            rig.workflow.cmd_CALIBRATE(GCmd(AXIS="Y", CENTERED=1, SAVE_CONFIG=0))
        message = str(raised.exception)
        for number in ("1.800", "2.100", "0.300", "0.500"):
            self.assertIn(number, message)
        self.assertEqual(rig.configfile.saved, {})

    def test_saved_selected_axis_model_routes_to_selected_detector_on_ordinary_home(self):
        reloaded = reload(model_options(self.calibrated_y()))
        reloaded.workflow._validate_model_connection()
        self.assertFalse(reloaded.workflow.simple_detection("y"))
        self.assertAlmostEqual(reloaded.owner.threshold_g["y"], 1.82)
        reloaded.workflow.check_home_axes(["y"])

    def test_existing_xyz_model_and_manual_threshold_route_unchanged(self):
        xyz = Rig()
        xyz.calibrate("X", save=0)
        options = dict(model_options(xyz), **model_options(self.calibrated_y()))
        reloaded = reload(options, {"y_threshold": 6.0})
        reloaded.workflow._validate_model_connection()
        self.assertTrue(reloaded.workflow.simple_detection("x"))
        self.assertEqual(reloaded.owner.threshold_g["x"], 1.5)
        self.assertFalse(reloaded.workflow.simple_detection("y"))
        # The rig's core keeps its own manual value; the saved Y model must not replace it.
        self.assertIn("y", reloaded.workflow.manual_axes)
        self.assertNotAlmostEqual(reloaded.owner.threshold_g["y"], 1.82)

    def test_xyz_calibration_still_saves_tested_plus_half_g(self):
        rig = Rig()
        rig.calibrate("X", save=0)
        md = json.loads(rig.configfile.saved[(MODEL, "x_metadata")])
        self.assertEqual(rig.configfile.saved[(MODEL, "x_threshold")], "1.5")
        self.assertEqual(md["impact_metric"], "xyz_magnitude")
        self.assertEqual(md["threshold_rule"], "tested_plus_margin")
        self.assertEqual(md["threshold_margin_g"], .5)
        self.assertEqual(md["tested_threshold_g"], 1.)

    def test_xyz_calibration_records_homing_axis_peaks_as_diagnostics(self):
        rig = NoiseRig(noise=(3.1, .4))
        rig.axis_peaks = [(.6, 2.5)]
        rig.workflow.cmd_CALIBRATE(GCmd(AXIS="X", CENTERED=1, SAVE_CONFIG=0))
        md = json.loads(rig.configfile.saved[(MODEL, "x_metadata")])
        self.assertEqual(md["impact_metric"], "xyz_magnitude")
        self.assertEqual(set(md["axis_noise_peak_g"]), {.6})
        self.assertEqual(set(md["axis_impact_peak_g"]), {2.5})
        self.assertEqual(rig.configfile.saved[(MODEL, "x_threshold")],
                         "%.17g" % (md["tested_threshold_g"] + .5))


if __name__ == "__main__":
    unittest.main()
