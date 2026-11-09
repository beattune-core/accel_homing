"""Selected-axis signal helpers; all data is synthetic."""
import unittest

from test_accuracy_charts import charts


def make_capture():
    rows = []
    for ms in range(10000, 10600):
        t = ms / 1000.0
        y = 0.1 + (1.3 if ms % 2 else -1.3)
        if ms == 10498:
            y = 0.1 + 5.0
        elif ms == 10510:
            y = 0.1 + 2.4
        elif ms == 10540:
            y = 0.1 + 8.0
        elif 10500 <= ms < 10530 and ms != 10510:
            y = 0.1
        elif ms >= 10530 and ms != 10540:
            y = 0.1
        rows.append((t, 0.0, y, 0.0))
    return {"samples": rows, "baseline_g": (0.0, 0.1, 0.0),
            "detector": {"arm_time_s": 10.0, "onset_time_s": 10.5}}


class SelectedAxisSignalTests(unittest.TestCase):
    def test_honserus_y_selects_axis_and_x_keeps_magnitude(self):
        self.assertEqual(charts.choose_impact_metric(1.43, 3.30), "selected_axis")
        self.assertEqual(charts.choose_impact_metric(3.10, 3.20), "xyz_magnitude")
        self.assertEqual(charts.choose_impact_metric(0.0, 0.0), "xyz_magnitude")

    def test_motion_noise_peaks_ignores_braking(self):
        stats = {"seed_magnitude_peak_g": 3.3, "phases": [
            {"seed_eligible": True, "axis_absolute_peak_g": {"x": 3.1, "y": 1.2, "z": 0.4}},
            {"seed_eligible": True, "axis_absolute_peak_g": {"x": 2.0, "y": 1.4, "z": 0.3}},
            {"seed_eligible": False, "axis_absolute_peak_g": {"x": 9.0, "y": 9.0, "z": 9.0}}]}
        self.assertEqual(charts.motion_noise_peaks(stats, "y"), (1.4, 3.3))

    def test_axis_check_peaks_windows(self):
        noise, impact = charts.axis_check_peaks(make_capture(), "y")
        self.assertAlmostEqual(noise, 1.3)
        self.assertAlmostEqual(impact, 2.4)

    def test_first_contact_peak_ignores_the_later_bounce(self):
        capture = make_capture()
        capture["samples"] = [(t, x, y + (2.1 if abs(t - 10.5) < 1e-9 else 0.), z)
                              for t, x, y, z in capture["samples"]]
        self.assertAlmostEqual(charts.axis_first_contact_peak(capture, "y"), 2.1)
        _noise, post_trigger = charts.axis_check_peaks(capture, "y")
        self.assertGreater(post_trigger, 2.1)

    def test_first_contact_window_is_one_ms_inclusive_of_onset(self):
        capture = {"samples": [(10.000, 0., 0.2, 0.), (10.001, 0., 1.0, 0.),
                               (10.002, 0., 9.0, 0.)],
                   "baseline_g": (0., 0.2, 0.), "detector": {"onset_time_s": 10.000}}
        self.assertAlmostEqual(charts.axis_first_contact_peak(capture, "y"), 0.8)
        capture["detector"]["onset_time_s"] = None
        with self.assertRaises(ValueError):
            charts.axis_first_contact_peak(capture, "y")

    def test_threshold_is_capped_at_the_tested_candidate(self):
        r = charts.selected_axis_threshold([1.5], [2.5], 1.8)
        self.assertAlmostEqual(r["threshold_g"], 1.8)
        self.assertEqual(r["threshold_set_by"], "tested_cap")
        r = charts.selected_axis_threshold([1.5], [2.5], 2.4)
        self.assertAlmostEqual(r["threshold_g"], 2.0)
        self.assertEqual(r["threshold_set_by"], "midpoint")

    def test_cap_at_or_below_noise_max_is_refused(self):
        with self.assertRaisesRegex(ValueError, "1.7.*1.9"):
            charts.selected_axis_threshold([1.9], [3.0], 1.75)
        with self.assertRaises(ValueError):
            charts.selected_axis_threshold([1.9], [3.0], 1.9)

    def test_axis_check_peaks_refuses_missing_windows(self):
        capture = make_capture()
        capture["detector"]["onset_time_s"] = None
        with self.assertRaises(ValueError):
            charts.axis_check_peaks(capture, "y")
        capture = make_capture()
        capture["detector"]["arm_time_s"] = 20.0
        capture["detector"]["onset_time_s"] = 20.5
        with self.assertRaises(ValueError):
            charts.axis_check_peaks(capture, "y")

    def test_selected_axis_threshold_midpoint_and_refusal(self):
        r = charts.selected_axis_threshold([1.20, 1.43, 1.30], [2.64, 2.21, 2.50])
        self.assertAlmostEqual(r["threshold_g"], 1.82)
        self.assertAlmostEqual(r["window_g"], 0.78)
        with self.assertRaisesRegex(ValueError, "1.8.*2.1"):
            charts.selected_axis_threshold([1.8], [2.1])


if __name__ == "__main__":
    unittest.main()
