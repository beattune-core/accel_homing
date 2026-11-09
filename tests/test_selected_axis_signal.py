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
