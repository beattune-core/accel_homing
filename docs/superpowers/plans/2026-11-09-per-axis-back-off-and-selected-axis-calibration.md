# Per-axis Back-off and Selected-axis Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Per-axis back-off distances with Kalico retract semantics, and a calibration that picks the homing-axis-only (selected-axis) detector when cross-axis noise swamps the XYZ magnitude.

**Architecture:** `back_off_dist` becomes a per-axis dict on the owner (like `threshold_g`). Calibration reuses the Beta central-stroke moving-noise pre-pass to choose the impact metric, runs the existing threshold grid with that detector, and saves selected-axis models with a window-midpoint threshold. Selected-axis homes wait for a quiet standstill reference. Pure signal helpers live in `accel_homing_charts.py` (next to `beta_motion_statistics`) so no new module has to be registered with installer/bootstrap/uninstaller.

**Tech Stack:** Python 3 (Klipper/Kalico klippy plugin), `unittest` offline suite with stub printers (`tests/harness.py`, `tests/test_workflows.py` `Rig`, `tests/test_beta_calibration.py` `BetaRig`).

**Spec:** `docs/superpowers/specs/2026-11-09-per-axis-back-off-and-selected-axis-calibration-design.md`

## Global Constraints

- Branch `feature/per-axis-backoff-selected-axis-calibration` in `D:\Workspace\beattune-core\accel_homing` (fork `beattune-core/accel_homing`, based on `dev`).
- Run tests on Linux (WSL), never judge by the Windows run:
  `MSYS_NO_PATHCONV=1 wsl.exe -d Ubuntu-24.04 --cd /mnt/d/Workspace/beattune-core/accel_homing -- bash -c '~/.venvs/accel_homing/bin/python -B -m unittest discover -s tests -p "<PATTERN>" -v'`
  (below: `wsltest <PATTERN>`; full suite = pattern `test*.py`).
- Baseline before any change: 1023 tests, **7 failures, 11 errors**; the exact list is in `C:\Users\TKram\AppData\Local\Temp\claude\80ab4aee1413391b\fb688524-b700-4dcf-9611-41a9c5804a29\scratchpad\baseline_failures.txt`. A task is green when it adds no failure/error outside that list.
- `back_off_dist`, `x_back_off_dist`, `y_back_off_dist`: default 5.0, `minval=0.0`, no maximum; per-axis falls back to `back_off_dist`.
- Out-of-range back-off target raises `command_error` (Kalico-like), never clamps.
- Selected-axis decision: ratio homing-axis peak / XYZ-magnitude peak `< 0.7` → `"selected_axis"`, else `"xyz_magnitude"`.
- Selected-axis saved threshold = midpoint of (max pre-impact homing-axis noise peak .. min homing-axis impact peak) over the ten checks; refuse to save when window `< 0.5` g. XYZ models keep the `+0.5 g` margin unchanged.
- Impact peak window: 30 ms from onset; noise window: arm time to 3 ms before onset; reference = `capture["baseline_g"]`.
- No new Python module files. Tests may be added (repo exception); they must not enumerate unrelated options or assert counts/positions of unrelated data.
- Commits end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. No pushes until Task 7.
- Printer steps (Task 7) are done together with the user, one checkpoint at a time; motion only after the user confirms XY is prepared; Claude never reboots the printer host.

## Review Focus

1. A config with only the legacy `back_off_dist` (no per-axis keys) behaves exactly as before for both axes in range → test in Task 1.
2. `back_off_dist: 0` plus `y_back_off_dist: 50` → X disabled, Y 50 (a per-axis value overrides a shared zero) → test in Task 1.
3. A calibration resumed from `state: waiting` keeps the chosen metric and does not rerun the pre-pass → test in Task 3.
4. Existing saved XYZ models and manual thresholds route exactly as before (honserus X model) → test in Task 4.
5. Cancel/motor-off during the pre-pass aborts calibration (no silent magnitude fallback) and restores accel/limits like Beta → test in Task 3.

---

### Task 1: Per-axis back-off with Kalico out-of-range semantics

**Files:**
- Modify: `beacon_accel_homing.py` (option parsing ~541–557; `_retract_completed_home` ~736–790; `release_workflow_home` ~792; `_back_off_others` ~1090–1113; status dict ~3333)
- Modify: `accel_homing_workflows.py` (`saved_back_off`, `owner.back_off_dist = 0.0`, snapshots: ~598, 613, 885, 987, 1332, 1453, 2576)
- Modify: `accel_homing_center.py` (~447, 476, 510), `accel_homing_travel.py` (~64, 208)
- Modify: `docs/CONFIGURATION.md` (~113, ~129), `docs/CENTER.md` (~40)
- Modify: `tests/test_retract_noise.py` (`test_both_directions_bounds_and_existing_kalico_retract`, `test_zero_disables...`)
- Create: `tests/test_back_off_per_axis.py`

**Interfaces:**
- Produces: `owner.back_off_dist: dict[str, float]` keyed `"x"`, `"y"`, `"z"` (z present for symmetry with `AXES`, unused). Every consumer indexes by axis. `owner.release_workflow_home(axis, source_record, distance: float, session)` keeps its float `distance` (callers pass `saved_back_off[axis]`). Workflow sessions store `saved_back_off` as `dict(owner.back_off_dist)`; suppression sets `owner.back_off_dist = {a: 0.0 for a in AXES}`.

- [ ] **Step 1: Write failing tests** in `tests/test_back_off_per_axis.py` (reuse `FinalRetractTests.machine` from `test_retract_noise` by subclassing; build configs with `harness.make_homing(options=...)`/`generic(options=...)` as existing tests do):

```python
def test_per_axis_options_fall_back_to_shared(self):
    obj = make_with_options({"back_off_dist": "7"})
    self.assertEqual((obj.back_off_dist["x"], obj.back_off_dist["y"]), (7.0, 7.0))
    obj = make_with_options({"back_off_dist": "0", "y_back_off_dist": "50"})
    self.assertEqual((obj.back_off_dist["x"], obj.back_off_dist["y"]), (0.0, 50.0))
    obj = make_with_options({})
    self.assertEqual((obj.back_off_dist["x"], obj.back_off_dist["y"]), (5.0, 5.0))

def test_large_values_accept_and_negative_rejects(self):
    self.assertEqual(make_with_options({"y_back_off_dist": "250"}).back_off_dist["y"], 250.0)
    with self.assertRaises(ConfigError):
        make_with_options({"x_back_off_dist": "-1"})

def test_final_release_uses_axis_distance(self):          # machine("y"), back_off y=50, bounds (0, 329)
    # positive_dir=True, endstop 329 -> move to 279.0 at retract speed
    self.assertEqual(obj.toolhead.moves, [([None, 279.0, None], 100.0)])

def test_out_of_range_target_raises_like_kalico(self):    # bounds (0, 30), y_back_off_dist 50
    with self.assertRaisesRegex(CommandError, "out of range"):
        kin.home(object())
    self.assertEqual(obj.toolhead.moves, [])

def test_cross_axis_release_uses_homed_axis_distance(self):
    # Y homed at its stop with y_back_off_dist 50; homing X first moves Y to 279.0
    ...
    self.assertEqual(obj.toolhead.moves[0], ([None, 279.0, None], 100.0))

def test_workflow_suppresses_and_restores_per_axis(self):
    # after a calibration/center workflow ends: owner.back_off_dist == {"x": 5.0, "y": 50.0, "z": 5.0}
```

Also update `test_both_directions_bounds_and_existing_kalico_retract`: the two cases with bounds `(0.0, 3.0)` must now `assertRaisesRegex(CommandError, "out of range")` instead of expecting a clamped move; the in-range cases keep their expected targets.

- [ ] **Step 2: Run** `wsltest test_back_off_per_axis.py` and `wsltest test_retract_noise.py` → new tests FAIL (`TypeError`/`AttributeError` on dict access, missing error).

- [ ] **Step 3: Implement.**
  - Parsing: `shared = config.getfloat("back_off_dist", 5.0, minval=0.0)`; `self.back_off_dist = {a: config.getfloat("%s_back_off_dist" % a, shared, minval=0.0) for a in AXES}`. Finite check covers each value (extend `scalar_options` handling to the dict values with option names `x_back_off_dist` etc.).
  - `_retract_completed_home`: per-axis `distance = context.get("distance", self.back_off_dist[axis])`; when `distance` is 0 skip that axis. Compute `target = info.position_endstop + away * distance` **without** clamping; if `target < low - 1e-9 or target > high + 1e-9` raise `command_error("accel_homing: Move out of range: %s back-off target %.3f outside %.3f..%.3f" % (axis.upper(), target, low, high))`. Keep the "never move back toward the stop / already released" check.
  - `_back_off_others`: same per-axis distance and the same error instead of `min(max(...))`.
  - Workflows/center/travel: replace scalar save/suppress/restore/compare with dict copies as in Interfaces; `release_workflow_home(axis, ..., session["saved_back_off"][axis], session)`.
  - Status (~3333): report `"back_off_dist": dict(self.back_off_dist)`.
  - Docs: CONFIGURATION.md table rows for `back_off_dist` (shared fallback, ≥ 0, no maximum), `x_back_off_dist`/`y_back_off_dist` (override per axis), and the sentence that an out-of-range target fails G28 like Kalico's retract ("Move out of range") instead of being clamped. CENTER.md: "per-axis back-off values".

- [ ] **Step 4: Run** `wsltest test_back_off_per_axis.py`, `wsltest test_retract_noise.py`, `wsltest test_center_calibration.py`, `wsltest test_travel*.py`, `wsltest test_workflows.py` → PASS (workflows: only baseline-listed failures).

- [ ] **Step 5: Commit** `git commit -m "Add per-axis back-off distances with Kalico retract semantics"`

---

### Task 2: Pure selected-axis signal helpers

**Files:**
- Modify: `accel_homing_charts.py` (after `beta_motion_statistics`)
- Create: `tests/test_selected_axis_signal.py`

**Interfaces:**
- Produces (module-level in `accel_homing_charts.py`):
  - `SELECTED_AXIS_RATIO_LIMIT = 0.7`, `SELECTED_AXIS_MIN_WINDOW_G = 0.5`, `IMPACT_PEAK_WINDOW_S = 0.03`, `NOISE_ONSET_GUARD_S = 0.003`
  - `motion_noise_peaks(statistics: dict, axis: str) -> tuple[float, float]` → `(axis_peak_g, magnitude_peak_g)` from a `beta_motion_statistics` result: axis peak = max of `phases[i]["axis_absolute_peak_g"][axis]` over `seed_eligible` phases; magnitude = `seed_magnitude_peak_g`.
  - `choose_impact_metric(axis_peak_g: float, magnitude_peak_g: float) -> str` → `"selected_axis"` iff `magnitude_peak_g > 0` and `axis_peak_g / magnitude_peak_g < SELECTED_AXIS_RATIO_LIMIT`, else `"xyz_magnitude"`.
  - `axis_check_peaks(capture: dict, axis: str) -> tuple[float, float]` → `(noise_peak_g, impact_peak_g)` of `|a_axis − baseline_g[axis]|`, noise over `arm_time_s <= t < onset_time_s − NOISE_ONSET_GUARD_S`, impact over `onset_time_s <= t < onset_time_s + IMPACT_PEAK_WINDOW_S`, times from `capture["detector"]`; raises `ValueError` when either window has no samples or times are missing/non-finite.
  - `selected_axis_threshold(noise_peaks: list[float], impact_peaks: list[float]) -> dict` → `{"threshold_g", "noise_max_g", "impact_min_g", "window_g"}`; threshold = `(noise_max + impact_min) / 2`; raises `ValueError` with all three numbers in the message when `window_g < SELECTED_AXIS_MIN_WINDOW_G`.

- [ ] **Step 1: Write failing tests**

```python
def test_honserus_y_selects_axis_and_x_keeps_magnitude(self):
    self.assertEqual(choose_impact_metric(1.43, 3.30), "selected_axis")
    self.assertEqual(choose_impact_metric(3.10, 3.20), "xyz_magnitude")
    self.assertEqual(choose_impact_metric(0.0, 0.0), "xyz_magnitude")

def test_motion_noise_peaks_ignores_braking(self):
    stats = {"seed_magnitude_peak_g": 3.3, "phases": [
        {"seed_eligible": True,  "axis_absolute_peak_g": {"x": 3.1, "y": 1.2, "z": 0.4}},
        {"seed_eligible": True,  "axis_absolute_peak_g": {"x": 2.0, "y": 1.4, "z": 0.3}},
        {"seed_eligible": False, "axis_absolute_peak_g": {"x": 9.0, "y": 9.0, "z": 9.0}}]}
    self.assertEqual(motion_noise_peaks(stats, "y"), (1.4, 3.3))

def test_axis_check_peaks_windows(self):
    # baseline y=0.1; rows at 1 kHz: arm 10.000, onset 10.500; noise rows ±1.3 before 10.497,
    # a 5.0 spike at 10.498 (inside the 3 ms guard, must be ignored), impact 2.4 at 10.510,
    # 8.0 at 10.540 (outside 30 ms, ignored)
    self.assertEqual(axis_check_peaks(capture, "y"), (approx 1.3, approx 2.4))

def test_selected_axis_threshold_midpoint_and_refusal(self):
    r = selected_axis_threshold([1.20, 1.43, 1.30], [2.64, 2.21, 2.50])
    self.assertAlmostEqual(r["threshold_g"], 1.82); self.assertAlmostEqual(r["window_g"], 0.78)
    with self.assertRaisesRegex(ValueError, "1.8.*2.1"):
        selected_axis_threshold([1.8], [2.1])
```

- [ ] **Step 2: Run** `wsltest test_selected_axis_signal.py` → FAIL (ImportError).
- [ ] **Step 3: Implement** the four functions/constants with the signatures above.
- [ ] **Step 4: Run** `wsltest test_selected_axis_signal.py` and `wsltest test_absolute_charts.py` → PASS.
- [ ] **Step 5: Commit** `git commit -m "Add selected-axis noise, impact and threshold helpers"`

---

### Task 3: Calibration pre-pass chooses the impact metric

**Files:**
- Modify: `accel_homing_workflows.py` (`_run_calibration` ~2037; `simple_detection` ~834; `_quality` ~1602; early-capture check ~1424; new `_select_calibration_metric`)
- Create: `tests/test_selected_axis_calibration.py` (fixtures: subclass `BetaRig` from `tests/test_beta_calibration.py`)

**Interfaces:**
- Consumes: `motion_noise_peaks`, `choose_impact_metric` (Task 2); `_beta_motion_plan`, `_beta_motion_noise` (existing).
- Produces: `session["impact_metric"]: "xyz_magnitude" | "selected_axis"` set once per fresh calibration; `session["metric_selection"]: dict` with keys `source` (`"moving_noise"` | `"skipped"`), `axis_peak_g`, `magnitude_peak_g`, `ratio`, `reason` (skip reason or `None`). `WorkflowManager._select_calibration_metric(self, session) -> None`.

- [ ] **Step 1: Write failing tests** (BetaRig-based, moving-noise captures whose homing-axis vs magnitude peaks are controlled by the rig's synthetic rows):

```python
def test_cross_axis_noise_selects_selected_axis_and_seeds_from_axis_peak(self):
    # rig noise: y component 1.4 g, x component 3.1 g during central strokes; AXIS=Y
    rig.workflow.cmd_CALIBRATE(GCmd(AXIS="Y", CENTERED=1, SAVE_CONFIG=0))
    s = rig.workflow.session or rig.workflow.last_result
    self.assertEqual(session["impact_metric"], "selected_axis")
    self.assertEqual(session["metric_selection"]["source"], "moving_noise")
    self.assertEqual(session["start_threshold_g"], 1.75)   # max(1.2*1.4, 1.4+0.1)=1.68 -> grid 1.0+k*0.25

def test_axis_dominated_noise_keeps_magnitude(self):       # x 3.1, y 0.4, AXIS=X -> "xyz_magnitude"

def test_prepass_unavailable_falls_back_with_warning(self):
    # remove motion_report -> _beta_motion_plan raises before motion
    self.assertEqual(session["impact_metric"], "xyz_magnitude")
    self.assertEqual(session["metric_selection"]["source"], "skipped")
    self.assertTrue(any("cross-axis" in line for line in rig.output))

def test_cancel_during_prepass_aborts_and_restores(self):   # Review Focus 5
    # cancel/motor_off inside a central stroke -> calibration aborted, no homes,
    # accel and kin.limits restored as Beta guarantees, impact_metric never set to a fallback
    
def test_resume_keeps_metric_without_rerunning_prepass(self):  # Review Focus 3
    # session waiting after selection -> cmd_CALIBRATE resume -> no new central_move events,
    # impact_metric unchanged

def test_selected_axis_calibration_homes_use_selected_detector(self):
    # during a calibration home with impact_metric selected_axis:
    self.assertFalse(rig.workflow.simple_detection("y"))
    # and a selected-axis impact record with motion proof passes _quality as "good"
```

- [ ] **Step 2: Run** `wsltest test_selected_axis_calibration.py` → FAIL.
- [ ] **Step 3: Implement.**
  - `_run_calibration`: after `_calibration_noise_preflight(session)`, call `_select_calibration_metric(session)` only when `"impact_metric" not in session`; then `_calibrate_trials()`.
  - `_select_calibration_metric`: set `beta_generation`, `beta_motion_iterations=[]` on the session (do **not** set `beta=True`); `plan = _beta_motion_plan(session)` inside `try/except` for `self.printer.command_error` → on error: `impact_metric="xyz_magnitude"`, `metric_selection` with `source="skipped"`, `reason=str(exc)`, console line `"cross-axis noise check skipped: <reason>; calibrating with XYZ magnitude"`, return. Otherwise run `_beta_motion_noise(session, plan)` **without** catching (cancel/fault abort the calibration). Peaks: max over iterations of `motion_noise_peaks(item["record"], axis)` (each `record` already contains the `beta_motion_statistics` fields). Choose metric. Unless `session["explicit_start_threshold"]`, seed `candidate`/`start_threshold_g` from the chosen metric's peak with Beta's rule (`max(start, 1.2*peak, peak+0.1)` rounded up onto the grid; reuse/extract the arithmetic from `_beta_seed` into `_motion_seed(session, peak) -> float` and make `_beta_seed` call it). Console line naming metric, both peaks, ratio and start threshold. After the pre-pass call `self._require_unhomed_xy()` as Beta does.
  - `simple_detection(axis)`: in the calibration branch return `self.session.get("impact_metric", "xyz_magnitude") == "xyz_magnitude"`.
  - `_quality` (calibration branch) and the early-capture check: accept `record["impact_metric"]` / `capture["detector"]["impact_metric"]` equal to `session["impact_metric"]` (`"selected_axis"` records additionally need `moving` or `motion_seen`, else `"fatal", "accelerometer did not establish motion before impact"`).
- [ ] **Step 4: Run** `wsltest test_selected_axis_calibration.py`, `wsltest test_beta_calibration.py`, `wsltest test_simple_calibration.py`, `wsltest test_calibration*.py` → PASS (baseline-listed only).
- [ ] **Step 5: Commit** `git commit -m "Choose calibration impact metric from moving cross-axis noise"`

---

### Task 4: Save and use selected-axis models

**Files:**
- Modify: `accel_homing_workflows.py` (`_home` record enrichment ~1394–1440; `_save_model` metadata ~1915–1935; `_calibration_saved_threshold` ~2004)
- Modify: `tests/test_selected_axis_calibration.py`

**Interfaces:**
- Consumes: `axis_check_peaks`, `selected_axis_threshold` (Task 2); `session["impact_metric"]`, `session["metric_selection"]` (Task 3).
- Produces: validation records carry `axis_noise_peak_g`, `axis_impact_peak_g` (all calibrations; XYZ models save them as diagnostics only). Saved metadata for selected-axis models: `impact_metric: "selected_axis"`, `polarity_policy: POLARITY_POLICY`, `metric_selection` (Task 3 dict), `selected_axis_window: {noise_max_g, impact_min_g, window_g}`, `threshold_rule: "window_midpoint"`, `threshold_margin_g: None`. XYZ models unchanged (`threshold_rule: "tested_plus_margin"` added).

- [ ] **Step 1: Write failing tests**

```python
def test_selected_axis_model_saves_window_midpoint(self):
    # ten checks with y noise peaks max 1.43 and impact peaks min 2.21
    md = saved_metadata("y")
    self.assertEqual(md["impact_metric"], "selected_axis")
    self.assertEqual(md["polarity_policy"], POLARITY_POLICY)
    self.assertAlmostEqual(saved_threshold("y"), 1.82)
    self.assertEqual(md["threshold_rule"], "window_midpoint")

def test_narrow_window_refuses_to_save(self):
    # noise max 1.8, impact min 2.1 -> calibration aborted with the three numbers, nothing staged
    self.assertFalse(rig.configfile.staged)

def test_saved_selected_axis_model_routes_to_selected_detector_on_ordinary_home(self):
    # reload with that model, no manual threshold: simple_detection("y") is False,
    # threshold_g["y"] == 1.82, check_home_axes(["y"]) does not raise

def test_existing_xyz_model_and_manual_threshold_route_unchanged(self):   # Review Focus 4
    # xyz model on X -> simple_detection("x") True, threshold from model;
    # y_threshold in config -> manual, simple_detection("y") False, model ignored for Y

def test_xyz_calibration_still_saves_tested_plus_half_g(self):
```

- [ ] **Step 2: Run** `wsltest test_selected_axis_calibration.py` → new tests FAIL.
- [ ] **Step 3: Implement.** In `_home`, for every calibration home with a non-early impact (both metrics, so XYZ calibrations also report homing-axis margins as the spec's diagnostic), set `record["axis_noise_peak_g"], record["axis_impact_peak_g"] = axis_check_peaks(capture, axis)` (convert `ValueError` into `self._error("selected-axis capture cannot be evaluated: ...")`). In the save path, branch on the session metric: selected-axis → `selected_axis_threshold([...noise], [...impact])` over the qualified validation records (convert `ValueError` into `self._error(...)` naming noise max, impact min, window and the 0.5 g minimum), validate with `_validate_calibration_threshold(threshold, "saved selected-axis threshold")`, write metadata as in Interfaces; XYZ → unchanged path plus `threshold_rule`. Console summary line: metric, saved threshold, noise→threshold and threshold→impact margins.
- [ ] **Step 4: Run** `wsltest test_selected_axis_calibration.py`, `wsltest test_calibration_margin.py`, `wsltest test_calibration_validation.py`, `wsltest test_workflows.py` → PASS (baseline-listed only).
- [ ] **Step 5: Commit** `git commit -m "Save selected-axis calibration models with window-midpoint threshold"`

---

### Task 5: Quiet standstill reference for the selected-axis detector

**Files:**
- Modify: `beacon_accel_homing.py` (`_prepare_reference` ~1170–1225; `_reference_motion_noise` ~1293; constant near `ENABLE_SETTLE`)
- Modify: `tests/test_quiet_reference.py`

**Interfaces:**
- Produces: `QUIET_REFERENCE_MAX_WAIT = 2.0` (seconds after the first ready reference); `AccelHoming._reference_quiet_for_proof(self, reference, axis) -> bool` = every axis range `(high − low)` ≤ `BASELINE_NOISE_RATIO * threshold_g[axis]` in raw counts (the condition under which `_reference_motion_noise` returns `None`). `baseline_status["result"]` gains `"not_quiet"`.

- [ ] **Step 1: Write failing tests** (extend `RollingReferenceIntegrationTest.machine` with a stream whose first 0.5 s rings ±0.4 g on X then settles to ±0.02 g):

```python
def test_selected_axis_waits_for_quiet_reference(self):
    # threshold 1.9 (limit 0.38 g range): reference accepted only after ringing decays;
    # arming happens later than baseline_time, attempt.proof_noise is None
def test_selected_axis_never_quiet_fails_with_measured_noise(self):
    # ringing for 3 s -> CommandError mentioning "not quiet" and the measured range in g;
    # no homing move dispatched (endstop._dispatch.started == [])
def test_xyz_magnitude_reference_unchanged(self):
    # vector detector: reference accepted after baseline_time as today, no extra wait
def test_selected_axis_calibration_home_uses_quiet_rule(self):
    # calibration home with impact_metric selected_axis: require_quiet True, same wait
```

- [ ] **Step 2: Run** `wsltest test_quiet_reference.py` → new tests FAIL.
- [ ] **Step 3: Implement.** In `_prepare_reference`: `require_quiet` becomes `not self._uses_vector_detector(axis)` (calibration homes with the selected-axis metric no longer get the amplitude-independent reference; XYZ calibration is unchanged because it is vector). In the acquisition loop, when the reference is ready but the detector is not vector and `not _reference_quiet_for_proof(reference, axis)`: restart acquisition (fresh `QuietReference` with the same arguments, cutoff = current estimated print time) until quiet or until `QUIET_REFERENCE_MAX_WAIT` after the first ready reference; on timeout `_remember_reference(..., "not_quiet")` and raise `command_error("accel_homing: standstill reference not quiet after %.1f s (range x %.3f / y %.3f / z %.3f g, limit %.3f g); increase enable_settle_time or check for vibration")`. Keep the existing `acquisition_timeout` for the first ready reference.
- [ ] **Step 4: Run** `wsltest test_quiet_reference.py`, `wsltest test_backends.py`, `wsltest test_warmup_timing.py`, `wsltest test_trigger.py` → PASS (baseline-listed only).
- [ ] **Step 5: Commit** `git commit -m "Wait for a quiet reference before selected-axis homes"`

---

### Task 6: Docs, status mapping line and full suite

**Files:**
- Modify: `beacon_accel_homing.py` (`ACCEL_HOMING_STATUS` output ~3580–3590)
- Modify: `docs/SIMPLE_CALIBRATION.md`, `docs/CONFIGURATION.md` (thresholds/models section ~45–83), `docs/COMMANDS.md` (calibration paragraph), `TESTING.md` (focused checks list)
- Modify: `tests/test_selected_axis_calibration.py` (status line test)

**Interfaces:** none new.

- [ ] **Step 1: Write failing test** `test_status_reports_axes_map_source`: for a configured backend whose sensor has `axes_map`, `ACCEL_HOMING_STATUS` output contains `"axes_map applied by the sensor driver"` and the mapping; for native Beacon it contains `"accel_axes_map"`.
- [ ] **Step 2: Run** `wsltest test_selected_axis_calibration.py` → FAIL.
- [ ] **Step 3: Implement** the status line. Docs: SIMPLE_CALIBRATION.md gets a "Cross-axis noise and the selected-axis metric" section (pre-pass, 0.7 ratio, fallback warning, window-midpoint threshold, 0.5 g refusal, quiet reference). CONFIGURATION.md: selected-axis models are now produced by calibration; `axes_map` is applied by the driver for configured sensors and by `accel_axes_map` for Beacon. COMMANDS.md: one sentence in the Calibration section. TESTING.md: add `test_back_off_per_axis.py`, `test_selected_axis_signal.py`, `test_selected_axis_calibration.py` to focused checks.
- [ ] **Step 4: Run full suite** `wsltest "test*.py"` → `Ran >1023 tests`, failures/errors exactly the baseline list (compare `grep -E "^(FAIL|ERROR):" | sort` against `baseline_failures.txt`); `git diff --check` clean.
- [ ] **Step 5: Commit** `git commit -m "Document selected-axis calibration and axes_map handling"`

---

### Task 7: Verify on honserus with the user, then upstream PR

**Files:** printer config only (`pi@honserus.home.beattune.de:~/printer_data/config/{accel_homing.cfg,config/stepper.cfg,printer.cfg}`); fork remote.

Each step is a checkpoint with the user (inline, not batched). Printer rules: read-only inventory first, moves only after the user confirms preparation, abort on first error.

- [ ] **Step 1: Push branch to the fork** `git push -u origin feature/per-axis-backoff-selected-axis-calibration` (user approves the push).
- [ ] **Step 2: Deploy** on the printer: in `~/accel_homing` add remote `fork` = `https://github.com/beattune-core/accel_homing.git`, fetch, checkout the branch (no new modules, so existing links stay valid; verify with `ls -l ~/klipper/klippy/extras/*accel_homing*`). `RESTART` via Moonraker; state must be `ready`.
- [ ] **Step 3: Config** (user OK first): `accel_homing.cfg`: add `y_back_off_dist: 50`, comment out `y_threshold` and `enable_settle_time`; move `position_min`/`position_max`/`position_endstop` of `[stepper_x]`/`[stepper_y]` from `config/stepper.cfg` into the `#*#` SAVE_CONFIG block of `printer.cfg` (same values); remove the TEST comment block in `stepper.cfg`. `RESTART` → `ready`.
- [ ] **Step 4: G28** (user prepares): Y backs off 50 mm (position 279), X homes without early trigger. Check `ACCEL_HOMING_STATUS` shows per-axis back-off and the axes_map line.
- [ ] **Step 5: Out-of-range check:** `y_back_off_dist: 400`, RESTART, G28 → fails with "Move out of range"; revert to 50.
- [ ] **Step 6: Calibrate Y** (`M84`, hand-center, e-stop): `ACCEL_HOMING_CALIBRATE AXIS=Y CENTERED=1` → console shows `selected_axis`, ratio ≈ 0.4–0.5, saved ≈ 1.8 g; Klipper restarts with the model.
- [ ] **Step 7: Calibrate X** the same way → `xyz_magnitude`, ratio ≥ 0.7 (if X's ratio is below 0.7, stop and bring the numbers to the user before changing the constant).
- [ ] **Step 8: Qualify Y:** `M84`, hand-center, `ACCEL_HOMING_TROUBLESHOOT AXIS=Y SAMPLES=20 CENTERED=1` → 21/21 impacts, spread ≤ 0.1 mm, no motion-proof abort.
- [ ] **Step 9: PR** to upstream `FrankRamm/accel_homing` base `dev` from `beattune-core:feature/per-axis-backoff-selected-axis-calibration`, after asking the user whether `docs/superpowers/` stays in the PR. PR body: summary of the three changes, honserus measurements (noise/impact table), test results vs baseline, and the attribution line `🤖 Generated with [Claude Code](https://claude.com/claude-code)`.
