# Per-axis back-off and selected-axis calibration — design

Date: 2026-11-09 · Branch: `feature/per-axis-backoff-selected-axis-calibration` (from `dev`)
Fork: `beattune-core/accel_homing`, intended as a PR to upstream `FrankRamm/accel_homing` after
verification on the test printer.

## Why

Test printer "honserus": Kalico, CoreXY with AWD (4 × TMC5160 at 48 V), Cartographer ADXL345
(`[adxl345 carto]`, `axes_map: y, -x, z`) on the toolhead, X on accel_homing with a saved
XYZ-magnitude model (4.5 g, impacts ~15–17 g).

Two problems on that printer:

1. **Back-off too short.** After Y homes against the rear stop, the 5 mm back-off leaves the
   toolhead next to the (now unused) Y switch. When X homes next, the Cartographer cable snags
   on that switch, the jolt reaches the ADXL and X triggers early. The old switch setup used
   `homing_retract_dist: 50` on Y for exactly this. `back_off_dist` is global and capped at 20 mm.
   Other machines need large per-axis clearances for other reasons (nozzle cleaner, purge bucket).

2. **Y calibration cannot succeed.** On Y the *gantry* hits the stop, so the toolhead only sees an
   attenuated knock. Meanwhile, during Y moves the stepper torque ripple excites the ~290 Hz motor
   resonance and shakes the toolhead mostly sideways (printer X). Measured on 21 real Y homes at
   50 mm/s, ACCEL 5000 (troubleshoot capture, detector-equivalent peak |a − reference|):

   | | pre-impact noise (max) | impact (weakest) | gap |
   |---|---|---|---|
   | XYZ magnitude | 3.2–3.4 g | ~3.2 g | none |
   | Y component only | 1.43 g | 2.21 g | ~0.8 g |

   The calibration always uses XYZ magnitude, so every Y calibration either false-triggers on
   noise or misses weak impacts and skips steps. Hand-configured `y_threshold: 1.9` (which
   selects the legacy selected-axis detector) passed 27/27 homes, spread 0.029 mm.

   A side finding: in selected-axis mode the startup motion proof needs a quiet standstill
   reference. The snap of motors re-enabled after `M84` + hand-centering rang up to 31 g p-p and
   was still decaying 0.4 s later; with the default `enable_settle_time` 0.25 s the reference
   was noisy, the motion proof's lower bound fell to 0 and homing aborted
   ("toolhead motion was not verified before the startup deadline"). `enable_settle_time: 1.0`
   fixed it by hand.

## Goals

1. Per-axis back-off distances with Klipper/Kalico `homing_retract_dist` semantics.
2. `ACCEL_HOMING_CALIBRATE AXIS=<X|Y>` produces a working model on machines where cross-axis
   noise swamps the XYZ magnitude, by choosing the homing-axis-only (selected-axis) metric when
   it separates noise and impact better, and saving that choice in the model.
3. Homing does not depend on a hand-tuned `enable_settle_time` for a quiet reference.
4. Document that `axes_map` is honoured (see "Axis mapping").

## Non-goals

- No detector filtering (low-/band-pass). Larger separate change.
- No changes to the travel wizard's include handling. The user will move `position_min`,
  `position_max`, `position_endstop` of `[stepper_x]`/`[stepper_y]` into the SAVE_CONFIG block
  of `printer.cfg` on the test printer (a config task, not code).
- No second touch / `second_homing_speed` behaviour.
- No change to machines where XYZ magnitude already works: they must keep getting XYZ models.

## Anti-goals

- Silently clamping a back-off that Kalico itself would reject.
- A calibration that touches the stop before it has a detector setting that is safe (i.e. the
  approach-from-below safety of the existing threshold grid must be kept: an early trigger is
  harmless, a missed impact grinds the motors).
- Breaking existing saved models or configs.

## Part 1 — Per-axis back-off (decided)

- New `[accel_homing]` options `x_back_off_dist`, `y_back_off_dist`. Existing `back_off_dist`
  stays as the shared fallback (precedence like `x_threshold`/`threshold`). Default 5.0,
  `minval=0.0`, **no maximum** (Kalico `homing_retract_dist` has `minval=0` only). 0 disables.
- Speed unchanged: `homing_retract_speed` of the rail, else `homing_speed` (already the case).
- Applies where `back_off_dist` applies today, now per axis: the final release after a
  successful home (`_retract_completed_home`, `release_workflow_home`) and the cross-axis
  release before homing another axis (`_back_off_others`).
- **Out-of-range behaviour matches Kalico:** Kalico accepts any non-negative
  `homing_retract_dist` at config load; once the axis is homed its limits are
  `position_min..position_max`, and an oversize retract fails G28 with "Move out of range".
  Accel_homing does the same: no config-time range check; if a back-off target lies outside
  the rail range, raise a `command_error` (message modelled on Kalico's) instead of today's
  silent clamp. Applies to both the final and the cross-axis release.
- Workflows that save/suppress/restore `back_off_dist` as one float (`saved_back_off` in
  `accel_homing_workflows.py`, `accel_homing_center.py`, `accel_homing_travel.py`, settings
  snapshots/fingerprints) handle the per-axis values, the same way `threshold_g` is handled.
- Status/diagnostic output that reports `back_off_dist` reports the per-axis values.
- `docs/CONFIGURATION.md`: replace "0–20" and describe the per-axis options and the Kalico-like
  error.

## Part 2 — Travel wizard include handling (dropped)

Originally requested; dropped after discussion. The user moves the position values into the
SAVE_CONFIG block of `printer.cfg` on the test printer instead (see Non-goals).

## Part 3 — Selected-axis calibration (decided parts)

**Metric selection from moving noise, before any stop contact.** Calibration gains a
moving-noise pre-pass at the effective homing speed and acceleration, short of the stop. Reuse
the BETA experiment's central-stroke motion plan and noise acquisition
(`_beta_motion_plan`, `_beta_motion_noise`, `_beta_profile`; docs/BETA.md), extended to record
per-component peaks (homing-axis component and XYZ magnitude, detector-equivalent
|a − reference|) for startup/acceleration/cruise segments.

From the pre-pass, pick the metric for the whole calibration:
- XYZ magnitude unless the homing-axis-only noise peak is clearly lower than the magnitude
  noise peak (cross-axis noise dominates). On honserus Y: 1.3–1.5 g vs 2.7–3.4 g.
- The chosen metric's moving-noise peak seeds the threshold grid start (as BETA seeds it today:
  max(20 %, 0.1 g) above the peak, rounded up onto the grid), so stepping still approaches the
  impact from below.

**Calibration runs with the chosen detector.** Discovery, early-trigger recovery, ten checks and
the 2 mm spread rule are unchanged. The +0.5 g saved margin stays for XYZ-magnitude models only.

**Saved threshold for selected-axis models (user decision).** The fixed +0.5 g margin would put
the saved value above the weakest impact on honserus Y (tested ~1.75 g → saved 2.25 g > 2.21 g).
Instead, from the ten qualified checks: window = highest pre-impact noise peak .. weakest impact
peak (both on the homing-axis component). Save the window midpoint. If the window is narrower than
~0.5 g, refuse to save with a message giving the measured noise, impact and window. On honserus Y
this yields ≈ 1.8 g (hand-tuned 1.9 g passed 27/27). `_quality()` currently rejects any record
that is not `xyz_magnitude` during calibration; it must accept the selected-axis detector when
that metric was chosen, including its motion-proof requirement (`moving`/`motion_seen`).

**Saved model** records `impact_metric: "selected_axis"` (or `xyz_magnitude`),
`polarity_policy: POLARITY_POLICY` (so `check_home_axes` accepts it), the pre-pass noise peaks
for both metrics, and the per-check onset/peak for the chosen metric. Ordinary homing with a
selected-axis model uses the selected-axis detector (`simple_detection()` today returns True only
for `xyz_magnitude` models; models with `selected_axis` must route to the selected-axis path).
Explicit `x_threshold`/`y_threshold` keep precedence, unchanged.

**Validation records keep both metrics.** For each of the ten checks, record the pre-impact noise
peak and impact peak of both metrics from the retained samples, and report the resulting margins
(noise→threshold, threshold→weakest impact) in the console and metadata. This is diagnostic: it
shows the user how much headroom the saved threshold has.

**Decision rule (Claude's call).** Ratio of homing-axis noise peak to XYZ-magnitude noise peak
(the component can never exceed the magnitude). Choose selected-axis when the ratio is below
~0.7. Expected on honserus: Y ≈ 0.4–0.5 (1.3–1.5 g vs 2.7–3.4 g); X ≈ 0.9+ (constant-speed tests
showed the X-direction component carrying almost all noise on X moves too). The builder confirms
X on the printer before fixing the constant.

**Pre-pass unavailable (Claude's call).** If the BETA-style pre-pass cannot run (no trapq
extractor, stroke too short, missing hooks), calibration falls back to today's behaviour
(XYZ magnitude, stationary-noise seed) and prints a clear console warning that cross-axis
selection was skipped and why. Calibration never aborts only because the pre-pass is missing.
When it runs, the pre-pass seed replaces the stationary-noise seed for the start threshold; the
stationary noise preflight itself stays (it checks the sensor at rest).

**Quiet reference instead of a fixed settle.** Scope (Claude's call): only the selected-axis
detector — ordinary homes with a selected-axis model or manual threshold, and calibration homes
when selected-axis was chosen. XYZ-magnitude homing and calibration keep today's behaviour.
Before arming, after motor enable and
cross-axis release, wait until the standstill reference is quiet enough for the active detector
(for selected-axis: quiet enough that the startup motion proof can succeed), bounded by a maximum
wait; if it never gets quiet, fail with a clear message naming the measured noise.
`enable_settle_time` stays as the minimum wait.

## Axis mapping (verified, document only)

The configured-sensor backend reads `batch_bulk` samples that Klipper/Kalico's `adxl345` driver
has already converted with `axes_map` (`_convert_samples`); the plugin then uses machine axes
directly (`_raw_axis()` returns `(AXES[axis], 1)` for configured backends: "The configured driver
already applies axes_map"). Troubleshoot CSVs on honserus are in the printer frame. Native
Beacon uses `accel_axes_map` explicitly. Calibration is therefore not affected by `axes_map`.
Add one sentence to the docs, and show the effective mapping in `ACCEL_HOMING_STATUS` if it is
not already shown.

## Constraints

- Klipper and Kalico both supported (plugin already handles both).
- Python as on the host (Kalico on honserus: Python 3.13 klippy-env).
- Plugin and upstream conventions: config via `config.getfloat(...)`, errors via
  `printer.command_error`/`config.error`, docs in `docs/*.md`.
- Homing speed ≥ 50 mm/s (plugin warning; upstream dev runs 75 mm/s).
- Verification on honserus is supervised by the user (hand on e-stop); Claude never reboots VMs
  or the printer host; motion runs only after the user confirms XY is prepared.

## Tests

Allowed and expected for this repo (user decision). Add tests in the existing `tests/` style
for: per-axis back-off options and fallback, the Kalico-like out-of-range error, metric selection
from pre-pass noise, the pre-pass fallback, selected-axis model save/route, and the
quiet-reference wait. Tests must not need edits when unrelated features or options are added
(no exhaustive option enumeration, no count/position assertions over unrelated data).

## Left to the builder

- Exact decision rule and constants for "clearly lower" (ratio vs. absolute gap), as long as
  honserus X stays magnitude and honserus Y becomes selected-axis.
- How to integrate the BETA pre-pass into normal calibration (shared helpers vs. extraction).
- Quiet-reference criterion, polling and maximum wait.
- Naming of new metadata keys and console wording.
- Whether `ACCEL_HOMING_TROUBLESHOOT` also reports both metrics (nice to have).

## Verification on honserus (after implementation)

1. Config: `y_back_off_dist: 50`, X default; move X/Y position values into SAVE_CONFIG; remove
   `y_threshold` and `enable_settle_time` hand overrides.
2. G28: Y backs off 50 mm, X homes without early trigger. An oversize `y_back_off_dist` fails G28
   like Kalico.
3. `ACCEL_HOMING_CALIBRATE AXIS=Y CENTERED=1` picks selected-axis and saves a model;
   `ACCEL_HOMING_CALIBRATE AXIS=X` still picks magnitude.
4. `ACCEL_HOMING_TROUBLESHOOT AXIS=Y SAMPLES=20`: 20/20 impacts, spread ≤ 0.1 mm, after `M84` +
   hand-centering (no motion-proof abort).
5. Then PR to upstream.
