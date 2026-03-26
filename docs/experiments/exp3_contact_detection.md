# Experiment 3: Contact Detection Validation

**Status:** TODO — run after Experiment 1 confirms τ_feedback is usable

## Objective

Validate the dual-criterion contact detector:
1. Position-stuck detection (|Δq| < pos_stuck_threshold for N consecutive samples)
2. Torque threshold (mean(|τ|) > tau_threshold over sliding window)

## Background

The gripper has no external force sensor. Contact detection relies on:
- τ_feedback rising when jaws touch an object (motor current increases)
- Position not changing (motor stalls against object)

The trapezoidal screw's low back-drive efficiency (~0.30) means small jaw forces
produce relatively large motor torques — beneficial for detection but must be
tuned to avoid false positives during deceleration.

## Procedure

### Test 1: False positive rate during free motion
1. Command gripper to fully close (no object).
2. Count contact detection triggers during approach phase.
3. Target: 0 false positives during approach at full speed.

### Test 2: Detection latency
1. Place rigid object at mid-opening.
2. Command close at various approach speeds.
3. Measure: time from first contact to detection trigger.
4. Target: < 50 ms at 200 Hz control rate.

### Test 3: Parameter sensitivity
- Sweep tau_threshold: 0.5, 1.0, 2.0, 3.0 Nm
- Sweep window_size: 5, 10, 20 samples
- Sweep pos_stuck_threshold: 0.0005, 0.001, 0.002 rad

## Metrics

- True positive rate (contact correctly detected)
- False positive rate (contact detected during free motion)
- Detection latency [ms]

## Results

_To be filled in after experiment._

## Recommended Parameters

_To be filled in._

```yaml
# Recommended values for gripper_params.yaml
contact_tau_threshold: ???
contact_pos_stuck_threshold: ???
contact_window_size: ???
```
