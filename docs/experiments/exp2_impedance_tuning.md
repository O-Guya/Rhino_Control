# Experiment 2: Impedance Controller Tuning

**Status:** TODO — run after Experiment 1 confirms τ_feedback is usable

## Objective

Tune the virtual spring-damper impedance controller parameters (k_spring, k_damper)
for stable, compliant grasping without oscillation.

## Background

Controller output: τ_cmd = k_spring * (q_target - q) - k_damper * dq

The controller runs at 200 Hz. The DM4310 MIT mode accepts torque feed-forward
(kp=kd=0 in the CAN command), so the motor's internal PD loop is bypassed.

## Procedure

### Phase 1: Stability boundary (no load)
1. Set k_spring = 5, k_damper = 0 (pure spring, no damping).
2. Increase k_spring until oscillation appears → record k_spring_crit.
3. Set k_damper = 0.5 * (2 * sqrt(k_spring * J_eff)) for critical damping estimate.
4. Sweep k_damper until no overshoot.

### Phase 2: Grasp compliance
1. Hold a rigid object between jaws.
2. Vary k_spring: 5, 10, 20, 50 Nm/rad.
3. Record: steady-state τ, jaw position error, object deformation (qualitative).

## Parameters to Tune

| Parameter   | Initial | Min | Max | Units  |
|-------------|---------|-----|-----|--------|
| k_spring    | 20.0    | 1   | 100 | Nm/rad |
| k_damper    | 0.5     | 0   | 5   | Nm·s/rad |
| tau_max     | 5.0     | 1   | 18  | Nm     |

## Results

_To be filled in after experiment._

## Conclusions

_To be filled in._
