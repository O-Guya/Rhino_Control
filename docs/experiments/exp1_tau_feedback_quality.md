# Experiment 1: Torque Feedback Quality

**Status:** TODO — run after bare motor is confirmed working with SocketCAN

## Objective

Characterize the quality of the torque feedback (τ_feedback) from the DM4310
motor in SocketCAN MIT mode. The feedback is a 12-bit current estimate; this
experiment evaluates its noise level, latency, and usefulness for contact
detection.

## Background

- Sensor: motor internal 12-bit current estimate → τ [Nm] via KT constant
- Transmission: trapezoidal screw, η ≈ 0.30 (low back-drive efficiency)
- Expected issue: τ_feedback reflects motor-side torque; jaw force ≈ τ × η / lead

## Procedure

1. Enable motor, set kp=0, kd=0.5, command static position.
2. Log τ_feedback at 200 Hz for 30 seconds with no load.
3. Apply known static loads (0 N, 5 N, 10 N, 20 N) via hanging mass on screw.
4. Record τ_feedback for each load.

## Metrics

- Noise: std(τ) with no load
- Resolution: minimum detectable Δτ
- SNR: mean(τ) / std(τ) for known loads
- Latency: estimated from step-response (apply load, measure τ rise time)

## Results

_To be filled in after experiment._

| Load [N] | τ_mean [Nm] | τ_std [Nm] | SNR |
|----------|-------------|------------|-----|
| 0        |             |            |     |
| 5        |             |            |     |
| 10       |             |            |     |
| 20       |             |            |     |

## Conclusions

_To be filled in._
