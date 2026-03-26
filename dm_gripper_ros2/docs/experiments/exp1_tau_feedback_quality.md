# Experiment 1: τ Feedback Quality Assessment

**Date**: TBD
**Status**: Pending
**Branch**: N/A

## Objective

评估 DM4310 电机通过 MIT 模式反馈的力矩估算质量（12-bit 电流→τ）。
量化噪声水平、梯形螺纹反传效率、以及在不同负载下的可用性。

## Setup

- 电机型号：DM4310
- 接口：SocketCAN (can0, 经典 CAN 1 Mbps)
- 控制模式：MIT mode (kp=0, kd=0, τ_ff=扫描值)

## Metrics

- τ_feedback 噪声标准差 (无负载)
- τ_feedback 噪声标准差 (已知负载)
- 传感器延迟估算

## Results

TODO

## Conclusions

TODO
