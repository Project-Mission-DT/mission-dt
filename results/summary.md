# Experiment summary (5 repetitions)

Repetitions: rep1, rep2, rep3, rep4, rep5. Cells show mean ± sample standard deviation over repetitions; columns marked *pooled* use all samples of all repetitions (percentile = sample of rank floor(pn/100)+1). Times in ms.

## E1 Scalability

| N | reps | overruns/frames | frame mean | frame p99 | frame p99 pooled | frame max pooled | jitter p99 pooled | tele p99 | tele p99 pooled | act p99 | act p99 pooled | stale % | dup | uplink KiB/s | core CPU % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 5 | 0/1200 | 0.214 ± 0.015 | 0.36 ± 0.06 | 0.34 | 1.08 | 0.32 | 0.79 ± 0.12 | 0.76 | 0.71 ± 0.25 | 0.63 | 0.08 ± 0.19 | 10 ± 0 | 4.6 ± 0.0 | 0.4 ± 0.0 |
| 2 | 5 | 0/1200 | 0.279 ± 0.013 | 0.50 ± 0.06 | 0.49 | 0.69 | 0.33 | 1.00 ± 0.11 | 1.02 | 0.94 ± 0.30 | 0.78 | 0.00 ± 0.00 | 20 ± 0 | 9.3 ± 0.0 | 0.6 ± 0.0 |
| 5 | 5 | 0/1200 | 0.381 ± 0.023 | 0.74 ± 0.10 | 0.72 | 1.20 | 0.35 | 1.97 ± 1.60 | 1.57 | 1.31 ± 0.25 | 1.28 | 0.10 ± 0.18 | 49 ± 3 | 23.2 ± 0.0 | 0.9 ± 0.0 |
| 10 | 5 | 0/1200 | 0.534 ± 0.016 | 0.99 ± 0.11 | 1.02 | 1.66 | 0.51 | 2.24 ± 0.19 | 2.27 | 1.70 ± 0.13 | 1.73 | 0.00 ± 0.00 | 94 ± 3 | 46.4 ± 0.0 | 1.4 ± 0.0 |
| 25 | 5 | 0/1200 | 1.146 ± 0.070 | 4.36 ± 1.32 | 4.93 | 13.20 | 0.65 | 3.54 ± 0.82 | 3.86 | 4.07 ± 0.46 | 4.04 | 0.00 ± 0.00 | 240 ± 5 | 115.9 ± 0.0 | 3.1 ± 0.2 |
| 50 | 5 | 0/1200 | 3.610 ± 0.266 | 10.36 ± 1.43 | 10.75 | 17.17 | 1.08 | 7.55 ± 0.81 | 7.57 | 7.74 ± 0.41 | 7.75 | 0.02 ± 0.04 | 482 ± 14 | 231.8 ± 0.0 | 5.5 ± 0.1 |
| 75 | 5 | 0/1200 | 7.217 ± 0.714 | 18.56 ± 0.99 | 18.76 | 33.39 | 1.00 | 13.26 ± 3.36 | 13.79 | 12.16 ± 0.58 | 12.14 | 0.02 ± 0.03 | 743 ± 6 | 347.5 ± 0.0 | 8.8 ± 0.4 |
| 100 | 5 | 0/1200 | 11.196 ± 1.202 | 29.98 ± 3.93 | 30.40 | 46.22 | 1.32 | 23.30 ± 3.29 | 22.78 | 18.06 ± 1.58 | 18.04 | 0.21 ± 0.21 | 1034 ± 52 | 463.1 ± 0.1 | 11.8 ± 0.5 |

## E2 Bandwidth regulator (N = 10)

| regulator | reps | uplink KiB/s | msgs/s | dup | stale % | tele p99 | tele p99 pooled | frame p99 | frame p99 pooled |
|---|---|---|---|---|---|---|---|---|---|
| ON | 5 | 46.4 ± 0.0 | 83.6 ± 0.0 | 95 ± 4 | 0.00 ± 0.00 | 2.04 ± 0.11 | 2.04 | 0.93 ± 0.10 | 0.92 |
| OFF | 5 | 277.6 ± 0.1 | 500.2 ± 0.1 | 12545 ± 3 | 0.00 ± 0.00 | 1.67 ± 0.37 | 1.72 | 0.99 ± 0.18 | 1.07 |

| ratio OFF/ON | mean | std | reps |
|---|---|---|---|
| uplink_Bps | 5.99 | 0.00 | 5 |
| dup_updates | 131.92 | 5.06 | 5 |

## E3 Swarm-reaction latency

| N | reps | events | p50 | p99 | mean pooled | p50 pooled | p95 pooled | p99 pooled | max pooled | n pooled | % ≤125 ms | % ≤250 ms | frame p99 pooled | overruns/frames |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 10 | 5 | 937 ± 83 | 61.2 ± 2.7 | 120.1 ± 0.6 | 61.5 | 60.7 | 114.9 | 120.0 | 121.9 | 4686 | 100.0 | 100.0 | 0.97 | 0/1600 |
| 25 | 5 | 4139 ± 113 | 62.9 ± 0.9 | 122.8 ± 0.9 | 63.2 | 63.1 | 117.8 | 123.0 | 134.2 | 20694 | 99.7 | 100.0 | 5.24 | 0/1600 |
| 50 | 5 | 13470 ± 178 | 65.3 ± 0.6 | 130.5 ± 1.2 | 66.2 | 65.4 | 124.1 | 130.8 | 287.6 | 67350 | 95.6 | 99.9 | 10.78 | 0/1600 |

## E4 Twin fidelity under packet loss (regulator ON)

| loss | reps | lost msgs | stale % | pos RMSE m | pos RMSE pooled | pos p99 m | pos p99 pooled | max stale err pooled | hdg RMSE deg | hdg p99 deg |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 % | 5 | 0 ± 0 | 0.00 ± 0.00 | 0.617 ± 0.001 | 0.617 | 1.487 ± 0.012 | 1.487 | 0.000 | 0.64 ± 0.09 | 3.20 ± 0.64 |
| 5 % | 5 | 129 ± 14 | 4.68 ± 0.48 | 0.624 ± 0.004 | 0.624 | 1.502 ± 0.017 | 1.499 | 2.345 | 0.73 ± 0.14 | 3.48 ± 0.69 |
| 10 % | 5 | 257 ± 19 | 9.37 ± 0.74 | 0.632 ± 0.011 | 0.632 | 1.514 ± 0.027 | 1.512 | 3.338 | 0.86 ± 0.17 | 3.73 ± 0.68 |

Pooled error by domain and stale streak m (dead reckoning vs. hold of the last received state):

| loss | domain | m | count | DR RMSE m | hold RMSE m |
|---|---|---|---|---|---|
| 0 % | aerial | 0 | 5985 | 0.849 | 0.849 |
| 0 % | surface | 0 | 5985 | 0.202 | 0.202 |
| 5 % | aerial | 0 | 5709 | 0.847 | 0.847 |
| 5 % | aerial | 1 | 259 | 0.999 | 2.201 |
| 5 % | aerial | 2 | 16 | 1.238 | 3.954 |
| 5 % | aerial | >=3 | 2 | 1.261 | 6.459 |
| 5 % | surface | 0 | 5708 | 0.204 | 0.204 |
| 5 % | surface | 1 | 274 | 0.382 | 0.396 |
| 5 % | surface | 2 | 8 | 0.535 | 0.665 |
| 5 % | surface | >=3 | 1 | 0.943 | 1.033 |
| 10 % | aerial | 0 | 5413 | 0.845 | 0.845 |
| 10 % | aerial | 1 | 521 | 1.002 | 2.242 |
| 10 % | aerial | 2 | 48 | 1.111 | 3.585 |
| 10 % | aerial | >=3 | 6 | 1.517 | 6.010 |
| 10 % | surface | 0 | 5443 | 0.201 | 0.201 |
| 10 % | surface | 1 | 496 | 0.380 | 0.397 |
| 10 % | surface | 2 | 49 | 0.681 | 0.606 |
| 10 % | surface | >=3 | 3 | 1.111 | 0.606 |

## E4 Twin fidelity under packet loss (regulator OFF)

| loss | reps | lost msgs | stale % | pos RMSE m | pos RMSE pooled | pos p99 m | pos p99 pooled | max stale err pooled | hdg RMSE deg | hdg p99 deg |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 % | 5 | 0 ± 0 | 0.00 ± 0.00 | 0.182 ± 0.003 | 0.182 | 0.406 ± 0.012 | 0.409 | 0.000 | 0.32 ± 0.01 | 0.90 ± 0.10 |
| 5 % | 5 | 789 ± 12 | 0.00 ± 0.00 | 0.185 ± 0.002 | 0.185 | 0.447 ± 0.013 | 0.443 | 0.000 | 0.33 ± 0.01 | 0.94 ± 0.09 |
| 10 % | 5 | 1610 ± 17 | 0.00 ± 0.00 | 0.195 ± 0.006 | 0.195 | 0.510 ± 0.021 | 0.507 | 0.000 | 0.34 ± 0.02 | 1.02 ± 0.16 |

Pooled error by domain and stale streak m (dead reckoning vs. hold of the last received state):

| loss | domain | m | count | DR RMSE m | hold RMSE m |
|---|---|---|---|---|---|
| 0 % | aerial | 0 | 5986 | 0.209 | 0.209 |
| 0 % | surface | 0 | 5989 | 0.151 | 0.151 |
| 5 % | aerial | 0 | 5990 | 0.214 | 0.214 |
| 5 % | surface | 0 | 5991 | 0.150 | 0.150 |
| 10 % | aerial | 0 | 5985 | 0.232 | 0.232 |
| 10 % | surface | 0 | 5987 | 0.150 | 0.150 |

## E5 Resource footprint of the Mission-DT process

| N | reps | CPU % of one core | peak RSS MiB | frames | overruns |
|---|---|---|---|---|---|
| 0 | 5 | 0.1 ± 0.0 | 22.6 ± 0.1 | 240 ± 0 | 0 ± 0 |
| 10 | 5 | 1.5 ± 0.1 | 23.1 ± 0.1 | 240 ± 0 | 0 ± 0 |
| 50 | 5 | 5.8 ± 0.2 | 23.8 ± 0.1 | 240 ± 0 | 0 ± 0 |
| 100 | 5 | 12.1 ± 0.2 | 24.8 ± 0.0 | 240 ± 0 | 0 ± 0 |

## E6 Resource comparison: Mission-DT (MQTT), ROS 2, Gazebo

Per process, over the measured window: CPU = (utime + stime) / wall in % of one core; peak RSS = VmHWM; RSS end = VmRSS at the end of the window. All processes on core 0. *total* sums the processes of the stack.

| stack | N | reps | process | CPU % of one core | peak RSS MiB | RSS end MiB |
|---|---|---|---|---|---|---|
| mission_dt | 10 | 5 | core | 1.4 ± 0.1 | 25.4 ± 0.0 | 25.4 ± 0.0 |
| mission_dt | 10 | 5 | agents | 2.6 ± 0.3 | 29.2 ± 0.0 | 29.2 ± 0.0 |
| mission_dt | 10 | 5 | broker | 0.3 ± 0.0 | 7.4 ± 0.0 | 7.4 ± 0.0 |
| mission_dt | 10 | 5 | total | 4.4 ± 0.4 | 62.0 ± 0.0 | 62.0 ± 0.0 |
| mission_dt | 50 | 5 | core | 5.9 ± 0.2 | 26.3 ± 0.0 | 26.3 ± 0.0 |
| mission_dt | 50 | 5 | agents | 11.6 ± 0.4 | 49.6 ± 0.1 | 49.6 ± 0.1 |
| mission_dt | 50 | 5 | broker | 1.3 ± 0.1 | 7.4 ± 0.0 | 7.4 ± 0.0 |
| mission_dt | 50 | 5 | total | 18.8 ± 0.6 | 83.3 ± 0.1 | 83.3 ± 0.1 |
| mission_dt | 100 | 5 | core | 11.8 ± 1.1 | 27.4 ± 0.1 | 27.4 ± 0.1 |
| mission_dt | 100 | 5 | agents | 21.1 ± 1.6 | 75.1 ± 0.1 | 75.1 ± 0.1 |
| mission_dt | 100 | 5 | broker | 2.4 ± 0.2 | 7.6 ± 0.0 | 7.6 ± 0.0 |
| mission_dt | 100 | 5 | total | 35.4 ± 2.9 | 110.1 ± 0.2 | 110.1 ± 0.2 |
| ros2 | 10 | 5 | core | 4.3 ± 0.4 | 69.6 ± 0.1 | 69.6 ± 0.1 |
| ros2 | 10 | 5 | agents | 3.5 ± 0.2 | 74.2 ± 0.0 | 74.2 ± 0.0 |
| ros2 | 10 | 5 | total | 7.8 ± 0.5 | 143.8 ± 0.1 | 143.8 ± 0.1 |
| ros2 | 50 | 5 | core | 19.3 ± 3.8 | 75.3 ± 0.1 | 75.3 ± 0.1 |
| ros2 | 50 | 5 | agents | 13.4 ± 0.3 | 97.1 ± 0.1 | 97.1 ± 0.1 |
| ros2 | 50 | 5 | total | 32.7 ± 3.7 | 172.4 ± 0.2 | 172.4 ± 0.2 |
| ros2 | 100 | 5 | core | 28.7 ± 2.1 | 82.4 ± 0.2 | 82.4 ± 0.2 |
| ros2 | 100 | 5 | agents | 23.0 ± 0.8 | 125.3 ± 0.0 | 125.3 ± 0.0 |
| ros2 | 100 | 5 | total | 51.8 ± 2.7 | 207.8 ± 0.2 | 207.8 ± 0.2 |
| gazebo | 10 | 5 | gz_sim | 47.4 ± 0.8 | 146.6 ± 0.1 | 146.6 ± 0.1 |
| gazebo | 10 | 5 | total | 47.4 ± 0.8 | 146.6 ± 0.1 | 146.6 ± 0.1 |
| gazebo | 50 | 5 | gz_sim | 97.2 ± 0.1 | 263.3 ± 0.2 | 263.3 ± 0.2 |
| gazebo | 50 | 5 | total | 97.2 ± 0.1 | 263.3 ± 0.2 | 263.3 ± 0.2 |
| gazebo | 100 | 5 | gz_sim | 97.2 ± 0.1 | 409.7 ± 0.3 | 409.7 ± 0.3 |
| gazebo | 100 | 5 | total | 97.2 ± 0.1 | 409.7 ± 0.3 | 409.7 ± 0.3 |

Mission layer in the window (ms):

| stack | N | reps | max overruns/mean frames | frame mean | frame max | agents received | min msgs per agent | telemetry Hz per agent | tele p50 | tele p99 | min act. per agent | act p99 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| mission_dt | 10 | 5 | 0/240 | 0.51 ± 0.03 | 1.07 ± 0.27 | 10 ± 0 | 250 ± 0 | 8.33 ± 0.00 | 0.68 ± 0.37 | 2.07 ± 0.37 | 241 ± 1 | 1.91 ± 0.31 |
| mission_dt | 50 | 5 | 0/240 | 4.40 ± 0.29 | 17.45 ± 3.41 | 50 ± 0 | 250 ± 0 | 8.33 ± 0.00 | 0.69 ± 0.18 | 8.56 ± 1.09 | 240 ± 0 | 8.91 ± 0.96 |
| mission_dt | 100 | 5 | 0/240 | 10.77 ± 1.42 | 42.22 ± 6.69 | 100 ± 0 | 250 ± 0 | 8.33 ± 0.00 | 1.52 ± 0.78 | 23.50 ± 1.85 | 240 ± 0 | 17.89 ± 1.85 |
| ros2 | 10 | 5 | 0/241 | 1.81 ± 0.14 | 7.57 ± 2.61 | 10 ± 0 | 250 ± 0 | 8.33 ± 0.00 | 0.63 ± 0.13 | 2.20 ± 0.31 | 240 ± 0 | 2.27 ± 0.26 |
| ros2 | 50 | 5 | 0/240 | 5.83 ± 0.51 | 16.68 ± 8.16 | 50 ± 0 | 250 ± 0 | 8.33 ± 0.01 | 2.32 ± 0.23 | 11.63 ± 0.91 | 240 ± 0 | 11.90 ± 1.05 |
| ros2 | 100 | 5 | 0/240 | 8.99 ± 0.53 | 22.84 ± 5.53 | 100 ± 0 | 250 ± 0 | 8.33 ± 0.00 | 8.17 ± 0.93 | 31.69 ± 4.80 | 240 ± 0 | 24.60 ± 4.25 |

Gazebo real-time factor over the window (sim time / wall time; 1 ms physics step, target 1.0) and odometry received by the monitor:

| N | reps | RTF | min RTF (stats msg) | steps/s | models received | odom Hz per model (wall) | monitor CPU % |
|---|---|---|---|---|---|---|---|
| 10 | 5 | 0.999 ± 0.000 | 0.446 ± 0.225 | 999 ± 0 | 10 ± 0 | 8.27 ± 0.00 | 2.1 ± 0.1 |
| 50 | 5 | 0.744 ± 0.014 | 0.192 ± 0.021 | 744 ± 14 | 50 ± 0 | 6.15 ± 0.12 | 2.7 ± 0.0 |
| 100 | 5 | 0.329 ± 0.006 | 0.095 ± 0.012 | 329 ± 6 | 100 ± 0 | 2.71 ± 0.05 | 2.8 ± 0.0 |

