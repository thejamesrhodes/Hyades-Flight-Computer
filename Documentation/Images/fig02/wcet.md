| Task | Pad, real IMU (cyc) | HITL max over cases (cyc) | case | µs |
|---|---|---|---|---|
| sense | 151,694 | 151,752 | drop_15fr_boost | 1581 |
| flight | 42,857 | 55,435 | gps_garbage_desc | 577 |
| log | 120 | 35,158 | accsf_p3 | 366 |
| telem | 61,974 | 61,713 | thrust_p10 | 646 |
| comms | 13,548 | 16,302 | accsf_m3 | 170 |
| leds | 416 | 490 | thrust_p10 | 5 |

| Flight_Step section | Pad (cyc) | HITL max (cyc) | case |
|---|---|---|---|
| estimate | 26,614 | 29,541 | wind9_gust6 |
| monitor | 13,132 | 33,715 | i205w_nominal |
| control | 1,377 | 10,967 | gyro_spike_coast |
| alloc | 603 | 653 | vib_nominal |

| Frame, by phase | WCET (cyc) | µs | % of frame | case |
|---|---|---|---|---|
| READY (pad, real IMU) | 215,371 | 2243 | 44.9 | pad |
| SETTLE (pad, real IMU) | 190,633 | 1986 | 39.7 | pad |
| BOOST (HITL) | 85,357 | 889 | 17.8 | pad_knock_short |
| CATO (HITL) | 62,093 | 647 | 12.9 | drop_15fr_boost |
| COAST (HITL) | 72,107 | 751 | 15.0 | combined_worst |
| DESCENT (HITL) | 68,627 | 715 | 14.3 | vib_nominal |
| LANDED (HITL) | 73,633 | 767 | 15.3 | drop_15fr_boost |
| READY (HITL) | 175,269 | 1826 | 36.5 | thrust_p10 |
| SETTLE (HITL) | 209,609 | 2183 | 43.7 | thrust_p10 |

Flight-frame bound, HITL worst BOOST/COAST frame plus the worst real I2C read: 237,109 cyc (2470 µs, 49.4 % of the frame).
Sum of every task's WCET, worse of pad and HITL (all coinciding in one frame): 321,111 cyc (3345 µs, 66.9 %). Budget (guideline B1, 80 %): 384,000 cyc.
Corner, not measured (log writer at the data-sheet 100 µs/word, telemetry idle): 454,379 cyc (4733 µs, 94.7 %).
