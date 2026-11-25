# Demo joint torques (d9-d11)

Per arm and joint, over the three final demo runs, N m (fraction of rated). Made with `python dev/torque_table.py dev/runs/d9_torque.npz dev/runs/d10_torque.npz dev/runs/d11_torque.npz`.

| Arm | measure | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|---|
| all | rated (continuous) | 9 | 9 | 9 | 3 | 3 | 3 |
| all | peak rating | 27 | 27 | 27 | 7 | 7 | 7 |
| A left | peak | 6.45 (0.72) | 5.37 (0.60) | 7.97 (0.89) | 2.15 (0.72) | 1.30 (0.43) | 1.99 (0.66) |
| A left | highest 1 s average | 2.54 (0.28) | 5.01 (0.56) | 6.92 (0.77) | 1.99 (0.66) | 1.13 (0.38) | 0.63 (0.21) |
| A left | highest 10 s average | 2.31 (0.26) | 4.57 (0.51) | 6.24 (0.69) | 1.78 (0.59) | 0.96 (0.32) | 0.43 (0.14) |
| A right | peak | 6.40 (0.71) | 6.65 (0.74) | 7.91 (0.88) | 2.29 (0.76) | 1.17 (0.39) | 1.35 (0.45) |
| A right | highest 1 s average | 1.11 (0.12) | 6.01 (0.67) | 6.81 (0.76) | 2.04 (0.68) | 0.62 (0.21) | 0.53 (0.18) |
| A right | highest 10 s average | 0.48 (0.05) | 5.00 (0.56) | 6.16 (0.68) | 1.81 (0.60) | 0.60 (0.20) | 0.40 (0.13) |
| B left | peak | 6.47 (0.72) | 7.61 (0.85) | 8.55 (0.95) | 2.06 (0.69) | 1.65 (0.55) | 2.18 (0.73) |
| B left | highest 1 s average | 2.49 (0.28) | 6.92 (0.77) | 7.11 (0.79) | 1.88 (0.63) | 1.38 (0.46) | 1.00 (0.33) |
| B left | highest 10 s average | 1.39 (0.15) | 4.75 (0.53) | 6.19 (0.69) | 1.71 (0.57) | 0.78 (0.26) | 0.80 (0.27) |
| B right | peak | 6.38 (0.71) | 7.63 (0.85) | 7.90 (0.88) | 2.40 (0.80) | 1.67 (0.56) | 1.33 (0.44) |
| B right | highest 1 s average | 2.53 (0.28) | 6.85 (0.76) | 7.08 (0.79) | 2.00 (0.67) | 1.34 (0.45) | 0.54 (0.18) |
| B right | highest 10 s average | 0.70 (0.08) | 4.80 (0.53) | 6.08 (0.68) | 1.73 (0.58) | 0.89 (0.30) | 0.43 (0.14) |

- highest peak / rated: 0.95 (B left J3)
- highest avg1 / rated: 0.79 (B left J3)
- highest avg10 / rated: 0.69 (A left J3)
