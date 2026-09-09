### Scored criterion (pelvis-to-reference > 0.5 m ends the rollout)

| arm | λ 0 | λ 0.5 | λ 1 | λ 1.5 | λ 2 | λ 3 |
|---|---|---|---|---|---|---|
| no_dr_s8600 | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) | 2/16 (12%, stop 8.6s) | 1/16 (6%, stop 8.6s) | 0/16 (0%, stop 8.6s) | 1/16 (6%, stop 8.6s) |
| deploy_dr_s8600 | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) |

### What actually happened (fell over vs drifted off the path while upright)

| arm | λ 0 | λ 0.5 | λ 1 | λ 1.5 | λ 2 | λ 3 |
|---|---|---|---|---|---|---|
| no_dr_s8600 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 14 · tracked 2 | fell 6 · drifted 9 · tracked 1 | fell 10 · drifted 6 · tracked 0 | fell 11 · drifted 4 · tracked 1 |
| deploy_dr_s8600 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 6 · drifted 10 · tracked 0 |
