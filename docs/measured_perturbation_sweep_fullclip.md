### Scored criterion (pelvis-to-reference > 0.5 m ends the rollout)

| arm | λ 0 | λ 0.5 | λ 1 | λ 1.5 | λ 2 |
|---|---|---|---|---|---|
| no_dr_s8600 | 0/16 (0%, stop 8.6s) | 3/16 (19%, stop 8.6s) | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) |
| deploy_dr_s8600 | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) | 0/16 (0%, stop 8.6s) |

### What actually happened (fell over vs drifted off the path while upright)

| arm | λ 0 | λ 0.5 | λ 1 | λ 1.5 | λ 2 |
|---|---|---|---|---|---|
| no_dr_s8600 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 13 · tracked 3 | fell 7 · drifted 9 · tracked 0 | fell 11 · drifted 5 · tracked 0 | fell 14 · drifted 2 · tracked 0 |
| deploy_dr_s8600 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 3 · drifted 13 · tracked 0 | fell 5 · drifted 11 · tracked 0 |
