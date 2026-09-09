### Scored criterion (pelvis-to-reference > 0.5 m ends the rollout)

| arm | λ 0 | λ 0.5 | λ 1 | λ 1.5 | λ 2 |
|---|---|---|---|---|---|
| no_dr_s8600 | 0/16 (0%, stop 5.5s) | 3/16 (19%, stop 3.4s) | 0/16 (0%, stop 3.1s) | 0/16 (0%, stop 2.2s) | 0/16 (0%, stop 1.7s) |
| deploy_dr_s8600 | 0/16 (0%, stop 1.3s) | 0/16 (0%, stop 1.3s) | 0/16 (0%, stop 1.4s) | 0/16 (0%, stop 1.4s) | 0/16 (0%, stop 1.5s) |

### What actually happened (fell over vs drifted off the path while upright)

| arm | λ 0 | λ 0.5 | λ 1 | λ 1.5 | λ 2 |
|---|---|---|---|---|---|
| no_dr_s8600 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 13 · tracked 3 | fell 0 · drifted 16 · tracked 0 | fell 3 · drifted 13 · tracked 0 | fell 5 · drifted 11 · tracked 0 |
| deploy_dr_s8600 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 1 · drifted 15 · tracked 0 |
