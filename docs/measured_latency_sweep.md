### Scored criterion (pelvis-to-reference > 0.5 m ends the rollout)

| arm | λ 0 | λ 0.5 | λ 1 | λ 1.5 | λ 2 | λ 3 |
|---|---|---|---|---|---|---|
| no_dr_s8600 | 0/16 (0%, stop 5.5s) | 0/16 (0%, stop 6.1s) | 2/16 (12%, stop 5.2s) | 1/16 (6%, stop 4.2s) | 0/16 (0%, stop 3.9s) | 1/16 (6%, stop 2.8s) |
| deploy_dr_s8600 | 0/16 (0%, stop 1.3s) | 0/16 (0%, stop 1.3s) | 0/16 (0%, stop 1.3s) | 0/16 (0%, stop 1.3s) | 0/16 (0%, stop 1.3s) | 0/16 (0%, stop 1.3s) |

### What actually happened (fell over vs drifted off the path while upright)

| arm | λ 0 | λ 0.5 | λ 1 | λ 1.5 | λ 2 | λ 3 |
|---|---|---|---|---|---|---|
| no_dr_s8600 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 14 · tracked 2 | fell 0 · drifted 15 · tracked 1 | fell 0 · drifted 16 · tracked 0 | fell 4 · drifted 11 · tracked 1 |
| deploy_dr_s8600 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 | fell 0 · drifted 16 · tracked 0 |
