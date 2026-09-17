# Hardware runs

Every time this bundle has been on a real G1, in order. One entry so far.

This file exists because the first run produced nothing. `run.sh` wrote no files
at the time, so the only copy of that run was the operator's terminal
scrollback, and it is gone — checked at the time for a log in `results/`, in
`/tmp`, and for a `tmux` or `screen` session to recover it from. There was none.
Commit `17491f9` made every run record itself to `results/run/<timestamp>-.../`
in direct response. **A run that is not in this file did not happen as far as
this repository is concerned.**

Fields that could not be recovered are marked `— unrecoverable`. They are left
empty on purpose. Reconstructing them from memory and writing them down as if
they had been recorded is the failure this file exists to prevent.

The same rule applies to evidence destroyed later. The runner binary below was
real evidence when this file was written and was overwritten six days after the
run, by this project's own verification work. That is recorded as a loss rather
than quietly softened into a claim that still sounds supported.

---

## Run 001 — 2026-09-11

| | |
|---|---|
| **Date** | 2026-09-11, approximately 17:15–17:30 EDT |
| **Commit** | `e784134` — includes the joint-order fix `04c56d2`, so the reference motions were in correct IsaacLab order |
| **Runner binary** | built 2026-09-10 14:20 from `runner/` at `a30501f` — observed, but **the artifact is gone**: `runner/target/` is gitignored and that binary was overwritten by a rebuild on 2026-09-17 while verifying the build still configures without network. What survives is the source provenance: `git diff --name-only a30501f -- runner/src runner/cmake` is still empty, so the runner *source* that ran is exactly what is in the tree |
| **Host** | Ubuntu 22.04.5 LTS, x86_64, NVIDIA GeForce RTX 5090 |
| **NIC** | `enp130s0`, the deploy machine's wired port |
| **TensorRT** | 10.13 (`policies/.trt_built_with`) |
| **Policy** | — unrecoverable |
| **Clip** | not applicable — the policy was never armed |
| **Robot support** | **overhead gantry harness**, suspended |

### What happened

The robot was powered and on the DDS bus. The runner reached `Init Done` and
held the fixed stand.

`]` was never pressed. The policy was never armed, no reference clip was
played, and there was no emergency stop to perform. The run ended at the fixed
stand.

### What it establishes

* This bundle talks to a real G1. DDS came up over `enp130s0`, the runner found
  the robot, and the INIT ramp ran to completion on hardware rather than on the
  MuJoCo stand-in.
* The `INIT` → fixed-stand half of `docs/DEPLOY_SEQUENCE.md` has now been
  executed on hardware, under support.

### What it does not establish

* **Nothing about any policy.** `]` was never pressed. Every behavioural number
  in this repository is still simulation, without exception.
* **Nothing about whether the fixed stand holds an unsupported G1.** The robot
  was in a gantry harness the whole time. The relevant measurement is still the
  simulation one: holding `default_angles` at the runner's own gains,
  unsupported, the MuJoCo G1 sits down in about 1.4 s. See
  `docs/DEPLOY_SEQUENCE.md`.
* **Nothing about the emergency stop on hardware.** `O` was not exercised.
* **No timing, no tracking error, no motor temperatures, no torques.** No data
  of any kind was written to disk.

### What changed because of it

* `17491f9` — `run.sh` now writes `results/run/<timestamp>-<policy>-<motion>/`
  on every run: `console.log`, `run_info.txt`, and the runner's full CSV set on
  hardware. CSV logging defaults **on** for a real robot and **off** for
  `--sim`.
* `docs/DEPLOY_DAY.md` was written against this run's shape, and its Phase 5 now
  says to copy the run directory off the machine before the next run.

