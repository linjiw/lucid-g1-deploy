# Connecting to the robot, and the Unitree SDK

Everything the runner sends and receives goes over DDS on one Ethernet
interface. Get that interface right and there is nothing else to configure; get
it wrong and the runner sits in INIT forever printing
`LowState is not available, waiting for robot to be ready`.

---

## The Unitree SDK, and where it already is

Both halves ship inside this bundle. Nothing is downloaded from Unitree.

| | where | what it is |
|---|---|---|
| **C++** `unitree_sdk2` | `runner/thirdparty/unitree_sdk2/` | headers plus prebuilt `libunitree_sdk2.a` for **x86_64 and aarch64**. This is what the runner links against; `build.sh` picks the right arch. |
| **CycloneDDS** | `runner/thirdparty/unitree_sdk2/thirdparty/` | headers and `libddsc.so` / `libddscxx.so`, **version 0.10.2**, for both arches. The DDS transport itself. |
| **Python** `unitree_sdk2py` | `sdk/unitree_sdk2py/` | vendored verbatim, pure python, not pip-installed. Used by the MuJoCo robot simulator. |

`setup.sh` assembles a `CYCLONEDDS_HOME` out of the vendored CycloneDDS and
builds the `cyclonedds==0.10.2` python bindings against it, into `.venv-sim`.
That is the only reason the python SDK needs anything compiled, and it is why
the versions line up automatically: `unitree_sdk2py` pins 0.10.2 and 0.10.2 is
exactly what the C++ SDK vendors.

Two things to know about that venv:

* It is **python 3.10–3.12**, separate from the bundle's main `.venv`.
  cyclonedds 0.10.2 predates python 3.13; its C extension references
  `_Py_IsFinalizing`, which 3.13 removed, so it builds on 3.13 and then fails at
  import. `setup.sh` picks `python3.10`, `3.11` or `3.12`, in that order.
* Its import roots are `sim/` and `sdk/` via `PYTHONPATH`, set by `env.sh`.

Neither SDK needs root, and neither is installed system-wide.

---

## Wiring

The G1 sits on **`192.168.123.0/24`**. The robot's onboard computer is normally
`192.168.123.161`; the Orin, when you deploy on it, is on the same subnet. Give
your machine a static address on that subnet — `192.168.123.222` is the address
Unitree's own examples use and is free:

```bash
ip link                                        # find the NIC, e.g. enp3s0
sudo ip addr add 192.168.123.222/24 dev enp3s0
sudo ip link set enp3s0 up
ping -c3 192.168.123.161                       # the robot should answer
```

To make it survive a reboot, add it through NetworkManager or netplan rather
than by hand.

Then check DDS traffic is actually flowing before you run anything that moves:

```bash
sudo tcpdump -i enp3s0 -c 20 udp portrange 7400-7500
```

RTPS discovery is on 7400/7401 by default. Silence here means DDS is not
reaching the robot, and no amount of restarting the runner will fix it.

### Passing the interface

Every entry point takes the interface as its first argument or `--iface`:

```bash
bash run.sh --policy deploy_dr                 # auto-detects a 192.168.123.x NIC
bash run.sh --policy deploy_dr --iface enp3s0  # explicit

# the rehearsal on that same NIC, robot powered off and unplugged
bash drill.sh --iface enp3s0 --robot-is-powered-off
```

`run.sh` refuses to guess if it cannot find a `192.168.123.x` interface, which
is deliberate: silently falling back to the wrong NIC is how you end up
commanding a robot you did not mean to.

**`drill.sh` refuses a non-loopback `--iface` outright** — `bash drill.sh
--iface enp3s0` exits 2 — unless `--robot-is-powered-off` is passed as well.
That flag is the override, and it is you asserting that nothing on that wire can
move. The drill earns the gate by being scripted rather than driven:

* it starts `sim/run_robot_sim.py` on the *same* interface, where the simulator
  publishes `rt/lowstate` — on that bus it **is** a robot, so a G1 powered on
  over there makes two of them, both answering the runner;
* it launches the runner with `--disable-crc-check` unconditionally (it has to:
  the simulator computes no CRC), and that same flag also switches off the
  joint-velocity abort — `g1_deploy_onnx_ref.cpp:2832` reads
  `if (body_dq[i] > 35 && !disable_crc_check_)`;
* it sends `]`, optionally `T`, and `O` into the runner's stdin on a timer, with
  **no prompt at all**, where `run.sh` without `--sim` first asks for a
  confirmation it reads from `/dev/tty` (a pipe cannot answer it).

The same reasoning, at length, is in `drill.sh`'s own header under
`THE INTERFACE GATE`; `bash drill.sh --help` prints it. A name the kernel does
not know gets a different refusal ("this machine has no interface named …"), so
a typo is not mistaken for the robot network.

### Loopback

`lo` works for a same-machine rehearsal and is the default in `drill.sh`.
CycloneDDS prints

```
selected interface "lo" is not multicast-capable: disabling multicast
```

which is expected and harmless — both processes are on the same host, so
unicast discovery finds them. **Do not carry a loopback DDS config over to the
robot network**; there multicast works and is what discovery expects.

### Two machines

The simulator and the runner do not have to be on the same host. Put both on the
same real NIC and subnet, then:

```bash
# machine A -- the "robot"
"$LUCID_SIM_PYTHON" sim/run_robot_sim.py --iface enp3s0

# machine B -- the controller
bash run.sh --policy deploy_dr --sim --no-auto-sim --iface enp3s0
```

Machine B needs `--sim` even though the robot is on machine A. `--sim` is what
appends `--disable-crc-check` to the runner's arguments (`run.sh`'s
`EXTRA+=(--disable-crc-check)` line), and that flag is not optional here: the
simulator computes no CRC at all (`grep -ic crc sim/run_robot_sim.py` is 0, run
in this session), so with the check on, `LowStateHandler` hits its
`return` at `g1_deploy_onnx_ref.cpp:2626` before `low_state_buffer_.SetData` at
`:2639` for every packet, and the runner waits in INIT forever. Passing the flag
by hand instead does not work — `run.sh` refuses it without `--sim` (its
`REFUSING: --disable-crc-check without --sim.` branch). `--no-auto-sim` stops
machine B from starting a second MuJoCo robot of its own (`run.sh`'s
`--no-auto-sim) AUTO_SIM=0` case). The explicit `--iface` survives `--sim`,
which only picks a default when none was given
(`[ "$SIM" -eq 1 ] && [ -z "$IFACE" ] && IFACE=lo`). Without `--sim`, machine B
is a `mode     REAL ROBOT` run: it would stop at the `/dev/tty` safety
confirmation, which is right for hardware and wrong for this.

Not `python3`. `env.sh` puts `.venv/bin` first on `PATH`, so `python3` resolves
to the main venv — which carries no `cyclonedds`; only `.venv-sim` does, the
0.10.2 described above. `python3 sim/run_robot_sim.py` therefore dies at
`import cyclonedds` inside `unitree_sdk2py.core.channel`, before the MuJoCo
model is ever loaded. `env.sh` exports `LUCID_SIM_PYTHON` (`.venv-sim/bin/python`)
and the `PYTHONPATH` that makes `sim/` and `sdk/` importable. All three scripts
that start the simulator run it under that interpreter and never `python3`, but
only `run.sh` honours the variable — it reads
`SIMPY="${LUCID_SIM_PYTHON:-$HERE/.venv-sim/bin/python}"`, while `drill.sh` and
`test.sh` both hardcode `SIMPY="$HERE/.venv-sim/bin/python"`. Pointing
`LUCID_SIM_PYTHON` at a different interpreter therefore changes `run.sh` and
not the other two.

This is the closest rehearsal to the real thing short of hardware: real
Ethernet, real multicast discovery, real latency and jitter on the wire.

---

## Releasing Unitree's own controller

The runner does this itself, at startup, before it opens its channels:

```cpp
while (msc_->CheckMode(form, name), !name.empty()) {
  if (msc_->ReleaseMode()) std::cout << "Failed to switch to Release Mode\n";
  sleep(5);
}
```

`MotionSwitcherClient` asks the robot what high-level mode it is in and releases
it, retrying every 5 s until the answer is "nothing". Until that succeeds the
built-in sport-mode controller is also writing `rt/lowcmd`, and two controllers
on one motor bus is exactly as bad as it sounds.

If the runner appears to hang before `Init Done`, watch for
`Failed to switch to Release Mode` — that is this loop, not a crash.

With no robot present `CheckMode` times out after 5 s (`SetTimeout(5.0f)`),
`name` stays empty, and the loop exits immediately. That is why the drill and
`test.sh` reach the control loop with nothing connected.

---

## DDS topics

The same set in simulation and on hardware. `sim/run_robot_sim.py` implements
the robot half of exactly this table.

| topic | direction | payload |
|---|---|---|
| `rt/lowcmd` | runner → robot | 29 motor commands: `q`, `dq`, `kp`, `kd`, `tau` |
| `rt/lowstate` | robot → runner | 29 motor states (`q`, `dq`, `ddq`, `tau_est`), IMU quaternion **wxyz**, gyroscope, accelerometer, `tick` |
| `rt/secondary_imu` | robot → runner | torso IMU |
| `rt/odostate` | robot → runner | base pose and velocity |
| `rt/dex3/left/cmd`, `rt/dex3/right/cmd` | runner → robot | 7 hand joints per side |
| `rt/dex3/left/state`, `rt/dex3/right/state` | robot → runner | hand states |

Domain id **0** throughout. The IDL is `unitree_hg` for the G1 (`unitree_go` is
the quadruped set — do not mix them).

`--disable-crc-check` skips LowState CRC validation. The simulator does not
compute a CRC, so the rehearsal needs it. **Do not pass it on hardware**: it is
the check that catches a corrupted state packet before you act on one.

---

## When it does not connect

| symptom | cause |
|---|---|
| `waiting for robot to be ready`, forever | no LowState. Wrong NIC, no route, robot not powered, or the wrong subnet. |
| `Failed to switch to Release Mode` repeating | the robot is reachable but will not release its high-level controller. Check the robot's own state; it may be in a mode that refuses. |
| `create domain error` | a DDS domain already exists in this process, or the interface name does not exist. Two `ChannelFactoryInitialize` calls on one domain id will do it. |
| `[ERROR] Lost LowState data connection from robot!` | LowState went stale mid-run. `CheckSafety()` forces a stop. Cable, switch, or a wedged sender. This is a terminal stop — see `DEPLOY_SEQUENCE.md`. |
| runs, robot does not move | you never pressed `]`. WAIT_FOR_CONTROL holds the stand and waits. |

---

## Deploying on the robot's own Orin

The onboard computer is **aarch64**, and the pinned TensorRT there is **10.7**,
not the 10.13 required on x86_64. Both numbers come from SONIC's
`installation_deploy.md`, under a `danger` admonition: a different version *"is
known to produce incorrect inference results — the planner will output wrong
motion, which can cause dangerous robot behavior."*

`setup.sh` targets x86_64 and refuses to run on aarch64. On the Orin use
`runner/scripts/install_deps.sh`, which handles the arm64 packages, and pin
TensorRT 10.7 yourself. The vendored `unitree_sdk2` and CycloneDDS already carry
aarch64 binaries, so those need nothing extra. JetPack 6 is required.
