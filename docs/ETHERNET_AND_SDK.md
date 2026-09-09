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
bash drill.sh --iface enp3s0                   # the rehearsal, on the same NIC
```

`run.sh` refuses to guess if it cannot find a `192.168.123.x` interface, which
is deliberate: silently falling back to the wrong NIC is how you end up
commanding a robot you did not mean to.

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
python3 sim/run_robot_sim.py --iface enp3s0

# machine B -- the controller
bash run.sh --policy deploy_dr --iface enp3s0
```

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
