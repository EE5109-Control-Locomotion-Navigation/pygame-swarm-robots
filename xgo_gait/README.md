# xgo_gait — first-principles quadruped gait for the XGO-lite mini

A small, dependency-light Python library that implements quadruped
locomotion from scratch on the Luwu XGO-lite mini (non-CM4), talking to
the dog over BLE with the standard XGO serial protocol.

```
gait scheduler  ->  foot trajectories  ->  our inverse kinematics
        ->  servo angle bytes  ->  BLE write to register 0x50 @ ~20 Hz
```

The firmware's own IK is only used for calibration and as a fallback
control path (register 0x40, per-leg Cartesian foot targets).

## Layout

- `xgo_gait/protocol.py` — packet framing, register map, byte<->deg
- `xgo_gait/transport.py` — bleak client on a background asyncio thread
- `xgo_gait/robot.py` — high-level register API (`XGoLite`)
- `xgo_gait/kinematics.py` — leg FK/IK (3-DOF), body-pose transforms,
  servo sign/offset mapping
- `xgo_gait/gait.py` — phase-scheduled gaits (crawl, trot, pace),
  cycloid swing / linear stance trajectories, velocity mapping,
  support-polygon body shift
- `xgo_gait/controller.py` — real-time 20 Hz loop with ramp-in, joint
  clamping and IMU tilt e-stop
- `xgo_gait/calibrate.py` — fits servo sign/offset from LEG_POS read-backs
- `xgo_gait/viz.py` — matplotlib stick figure / gait animation
- `demos/` — `stand_demo`, `leg_circle_demo`, `calibrate_demo`,
  `walk_demo`, `teleop_demo`, `viz_demo` (offline)
- `sim/sim_check.py` — PyBullet sanity check on the mini2 URDF
- `tests/` — pytest suite (offline)

ROS 2 wrapper: `xgo_gait_node` in `xgolite_ws/src/xgo_ros`
(`ros2 launch xgo2_ros xgo_gait_launch.py`), subscribes `/cmd_vel`.

## Geometry / conventions

- Leg frame (matches firmware LEG_POS): origin at thigh joint,
  +x forward, +y left, +z **down**, mm.
- Legs: 1=LF, 2=RF, 3=RR, 4=LR. Servo slots per leg: x1=calf, x2=thigh,
  x3=hip (abduction). Servo block order 11,12,13,21,...,43.
- From `mini2_description.urdf`: hip pivots (±75, ±22.5, 0) mm.
- Calibrated on hardware (L-4.3.7): thigh L1≈59 mm, calf L2≈80 mm,
  |hip→thigh offset| ≈15 mm — effective values from `calibrate_demo`,
  see `calibration.npz`.
- Servo limits (deg, xgolite): calf [−70,50], thigh [−70,90], hip [−30,30].
  Servo byte = 255·(deg−min)/(max−min).

## Hardware findings (XGO-Lite, firmware L-4.3.7)

Verified on the physical robot (`XGOLITE_9878` @ `38:18:2B:4F:E9:DA`):

- **The physical legs are built x-mirrored vs. our nominal IK model**
  (the knees fold the other way). `LegKinematics` negates x at the
  ik/fk/chain boundary, so leg-frame +x = physical forward everywhere
  outside `kinematics.py`. Confirmed by a crawl test that walked
  *backward* before the fix. (Originally misdiagnosed as a firmware
  wire inversion — the firmware LEG_POS convention is the documented
  one, `FOOT_X_SIGN = +1`.)
- **IMU reads must be per-axis**: request 4 bytes at 0x62/0x63/0x64;
  a single 12-byte read at 0x62 returns nothing.
- **BLE write coalescing**: read requests must live in a separate
  FIFO from the latest-wins servo-write mailbox, otherwise state
  polling starves behind a 20 Hz command stream.
- **Servo sign map** (joint order hip,thigh,calf): hip −1 on left
  legs / +1 on right legs, thigh +1, calf +1. Offsets ~+0.7/−8..−18/
  +107 deg — fitted per leg by `calibrate_demo` (joint RMSE ≈ 3–5°).
- **Servo-byte decode scale is approximate**: the measured hip/thigh
  response vs model is <1:1, absorbed by the affine sign+offset fit.
  Effective geometry therefore differs from the URDF values.
- Battery % is register 0x01 (1 byte).

### Latency budget (measured)

The ~0.5 s command-to-motion delay that dominates control design:

| Path segment | Latency |
|---|---|
| BLE link (one way) | ~50 ms (98 ms read round-trip, median) |
| firmware → servo pipeline (dead time) | ~250–350 ms |
| servo move + under-reach deadband | ~200 ms to ~80 % of step |

Step responses show a *fixed* ~300 ms dead time then a fast move —
not a first-order lag, and `MOTOR_SPEED` (0x5C) does not change it
(that register governs built-in actions, not direct 0x50 writes).
The dead time is inside the firmware, so no transport improvement
can remove it; stabilizers must be designed for ~0.5 s of delay.

### USB-serial port (tested, dead end for control)

The robot's USB-C port exposes a UART (`/dev/ttyUSB0`, 115200 baud,
same packet protocol): register *reads* work and are ~2.5x faster
than BLE (~40 ms round-trip).  However actuation *writes* (0x50 servo
angles, ACTION, PERFORM) are silently ignored — it is a debug/
telemetry port, not the control port.  Useful only if faster state
logging is ever needed; BLE remains the only command channel.

## Closed-loop stabilization (`balance.py`, `mpc.py`)

Stabilizers plug into `GaitController(stabilizers=[...])`; each tick they
read the IMU/servo read-back and adjust foot targets and/or body shift
before IK.  Literature mapping:

| Layer | Class | Method | Reference |
|---|---|---|---|
| PID | `PosturePID`, `HeadingPID`, `TouchdownPID` | leg-length modulation, heading trim, Raibert touchdown | Raibert 1986; Pratt VMC 2001 |
| LQR | `AttitudeLQR` | linearized pendulum tilt + DARE | Kajita LIPM |
| MPC | `ZmpMPC` | LIPM-ZMP, box-constrained least squares (`scipy lsq_linear`) | Di Carlo 2018 adapted to position control |

Estimators: `LegOdometry` (body velocity from stance-foot kinematics),
`AngleRate` (filtered IMU derivatives — the firmware exposes no gyro).

```bash
walk_demo.py --gait trot --vx 30 --stabilize pid,heading
walk_demo.py --gait trot --stabilize pid,lqr,mpc
python demos/compare_demo.py --gait trot --weak-leg 0   # sim A/B
```

`sim/sim_env.py` provides `SimRobot`, a PyBullet robot duck-typed to the
`XGoLite` API with a first-order servo-lag model (τ≈0.4 s), command
latency, and `servo_err` miscalibration injection.

### Recommended trot config (hardware-verified)

```bash
demos/walk_demo.py --gait trot --vx 60 --stabilize heading \
    --calibrate calibration.npz
```

vx 50–70 mm/s all work; 60 gives the best stability (rms tilt ~6 deg,
yaw wander ~4 deg over 12 s).  vx 80 exceeds the swing-speed limit:
feet must cover ~79 mm in the 0.6 s swing window and tracking collapses
(~30 % realized).  Faster trots are *more* stable than slow ones — the
body spends less time rocking on each diagonal pair.  Not helpful:
`--body-shift` (lateral CoG motion pumps the rock), motor_speed 255
(breaks servo tracking — keep 200), shorter period at same vx (higher
cadence = more rock cycles/sec).

### Hardware A/B findings (trot, vx 30 mm/s, 12 s runs)

| Config | rms tilt | yaw end |
|---|---|---|
| open-loop | 4.9° | −7.0° drift |
| +heading | — | **~+1°** (held) |
| +posture PID (fast gains) | fell over (e-stop −43°) | held |
| +posture PID (bias-only, smoothed) | 6.4° | held |
| +LQR (compliance plant) | 5.4° | held |
| +MPC footholds | 6.9° | held |

Lesson: **heading hold is a clear win**; per-tick posture feedback
through ~0.5 s of loop delay (servo lag + IMU + BLE) pumps the gait-cycle
rocking.  Posture corrections only help against *sustained* lean — keep
them slow (smoothed error, slew-limited, small ki) or off during trot.

## Use

Python env: `/home/brian/xgolite/venv` (bleak, numpy, matplotlib,
pytest, pybullet already installed).

### On the robot (staged)

Status on this unit: steps 1–5 have all executed on hardware
(BLE link, reads, foot-mode stand, calibration, joint-mode stand,
leg circle, and a first crawl run). Watch battery — servos brown
out below ~25%.

```bash
cd ~/EE5109/xgo_gait
V=~/xgolite/venv/bin/python

# 1. smoke test: reads battery/firmware/servo angles
$V demos/stand_demo.py --hold 2 --mode foot

# 2. fit servo sign/offset + link geometry (robot standing, flat floor)
$V demos/calibrate_demo.py            # writes calibration.npz

# 3. static stand with OUR IK (joint mode)
$V demos/stand_demo.py --mode joint --calibrate calibration.npz

# 4. one foot draws a circle — firmware IK vs ours
$V demos/leg_circle_demo.py --mode foot --leg 1
$V demos/leg_circle_demo.py --mode joint --leg 1 --calibrate calibration.npz

# 5. walking
$V demos/walk_demo.py --gait crawl --vx 30 --calibrate calibration.npz
$V demos/walk_demo.py --gait trot  --vx 50 --calibrate calibration.npz
$V demos/teleop_demo.py --calibrate calibration.npz    # w/s/a/d/q/e
```

Ctrl-C always sends the firmware reset action (0x3E/0xFF).

### Offline

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 $V -m pytest tests -q
$V demos/viz_demo.py --gait trot --vx 40   # stick-figure animation
$V sim/sim_check.py --gait crawl --vx 30 --seconds 8
```

## Safety notes

- All commanded servo angles are clamped to limits − 2° and foot targets
  to the reachable workspace before any write.
- The controller ramps from the measured pose to the gait over ~2.5 s.
- IMU tilt watchdog (>30°) triggers reset; BLE disconnect stops the loop.
- Small workspace: feet reach x ±25 mm, y ±18 mm, z 60–110 mm
  (firmware limit); keep steps modest (±15 mm, height ≤15 mm).

## Teaching notes (mapping to lectures)

- Forward kinematics: `LegKinematics.fk` (hip abduction + planar 2-link).
- Inverse kinematics: `LegKinematics.ik` (circle about hip-offset
  cylinder + law-of-cosines 2-link solve).
- Frame transforms: `QuadKinematics.apply_body_pose` (body translation +
  RPY while feet stay planted).
- Gait scheduling: phase offsets + duty factor (`GaitParams`), swing vs
  stance trajectory shapes in `Gait.foot_targets`.
- Static stability: support-polygon centroid shift
  (`Gait.support_shift`, used by the crawl gait).
- Calibration as least-squares fitting: `calibrate.fit_servo_map` (same
  routine used in `sim/sim_check.py` against the URDF).
