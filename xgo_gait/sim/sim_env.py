#!/usr/bin/env python3
"""PyBullet simulation of the XGO-Lite, duck-typed to the XGoLite API.

SimRobot implements the subset of the robot interface the controllers use —
set_leg_servo_angles, set_foot_positions, read_motors, imu_deg,
request_state, reset, .t.is_connected() — so GaitController and every
stabilizer in balance.py run identically in sim and on hardware.

The mini2 URDF's legs fold opposite the physical XGO-Lite's (that's the
x-mirror discovered on hardware).  We compensate at the URDF boundary:
external targets keep +x = physical forward; internally the URDF joints
get the mirrored pattern  (hip, thigh, knee) -> (hip, -thigh, -knee),
which mirrors the foot's x exactly (verified in fk).

Faithfulness knobs:
    lag_tau    first-order servo lag (s).  Hardware correlation tests show
               ~0.4 s; keep it so tuned gains transfer.
    latency    extra fixed command delay (s), same reason.
    friction   ground friction; lower it to study slipping.
    imu_noise  std dev (deg) of IMU readouts.

Physics advances by wall-clock time inside each API call (the controller
runs in real time, so the sim does too).
"""

import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(HERE))

from sim_check import (LEG_JOINTS, FOOT_LINK, SPAWN_POS,              # noqa: E402
                       _free_urdf, _pin_base, _foot_world,           # noqa: E402
                       URDF, JOINT_INFO, LINK_IDS)                   # noqa: E402
from xgo_gait import QuadKinematics                                  # noqa: E402
from xgo_gait.calibrate import fit_servo_map                         # noqa: E402

# external -> URDF-model mirror (legs fold opposite on mini2)
X_MIRROR = np.array([-1.0, 1.0, 1.0])
J_MIRROR = np.array([1.0, -1.0, -1.0])     # (hip, thigh, knee)


class _SimTransport:
    def __init__(self):
        self.written = 0

    def write(self, *a, **kw):
        self.written += 1

    def is_connected(self):
        return True


def _fit_urdf_map(pb, bid, kin):
    """Fit  urdf_deg = sign * joints_old_deg + offset  per leg.

    joints_old are model joints for the *URDF* fold direction, i.e.
    kin.ik evaluated at X_MIRROR * target.
    """
    sign = np.zeros((4, 3))
    offset = np.zeros((4, 3))
    xs = (-12.0, 0.0, 12.0)
    zs = (78.0, 92.0, 106.0)
    ys = (-8.0, 0.0, 8.0)
    grid = [np.array([x, y, z]) for x in xs for y in ys for z in zs]
    for leg in range(4):
        link_id = LINK_IDS[FOOT_LINK[leg]]
        pred, meas = [], []
        for tgt in grid:
            t = kin.legs[leg].clamp_target(tgt)
            try:
                qm = kin.legs[leg].ik(t * X_MIRROR)      # old-fold joints
            except ValueError:
                continue
            sol = pb.calculateInverseKinematics(
                bid, link_id, _foot_world(kin, t, leg),
                maxNumIterations=100, residualThreshold=1e-6)
            if sol is None:
                continue
            qu = [sol[JOINT_INFO[n]] for n in LEG_JOINTS[leg]]
            pred.append(np.degrees(qm))
            meas.append(np.degrees(qu))
        sign[leg], offset[leg], _ = fit_servo_map(
            np.asarray(pred), np.asarray(meas))
    return sign, offset


class SimRobot:
    """XGoLite API-compatible PyBullet robot."""

    SIM_HZ = 240.0

    def __init__(self, gui: bool = False, lag_tau: float = 0.4,
                 latency: float = 0.0, friction: float = 1.0,
                 imu_noise: float = 0.3, kin: QuadKinematics | None = None):
        import pybullet as pb
        import pybullet_data

        self.pb = pb
        pb.connect(pb.GUI if gui else pb.DIRECT)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0, 0, -9.81)
        self.plane = pb.loadURDF('plane.urdf')
        pb.changeDynamics(self.plane, -1, lateralFriction=friction)

        tmp = Path('/tmp/xgo_sim_env.urdf')
        _free_urdf(URDF, tmp)
        self.bid = pb.loadURDF(str(tmp), SPAWN_POS, useFixedBase=False)
        for j in range(pb.getNumJoints(self.bid)):
            info = pb.getJointInfo(self.bid, j)
            JOINT_INFO[info[1].decode()] = j
            LINK_IDS[info[12].decode()] = j
            pb.setJointMotorControl2(self.bid, j, pb.VELOCITY_CONTROL,
                                   force=0)

        self.kin = kin or QuadKinematics()
        # per-joint scale error on the servos, (4,3) joint order
        # e.g. servo_err[0] = [1, .8, .8] -> leg 0 sits short (weak leg)
        self.servo_err = np.ones((4, 3))
        self.lag_tau = lag_tau
        self.latency = latency
        self.imu_noise = imu_noise

        self.sign, self.offset = _fit_urdf_map(pb, self.bid, self.kin)
        _pin_base(pb, self.bid)

        self._cmd_deg = self._meas_deg()
        self._lag_deg = self._cmd_deg.copy()
        self._delay_buf = []
        self._last_wall = time.monotonic()
        self.t = _SimTransport()
        self.imu_deg = np.zeros(3)
        self.motor_deg = np.zeros(12)
        self.torque_on = True
        self._update_sensors()

    # ------------------------------------------------------------------
    # physics — advance by elapsed wall time, emulating servo lag
    # ------------------------------------------------------------------
    def _advance(self):
        now = time.monotonic()
        elapsed = now - self._last_wall
        self._last_wall = now
        steps = int(elapsed * self.SIM_HZ)
        if steps <= 0:
            return
        dt = 1.0 / self.SIM_HZ
        a = min(1.0, dt / max(self.lag_tau, 1e-3))
        for _ in range(steps):
            tnow = time.monotonic()
            while self._delay_buf and tnow >= self._delay_buf[0][0]:
                self._cmd_deg = self._delay_buf.pop(0)[1]
            self._lag_deg += a * (self._cmd_deg - self._lag_deg)
            if self.torque_on:
                self._apply_servos(self._lag_deg)
            self.pb.stepSimulation()

    def _apply_servos(self, deg12):
        """Our 12 servo degrees -> URDF joint position targets."""
        pb = self.pb
        joints_new = self.kin.servo_deg_to_joints(deg12)   # (4,3) rad
        joints_old_deg = np.degrees(joints_new) * J_MIRROR  # mirror fold
        joints_old_deg = joints_old_deg * self.servo_err    # servo miscalib
        for leg in range(4):
            q_urdf = np.radians(
                self.sign[leg] * joints_old_deg[leg] + self.offset[leg])
            for k, name in enumerate(LEG_JOINTS[leg]):
                pb.setJointMotorControl2(
                    self.bid, JOINT_INFO[name], pb.POSITION_CONTROL,
                    targetPosition=float(q_urdf[k]),
                    force=0.06, maxVelocity=10.0)

    def _meas_deg(self):
        """URDF joint states -> our 12 servo degrees [calf,thigh,hip]x4."""
        out = np.zeros((4, 3))                  # joint order hip,thigh,knee
        for leg in range(4):
            for k, name in enumerate(LEG_JOINTS[leg]):
                q = np.degrees(self.pb.getJointState(
                    self.bid, JOINT_INFO[name])[0])
                out[leg][k] = (q - self.offset[leg][k]) / self.sign[leg][k]
        joints_new_deg = out * J_MIRROR         # back to physical fold
        servo = (self.kin.servo_sign * joints_new_deg
                 + self.kin.servo_offset_deg)
        flat = np.zeros(12)
        for leg in range(4):
            flat[3 * leg + 0] = servo[leg][2]   # calf
            flat[3 * leg + 1] = servo[leg][1]   # thigh
            flat[3 * leg + 2] = servo[leg][0]   # hip
        return flat

    def _update_sensors(self):
        pos, orn = self.pb.getBasePositionAndOrientation(self.bid)
        eul = np.degrees(self.pb.getEulerFromQuaternion(orn))
        self.imu_deg = np.asarray(eul) + np.random.normal(
            0.0, self.imu_noise, 3)
        self.motor_deg[:12] = self._meas_deg()
        self.base_pos = np.asarray(pos)
        self.base_vel = np.asarray(
            self.pb.getBaseVelocity(self.bid)[0]) * 1000.0   # mm/s world

    # ------------------------------------------------------------------
    # XGoLite-compatible API
    # ------------------------------------------------------------------
    def set_leg_servo_angles(self, deg):
        deg = np.asarray(deg, dtype=float)[:12]
        if self.latency > 0:
            self._delay_buf.append((time.monotonic() + self.latency,
                                    deg.copy()))
        else:
            self._cmd_deg = deg.copy()
        self._advance()
        self._update_sensors()

    def set_foot_positions(self, feet_mm):
        # sim "firmware IK": our kinematics -> servo deg -> same path
        deg = self.kin.feet_mm_to_servo_deg(np.asarray(feet_mm, float))
        self.set_leg_servo_angles(deg)

    def read_motors(self):
        self._advance()
        self._update_sensors()
        return list(self.motor_deg)

    def request_state(self, motors=True, imu=True):
        self._advance()
        self._update_sensors()

    def set_motor_speed(self, speed):
        pass                          # lag_tau models this already

    def set_torque(self, on: bool):
        self.torque_on = bool(on)

    def reset(self):
        # firmware ACTION_RESET -> return to nominal standing pose
        feet = np.tile(np.array([0.0, 0.0, 95.0]), (4, 1))
        self._cmd_deg = self.kin.feet_mm_to_servo_deg(feet)
        self._advance()

    def battery(self):
        return 100.0

    def is_connected(self):
        return True

    # context manager parity with XGoLite
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        try:
            self.pb.disconnect()
        except Exception:
            pass

    def close(self):
        self.__exit__()


def make_sim(gui=False, lag_tau=0.4, **kw) -> SimRobot:
    """Convenience constructor matching XGoLite usage patterns."""
    return SimRobot(gui=gui, lag_tau=lag_tau, **kw)


if __name__ == '__main__':
    # smoke test: crawl against the sim
    sys.path.insert(0, str(PKG))
    from xgo_gait import Gait, GaitController, crawl_params
    with SimRobot() as dog:
        ctl = GaitController(dog, dog.kin, Gait(crawl_params()), hz=20,
                             mode='joint')
        ctl.set_velocity(15, 0, 0)
        ctl.start(ramp_s=2.0)
        t0 = time.monotonic()
        while ctl.is_running() and time.monotonic() - t0 < 10:
            dog.request_state()
            time.sleep(0.25)
            print(f'\rt={time.monotonic()-t0:4.1f} '
                  f'roll{dog.imu_deg[0]:+5.1f} pitch{dog.imu_deg[1]:+5.1f} '
                  f'x={dog.base_pos[0]*1000:+.0f}mm', end='')
        ctl.stop()
        print(f'\ntravelled {dog.base_pos[0]*1000:.0f} mm')
