"""Tests for balance.py (PID/LQR stabilizers) and mpc.py (ZMP MPC)."""
import numpy as np
import pytest

from xgo_gait import (QuadKinematics, Gait, crawl_params, trot_params,
                      PosturePID, HeadingPID, TouchdownPID, AttitudeLQR,
                      AngleRate)
from xgo_gait.mpc import ZmpMPC
from xgo_gait.balance import G_MM_S2


class MockRobot:
    """Minimal duck-typed robot for stabilizer tests."""
    def __init__(self, imu=(0, 0, 0), motor_deg=None):
        self.imu_deg = np.asarray(imu, dtype=float)
        self.motor_deg = np.zeros(15) if motor_deg is None \
            else np.asarray(motor_deg, dtype=float)


@pytest.fixture
def kin():
    return QuadKinematics()


@pytest.fixture
def gait():
    return Gait(crawl_params())


def nominal(kin):
    return np.tile(np.array([0.0, 0.0, 95.0]), (4, 1))


# ---------------------------------------------------------------------------
# PosturePID
# ---------------------------------------------------------------------------
def test_posture_pid_extends_low_side(kin, gait):
    """roll > 0 (left side up) -> right legs extend (+z), left retract."""
    stab = PosturePID(kp=0.5, kd=0.0)
    robot = MockRobot(imu=(5.0, 0.0, 0.0))
    feet, shift = stab.update(0.0, 0.05, robot, gait, nominal(kin))
    dz = feet[:, 2] - 95.0
    # right legs (RF=1, RR=2) should extend downward (+), left retract
    assert dz[1] > 0 and dz[2] > 0
    assert dz[0] < 0 and dz[3] < 0
    assert np.all(shift == 0)


def test_posture_pid_pitch(kin, gait):
    """pitch > 0 (nose up -> rear low) -> rear legs extend."""
    stab = PosturePID(kp=0.5, kd=0.0)
    robot = MockRobot(imu=(0.0, 5.0, 0.0))
    feet, _ = stab.update(0.0, 0.05, robot, gait, nominal(kin))
    dz = feet[:, 2] - 95.0
    assert dz[2] > 0 and dz[3] > 0      # rear legs extend
    assert dz[0] < 0 and dz[1] < 0      # front legs retract


def test_posture_pid_clamped(kin, gait):
    stab = PosturePID(kp=10.0, kd=0.0, max_dz=8.0)
    robot = MockRobot(imu=(60.0, 60.0, 0.0))
    feet, _ = stab.update(0.0, 0.05, robot, gait, nominal(kin))
    assert np.abs(feet[:, 2] - 95.0).max() <= 8.0 + 1e-9


# ---------------------------------------------------------------------------
# HeadingPID
# ---------------------------------------------------------------------------
def test_heading_pid_opposes_drift(kin, gait):
    stab = HeadingPID(kp=0.05, kd=0.0)
    robot = MockRobot(imu=(0, 0, 10.0))
    stab.update(0.0, 0.05, robot, gait, nominal(kin))   # locks heading 10
    robot.imu_deg[2] = 14.0                            # yaw drifted +4deg
    stab.update(0.05, 0.05, robot, gait, nominal(kin))
    assert stab.trim < 0                               # trims the other way


# ---------------------------------------------------------------------------
# LegOdometry
# ---------------------------------------------------------------------------
def test_odometry_recovers_velocity(kin):
    """Feet sweeping backward at v -> odometry sees body moving +v."""
    from xgo_gait.balance import LegOdometry
    odo = LegOdometry(kin, tau=0.05)
    gait = Gait(crawl_params(duty=1.0))      # all legs always in stance
    kin.servo_sign = np.ones((4, 3))
    kin.servo_offset_deg = np.zeros((4, 3))
    v_true = 40.0                            # body +x at 40 mm/s
    f0 = np.tile(np.array([0.0, 0.0, 95.0]), (4, 1))
    robot = MockRobot()
    t = 0.0
    for k in range(40):
        f = f0.copy()
        f[:, 0] -= v_true * t                # feet move backward = body fwd
        robot.motor_deg[:12] = kin.feet_mm_to_servo_deg(f)
        odo.update(robot, gait, t, 0.05)
        t += 0.05
    assert odo.vel[0] == pytest.approx(v_true, rel=0.15)


# ---------------------------------------------------------------------------
# AttitudeLQR — closed-loop on the linear plant
# ---------------------------------------------------------------------------
def test_lqr_stabilizes_pendulum():
    """Compliance plant theta' = -(1/tau)(theta - beta*s) driven by the
    LQR shift-rate law converges: tilt decays and the shift releases."""
    dt = 0.05
    tau, beta = 0.5, np.radians(-0.5)
    stab = AttitudeLQR(dt=dt, tau=tau, beta_deg_per_mm=np.degrees(beta))
    th, s = np.radians(8.0), 0.0
    for _ in range(400):
        ds = -float(stab.K @ np.array([th, s]))
        s += ds
        th += dt * (-(1.0 / tau) * th + (beta / tau) * s)
    assert abs(th) < np.radians(1.0)


def test_lqr_shift_direction(kin, gait):
    """pitch > 0 (nose up = CoM rearward) -> shift body +x (load front
    feet, level the nose)."""
    stab = AttitudeLQR(dt=0.05)
    robot = MockRobot(imu=(0.0, 10.0, 0.0))
    for k in range(10):                     # let the smoother settle
        _, shift = stab.update(k * 0.05, 0.05, robot, gait, nominal(kin))
    assert shift[0] > 0


# ---------------------------------------------------------------------------
# ZmpMPC
# ---------------------------------------------------------------------------
def test_mpc_zmp_within_support(kin, gait):
    mpc = ZmpMPC(kin, horizon_s=1.0, dt_mpc=0.1)
    mpc._com[:] = 0.0
    mpc._solve(0.0, gait)
    lo, hi = mpc._support_box(gait, 0.0)
    assert lo[0] - 1e-6 <= mpc._plan_zmp[0] <= hi[0] + 1e-6
    assert lo[1] - 1e-6 <= mpc._plan_zmp[1] <= hi[1] + 1e-6


def test_mpc_biases_swing_feet(kin):
    """During crawl, the single swing foot gets a foothold adjustment."""
    gait = Gait(crawl_params())
    mpc = ZmpMPC(kin, horizon_s=1.0, dt_mpc=0.1)
    robot = MockRobot()
    feet0 = nominal(kin)
    # find a time with exactly one swing leg
    t = next(tt for tt in np.linspace(0, 3.5, 700)
             if (~np.asarray(gait.in_stance(tt), dtype=bool)).sum() == 1)
    feet, shift = mpc.update(t, 0.05, robot, gait, feet0)
    stance = np.asarray(gait.in_stance(t), dtype=bool)
    # stance feet unchanged; swing foot possibly adjusted
    assert np.allclose(feet[stance], feet0[stance])
    assert np.abs(feet[~stance] - feet0[~stance]).max() <= mpc.max_step + 1e-9


def test_angle_rate_smoothing():
    ar = AngleRate(tau=0.2)
    imu = np.zeros(3)
    ar.update(imu, 0.05)
    imu = np.array([2.0, 0, 0])
    r = ar.update(imu, 0.05)                 # instant rate 40 deg/s raw
    assert 0 < r[0] < 40.0                   # filtered below raw
