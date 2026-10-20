"""Closed-loop gait stabilization for the XGO-Lite.

The robot is position-controlled: the only actuation authority we have is
where the feet go and where the body sits.  All stabilizers therefore act
in that space — per-leg foot-target corrections and body-shift commands —
layered on top of the open-loop gait pattern.

Stabilizer interface
--------------------
    feet, shift = stab.update(t, dt, robot, gait, feet)

    feet   4x3 leg-frame foot targets (mm); may be modified in place
    shift  (3,) extra body translation (mm, z-up body frame) applied via
           apply_body_pose AFTER the gait's own support shift
"""
import numpy as np

from .kinematics import HIP_OFFSET_SIGN, QuadKinematics

G_MM_S2 = 9810.0        # mm/s^2


# ---------------------------------------------------------------------------
# estimators
# ---------------------------------------------------------------------------
class AngleRate:
    """Filtered derivative of IMU angles (deg -> deg/s).

    The IMU exposes angles only (0x62..0x64); rates are estimated by a
    first-order filtered difference.
    """

    def __init__(self, tau: float = 0.15):
        self.tau = tau
        self.prev = None
        self.rate = np.zeros(3)

    def update(self, imu_deg, dt: float) -> np.ndarray:
        imu = np.asarray(imu_deg, dtype=float)
        if self.prev is not None and dt > 1e-4:
            r = (imu - self.prev) / dt
            a = min(1.0, dt / self.tau)
            self.rate += a * (r - self.rate)
        self.prev = imu
        return self.rate


class LegOdometry:
    """Body-velocity estimate from stance-leg kinematics.

    A planted foot is stationary in the world, so in the leg frame the
    body moves at -d(foot)/dt.  Averaging over stance legs gives a rough
    body velocity (mm/s).  Needs servo read-backs + FK (kin must carry a
    servo map — hardware calibration or the sim's).
    """

    def __init__(self, kin: QuadKinematics, tau: float = 0.25):
        self.kin = kin
        self.tau = tau
        self.prev_feet = None
        self.vel = np.zeros(2)          # vx, vy estimate mm/s

    def update(self, robot, gait, t: float, dt: float) -> np.ndarray:
        try:
            deg = np.asarray(robot.motor_deg[:12], dtype=float)
            feet = self.kin.servo_deg_to_feet_mm(deg)
        except Exception:
            return self.vel
        stance = np.asarray(gait.in_stance(t), dtype=bool)
        if self.prev_feet is not None and dt > 1e-4:
            v = -(feet[stance] - self.prev_feet[stance]) / dt   # (n,3)
            if v.size:
                a = min(1.0, dt / self.tau)
                self.vel += a * (v[:, :2].mean(axis=0) - self.vel)
        self.prev_feet = feet
        return self.vel


# ---------------------------------------------------------------------------
# stabilizer base
# ---------------------------------------------------------------------------
class Stabilizer:
    """Interface: update() -> (feet, body_shift_xyz)."""

    def reset(self):
        pass

    def update(self, t, dt, robot, gait, feet):
        return feet, np.zeros(3)


class PosturePID(Stabilizer):
    """Keep roll/pitch level by modulating leg length (virtual spring
    diagonal, cf. Pratt's Virtual Model Control).

    roll>0 (left up)   -> extend right legs / retract left
    pitch>0 (nose up)  -> extend rear legs / retract front
    +z is down, so +dz extends a leg (lifts its body corner).

    The servo/IMU loop delay (~0.5 s) is comparable to the gait rocking
    period, so the raw tilt error is low-passed (error_tau) and the
    correction slew-limited (out_tau): the PID chases persistent lean,
    never the per-cycle rocking — reacting fast would only pump it.
    """

    # per-leg signs: (front=+1/rear=-1, left=+1/right=-1)
    SX = np.array([+1.0, +1.0, -1.0, -1.0])   # LF, RF, RR, LR
    SY = np.array([+1.0, -1.0, -1.0, +1.0])

    def __init__(self, kp: float = 0.12, kd: float = 0.015,
                 ki: float = 0.0, max_dz: float = 8.0,
                 rate_tau: float = 0.15, error_tau: float = 0.8,
                 out_tau: float = 0.5):
        """kp: mm of leg-length correction per degree of (smoothed) tilt."""
        self.kp, self.kd, self.ki = kp, kd, ki
        self.max_dz = max_dz
        self.error_tau = error_tau
        self.out_tau = out_tau
        self.rate = AngleRate(rate_tau)
        self.integ = np.zeros(2)
        self._err = np.zeros(2)
        self._dz = np.zeros(4)

    def reset(self):
        self.integ[:] = 0.0
        self._err[:] = 0.0
        self._dz[:] = 0.0
        self.rate.prev = None
        self.rate.rate[:] = 0.0

    def update(self, t, dt, robot, gait, feet):
        err_raw = np.array([robot.imu_deg[0], robot.imu_deg[1]])
        rate = self.rate.update(robot.imu_deg, dt)
        a_err = min(1.0, dt / self.error_tau)
        self._err += a_err * (err_raw - self._err)

        self.integ += self._err * dt
        self.integ = np.clip(self.integ, -50.0, 50.0)

        corr_r = (self.kp * self._err[0] + self.kd * rate[0]
                  + self.ki * self.integ[0])
        corr_p = (self.kp * self._err[1] + self.kd * rate[1]
                  + self.ki * self.integ[1])

        target = -(corr_p * self.SX + corr_r * self.SY)
        a_out = min(1.0, dt / self.out_tau)
        self._dz += a_out * (target - self._dz)
        self._dz = np.clip(self._dz, -self.max_dz, self.max_dz)
        feet = np.asarray(feet, dtype=float).copy()
        feet[:, 2] += self._dz
        return feet, np.zeros(3)


class HeadingPID(Stabilizer):
    """Hold a heading by nudging commanded vyaw (yaw-rate trim).

    Locks onto the yaw present at arm time so the robot walks its
    original heading; pushing a new set_velocity updates the lock.
    """

    def __init__(self, kp: float = 0.02, kd: float = 0.005,
                 ki: float = 0.0, max_vyaw: float = 0.4,
                 rate_tau: float = 0.2):
        """kp: rad/s of yaw trim per degree of heading error."""
        self.kp, self.kd, self.ki = kp, kd, ki
        self.max_vyaw = max_vyaw
        self.rate = AngleRate(rate_tau)
        self.integ = 0.0
        self.heading = None
        self.trim = 0.0

    def reset(self):
        self.heading = None
        self.integ = 0.0
        self.trim = 0.0

    def update(self, t, dt, robot, gait, feet):
        yaw = robot.imu_deg[2]
        if self.heading is None:
            self.heading = yaw
        err = np.radians((yaw - self.heading + 180) % 360 - 180)
        rate = np.radians(self.rate.update(robot.imu_deg, dt)[2])
        self.integ = np.clip(self.integ + err * dt, -1.0, 1.0)
        self.trim = float(np.clip(
            -(self.kp * np.degrees(err) + self.kd * np.degrees(rate)
              + self.ki * self.integ),
            -self.max_vyaw, self.max_vyaw))
        return feet, np.zeros(3)

    def velocity_delta(self):
        """(0, 0, dvyaw) — merged into the velocity command by the caller."""
        return np.array([0.0, 0.0, self.trim])


class TouchdownPID(Stabilizer):
    """Raibert-style touchdown control: adjust swing-foot landing point
    to track commanded forward speed.

        dx_td = kv * (v_des - v_est)    (mm, along direction of travel)

    Requires leg odometry (servo read-back).  Only meaningful while
    stepping; correction is applied to feet currently in swing.
    """

    def __init__(self, kin: QuadKinematics, kv: float = 0.08,
                 max_dx: float = 10.0, odo_tau: float = 0.25):
        self.odo = LegOdometry(kin, tau=odo_tau)
        self.kv = kv
        self.max_dx = max_dx
        self.v_des = np.zeros(2)

    def set_desired(self, vx_mm_s, vy_mm_s=0.0):
        self.v_des = np.array([vx_mm_s, vy_mm_s], dtype=float)

    def reset(self):
        self.odo.prev_feet = None
        self.odo.vel[:] = 0.0

    def update(self, t, dt, robot, gait, feet):
        v = self.odo.update(robot, gait, t, dt)
        dv = self.v_des - v
        dx = float(np.clip(self.kv * dv[0], -self.max_dx, self.max_dx))
        dy = float(np.clip(self.kv * dv[1], -self.max_dx, self.max_dx))
        feet = np.asarray(feet, dtype=float).copy()
        for leg in range(4):
            if not gait.in_stance(t)[leg]:
                feet[leg, 0] += dx
                feet[leg, 1] += dy
        return feet, np.zeros(3)


# ---------------------------------------------------------------------------
# LQR — attitude stabilization via CoM shift
# ---------------------------------------------------------------------------
class AttitudeLQR(Stabilizer):
    """LQR attitude stabilization via body shift on a compliance plant.

    The robot is position-controlled: a body xy shift does not produce a
    pendulum torque — it moves the centre of mass relative to the support
    polygon, which the compliant legs translate into a lean change.  The
    honest first-order plant per axis (roll about x, pitch about y):

        theta' = -(1/tau) * theta + (beta/tau) * s
        s'     = ds                                  (control = shift rate)

    state x = [theta, s]; input ds (mm/tick).  Q on theta releases s to
    zero once level — the shift is applied only while needed, never held
    as a permanent offset.  beta < 0 (compliance): shifting the body
    toward a side loads that side's feet and lowers it — so the control
    law shifts toward the HIGH side to level it.  beta = tilt change per
    mm of body shift (conservative default −0.5 deg/mm, tune on hardware).
    """

    def __init__(self, dt: float = 0.05, tau: float = 0.5,
                 beta_deg_per_mm: float = -0.5,
                 q_tilt: float = 1.0, q_shift: float = 0.0,
                 r_rate: float = 0.001, max_shift: float = 15.0,
                 max_step_mm: float = 3.0,
                 err_tau: float = 0.3):
        self.max_shift = max_shift
        self.max_step = max_step_mm
        self.err_tau = err_tau
        self._s = np.zeros(2)              # applied shift per axis (x=pitch,y=roll)
        self._th = np.zeros(2)             # smoothed tilt rad
        self._K = self._design(dt, tau,
                               np.radians(beta_deg_per_mm),
                               q_tilt, q_shift, r_rate)

    @staticmethod
    def _design(dt, tau, beta, q_tilt, q_shift, r_rate):
        """DARE on [theta, s] with rate input ds -> gain K (2,)."""
        from scipy.linalg import solve_discrete_are
        a = np.exp(-dt / tau)
        b = beta * (1.0 - a)
        A = np.array([[a, b],
                      [0.0, 1.0]])
        B = np.array([[0.0], [1.0]])
        Q = np.diag([q_tilt, q_shift])
        R = np.array([[r_rate]])
        S = solve_discrete_are(A, B, Q, R)
        return np.linalg.solve(B.T @ S @ B + R, B.T @ S @ A)[0]

    @property
    def K(self):
        return self._K

    def reset(self):
        self._s[:] = 0.0
        self._th[:] = 0.0

    def update(self, t, dt, robot, gait, feet):
        th_raw = np.array([np.radians(robot.imu_deg[1]),   # x-axis: pitch
                           np.radians(robot.imu_deg[0])])  # y-axis: roll
        a = min(1.0, dt / self.err_tau)
        self._th += a * (th_raw - self._th)

        for i in range(2):
            x = np.array([self._th[i], self._s[i]])
            ds = -float(self._K @ x)
            ds = np.clip(ds, -self.max_step, self.max_step)
            self._s[i] = np.clip(self._s[i] + ds,
                                 -self.max_shift, self.max_shift)
        return feet, np.array([self._s[0], self._s[1], 0.0])
