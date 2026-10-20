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
    """

    # per-leg signs: (front=+1/rear=-1, left=+1/right=-1)
    SX = np.array([+1.0, +1.0, -1.0, -1.0])   # LF, RF, RR, LR
    SY = np.array([+1.0, -1.0, -1.0, +1.0])

    def __init__(self, kp: float = 0.4, kd: float = 0.05,
                 ki: float = 0.0, max_dz: float = 8.0,
                 rate_tau: float = 0.15):
        """kp: mm of leg-length correction per degree of tilt."""
        self.kp, self.kd, self.ki = kp, kd, ki
        self.max_dz = max_dz
        self.rate = AngleRate(rate_tau)
        self.integ = np.zeros(2)

    def reset(self):
        self.integ[:] = 0.0
        self.rate.prev = None
        self.rate.rate[:] = 0.0

    def update(self, t, dt, robot, gait, feet):
        roll, pitch = robot.imu_deg[0], robot.imu_deg[1]
        rate = self.rate.update(robot.imu_deg, dt)
        err = np.array([roll, pitch])
        self.integ += err * dt
        self.integ = np.clip(self.integ, -50.0, 50.0)

        corr_r = (self.kp * err[0] + self.kd * rate[0]
                  + self.ki * self.integ[0])
        corr_p = (self.kp * err[1] + self.kd * rate[1]
                  + self.ki * self.integ[1])

        dz = -(corr_p * self.SX + corr_r * self.SY)
        dz = np.clip(dz, -self.max_dz, self.max_dz)
        feet = np.asarray(feet, dtype=float).copy()
        feet[:, 2] += dz
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
    """LQR on the linearized inverted-pendulum tilt dynamics (LIPM).

    Per axis (roll about x, pitch about y), treating the body as a point
    mass on a massless support of height h, a horizontal CoM shift u
    moves the centre of pressure:

        theta'' = (g/h) * theta  +  (g/h^2) * u        (small angles)

    u (mm) is realised as an added body translation, i.e. shift the body
    toward the fall direction so the support polygon catches it.

    Gains come from the discrete algebraic Riccati equation — closed-loop
    optimal for the linear plant, which is the point of the comparison
    against hand-tuned PID.
    """

    def __init__(self, dt: float = 0.05, h_mm: float = 95.0,
                 q_theta: float = 1.0, q_rate: float = 0.05,
                 r_u: float = 0.002, max_shift: float = 15.0,
                 rate_tau: float = 0.15):
        self.max_shift = max_shift
        self.rate = AngleRate(rate_tau)
        self.shift = np.zeros(2)
        self.K = self._design(dt, h_mm, q_theta, q_rate, r_u)

    @staticmethod
    def _design(dt, h, q_theta, q_rate, r_u):
        """Discretize the pendulum and solve the DARE -> gain K (1x2)."""
        from scipy.linalg import solve_discrete_are
        w2 = G_MM_S2 / h                    # (g/h), 1/s^2
        A = np.array([[1.0, dt],
                      [w2 * dt, 1.0]])      # theta, theta_dot
        B = np.array([[0.0],
                      [w2 * dt / h]])       # u = body xy shift, mm
        Q = np.diag([q_theta, q_rate])
        R = np.array([[r_u]])
        S = solve_discrete_are(A, B, Q, R)
        return np.linalg.solve(B.T @ S @ B + R, B.T @ S @ A)[0]  # (2,)

    def reset(self):
        self.shift[:] = 0.0
        self.rate.prev = None
        self.rate.rate[:] = 0.0

    def update(self, t, dt, robot, gait, feet):
        rate_deg = self.rate.update(robot.imu_deg, dt)
        # state: [tilt_deg, tilt_rate_deg/s] per axis
        x_r = np.array([robot.imu_deg[0], rate_deg[0]])
        x_p = np.array([robot.imu_deg[1], rate_deg[1]])
        u_r = -self.K @ np.radians(x_r)     # mm lateral CoM shift
        u_p = -self.K @ np.radians(x_p)     # mm fore-aft CoM shift
        # smooth the correction a touch (servo lag dislikes steps)
        a = min(1.0, dt / 0.1)
        self.shift += a * (np.array([float(u_p), float(u_r)])
                           - self.shift)
        self.shift = np.clip(self.shift, -self.max_shift, self.max_shift)
        # body translation (x fwd, y left, z up) — shift body TOWARD the
        # fall side; sign absorbed into K/u convention
        return feet, np.array([self.shift[0], self.shift[1], 0.0])
