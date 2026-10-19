"""Leg kinematics for the XGO-lite mini, derived from mini2_description.urdf.

Conventions
-----------
Leg frame (matches the firmware's LEG_POS convention so joint-level and
foot-level paths are directly comparable):
    origin  = thigh (x2) joint axis
    +x      = forward
    +y      = left   (body-left; same sign for all four legs)
    +z      = down

Joint angle conventions (our own, radians):
    hip   (servo x3): abduction, rotation about +x. 0 = leg plane vertical;
            positive rotates the plane's "down" direction toward -y.
    thigh (servo x2): pitch of the upper link. 0 = pointing straight down;
            positive swings the link toward +x.
    knee  (servo x1): flexion of the calf relative to the thigh direction.
            0 = straight continuation of the thigh; negative folds the calf
            toward -x (dog-like "elbow back" configuration).

Servo degrees = sign * joint_deg + offset, where sign/offset per servo are
calibration parameters (defaults sign=+1, offset=0 until fitted).
"""

import numpy as np

# ---------------------------------------------------------------------------
# Geometry (mm) — from xgo_gazebo_ws/src/mini2/mini2_description/urdf
# ---------------------------------------------------------------------------
# Hip pivot (servo x3 axis) position in the body frame (x fwd, y left, z up)
HIP_POS_MM = np.array([
    [ 75.0,  22.5, 0.0],   # leg 1 = LF
    [ 75.0, -22.5, 0.0],   # leg 2 = RF
    [-75.0, -22.5, 0.0],   # leg 3 = RR
    [-75.0,  22.5, 0.0],   # leg 4 = LR
])

# Hip pivot -> thigh pivot lateral offset. +y (outboard) for left legs.
D_HIP = 30.988           # mm, magnitude
HIP_OFFSET_SIGN = [+1.0, -1.0, -1.0, +1.0]   # LF, RF, RR, LR

L1 = 59.9                # mm thigh link (thigh pivot -> knee pivot)
L2 = 85.0                # mm calf link (knee pivot -> foot tip, nominal)

# Servo limits (deg) per leg slot [calf x1, thigh x2, hip x3] — xgolite
JOINT_LIMIT_DEG = np.array([
    [-70.0, 50.0],
    [-70.0, 90.0],
    [-30.0, 30.0],
])

# Names for clarity
LF, RF, RR, LR = 0, 1, 2, 3


class LegKinematics:
    """FK/IK for one 3-DOF leg in its leg frame (origin at thigh joint)."""

    def __init__(self, leg_index: int,
                 l1: float = L1, l2: float = L2,
                 d_sign: float | None = None,
                 knee_sign: float = -1.0):
        self.leg = leg_index
        self.l1 = l1
        self.l2 = l2
        self.d = (HIP_OFFSET_SIGN[leg_index] if d_sign is None
                  else d_sign) * D_HIP
        # knee_sign=-1 -> calf folds toward -x (elbow-back), +1 -> forward
        self.knee_sign = knee_sign

    # ------------------------------------------------------------------
    # Internal math is done in the "hip frame" (origin at the hip/abduction
    # pivot); the public API uses the firmware leg frame (origin at the
    # thigh pivot's nominal position) = hip frame shifted by (0, d, 0).
    # ------------------------------------------------------------------
    def fk(self, hip: float, thigh: float, knee: float) -> np.ndarray:
        """Joint angles (rad) -> foot position (x, y, z) mm in leg frame."""
        px = self.l1 * np.sin(thigh) + self.l2 * np.sin(thigh + knee)
        pd = self.l1 * np.cos(thigh) + self.l2 * np.cos(thigh + knee)
        ch, sh = np.cos(hip), np.sin(hip)
        x = px
        y_hip = self.d * ch - pd * sh
        z = self.d * sh + pd * ch
        return np.array([x, y_hip - self.d, z])

    def chain(self, hip: float, thigh: float, knee: float):
        """Return (hip_pivot, thigh_pivot, knee, foot) positions in the
        leg frame (mm)."""
        ch, sh = np.cos(hip), np.sin(hip)
        hip_pt = np.array([0.0, -self.d, 0.0])
        thigh_pt = np.array([0.0, self.d * (ch - 1.0), self.d * sh])
        px_k = self.l1 * np.sin(thigh)
        pd_k = self.l1 * np.cos(thigh)
        knee_pt = thigh_pt + np.array([px_k, -pd_k * sh, pd_k * ch])
        foot = self.fk(hip, thigh, knee)
        return hip_pt, thigh_pt, knee_pt, foot

    def ik(self, target) -> np.ndarray:
        """Foot position (x, y, z) mm -> (hip, thigh, knee) rad.

        Raises ValueError if unreachable.
        """
        x, y, z = np.asarray(target, dtype=float)
        y = y + self.d          # leg frame -> hip frame
        d = self.d
        rho = np.hypot(y, z)
        if rho <= abs(d) + 1e-6:
            raise ValueError(f"target inside hip offset cylinder: rho={rho:.1f}")

        # y cos h + z sin h = d  ->  h = asin(d/rho) - atan2(y, z)
        hip = np.arcsin(np.clip(d / rho, -1.0, 1.0)) - np.arctan2(y, z)
        s = -y * np.sin(hip) + z * np.cos(hip)   # in-plane down coordinate

        r2 = x * x + s * s
        cos_k = (r2 - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)
        cos_k = np.clip(cos_k, -1.0, 1.0)
        knee = self.knee_sign * np.arccos(cos_k)
        thigh = np.arctan2(x, s) - np.arctan2(
            self.l2 * np.sin(knee), self.l1 + self.l2 * np.cos(knee))
        return np.array([hip, thigh, knee])

    # ------------------------------------------------------------------
    def clamp_target(self, target) -> np.ndarray:
        """Pull a foot target into the reachable workspace."""
        x, y, z = np.asarray(target, dtype=float)
        y = y + self.d          # leg frame -> hip frame
        d = self.d

        rho = np.hypot(y, z)
        rho_min = abs(d) + 4.0
        if rho < rho_min:
            scale = rho_min / max(rho, 1e-6)
            y, z = y * scale, z * scale
            rho = rho_min

        hip = np.arcsin(np.clip(d / rho, -1.0, 1.0)) - np.arctan2(y, z)
        s = -y * np.sin(hip) + z * np.cos(hip)

        r = np.hypot(x, s)
        r_max = self.l1 + self.l2 - 4.0
        r_min = abs(self.l1 - self.l2) + 4.0
        if r > r_max:
            x, s = x * r_max / r, s * r_max / r
        elif r < r_min:
            scale = r_min / max(r, 1e-6)
            x, s = x * scale, s * scale

        # map back to (y, z) on the same leg plane
        pd = s
        ch, sh = np.cos(hip), np.sin(hip)
        y = d * ch - pd * sh
        z = d * sh + pd * ch
        return np.array([x, y - d, z])

    # ------------------------------------------------------------------
    def reachable(self, target) -> bool:
        try:
            q = self.ik(target)
        except ValueError:
            return False
        return np.all(np.isfinite(q))


class QuadKinematics:
    """All four legs + body-pose transforms + servo angle mapping."""

    def __init__(self,
                 l1: float = L1, l2: float = L2,
                 servo_sign=None, servo_offset_deg=None,
                 knee_sign: float = -1.0):
        self.legs = [LegKinematics(i, l1, l2, knee_sign=knee_sign)
                     for i in range(4)]
        # joint order per leg in the servo block is [calf, thigh, hip];
        # we store sign/offset in JOINT order [hip, thigh, calf] per leg.
        # default sign guess (-1,-1,+1) came from fitting against the URDF
        # in PyBullet — still a prior; run calibrate_demo.py on the robot.
        if servo_sign is None:
            servo_sign = np.tile(np.array([-1.0, -1.0, 1.0]), (4, 1))
        if servo_offset_deg is None:
            servo_offset_deg = np.zeros((4, 3))
        self.servo_sign = np.asarray(servo_sign, dtype=float)        # (4,3)
        self.servo_offset_deg = np.asarray(servo_offset_deg, float)  # (4,3)

    def set_geometry(self, l1: float, l2: float, dmag: float | None = None):
        """Set shared link lengths; |d| applied with each leg's sign."""
        for leg in self.legs:
            leg.l1, leg.l2 = l1, l2
            if dmag is not None:
                s = np.sign(leg.d) or 1.0
                leg.d = s * dmag

    # ------------------------------------------------------------------
    # leg-frame foot targets <-> joint angles
    # ------------------------------------------------------------------
    def leg_ik(self, leg: int, foot_mm) -> np.ndarray:
        """Return (hip, thigh, knee) rad for a leg-frame foot target."""
        return self.legs[leg].ik(foot_mm)

    def joints_to_servo_deg(self, joints) -> np.ndarray:
        """(4,3) joint angles rad [(hip,thigh,knee)] -> 12 servo degrees in
        servo-block order [11,12,13, 21,22,23, 31,32,33, 41,42,43]."""
        joints = np.asarray(joints, dtype=float)
        out = np.zeros(12)
        for leg in range(4):
            joint_deg = np.degrees(joints[leg])          # (hip, thigh, knee)
            sd = self.servo_sign[leg] * joint_deg \
                + self.servo_offset_deg[leg]
            # servo block order per leg: [calf, thigh, hip]
            out[3 * leg + 0] = sd[2]
            out[3 * leg + 1] = sd[1]
            out[3 * leg + 2] = sd[0]
        return out

    def servo_deg_to_joints(self, servo_deg) -> np.ndarray:
        """Inverse of joints_to_servo_deg -> (4,3) rad (hip, thigh, knee)."""
        servo_deg = np.asarray(servo_deg, dtype=float)
        out = np.zeros((4, 3))
        for leg in range(4):
            sd = np.array([servo_deg[3 * leg + 2],   # hip
                           servo_deg[3 * leg + 1],   # thigh
                           servo_deg[3 * leg + 0]])  # calf
            joint_deg = (sd - self.servo_offset_deg[leg]) \
                / self.servo_sign[leg]
            out[leg] = np.radians(joint_deg)
        return out

    def feet_mm_to_servo_deg(self, feet_mm) -> np.ndarray:
        """4x3 leg-frame foot targets -> 12 servo degrees."""
        joints = np.array([
            self.legs[leg].ik(self.legs[leg].clamp_target(feet_mm[leg]))
            for leg in range(4)])
        return self.joints_to_servo_deg(joints)

    def servo_deg_to_feet_mm(self, servo_deg) -> np.ndarray:
        """12 servo degrees -> 4x3 leg-frame foot positions via FK."""
        joints = self.servo_deg_to_joints(servo_deg)
        return np.array([
            self.legs[leg].fk(*joints[leg]) for leg in range(4)])

    # ------------------------------------------------------------------
    # body-frame handling
    # ------------------------------------------------------------------
    def _leg_origin_body(self, leg: int) -> np.ndarray:
        """Position of the leg-frame origin (thigh-joint nominal) in the
        body frame = hip pivot + (0, d, 0)."""
        return HIP_POS_MM[leg] + np.array([0.0, self.legs[leg].d, 0.0])

    def feet_leg_to_body(self, feet_mm) -> np.ndarray:
        """4x3 leg-frame feet -> body frame (x fwd, y left, z UP)."""
        feet_mm = np.asarray(feet_mm, dtype=float)
        out = np.zeros((4, 3))
        for leg in range(4):
            out[leg] = (self._leg_origin_body(leg)
                        + np.array([feet_mm[leg][0],
                                    feet_mm[leg][1],
                                    -feet_mm[leg][2]]))
        return out

    def feet_body_to_leg(self, feet_body) -> np.ndarray:
        """Inverse of feet_leg_to_body."""
        feet_body = np.asarray(feet_body, dtype=float)
        out = np.zeros((4, 3))
        for leg in range(4):
            rel = feet_body[leg] - self._leg_origin_body(leg)
            out[leg] = np.array([rel[0], rel[1], -rel[2]])
        return out

    def chain_body(self, leg: int, joints) -> np.ndarray:
        """Hip -> thigh -> knee -> foot positions in the body frame (z up).

        joints = (hip, thigh, knee) rad. Returns 4x3 array.
        """
        hip, thigh, knee = joints
        pts_leg = self.legs[leg].chain(hip, thigh, knee)
        origin = self._leg_origin_body(leg)
        to_body = np.array([1.0, 1.0, -1.0])
        return np.array([origin + p * to_body for p in pts_leg])

    @staticmethod
    def _rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    def apply_body_pose(self, feet_mm,
                        translation=(0, 0, 0), rpy=(0, 0, 0)) -> np.ndarray:
        """Keep feet fixed in the world while the body translates/rotates.

        translation: (tx, ty, tz) mm in body frame (z UP positive).
        rpy: (roll, pitch, yaw) rad.
        Returns new leg-frame foot targets.
        """
        feet_body = self.feet_leg_to_body(feet_mm)
        R = self._rot(*rpy)
        T = np.asarray(translation, dtype=float)
        world_feet = (R.T @ (feet_body - T).T).T
        return self.feet_body_to_leg(world_feet)
