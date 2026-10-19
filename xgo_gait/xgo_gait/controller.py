"""Real-time gait control loop.

Each tick: phase -> foot targets (leg frame) -> optional body pose ->
IK -> clamp -> batch write to MOTOR_ANGLE (0x50), or LEG_POS (0x40)
in 'foot' mode.
"""

import threading
import time

import numpy as np

from .gait import Gait, GaitParams, crawl_params
from .kinematics import QuadKinematics, JOINT_LIMIT_DEG


class GaitController:
    MODE_JOINT = 'joint'     # our IK -> servo angles (0x50)
    MODE_FOOT = 'foot'       # foot targets -> firmware IK (0x40)

    def __init__(self, robot, kin: QuadKinematics | None = None,
                 gait: Gait | None = None, hz: float = 20.0,
                 mode: str = MODE_JOINT,
                 limit_margin_deg: float = 2.0,
                 tilt_limit_deg: float = 30.0):
        self.robot = robot
        self.kin = kin or QuadKinematics()
        self.gait = gait or Gait(crawl_params())
        self.hz = hz
        self.mode = mode
        self.limit_margin_deg = limit_margin_deg
        self.tilt_limit_deg = tilt_limit_deg

        self._v = np.zeros(3)              # vx, vy (mm/s), vyaw (rad/s)
        self._body_trans = np.zeros(3)     # mm, z up
        self._body_rpy = np.zeros(3)       # rad
        self._use_body_pose = False
        self._running = threading.Event()
        self._thread = None
        self._t0 = 0.0
        self._ramp_s = 2.0
        self._start_feet = None
        self.last_feet = None              # last commanded 4x3 feet (leg frame)
        self.on_estop = None               # callable(reason:str)

    # ------------------------------------------------------------------
    # commands
    # ------------------------------------------------------------------
    def set_velocity(self, vx_mm_s=0.0, vy_mm_s=0.0, vyaw_rad_s=0.0):
        self._v = np.array([vx_mm_s, vy_mm_s, vyaw_rad_s], dtype=float)

    def set_body_pose(self, translation_mm=None, rpy_rad=None):
        if translation_mm is not None:
            self._body_trans = np.asarray(translation_mm, float)
        if rpy_rad is not None:
            self._body_rpy = np.asarray(rpy_rad, float)
        self._use_body_pose = True

    def clear_body_pose(self):
        self._use_body_pose = False
        self._body_trans[:] = 0.0
        self._body_rpy[:] = 0.0

    # ------------------------------------------------------------------
    # loop
    # ------------------------------------------------------------------
    def start(self, ramp_s: float = 2.0):
        self._ramp_s = ramp_s
        # capture current foot positions for the ramp-in blend
        try:
            deg = self.robot.read_motors()[:12]
            self._start_feet = self.kin.servo_deg_to_feet_mm(deg)
        except Exception:
            self._start_feet = self.gait.nominal_feet()
        self._t0 = time.monotonic()
        self._running.set()
        self._thread = threading.Thread(
            target=self._loop, name='xgo-gait', daemon=True)
        self._thread.start()

    def stop(self, reset: bool = False):
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if reset:
            self.robot.reset()

    def is_running(self) -> bool:
        return self._running.is_set()

    # ------------------------------------------------------------------
    def _loop(self):
        dt = 1.0 / self.hz
        while self._running.is_set() and self.robot.t.is_connected():
            tick = time.monotonic()
            t = tick - self._t0
            try:
                self._step(t)
            except Exception as exc:
                self._estop(f'control step failed: {exc}')
                break
            # IMU tilt watchdog (uses latest polled values)
            roll, pitch = self.robot.imu_deg[0], self.robot.imu_deg[1]
            if max(abs(roll), abs(pitch)) > self.tilt_limit_deg:
                self._estop(f'tilt {roll:.0f}/{pitch:.0f} deg')
                break
            spent = time.monotonic() - tick
            if spent < dt:
                time.sleep(dt - spent)
        self._running.clear()

    def _estop(self, reason: str):
        try:
            self.robot.reset()
        finally:
            if self.on_estop:
                self.on_estop(reason)

    # ------------------------------------------------------------------
    def _step(self, t: float):
        vx, vy, vyaw = self._v
        feet = self.gait.foot_targets(t, vx, vy, vyaw)

        shift = self.gait.support_shift(t, feet)
        if np.any(shift) or self._use_body_pose:
            T = np.array([shift[0] + self._body_trans[0],
                          shift[1] + self._body_trans[1],
                          self._body_trans[2]])
            feet = self.kin.apply_body_pose(
                feet, translation=T, rpy=self._body_rpy)

        # ramp-in: blend measured start pose into the gait
        if self._start_feet is not None:
            a = min(1.0, t / self._ramp_s)
            if a < 1.0:
                w = 0.5 - 0.5 * np.cos(np.pi * a)
                feet = (1 - w) * self._start_feet + w * feet
            else:
                self._start_feet = None

        self.last_feet = feet.copy()

        if self.mode == self.MODE_FOOT:
            self.robot.set_foot_positions(feet)
            return

        deg = self.kin.feet_mm_to_servo_deg(feet)
        self._clamp_servo_deg(deg)
        self.robot.set_leg_servo_angles(deg)

    def _clamp_servo_deg(self, deg: np.ndarray):
        """In-place clamp to joint limits minus margin (deg)."""
        m = self.limit_margin_deg
        lim = JOINT_LIMIT_DEG + np.array([m, -m])
        for leg in range(4):
            for slot in range(3):    # servo slot order [calf, thigh, hip]
                i = 3 * leg + slot
                lo, hi = lim[slot]
                deg[i] = np.clip(deg[i], lo, hi)
