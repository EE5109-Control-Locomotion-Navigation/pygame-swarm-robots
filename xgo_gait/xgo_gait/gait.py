"""Gait generation: phase scheduling + foot trajectories.

Leg order throughout: [LF, RF, RR, LR] (firmware legs 1..4).

Foot trajectory (per leg, in the leg frame):
    stance (fraction `duty` of the cycle): the foot sweeps linearly from
        +D/2 to -D/2 along its displacement vector — this is what propels
        the body.
    swing (remainder): the foot returns from -D/2 to +D/2 with a cycloidal
        horizontal profile (zero end velocities) and a raised-cosine
        height profile H*sin^2(pi*sigma) (zero slope at lift-off and
        touchdown).

Commanded body velocity (vx, vy [mm/s], vyaw [rad/s]) maps to a per-foot
stance displacement vector D_i = v_foot_i * T_stance, where
    v_foot_i = (vx - vyaw * y_hip_i,  vy + vyaw * x_hip_i)
and T_stance = duty * period. (v_foot is the *body* velocity relative to
the foot, so the foot moves opposite to it in stance.)
"""

import numpy as np
from dataclasses import dataclass, field

from .kinematics import QuadKinematics


@dataclass
class GaitParams:
    period: float = 1.2                 # s, full cycle
    duty: float = 0.75                  # stance fraction
    step_height: float = 12.0           # mm
    stand_x: float = 0.0                # mm nominal foot x (leg frame)
    stand_y: float = 0.0                # mm nominal foot y
    stand_z: float = 95.0               # mm nominal foot z (down)
    phase_offsets: tuple = (0.5, 0.0, 0.25, 0.75)   # LF, RF, RR, LR
    body_shift_gain: float = 0.0        # 0..1 shift CoG over support polygon
    idle_step: bool = False             # True = step in place at zero velocity
    leg_gain: tuple = (1.0,) * 4        # per-leg stride scale (LF,RF,RR,LR)


def trot_params(**kw) -> GaitParams:
    # hardware-verified on XGO-Lite (L-4.3.7): walks fwd ~30-40 mm/s.
    # stand_x<0 biases CoG forward over the diagonal support pairs —
    # needed for propulsion on this robot given its ~0.4 s servo lag.
    # period 1.6 + step 15 gives a ~40 mm stance sweep at vx=40 with
    # enough swing time for the lagged servos to clear the floor.
    base = dict(period=1.6, duty=0.62, step_height=15.0,
                stand_x=-10.0, stand_z=96.0,
                phase_offsets=(0.0, 0.5, 0.0, 0.5))  # LF+RR / RF+LR
    base.update(kw)
    return GaitParams(**base)


def crawl_params(**kw) -> GaitParams:
    # lateral-sequence walk, hind-before-fore on each side:
    # swing order LR -> LF -> RR -> RF; statically stable for duty >= 0.75
    # hardware-verified: walks fwd at vx=10..20 mm/s
    base = dict(period=3.5, duty=0.75, step_height=14.0,
                stand_z=98.0,
                phase_offsets=(0.5, 0.0, 0.25, 0.75),
                body_shift_gain=0.1)
    base.update(kw)
    return GaitParams(**base)


def pace_params(**kw) -> GaitParams:
    base = dict(period=0.9, duty=0.55,
                phase_offsets=(0.0, 0.5, 0.5, 0.0))   # left / right pairs
    base.update(kw)
    return GaitParams(**base)


def _cycloid(sigma: float) -> float:
    """0->1 smooth profile with zero end velocities."""
    return sigma - np.sin(2.0 * np.pi * sigma) / (2.0 * np.pi)


class Gait:
    def __init__(self, params: GaitParams | None = None,
                 kin: QuadKinematics | None = None):
        self.p = params or crawl_params()
        self._kin = kin or QuadKinematics()

    # ------------------------------------------------------------------
    def nominal_feet(self) -> np.ndarray:
        """4x3 nominal leg-frame foot positions for a level stance."""
        return np.array([[self.p.stand_x, self.p.stand_y, self.p.stand_z]]
                        * 4, dtype=float)

    # ------------------------------------------------------------------
    def foot_displacements(self, vx: float, vy: float, vyaw: float):
        """(vx mm/s, vy mm/s, vyaw rad/s) -> per-foot (dx, dy) mm vectors."""
        t_stance = self.p.duty * self.p.period
        D = np.zeros((4, 2))
        for leg in range(4):
            o = self._kin._leg_origin_body(leg)
            hx, hy = o[0], o[1]
            v_foot_x = vx - vyaw * hy
            v_foot_y = vy + vyaw * hx
            g = self.p.leg_gain[leg]
            D[leg] = (v_foot_x * t_stance * g, v_foot_y * t_stance * g)
        return D

    # ------------------------------------------------------------------
    def phases(self, t: float) -> np.ndarray:
        return (t / self.p.period
                + np.asarray(self.p.phase_offsets)) % 1.0

    def in_stance(self, t: float) -> np.ndarray:
        return self.phases(t) < self.p.duty

    # ------------------------------------------------------------------
    def foot_targets(self, t: float, vx=0.0, vy=0.0, vyaw=0.0) -> np.ndarray:
        """4x3 leg-frame foot targets (mm) at time t."""
        feet = self.nominal_feet()
        D = self.foot_displacements(vx, vy, vyaw)
        ph = self.phases(t)
        beta = self.p.duty
        for leg in range(4):
            dx, dy = D[leg]
            if ph[leg] < beta:
                sigma = ph[leg] / beta
                off = 0.5 - sigma                       # +0.5 -> -0.5 linear
                feet[leg, 0] += dx * off
                feet[leg, 1] += dy * off
            else:
                sigma = (ph[leg] - beta) / (1.0 - beta)
                off = _cycloid(sigma) - 0.5             # -0.5 -> +0.5 smooth
                feet[leg, 0] += dx * off
                feet[leg, 1] += dy * off
                if self.p.idle_step or abs(dx) + abs(dy) > 0.5:
                    feet[leg, 2] -= self.p.step_height \
                        * np.sin(np.pi * sigma) ** 2
        return feet

    # ------------------------------------------------------------------
    def support_shift(self, t: float, feet_mm=None) -> np.ndarray:
        """(dx, dy) mm body offset toward the centroid of the stance feet.

        Returns zeros when all four feet are in stance or the gain is 0.
        """
        if self.p.body_shift_gain <= 0.0:
            return np.zeros(2)
        stance = self.in_stance(t)
        if stance.all() or not stance.any():
            return np.zeros(2)
        feet = feet_mm if feet_mm is not None else self.nominal_feet()
        feet_body = self._kin.feet_leg_to_body(feet)
        centroid = feet_body[stance].mean(axis=0)
        # desired body position: above the support centroid; relative to the
        # centroid of all four leg origins for symmetry
        origins = np.array([self._kin._leg_origin_body(l) for l in range(4)])
        hip_centroid = origins[:, :2].mean(axis=0)
        shift = centroid[:2] - hip_centroid
        return self.p.body_shift_gain * shift
