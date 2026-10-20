"""Model-predictive gait stabilization via the LIPM/ZMP formulation.

Literature: the canonical convex-MPC result is Di Carlo et al. 2018
(MIT Cheetah 3) — a single-rigid-body QP producing ground reaction
forces for a *torque-controlled* robot.  The XGO-Lite is position-
controlled, so the useful variant here is the older Kajita-lineage
LIPM/ZMP MPC: predict the centre of mass, choose the zero-moment point
inside the evolving support polygon, and realize it by foothold
placement + body shift — the quantities we can actually command.

Model (per axis, linear inverted pendulum about the feet):

    x'' = w^2 (x - zx),    w^2 = g/h
    y'' = w^2 (y - zy)

State s = [x, vx, y, vy] (mm, mm/s); input u_k = [zx_k, zy_k] is the ZMP
position at prediction step k.  The gait's contact schedule supplies a
time-varying box constraint  u_k in support_polygon(t + k*dt)  (shrunk
by a safety margin).  Cost = CoM velocity/position tracking + ZMP
centering.  This is a box-constrained least-squares problem — solved
exactly by scipy.optimize.lsq_linear (BVLS); no QP dependency.

What comes out, per control tick:
    foothold_delta : (x,y) mm added to feet that are about to swing,
                     so they land where the plan wants support next
    body_shift     : (x,y) mm immediate body translation toward the
                     planned ZMP/CoM relationship
"""
import numpy as np
from scipy.optimize import lsq_linear

from .balance import LegOdometry, Stabilizer

G_MM_S2 = 9810.0


class ZmpMPC(Stabilizer):
    """LIPM-ZMP MPC over one gait horizon.

    update() returns (feet, shift) like the other stabilizers; foothold
    deltas are applied to legs that are swinging (or about to swing).
    """

    def __init__(self, kin, horizon_s: float = 1.6, dt_mpc: float = 0.08,
                 h_mm: float = 95.0, margin_mm: float = 8.0,
                 w_vel: float = 1.0, w_pos: float = 0.05,
                 w_zmp: float = 0.02,
                 foothold_gain: float = 0.8, max_step_mm: float = 18.0,
                 max_shift_mm: float = 15.0, rate_hz: float = 10.0):
        self.kin = kin
        self.horizon_s = horizon_s
        self.dt = dt_mpc
        self.h = h_mm
        self.margin = margin_mm
        self.w_vel, self.w_pos, self.w_zmp = w_vel, w_pos, w_zmp
        self.foothold_gain = foothold_gain
        self.max_step = max_step_mm
        self.max_shift = max_shift_mm
        self.solve_dt = 1.0 / rate_hz

        self._w2 = G_MM_S2 / self.h
        self._last_solve = -1e9
        self._com = np.zeros(2)          # CoM xy est, body frame mm
        self._com_v = np.zeros(2)
        self._plan_zmp = np.zeros(2)     # last optimal zmp (body frame)
        self._plan_drift = np.zeros(2)   # com offset from centroid
        self._odo = LegOdometry(kin)     # body-velocity estimate
        self.v_des = np.zeros(2)

    # ------------------------------------------------------------------
    def set_desired(self, vx_mm_s, vy_mm_s=0.0):
        self.v_des = np.array([vx_mm_s, vy_mm_s], dtype=float)

    def _support_box(self, gait, t):
        """(lo, hi) x/y bounds of the stance feet at time t, body frame."""
        stance = np.asarray(gait.in_stance(t), dtype=bool)
        if stance.sum() == 0:
            stance[:] = True
        nominal = gait.nominal_feet()                 # (4,3) leg frame
        feet_b = self.kin.feet_leg_to_body(nominal)   # body frame, z up
        pts = feet_b[stance, :2]
        lo = pts.min(axis=0) + self.margin
        hi = pts.max(axis=0) - self.margin
        # degenerate polygon -> small box around centroid
        for i in range(2):
            if lo[i] >= hi[i]:
                c = pts[:, i].mean()
                lo[i], hi[i] = c - 5.0, c + 5.0
        return lo, hi

    def _solve(self, t, gait):
        """One MPC solve -> updates _plan_zmp, _plan_drift."""
        N = int(round(self.horizon_s / self.dt))
        w2, dt = self._w2, self.dt
        A = np.array([[1, dt, 0, 0],
                      [w2 * dt, 1, 0, 0],
                      [0, 0, 1, dt],
                      [0, 0, w2 * dt, 1]])
        B = np.array([[0, 0],
                      [-w2 * dt, 0],
                      [0, 0],
                      [0, -w2 * dt]])

        # bounds: ZMP inside support polygon at each future step
        lo = np.zeros((N, 2)); hi = np.zeros((N, 2))
        cents = np.zeros((N, 2))
        for k in range(N):
            lo[k], hi[k] = self._support_box(gait, t + k * dt)
            cents[k] = 0.5 * (lo[k] + hi[k])

        # stack:  x_{k+1} = A x_k + B u_k ; cost on vx-vdes, vy-vdes,
        # position vs (centroid + desired travel), and ||u - centroid||
        n_u = 2 * N
        rows, rhs, wts = [], [], []
        Phi = np.eye(4)                  # x_k = Phi_k @ x0 + E_k @ u
        E = np.zeros((4, n_u))
        s0 = np.array([self._com[0], self._com_v[0],
                       self._com[1], self._com_v[1]])
        for k in range(N):
            # propagate
            Phi = A @ Phi
            E_new = np.zeros((4, n_u))
            E_new[:, 2 * k:2 * k + 2] = B
            E = A @ E + E_new
            xk_free = Phi @ s0           # contribution of current state

            # velocity tracking rows
            for ax in range(2):
                row = E[2 * ax + 1]
                rows.append(row)
                rhs.append(self.v_des[ax] - xk_free[2 * ax + 1])
                wts.append(self.w_vel)
            # position tracking toward centroid + desired travel
            ref = cents[k] + self.v_des * (k * dt)
            for ax in range(2):
                row = E[2 * ax]
                rows.append(row)
                rhs.append(ref[ax] - xk_free[2 * ax])
                wts.append(self.w_pos)
            # zmp centering (regularization)
            for ax in range(2):
                row = np.zeros(n_u)
                row[2 * k + ax] = 1.0
                rows.append(row)
                rhs.append(cents[k][ax])
                wts.append(self.w_zmp)

        M = np.asarray(rows) * np.asarray(wts)[:, None]
        r = np.asarray(rhs) * np.asarray(wts)

        res = lsq_linear(M, r, bounds=(lo.ravel(), hi.ravel()),
                         method='bvls', max_iter=40)
        u = res.x
        self._plan_zmp = u[:2]
        self._plan_drift = self._com - cents[0]
        self._last_solve = t

    # ------------------------------------------------------------------
    def update(self, t, dt, robot, gait, feet):
        # CoM state estimate: odometry velocity + leaky integration
        # anchored toward the current support centroid (bounds drift)
        v = self._odo.update(robot, gait, t, dt)
        self._com_v = v.copy()
        lo, hi = self._support_box(gait, t)
        cent = 0.5 * (lo + hi)
        self._com += v * dt
        self._com += 0.5 * (cent - self._com) * dt    # leak toward centroid
        if t - self._last_solve > self.solve_dt:
            try:
                self._solve(t, gait)
            except Exception:
                pass                       # keep last plan on failure

        # foothold: swing feet land toward planned ZMP
        feet = np.asarray(feet, dtype=float).copy()
        offs = self._plan_zmp - cent      # how far to bias touchdown
        offs = np.clip(offs * self.foothold_gain,
                       -self.max_step, self.max_step)
        stance = np.asarray(gait.in_stance(t), dtype=bool)
        for leg in range(4):
            if not stance[leg]:
                feet[leg, 0] += offs[0]
                feet[leg, 1] += offs[1]

        # body shift: nudge CoM toward planned ZMP relationship
        shift = np.clip((self._plan_zmp - cent) * 0.5,
                        -self.max_shift, self.max_shift)
        return feet, np.array([shift[0], shift[1], 0.0])

    def reset(self):
        self._plan_zmp[:] = 0.0
        self._com[:] = 0.0
        self._com_v[:] = 0.0
        self._last_solve = -1e9
