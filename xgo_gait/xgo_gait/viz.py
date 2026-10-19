"""Matplotlib visualisation: stick-figure posture and gait animation.

Only used offline — not needed on the robot.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from .kinematics import QuadKinematics, HIP_POS_MM, LF, RF, RR, LR

LEG_NAMES = ['LF', 'RF', 'RR', 'LR']


def plot_posture(kin: QuadKinematics, joints, ax=None, title=''):
    """Draw the body + all leg chains for a 4x3 joint array (rad)."""
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
    ax.clear()

    # body outline through the four hip pivots
    corners = [HIP_POS_MM[LF], HIP_POS_MM[RF], HIP_POS_MM[RR],
               HIP_POS_MM[LR], HIP_POS_MM[LF]]
    body = np.asarray(corners)
    ax.plot(body[:, 0], body[:, 1], body[:, 2], 'k-', lw=3)
    ax.scatter(*HIP_POS_MM[:, :3].T, c='k', s=20)

    for leg in range(4):
        pts = kin.chain_body(leg, joints[leg])
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], '-o', ms=4)
        ax.text(*pts[-1], LEG_NAMES[leg])

    ax.set_xlabel('x fwd (mm)')
    ax.set_ylabel('y left (mm)')
    ax.set_zlabel('z up (mm)')
    _set_equal_axes(ax)
    if title:
        ax.set_title(title)
    return ax


def _set_equal_axes(ax):
    lims = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    ctr = lims.mean(axis=1)
    r = (lims[:, 1] - lims[:, 0]).max() / 2
    for axis, c in zip('xyz', ctr):
        getattr(ax, f'set_{axis}lim3d')(c - r, c + r)


def animate_gait(kin: QuadKinematics, gait, vx=30.0, vy=0.0, vyaw=0.0,
                 seconds=4.0, fps=30):
    """Animate the gait's joint trajectories as a stick figure.

    The body is drawn moving 'through' the feet: stance feet stay fixed in
    the world, so we shift the body drawing by integrating the commanded
    velocity — purely cosmetic.
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    dt = 1.0 / fps
    n = int(seconds / dt)
    t = 0.0

    def update(_i):
        nonlocal t
        t += dt
        feet = gait.foot_targets(t, vx, vy, vyaw)
        joints = np.array([kin.legs[l].ik(kin.legs[l].clamp_target(feet[l]))
                           for l in range(4)])
        ax.clear()
        # draw ground and nominal footholds
        ax.scatter(HIP_POS_MM[:, 0], HIP_POS_MM[:, 1],
                   [-kin.legs[0].l1 - kin.legs[0].l2] * 4, c='gray', s=8)
        corners = [HIP_POS_MM[LF], HIP_POS_MM[RF], HIP_POS_MM[RR],
                   HIP_POS_MM[LR], HIP_POS_MM[LF]]
        body = np.asarray(corners)
        ax.plot(body[:, 0], body[:, 1], body[:, 2], 'k-', lw=3)
        for leg in range(4):
            pts = kin.chain_body(leg, joints[leg])
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], '-o', ms=4)
        ax.set_zlim(-120, 40)
        ax.set_xlim(-120, 120)
        ax.set_ylim(-80, 80)
        ax.set_title(f't={t:.2f}s  vx={vx:.0f}mm/s')
        return []

    ani = FuncAnimation(fig, update, frames=n, interval=1000 * dt)
    return ani
