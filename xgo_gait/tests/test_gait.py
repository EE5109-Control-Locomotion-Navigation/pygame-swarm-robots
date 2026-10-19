import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from xgo_gait import (QuadKinematics, Gait,
                      crawl_params, trot_params, pace_params)


def _reachable_all(kin, feet):
    for leg in range(4):
        assert kin.legs[leg].reachable(kin.legs[leg].clamp_target(feet[leg]))


def test_trot_phase_pairs():
    g = Gait(trot_params())
    # LF & RR share phase; RF & LR share phase; half a cycle apart
    off = np.array(g.p.phase_offsets)
    assert np.isclose((off[0] - off[2]) % 1.0, 0.0)
    assert np.isclose((off[1] - off[3]) % 1.0, 0.0)
    assert np.isclose((off[0] - off[1]) % 1.0, 0.5)


def test_crawl_stance_count():
    g = Gait(crawl_params())
    # duty 0.75: exactly one foot in swing at any time (or none at edges)
    for k in range(200):
        t = k * g.p.period / 200.0
        st = g.in_stance(t)
        assert st.sum() >= 3


def test_trajectory_continuity():
    g = Gait(trot_params(period=0.8))
    prev = None
    for k in range(400):
        t = k * g.p.period / 400.0 * 2
        feet = g.foot_targets(t, vx=60.0)
        if prev is not None:
            # foot targets move smoothly (< ~3 mm per 4 ms step)
            assert np.abs(feet - prev).max() < 5.0
        prev = feet


def test_zero_velocity_static():
    g = Gait(crawl_params())
    feet = g.foot_targets(1.234, vx=0.0, vy=0.0, vyaw=0.0)
    assert np.allclose(feet, g.nominal_feet())


def test_forward_motion_stance_sweep():
    g = Gait(trot_params())
    # in stance, a foot sweeps backward (x decreases) as phase advances
    g.p.duty = 0.5
    feet_a = g.foot_targets(0.0, vx=50.0)
    # LF phase=0 at t=0 -> stance starts with +D/2 offset
    assert feet_a[0, 0] > 0.0
    # halfway through stance the offset is ~0, at the end -D/2
    mid = 0.25 * g.p.period
    feet_b = g.foot_targets(mid, vx=50.0)
    assert feet_b[0, 0] < feet_a[0, 0]


def test_all_targets_ik_solvable():
    kin = QuadKinematics()
    for mk in (crawl_params, trot_params, pace_params):
        g = Gait(mk())
        for k in range(120):
            t = k * g.p.period / 120.0 * 3
            feet = g.foot_targets(t, vx=40.0, vy=10.0, vyaw=0.3)
            _reachable_all(kin, feet)


def test_turn_opposite_displacements():
    g = Gait(trot_params())
    D = g.foot_displacements(0.0, 0.0, 0.5)   # pure yaw
    # left feet should get -x? (yaw + = left turn: left feet slower/backward)
    # v_foot_x = vx - vyaw*y_hip ; hips y: LF/LR +, RF/RR -.
    # -> left feet D.x negative, right feet positive (they move opposite)
    assert D[0, 0] < 0 < D[1, 0]     # LF backward, RF forward
    assert D[3, 0] < 0 < D[2, 0]     # LR backward, RR forward


def test_support_shift_direction():
    g = Gait(crawl_params())
    # at t where LF is swinging, support = RF+RR+LR -> centroid shifts
    # body toward the right/rear feet; shift should be nonzero when active
    g.p.body_shift_gain = 0.6
    # find a moment with exactly 3 in stance
    for k in range(400):
        t = k * g.p.period / 400.0
        st = g.in_stance(t)
        if st.sum() == 3:
            s = g.support_shift(t)
            assert np.linalg.norm(s) > 0.1
            break
