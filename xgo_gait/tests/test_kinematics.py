import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from xgo_gait import QuadKinematics, LegKinematics, HIP_POS_MM


@pytest.fixture
def kin():
    return QuadKinematics()


def test_fk_zero(kin):
    # all-zero joints -> foot directly below thigh joint, fully extended
    for leg in range(4):
        f = kin.legs[leg].fk(0.0, 0.0, 0.0)
        assert np.allclose(f, [0.0, 0.0, 59.9 + 85.0], atol=1e-6)


def test_ik_fk_roundtrip(kin):
    rng = np.random.default_rng(0)
    n_ok = 0
    for _ in range(300):
        leg = int(rng.integers(4))
        # sample a target inside a reasonable workspace box
        x = rng.uniform(-20, 20)
        y = rng.uniform(-12, 12)
        z = rng.uniform(70, 105)
        tgt = np.array([x, y, z])
        try:
            q = kin.legs[leg].ik(tgt)
        except ValueError:
            continue
        back = kin.legs[leg].fk(*q)
        assert np.allclose(back, tgt, atol=0.5)
        n_ok += 1
    assert n_ok > 250


def test_ik_rejects_inside_offset(kin):
    # target at the hip axis (leg-frame y = -d) is unreachable
    leg = kin.legs[0]
    with pytest.raises(ValueError):
        leg.ik(np.array([0.0, -leg.d, 1.0]))


def test_clamp_pulls_into_workspace(kin):
    for leg in range(4):
        tgt = np.array([60.0, 0.0, 60.0])   # beyond reach
        c = kin.legs[leg].clamp_target(tgt)
        assert kin.legs[leg].reachable(c)


def test_servo_roundtrip_default_map(kin):
    kin2 = QuadKinematics()
    joints = np.array([[0.1, -0.2, -0.6],
                       [-0.1, 0.3, -0.5],
                       [0.05, 0.1, -0.7],
                       [0.0, 0.0, -0.4]])
    deg = kin2.joints_to_servo_deg(joints)
    back = kin2.servo_deg_to_joints(deg)
    assert np.allclose(back, joints, atol=1e-9)


def test_servo_map_with_sign_offset(kin):
    kin2 = QuadKinematics(
        servo_sign=np.array([[1, -1, 1], [-1, -1, 1],
                             [-1, 1, -1], [1, 1, -1]], float),
        servo_offset_deg=np.array([[5, -3, 10], [-5, 3, -10],
                                   [0, 0, 0], [2, -2, 2]], float))
    joints = np.array([[0.1, -0.2, -0.6],
                       [-0.1, 0.3, -0.5],
                       [0.05, 0.1, -0.7],
                       [0.0, 0.0, -0.4]])
    deg = kin2.joints_to_servo_deg(joints)
    back = kin2.servo_deg_to_joints(deg)
    assert np.allclose(back, joints, atol=1e-9)


def test_body_pose_identity(kin):
    feet = np.array([[0.0, 0.0, 95.0]] * 4)
    out = kin.apply_body_pose(feet, (0, 0, 0), (0, 0, 0))
    assert np.allclose(out, feet)


def test_body_translate_z(kin):
    feet = np.array([[0.0, 0.0, 95.0]] * 4)
    # body +10 mm up with feet fixed -> feet must appear 10 mm deeper (z down)
    out = kin.apply_body_pose(feet, (0, 0, 10.0), (0, 0, 0))
    assert np.allclose(out[:, 2], 105.0)


def test_body_pitch_moves_front_back(kin):
    feet = np.array([[0.0, 0.0, 95.0]] * 4)
    pitch = np.radians(10)   # nose up
    out = kin.apply_body_pose(feet, (0, 0, 0), (0, pitch, 0))
    # front feet must get shorter z (closer to hip), rear feet deeper
    assert out[0, 2] < 95.0 and out[2, 2] > 95.0
    # and all targets should still be solvable
    for leg in range(4):
        q = kin.legs[leg].ik(kin.legs[leg].clamp_target(out[leg]))
        assert np.all(np.isfinite(q))


def test_chain_body_geometry(kin):
    joints = np.zeros(3)
    pts = kin.chain_body(0, joints)   # LF, straight leg
    hip, thigh, knee, foot = pts
    # thigh is outboard of hip by d, foot is directly below thigh
    assert np.isclose(np.linalg.norm(thigh - hip), 30.988)
    assert np.isclose(foot[0], thigh[0])
    assert np.isclose(foot[1], thigh[1])
    assert np.isclose(hip[2] - foot[2], 59.9 + 85.0)


def test_feet_frame_roundtrip(kin):
    feet = np.array([[5.0, 0.0, 95.0], [-3.0, 4.0, 90.0],
                     [0.0, -4.0, 100.0], [8.0, 0.0, 80.0]])
    body = kin.feet_leg_to_body(feet)
    back = kin.feet_body_to_leg(body)
    assert np.allclose(back, feet)
