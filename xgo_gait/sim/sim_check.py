#!/usr/bin/env python3
"""Offline sanity check: run our gait in PyBullet on the mini2 URDF.

Two validations:
  1. Geometry: for a grid of our foot targets, solve the URDF chain with
     PyBullet's IK and fit  urdf_deg = sign*model_deg + offset  per joint.
     Small residual = our kinematic model matches the URDF.
  2. Gait: drive the URDF joints with our IK through the fitted map and
     check the robot walks forward without tipping.

The URDF pins base_link to 'world' via a fixed joint — stripped for the
simulation.  Joint probing pins the base so measurements are clean.

Usage:
    sim_check.py [--gait trot|crawl] [--vx MM_S] [--seconds N] [--gui]
"""

import argparse
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
DESC = Path("/home/brian/EE5109/xgo_gazebo_ws/src/mini2/mini2_description")
URDF = DESC / "urdf" / "mini2_description.urdf"

sys.path.insert(0, str(PKG))
from xgo_gait import QuadKinematics, Gait, crawl_params, trot_params  # noqa: E402
from xgo_gait.calibrate import fit_servo_map                          # noqa: E402

# per-leg URDF joints in OUR joint order (hip x3, thigh x2, knee x1)
LEG_JOINTS = {0: ('13_Joint', '12_Joint', '11_Joint'),   # LF
              1: ('23_Joint', '22_Joint', '21_Joint'),   # RF
              2: ('33_Joint', '32_Joint', '31_Joint'),   # RR
              3: ('43_Joint', '42_Joint', '41_Joint')}   # LR
FOOT_LINK = {0: '11_Link', 1: '21_Link', 2: '31_Link', 3: '41_Link'}
SPAWN_POS = (0.0, 0.0, 0.15)
STAND_Z = 95.0

JOINT_INFO = {}
LINK_IDS = {}


def _free_urdf(src: Path, dst: Path):
    text = src.read_text()
    text = text.replace('package://mini2_description/', str(DESC) + '/')
    text = re.sub(r'<link name="world"/>', '', text)
    text = re.sub(r'<joint name="world_fixed".*?</joint>', '', text,
                  flags=re.S)
    dst.write_text(text)
    return dst


def _pin_base(pb, bid):
    pb.resetBasePositionAndOrientation(bid, SPAWN_POS, [0, 0, 0, 1])
    pb.resetBaseVelocity(bid, [0, 0, 0], [0, 0, 0])


def _foot_world(kin, foot_leg_mm, leg):
    """Model foot target (leg frame mm) -> world position (m) with the
    base pinned at SPAWN_POS."""
    origin = kin._leg_origin_body(leg)
    p = origin + np.array([foot_leg_mm[0], foot_leg_mm[1], -foot_leg_mm[2]])
    return (SPAWN_POS + p * 1e-3).tolist()


def fit_model_to_urdf(pb, bid, kin):
    """Fit sign/offset between model joints and URDF joints.

    Returns (sign, offset) arrays (4,3) in degrees, mapping
        urdf_deg = sign * model_deg + offset.
    """
    sign = np.zeros((4, 3))
    offset = np.zeros((4, 3))
    xs = (-12.0, 0.0, 12.0)
    zs = (78.0, 92.0, 106.0)
    ys = (-8.0, 0.0, 8.0)
    grid = [np.array([x, y, z]) for x in xs for y in ys for z in zs]

    for leg in range(4):
        link_id = LINK_IDS[FOOT_LINK[leg]]
        pred, meas = [], []
        for tgt in grid:
            t = kin.legs[leg].clamp_target(tgt)
            try:
                qm = kin.legs[leg].ik(t)
            except ValueError:
                continue
            sol = pb.calculateInverseKinematics(
                bid, link_id, _foot_world(kin, t, leg),
                maxNumIterations=100, residualThreshold=1e-6)
            if sol is None:
                continue
            qu = [sol[JOINT_INFO[n]] for n in LEG_JOINTS[leg]]
            pred.append(np.degrees(qm))
            meas.append(np.degrees(qu))
        pred, meas = np.asarray(pred), np.asarray(meas)
        sign[leg], offset[leg], rmse = fit_servo_map(pred, meas)
        # note: residual includes pybullet-IK branch flips, so this is an
        # approximate map — good enough for a sanity check, NOT for the
        # real robot (use calibrate_demo.py against LEG_POS there)
        print(f'leg {leg}: sign={np.round(sign[leg])} '
              f'offset_deg={np.round(offset[leg], 1)} '
              f'rmse_deg={np.round(rmse, 2)}')
    return sign, offset


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gait', choices=['crawl', 'trot'], default='crawl')
    ap.add_argument('--vx', type=float, default=30.0)
    ap.add_argument('--seconds', type=float, default=8.0)
    ap.add_argument('--gui', action='store_true')
    args = ap.parse_args()

    import pybullet as pb
    import pybullet_data

    tmp = Path('/tmp/xgo_sim.urdf')
    _free_urdf(URDF, tmp)

    pb.connect(pb.GUI if args.gui else pb.DIRECT)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -9.81)
    pb.loadURDF('plane.urdf')
    bid = pb.loadURDF(str(tmp), SPAWN_POS, useFixedBase=False)

    for j in range(pb.getNumJoints(bid)):
        info = pb.getJointInfo(bid, j)
        JOINT_INFO[info[1].decode()] = j
        LINK_IDS[info[12].decode()] = j
        pb.setJointMotorControl2(bid, j, pb.VELOCITY_CONTROL, force=0)

    kin = QuadKinematics()
    gait = Gait(crawl_params() if args.gait == 'crawl' else trot_params())
    gait.p.stand_z = STAND_Z

    print('fitting model<->URDF joint map ...')
    sign, offset = fit_model_to_urdf(pb, bid, kin)

    def model_to_urdf(leg, q_model_rad):
        """our (hip,thigh,knee) rad -> urdf joint targets rad"""
        qd = np.degrees(q_model_rad)
        return np.radians(sign[leg] * qd + offset[leg])

    dt = 1.0 / 240.0
    hz = 20.0
    z_min, z_max = 1e9, -1e9
    tilt_max = 0.0
    ok = True
    x0 = None

    for step in range(int(args.seconds / dt)):
        t = step * dt
        if step % int(1 / (hz * dt)) == 0:
            feet = gait.foot_targets(t, args.vx)
            joints = np.array([
                kin.legs[l].ik(kin.legs[l].clamp_target(feet[l]))
                for l in range(4)])
            for leg in range(4):
                q_urdf = model_to_urdf(leg, joints[leg])
                for k, name in enumerate(LEG_JOINTS[leg]):
                    pb.setJointMotorControl2(
                        bid, JOINT_INFO[name], pb.POSITION_CONTROL,
                        targetPosition=float(q_urdf[k]),
                        force=0.05, maxVelocity=8.0)
        pb.stepSimulation()
        pos, orn = pb.getBasePositionAndOrientation(bid)
        if x0 is None:
            x0 = np.array(pos)
        z_min, z_max = min(z_min, pos[2]), max(z_max, pos[2])
        eul = np.degrees(pb.getEulerFromQuaternion(orn))
        tilt_max = max(tilt_max, abs(eul[0]), abs(eul[1]))
        if pos[2] < 0.03:
            print(f'body hit ground at t={t:.2f}s')
            ok = False
            break

    pos, orn = pb.getBasePositionAndOrientation(bid)
    dx = (pos[0] - x0[0]) * 1000.0
    dy = (pos[1] - x0[1]) * 1000.0
    expect = args.vx * args.seconds
    print(f'travel: dx={dx:.0f}mm dy={dy:.0f}mm (expected ~{expect:.0f}mm), '
          f'base z [{z_min*1000:.0f},{z_max*1000:.0f}]mm, '
          f'max tilt {tilt_max:.1f}deg')
    moved_ok = dx > 0.4 * expect
    print('SIM', 'OK' if ok and tilt_max < 45 and moved_ok else 'CHECK')
    pb.disconnect()


if __name__ == '__main__':
    main()
