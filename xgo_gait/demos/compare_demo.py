#!/usr/bin/env python3
"""compare_demo: A/B gait stabilizers in PyBullet — no hardware needed.

Runs each configuration for --seconds and reports travel, RMS tilt,
mean roll/pitch bias and yaw drift.  Optional weak-leg miscalibration
to exercise the stabilizers.

Examples:
    compare_demo.py                          # crawl, all stabilizers
    compare_demo.py --gait trot --weak-leg 0 --weak 0.75
    compare_demo.py --lag 0.4 --vx 30
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(PKG / 'sim'))

from xgo_gait import (QuadKinematics, GaitController, Gait,          # noqa: E402
                      crawl_params, trot_params, PosturePID,
                      HeadingPID, TouchdownPID, AttitudeLQR, ZmpMPC)


def run(cfg, gait, vx, seconds, lag, weak_leg, weak):
    from sim_env import SimRobot
    kin = QuadKinematics()
    stabs = []
    if 'pid' in cfg:
        stabs.append(PosturePID(kp=0.4, kd=0.06, ki=0.03))
    if 'heading' in cfg:
        stabs.append(HeadingPID())
    if 'odom' in cfg:
        stabs.append(TouchdownPID(kin))
    if 'lqr' in cfg:
        stabs.append(AttitudeLQR())
    if 'mpc' in cfg:
        stabs.append(ZmpMPC(kin))
    with SimRobot(lag_tau=lag, imu_noise=0.5, kin=kin) as dog:
        if weak_leg is not None:
            dog.servo_err[weak_leg] = [1.0, weak, weak]
        ctl = GaitController(dog, kin, Gait(gait), hz=20, mode='joint',
                             stabilizers=stabs)
        ctl.set_velocity(vx, 0, 0)
        ctl.start(ramp_s=2.5)
        t0 = time.monotonic()
        tilts, rolls, yaws = [], [], []
        while ctl.is_running() and time.monotonic() - t0 < seconds:
            time.sleep(0.05)
            dog.request_state()
            t = time.monotonic() - t0
            if t > 3:                          # after ramp-in
                tilts.append(max(abs(dog.imu_deg[0]),
                                 abs(dog.imu_deg[1])))
                rolls.append(dog.imu_deg[0])
                yaws.append(dog.imu_deg[2])
        ctl.stop()
        return (dog.base_pos[0] * 1000, dog.base_pos[1] * 1000,
                np.sqrt(np.mean(np.square(tilts))) if tilts else 0,
                np.mean(rolls) if rolls else 0,
                yaws[-1] - yaws[0] if len(yaws) > 1 else 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gait', choices=['crawl', 'trot'], default='trot')
    ap.add_argument('--vx', type=float, default=30.0)
    ap.add_argument('--seconds', type=float, default=12.0)
    ap.add_argument('--lag', type=float, default=0.4)
    ap.add_argument('--weak-leg', type=int, default=None)
    ap.add_argument('--weak', type=float, default=0.75)
    args = ap.parse_args()

    gait = crawl_params() if args.gait == 'crawl' else trot_params()
    configs = [
        ('open-loop', []),
        ('+pid', ['pid']),
        ('+pid+heading+odom', ['pid', 'heading', 'odom']),
        ('+pid+lqr', ['pid', 'lqr']),
        ('+pid+lqr+mpc', ['pid', 'lqr', 'mpc']),
    ]
    print(f'{args.gait} vx={args.vx} lag={args.lag}s '
          + (f'weak leg {args.weak_leg}={args.weak}'
             if args.weak_leg is not None else ''))
    print(f'{"config":<22}{"dx(mm)":>8}{"dy(mm)":>8}'
          f'{"rmsTilt":>9}{"roll":>7}{"dyaw":>7}')
    for name, cfg in configs:
        dx, dy, rms, mr, dyaw = run(cfg, gait, args.vx, args.seconds,
                                  args.lag, args.weak_leg, args.weak)
        print(f'{name:<22}{dx:+8.0f}{dy:+8.0f}{rms:9.2f}'
              f'{mr:+7.2f}{dyaw:+7.1f}')


if __name__ == '__main__':
    main()
