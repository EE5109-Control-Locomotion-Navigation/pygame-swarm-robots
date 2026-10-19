#!/usr/bin/env python3
"""leg_circle_demo: trace a small circle with one foot.

Compares our IK (joint mode, 0x50) against the firmware IK
(foot mode, 0x40). Run with --mode foot first to see the reference,
then --mode joint to see ours.

Usage:
    leg_circle_demo.py [--leg 1..4] [--radius MM] [--mode joint|foot]
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from xgo_gait import XGoLite, QuadKinematics

ADDRESS = "38:18:2B:4F:E9:DA"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--address', default=ADDRESS)
    ap.add_argument('--leg', type=int, default=1, choices=[1, 2, 3, 4])
    ap.add_argument('--radius', type=float, default=10.0)
    ap.add_argument('--z', type=float, default=95.0)
    ap.add_argument('--mode', choices=['joint', 'foot'], default='joint')
    ap.add_argument('--calibrate', default='')
    ap.add_argument('--cycles', type=float, default=2.0)
    ap.add_argument('--hz', type=float, default=20.0)
    args = ap.parse_args()

    leg = args.leg - 1
    kin = QuadKinematics()
    if args.calibrate:
        from xgo_gait import load_calibration
        load_calibration(kin, args.calibrate)
        print('loaded calibration')

    with XGoLite(args.address) as dog:
        dog.set_motor_speed(140)
        nominal = np.array([[0.0, 0.0, args.z]] * 4)
        # establish nominal stance first
        deg = kin.feet_mm_to_servo_deg(nominal)
        cur = np.asarray(dog.read_motors()[:12])
        for a in np.linspace(0, 1, 30):
            w = 0.5 - 0.5 * np.cos(np.pi * a)
            if args.mode == 'joint':
                dog.set_leg_servo_angles(cur + w * (deg - cur))
            else:
                dog.set_foot_positions(nominal)
            time.sleep(0.05)

        print(f'leg {args.leg}: circle r={args.radius}mm '
              f'mode={args.mode} — Ctrl-C to stop')
        t0 = time.monotonic()
        dt = 1.0 / args.hz
        try:
            while time.monotonic() - t0 < args.cycles * 2 * np.pi / 1.0:
                t = time.monotonic() - t0
                ang = t                     # 1 rad/s around the circle
                feet = nominal.copy()
                feet[leg, 0] += args.radius * np.cos(ang)
                feet[leg, 1] += args.radius * np.sin(ang)
                if args.mode == 'joint':
                    dog.set_leg_servo_angles(kin.feet_mm_to_servo_deg(feet))
                else:
                    dog.set_foot_positions(feet)
                time.sleep(dt)
        except KeyboardInterrupt:
            pass
        finally:
            # settle back to nominal then reset
            if args.mode == 'joint':
                dog.set_leg_servo_angles(deg)
                time.sleep(0.5)
            dog.reset()


if __name__ == '__main__':
    main()
