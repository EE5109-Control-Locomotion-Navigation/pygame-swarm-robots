#!/usr/bin/env python3
"""stand_demo: hold a static stance using our own IK -> joint writes (0x50).

Usage:
    stand_demo.py [--address MAC] [--height MM] [--mode joint|foot]
                  [--calibrate calibration.npz]

Options:
    --calibrate FILE   load servo sign/offset fitted by calibrate_demo.py
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from xgo_gait import XGoLite, QuadKinematics, Gait, crawl_params

ADDRESS = "38:18:2B:4F:E9:DA"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--address', default=ADDRESS)
    ap.add_argument('--height', type=float, default=95.0)
    ap.add_argument('--mode', choices=['joint', 'foot'], default='joint')
    ap.add_argument('--calibrate', default='')
    ap.add_argument('--hold', type=float, default=8.0)
    args = ap.parse_args()

    kin = QuadKinematics()
    if args.calibrate:
        from xgo_gait import load_calibration
        load_calibration(kin, args.calibrate)
        print('loaded calibration:', args.calibrate)

    p = crawl_params()
    p.stand_z = args.height
    feet = np.array([[0.0, 0.0, p.stand_z]] * 4)

    with XGoLite(args.address) as dog:
        print('firmware:', dog.read_firmware())
        print('battery :', dog.read_battery(), '%')
        deg0 = dog.read_motors()
        print('servo read-back (deg):', np.round(deg0[:12], 1))
        feet_meas = kin.servo_deg_to_feet_mm(deg0[:12])
        print('FK-estimated feet (mm):\n', np.round(feet_meas, 1))

        dog.set_motor_speed(120)
        if args.mode == 'joint':
            deg = kin.feet_mm_to_servo_deg(feet)
            print('commanded servo deg:', np.round(deg, 1))
            # gentle ramp over ~2 s
            cur = np.asarray(deg0[:12])
            for a in np.linspace(0, 1, 40):
                w = 0.5 - 0.5 * np.cos(np.pi * a)
                dog.set_leg_servo_angles(cur + w * (deg - cur))
                time.sleep(0.05)
        else:
            dog.set_foot_positions(feet)

        print(f'holding {args.hold:.0f}s ... (Ctrl-C aborts to reset)')
        t0 = time.monotonic()
        try:
            while time.monotonic() - t0 < args.hold:
                if args.mode == 'joint':
                    dog.set_leg_servo_angles(deg)
                else:
                    dog.set_foot_positions(feet)
                dog.request_state()
                time.sleep(0.05)
                print(f"\rroll {dog.imu_deg[0]:+5.1f} pitch {dog.imu_deg[1]:+5.1f}"
                      f" yaw {dog.imu_deg[2]:+5.1f}   ", end='')
        except KeyboardInterrupt:
            print('\nresetting ...')
            dog.reset()


if __name__ == '__main__':
    main()
