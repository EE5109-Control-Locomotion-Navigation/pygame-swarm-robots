#!/usr/bin/env python3
"""walk_demo: run the custom gait controller on the robot.

Examples:
    walk_demo.py --gait crawl --vx 40            # forward crawl, 40 mm/s
    walk_demo.py --gait trot  --vx 60 --seconds 10
    walk_demo.py --gait crawl --vyaw 0.4         # turn in place
    walk_demo.py --mode foot --gait trot --vx 50 # firmware-IK path

Keyboard-free: give --seconds or interrupt with Ctrl-C (sends reset).
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from xgo_gait import (XGoLite, QuadKinematics, GaitController, Gait,
                      crawl_params, trot_params, pace_params)

ADDRESS = "38:18:2B:4F:E9:DA"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--address', default=ADDRESS)
    ap.add_argument('--gait', choices=['crawl', 'trot', 'pace'],
                    default='crawl')
    ap.add_argument('--mode', choices=['joint', 'foot'], default='joint')
    ap.add_argument('--vx', type=float, default=30.0, help='mm/s')
    ap.add_argument('--vy', type=float, default=0.0, help='mm/s')
    ap.add_argument('--vyaw', type=float, default=0.0, help='rad/s')
    ap.add_argument('--step-height', type=float, default=None)
    ap.add_argument('--period', type=float, default=None)
    ap.add_argument('--stand-z', type=float, default=95.0)
    ap.add_argument('--seconds', type=float, default=15.0)
    ap.add_argument('--hz', type=float, default=20.0)
    ap.add_argument('--calibrate', default='')
    args = ap.parse_args()

    kin = QuadKinematics()
    if args.calibrate:
        from xgo_gait import load_calibration
        load_calibration(kin, args.calibrate)
        print('loaded calibration:', args.calibrate)

    mk = {'crawl': crawl_params, 'trot': trot_params, 'pace': pace_params}
    p = mk[args.gait]()
    p.stand_z = args.stand_z
    if args.step_height is not None:
        p.step_height = args.step_height
    if args.period is not None:
        p.period = args.period

    with XGoLite(args.address) as dog:
        print('firmware:', dog.read_firmware(),
              '| battery:', dog.read_battery(), '%')
        dog.set_motor_speed(200)

        ctl = GaitController(dog, kin, Gait(p), hz=args.hz, mode=args.mode)
        ctl.on_estop = lambda why: print(f'\n*** e-stop: {why}')
        ctl.set_velocity(args.vx, args.vy, args.vyaw)

        print(f'{args.gait} via {args.mode} mode at {args.hz} Hz, '
              f'v=({args.vx},{args.vy})mm/s yaw={args.vyaw}rad/s — '
              f'Ctrl-C to stop')
        ctl.start(ramp_s=2.5)
        t0 = time.monotonic()
        try:
            while ctl.is_running() and \
                    time.monotonic() - t0 < args.seconds:
                dog.request_state(motors=True, imu=True)
                time.sleep(0.2)
                m = np.asarray(dog.motor_deg[:12])
                print(f"\rroll {dog.imu_deg[0]:+5.1f} "
                      f"pitch {dog.imu_deg[1]:+5.1f} "
                      f"yaw {dog.imu_deg[2]:+6.1f} "
                      f"servo[{m.min():+6.1f},{m.max():+6.1f}]   ", end='')
        except KeyboardInterrupt:
            print('\ninterrupted')
        finally:
            ctl.stop()
            dog.reset()
            print('\nreset sent')


if __name__ == '__main__':
    main()
