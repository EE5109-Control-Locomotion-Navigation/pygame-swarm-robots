#!/usr/bin/env python3
"""teleop_demo: drive the custom gait from the keyboard.

    w/s  : forward / backward
    a/d  : strafe left / right
    q/e  : yaw left / right
    space: stop      x: quit (sends reset)

Requires a POSIX terminal (raw mode). Ctrl-C also aborts to reset.
"""

import argparse
import sys
import termios
import time
import tty
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from xgo_gait import (XGoLite, QuadKinematics, GaitController, Gait,
                      crawl_params, trot_params)

ADDRESS = "38:18:2B:4F:E9:DA"
V_STEP = 15.0      # mm/s per press
YAW_STEP = 0.15    # rad/s per press


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--address', default=ADDRESS)
    ap.add_argument('--gait', choices=['crawl', 'trot'], default='crawl')
    ap.add_argument('--mode', choices=['joint', 'foot'], default='joint')
    ap.add_argument('--calibrate', default='')
    ap.add_argument('--hz', type=float, default=20.0)
    args = ap.parse_args()

    kin = QuadKinematics()
    if args.calibrate:
        from xgo_gait import load_calibration
        load_calibration(kin, args.calibrate)

    p = crawl_params() if args.gait == 'crawl' else trot_params()

    with XGoLite(args.address) as dog:
        dog.set_motor_speed(200)
        ctl = GaitController(dog, kin, Gait(p), hz=args.hz, mode=args.mode)
        ctl.on_estop = lambda why: print(f'\n*** e-stop: {why}')
        ctl.start(ramp_s=2.5)

        v = [0.0, 0.0, 0.0]
        print('teleop: w/s fwd/back, a/d strafe, q/e yaw, space stop, x quit')
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while ctl.is_running():
                ch = sys.stdin.read(1)
                if ch == 'w':
                    v[0] += V_STEP
                elif ch == 's':
                    v[0] -= V_STEP
                elif ch == 'a':
                    v[1] += V_STEP
                elif ch == 'd':
                    v[1] -= V_STEP
                elif ch == 'q':
                    v[2] += YAW_STEP
                elif ch == 'e':
                    v[2] -= YAW_STEP
                elif ch == ' ':
                    v = [0.0, 0.0, 0.0]
                elif ch in ('x', '\x03'):
                    break
                else:
                    continue
                ctl.set_velocity(*v)
                print(f'\rv=({v[0]:+5.0f},{v[1]:+5.0f})mm/s '
                      f'yaw={v[2]:+.2f}rad/s   ', end='')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            ctl.stop()
            dog.reset()
            print('\nreset sent')


if __name__ == '__main__':
    main()
