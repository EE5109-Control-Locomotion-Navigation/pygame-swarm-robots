#!/usr/bin/env python3
"""calibrate_demo: fit servo sign/offset using the firmware's LEG_POS.

The robot must be standing on a flat surface and stationary. Each leg in
turn is commanded through a small grid of foot positions (firmware IK);
the resulting servo angles are read back and fitted against our model.

Writes xgo_gait/calibration.npz which the other demos can --calibrate from.
"""

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from xgo_gait import XGoLite, QuadKinematics, calibrate_servo_map

ADDRESS = "38:18:2B:4F:E9:DA"
OUT = Path(__file__).resolve().parent.parent / 'calibration.npz'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--address', default=ADDRESS)
    ap.add_argument('--out', default=str(OUT))
    ap.add_argument('--legs', default='1,2,3,4')
    ap.add_argument('--settle', type=float, default=0.6)
    args = ap.parse_args()

    legs = tuple(int(s) - 1 for s in args.legs.split(','))
    kin = QuadKinematics()

    with XGoLite(args.address) as dog:
        print('firmware:', dog.read_firmware())
        print('battery :', dog.read_battery(), '%')
        input('Robot standing still on flat ground? Press Enter to start,'
              ' Ctrl-C to abort ')
        res = calibrate_servo_map(dog, kin, legs=legs,
                                  settle_s=args.settle)
        print(res.report())
        np.savez(args.out, sign=res.sign, offset=res.offset_deg,
                 rmse=res.rmse_deg,
                 geometry=np.array(res.geometry
                                   if res.geometry else (np.nan,) * 3))
        print('saved ->', args.out)
        dog.reset()


if __name__ == '__main__':
    main()
