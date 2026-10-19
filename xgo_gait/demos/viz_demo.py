#!/usr/bin/env python3
"""viz_demo: offline stick-figure animation of the gait (no robot needed).

Usage:
    viz_demo.py --gait trot --vx 40
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from xgo_gait import (QuadKinematics, Gait,
                      crawl_params, trot_params, pace_params)
from xgo_gait.viz import animate_gait, plot_posture


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--gait', choices=['crawl', 'trot', 'pace'],
                    default='trot')
    ap.add_argument('--vx', type=float, default=40.0)
    ap.add_argument('--vy', type=float, default=0.0)
    ap.add_argument('--vyaw', type=float, default=0.0)
    ap.add_argument('--seconds', type=float, default=6.0)
    args = ap.parse_args()

    mk = {'crawl': crawl_params, 'trot': trot_params, 'pace': pace_params}
    kin = QuadKinematics()
    gait = Gait(mk[args.gait]())

    ani = animate_gait(kin, gait, vx=args.vx, vy=args.vy, vyaw=args.vyaw,
                       seconds=args.seconds)
    plt.show()


if __name__ == '__main__':
    main()
