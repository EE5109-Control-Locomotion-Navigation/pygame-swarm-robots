"""Empirical servo-calibration.

The firmware can position a foot in Cartesian space via LEG_POS (0x40) using
its own IK.  By commanding a small grid of foot targets and reading back the
servo angles (0x50), we get (predicted joint angles, measured servo degrees)
pairs and can fit the per-servo model

    servo_deg = sign * joint_deg + offset      (sign in {+1, -1})

which fills QuadKinematics.servo_sign / servo_offset_deg.

Usage (robot must be powered and standing still, on a flat surface):

    robot = XGoLite(ADDR); robot.connect()
    kin = QuadKinematics()
    result = calibrate_servo_map(robot, kin)
    print(result.report())
"""

import time
from dataclasses import dataclass, field

import numpy as np

from .kinematics import QuadKinematics


@dataclass
class CalibrationResult:
    sign: np.ndarray           # (4,3) +-1 per (leg, joint[hip,thigh,knee])
    offset_deg: np.ndarray     # (4,3)
    rmse_deg: np.ndarray       # (4,3) residual rmse of the fit
    samples: int = 0
    geometry: tuple | None = None   # (L1, L2, |d|) mm if refined

    def report(self) -> str:
        names = ['hip', 'thigh', 'calf']
        lines = []
        for leg in range(4):
            for j in range(3):
                lines.append(
                    f"leg{leg + 1} {names[j]:6s} sign={self.sign[leg, j]:+.0f} "
                    f"off={self.offset_deg[leg, j]:+7.2f} deg "
                    f"rmse={self.rmse_deg[leg, j]:5.2f} deg")
        return "\n".join(lines)


def fit_servo_map(pred_joint_deg, meas_joint_deg):
    """Fit  servo_deg = sign*joint_deg + offset  per joint column.

    pred_joint_deg, meas_joint_deg: (N,3) arrays in degrees, columns in
    JOINT order (hip, thigh, calf/knee).
    Returns (sign, offset, rmse) each (3,).
    """
    sign = np.ones(3)
    offset = np.zeros(3)
    rmse = np.full(3, np.nan)
    for j in range(3):
        best = None
        for s in (1.0, -1.0):
            off = float(np.mean(meas_joint_deg[:, j]
                                - s * pred_joint_deg[:, j]))
            resid = meas_joint_deg[:, j] - (s * pred_joint_deg[:, j] + off)
            err = float(np.sqrt(np.mean(resid ** 2)))
            if best is None or err < best[2]:
                best = (s, off, err)
        sign[j], offset[j], rmse[j] = best
    return sign, offset, rmse


def _sample_grid():
    """Foot targets in the leg frame (mm) for one leg.

    Kept well inside the firmware LEG_LIMIT workspace (x +-25, y +-18,
    z 60..110) and our reachable set.  The y-range is wide enough to
    excite the hip offset so link geometry is identifiable.
    """
    xs = (-10.0, 0.0, 10.0)
    ys = (-10.0, 0.0, 10.0)
    zs = (78.0, 92.0, 106.0)
    grid = []
    for x in xs:
        for y in ys:
            for z in zs:
                grid.append((x, y, z))
    return grid


def load_calibration(kin: QuadKinematics, path):
    """Apply a calibration.npz (sign, offset, optional geometry) to kin."""
    d = np.load(path)
    kin.servo_sign = d['sign']
    kin.servo_offset_deg = d['offset']
    if 'geometry' in d.files and np.all(np.isfinite(d['geometry'])):
        kin.set_geometry(*d['geometry'].tolist())


def refine_geometry(samples, kin: QuadKinematics, signs, offsets,
                    bounds=((50.0, 60.0, 15.0), (75.0, 95.0, 40.0)),
                    verbose=False):
    """Fit shared (L1, L2, |d|) across legs plus per-leg affine offsets.

    samples: dict leg -> (targets (N,3) our frame, meas_j (N,3)
    [hip,thigh,knee] servo deg).  signs: (4,3) +-1 fixed from stage 1.
    Returns (L1, L2, dmag); kin.legs[*].l1/l2/d updated in place.
    """
    from scipy.optimize import least_squares

    legs = sorted(samples)
    p0 = [kin.legs[legs[0]].l1, kin.legs[legs[0]].l2,
          abs(kin.legs[legs[0]].d)]
    # per-leg offsets as nuisance parameters, warm-started from stage 1
    off0 = [offsets[l].tolist() for l in legs]
    x0 = p0 + [v for trio in off0 for v in trio]

    dsign = {l: np.sign(kin.legs[l].d) or 1.0 for l in legs}

    def unpack(x):
        L1, L2, dmag = x[:3]
        offs = {l: np.asarray(x[3 + 3 * i:6 + 3 * i])
                for i, l in enumerate(legs)}
        return L1, L2, dmag, offs

    def resid(x):
        L1, L2, dmag, offs = unpack(x)
        out = []
        for l in legs:
            leg = kin.legs[l]
            l1_, l2_, d_ = leg.l1, leg.l2, leg.d
            leg.l1, leg.l2, leg.d = L1, L2, dsign[l] * dmag
            try:
                tgts, meas = samples[l]
                for t, m in zip(tgts, meas):
                    try:
                        q = np.degrees(leg.ik(t))
                    except ValueError:
                        q = np.full(3, np.nan)
                    out.append(signs[l] * q + offs[l] - m)
            finally:
                leg.l1, leg.l2, leg.d = l1_, l2_, d_
        return np.nan_to_num(np.concatenate(out), nan=1e3)

    lo = list(bounds[0]) + [-60.0] * (3 * len(legs))
    hi = list(bounds[1]) + [140.0] * (3 * len(legs))
    r = least_squares(resid, x0, bounds=(lo, hi), max_nfev=8000)
    L1, L2, dmag, _ = unpack(r.x)
    if verbose:
        print(f"refined geometry: L1={L1:.1f} L2={L2:.1f} |d|={dmag:.1f} mm")
    for l in legs:
        kin.legs[l].l1, kin.legs[l].l2 = L1, L2
        kin.legs[l].d = dsign[l] * dmag
    return L1, L2, dmag


def calibrate_servo_map(robot, kin: QuadKinematics,
                        legs=(0, 1, 2, 3),
                        grid=None, settle_s: float = 0.6,
                        fit_geometry: bool = True,
                        verbose: bool = True) -> CalibrationResult:
    """Fit servo sign/offset per leg joint from LEG_POS read-backs.

    Stage 1 fits servo_deg = sign*joint_deg + offset per joint.
    Stage 2 (fit_geometry=True) refines shared link lengths
    (L1, L2, |d|) against the same samples and re-fits the affine map.
    """
    if grid is None:
        grid = _sample_grid()

    nominal = [0.0, 0.0, 92.0]
    sign = np.ones((4, 3))
    offset = np.zeros((4, 3))
    rmse = np.full((4, 3), np.nan)
    samples = {}

    def fit_leg(leg, pred_joint, meas_j):
        s, o, e = fit_servo_map(pred_joint, meas_j)
        sign[leg], offset[leg], rmse[leg] = s, o, e
        if verbose:
            names = ['hip', 'thigh', 'calf']
            for j in range(3):
                print(f"  {names[j]:6s}: sign={s[j]:+.0f} "
                      f"offset={o[j]:+7.2f} rmse={e[j]:5.2f} deg")

    for leg in legs:
        if verbose:
            print(f"calibrating leg {leg + 1} ...")
        tgts, meas = [], []

        for target in grid:
            t = kin.legs[leg].clamp_target(np.asarray(target, float))
            robot.set_foot_position(leg, list(t))
            time.sleep(settle_s)
            deg = robot.read_motors()
            if len(deg) < 12:
                continue
            tgts.append(t)
            meas.append(np.asarray(deg[3 * leg:3 * leg + 3]))
            # return to nominal between samples
            robot.set_foot_position(leg, nominal)
            time.sleep(settle_s * 0.5)

        if len(tgts) < 3:
            print(f"  leg {leg + 1}: insufficient samples, skipped")
            continue
        tgts = np.asarray(tgts)
        meas = np.asarray(meas)
        # measured servo order is [calf, thigh, hip] -> [hip, thigh, calf]
        meas_j = meas[:, [2, 1, 0]]
        samples[leg] = (tgts, meas_j)

        try:
            pred = np.array([np.degrees(kin.legs[leg].ik(t))
                             for t in tgts])
        except ValueError:
            print(f"  leg {leg + 1}: IK failed on samples, skipped")
            continue
        fit_leg(leg, pred, meas_j)
        robot.set_foot_position(leg, nominal)
        time.sleep(settle_s)

    geometry = None
    if fit_geometry and len(samples) >= 2:
        try:
            geometry = refine_geometry(samples, kin, sign, offset,
                                       verbose=verbose)
        except Exception as e:
            print('geometry refine failed:', e)
        else:
            for leg in samples:
                tgts, meas_j = samples[leg]
                pred = np.array([np.degrees(kin.legs[leg].ik(t))
                                 for t in tgts])
                fit_leg(leg, pred, meas_j)

    kin.servo_sign = sign
    kin.servo_offset_deg = offset
    return CalibrationResult(sign, offset, rmse,
                             samples=len(grid) * len(legs),
                             geometry=geometry)
