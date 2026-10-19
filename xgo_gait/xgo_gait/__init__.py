"""xgo_gait — first-principles quadruped gait for the XGO-lite mini."""

from . import protocol
from .kinematics import (
    QuadKinematics, LegKinematics, HIP_POS_MM, L1, L2, D_HIP,
    LF, RF, RR, LR,
)
from .gait import Gait, GaitParams, trot_params, crawl_params, pace_params
from .calibrate import (
    calibrate_servo_map, load_calibration, CalibrationResult,
)

try:
    from .transport import BleTransport
    from .robot import XGoLite
    from .controller import GaitController
    _HAS_BLE = True
except ImportError:  # bleak not installed (offline / sim-only use)
    _HAS_BLE = False

__all__ = [
    'protocol', 'QuadKinematics', 'LegKinematics', 'HIP_POS_MM', 'L1', 'L2',
    'D_HIP', 'LF', 'RF', 'RR', 'LR', 'Gait', 'GaitParams',
    'trot_params', 'crawl_params', 'pace_params',
    'calibrate_servo_map', 'load_calibration', 'CalibrationResult',
    'BleTransport', 'XGoLite', 'GaitController',
]
