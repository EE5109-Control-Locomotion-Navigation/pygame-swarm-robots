"""High-level XGO-lite interface built on BleTransport.

Units: angles in degrees, distances in mm, IMU in degrees.
Servo block order (0x50): [11,12,13, 21,22,23, 31,32,33, 41,42,43, 51,52,53]
i.e. legs LF, RF, RR, LR; per leg x1=calf, x2=thigh, x3=hip.
"""

import struct
import threading

from . import protocol as P
from .transport import BleTransport

N_LEG_SERVOS = 12
N_SERVOS = 15


def _slot_for_servo_index(i: int) -> int:
    """MOTOR_LIMIT slot for a 0-based index into the 15-byte angle block."""
    return i % 3 if i < 12 else i - 9


class XGoLite:
    def __init__(self, address: str, transport: BleTransport | None = None):
        self.t = transport or BleTransport(address)
        self._owns_transport = transport is None
        # shadow copy of the full 15-servo angle block (degrees)
        self._motor_deg = [0.0] * N_SERVOS
        self._leg_pos_mm = [[0.0, 0.0, 90.0] for _ in range(4)]
        self._lock = threading.Lock()
        # latest polled state, updated by on_notify when polling is enabled
        self.motor_deg = [0.0] * N_SERVOS
        self.imu_deg = [0.0, 0.0, 0.0]   # roll, pitch, yaw
        self.battery = -1

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def connect(self, timeout: float = 60.0):
        self.t.on_notify = self._on_notify
        if self._owns_transport:
            self.t.start(timeout)
        # snapshot current servo targets as our shadow state
        try:
            self._motor_deg = self.read_motors()
            self.motor_deg = list(self._motor_deg)
        except Exception:
            pass

    def close(self):
        if self._owns_transport:
            self.t.close()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------------
    # register helpers
    # ------------------------------------------------------------------
    def _deg_block_to_bytes(self, deg) -> bytes:
        out = bytearray(N_SERVOS)
        for i in range(N_SERVOS):
            out[i] = P.deg_to_u8(deg[i], P.MOTOR_LIMIT[_slot_for_servo_index(i)])
        return bytes(out)

    # ------------------------------------------------------------------
    # joint-level control (primary path for custom gait)
    # ------------------------------------------------------------------
    def set_leg_servo_angles(self, deg):
        """Set the 12 leg servo targets (degrees, order 11..43).

        Writes only the first 12 bytes of the 0x50 block so the arm
        servos are never touched.
        """
        with self._lock:
            for i in range(N_LEG_SERVOS):
                self._motor_deg[i] = deg[i]
            block = list(self._motor_deg[:N_LEG_SERVOS])
        out = bytes(P.deg_to_u8(block[i], P.MOTOR_LIMIT[i % 3])
                    for i in range(N_LEG_SERVOS))
        self.t.write(P.REG_MOTOR_ANGLE, out)

    def set_servo_angle(self, servo_id: int, deg: float):
        idx = P.SERVO_IDS.index(servo_id)
        with self._lock:
            self._motor_deg[idx] = deg
            block = list(self._motor_deg)
        self.t.write(P.REG_MOTOR_ANGLE, self._deg_block_to_bytes(block))

    def read_motors(self):
        """Read back the 15 servo angles (degrees)."""
        data = self.t.read(P.REG_MOTOR_ANGLE, N_SERVOS)
        out = []
        for i in range(len(data)):
            out.append(P.u8_to_deg(data[i],
                                   P.MOTOR_LIMIT[_slot_for_servo_index(i)]))
        return out

    def set_motor_speed(self, speed: int):
        """Slew rate for direct servo control, 1..255 (higher = faster)."""
        self.t.write(P.REG_MOTOR_SPEED, [max(1, min(255, int(speed)))])

    # ------------------------------------------------------------------
    # foot-level control (firmware IK; fallback + calibration oracle)
    # ------------------------------------------------------------------
    def set_foot_positions(self, feet_mm):
        """feet_mm: 4x3 array-like of (x fwd, y left, z down) mm in leg frames.

        Sends one 12-byte LEG_POS write (all four feet at once).
        """
        buf = bytearray(12)
        for leg in range(4):
            for axis in range(3):
                v = feet_mm[leg][axis]
                wire = v * P.FOOT_X_SIGN if axis == 0 else v
                lo, hi = P.LEG_LIMIT[axis]
                buf[3 * leg + axis] = P.deg_to_u8(wire, (lo, hi))
                self._leg_pos_mm[leg][axis] = v
        self.t.write(P.REG_LEG_POS, bytes(buf))

    def set_foot_position(self, leg: int, xyz_mm):
        feet = [list(p) for p in self._leg_pos_mm]
        feet[leg] = list(xyz_mm)
        self.set_foot_positions(feet)

    # ------------------------------------------------------------------
    # torso / gait-engine registers
    # ------------------------------------------------------------------
    def set_translation(self, x_mm=None, y_mm=None, z_mm=None):
        vals = [x_mm, y_mm, z_mm]
        for i, v in enumerate(vals):
            if v is not None:
                self.t.write(P.REG_TRANSLATION + i,
                             [P.deg_to_u8(v, P.LEG_LIMIT[i])])

    def set_attitude(self, roll=None, pitch=None, yaw=None):
        limits = [(-20, 20), (-10, 10), (-12, 12)]
        for i, v in enumerate((roll, pitch, yaw)):
            if v is not None:
                self.t.write(P.REG_ATTITUDE + i,
                             [P.deg_to_u8(v, limits[i])])

    def set_velocity(self, vx=0.0, vy=0.0, vyaw=0.0):
        """cm/s, cm/s, deg/s through the firmware gait engine."""
        self.t.write(P.REG_VX, [P.centered_to_u8(vx, 25)])
        self.t.write(P.REG_VY, [P.centered_to_u8(vy, 18)])
        self.t.write(P.REG_VYAW, [P.centered_to_u8(vyaw, 100)])

    def set_gait_type(self, gait: str):
        self.t.write(P.REG_GAIT_TYPE,
                     [{'trot': 0, 'walk': 1, 'high_walk': 2}[gait]])

    def set_pace(self, mode: str):
        self.t.write(P.REG_MOVE_MODE,
                     [{'normal': 0, 'slow': 1, 'high': 2}[mode]])

    def mark_time(self, height_mm: float = 0.0):
        if height_mm <= 0:
            self.t.write(P.REG_MARK_TIME, [0])
        else:
            self.t.write(P.REG_MARK_TIME,
                         [max(1, P.deg_to_u8(height_mm, (10, 25)))])

    def action(self, action_id: int):
        self.t.write(P.REG_ACTION, [action_id & 0xFF])

    def reset(self):
        self.action(P.ACTION_RESET)

    # ------------------------------------------------------------------
    # servo torque
    # ------------------------------------------------------------------
    def unload_all(self):
        self.t.write(P.REG_MOTOR_TORQUE, [0x01])

    def load_all(self):
        self.t.write(P.REG_MOTOR_TORQUE, [0x00])

    def unload_leg(self, leg: int):
        self.t.write(P.REG_MOTOR_TORQUE, [0x10 + leg])

    def load_leg(self, leg: int):
        self.t.write(P.REG_MOTOR_TORQUE, [0x20 + leg])

    # ------------------------------------------------------------------
    # state reads
    # ------------------------------------------------------------------
    def read_imu(self):
        """Return (roll, pitch, yaw) degrees."""
        out = []
        for addr in (P.REG_ROLL, P.REG_PITCH, P.REG_YAW):
            data = self.t.read(addr, 4)
            if len(data) < 4:
                raise IOError("short IMU response")
            out.append(struct.unpack_from('<f', data, 0)[0])
        return out

    def read_battery(self):
        data = self.t.read(P.REG_BATTERY, 1)
        return data[0]

    def read_firmware(self):
        data = self.t.read(P.REG_FIRMWARE, 10)
        return data.decode('utf-8', errors='ignore').strip('\0')

    def imu_stabilize(self, on: bool):
        self.t.write(P.REG_IMU_MODE, [1 if on else 0])

    # ------------------------------------------------------------------
    # polling (non-blocking state updates via on_notify)
    # ------------------------------------------------------------------
    def request_state(self, motors=True, imu=True, battery=False):
        if motors:
            self.t.request_async(P.REG_MOTOR_ANGLE, N_SERVOS)
        if imu:
            for addr in (P.REG_ROLL, P.REG_PITCH, P.REG_YAW):
                self.t.request_async(addr, 4)
        if battery:
            self.t.request_async(P.REG_BATTERY, 1)

    def _on_notify(self, addr: int, payload: bytes):
        if addr == P.REG_MOTOR_ANGLE and len(payload) >= N_SERVOS:
            self.motor_deg = [
                P.u8_to_deg(payload[i],
                            P.MOTOR_LIMIT[_slot_for_servo_index(i)])
                for i in range(N_SERVOS)]
        elif addr in (P.REG_ROLL, P.REG_PITCH, P.REG_YAW) and len(payload) >= 4:
            self.imu_deg[addr - P.REG_ROLL] = \
                struct.unpack_from('<f', payload, 0)[0]
        elif addr == P.REG_BATTERY and len(payload) >= 1:
            self.battery = payload[0]
