"""XGO serial/BLE protocol: packet framing and value encoding.

Frame format (write, no response, mode=0x01):
    0x55 0x00 LEN 0x01 ADDR data... CHECKSUM 0x00 0xAA
    LEN      = len(data) + 8   (total bytes in frame)
    CHECKSUM = 255 - ((LEN + mode + ADDR + sum(data)) % 256)

Read request (mode=0x02):
    0x55 0x00 0x09 0x02 ADDR READ_LEN CHECKSUM 0x00 0xAA

Read response (type=0x12):
    0x55 0x00 LEN 0x12 ADDR data[LEN-8] CHECKSUM 0x00 0xAA
"""

# ---------------------------------------------------------------------------
# Register addresses (XGO memory table)
# ---------------------------------------------------------------------------
REG_BATTERY = 0x01          # 1 byte, read, 0-100
REG_PERFORM = 0x03          # 1 byte, write: 0 normal, 1 loop actions
REG_FIRMWARE = 0x07         # 10 bytes, read
REG_GAIT_TYPE = 0x09        # 1 byte, write: 0 trot, 1 walk, 2 high_walk
REG_BT_NAME = 0x13          # 10 bytes, write
REG_MOTOR_TORQUE = 0x20     # 1 byte, write: 0x00 load all, 0x01 unload all,
                            #                0x10+leg unload leg, 0x20+leg load leg
REG_VX = 0x30               # 1 byte, write (u8, 128=0)
REG_VY = 0x31               # 1 byte
REG_VYAW = 0x32             # 1 byte
REG_TRANSLATION = 0x33      # 3 bytes: body x, y, z (mm)
REG_ATTITUDE = 0x36         # 3 bytes: roll, pitch, yaw (deg)
REG_PERIODIC_ROT = 0x39     # 3 bytes
REG_MARK_TIME = 0x3C        # 1 byte
REG_MOVE_MODE = 0x3D        # 1 byte: 0 normal, 1 slow, 2 high
REG_ACTION = 0x3E           # 1 byte: preset action id, 0xFF = reset
REG_MOVE_TO = 0x3F          # 3 bytes
REG_LEG_POS = 0x40          # 12 bytes: foot (x, y, z) per leg, firmware does IK
REG_MOTOR_ANGLE = 0x50      # 15 bytes: target angle per servo (12 leg + 3 arm)
REG_MOTOR_SPEED = 0x5C      # 1 byte: slew rate for direct servo writes (1-255)
REG_IMU_MODE = 0x61         # 1 byte: self-stabiliser on/off
REG_ROLL = 0x62             # 4-byte float (little-endian), read
REG_PITCH = 0x63            # 4-byte float, read
REG_YAW = 0x64              # 4-byte float, read
REG_PERIODIC_TRAN = 0x80    # 3 bytes

# Servo ids in the order they appear in the 0x50 block (index 0..14)
SERVO_IDS = [11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43, 51, 52, 53]

# Servo angle limits (degrees) per slot, for xgolite firmware:
#   slot 0 = "x1" calf/knee, slot 1 = "x2" thigh, slot 2 = "x3" hip abduction
#   slots 3..5 = arm servos 51, 52, 53
MOTOR_LIMIT = [
    [-70.0, 50.0],   # x1 calf
    [-70.0, 90.0],   # x2 thigh
    [-30.0, 30.0],   # x3 hip
    [-65.0, 65.0],   # 51
    [-70.0, 60.0],   # 52
    [-90.0, 105.0],  # 53
]

# Foot workspace in the firmware leg frame (mm): x fwd, y left, z down
LEG_LIMIT = [(-25.0, 25.0), (-18.0, 18.0), (60.0, 110.0)]

# Empirically (L-4.3.7 firmware, XGO-Lite), commanding LEG_POS +x moves the
# foot toward the tail -- the wire convention is opposite the documented
# "+x forward" and our model convention.  Apply this sign to x on the wire.
FOOT_X_SIGN = -1.0

ACTION_RESET = 0xFF


def build_write_packet(addr: int, data: bytes) -> bytes:
    """Build a write (mode=0x01) packet for register `addr`."""
    data = bytes(data)
    length = len(data) + 8
    checksum = (255 - (length + 0x01 + addr + sum(data)) % 256) & 0xFF
    return bytes([0x55, 0x00, length, 0x01, addr]) + data + bytes(
        [checksum, 0x00, 0xAA]
    )


def build_read_packet(addr: int, read_len: int) -> bytes:
    """Build a read-request (mode=0x02) packet."""
    checksum = (255 - (0x09 + 0x02 + addr + read_len) % 256) & 0xFF
    return bytes([0x55, 0x00, 0x09, 0x02, addr, read_len, checksum, 0x00, 0xAA])


class PacketParser:
    """Byte-stream parser for response packets (type 0x12).

    Feed bytes via `feed()`; each call returns a list of (addr, payload) for
    every complete frame seen in the input.
    """

    def __init__(self):
        self._state = 0
        self._len = 0
        self._addr = 0
        self._data = bytearray(256)
        self._ptr = 0
        self._data_len = 0

    def feed(self, data) -> list:
        out = []
        for byte in data:
            result = self._feed_byte(byte)
            if result is not None:
                out.append(result)
        return out

    def _feed_byte(self, byte: int):
        st = self._state
        if st == 0:
            if byte == 0x55:
                self._state = 1
        elif st == 1:
            self._state = 2 if byte == 0x00 else 0
        elif st == 2:
            self._len = byte
            self._data_len = byte - 8
            self._state = 3
        elif st == 3:  # frame type (expect 0x12 for read responses)
            self._state = 4
        elif st == 4:
            self._addr = byte
            self._ptr = 0
            self._state = 5
        elif st == 5:
            if self._ptr < len(self._data):
                self._data[self._ptr] = byte
            self._ptr += 1
            if self._ptr >= self._data_len:
                self._state = 6
        elif st == 6:  # checksum (not verified)
            self._state = 7
        elif st == 7:
            self._state = 8 if byte == 0x00 else 0
        elif st == 8:
            self._state = 0
            if byte == 0xAA:
                return (self._addr, bytes(self._data[: self._data_len]))
        return None


# ---------------------------------------------------------------------------
# Value encoding (mirrors xgolib conver2u8 / conver2float)
# ---------------------------------------------------------------------------

def deg_to_u8(value: float, limits) -> int:
    """Map a physical value (deg or mm) within `limits`=[lo,hi] to 0..255."""
    lo, hi = limits
    if value >= hi:
        return 0xFF
    if value <= lo:
        return 0x00
    return int(255.0 / (hi - lo) * (value - lo))


def u8_to_deg(byte: int, limits) -> float:
    """Inverse of deg_to_u8."""
    lo, hi = limits
    return byte / 255.0 * (hi - lo) + lo


def centered_to_u8(value: float, half_range: float) -> int:
    """Map a symmetric value (-half_range..half_range) to 0..255, 128 = 0."""
    if value >= half_range:
        return 0xFF
    if value <= -half_range:
        return 0x00
    return int(128 + 128 * value / half_range)
