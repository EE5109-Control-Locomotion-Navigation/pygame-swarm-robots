import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from xgo_gait import protocol as P


def test_write_packet_known_example():
    # From the vendor wiki: VX=0xFF -> 55 00 09 01 30 FF <chk> 00 AA
    # checksum = 255 - ((0x09 + 0x01 + 0x30 + 0xFF) % 256)
    pkt = P.build_write_packet(0x30, [0xFF])
    assert pkt[:5] == bytes([0x55, 0x00, 0x09, 0x01, 0x30])
    assert pkt[5] == 0xFF
    assert pkt[-2:] == bytes([0x00, 0xAA])
    # verify checksum property: (LEN+mode+addr+data+chk) % 256 == 255
    total = pkt[2] + pkt[3] + pkt[4] + sum(pkt[5:-3]) + pkt[-3]
    assert total % 256 == 255


def test_read_packet():
    pkt = P.build_read_packet(0x50, 0x0C)
    assert pkt[:6] == bytes([0x55, 0x00, 0x09, 0x02, 0x50, 0x0C])
    total = pkt[2] + pkt[3] + pkt[4] + pkt[5] + pkt[6]
    assert total % 256 == 255
    assert pkt[-2:] == bytes([0x00, 0xAA])


def test_parser_single_frame():
    data = bytes(range(12))
    frame = bytes([0x55, 0x00, 12 + 8, 0x12, 0x50]) + data \
        + bytes([0x00, 0x00, 0xAA])
    out = P.PacketParser().feed(frame)
    assert len(out) == 1
    assert out[0][0] == 0x50
    assert out[0][1] == data


def test_parser_garbage_resync_and_split():
    p = P.PacketParser()
    assert p.feed(b'\x01\x02\x03') == []
    frame = bytes([0x55, 0x00, 0x09, 0x12, 0x01, 0x64, 0x00, 0x00, 0xAA])
    # split delivery
    assert p.feed(frame[:4]) == []
    out = p.feed(frame[4:])
    assert len(out) == 1 and out[0][0] == 0x01 and out[0][1] == b'\x64'


def test_u8_roundtrip():
    for lim in [(-70, 50), (-30, 30), (60, 110)]:
        for v in [lim[0], (lim[0] + lim[1]) / 2, lim[1] - 0.01]:
            b = P.deg_to_u8(v, lim)
            assert 0 <= b <= 255
            back = P.u8_to_deg(b, lim)
            assert abs(back - v) <= (lim[1] - lim[0]) / 255 + 1e-9


def test_centered_u8():
    assert P.centered_to_u8(0.0, 25) == 128
    assert P.centered_to_u8(25.0, 25) == 255
    assert P.centered_to_u8(-25.0, 25) == 0
