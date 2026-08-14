"""protocol.py 규격 테스트 — 펌웨어 헤더 표와 1:1 대응.

펌웨어 없이도 프로토콜 회귀를 잡는 게 목적이라 시리얼 포트를 열지 않는다.
"""
import struct

from jdamr_cube_node.protocol import (
    CMD_VELOCITY, STATE_LEN, FrameParser, crc8,
    make_reboot_cmd, make_stop_cmd, make_velocity_cmd,
)


def _state_frame(seq=0, lp=0, rp=0, accel=(0, 0, 0), gyro=(0, 0, 0),
                 mag=(0, 0, 0), mv=12000, flags=0):
    payload = struct.pack('<Bii9hHB', seq, lp, rp, *accel, *gyro, *mag, mv, flags)
    body = bytes([len(payload)]) + payload
    return b'\xA5\x5A' + body + bytes([crc8(body)])


def test_crc8_known_vector():
    assert crc8(b'123456789') == 0xA1


def test_velocity_cmd_layout():
    f = make_velocity_cmd(500, -300)
    assert f[:2] == b'\xA5\x5A' and f[2] == 5 and len(f) == 9
    cmd, left, right = struct.unpack('<Bhh', f[3:8])
    assert (cmd, left, right) == (CMD_VELOCITY, 500, -300)
    assert crc8(f[2:8]) == f[8]


def test_stop_and_reboot_cmds():
    assert struct.unpack('<Bhh', make_stop_cmd()[3:8]) == (0x00, 0, 0)
    assert struct.unpack('<Bhh', make_reboot_cmd()[3:8]) == (0x0F, 0x0F0F, 0)


def test_state_roundtrip():
    f = _state_frame(seq=7, lp=123456, rp=-654321, accel=(981, -12, 17),
                     gyro=(250, -30, 5), mag=(480, -320, 100), mv=11870, flags=0x04)
    (s,) = FrameParser().feed(f)
    assert (s.seq, s.left_pos, s.right_pos) == (7, 123456, -654321)
    assert s.accel_mg == (981, -12, 17) and s.gyro_cdps == (250, -30, 5)
    assert s.batt_mv == 11870
    assert s.watchdog_stopped and not s.servo_error


def test_resync_after_garbage():
    p = FrameParser()
    assert len(p.feed(b'\xFF\x00\x33' + _state_frame(seq=1))) == 1


def test_truncated_frame_costs_at_most_one():
    # LEN까지 도착한 짤린 프레임: 다음 1프레임 희생 후 복구 (문서화된 속성)
    p = FrameParser()
    got = p.feed(b'\xA5\x5A\x1E\x01' + _state_frame(seq=1) + _state_frame(seq=2))
    assert [s.seq for s in got] == [2] or len(got) == 1


def test_crc_error_counted_and_dropped():
    bad = bytearray(_state_frame(seq=1))
    bad[-1] ^= 0xFF
    p = FrameParser()
    got = p.feed(bytes(bad) + _state_frame(seq=2))
    assert [s.seq for s in got] == [2] and p.crc_errors == 1


def test_steady_stream_lossless():
    stream = b''.join(_state_frame(seq=i & 0xFF) for i in range(200))
    got = FrameParser().feed(stream)
    assert [s.seq for s in got] == [i & 0xFF for i in range(200)]
