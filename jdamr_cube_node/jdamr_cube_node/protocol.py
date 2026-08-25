"""JD-AMR Cube 시리얼 프로토콜 v2 — 호스트(파이) 쪽 구현.

펌웨어 짝: firmware/jdamr_cube_fw/jdamr_cube_fw.ino (규격 주석이 단일 출처).
프레임: [0xA5][0x5A][LEN][PAYLOAD×LEN][CRC8]  · CRC-8/MAXIM, LEN+PAYLOAD 대상
바이트오더: 리틀엔디언 고정 (ESP32 native = struct '<').

재동기 속성(펌웨어와 동일): 짤린 프레임이 LEN까지 도착한 경우 뒤따르는
프레임 1개까지 삼킬 수 있고, 그다음 프레임부터 복구가 보장된다.
50Hz 상태 스트림에서 최대 40ms 공백 — 오도메트리 적분에는 무해.
"""
from dataclasses import dataclass
import struct

HDR1, HDR2 = 0xA5, 0x5A
CMD_STOP, CMD_VELOCITY, CMD_REBOOT = 0x00, 0x01, 0x0F
REBOOT_MAGIC = 0x0F0F
STATE_LEN = 30
_STATE_FMT = '<Bii9hHB'   # seq, l_pos, r_pos, accel×3, gyro×3, mag×3, mV, flags

# flags 비트 (펌웨어 헤더 표와 동일)
FLAG_SERVO_L_ERR = 0x01
FLAG_SERVO_R_ERR = 0x02
FLAG_WATCHDOG = 0x04
FLAG_NO_INA219 = 0x08
FLAG_QMI8658_ERR = 0x10
FLAG_AK09918_ERR = 0x20


def crc8(data: bytes) -> int:
    """CRC-8/MAXIM (poly 0x31 reflected). 검증 벡터: crc8(b'123456789') == 0xA1."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8C) if (crc & 1) else (crc >> 1)
    return crc


def _frame(payload: bytes) -> bytes:
    body = bytes([len(payload)]) + payload
    return bytes([HDR1, HDR2]) + body + bytes([crc8(body)])


def make_velocity_cmd(left_counts_s: int, right_counts_s: int) -> bytes:
    """바퀴별 속도 지령 [counts/s]. + = 로봇 전진 (우측 반전은 펌웨어 책임)."""
    return _frame(struct.pack('<Bhh', CMD_VELOCITY, left_counts_s, right_counts_s))


def make_stop_cmd() -> bytes:
    return _frame(struct.pack('<Bhh', CMD_STOP, 0, 0))


def make_reboot_cmd() -> bytes:
    return _frame(struct.pack('<Bhh', CMD_REBOOT, REBOOT_MAGIC, 0))


@dataclass
class State:
    seq: int
    left_pos: int          # 누적 counts, + = 전진 (펌웨어에서 언랩 완료)
    right_pos: int
    accel_mg: tuple        # (x,y,z) [mg]
    gyro_cdps: tuple       # (x,y,z) [0.01 deg/s]
    mag_dut: tuple         # (x,y,z) [0.1 uT]
    batt_mv: int
    flags: int

    @property
    def watchdog_stopped(self) -> bool:
        return bool(self.flags & FLAG_WATCHDOG)

    @property
    def servo_error(self) -> bool:
        return bool(self.flags & (FLAG_SERVO_L_ERR | FLAG_SERVO_R_ERR))

    @property
    def qmi8658_error(self) -> bool:
        return bool(self.flags & FLAG_QMI8658_ERR)

    @property
    def ak09918_error(self) -> bool:
        return bool(self.flags & FLAG_AK09918_ERR)

    @property
    def ina219_error(self) -> bool:
        return bool(self.flags & FLAG_NO_INA219)


def _parse_state(payload: bytes) -> State:
    v = struct.unpack(_STATE_FMT, payload)
    return State(seq=v[0], left_pos=v[1], right_pos=v[2],
                 accel_mg=v[3:6], gyro_cdps=v[6:9], mag_dut=v[9:12],
                 batt_mv=v[12], flags=v[13])


class FrameParser:
    """바이트 스트림 → 프레임. 펌웨어 feed_rx()와 동일한 상태기계."""

    _H1, _H2, _LEN, _BODY = range(4)

    def __init__(self):
        self._st = self._H1
        self._buf = b''
        self._ln = 0
        self.crc_errors = 0     # 진단용: 워치독·케이블 문제 추적에 쓴다
        self.frames_ok = 0

    def feed(self, data: bytes):
        """수신 바이트를 먹이고, 완성된 State 목록을 돌려준다."""
        out = []
        for b in data:
            st = self._st
            if st == self._H1:
                self._st = self._H2 if b == HDR1 else self._H1
            elif st == self._H2:
                self._st = self._LEN if b == HDR2 else (self._H2 if b == HDR1 else self._H1)
            elif st == self._LEN:
                if b == 0 or b > 38:
                    self._st = self._H1
                else:
                    self._ln, self._buf, self._st = b, bytes([b]), self._BODY
            else:
                self._buf += bytes([b])
                if len(self._buf) == self._ln + 2:
                    if crc8(self._buf[:-1]) == self._buf[-1]:
                        self.frames_ok += 1
                        if self._ln == STATE_LEN:
                            out.append(_parse_state(self._buf[1:-1]))
                        # 다른 길이의 유효 프레임은 상위 호환용으로 무시
                    else:
                        self.crc_errors += 1
                    self._st = self._H1
        return out
