// JD-AMR Cube 시리얼 프로토콜 v2 — C++ 구현 (헤더 온리, ROS 의존 없음).
//
// 규격 단일 출처: firmware/jdamr_cube_fw/jdamr_cube_fw.ino 헤더 주석.
// 파이썬 참조 구현:  jdamr_cube_node/jdamr_cube_node/protocol.py
// 두 구현은 test_protocol.cpp 의 공유 바이트 벡터(파이썬 출력 하드코딩)로
// 교차 검증된다 — 한쪽만 고치면 테스트가 깨진다.
//
// 성능 설계:
//   * 파서는 고정 40B 버퍼, feed() 경로에 힙 할당 0회
//   * CRC-8/MAXIM 은 constexpr 256-엔트리 LUT (비트루프 대비 바이트당 분기 8→1)
//   * 멀티바이트 판독은 memcpy 기반 리틀엔디언 리더 — 비정렬 접근·strict
//     aliasing UB 없이 컴파일러가 단일 load 로 최적화한다
#pragma once

#include <array>
#include <cstdint>
#include <cstring>

namespace jdamr
{

inline constexpr uint8_t kHdr1 = 0xA5;
inline constexpr uint8_t kHdr2 = 0x5A;
inline constexpr uint8_t kCmdStop = 0x00;
inline constexpr uint8_t kCmdVelocity = 0x01;
inline constexpr uint8_t kCmdReboot = 0x0F;
inline constexpr int16_t kRebootMagic = 0x0F0F;
inline constexpr uint8_t kStateLen = 30;
inline constexpr size_t kCmdFrameSize = 9;  // 2 hdr + 1 len + 5 payload + 1 crc
inline constexpr double kMaxStateIntegrationGapSeconds = 0.5;

// flags 비트 (펌웨어와 동일)
inline constexpr uint8_t kFlagServoLErr = 0x01;
inline constexpr uint8_t kFlagServoRErr = 0x02;
inline constexpr uint8_t kFlagWatchdog = 0x04;
inline constexpr uint8_t kFlagNoIna219 = 0x08;
inline constexpr uint8_t kFlagQmi8658Err = 0x10;
inline constexpr uint8_t kFlagAk09918Err = 0x20;

// ── CRC-8/MAXIM ──
constexpr std::array<uint8_t, 256> make_crc_table()
{
  std::array<uint8_t, 256> t{};
  for (int i = 0; i < 256; ++i) {
    uint8_t c = static_cast<uint8_t>(i);
    for (int j = 0; j < 8; ++j) {
      c = (c & 1) ? static_cast<uint8_t>((c >> 1) ^ 0x8C) : static_cast<uint8_t>(c >> 1);
    }
    t[i] = c;
  }
  return t;
}
inline constexpr auto kCrcTable = make_crc_table();

inline uint8_t crc8(const uint8_t * d, size_t n)
{
  uint8_t c = 0;
  while (n--) {c = kCrcTable[c ^ *d++];}
  return c;
}

// ── 리틀엔디언 리더 (비정렬 안전) ──
inline int16_t rd_i16(const uint8_t * p)
{
  int16_t v;
  std::memcpy(&v, p, 2);
  return v;  // 호스트(파이4·x86)는 리틀엔디언 — 펌웨어와 동일
}
inline int32_t rd_i32(const uint8_t * p)
{
  int32_t v;
  std::memcpy(&v, p, 4);
  return v;
}
inline uint16_t rd_u16(const uint8_t * p)
{
  uint16_t v;
  std::memcpy(&v, p, 2);
  return v;
}
inline void wr_i16(uint8_t * p, int16_t v) {std::memcpy(p, &v, 2);}

// ── 상태 패킷 (MCU→Host, 50Hz) ──
struct State
{
  uint8_t seq;
  int32_t left_pos;        // 누적 counts, + = 전진 (펌웨어 언랩 완료)
  int32_t right_pos;
  int16_t accel_mg[3];     // [mg]
  int16_t gyro_cdps[3];    // [0.01 deg/s]
  int16_t mag_dut[3];      // [0.1 uT]
  uint16_t batt_mv;
  uint8_t flags;

  bool watchdog_stopped() const {return flags & kFlagWatchdog;}
  bool servo_error() const {return flags & (kFlagServoLErr | kFlagServoRErr);}
  bool qmi8658_error() const {return flags & kFlagQmi8658Err;}
  bool ak09918_error() const {return flags & kFlagAk09918Err;}
  bool ina219_error() const {return flags & kFlagNoIna219;}
};

inline bool state_interval_is_integrable(
  uint8_t previous_seq, uint8_t current_seq, double receive_gap_seconds)
{
  const uint8_t sequence_gap = static_cast<uint8_t>(current_seq - previous_seq);
  return sequence_gap > 0 && sequence_gap <= 25 &&
         receive_gap_seconds >= 0.0 &&
         receive_gap_seconds <= kMaxStateIntegrationGapSeconds;
}

inline State parse_state(const uint8_t * p)
{
  State s;
  s.seq = p[0];
  s.left_pos = rd_i32(p + 1);
  s.right_pos = rd_i32(p + 5);
  for (int i = 0; i < 3; ++i) {s.accel_mg[i] = rd_i16(p + 9 + 2 * i);}
  for (int i = 0; i < 3; ++i) {s.gyro_cdps[i] = rd_i16(p + 15 + 2 * i);}
  for (int i = 0; i < 3; ++i) {s.mag_dut[i] = rd_i16(p + 21 + 2 * i);}
  s.batt_mv = rd_u16(p + 27);
  s.flags = p[29];
  return s;
}

// ── 명령 프레임 생성 (Host→MCU). out 은 kCmdFrameSize 이상. 반환 = 프레임 길이 ──
inline size_t make_cmd(uint8_t * out, uint8_t cmd, int16_t left, int16_t right)
{
  out[0] = kHdr1;
  out[1] = kHdr2;
  out[2] = 5;
  out[3] = cmd;
  wr_i16(out + 4, left);
  wr_i16(out + 6, right);
  out[8] = crc8(out + 2, 6);
  return kCmdFrameSize;
}
inline size_t make_velocity_cmd(uint8_t * out, int16_t l, int16_t r)
{
  return make_cmd(out, kCmdVelocity, l, r);
}
inline size_t make_stop_cmd(uint8_t * out) {return make_cmd(out, kCmdStop, 0, 0);}

// ── 수신 파서 — 펌웨어 feed_rx() 와 동일한 상태기계 ──
// 재동기 속성: LEN까지 도착한 짤린 프레임은 뒤따르는 1프레임까지 삼킬 수
// 있고 그다음부터 복구 보장 (50Hz 스트림에서 최대 40ms 공백).
class FrameParser
{
public:
  // 완성된 상태 프레임마다 on_state(const State &) 호출. 힙 할당 없음.
  template<typename F>
  void feed(const uint8_t * data, size_t n, F && on_state)
  {
    for (size_t i = 0; i < n; ++i) {
      const uint8_t b = data[i];
      switch (st_) {
        case Rx::H1:
          st_ = (b == kHdr1) ? Rx::H2 : Rx::H1;
          break;
        case Rx::H2:
          st_ = (b == kHdr2) ? Rx::Len : (b == kHdr1 ? Rx::H2 : Rx::H1);
          break;
        case Rx::Len:
          if (b == 0 || b > kMaxPayload) {
            st_ = Rx::H1;
          } else {
            len_ = b;
            idx_ = 0;
            buf_[0] = b;
            st_ = Rx::Body;
          }
          break;
        case Rx::Body:
          buf_[1 + idx_++] = b;
          if (idx_ == static_cast<size_t>(len_) + 1) {  // payload + crc
            if (crc8(buf_, 1 + len_) == buf_[1 + len_]) {
              ++frames_ok_;
              if (len_ == kStateLen) {
                on_state(parse_state(buf_ + 1));
              }
              // 다른 길이의 유효 프레임은 상위 호환용으로 무시
            } else {
              ++crc_errors_;
            }
            st_ = Rx::H1;
          }
          break;
      }
    }
  }

  uint64_t crc_errors() const {return crc_errors_;}
  uint64_t frames_ok() const {return frames_ok_;}

private:
  static constexpr uint8_t kMaxPayload = 38;
  enum class Rx : uint8_t {H1, H2, Len, Body};
  Rx st_{Rx::H1};
  uint8_t len_{0};
  size_t idx_{0};
  uint8_t buf_[kMaxPayload + 2]{};   // len + payload + crc, 고정 크기
  uint64_t crc_errors_{0};
  uint64_t frames_ok_{0};
};

}  // namespace jdamr
