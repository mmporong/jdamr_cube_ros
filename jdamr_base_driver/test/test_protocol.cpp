// protocol.hpp 규격 테스트.
//
// 교차 언어 정합: kPyVel / kPyStop / kPyState 는 파이썬 참조 구현
// (jdamr_cube_node/protocol.py) 이 실제로 출력한 바이트를 그대로 하드코딩한 것.
// 어느 한쪽 구현이 규격에서 이탈하면 여기서 깨진다.
#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <vector>

#include "jdamr_base_driver/protocol.hpp"

using namespace jdamr;

static std::vector<uint8_t> hex(const char * s)
{
  std::vector<uint8_t> out;
  for (size_t i = 0; s[i] && s[i + 1]; i += 2) {
    char b[3] = {s[i], s[i + 1], 0};
    out.push_back(static_cast<uint8_t>(strtol(b, nullptr, 16)));
  }
  return out;
}

// 파이썬 protocol.py 출력 (2026-08-14 채취)
static const auto kPyVel = hex("a55a0501f401d4fefa");     // make_velocity_cmd(500, -300)
static const auto kPyStop = hex("a55a050000000000eb");    // make_stop_cmd()
static const auto kPyState = hex(                          // seq=7 상태 프레임
  "a55a1e0740e201000f04f6ffd503f4ff1100fa00e2ff0500e001c0fe64005e2e04a3");

TEST(Crc8, KnownVector)
{
  const uint8_t d[] = "123456789";
  EXPECT_EQ(crc8(d, 9), 0xA1);
}

TEST(Crc8, TableMatchesBitwise)
{
  // LUT 최적화가 비트루프 정의와 동치인지 전수 확인
  for (int i = 0; i < 256; ++i) {
    uint8_t c = static_cast<uint8_t>(i);
    for (int j = 0; j < 8; ++j) {
      c = (c & 1) ? static_cast<uint8_t>((c >> 1) ^ 0x8C) : static_cast<uint8_t>(c >> 1);
    }
    EXPECT_EQ(kCrcTable[i], c);
  }
}

TEST(Cmd, MatchesPythonBytes)
{
  uint8_t f[kCmdFrameSize];
  ASSERT_EQ(make_velocity_cmd(f, 500, -300), kPyVel.size());
  EXPECT_EQ(0, std::memcmp(f, kPyVel.data(), kPyVel.size()));

  ASSERT_EQ(make_stop_cmd(f), kPyStop.size());
  EXPECT_EQ(0, std::memcmp(f, kPyStop.data(), kPyStop.size()));
}

TEST(Parser, StateRoundtripFromPythonBytes)
{
  FrameParser p;
  std::vector<State> got;
  p.feed(kPyState.data(), kPyState.size(), [&](const State & s) {got.push_back(s);});
  ASSERT_EQ(got.size(), 1u);
  const State & s = got[0];
  EXPECT_EQ(s.seq, 7);
  EXPECT_EQ(s.left_pos, 123456);
  EXPECT_EQ(s.right_pos, -654321);
  EXPECT_EQ(s.accel_mg[0], 981);
  EXPECT_EQ(s.gyro_cdps[1], -30);
  EXPECT_EQ(s.mag_dut[2], 100);
  EXPECT_EQ(s.batt_mv, 11870);
  EXPECT_TRUE(s.watchdog_stopped());
  EXPECT_FALSE(s.servo_error());
}

TEST(State, SensorHealthFlags)
{
  State s{};
  s.flags = kFlagQmi8658Err | kFlagAk09918Err | kFlagNoIna219;
  EXPECT_TRUE(s.qmi8658_error());
  EXPECT_TRUE(s.ak09918_error());
  EXPECT_TRUE(s.ina219_error());
  EXPECT_FALSE(s.servo_error());
}

TEST(StateInterval, RejectsLongReconnectAndSequenceWrap)
{
  EXPECT_TRUE(state_interval_is_integrable(10, 11, 0.020));
  EXPECT_TRUE(state_interval_is_integrable(250, 5, 0.220));
  EXPECT_FALSE(state_interval_is_integrable(10, 10, 5.120));  // 256 lost frames
  EXPECT_FALSE(state_interval_is_integrable(10, 11, 5.140));  // 257 lost frames
  EXPECT_FALSE(state_interval_is_integrable(10, 36, 0.520));
}

TEST(Parser, ResyncAfterGarbage)
{
  FrameParser p;
  std::vector<uint8_t> stream = {0xFF, 0x00, 0x33};
  stream.insert(stream.end(), kPyState.begin(), kPyState.end());
  int n = 0;
  p.feed(stream.data(), stream.size(), [&](const State &) {++n;});
  EXPECT_EQ(n, 1);
}

TEST(Parser, CrcErrorCountedAndDropped)
{
  auto bad = kPyState;
  bad.back() ^= 0xFF;
  bad.insert(bad.end(), kPyState.begin(), kPyState.end());
  FrameParser p;
  int n = 0;
  p.feed(bad.data(), bad.size(), [&](const State &) {++n;});
  EXPECT_EQ(n, 1);
  EXPECT_EQ(p.crc_errors(), 1u);
}

TEST(Parser, SteadyStreamLossless)
{
  // seq 를 바꿔 가며 200프레임 — CRC 재계산 포함
  std::vector<uint8_t> stream;
  for (int i = 0; i < 200; ++i) {
    auto f = kPyState;
    f[3] = static_cast<uint8_t>(i);                 // payload 첫 바이트 = seq
    f.back() = crc8(f.data() + 2, 1 + kStateLen);   // LEN+payload 재서명
    stream.insert(stream.end(), f.begin(), f.end());
  }
  FrameParser p;
  std::vector<uint8_t> seqs;
  p.feed(stream.data(), stream.size(), [&](const State & s) {seqs.push_back(s.seq);});
  ASSERT_EQ(seqs.size(), 200u);
  for (int i = 0; i < 200; ++i) {EXPECT_EQ(seqs[i], static_cast<uint8_t>(i));}
  EXPECT_EQ(p.crc_errors(), 0u);
}

TEST(Parser, ThroughputInformational)
{
  // 성능 회귀 감지용 정보성 측정. 실기 부하는 1.7KB/s — 여유 배율을 기록한다.
  // 단언은 느슨하게(>5MB/s): CI·저전력 기기에서의 플레이크 방지.
  std::vector<uint8_t> stream;
  stream.reserve(kPyState.size() * 3000);
  for (int i = 0; i < 3000; ++i) {
    stream.insert(stream.end(), kPyState.begin(), kPyState.end());
  }
  FrameParser p;
  size_t n = 0;
  const auto t0 = std::chrono::steady_clock::now();
  p.feed(stream.data(), stream.size(), [&](const State &) {++n;});
  const auto dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  const double mbps = stream.size() / dt / 1e6;
  RecordProperty("parse_MB_per_s", static_cast<int>(mbps));
  printf("[perf] 파싱 %.1f MB/s (실기 부하의 %.0f배)\n", mbps, mbps * 1e6 / 1700.0);
  EXPECT_EQ(n, 3000u);
  EXPECT_GT(mbps, 5.0);
}
