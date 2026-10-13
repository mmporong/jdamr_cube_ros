/*
 * jdamr_cube_fw — JD-AMR Cube ESP32 펌웨어 (강사원본 ros_firmware.ino 대체)
 *
 * 보드: Waveshare General Driver for Robots (ESP32-WROOM-32)
 * 서보: Feetech STS3215 ×4 (전좌2, 전우1, 후좌3, 후우4), 1,000,000 baud
 * 센서: QMI8658C(6축) + AK09918(지자기 3축), INA219(전압), SSD1306 128×32
 *
 * 이 판이 고치는 강사원본 결함 (번호 = ~/jdamr_cube_bringup/JDAMR_Cube_조사기록.md):
 *   A1  자이로 Y/Z·지자기 3축 미전송            → 9축 전부 전송
 *   A2  이미 mg인 가속도에 ×1000해 int16 overflow → 단위별 포화·반올림 패커 사용
 *   A3  체크섬 항상 0x00                        → CRC-8/MAXIM 실제 계산
 *   A4  헤더 재동기화 없음                       → 바이트 단위 상태기계로 재동기
 *   A5  후진·회전 분기 부재                      → 부호 있는 바퀴별 속도라 방향 분기 자체가 없음
 *   A6  같은 서보 위치 반복 조회                  → 바퀴마다 주기당 1회 읽어 재사용
 *   A7  100ms마다 OLED 전체 프레임 전송(블로킹)   → 500ms 주기 + 내용 변한 때만
 *   A8  0~4095 랩어라운드 언랩 없음              → int64 누적 후 int32 포화 전송
 *   A9  cmd 워치독 없음                         → 400ms 무명령 시 정지
 *   A10 INA219 0x42 하드코딩                    → 0x40~0x45 스캔
 *   A11 좌우 부호 규약 미문서화                  → "+ = 로봇 전진" 통일, 좌측 반전은 여기서만
 *
 * ⚠ B1 주의: STSServoDriver 라이브러리가 두 판본으로 돈다.
 *   강사원본 ros_firmware.ino:  st.init(255, &ServoSerial, 1000000)
 *   강사원본 speed_control.ino: st.init(&Serial1, 1000000)
 *   아래는 ros_firmware 쪽 3인자 판 기준. 컴파일이 깨지면 설치된 STSServoDriver.h의
 *   init 서명을 확인하고 SERVO_INIT() 매크로만 고칠 것.
 *
 * ── 프로토콜 v2 (양방향 공통 프레임) ─────────────────────────────────
 *   [0xA5][0x5A][LEN][PAYLOAD ×LEN][CRC8]   CRC는 LEN+PAYLOAD 대상, CRC-8/MAXIM
 *   멀티바이트는 전부 리틀엔디언 (ESP32 native, 파이썬 struct '<')
 *
 * Host→MCU PAYLOAD (LEN=5):
 *   uint8  cmd      0x01=속도지령  0x00=정지  0x0F=재부팅(left==0x0F0F 일 때만)
 *   int16  left     좌바퀴 목표속도 [counts/s], + = 로봇 전진 방향
 *   int16  right    우바퀴 목표속도 [counts/s], + = 로봇 전진 방향
 *
 * MCU→Host PAYLOAD (LEN=30, 주기 20ms = 50Hz):
 *   uint8  seq          순번 (프레임 유실 감지)
 *   int32  left_pos     좌측 앞뒤 누적 위치 평균 [counts] (언랩, + = 전진)
 *   int32  right_pos    우측 앞뒤 누적 위치 평균 [counts] (언랩, + = 전진)
 *   int16  accel[3]     가속도 [mg]        (raw g ×1000)
 *   int16  gyro[3]      각속도 [0.01 dps]  (raw dps ×100)
 *   int16  mag[3]       지자기 [0.1 uT]    (raw uT ×10)
 *   uint16 batt_mV      배터리 전압 [mV]
 *   uint8  flags        bit0 좌측 서보 중 하나 실패  bit1 우측 서보 중 하나 실패
 *                       bit2 워치독 정지중    bit3 INA219 미검출
 *                       bit4 QMI8658 읽기실패 bit5 AK09918 읽기실패
 *
 * 물리 제원(바퀴 지름·트레드)은 펌웨어에 두지 않는다 — 오도메트리 환산은
 * ROS 노드 파라미터가 단일 출처다 (counts/s ↔ m/s 변환은 호스트 책임).
 */

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <INA219_WE.h>
#include "IMU.h"
#include "imu_packet_units.h"
#include "STSServoDriver.h"
#include "wheel_layout.h"

// ── 핀·주소 (강사원본과 동일) ──
#define S_SCL 33
#define S_SDA 32
#define S_RX  18
#define S_TX  19
#define SCREEN_ADDRESS 0x3C

// ── 규격 상수 ──
static const uint32_t STATE_PERIOD_MS = 20;    // 50Hz 상태 송신
static const uint32_t OLED_PERIOD_MS  = 500;   // A7: OLED는 0.5s + 변경시만
static const uint32_t CMD_TIMEOUT_MS  = 400;   // A9: 워치독
static const int16_t  VEL_LIMIT = 3400;        // STS3215 속도 지령 상한 [counts/s]
static const uint8_t  FLAG_SERVO_L_ERR = 0x01;
static const uint8_t  FLAG_SERVO_R_ERR = 0x02;
static const uint8_t  FLAG_WATCHDOG    = 0x04;
static const uint8_t  FLAG_NO_INA219   = 0x08;
static const uint8_t  FLAG_QMI8658_ERR = 0x10;
static const uint8_t  FLAG_AK09918_ERR = 0x20;

HardwareSerial ServoSerial(2);
Adafruit_SSD1306 display(128, 32, &Wire, -1);
STSServoDriver st;
INA219_WE *ina219 = nullptr;             // A10: 주소 스캔 후 생성
static bool oled_available = false;

// B1: 라이브러리 판본에 따라 이 줄만 고친다
#define SERVO_INIT() st.init(255, &ServoSerial, 1000000)

EulerAngles stAngles;
IMU_ST_SENSOR_DATA_FLOAT stGyroRawData, stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;

// ── CRC-8/MAXIM (poly 0x31 reflected) ──
static uint8_t crc8(const uint8_t *d, size_t n) {
  uint8_t crc = 0;
  while (n--) {
    crc ^= *d++;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 1) ? (crc >> 1) ^ 0x8C : crc >> 1;
  }
  return crc;
}

// ── 수신 상태기계 (A4: 1바이트씩 먹으며 헤더 재동기) ──
enum RxState { RX_H1, RX_H2, RX_LEN, RX_BODY };
static RxState rx_state = RX_H1;
static uint8_t rx_len = 0, rx_idx = 0;
static uint8_t rx_buf[40];

// ── 언랩 상태 (A8) ──
static jdamr_fw::WheelUnwrapState wheel_position[jdamr_fw::WHEEL_COUNT] = {};

// ── 런타임 상태 ──
static uint32_t last_cmd_ms = 0;
static bool     watchdog_stopped = true;   // 부팅 직후는 정지 상태로 시작
static uint8_t  seq = 0;
static int16_t  cmd_left = 0, cmd_right = 0;
static String   oled_prev = "";

static bool servo_ok[jdamr_fw::WHEEL_COUNT] = {false, false, false, false};
static bool servo_fault_latched = true;

// A11: 부호 규약의 유일한 반전 지점.
// ID2/3은 왼쪽(전진 부호 -), ID1/4는 오른쪽(전진 부호 +)이다.
// 네 목표값은 한 SYNCWRITE 패킷에 담아 같은 쪽 앞뒤에 같은 명령을 적용한다.
static bool set_wheel_velocity(int16_t left_fwd, int16_t right_fwd) {
  int velocities[jdamr_fw::WHEEL_COUNT];
  for (uint8_t i = 0; i < jdamr_fw::WHEEL_COUNT; i++) {
    velocities[i] = jdamr_fw::servo_velocity(i, left_fwd, right_fwd, VEL_LIMIT);
  }
  return st.setTargetVelocities(
    jdamr_fw::WHEEL_COUNT, jdamr_fw::WHEEL_IDS, velocities);
}

static void stop_motors() {
  if (!set_wheel_velocity(0, 0)) {
    servo_fault_latched = true;
    for (uint8_t i = 0; i < jdamr_fw::WHEEL_COUNT; i++) servo_ok[i] = false;
  }
  cmd_left = cmd_right = 0;
}

static bool configure_servo(uint8_t wheel_index) {
  const uint8_t id = jdamr_fw::WHEEL_IDS[wheel_index];
  byte mode = 0;
  byte torque = 0;

  // Disable torque while setting the mode. Clear the retained target before
  // torque is enabled so an ESP32-only reset cannot restart an old command.
  if (!st.writeRegister(id, STSRegisters::TORQUE_SWITCH, 0)) return false;
  if (!st.setTargetVelocity(id, 0)) return false;
  if (!st.setMode(id, VELOCITY)) return false;
  if (!st.tryReadRegister(id, STSRegisters::OPERATION_MODE, mode) ||
      mode != static_cast<byte>(VELOCITY)) return false;
  if (!st.setTargetVelocity(id, 0)) return false;
  if (!st.writeRegister(id, STSRegisters::TORQUE_SWITCH, 1)) return false;
  if (!st.tryReadRegister(id, STSRegisters::TORQUE_SWITCH, torque) || torque != 1)
    return false;
  jdamr_fw::reset_unwrap(wheel_position[wheel_index]);
  return true;
}

static void ensure_servo_config() {
  bool all_ok = true;
  for (uint8_t i = 0; i < jdamr_fw::WHEEL_COUNT; i++) {
    const uint8_t id = jdamr_fw::WHEEL_IDS[i];
    const bool alive = st.ping(id);
    if (!alive) {
      servo_ok[i] = false;
    } else if (!servo_ok[i]) {
      servo_ok[i] = configure_servo(i);
    }
    all_ok = all_ok && servo_ok[i];
  }
  if (!all_ok) {
    servo_fault_latched = true;
    stop_motors();
  }
}

// ── 수신 명령 처리 ──
static void handle_cmd(const uint8_t *p, uint8_t len) {
  if (len != 5) return;
  uint8_t cmd = p[0];
  int16_t left, right;
  memcpy(&left,  p + 1, 2);
  memcpy(&right, p + 3, 2);

  if (cmd == 0x01) {                       // 속도 지령
    last_cmd_ms = millis();
    if ((left != 0 || right != 0) && servo_fault_latched) {
      stop_motors();                        // 정상 회복 전 비영 속도 거부
      return;
    }
    if (!set_wheel_velocity(left, right)) {
      servo_fault_latched = true;
      for (uint8_t i = 0; i < jdamr_fw::WHEEL_COUNT; i++) servo_ok[i] = false;
      stop_motors();
      return;
    }
    cmd_left = left; cmd_right = right;
    watchdog_stopped = false;
  } else if (cmd == 0x00) {                // 명시적 정지
    stop_motors();
    last_cmd_ms = millis();
    watchdog_stopped = false;
  } else if (cmd == 0x0F && left == 0x0F0F) {  // 재부팅 (매직 요구)
    stop_motors();
    delay(50);
    ESP.restart();
  }
}

static void feed_rx(uint8_t b) {
  switch (rx_state) {
    case RX_H1:  if (b == 0xA5) rx_state = RX_H2;                      break;
    case RX_H2:  rx_state = (b == 0x5A) ? RX_LEN : RX_H1;              break;
    case RX_LEN:
      if (b == 0 || b > sizeof(rx_buf) - 2) { rx_state = RX_H1; break; }
      rx_len = b; rx_idx = 0; rx_buf[0] = b; rx_state = RX_BODY;       break;
    case RX_BODY:
      rx_buf[1 + rx_idx++] = b;
      if (rx_idx == rx_len + 1) {          // payload + crc 다 받음
        if (crc8(rx_buf, 1 + rx_len) == rx_buf[1 + rx_len])
          handle_cmd(rx_buf + 1, rx_len);
        rx_state = RX_H1;                  // CRC 불일치면 조용히 버리고 재동기
      }
      break;
  }
}

// ── 상태 패킷 송신 ──
static void send_state() {
  uint8_t flags = watchdog_stopped ? FLAG_WATCHDOG : 0x00;
  if (!ina219) flags |= FLAG_NO_INA219;

  // A6: 각 서보 위치는 여기서 주기당 한 번만 읽는다.
  int16_t raw[jdamr_fw::WHEEL_COUNT] = {0, 0, 0, 0};
  bool position_ok[jdamr_fw::WHEEL_COUNT] = {false, false, false, false};
  bool all_positions_ok = true;
  for (uint8_t i = 0; i < jdamr_fw::WHEEL_COUNT; i++) {
    position_ok[i] = st.getCurrentPosition(jdamr_fw::WHEEL_IDS[i], raw[i]);
    if (!servo_ok[i] || !position_ok[i]) {
      flags |= jdamr_fw::WHEEL_SIDES[i] == jdamr_fw::SIDE_LEFT
                 ? FLAG_SERVO_L_ERR : FLAG_SERVO_R_ERR;
      servo_ok[i] = false;
      jdamr_fw::reset_unwrap(wheel_position[i]);
      all_positions_ok = false;
      continue;
    }
    // 이 라이브러리는 읽기 타임아웃을 0 으로 돌려준다 — 20ms 에 물리적으로
    // 불가능한 점프(|d|>300, 상한 지령 68의 4배)는 글리치로 보고 버린다.
    if (!jdamr_fw::update_unwrap(
          wheel_position[i], raw[i], jdamr_fw::WHEEL_FORWARD_SIGNS[i], 300)) {
      flags |= jdamr_fw::WHEEL_SIDES[i] == jdamr_fw::SIDE_LEFT
                 ? FLAG_SERVO_L_ERR : FLAG_SERVO_R_ERR;
      servo_ok[i] = false;
      all_positions_ok = false;
      continue;
    }
  }

  if (!all_positions_ok) {
    servo_fault_latched = true;
    stop_motors();
  } else {
    // A fault clears only after every servo is configured and has supplied a
    // valid position sample. A recovered servo's first sample adds no delta.
    servo_fault_latched = false;
  }

  const int32_t left_pos = jdamr_fw::average_clamped(
    wheel_position[0].total, wheel_position[2].total);
  const int32_t right_pos = jdamr_fw::average_clamped(
    wheel_position[1].total, wheel_position[3].total);

  const uint8_t imu_health =
    imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  if (!(imu_health & IMU_HEALTH_QMI8658)) flags |= FLAG_QMI8658_ERR;
  if (!(imu_health & IMU_HEALTH_AK09918)) flags |= FLAG_AK09918_ERR;
  uint16_t mv = ina219 ? (uint16_t)(ina219->getBusVoltage_V() * 1000.0f) : 0;

  uint8_t f[3 + 30 + 1];                   // 헤더2 + LEN + payload30 + crc
  f[0] = 0xA5; f[1] = 0x5A; f[2] = 30;
  uint8_t *p = f + 3;
  *p++ = seq++;
  memcpy(p, &left_pos, 4); p += 4;
  memcpy(p, &right_pos, 4); p += 4;
  int16_t v;
  v = jdamr_fw::pack_accel_mg(stAccelRawData.X); memcpy(p, &v, 2); p += 2;
  v = jdamr_fw::pack_accel_mg(stAccelRawData.Y); memcpy(p, &v, 2); p += 2;
  v = jdamr_fw::pack_accel_mg(stAccelRawData.Z); memcpy(p, &v, 2); p += 2;
  v = jdamr_fw::pack_gyro_dps(stGyroRawData.X);  memcpy(p, &v, 2); p += 2;
  v = jdamr_fw::pack_gyro_dps(stGyroRawData.Y);  memcpy(p, &v, 2); p += 2;
  v = jdamr_fw::pack_gyro_dps(stGyroRawData.Z);  memcpy(p, &v, 2); p += 2;
  v = jdamr_fw::pack_magnetic_ut(stMagnRawData.s16X); memcpy(p, &v, 2); p += 2;
  v = jdamr_fw::pack_magnetic_ut(stMagnRawData.s16Y); memcpy(p, &v, 2); p += 2;
  v = jdamr_fw::pack_magnetic_ut(stMagnRawData.s16Z); memcpy(p, &v, 2); p += 2;
  memcpy(p, &mv, 2); p += 2;
  *p++ = flags;
  f[sizeof(f) - 1] = crc8(f + 2, 31);      // A3: LEN+payload 실제 CRC
  Serial.write(f, sizeof(f));
}

// ── OLED (A7: 저주기 + 변경시만) ──
static void update_oled(uint16_t mv) {
  if (!oled_available) return;
  // 128x32 3줄 배치. 현장에서 로봇 옆에 서서 보는 순서로 놓는다:
  //   1행 전압 + 무장상태  — 충전 시점과 "왜 안 움직이나"의 답
  //   2행 바퀴 지령        — 조종이 닿고 있는지
  //   3행 서보 ID·건강     — 네 바퀴 버스 상태
  // 변경 없으면 그리지 않는다 — display() 는 I2C 전체 프레임이라 수십 ms 블로킹(A7).
  // 전압은 0.1V 로 양자화돼 있어 미세 변동으로 매번 다시 그리지 않는다.
  String l1 = String(mv / 1000.0f, 2) + "V " +
              (servo_fault_latched ? "FAULT" : (watchdog_stopped ? "IDLE" : "ARM"));
  String l2 = "L" + String(cmd_left) + "  R" + String(cmd_right);
  String l3 = "S ";
  for (uint8_t i = 0; i < jdamr_fw::WHEEL_COUNT; i++) {
    l3 += String(jdamr_fw::WHEEL_IDS[i]);
    l3 += servo_ok[i] ? "+" : "!";
  }
  String txt = l1 + "\n" + l2 + "\n" + l3;
  if (txt == oled_prev) return;
  oled_prev = txt;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  display.print(l1);
  display.setCursor(0, 11); display.print(l2);
  display.setCursor(0, 22); display.print(l3);
  // The 512-byte OLED transfer needs a wider limit than sensor reads, but is
  // still bounded below the 400 ms motor watchdog budget.
  Wire.setTimeOut(20);
  display.display();
  Wire.setTimeOut(3);
}

void setup() {
  Serial.begin(115200);
  ServoSerial.begin(1000000, SERIAL_8N1, S_RX, S_TX);

  // Stop first: if only the ESP32 resets while servo power remains, the
  // previous velocity target must not survive the multi-second sensor setup.
  SERVO_INIT();
  ensure_servo_config();
  stop_motors();

  Wire.begin(S_SDA, S_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(20);
  delay(500);

  Wire.beginTransmission(SCREEN_ADDRESS);
  if (Wire.endTransmission() == 0) {
    oled_available = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  }
  if (!oled_available) {
    Serial.println("OLED not detected; display updates disabled");
  }
  // Sensor transactions are short at 400 kHz. Three milliseconds bounds all
  // I2C retry paths so a failed bus cannot starve the 400 ms motor watchdog.
  Wire.setTimeOut(3);
  imuInit();

  // A10: INA219 주소 스캔 (보드 리비전별 0x40~0x45)
  for (uint8_t a = 0x40; a <= 0x45; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      static INA219_WE dev(a);
      if (dev.init()) {
        ina219 = &dev;
        Serial.printf("INA219 ready at 0x%02X\n", a);
      } else {
        Serial.printf("INA219 init failed at 0x%02X\n", a);
      }
      break;
    }
  }

  ensure_servo_config();                    // 센서 초기화 중 전원 투입된 서보도 재탐지
  stop_motors();                            // A9: 첫 유효 명령 전까지 정지
}

void loop() {
  while (Serial.available()) feed_rx(Serial.read());

  uint32_t now = millis();

  // A9: 워치독 — 명령 두절 시 1회 정지 (버스 스팸 방지)
  if (!watchdog_stopped && now - last_cmd_ms > CMD_TIMEOUT_MS) {
    stop_motors();
    watchdog_stopped = true;
  }

  static uint32_t t_state = 0;
  if (now - t_state >= STATE_PERIOD_MS) {
    t_state = now;
    send_state();
  }

  // 1초마다 서보 생존 확인 + 부활 시 모드 재설정 (배터리 늦은 연결 대응)
  static uint32_t t_ping = 0;
  if (now - t_ping >= 1000) {
    t_ping = now;
    ensure_servo_config();
  }

  static uint32_t t_oled = 0;
  if (now - t_oled >= OLED_PERIOD_MS) {
    t_oled = now;
    update_oled(ina219 ? (uint16_t)(ina219->getBusVoltage_V() * 1000.0f) : 0);
  }
}
