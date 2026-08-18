/*
 * jdamr_cube_fw — JD-AMR Cube ESP32 펌웨어 (강사원본 ros_firmware.ino 대체)
 *
 * 보드: Waveshare General Driver for Robots (ESP32-WROOM-32)
 * 서보: Feetech STS3215 ×2 (좌 ID=1, 우 ID=2), 시리얼 버스 1,000,000 baud
 * 센서: QMI8658C(6축) + AK09918(지자기 3축), INA219(전압), SSD1306 128×32
 *
 * 이 판이 고치는 강사원본 결함 (번호 = ~/jdamr_cube_bringup/JDAMR_Cube_조사기록.md):
 *   A1  자이로 Y/Z·지자기 3축 미전송            → 9축 전부 전송
 *   A2  가속도 ×1000 / 자이로 ×100 스케일 혼재   → 스케일을 패킷 규격에 명시(아래 표)
 *   A3  체크섬 항상 0x00                        → CRC-8/MAXIM 실제 계산
 *   A4  헤더 재동기화 없음                       → 바이트 단위 상태기계로 재동기
 *   A5  후진·회전 분기 부재                      → 부호 있는 바퀴별 속도라 방향 분기 자체가 없음
 *   A6  100ms마다 서보 위치 4회 조회             → 주기당 1회 읽어 재사용
 *   A7  100ms마다 OLED 전체 프레임 전송(블로킹)   → 500ms 주기 + 내용 변한 때만
 *   A8  0~4095 랩어라운드 언랩 없음              → int32 누적 위치로 언랩
 *   A9  cmd 워치독 없음                         → 400ms 무명령 시 정지
 *   A10 INA219 0x42 하드코딩                    → 0x40~0x45 스캔
 *   A11 좌우 부호 규약 미문서화                  → "+ = 로봇 전진" 통일, 우측 반전은 여기서만
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
 *   int32  left_pos     좌바퀴 누적 위치 [counts]  (언랩, + = 전진)
 *   int32  right_pos    우바퀴 누적 위치 [counts]  (언랩, + = 전진)
 *   int16  accel[3]     가속도 [mg]        (raw g ×1000)
 *   int16  gyro[3]      각속도 [0.01 dps]  (raw dps ×100)
 *   int16  mag[3]       지자기 [0.1 uT]    (raw uT ×10)
 *   uint16 batt_mV      배터리 전압 [mV]
 *   uint8  flags        bit0 좌서보 읽기실패  bit1 우서보 읽기실패
 *                       bit2 워치독 정지중    bit3 INA219 미검출
 *
 * 물리 제원(바퀴 지름·트레드)은 펌웨어에 두지 않는다 — 오도메트리 환산은
 * ROS 노드 파라미터가 단일 출처다 (counts/s ↔ m/s 변환은 호스트 책임).
 */

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <INA219_WE.h>
#include "IMU.h"
#include "STSServoDriver.h"

// ── 핀·주소 (강사원본과 동일) ──
#define S_SCL 33
#define S_SDA 32
#define S_RX  18
#define S_TX  19
#define SCREEN_ADDRESS 0x3C

// ── 규격 상수 ──
// 2026-08-14 실기 확정 — 단일 바퀴 시험으로 판정: "ID1 전진" 지령에 로봇이
// 좌회전 = ID1 이 실물 오른쪽. 가정(좌=1)이 뒤집혀 있었다.
static const uint8_t  LEFT_ID  = 2;
static const uint8_t  RIGHT_ID = 1;
static const uint32_t STATE_PERIOD_MS = 20;    // 50Hz 상태 송신
static const uint32_t OLED_PERIOD_MS  = 500;   // A7: OLED는 0.5s + 변경시만
static const uint32_t CMD_TIMEOUT_MS  = 400;   // A9: 워치독
static const int16_t  VEL_LIMIT = 3400;        // STS3215 속도 지령 상한 [counts/s]

HardwareSerial ServoSerial(2);
Adafruit_SSD1306 display(128, 32, &Wire, -1);
STSServoDriver st;
INA219_WE *ina219 = nullptr;             // A10: 주소 스캔 후 생성

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
static int32_t pos_total[2] = {0, 0};
static int16_t pos_last[2]  = {0, 0};
static bool    pos_init[2]  = {false, false};

// ── 런타임 상태 ──
static uint32_t last_cmd_ms = 0;
static bool     watchdog_stopped = true;   // 부팅 직후는 정지 상태로 시작
static uint8_t  seq = 0;
static int16_t  cmd_left = 0, cmd_right = 0;
static String   oled_prev = "";
static String   oled_scan = "scan...";     // 부팅 ID 스캔 결과 (OLED 2행 고정)

// 서보 생존·설정 상태.
// 주의: 이 STS_servos 라이브러리는 읽기 타임아웃에 -1 이 아니라 0 을 돌려준다
// (readTwoBytesRegister: rc<0 → return 0). 그래서 생존 판정은 ping() 으로만 하고,
// 배터리가 ESP32 보다 늦게 연결되는 경우(부팅 때 setMode 유실)를 죽음→부활
// 전이 감지로 잡아 모드를 재설정한다.
static bool servo_ok[2] = {false, false};

static void ensure_servo_config() {
  const uint8_t ids[2] = {LEFT_ID, RIGHT_ID};
  for (int i = 0; i < 2; i++) {
    const bool alive = st.ping(ids[i]);
    if (alive && !servo_ok[i]) {
      st.setMode(ids[i], VELOCITY);        // 부활 시점에 모드 재주입
    }
    servo_ok[i] = alive;
  }
}

// A11: 부호 규약의 유일한 반전 지점.
// 2026-08-14 실기 확정 (관측 3건으로 유일해): ID1=실물 오른쪽(+ 가 전진),
// ID2=실물 왼쪽(− 가 전진). 미러 반전은 왼쪽(ID2)에 있다 — 강사원본 규약
// (LEFT +, RIGHT −)과 반대이므로 코드를 옮길 때 주의.
static inline void set_wheel_velocity(int16_t left_fwd, int16_t right_fwd) {
  st.setTargetVelocity(LEFT_ID,  constrain((int16_t)-left_fwd, -VEL_LIMIT, VEL_LIMIT));
  st.setTargetVelocity(RIGHT_ID, constrain(right_fwd, -VEL_LIMIT, VEL_LIMIT));
}

static void stop_motors() {
  set_wheel_velocity(0, 0);
  cmd_left = cmd_right = 0;
}

// ── 수신 명령 처리 ──
static void handle_cmd(const uint8_t *p, uint8_t len) {
  if (len != 5) return;
  uint8_t cmd = p[0];
  int16_t left, right;
  memcpy(&left,  p + 1, 2);
  memcpy(&right, p + 3, 2);

  if (cmd == 0x01) {                       // 속도 지령
    cmd_left = left; cmd_right = right;
    set_wheel_velocity(cmd_left, cmd_right);
    last_cmd_ms = millis();
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
  uint8_t flags = watchdog_stopped ? 0x04 : 0x00;
  if (!ina219) flags |= 0x08;

  // A6: 서보 위치는 여기서 한 번만 읽는다 (OLED도 이 값을 재사용)
  int raw[2] = { st.getCurrentPosition(LEFT_ID), st.getCurrentPosition(RIGHT_ID) };
  for (int i = 0; i < 2; i++) {
    if (!servo_ok[i]) { flags |= (1 << i); continue; } // ping 실패 → 적분 제외
    int16_t r = (int16_t)(raw[i] & 0x0FFF);
    if (!pos_init[i]) { pos_last[i] = r; pos_init[i] = true; }
    int16_t d = (int16_t)((r - pos_last[i]) & 0x0FFF); // A8: 12비트 차분
    if (d > 2048) d -= 4096;                           //     부호 확장
    // 이 라이브러리는 읽기 타임아웃을 0 으로 돌려준다 — 20ms 에 물리적으로
    // 불가능한 점프(|d|>300, 상한 지령 68의 4배)는 글리치로 보고 버린다.
    if (d > 300 || d < -300) { flags |= (1 << i); continue; }
    pos_last[i] = r;
    // 왼쪽(ID2)이 미러 — 축 + 가 로봇 후진. 전진 = + 로 통일해 누적
    pos_total[i] += (i == 0) ? -d : d;
  }

  imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  uint16_t mv = ina219 ? (uint16_t)(ina219->getBusVoltage_V() * 1000.0f) : 0;

  uint8_t f[3 + 30 + 1];                   // 헤더2 + LEN + payload30 + crc
  f[0] = 0xA5; f[1] = 0x5A; f[2] = 30;
  uint8_t *p = f + 3;
  *p++ = seq++;
  memcpy(p, &pos_total[0], 4); p += 4;
  memcpy(p, &pos_total[1], 4); p += 4;
  int16_t v;
  v = (int16_t)(stAccelRawData.X * 1000); memcpy(p, &v, 2); p += 2;  // A1·A2:
  v = (int16_t)(stAccelRawData.Y * 1000); memcpy(p, &v, 2); p += 2;  // 9축 전부,
  v = (int16_t)(stAccelRawData.Z * 1000); memcpy(p, &v, 2); p += 2;  // 스케일은
  v = (int16_t)(stGyroRawData.X  * 100);  memcpy(p, &v, 2); p += 2;  // 헤더 표가
  v = (int16_t)(stGyroRawData.Y  * 100);  memcpy(p, &v, 2); p += 2;  // 단일 규격
  v = (int16_t)(stGyroRawData.Z  * 100);  memcpy(p, &v, 2); p += 2;
  v = (int16_t)(stMagnRawData.s16X * 10); memcpy(p, &v, 2); p += 2;
  v = (int16_t)(stMagnRawData.s16Y * 10); memcpy(p, &v, 2); p += 2;
  v = (int16_t)(stMagnRawData.s16Z * 10); memcpy(p, &v, 2); p += 2;
  memcpy(p, &mv, 2); p += 2;
  *p++ = flags;
  f[sizeof(f) - 1] = crc8(f + 2, 31);      // A3: LEN+payload 실제 CRC
  Serial.write(f, sizeof(f));
}

// ── OLED (A7: 저주기 + 변경시만) ──
static void update_oled(uint16_t mv) {
  // 128x32 3줄 배치. 현장에서 로봇 옆에 서서 보는 순서로 놓는다:
  //   1행 전압 + 무장상태  — 충전 시점과 "왜 안 움직이나"의 답
  //   2행 바퀴 지령        — 조종이 닿고 있는지
  //   3행 서보 ID·건강     — 버스가 살아 있는지 (부팅 스캔 결과 상시 노출)
  // 변경 없으면 그리지 않는다 — display() 는 I2C 전체 프레임이라 수십 ms 블로킹(A7).
  // 전압은 0.1V 로 양자화돼 있어 미세 변동으로 매번 다시 그리지 않는다.
  String l1 = String(mv / 1000.0f, 2) + "V  " + (watchdog_stopped ? "IDLE" : "ARM");
  String l2 = "L" + String(cmd_left) + "  R" + String(cmd_right);
  String l3 = oled_scan + "  " +
              (servo_ok[0] && servo_ok[1] ? "SRV ok"
                                          : (String("SRV ") + (servo_ok[0] ? "" : "L")
                                                            + (servo_ok[1] ? "" : "R") + "!"));
  String txt = l1 + "\n" + l2 + "\n" + l3;
  if (txt == oled_prev) return;
  oled_prev = txt;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  display.print(l1);
  display.setCursor(0, 11); display.print(l2);
  display.setCursor(0, 22); display.print(l3);
  display.display();
}

void setup() {
  Serial.begin(115200);
  ServoSerial.begin(1000000, SERIAL_8N1, S_RX, S_TX);
  Wire.begin(S_SDA, S_SCL);
  delay(500);

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  imuInit();

  // A10: INA219 주소 스캔 (보드 리비전별 0x40~0x45)
  for (uint8_t a = 0x40; a <= 0x45; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      static INA219_WE dev(a);
      dev.init();
      ina219 = &dev;
      break;
    }
  }

  SERVO_INIT();
  // 부팅 ID 스캔 — 결과를 OLED 1행에 상시 표시 (실물에서 ID 논쟁 종결용).
  // 배터리가 아직 없으면 "ID:none" 이 뜨고, 연결되면 ensure_servo_config 가
  // 생존을 다시 잡으면서 모드도 재설정한다.
  oled_scan = "ID:";
  for (uint8_t id = 1; id <= 12; id++) {
    if (st.ping(id)) { oled_scan += String(id) + " "; }
  }
  if (oled_scan == "ID:") { oled_scan = "ID:none"; }

  ensure_servo_config();                    // 살아 있으면 VELOCITY 모드 진입
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
