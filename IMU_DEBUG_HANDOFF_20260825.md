# JD-AMR General Driver·IMU 복구 인계 — 2026-08-25

## 결론

- General Driver 보드의 QMI8658C IMU는 사용할 수 있다. 정지 상태 실측에서 가속도 크기 약
  `0.95 g`, 자이로 p95 `1.18 deg/s`, 50 Hz 상태 스트림 CRC/순번 오류 0이었다.
- 기존 약 `0.21 g` 가속도는 센서 불량이 아니라 펌웨어의 시작 보정 오류였다. 한 자세에서
  중력까지 가속도 오프셋으로 제거하고, 누적 중인 오프셋을 다시 입력에서 빼던 것이 원인이다.
- 가속도 시작 보정은 제거하고 자이로 바이어스만 정지 보정한다. 가속도 축별 보정은 나중에
  6면 보정으로 해야 한다.
- 최종 USB 검증 시 메인 배터리/서보 전원 레일은 꺼졌거나 분리된 상태였다. INA219가 약
  `0.15 V`, 플래그가 `0x07`(좌·우 서보 읽기 오류 + 명령 워치독 정지)이었던 것은 이 전원
  상태와 일치한다. 직전 메인 전원 연결 검증에서는 플래그 `0x04`, 배터리 약 `10.2 V`,
  엔코더/서보 오류 0이었다.
- 3S 배터리가 약 `10.2 V`까지 내려갔으므로 충전 전에는 주행·라이다 장시간 구동을 금지한다.
  자율주행 시작/지속 차단 기준은 `10.5 V`다.

## 하드웨어 식별

- Raspberry Pi 4 + Waveshare General Driver for Robots 40핀 헤더 결합
- 제어기: ESP32-WROOM-32
- IMU: QMI8658C, 자력계: AK09918C
- 전압계: INA219 (`0x42` 실측)
- 구동 서보: STS3215
- 노트북 General Driver 포트: `/dev/ttyUSB0`
  - CP2102N VID:PID `10c4:ea60`
  - 시리얼 `2ed30009fe73ef119409cf8c8fcc3fa0`
- 노트북 `/dev/ttyACM0` CH343은 SO101 팔이며 General Driver가 아니다.
- 보드 버튼:
  - USB Type-C에 가장 가까운 버튼: BOOT/다운로드
  - 40핀 헤더 쪽 뒤 버튼: EN/리셋

## USB 업로드 시 UART 충돌 방지

General Driver의 ESP32 UART는 Pi 헤더와 CP2102N USB에 병렬로 연결된다. USB 업로드 전에 Pi
GPIO14를 반드시 입력으로 풀어야 한다. 그렇지 않으면 esptool이 `Wrong boot mode detected
(0x13)`으로 실패할 수 있다.

Pi에서 USB 연결 전/업로드 전 실행:

```bash
sudo systemctl stop jdamr-base.service
echo fe215040.serial | sudo tee /sys/bus/platform/drivers/bcm2835-aux-uart/unbind
echo 526 | sudo tee /sys/class/gpio/export
echo in | sudo tee /sys/class/gpio/gpio526/direction
```

`gpiochip512` 기준 GPIO14의 전역 번호가 526이다. 2026-08-25 최종 확인 상태는 서비스
`inactive`, UART `unbound`, `gpio526=in`이다.

USB Type-C를 **물리적으로 뽑은 뒤에만** Pi UART를 복구한다:

```bash
echo 526 | sudo tee /sys/class/gpio/unexport
echo fe215040.serial | sudo tee /sys/bus/platform/drivers/bcm2835-aux-uart/bind
sudo systemctl start jdamr-base.service
```

Type-C가 연결된 채 위 복구 명령을 실행하면 두 UART 송신기가 다시 충돌할 수 있다.

## 원본 백업과 최종 펌웨어

- 수정 전 4 MiB 전체 플래시 백업:
  `/home/lim/jdamr_controller_flash_backup_pre_imu_fix_20260825.bin`
- 백업 SHA-256:
  `ccb02962de19f6ebceb2351f2872710a9bb7c7597e2e58526a766a693499025e`
- 최종 앱 바이너리:
  `/tmp/jdamr-fw-build-general-driver-final-20260825/jdamr_cube_fw.ino.bin`
- 최종 앱 SHA-256:
  `e9c4b55d59e32749d55e083e44e746d2168de2894bbab4674f413cfab3a42cdf`
- Arduino ESP32 빌드: 플래시 24%, RAM 7%
- 업로드한 네 영역(bootloader, partition table, boot_app0, app)은 각각 쓰기 해시 검증을
  통과했다. NVS 영역은 덮어쓰지 않았다.

## 적용한 펌웨어·호스트 개선

- QMI8658C Ax~Gz 12바이트를 한 번의 coherent burst로 읽는다.
- SyncSample 잠금 명령 순서를 데이터시트에 맞췄다.
- `CTRL1=0x40`: 주소 자동 증가, little-endian, 사용하지 않는 INT 출력 비활성화
- `CTRL2=0x33`: 가속도 설정
- `CTRL3=0x63`: 데이터시트에서 유효한 최대 범위 `±1024 dps`
- `CTRL5=0x11`: 가속도·자이로 LPF mode 00, 약 `23.9 Hz`
- `CTRL7=0x83`: 가속도·자이로 + SyncSample
- 시작 보정은 최소 40개의 coherent sample이 있을 때만 성공하며, 가속도는 건드리지 않고
  자이로 바이어스만 뺀다.
- 패킷 가속도는 QMI가 반환하는 mg를 다시 1000배 하지 않고 그대로 포화·반올림한다.
- QMI8658/AK09918/INA219/좌우 서보 오류를 상태 플래그로 분리했다.
- 센서 오류 프레임은 ROS IMU/자력계로 오래된 값을 재발행하지 않는다.
- INA219 미검출은 BatteryState를 `present=false`, 전압 `NaN`으로 발행한다.
- 상태 스트림이 0.5초 이상 끊기거나 순번이 비정상적으로 건너뛰면 해당 엔코더 구간을 적분하지
  않아 재연결 뒤 오도메트리 점프를 막는다.
- 센서 초기화보다 먼저 STS3215에 정지를 보내 ESP32 리셋 중 이전 속도 지령이 남지 않게 했다.
- OLED가 없으면 시작 때 한 번만 탐지하고 이후 I2C 전송을 하지 않는다. 센서 I2C timeout은
  3 ms, OLED는 20 ms로 제한했다.
- 기본 `jdamr_cube_bringup.launch.py`는 더 이상 구형 `/dev/ttyAMA0`·Python 드라이버·LD14를
  실행하지 않고 검증된 `real_bringup.launch.py`를 호출한다.

## 최종 실측 증거

메인 배터리/서보 레일이 켜져 있던 이전 1,027프레임:

- CRC 0, 순번 유실 0
- 플래그 `0x04`만 존재(명령이 없어 워치독 정지), 센서·서보 오류 0
- 가속도 norm 평균 `954.15 mg`, p95 `988.56 mg`
- 자이로 norm p95 `1.16 deg/s`
- 자력 norm 평균 `68.6 uT`
- 배터리 평균 `10.228 V`, 최소 `10.192 V`, 최대 `10.256 V`
- 좌·우 엔코더 변화 0

LPF 최종 펌웨어 업로드 후, 메인 배터리/서보 레일이 꺼진 상태의 598프레임:

- QMI 설정 readback `CTRL1=40 CTRL2=33 CTRL3=63 CTRL5=11 CTRL7=83`
- QMI 원시 평균 가속도 `(-75.1, 144.1, -936.0) mg`, norm `950.0 mg`
- 자이로 시작 바이어스 `(1.827, -2.690, 0.388) deg/s`, coherent sample 50개
- AK09918 ID `0x480C`, INA219 `0x42`
- CRC 0, 순번 유실·중복 0, 평균 주기 `20.000 ms`
- 가속도 norm 평균 `951.16 mg`, p95 `954.37 mg`
- 자이로 norm 평균 `0.448 deg/s`, p95 `1.183 deg/s`
- 자력 norm 평균 `69.72 uT`, 표준편차 `1.62 uT`
- 좌·우 엔코더 변화 0
- 플래그 `0x07`, INA219 약 `0.15 V`: 메인 배터리/서보 레일 부재 상태

최신 C++ 호스트 드라이버도 외부 ROS 그래프와 격리한 도메인에서 연결 검증했다. `/cmd_vel`은
존재하지 않는 검증용 토픽으로 remap하여 시작·종료 정지 패킷 외에는 전송하지 않았다.

- `/imu/data_raw`: 가속도 `(-0.7845, 1.3925, -9.1888) m/s²`, 자이로 정상 발행
- `/odom`: 위치 `(0, 0)`, 선속도·각속도 모두 0
- `/battery_state`: `present=true`, `0.152 V` — INA219는 존재하고 메인 전원 레일만 부재
- QMI/AK/INA 오류 로그 없음, 전원 없는 좌·우 서보 오류만 진단됨

## 다음 안전 검증 순서

1. General Driver Type-C를 물리적으로 뽑는다.
2. 3S 배터리를 전용 충전기로 충전하고 셀 상태를 확인한다.
3. 위 절차대로 Pi UART를 bind하고 `jdamr-base.service`를 시작한다.
4. 바퀴를 바닥에서 띄운 상태에서 먼저 `/imu/data_raw`, `/battery_state`, `/odom` 정지 검증을 한다.
5. 가속도 6면 자세로 각 축 방향·부호·스케일을 확인한다.
6. 손으로 좌우 회전시켜 자이로 Z 부호와 적분각을 확인한다.
7. 자력계는 최종 장착 위치에서 8자 회전으로 hard/soft-iron 보정을 한다.
8. 위 동적 검증이 끝나기 전까지 Cartographer는 `use_imu_data=false`를 유지한다.
9. 실제 주행은 충전된 배터리, 작업자 즉시 정지 수단, 저속 제한, 전방 장애물 여유가 확보된
   뒤 별도 단계로 한다.
