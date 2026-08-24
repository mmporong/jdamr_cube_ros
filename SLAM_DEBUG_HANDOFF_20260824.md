# JD-AMR SLAM 디버깅 핸드오프 (2026-08-24)

다음 세션이 이어받기 위한 정리. 이번 세션은 **SLAM 지도 품질 문제를 못 풀었다.** 하드웨어·주행은 다 정상으로 확인됐고, 남은 건 순수 SLAM 정합 문제다.

## 접속·운영

- **로봇 접속**: `jdamr.local` (mDNS). **IP가 DHCP로 계속 바뀐다**(오늘 .159↔.160 반복). IP 직접 쓰지 말고 `jdamr.local` 사용. 안 풀리면 `getent hosts jdamr.local`.
- **DDS**: `ROS_DOMAIN_ID=12`, 노트북은 `ROS_STATIC_PEERS=<로봇IP>` 또는 `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` 필요(강의실 AP 멀티캐스트 안 흘림).
- **파이 DDS 반복 사망**: 서비스 재기동·`ros2 daemon stop`·CLI 조회를 반복하면 FastDDS 공유메모리가 오염돼 파이 내부 토픽이 통째로 0이 된다(scan/odom/battery 다 끊김). 복구: 전체 정지 → `sudo rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* /dev/shm/sem.*` → 서비스 재기동. **확인은 CLI 말고 단일 python 노드로**(CLI가 상태를 더 흔든다). 근본 해결은 FastDDS를 UDP 전송으로 바꾸는 것(미적용).
- **webteleop 부팅 미기동 버그**: systemd ordering cycle 때문에 부팅 시 `jdamr-webteleop`이 자동으로 안 뜬다. 매 부팅 후 `sudo systemctl start jdamr-webteleop` 수동 실행 필요. **영구 수정 미적용**(유닛 파일 `After=` 의존 정리 필요).
- **웹 조종**: `http://jdamr.local:8080` (WASD/터치). 브라우저 페이지가 낡으면(IP 바뀐 뒤) 버튼이 죽은 옛 주소로 POST해 안 먹는다 → **하드리프레시(Ctrl+Shift+R)**.
- **지도 초기화 스크립트**: 파이에 `~/reset_map.sh` (현재 지도 저장 → 카토그래퍼 재시작). `ssh lim@jdamr.local '~/reset_map.sh'`.

## 하드웨어 상태 (전부 정상 확인)

- **배터리 3S2P** — 완충 12.6V. 오늘 9.81V로 방전됐다가(주행 불가·파이 브라운아웃의 원인) 충전기로 충전. 저전압이면 서보가 명령 무시하고 파이가 네트워크에서 사라진다. 충전은 **로봇 OFF**로, 12V 어댑터 금지(3S 리튬이온 전용 충전기 사용).
- **서보 L·R 정상**, INA219(전압계) 정상.
- **IMU**: 칩 살아있음(WHO_AM_I 0x05). **자이로 정상**(720° 회전 1.3% 오차). **가속도계 이상**(정지 |a|=19.28, 정상 9.81의 ~2배 + 방향 이상). 원인은 [[jdamr-oled-kills-i2c-bus]] — **불량 OLED가 I2C 버스를 끌어내려 IMU·INA219까지 죽였다. OLED를 뽑아야 IMU가 산다. 다시 꽂지 말 것.**
- **주행 정상** — 직접 cmd_vel로 바퀴 돈다(12cm 실측). base·서보·배터리 다 정상.
- **오도메트리 훌륭** — 폐루프 위치 오차 0.2~0.3m. 회전 환산 1.3%.

## 핵심 문제 — SLAM 정합 실패 (미해결)

증상: P자로 돌아 **제자리 복귀했는데 SLAM은 1.4~1.7m 떨어진 곳을 가리킨다.** 지도는 방사형 가시 + 벽이 두 겹으로 그려짐(스캔매칭이 같은 벽을 두 곳에).

**일관된 4분면 판정 (3회 반복)**:
| 실험 | odom 폐루프 | SLAM 폐루프 | 판정 |
|---|---|---|---|
| ①이식 TB3 설정(상관on·1e3) | 0.12m | 1.73m | SLAM이 좋은 odom을 뒤엎음 |
| ②IMU on | 0.22m | 1.50m | 차이 없음 |
| ③odom-trust(상관off·1e5) | 0.33m | 1.40m | 여전히 실패 |

**핵심**: odom은 매번 0.2~0.3m로 훌륭한데 스캔매칭이 1.5m를 튄다. 세 설정 다 못 고쳤다.

## 이번 세션에서 시도한 것과 결과 (전부 실패)

1. **터틀봇 성공 레시피 이식**(`carto_lds01.lua` → `jdamr_cube_2d_corridor.lua`): 상관매칭 on, 균형 가중치 1e3/1e3, IMU off. → 실패(1.7m). **잘못된 판단**: TB3는 odom이 나빠서 공격적 스캔매칭이 필요했지만, JD-AMR은 odom이 좋아서 그게 독이 됐다.
2. **IMU 켜기**: base_link→imu_link identity static TF + `use_imu_data=true` + `imu:=/imu/data_raw` 리맵. → 도움 안 됨(1.5m). 가속도계가 이상해 중력 정렬이 안 맞는 게 원인일 수 있음(자이로만 쓰게 하는 설정 미시도).
3. **오도메트리 신뢰**: 상관매칭 off, odometry_translation_weight 1e5, rotation 1e4, IMU off. → 여전히 실패(1.4m). 현재 소스·파이 설정이 이 상태.

## 다음 세션 가설 (우선순위)

1. **스캔 타임스탬프 지연 재확인** — 이 세션에서 `/scan` stamp가 수신보다 +140ms 과거로 관측됨. 스캔-odom 시간 정렬이 틀어지면 어떤 가중치로도 못 고친다. 카토그래퍼가 스캔을 잘못된 과거 pose에 놓는지 확인. **가장 유력.**
2. **8/18 강의실 성공본(_real, 5.49%)으로 되돌려 baseline 재현** — 같은 로봇에서 됐던 설정이다. 지금 방에서 `_real` 그대로 돌려 되는지부터 확인(환경 문제 vs 설정 문제 분리). 이번 세션은 전부 `_corridor`만 건드렸다.
3. **ceres_scan_matcher 가중치까지 올리기** — POSE_GRAPH(전역) odom 가중치만 올렸지 로컬 ceres(translation/rotation_weight=10)는 안 건드렸다. 로컬 스캔매칭이 per-scan으로 드리프트하는 걸 막으려면 여기도 올려야 할 수 있다.
4. **motion skew** — 6Hz 라이다 + `num_subdivisions=1`. 회전 구간 왜곡. subdivisions 5~10으로.
5. **rosbag 녹화 후 오프라인 디버깅** — 라이브 튜닝이 3번 실패했으니, `/scan /odom /tf /imu`를 bag으로 떠서 오프라인에서 여러 설정을 재현 가능하게 돌리는 게 효율적. 라이브 튜닝 그만.

## 파일 위치

- 카토그래퍼 설정: `jdamr_cube_cartographer/config/jdamr_cube_2d_corridor.lua`(현재 odom-trust), `_real.lua`(8/18 강의실 성공본, num_subdivisions만 10으로 바뀜)
- 폐루프 측정 도구: 파이 `/tmp/pose.py` (map/odom→base_footprint 비교)
- 프레임 직접 읽기: 파이 `/tmp/readframe3.py` (ESP32 배터리·서보 플래그, ROS 우회)
- 지난 이론·절차: `~/physical-ai-lab/learning/M02_linux_ros2/slam_nav2/JDAMR_NEXT_SESSION.md`
- 결함 원장: `reference/JDAMR_Cube_조사기록.md`

## 라이다 교체 관련 (미결 결정)

LD14(현재) vs YDLIDAR G4(16m·고샘플링) 교체는 **SLAM이 고쳐진 뒤** 판단. 라이다가 병목인지 미확인. LD14로 먼저 지도가 되면 교체 불필요. 중앙 이동은 리마운트만이면 LD14로 먼저, 커스텀 브래킷이면 목표환경(복도=G4/방=LD14)으로.
