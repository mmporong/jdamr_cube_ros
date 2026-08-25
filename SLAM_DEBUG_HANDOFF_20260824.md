# JD-AMR SLAM 디버깅 핸드오프 (2026-08-24)

다음 세션이 이어받기 위한 정리. 최초 조사에서는 SLAM 지도 품질 문제를 풀지 못했지만, 같은 날 후속 조사에서 **LD14 드라이버 결함 2개와 6Hz 폴링 결함을 확인·수정했고, 사용자 수동 주행에서 1.5m 점프와 굵은 이중벽이 사라졌다.** 이후 사용자가 예비 YDLIDAR G4를 판 중앙에 장착해 G4 전용 드라이버까지 배포했다. G4 정지 입력과 사용자 수동 주행 검증도 통과했다. **현재 성능 우선 라이다는 G4로 확정하며, SLAM 입력 시간 결함은 해결 완료로 판정한다.**

## 17:55 G4 전환 — 장착·입력·수동 주행 검증 완료

### 하드웨어·좌표

- SDK probe로 **YDLIDAR G4, firmware 3.2, hardware 3, 9K sample, 10Hz 설정**을 확인했다.
- 사용자가 G4를 판의 한가운데에 장착했다. **5핀 케이블 출구가 로봇 후면**을 향한다. G4의 0° 기준이 케이블 출구이므로 `base_link → laser_link`는 `x=0, y=0, z=0.1m, yaw=π`다. live TF도 `[0, 0, 0.1]`, yaw `180.000°`로 확인했다.
- CP2102 어댑터를 udev로 `/dev/ydlidar_g4`에 고정했다. 규칙 파일은 `ydlidar_g4_ros2/udev/99-ydlidar-g4.rules`, Pi 설치 위치는 `/etc/udev/rules.d/99-ydlidar-g4.rules`다.
- G4 크기·질량을 URDF에 반영했다(Ø72.3×41.2mm, 214g). 센서 높이 `z=0.1m`는 기존 광학 중심 높이를 유지했다.

### 드라이버와 추가로 발견한 시간 결함

- 새 `ydlidar_g4_ros2` 패키지는 공식 YDLidar-SDK 1.2.20의 필요한 소스만 commit `01cdda4f2b36dff2a706d0535c64228d863c7411`에서 vendoring했다.
- G4의 실제 시계방향 취득 배열은 유지하고 ROS 각도만 반전한다. 따라서 `/scan`은 **음수 `angle_increment` + 양수 `time_increment`**이며, 공간 인덱스와 취득 시간 인덱스가 일치한다.
- SDK는 첫 패킷 시각을 scan header로 쓰지만 `scan_time`은 현재·직전 마지막 패킷 시각 차이로 만든다. 이 값이 드물게 약 206ms로 두 배가 되어 Cartographer가 한 스캔의 대부분(최대 592점)을 `Dropped earlier points`로 버리는 현상을 재현했다.
- 시작 직후에는 `scan_time=0`인 부분 회전도 한 번 나온다. 이를 100ms 완전 회전으로 간주하면 다음 header를 최대 34ms 미래로 밀게 된다.
- 수정: 부분 회전과 첫 완전 회전은 기준 수집용으로 publish하지 않고 두 번째 완전 회전부터 시작한다. 설정 주기의 80~120% 밖 `scan_time`은 10Hz 주기로 fallback한다. 연속 scan 끝/시작은 float32 반올림 여유 1µs를 포함해 단조 증가시키며, 1ms 초과 header 보정은 경고한다.
- publisher는 RELIABLE KeepLast(10)이다. 현재 `/scan` publisher 1개, subscriber는 Cartographer와 RViz 2개이며 QoS가 호환된다.

### 최종 정지 검증 증거

- 노트북과 Pi 모두 빌드 성공. G4 변환·시간 회귀 테스트 **8개 gtest / ament 합계 9 tests**, 오류·실패 0.
- Pi 60.033초 실측: 582 frames, **9.6781Hz**, 927~931 beams/scan(중앙값 929), 유효 빔 중앙값 608개·65.45%.
- `scan_time` 101.650~105.150ms(중앙값 103.330ms), header gap 101.590~105.038ms.
- 다음 scan 시작과 이전 scan 끝의 최소 여유 0.715µs, 전체/유효 광선 모두 overlap 0건, 비단조 header 0건, 150ms 초과 gap 0건.
- `수신 - header - scan_time` 잔여 지연 중앙값 2.906ms, p95 3.967ms, 최대 10.721ms.
- 새 Cartographer 로그: `Dropped earlier points=0`, `Ignored subdivision=0`, `FATAL=0`, scan 9.67~9.68Hz. 드라이버 warm-up 뒤 checksum·scan read·timeout·큰 timestamp 보정 경고도 0건.
- G4는 `jdamr-base.service`에서 계속 회전·publish 중이고 Cartographer와 occupancy grid도 실행 중이다.
- G4 전환 전 원격 파일 백업: `~/jdamr_backups/g4-swap-20260824T1652KST`.

### 17:55 사용자 수동 주행 최종 결과

사용자가 웹 조종으로 약 117.4초 동안 직접 주행했고, **로봇을 들어 올리기 전 RViz 지도는 깔끔하게 생성됐다.** 오른쪽·하단의 긴 벽은 단일 선으로 정합됐고, 과거의 일정 간격 평행 이중벽과 1m급 점프는 보이지 않았다. G4의 180° yaw 보정이나 중앙 장착이 틀렸다는 시각적 근거도 없다.

- raw bag: `~/bags/g4_userloop_reset_20260824T175151` (195.889초, 63,955 messages)
- 로컬 보존본: `~/jdamr_artifacts/g4_userloop_reset_20260824T175151`
- `/scan` 1,883개, **9.652Hz**, header 간격 중앙값 103.583ms·최대 107.697ms, 150ms 초과 gap 0건.
- 다음 scan과 이전 scan 사이 최소 여유 **+0.715µs**, 시간 overlap·역전 0건. recorder 초기 backlog 3프레임을 제외한 기록 지연은 중앙값 1.66ms·p95 2.67ms·최대 4.40ms다.
- 해당 Cartographer 로그의 `Dropped earlier points=0`, `Ignored subdivision=0`, `FATAL=0`. 후단 constraint score는 대체로 65~84%, 보정량은 주로 1~4cm 범위였다.
- 리프트 전 `map→odom` 변화의 p99는 약 3.1cm로 안정적이었지만, bag 상대시각 109~113초에는 0.28~0.42m·6.1~8.5°의 연속 보정 구간이 있었다. 사용자 화면에서는 지도가 깨끗했고 과거 1.5m급 실패보다 작지만, 다음 동일 경로 2회에서 반복되는지 확인한다.
- 마지막 주행 명령 뒤 약 2.18초부터 스캔과 SLAM 자세가 급변했다. 사용자가 로봇을 들어 올린 시점과 일치하며, 바퀴 odom은 고정된 채 라이다만 물리적으로 이동해 scan-odom 일관성이 깨진 것이다.
- 리프트 뒤 저장한 `~/maps/g4_userloop_success_20260824T175517.{pgm,yaml}`에는 좌상단~중앙 방사형 아티팩트가 포함됐다. **이 파일은 항법용 최종 지도로 쓰지 않는다.** 긴 벽의 정상 정합 확인용 증거로만 보존한다.

결론: **G4 전환과 SLAM 시간축 수정은 1차 실주행까지 통과했다.** 기능상 해결로 판정하되 재현성 게이트는 동일 경로 2회 추가 주행으로 남긴다. 앞으로는 지도 저장 또는 SLAM 정지를 먼저 한 뒤 로봇을 들어 올린다. 지도가 직선인데 센서 데이터시트의 0° 공차만을 이유로 `laser_link` yaw를 추가 보정하지 않는다.

## 15:29 후속 조사 — 원인 수정 완료, 주행 검증 대기

### 확정 원인

1. **빔 공간 순서와 시간 순서가 반대였다.** LD14 SDK는 포인트를 실제 취득 timestamp 오름차순으로 넘긴다. 기존 publisher는 `laser_scan_dir=true`에서 range 배열만 뒤집고 `time_increment`는 양수로 유지했다. Cartographer는 `i * time_increment`로 각 빔 자세를 보간하므로, 회전 중 스캔을 실제와 반대 시간 방향으로 dewarp했다. 부챗살과 겹벽을 직접 만들 수 있는 `LaserScan` 계약 위반이다.
2. **publisher stall이 `scan_time`을 수 초로 늘렸다.** 기존 코드는 publish 호출 간격을 `scan_time`으로 사용하면서 header 보정값만 0.1667초로 제한했다. `carto_ported.log`에서 scan rate stall 직후 `Ignored subdivision` 16건과 `Dropped 256 earlier points` 1건이 실제로 확인됐다. 다음 스캔보다 미래 시각을 가진 빔이 생겨 Cartographer가 후속 데이터를 버린 것이다.
3. **라이다와 똑같은 6Hz 폴링이 프레임 회수 지연·누락을 만들었다.** sensor timestamp 적용 직후 정지 측정에서 잔여 지연 중앙값 149ms, 간헐적 331ms scan 간격이 나왔다. 폴링을 100Hz로 올린 뒤 잔여 지연 중앙값 9.6ms·p95 15ms, scan stamp 간격 165.1~167.3ms로 정상화됐다.

`/scan` header가 수신보다 약 140~170ms 과거인 것 자체는 결함이 아니다. 6Hz 한 바퀴의 **첫 광선 취득 시각**이므로 정상이다. 판정값은 `수신 시각 - header.stamp - scan_time`인 잔여 지연이다.

TB3 이식본·IMU본의 큰 순간 점프는 잘못된 backend loop constraint 채택이 로그로 확인됐고, odom-trust 본은 frontend가 먼저 틀어졌다. 이름과 달리 odom-trust 설정은 backend pose graph 가중치만 올렸고 frontend Ceres odom prior는 강화하지 않았다. 세 실패가 완전히 같은 경로는 아니지만, 위 시간 계약 위반은 모두의 공통 입력 결함이었다.

### 적용한 수정

- `ldlidar_sl_ros2`: 포인트의 실제 front/back timestamp로 header·`scan_time`·`time_increment` 생성, 50~500ms 범위를 벗어나면 회전 주기 fallback.
- `laser_scan_dir=true`: range를 취득 순서로 유지하고 ROS 좌표로 각도를 반전해 음수 `angle_increment` 사용. 0°/360° wrap도 시간 순서대로 인덱싱.
- 완성 프레임 폴링 6Hz → 100Hz.
- 회귀 테스트 4개 추가: publish stall, timestamp fallback, clockwise wrap, ROS 각도 반전.
- `jdamr_cube_2d_real.lua`: 8/18 성공본과 같은 `num_subdivisions_per_laser_scan=1`로 복구.
- `reset_map.sh`: 기본 설정을 실패한 corridor가 아니라 `jdamr_cube_2d_real.lua`로 변경. 다른 설정은 첫 인자로 명시할 때만 사용한다.

### 배포·정지 검증 증거

- 노트북과 Pi 모두 `ldlidar_sl_ros2` 빌드 성공, 핵심 gtest 4개 통과.
- 새 `/scan`: 음수 `angle_increment`, 빔 duration/scan_time 비율 1.0, scan 6.03Hz, 유효 빔 중앙값 약 88%.
- 기존/수정 후 10°별 거리 분포가 거의 같아 좌우 반전이나 공간 geometry 회귀 없음.
- 정확한 `_real` 설정으로 Cartographer 정지 smoke test: `Ignored subdivision=0`, earlier-point drop=0, FATAL=0, scan 6.03Hz, odom 50Hz.
- 순수 입력 bag: `~/bags/slam_timingfix_stationary_clean_20260824T1528` (Cartographer·IMU 제외), `cartographer_rosbag_validate` 오류 0건. 첫 광선 시각 때문에 serialization time과 약 172ms 차이 난다는 예상 경고 1건만 존재.
- 원격 원본 백업: `~/jdamr_backups/ldlidar-timing-20260824T1515KST`.

### 남은 최종 게이트

로봇 이동은 건별 승인이 필요하다. 승인 후 Cartographer를 끈 상태에서 raw bag(`/scan /odom /tf /tf_static /cmd_vel /joint_states`)을 녹화하며 동일 P자 폐루프를 수행하고, 같은 bag을 정확한 8/18 설정으로 replay한다. 3회 모두 odom 폐루프 ≤0.30m, SLAM 폐루프 ≤0.10m, yaw ≤3°, 겹벽 없음, ignored/dropped scan 0건이면 해결 완료로 판정한다. 이 게이트 전에는 예비 G4로 교체하지 않는다.

### 15:36 사용자 수동 주행 1차 결과

사용자가 웹 조종으로 직접 주행했다. 이 주행은 녹화 시작 전에 진행돼 raw bag은 없지만, live Cartographer 로그·TF·최적화 전후 지도 snapshot을 보존했다.

- 최적화 정착 후 폐루프 위치: SLAM 0.128m, odom 0.314m. 이전 SLAM 1.4~1.7m 실패 대비 약 91% 감소했다.
- 종료 pose: map yaw -127.0°, odom yaw +171.0°, map→odom yaw 보정 +62.1°. 사용자가 출발 때와 같은 방향으로 끝냈는지 확인 전이라 yaw 폐루프 판정에서는 제외한다.
- 시간 오류: `Ignored subdivision=0`, earlier-point drop=0, FATAL=0. scan 6.03Hz·odom 50Hz 유지.
- 후단 제약: 최대 translation 0.51m/rotation 0.062rad. 이후 현재 submap 제약은 translation 0.02m, score 82~85%로 안정됐다. 과거 실패 때의 1.5m급 제약은 없다.
- 정착 지도: `~/maps/timingfix_userloop_settled_20260824T154119.{pgm,yaml}`. 주요 벽은 하나의 긴 직선과 코너로 연결되고 굵은 평행 이중벽은 사라졌다.
- 이미지 정량 비교(현재 vs 8/18 `room_nomirror`): 최대 연결 occupied component 39.2% vs 34.8%, 5픽셀 이하 점 노이즈 9.3% vs 10.2%, 최장 Hough 선분 84.9px vs 84.1px. 성공본과 동급이다.

따라서 **1.5m 점프와 이중벽을 만들던 치명적 SLAM 입력 결함은 해결된 것으로 판정한다.** 지도 하단의 들쭉날쭉한 회색/흰색 경계는 벽이 아니라 관측/미관측 frontier이고, 검은 점·짧은 선 노이즈는 일부 남는다. 재현성까지 엄격히 확인하려면 다음 주행부터 raw bag을 먼저 시작하고 동일 경로를 2회 더 반복한다.

## 접속·운영

- **로봇 접속**: `jdamr.local` (mDNS). **IP가 DHCP로 계속 바뀐다**(오늘 .159↔.160 반복). IP 직접 쓰지 말고 `jdamr.local` 사용. 안 풀리면 `getent hosts jdamr.local`.
- **DDS**: `ROS_DOMAIN_ID=12`, 노트북은 `ROS_STATIC_PEERS=<로봇IP>` 또는 `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` 필요(강의실 AP 멀티캐스트 안 흘림).
- **파이 DDS 반복 사망**: 서비스 재기동·`ros2 daemon stop`·CLI 조회를 반복하면 FastDDS 공유메모리가 오염돼 파이 내부 토픽이 통째로 0이 된다(scan/odom/battery 다 끊김). 복구: 전체 정지 → `sudo rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* /dev/shm/sem.*` → 서비스 재기동. **확인은 CLI 말고 단일 python 노드로**(CLI가 상태를 더 흔든다). 근본 해결은 FastDDS를 UDP 전송으로 바꾸는 것(미적용).
- **webteleop 부팅 미기동 버그**: systemd ordering cycle 때문에 부팅 시 `jdamr-webteleop`이 자동으로 안 뜬다. 매 부팅 후 `sudo systemctl start jdamr-webteleop` 수동 실행 필요. **영구 수정 미적용**(유닛 파일 `After=` 의존 정리 필요).
- **웹 조종**: `http://jdamr.local:8080` (WASD/터치). 브라우저 페이지가 낡으면(IP 바뀐 뒤) 버튼이 죽은 옛 주소로 POST해 안 먹는다 → **하드리프레시(Ctrl+Shift+R)**.
- **지도 초기화 스크립트**: 파이에 `~/reset_map.sh` (현재 지도 저장 → 8/18 성공 설정으로 카토그래퍼 재시작). `ssh lim@jdamr.local '~/reset_map.sh'`. 다른 설정은 파일명을 첫 인자로 명시한다.

## 하드웨어 상태 (전부 정상 확인)

- **배터리 3S2P** — 완충 12.6V. 오늘 9.81V로 방전됐다가(주행 불가·파이 브라운아웃의 원인) 충전기로 충전. 저전압이면 서보가 명령 무시하고 파이가 네트워크에서 사라진다. 충전은 **로봇 OFF**로, 12V 어댑터 금지(3S 리튬이온 전용 충전기 사용).
- **서보 L·R 정상**, INA219(전압계) 정상.
- **IMU**: 칩 살아있음(WHO_AM_I 0x05). **자이로 정상**(720° 회전 1.3% 오차). **가속도계 이상**(정지 |a|=19.28, 정상 9.81의 ~2배 + 방향 이상). 원인은 [[jdamr-oled-kills-i2c-bus]] — **불량 OLED가 I2C 버스를 끌어내려 IMU·INA219까지 죽였다. OLED를 뽑아야 IMU가 산다. 다시 꽂지 말 것.**
- **주행 정상** — 직접 cmd_vel로 바퀴 돈다(12cm 실측). base·서보·배터리 다 정상.
- **오도메트리 훌륭** — 폐루프 위치 오차 0.2~0.3m. 회전 환산 1.3%.

## 최초 핵심 문제 — SLAM 정합 실패 (후속 원인 수정 전 기록)

증상: P자로 돌아 **제자리 복귀했는데 SLAM은 1.4~1.7m 떨어진 곳을 가리킨다.** 지도는 방사형 가시 + 벽이 두 겹으로 그려짐(스캔매칭이 같은 벽을 두 곳에).

**일관된 4분면 판정 (3회 반복)**:
| 실험 | odom 폐루프 | SLAM 폐루프 | 판정 |
|---|---|---|---|
| ①이식 TB3 설정(상관on·1e3) | 0.12m | 1.73m | SLAM이 좋은 odom을 뒤엎음 |
| ②IMU on | 0.22m | 1.50m | 차이 없음 |
| ③odom-trust(상관off·1e5) | 0.33m | 1.40m | 여전히 실패 |

**핵심**: odom은 매번 0.2~0.3m로 훌륭한데 스캔매칭이 1.5m를 튄다. 세 설정 다 못 고쳤다.

## 최초 세션에서 시도한 것과 결과 (전부 실패)

1. **터틀봇 성공 레시피 이식**(`carto_lds01.lua` → `jdamr_cube_2d_corridor.lua`): 상관매칭 on, 균형 가중치 1e3/1e3, IMU off. → 실패(1.7m). **잘못된 판단**: TB3는 odom이 나빠서 공격적 스캔매칭이 필요했지만, JD-AMR은 odom이 좋아서 그게 독이 됐다.
2. **IMU 켜기**: base_link→imu_link identity static TF + `use_imu_data=true` + `imu:=/imu/data_raw` 리맵. → 도움 안 됨(1.5m). 가속도계가 이상해 중력 정렬이 안 맞는 게 원인일 수 있음(자이로만 쓰게 하는 설정 미시도).
3. **오도메트리 신뢰**: 상관매칭 off, odometry_translation_weight 1e5, rotation 1e4, IMU off. → 여전히 실패(1.4m). 현재 소스·파이 설정이 이 상태.

## 최초 세션 가설 (후속 조사 전 기록)

1. **스캔 타임스탬프 지연 재확인** — 이 세션에서 `/scan` stamp가 수신보다 +140ms 과거로 관측됨. 스캔-odom 시간 정렬이 틀어지면 어떤 가중치로도 못 고친다. 카토그래퍼가 스캔을 잘못된 과거 pose에 놓는지 확인. **가장 유력.**
2. **8/18 강의실 성공본(_real, 5.49%)으로 되돌려 baseline 재현** — 같은 로봇에서 됐던 설정이다. 지금 방에서 `_real` 그대로 돌려 되는지부터 확인(환경 문제 vs 설정 문제 분리). 이번 세션은 전부 `_corridor`만 건드렸다.
3. **ceres_scan_matcher 가중치까지 올리기** — POSE_GRAPH(전역) odom 가중치만 올렸지 로컬 ceres(translation/rotation_weight=10)는 안 건드렸다. 로컬 스캔매칭이 per-scan으로 드리프트하는 걸 막으려면 여기도 올려야 할 수 있다.
4. **motion skew** — 6Hz 라이다 + `num_subdivisions=1`. 회전 구간 왜곡. subdivisions 5~10으로.
5. **rosbag 녹화 후 오프라인 디버깅** — 라이브 튜닝이 3번 실패했으니, `/scan /odom /tf /imu`를 bag으로 떠서 오프라인에서 여러 설정을 재현 가능하게 돌리는 게 효율적. 라이브 튜닝 그만.

## 파일 위치

- 카토그래퍼 설정: `jdamr_cube_cartographer/config/jdamr_cube_2d_corridor.lua`(실패한 odom-trust 보존본), `_real.lua`(8/18 강의실 성공본과 같이 subdivisions=1, 현재 기본값)
- 폐루프 측정 도구: 파이 `/tmp/pose.py` (map/odom→base_footprint 비교)
- 프레임 직접 읽기: 파이 `/tmp/readframe3.py` (ESP32 배터리·서보 플래그, ROS 우회)
- 지난 이론·절차: `~/physical-ai-lab/learning/M02_linux_ros2/slam_nav2/JDAMR_NEXT_SESSION.md`
- 결함 원장: `reference/JDAMR_Cube_조사기록.md`

## 라이다 교체 관련 (G4 장착·정지 검증 완료)

LD14도 수정 후 광학·지도 성능이 정상이었으므로 고장 때문에 교체한 것은 아니다. 사용자가 판 중앙에 장착한 G4는 9.68Hz·중앙값 929 beams로 LD14의 6.03Hz·약 666 beams보다 시간·각도 해상도가 높고, 유효 비율은 낮지만 절대 유효 빔 수는 비슷하거나 조금 많다. 수동 주행에서도 긴 벽이 단일 선으로 정합됐으므로 **성능 우선 선택은 현재 G4로 확정한다.**

## 자율 매핑 안전 운용 (2026-08-24)

- `autonomous_mapping.launch.py`는 실기 기본값 `use_sim_time=false`이며 Nav2 lifecycle `autostart=true`여도 `frontier_explorer`는 `IDLE`로 시작하고 자동 출발하지 않는다.
- 안정 운용 API는 `/autonomy/start`, `/autonomy/pause`, `/autonomy/resume`, `/autonomy/stop`, `/autonomy/save_map`이다. 기존 `/frontier_explorer/start`, `/frontier_explorer/pause`, `/frontier_explorer/resume`, `/frontier_explorer/stop`은 호환 alias로 유지한다.
- 시작·재개 전 로봇이 완전히 정지해 있어야 하며 map/scan/TF, 0.5초 이내 `/odom`, `/battery_state` 최신성, Nav2 planner/navigator, 1초 이내 Collision Monitor lifecycle `ACTIVE`, `/cmd_vel`의 정확히 하나인 root `/collision_monitor` 소유가 모두 확인돼야 한다. 주행 중에는 정지 조건만 제외하며 odom 최신성은 계속 필수다. 조건이 깨지면 fail-closed로 `PAUSED`가 되고 자동 재개하지 않는다.
- 자율 매핑 전 `jdamr-webteleop`을 중지한다. webteleop도 `/cmd_vel`을 직접 발행하므로 함께 실행하면 명령 소유 충돌이며 explorer가 시작을 거부한다.
- 설정은 후진 없이 저속으로 주행하고, known space 안에서만 경로를 만들며, Regulated Pure Pursuit 뒤에 STOP/SLOWDOWN/APPROACH Collision Monitor를 최종 속도 발행자로 둔다.
- `/battery_state`가 기본 2.5초보다 오래됐거나 전압이 기본 10.5V 미만이면 `battery_ok=False`로 시작·재개와 계속 주행을 자동 차단한다. 작업자의 3S 배터리 사전 확인과 물리 전원 접근도 계속 필수다.
- G4는 높이 약 0.1m의 한 평면만 측정한다. 투명·검은·반사 물체, 라이다보다 낮거나 높은 물체, 사람을 항상 검출한다고 가정하지 않는다. Collision Monitor는 안전 인증 장치가 아니므로 사람이 계속 감시한다.
- `/autonomy/pause`는 저장 없이 안전정지만 요청한다. `/autonomy/stop`은 navigation terminal과 최신 map/odom 기반 정지를 확인한 뒤 자동 저장하며, 대기 중 `save=waiting_for_stop`이다. 수동 중간 저장은 pause 후 `motion_transition=clear`와 정지를 확인하고 `/autonomy/save_map`을 호출한다. transition pending 동안 저장하거나 로봇을 들지 않는다. `success=true`는 비동기 저장 요청 접수만 뜻한다. `/frontier_explorer/status`의 `save=in_progress`가 `succeeded`로 바뀐 뒤 파일을 확인한다(`rejected`는 미접수, `failed`는 디렉터리 준비 실패 또는 접수 후 저장 실패).
- 상태 토픽은 `state`, `fault`, `goal`, `battery_voltage`, `battery_ok`, `stationary`, `readiness_missing`, `collision_lifecycle_error`, `save`, `motion_transition`, `frontiers`, `map_sequence`를 제공한다. `collision_lifecycle_error`는 최근 Collision Monitor lifecycle 서비스 조회 예외를 `<ExceptionType>: <message>`로 표시하고, 정상 응답을 받으면 `none`으로 복구한다. 이 값이 `none`이 아니면 fail-closed로 운행을 차단한다. 완료 자동 저장 파일은 기본적으로 `/home/lim/maps/autonomous_YYYYmmddTHHMMSS.{yaml,pgm}`이다.
- explorer crash, Nav2 목표 전송·취소 1초 timeout, 취소 거부 시 `autonomous_mapping.launch.py`가 전체 Nav2 mapping launch를 종료한다. 이 안전 경계를 우회하는 explorer 단독 실행은 금지한다.
- `.yaml`과 `.pgm`이 모두 존재하고 YAML의 `image:`가 실제 PGM을 가리키는지 확인한다. **`save=succeeded`와 탐색 중단을 확인하기 전에는 로봇을 들거나 라이다 위치를 바꾸지 않는다.**
- 상세 실행·저장 명령은 `jdamr_cube_navigation/README_JDAMR.md`에 있다.
