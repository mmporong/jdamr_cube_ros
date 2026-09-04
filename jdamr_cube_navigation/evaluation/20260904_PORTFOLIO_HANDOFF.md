# SLAM 포트폴리오 고도화 인계

## 공개 서사의 중심

JD-AMR은 저장 지도와 Keepout을 사용해 복도 왕복 경로의 20개 목표를 Nav2 recovery 없이
완주했다. 제어 경로를 온보드 DDS로 격리한 뒤 LiDAR 최대 공백은 10.750초에서
0.112초로 줄었다. 성공 주행 MCAP을 두 SLAM 백엔드에 같은 조건으로 다시 재생해 복도
환경에 맞는 백엔드도 선택했다.

공개 페이지는 다음 순서로 쓴다.

1. 약한 무선 구간에서 센서와 TF가 함께 정체된 문제
2. 온보드 제어 그래프와 원격 시각화 그래프의 분리
3. 20/20 waypoint, recovery 0회, AMCL 경로 77.090m의 완주 결과
4. 동일 MCAP 오프라인 재생으로 Cartographer와 SLAM Toolbox 비교
5. 실측 센서·시간 분포를 다음 시뮬레이션 실험의 입력으로 연결

## 공개에 쓸 수치

| 항목 | 확인된 값 | 표현 범위 |
|---|---:|---|
| Nav2 waypoint | 20/20 | 단일 완주 기록 |
| Nav2 recovery | 0회 | 해당 주행 구간 |
| 사전 계획 경로 | 77.278m | Nav2 planning 결과 |
| 기록된 AMCL 경로 | 77.090m | 저장 지도 localization 기준 |
| 시작점 복귀 | 0.113m | AMCL 시작·종료 위치 차이 |
| LiDAR 최대 공백 | 10.750초에서 0.112초 | DDS 격리 전후 동일 계산식 |
| Cartographer 시작–종료 | 0.996m | 전체 MCAP 오프라인 재생 |
| SLAM Toolbox 시작–종료 | 9.139m | 전체 MCAP 오프라인 재생 |
| Cartographer AMCL 기준 정렬 RMS | 0.552m | ground truth가 아닌 일관성 지표 |
| SLAM Toolbox AMCL 기준 정렬 RMS | 1.944m | ground truth가 아닌 일관성 지표 |
| LiDAR 주기 jitter p99 | 0.002164초 | 성공 주행 구간 header timestamp |
| 정적 IMU z축 robust sigma | 0.0018113rad/s | 주행 직전 60초 구간 |

Cartographer는 시작–종료 불일치가 9.18배 작고 AMCL 기준 정렬 RMS가 3.52배 작았다.
두 기준이 같은 백엔드를 가리켜 현재 복도 데이터의 기본 mapping backend로 선택했다.

## 공개 미디어

- `media/corridor_localdds_armed_20260904T152036/success_card.png`
- `media/corridor_localdds_armed_20260904T152036/route_evidence.png`
- `media/corridor_localdds_armed_20260904T152036/continuity_comparison.png`
- `media/corridor_localdds_armed_20260904T152036/corridor_roundtrip.mp4`
- `media/corridor_localdds_armed_20260904T152036/slam_backend_comparison.png`
- `media/corridor_localdds_armed_20260904T152036/sensor_profile.png`
- `media/corridor_localdds_armed_20260904T152036/timing_profile.png`

## 공개 문구에서 제외할 내용

내부 디버깅에 필요했던 프로세스 ID, 중간 실행 횟수, 종료 신호 수정 과정, 과거 계측기
오분류는 공개 본문에서 뺀다. SLAM Toolbox의 가변 scan 길이 경고도 횟수를 나열하지
않고, 실제 드라이버 입력 변동을 포함한 동일 데이터 비교를 수행했다는 실험 조건으로만
설명한다.

한계는 짧고 정확하게 남긴다. AMCL은 외부 ground truth가 아니므로 ATE/RPE나 절대
정확도를 주장하지 않는다. 한 번의 완주로 반복 성공률을 만들지 않으며, 카메라 데이터가
없는 현재 결과를 Visual SLAM으로 부르지 않는다.

## 근거 파일

- `20260904_CORRIDOR_LOCALDDS_SUCCESS.md`
- `media/corridor_localdds_armed_20260904T152036/metrics.yaml`
- `media/corridor_localdds_armed_20260904T152036/sensor_profile.md`
- `media/corridor_localdds_armed_20260904T152036/slam_backend_comparison.md`
- `media/corridor_localdds_armed_20260904T152036/media_manifest.yaml`

포트폴리오 세션은 이 문서의 공개 수치와 미디어만 먼저 사용한다. 자세한 디버깅 기록은
면접에서 원인 분석 과정을 질문받았을 때 근거로 연다.
