# SLAM Toolbox 복도 지도 왜곡 원인분리

## 결론

SLAM Toolbox 자체의 단순 실패가 아니었다. LD14 드라이버가 프레임마다 다른 빔 수와
각도 격자를 내보낸 상태에서 긴 반복 복도를 처리한 것이 첫 번째 원인이었고, 먼 거리
빔과 성긴 키프레임 간격이 문제를 키웠다. 입력을 고정 격자로 정규화하고 사용 거리를
제한한 뒤 키프레임을 촘촘하게 만들자, 시작–종료 불일치는 9.139m에서 1.032m로 줄고
겹쳐진 복도도 단일 구조로 회복됐다.

같은 정규화 입력에서 Cartographer와 튜닝한 SLAM Toolbox의 시작–종료 불일치는 각각
1.026m와 1.032m로 수치상 유사했다. AMCL 기준 정렬 RMS는 0.553m와 0.817m였으므로
현재 복도용 기본 백엔드는 Cartographer로 유지한다.

## 확인된 입력 계약 문제

원본 MCAP의 `/scan` 7,304개를 전수 확인했다.

| 항목 | 확인값 |
|---|---:|
| 빔 수 범위 | 905–953개 |
| 최빈 빔 수 | 931개, 3,262 frame |
| 최빈값과 다른 frame | 4,042개, 55.34% |
| 서로 다른 빔 수 | 24종 |
| 원본 SLAM Toolbox expected-count 경고 | 77회 |

SLAM Toolbox가 사용하는 Karto 레이저 모델은 처음 등록한 각도·해상도에서 기대 빔 수를
계산하고, 이후 스캔 길이가 다르면 해당 검증을 통과시키지 않는다. 원본 로그의
`contains ... range readings, expected 931`은 이 계약 불일치를 직접 보여준다.

정규화기는 각 빔의 실제 각도를 먼저 계산한 뒤 `[-π, π)`의 고정 931-bin 격자에 최근접
각도 방식으로 재배치한다. 보간으로 새 거리값을 만들지 않으며 `NaN`과 무한대도 그대로
보존한다. 생성된 bag은 다음 전수 검사를 통과했다.

- 전체 130,798개 메시지의 순서와 수신 timestamp 일치
- `/scan` 이외 payload 바이트 일치
- `/scan` 7,304개가 모두 정규화 함수의 예상 출력과 일치
- 모든 결과가 같은 원본에서 파생됐음을 SHA-256으로 기록

## 원인분리 실험

모든 정규화 실험은 같은 파생 MCAP을 domain 199에서 1배속으로 재생했다. 저장 지도,
이동 명령, 기록된 AMCL `map→odom`은 입력에서 제외했고 각 실행의 backend 하나만 새
`map→odom`을 발행했다. 다섯 실행 모두 전체 재생, 지도 저장, 결과 MCAP 마감을
완료했다.

| 조건 | 시작–종료 | AMCL 정렬 RMS | 지도 범위 |
|---|---:|---:|---:|
| SLAM Toolbox, 고정 scan | 3.002m | 1.152m | 47.35×20.2m |
| + 최대 거리 3.5m | 2.764m | 0.963m | 44.25×9.65m |
| + 이동 0.1m·회전 0.15rad 키프레임 | 1.032m | 0.817m | 44.75×10.3m |
| + 루프 클로징 비활성 | 2.774m | 1.008m | 45.35×10.1m |
| Cartographer, 같은 고정 scan | 1.026m | 0.553m | 46.35×12.15m |

### 해석

1. 고정 scan만 적용해도 원본 대비 시작–종료와 RMS가 크게 줄었다. 센서 메시지 계약
   불일치가 실제 원인 중 하나였다는 근거다.
2. 최대 거리를 3.5m로 줄이자 지도 세로 범위가 20.2m에서 9.65m로 줄고 주 복도 형상이
   회복됐다. 반복 복도에서 먼 반사점을 더 쓰는 것이 항상 유리하지 않았다.
3. 키프레임 간격을 촘촘하게 하자 좌측 이중 복도가 사라지고 폐루프 일관성이
   Cartographer 수준까지 회복됐다.
4. 같은 조건에서 루프 클로징을 끄자 두 지표가 다시 나빠졌다. 루프 클로징 자체가
   왜곡 원인이 아니라, 정상 입력과 충분한 그래프 밀도가 있을 때 오히려 복귀 오차를
   줄였다.

## 선택과 한계

- SLAM Toolbox는 입력 정규화와 환경별 설정을 적용하면 사용 가능한 지도를 만들었다.
- Cartographer는 같은 입력에서 폐루프 수치는 거의 같았고 AMCL 기준 전체 궤적
  일관성이 더 좋아 기본 백엔드로 유지한다.
- AMCL은 저장 지도 localization reference이며 외부 ground truth가 아니다. 이 결과를
  ATE/RPE나 절대 정확도로 부르지 않는다.
- 한 bag으로 설정을 고르고 같은 bag으로 평가했으므로 과적합 가능성이 있다. 새 실차
  반복은 성공률을 만들 때 수행하고, 다음 단계에서는 시뮬레이션 ground truth로
  ATE/RPE를 측정한다.
- 운영자가 제공한 실제 층 평면도는 축척과 SLAM 좌표계 변환이 없어 형상 참고에만
  사용했고 정량 계산에서는 제외했다.
- 최근접 정규화는 개별 빔의 취득 위상을 복원하지 않는다. header timestamp는 보존했다.

## 재현 파일

- 정규화: `evaluation/normalize_scan_bag.py`
- 설정 파생: `evaluation/make_slam_toolbox_ablation.py`
- 격리 재생: `scripts/offline_slam_replay.sh`
- 정량 비교: `evaluation/compare_slam_runs.py`
- 전체 결과: `media/corridor_localdds_armed_20260904T152036/slam_toolbox_ablation.json`
- 비교 그림: `media/corridor_localdds_armed_20260904T152036/slam_toolbox_ablation.png`
- 비교 요약: `media/corridor_localdds_armed_20260904T152036/slam_toolbox_ablation.md`

SLAM Toolbox 파라미터 탐색은 여기서 종료했다. 후속 Gazebo 실험에서 같은 결론을
독립 ground truth로 확인했다. 기준 조건 이동 ATE RMS는 Cartographer 0.645m,
SLAM Toolbox 4.355m였고, LiDAR 가우시안 표준편차를 0.01m에서 0.05m로 높인
stress 조건에서는 각각 0.752m와 3.990m였다. 상세 결과와 입력 해시는
`media/sim_slam_corridor_gt_20260904/sim_slam_robustness.md`에 있다.
