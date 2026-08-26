# Physical AI 포트폴리오 적용안

- 기준일: 2026-08-26
- 상태: 구현 완료 항목과 다음 실험 추천을 분리한 저장소 안내

## 현재 구현의 위치

이 포크는 JD-AMR cube 기반 위에 `capstone_pick` 패키지를 추가해 비전 검출, 접근, 손목 카메라 정렬, 파지, 운반을 하나의 ROS 2 작업으로 연결합니다. Nav2·SLAM·기체 원본은 기반 저장소의 기여이고, 픽앤플레이스와 검증 도구는 이 포크의 기여입니다.

포트폴리오에서는 이 저장소를 실행 계층으로 설명합니다. Gazebo의 물리 정답과 구현 기록은 [gazebo-so101-capstone](https://github.com/mmporong/gazebo-so101-capstone), 실행 기록의 재판정은 [robot-dashboard](https://github.com/mmporong/robot-dashboard)에서 보여줍니다.

## Physical AI로 고도화하는 경계

학습 모델이 들어가도 ROS 2 실행기와 안전 계층은 분리합니다.

```text
자연어·비전 입력
      ↓
typed task contract 또는 행동 정책
      ↓
ROS 2 상태기계·BT·제어기
      ↓
FK·관절 제한·충돌·비상 정지
      ↓
실물 또는 Gazebo
      ↓
MCAP 기록과 물리 결과 재판정
```

AI를 적용하기 좋은 곳은 세 군데입니다.

- 카메라·관절 상태에서 다음 행동 청크를 만드는 정책
- 규칙이 놓친 실패를 찾는 시계열 성공 판정기
- 자연어 지시를 제한된 Task Contract나 BT 매개변수로 바꾸는 상위 해석기

Nav2, 좌표 변환, FK, 충돌 제한, 비상 정지, 물리 정답은 학습 모델 밖에 둡니다.

## sim-to-real 추천 순서

1. 실물과 Gazebo에 같은 안전 궤적을 보내 관절 영점, 지연, 정착 시간, 방향 전환 dead zone을 측정합니다.
2. 링크·카메라 좌표와 hand-eye를 맞춘 뒤 말단 위치 오차를 구합니다.
3. 실물 측정 범위를 바탕으로 마찰, 질량, 지연, 카메라 위치를 randomization합니다.
4. Gazebo·실물 실행을 같은 MCAP 스키마로 기록합니다.
5. 실물 ID·OOD 조건에서 zero-shot과 소량 보정 결과를 분리해 보고합니다.

## 논문별 적용 추천

| 논문 | 이 저장소에서의 적용 방법 | 추천 |
| --- | --- | --- |
| [Domain Randomization](https://arxiv.org/abs/1703.06907) | YOLO 입력의 조명·재질·배경·카메라 자세를 바꾸고, 실제 카메라 조건을 별도 평가셋으로 둡니다. | 비전 OOD에 추천 |
| [Dynamics Randomization](https://arxiv.org/abs/1710.06537) | 주행·팔의 지연, 마찰, 질량, 모터 응답을 측정 범위 안에서 흔듭니다. | 실물 동정 뒤 추천 |
| [Closing the Sim-to-Real Loop](https://arxiv.org/abs/1810.05687) | 실물 rollout과 Gazebo 행동 차이로 randomization 분포를 반복 갱신합니다. 변경 전후 오차를 MCAP으로 남깁니다. | 핵심 방법으로 추천 |
| [ACT](https://arxiv.org/abs/2304.13705) | 정밀 접촉 전후를 짧은 단일 행동보다 action chunk로 예측합니다. 기존 규칙 기반 파이프라인은 안전한 시연자와 비교 기준선으로 유지합니다. | SO-101 정책 기준선으로 추천 |
| [Diffusion Policy](https://arxiv.org/abs/2303.04137) | 접근·놓기 궤적이 여러 모드로 나타날 때 ACT와 같은 데이터로 비교합니다. 반복 추론 지연도 제어 주기와 함께 공개합니다. | 데이터가 다봉일 때 추천 |
| [RMA](https://arxiv.org/abs/2107.04034) | 관절·행동 이력으로 부하와 마찰을 추정하는 adaptation module 개념만 검토합니다. | 기준선 이후 후순위 |
| [See like a Robot](https://arxiv.org/abs/2607.11498) | 손목·전면 카메라 RGB를 베이스 좌표 pointmap과 비교해 카메라 이동에 대한 일반화를 측정합니다. | depth·hand-eye 검증 뒤 추천 |

트랜스포머는 행동 청크나 이력 추론에 쓸 수 있지만, 모델을 넣었다는 사실만으로 Physical AI가 되지는 않습니다. 실제 관측, 행동, 접촉 결과, 지연, 안전 개입이 하나의 폐루프로 측정돼야 합니다.

## 이 저장소에서 남길 포트폴리오 증거

- 본인 기여와 기반 저장소 기여를 나눈 파일 표
- 실행 단계와 실패 복구가 보이는 시스템 다이어그램
- Gazebo·실물의 같은 Task Contract와 MCAP 스키마
- 보정 전후 관절·말단·접촉 오차
- ID·OOD 성공률, 거짓 성공, 안전 개입, 추론 지연
- 재현 명령과 commit·환경 버전

계획 항목은 완료 수치처럼 쓰지 않습니다. 실물 평가 전에는 `sim-to-real readiness` 또는 `적용 추천`으로 표시합니다.
