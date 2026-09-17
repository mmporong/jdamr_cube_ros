# 식당 서비스 목적지 등록 후속 작업

## 상태

- 상태: `DEFERRED`
- 현재 우선순위: RGB-D Visual SLAM 검증 이후
- 구현 브랜치 후보: `feat/restaurant-service-destinations`
- 현재 주행 동작에는 연결하지 않는다.

## 목표

테이블 중심점이 아니라 로봇이 실제로 정차해야 하는 **서비스 pose**를 저장 지도 `map`
좌표계에 등록한다. Nav2는 이 pose까지 이동하고, 이후 양팔 작업이 필요하면 Depth 카메라가
상판과 놓을 공간을 국소적으로 다시 확인한다.

태그, 반사판, 테이블 다리 기하 추정이나 5cm 밀착 주차는 이 작업의 필수 조건이 아니다.
고정된 식당 테이블은 초기 설치 때 서비스 pose를 교시하고 반복 사용한다.

## 데이터 모델

목적지 파일은 지도와 함께 버전 관리하며 다음 필드를 가진다.

```yaml
map:
  yaml: autonomous_20260826T161908.yaml
  map_sha256: TO_BE_FILLED
  keepout_mask: autonomous_20260826T161908_keepout_multi.yaml
  keepout_sha256: TO_BE_FILLED

tables:
  - table_id: table_01
    enabled: true
    service_poses:
      - id: table_01_main
        side: aisle
        x_m: TO_BE_TAUGHT
        y_m: TO_BE_TAUGHT
        yaw_rad: TO_BE_TAUGHT
        xy_tolerance_m: 0.05
        yaw_tolerance_rad: 0.087266
        priority: 1
        manipulation_side: front
      - id: table_01_alternate
        side: aisle
        x_m: TO_BE_TAUGHT
        y_m: TO_BE_TAUGHT
        yaw_rad: TO_BE_TAUGHT
        xy_tolerance_m: 0.08
        yaw_tolerance_rad: 0.139626
        priority: 2
        manipulation_side: front
```

`x_m`, `y_m`, `yaw_rad`는 테이블 중심이 아니라 정차한 `base_link`의 목표 자세다. 허용
오차는 최초값일 뿐이며 반복 도착 실측으로 조정한다. 지도 또는 Keepout 마스크가 바뀌면
해시 불일치로 기존 목적지를 그대로 쓰지 못하게 한다.

## 교시 절차

1. 운영에 사용할 저장 지도와 Keepout 마스크를 띄운다.
2. 로봇을 테이블의 실제 서비스 위치와 방향에 수동으로 맞춘다.
3. 안정화된 `/amcl_pose`의 위치와 quaternion을 읽어 yaw로 변환한다.
4. 같은 자세에서 전역·지역 costmap의 충돌 여부와 팔 작업 공간을 확인한다.
5. 주 접근점과 통로가 막혔을 때의 대체 접근점을 각각 저장한다.
6. 같은 시작점에서 반복 도착해 위치·방향 오차와 Nav2 결과를 기록한다.

좌표는 RViz 화면을 보고 추정해 입력하지 않는다. 실제 기체를 세운 뒤 AMCL pose를 캡처한
값과 지도·마스크 해시를 함께 저장한다.

## 후속 구현 작업

1. `restaurant_service_destinations.yaml` 스키마와 샘플을 추가한다.
2. 현재 `/amcl_pose`를 이름 있는 서비스 pose로 저장하는 교시 도구를 만든다.
3. 지도·Keepout 해시, 중복 ID, 좌표 유한성, tolerance 범위를 검사하는 validator를 만든다.
4. `table_id`를 받아 가능한 pose를 우선순위와 Nav2 계획 가능 여부로 선택하는 destination
   manager를 만든다.
5. 주 pose가 막히면 alternate pose로 한 번만 전환하고, 둘 다 실패하면 해당 테이블 작업을
   중단한다.
6. 도착 후에는 Nav2 결과와 AMCL 최종 오차를 기록하고, 양팔 작업이 있으면 Depth 국소 인지로
   상판과 빈 공간을 확인한다.
7. 고정 지도 단위 테스트와 bag replay 평가를 추가한 뒤 실차 반복 도착으로 검증한다.

## 완료 기준

- 테이블 ID만으로 지도에 종속된 서비스 pose를 선택할 수 있다.
- 지도 또는 Keepout 마스크가 바뀌면 오래된 목적지가 거부된다.
- 주 pose가 막힌 경우에만 대체 pose를 사용한다.
- Nav2 도착 여부와 별도로 최종 위치·방향 오차가 기록된다.
- 5cm 정확도는 반복 실측 결과가 있을 때만 주장한다.
- Depth 국소 인지와 베이스 목적지 등록의 책임이 분리돼 있다.

