# RGB-D Visual SLAM 휠 오도메트리 보조 A/B

## 결론

같은 P턴 bag에서 카메라 단독 저텍스처 설정과 휠 오도메트리 보조 통합 설정을 비교했다.
휠 보조는 registration failure를 32회에서 9회로 줄였고 자동 odometry reset 2회를
제거했다. 휠 odometry 기준 SE(2) 정렬 ATE RMSE는 0.7346m에서 0.2070m로 71.8%
감소했다.

그러나 이 기록은 출발점으로 돌아오는 폐루프 주행이 아니며 accepted loop closure가 없다.
내보낸 점군도 장면 전체가 하나의 연속된 구조로 정합됐다고 보기 어렵다. 따라서 이번 결과는
휠 보조 Visual SLAM의 강건성 개선 증거이며, 완성된 3D 지도나 절대 정확도 증거로 사용하지
않는다.

## 외부 파라미터와 TF 구조

현재 임시 Astra S 장착은 다음 값으로 기록했다.

| 항목 | 값 | 근거 |
| --- | ---: | --- |
| `base_link → camera_link` x | 0.065m | 바퀴축에서 차체 전면까지 기존 실측, 카메라 렌즈면 전면 정렬 |
| y | 0.000m | 차체 중앙 장착 확인 |
| z | 0.215m | 바닥에서 렌즈 중심까지 줄자 실측 |
| roll / pitch / yaw | 0 / 0 / 0rad | 정면 확인, 수평 브래킷 기준 명목값이며 경사계 캘리브레이션은 아님 |

휠 odometry guess frame은 `odom`, visual odometry 출력 frame은 `vslam_odom`으로
분리했다. 최종 TF 흐름은 다음과 같다.

```text
vslam_odom → odom → base_footprint → base_link → camera_link
```

초기 실험에서 `guess_frame_id=base_footprint`를 사용하자 RTAB-Map 보정 TF가 bag의
`odom → base_footprint`와 같은 자식 frame을 다시 발행했다. 이 구성은 TF 충돌로
판정해 폐기했다. 공식 RTAB-Map의 guess 동작처럼 입력 odometry frame과 visual odometry
출력 frame을 분리한 뒤 다시 처리했다.

## 동일 bag 비교

입력은 `$HOME/jdamr_data/vslam/rgbd_20260916T172638/bag_paired_10fps`이며 RGB-D
1,482쌍, `/odom`, `/scan`, TF를 포함한다. null odometry가 실제 원점 pose로 계산되지
않도록 quaternion norm이 0인 행은 궤적 평가에서 제외했다.

| 지표 | camera-only low-texture | wheel-assisted | 변화 |
| --- | ---: | ---: | ---: |
| visual 처리 / skip | 1,427 / 0 | 372 / 1,056 | 정지·미소 이동 frame 생략 |
| `quality=0` | 20 | 5 | 15회 감소 |
| registration failure | 32 | 9 | 71.9% 감소 |
| 자동 odometry reset | 2 | 0 | 제거 |
| graph pose / link | 33 / 36 | 38 / 37 | 단일 neighbor graph 유지 |
| 2cm 점군 | 44,498점 | 45,989점 | 유사 밀도 |
| translation ATE RMSE | 0.7346m | 0.2070m | 71.8% 감소 |
| translation RPE RMSE | 0.0411m | 0.0214m | 47.9% 감소 |
| yaw RMSE | 54.6663° | 17.8910° | 67.3% 감소 |
| yaw RPE RMSE | 1.1508° | 0.5948° | 48.3% 감소 |
| visual / wheel path | 17.8186 / 3.2096m | 9.3119 / 3.2096m | 과대 누적 완화, 미해결 |
| accepted loop closure | 0 | 0 | 폐루프 입력 필요 |

휠 odometry는 ground truth가 아니므로 위 수치는 절대 정확도가 아니라 동일 기록에서의 상대
비교다. visual path가 휠 path보다 190.1% 긴 현상도 남아 있어 카메라 궤적의 고주파 보정과
잔여 드리프트를 추가로 확인해야 한다.

wheel-assisted 설정에는 올바른 TF frame 분리, wheel guess와 5mm·0.005rad 이동 문턱이
함께 들어갔다. 따라서 개선량을 wheel guess 하나의 인과 효과로 해석하지 않는다. 각 요소의
기여도가 필요하면 새 폐루프 bag에 대해 `camera-only → wheel guess, gate 0 → wheel guess,
gate 0.005` 순으로 ablation한다.

## 실행 명령과 산출물

```bash
bash "$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros/jdamr_cube_vslam/scripts/run_rtabmap_docker.sh" \
  "$HOME/jdamr_data/vslam/rgbd_20260916T172638/bag_paired_10fps" \
  "$HOME/jdamr_data/vslam/rgbd_20260916T172638/rtabmap_paired_wheel_guess_odom_gate" \
  --profile low-texture \
  --odom-guess-frame odom
```

주요 산출물은 다음 경로에 있다.

- DB: `$HOME/jdamr_data/vslam/rgbd_20260916T172638/rtabmap_paired_wheel_guess_odom_gate/jdamr_rgbd.db`
- 2cm 점군: 같은 디렉터리의 `jdamr_rgbd_cloud.ply`
- A/B 이미지: 같은 디렉터리의 `ab_cloud_preview.png`
- 궤적 평가: 같은 디렉터리의 `accuracy.json`, `accuracy.md`

## 다음 게이트

충전 후 새 실차 기록은 정지 5초, 특징 구간, 저텍스처 구간, 출발 pose 재방문, 복귀 후
정지 5초를 포함한다. 같은 bag에서 camera-only와 wheel-assisted를 다시 실행하고 다음을
모두 만족해야 3D 지도 성공으로 판정한다.

- 자동 odometry reset 0회
- 한 개의 연속 graph와 점군
- 출발점 재방문에서 유효한 loop 또는 proximity link 생성
- 벽의 중복·부채꼴 찢어짐이 없음
- 휠 기준 ATE·RPE와 시작·종료 오차가 camera-only보다 개선
- 카메라 장착을 움직이지 않았거나 외부 파라미터를 다시 측정함
