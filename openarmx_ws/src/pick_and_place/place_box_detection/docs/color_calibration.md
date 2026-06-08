# 색상 캘리브레이션 — 현장 재튜닝 절차 + 컨테이너 색 학습 계획

place-box 검출의 3단계 중 **Stage 3 색상 분류**는 inlier RGB → HSV(Hue Saturation Value)
색상값(Hue) 밴드로 5색을 판정한다. **HSV 임계는 카메라·조명에 의존**하므로 배치 현장이
바뀌면 다시 캘리브레이션해야 한다(본 문서가 그 절차의 단일 근원).

관련 코드: [src/color_classifier.cpp](../src/color_classifier.cpp) `hsvToColor()`,
캘리브 도구: [scripts/color_calib.py](../scripts/color_calib.py)

---

## 1. 현재 임계 (2026-06-08, 이 D435 실측 기준)

| 색 | 밴드 (Hue°) |
| --- | --- |
| red | H < 9 (또는 H ≥ 345) |
| orange | 9 ≤ H < 30 |
| yellow | 30 ≤ H < 70 |
| green | 70 ≤ H < 170 |
| blue | 170 ≤ H < 265 |
| (그 외 / 채도 S<0.25·명도 V<0.15) | unknown |

> 기본 15°였던 red/orange 경계를 **9°** 로, 45°였던 orange/yellow 경계를 **30°** 로
> 실측 재튜닝함. 이유는 §2 레퍼런스 참조.

## 2. 측정 레퍼런스 (2026-06-08, 동일 카메라/조명)

| 색 | RGB(평균) | H_med (p05~p95) | G/R | B/R | sat | val |
| --- | --- | --- | --- | --- | --- | --- |
| red | (67, 16, 10) | **6.1** (3.9~8.9) | 0.241 | 0.154 | 0.85 | 0.26 |
| orange | (91, 18, 3) | **10.3** (8.3~12.9) | 0.196 | 0.028 | 0.97 | 0.35 |
| yellow | (113, 88, 1) | **46.7** (45.7~47.7) | 0.784 | 0.011 | 0.99 | 0.44 |
| green | (5, 46, 19) | **142.0** (137~145) | — | — | 0.91 | 0.18 |
| blue | (0, 24, 62) | **216.8** (215~218) | — | — | 1.0 | 0.24 |

요점:
- **red/orange가 유일한 접전 쌍** (둘 다 저(低) Hue, 꼬리가 8.3~8.9°에서 겹침). 경계 9°는
  마진이 얇음 — 현장에서 깨지면 가장 먼저 재측정할 곳.
- red가 orange보다 Hue가 낮은 본질 원인 = **파랑 함량(B/R)** 차이(red 0.154 vs orange 0.028).
  red는 파랑기 도는 "차가운" 색이라 G−B가 작아 Hue가 0에 가까움.
- green/yellow/blue는 서로 멀어(빈 구간 큼) 안정적.

## 3. 현장 재캘리브레이션 절차

### (0) 전제 검증 — "실행 전 항상 노드 확인" (조용한 nan 방지)
```bash
# 셋 다 실제로 발행 중인지 확인(프로세스 UP만으론 부족)
ros2 topic hz /camera/camera/depth/color/points     # D435 cloud
ros2 topic hz /tof/range                            # TOF 거리
ros2 run tf2_ros tf2_echo openarmx_body_link0 camera_depth_optical_frame
```
부분 재시작을 반복했다면 DDS(Data Distribution Service)가 신규 노드 수신을 막을 수 있다.
이때는 `kill_all_ros2.sh`(FastDDS shm 정리) 후 클린 재기동.

### (1) 색별 시그니처 측정 (한 번에 한 색)
각 색 박스를 카메라 앞(벽 위치)에 두고:
```bash
cd .../place_box_detection
python3 scripts/color_calib.py red      # 이어서 orange, yellow, green, blue
column -t -s, /tmp/color_calib.csv      # 누적 표 확인
```
도구는 TOF 거리 ROI + 수직평면 RANSAC 으로 **벽만** 분리해 색을 읽는다(데스크·소형박스 배제).

### (2) 경계 도출
- 인접한 두 색의 `H_med` **사이**에 경계를 둔다.
- p05/p95 꼬리가 겹치면, 다수결로 양쪽 median 이 옳게 분류되는 값을 고른다(빈 구간이 크면
  그 중앙). 예: orange(10.3) / yellow(46.7) → 빈 구간 13~45 중앙 = **30**.

### (3) 코드 반영 + 검증
- [src/color_classifier.cpp](../src/color_classifier.cpp) `hsvToColor()` 의 경계 상수 수정.
- 재빌드: `colcon build --packages-select place_box_detection`.
- 측정한 평균 RGB 를 새 임계에 넣어 5색 전부 제 라벨로 나오는지 검산(논리/라이브 둘 다).

### 주의
- **min_saturation(0.25)/min_value(0.15)**: 박스가 어둡거나(현 val 0.18~0.44) 조명이 약하면
  일부 점이 unknown 이 된다. 너무 많이 빠지면 두 하한을 낮춘다.
- HSV 단독으로 red/orange 분리가 계속 불안하면 **B/R(또는 G−B) 보조 판별**을 저Hue 구간에
  추가하는 것을 고려(레퍼런스상 B/R 갭이 Hue 갭보다 큼).

---

## 4. 큰 박스(컨테이너) 색 학습 — 추후 계획

현재 색 분류는 **yolov8 mini-box 5색(blue/green/orange/red/yellow)** 만 대상이고,
큰 박스(컨테이너) 자체의 색은 "나중에 학습"으로 보류됨(사용자 지시 2026-06-08).

- 현 동작: 컨테이너가 위 5색 Hue 밴드에 들면 그 색으로, 아니면(예: 마젠타/보라 H 265~345)
  **unknown** 으로 분류. UI Auto "컨테이너 색" 모드는 unknown 이면 자동 시작을 보류한다.
- 학습 방향 (택1, 추후 결정):
  1. **yolov8 재학습**: mini-box-seg 모델에 컨테이너 색 클래스를 6번째로 추가 학습 →
     색 판정을 학습 모델로 일원화(red/orange 같은 모호색에 더 견고).
  2. **HSV 밴드 확장**: 컨테이너 색을 §1 표에 6번째 밴드로 추가(이 도구로 시그니처 측정 후).
- 어느 쪽이든 **현장 조명에서 컨테이너 박스를 본 도구(`color_calib.py container`)로 측정**하는
  단계가 선행되어야 한다(학습 데이터/밴드 모두 현장 색이 기준).
