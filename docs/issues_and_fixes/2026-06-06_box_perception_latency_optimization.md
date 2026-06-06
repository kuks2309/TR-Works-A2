# 2026-06-06 09:12 (KST) — box_perception 인지 지연 최적화 (검출→마커 박스당 7.2→1.3ms)

원격 Hailo 추론 최적화([2026-06-05](2026-06-05_remote_hailo_seg_postprocess_optimization.md)) 직후
다음 병목인 `box_perception_node` 의 검출→마커(인지 stage 5 box 위치계산 + stage 6 마커 발행)
지연을 계측·제거했다.

## 증상

- 라이브 전체 파이프라인에서 `/yolov8_node/detections` → `/detected_boxes_markers` 구간이
  **8박스에 57.6 ms** (박스당 ~7.2 ms). 최적화된 Pi 추론(~31 ms)과 맞먹는 수준.

## 원인

`box_perception_node` 의 `_on_detections` 내부 sub-stage 를 계측(box당):

| sub-stage | 박스당 | 원인 |
|---|---|---|
| `_point3d` (depth ROI 투영) | ~7 ms | depth ROI 를 통째로 `float64` 변환 후 `np.where` |
| `_on_depth` | 상시 부하 | **매 depth 프레임(~30 Hz) `cv_bridge.imgmsg_to_cv2`** 변환을 single-thread executor 에서 수행 → `_on_detections` 와 GIL 경합 |
| TF lookup | 작음(캐시) | 단, `_cam_to_base` 가 **박스마다 TF lookup 반복**(프레임당 동일 변환인데 N회) |

병목은 TF 가 아니라 `_point3d` + 상시 depth 변환 경합이었다(처음 가설이던 "TF per-box"
는 계측으로 정정).

## 수정

`3d_detect_ws/.../box_perception_node.py`:

1. **depth 지연 변환** — `_on_depth` 는 raw 메시지만 저장, `_latest_depth()` 가
   `np.frombuffer(...).reshape(H,W)` numpy view(복사·cv_bridge 없음)로 **검출당 1회**만 디코드.
   매 프레임 30 Hz 변환 제거.
2. **`_point3d` 유효 픽셀만 변환** — ROI 전체 `float64` 대신 `np.nonzero` 후 유효(>0)
   픽셀만 `float32 * 1e-3`.
3. **TF lookup 루프 밖 1회** — 프레임당 camera→base 변환을 한 번 구해 R/t 를 모든 박스에
   재사용(`_cam_to_base` 제거). 박스 수와 무관.

## 검증

클린 재기동(kill_all → hardware + d435_camera + yolo_remote(node_name:=yolov8_node) + rviz),
단일 인스턴스 확인 후:

- `_point3d` ~7 → ~2 ms/box (~3.5×), TF 박스당 → 0.7 ms 총합(박스 무관).
- 검출→마커: **8박스 57.6 ms → 22박스 28.8 ms** (박스당 **7.2 → 1.3 ms, ~5.5×**).
- 기능: goal → 18 검출 → 15박스 → **15 pose + 마커 발행, 에러 0**, TF 정상,
  RViz 렌더 확인(`experiments/capture/meas_rviz_boxes.png`).

## 재발 방지

- 인지 노드에서 **센서 메시지를 매 프레임 변환하지 말 것** — 소비 시점(검출 콜백)에 1회만 디코드,
  가능하면 numpy view 로(복사 회피).
- 프레임 단위로 불변인 값(TF 등)은 **루프 밖에서 1회** 계산.
- depth/마스크 통계는 필요한 픽셀(유효/ROI)만 변환.
- 주의: 같은 파일에 사용자 `ws_z` 재캘리브([0.10,0.32]→[0.64,0.86], raised-arm +0.54 m,
  보드 base z≈0.72, 박스 윗면 z≈0.78)가 동시 편집으로 공존. 이 값이 안 맞으면 검출은 되나
  markers=0(워크스페이스 필터 탈락)이 된다.
