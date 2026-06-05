# 2026-06-05 21:53 (KST) — 원격 Hailo seg 추론 지연: 마스크 후처리 병목 제거 (infer 77→31ms)

원격 Pi5+Hailo-8 박스검출 파이프라인의 단계별 처리속도를 실측하던 중, Pi 추론
구간(`infer_ms`)이 전체 비전 루프의 ~85%를 차지하는 것을 확인하고 그 원인을
규명·제거했다. 측정 스크립트: `experiments/measure_{pipeline_stages,live_pipeline,detect_action,full_pipeline}.py`,
`pi_yolo_server/{profile_seg,verify_seg_opt}.py`.

## 증상

- 원격 검출 비전 루프가 **11 Hz (주기 90ms)** 에 머묾. 연속 스트리밍/온디맨드 모두
  Pi `infer_ms` 가 지배적.
- 라이브 D435(640×480) E2E 실측: 캡처 5.6 / JPEG 인코딩 1.2 / **Pi 추론 77 / 네트워크+Flask 6.6 / 3D 역투영 0.05 ms**.
- "Hailo NPU 가 빠른데 왜 느린가" 가 핵심 의문.

## 원인

Pi에서 `HailoYoloSeg.infer()` 의 sub-stage 를 직접 계측(14박스):

| sub-stage | 시간 | 비중 |
|---|---|---|
| preprocess (resize+cvtColor) | 1.4 ms | 2% |
| **NPU dataflow (Hailo 추론)** | **14.4 ms** | 23% |
| postprocess (DFL decode + NMS) | 4.7 ms | 7% |
| **마스크 무게중심 루프** | **42.1 ms** | **67%** |
| 합계(=infer_ms) | 62.7 ms | |

병목은 NPU 가 아니라 **CPU 마스크 후처리**. 근본 원인은
`pi_yolo_server/hailo_seg.py` `make_mask()` 가 검출 1개마다
`sigmoid(proto@coeff)` → **`cv2.resize` 160→640 풀프레임 업샘플** → threshold →
`np.where(640×640)` 를 수행한 것(검출당 ~3ms, 박스 수에 선형). 라이브 ~20박스에선
마스크 루프만 ~60ms → infer 77ms.

## 수정

`pi_yolo_server/hailo_seg.py` — 무게중심을 640 으로 키우지 않고 **proto(160) 해상도에서
계산**하는 `mask_centroids()` 신설, `infer()` 가 이를 사용:

1. **640×640 리사이즈 제거** — 무게중심은 해상도에 둔감하므로 160 그리드에서 구해
   `s=NET/Hp(=4)` 로 환산. 검출당 `np.zeros(640,640)` / `cv2.resize` / `np.where(640²)` 제거.
2. **전 검출 배치 행렬곱** — `proto.reshape(-1,C) @ coeffs.T` 로 K개 인스턴스 마스크를
   한 번에(Python 검출별 행렬곱 루프 제거).
3. **sigmoid 제거** — `sigmoid(x)>0.5 ⟺ x>0` 이므로 logit 을 직접 `>0` 임계, per-element
   `np.exp` 전부 제거.
4. polygon(뷰어 `?masks=1`)만 기존 640 `make_mask()` 경로 유지(픽 경로는 미사용).

검증(`verify_seg_opt.py`, Pi에서 동일 NPU 출력으로 구·신 경로 비교):

- 후처리 **41.0 → 8.8 ms (4.7×)**, 정확도 동일 — 무게중심 차 평균 1.8px(=0.6m 깊이에서 ~1.8mm), 마스크면적 차 미미.
- 동일 14박스 이미지 POST: **Pi infer 62.6 → 31.0 ms**, RTT 91.8 → 37.6 ms, E2E측정구간 96.5 → 40.4 ms.
- 라이브 연속 비전 루프 **11.0 → 28.3 Hz (2.6×)**.

전체 라이브 ROS2 파이프라인(클린 재기동, 실 production 노드, 8박스): DetectBox 액션
왕복 91.6ms(이 중 RTT 43 / Pi infer 31), 검출→마커(box_perception, stage5+6) 57.6ms,
RViz 마커 렌더 시각확인(`experiments/capture/meas_rviz_boxes.png`).

Pi 배포: `~/AI/TR-Works/hailo_seg.py` 교체 + `yolo_server.service` 재시작, 백업 `hailo_seg.py.bak_*` 보존.

## 재발 방지

- 세그멘테이션 후처리에서 **per-detection 풀프레임 리사이즈 금지** — 마스크 통계(무게중심/면적)는
  proto 해상도에서 구하고 스케일 환산한다.
- threshold 전용 경로에서는 **sigmoid 생략**(단조함수라 임계점만 비교).
- 추가 관찰: 최적화 후 다음 병목은 **`box_perception_node`** 의 검출→마커(57.6ms/8박스,
  박스별 TF lookup + depth percentile Python 루프). 더 빠른 루프가 필요하면 여기와
  ROS2 액션 핸드셰이크(RTT 위 ~48ms)를 손봐야 한다.
- 하드 상한: D435 컬러 60fps → **100Hz 비전 루프는 불가**. 인식(≤~28Hz)과 제어(100Hz+)는 분리.
