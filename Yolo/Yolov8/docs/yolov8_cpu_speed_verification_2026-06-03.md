# YOLOv8 CPU 추론 속도 검증 (10 Hz 달성 여부)

- **검증일**: 2026-06-03
- **대상**: `Yolo/Yolov8` venv(`yolov8_env`)로 구동되는 `yolov8_detection` 노드의 YOLOv8 CPU 추론
- **목적**: CPU에서 YOLOv8 추론 속도를 측정하고 **10 Hz(프레임당 ≤ 100 ms)** 실시간 목표 달성 여부를 판정
- **결론**: **달성 불가 (FAIL).** 실제 배포 모델(`yolov8l-worldv2.pt`)은 전체 8코어를 써도 프레임당 **약 1.4 s (≈ 0.7 FPS)** 로, 10 Hz 대비 **약 14배 느림**. 가장 가벼운 `yolov8n.pt` 의 단일 최고 기록조차 178 ms(≈ 5.6 FPS)로 100 ms 예산을 넘김.

---

## 1. 요약 판정

| 모델 | 용도 | 평균 지연(1스레드) | 평균 지연(8스레드) | 최고(min, 8스레드) | 최대 FPS | 10 Hz |
|---|---|---:|---:|---:|---:|:---:|
| `yolov8n.pt` | 노드 기본 파라미터 | 523.6 ms | 401.8 ms | 250.7 ms | ~4.0 FPS | ❌ FAIL |
| `yolov8s-world.pt` | (경량 오픈보캐) | 780.8 ms | 507.3 ms | 354.7 ms | ~2.8 FPS | ❌ FAIL |
| `yolov8l-worldv2.pt` | **실행 스크립트 기본(배포)** | 3169.6 ms | 1404.2 ms | 1329.9 ms | ~0.75 FPS | ❌ FAIL |

> 10 Hz = 프레임당 100 ms. **세 모델 모두 단일 최고 기록조차 100 ms를 초과**하므로 CPU 단독으로 10 Hz 연속 추론은 불가능하다.

---

## 2. 검증 환경

| 항목 | 값 |
|---|---|
| CPU | Intel Core i7-10610U @ 1.80 GHz (4 코어 / 8 스레드, turbo 4.9 GHz) |
| RAM | 15 GiB |
| GPU | 없음 (CUDA 사용 불가) |
| Python | 3.10.12 (`yolov8_env`) |
| PyTorch | 2.12.0+cu130 — **CUDA 빌드지만 `torch.cuda.is_available() == False` → CPU 폴백** |
| Ultralytics | 8.4.55 |
| 입력 프레임 | 640×480 BGR (D435 컬러 스트림 해상도) |
| 추론 설정 | `imgsz=640, conf=0.10, iou=0.5, device='cpu'` — `yolov8_node.py::_process` 의 `predict()` 호출과 동일 |

### 측정 시 시스템 부하 (중요)
검증 시점에 로봇 스택이 동시 구동 중이었다. `loadavg(1m) ≈ 5–10` (8 논리코어 기준 과부하), 주요 CPU 점유: `rviz2`(36%), `anydesk`(17%), `Xorg`(15%), `box_align_node`, `omx_movel_controller`, `ros2_control_node`, `chrome`, `claude`. 따라서 **평균(mean)/최대(max) 값은 경합으로 부풀려져 있고, 최소(min) 값이 무경합 상태에 가장 가까운 best-case 근사**다. 단, 결론(10 Hz 불가)은 best-case 기준으로도 동일하게 성립한다.

---

## 3. 측정 방법

- 스크립트: [scripts/benchmark_yolov8_cpu.py](../scripts/benchmark_yolov8_cpu.py)
- 노드의 실제 추론 호출과 동일하게
  `self._yolo.predict(source=frame, conf, iou, imgsz=640, device='cpu', verbose=False)` 을 호출.
- YOLO-World 모델(`*-world*.pt`)은 실행 스크립트와 동일하게 `set_classes(["cardboard box","box","carton","package"])` 적용 후 측정.
- 각 모델 warmup 2회 후 15회 반복, 벽시계(wall-clock) 지연과 Ultralytics 내부 `speed`(pre/inf/post)를 모두 기록.
- 스레드 수를 명시 통제: **시나리오 A = 1스레드(운영 기본값)**, **시나리오 B = 8스레드(전체 논리코어, best-case)**.

### 운영 스레드 수에 대한 발견 (OMP_NUM_THREADS=1)
`yolov8_node.py` 는 `from ultralytics import YOLO` 를 (torch보다 먼저) import한다. 이때 **ultralytics가 `OMP_NUM_THREADS=1` 을 설정**하여 PyTorch 인트라옵 스레드가 **1개로 고정**된다(검증으로 확인). 즉 운영 노드는 기본적으로 **단일 스레드 CPU 추론**으로 동작한다 → 시나리오 A가 실제 운영 조건. (torch를 먼저 import하면 4스레드가 기본)

---

## 4. 측정 결과 (원자료)

원본 로그: [_bench_threadsweep.log](_bench_threadsweep.log), [_bench_imgsz640.log](_bench_imgsz640.log)

### 시나리오 A — threads=1 (운영 기본값), loadavg ≈ 4.9–9.9

| 모델 | mean | median | p95 | min | max | FPS(mean) |
|---|---:|---:|---:|---:|---:|---:|
| `yolov8n.pt` | 523.6 ms | 389.1 ms | 744.6 ms | 178.0 ms | 1975.1 ms | 1.91 |
| `yolov8s-world.pt` | 780.8 ms | 677.4 ms | 1115.5 ms | 544.1 ms | 1277.8 ms | 1.28 |
| `yolov8l-worldv2.pt` | 3169.6 ms | 2878.3 ms | 4436.3 ms | 2064.3 ms | 5162.7 ms | 0.32 |

### 시나리오 B — threads=8 (전체 코어, best-case), loadavg ≈ 9.0

| 모델 | mean | median | p95 | min | max | FPS(mean) |
|---|---:|---:|---:|---:|---:|---:|
| `yolov8n.pt` | 401.8 ms | 415.3 ms | 510.2 ms | 250.7 ms | 642.2 ms | 2.49 |
| `yolov8s-world.pt` | 507.3 ms | 407.1 ms | 737.5 ms | 354.7 ms | 1194.5 ms | 1.97 |
| `yolov8l-worldv2.pt` | 1404.2 ms | 1407.5 ms | 1463.0 ms | 1329.9 ms | 1482.4 ms | 0.71 |

> 8스레드 `yolov8l-worldv2` 는 분산이 매우 작다(1329.9–1482.4 ms). 연산 바운드(compute-bound)로 8코어를 포화시켜 동시 부하의 영향을 거의 받지 않으므로, **이 ≈ 1.4 s 값이 가장 신뢰도 높은 배포 모델의 실측 지연**이다.

추론(inference) 비중: 모든 경우 전처리/후처리는 합 ≈ 10–20 ms 이하이며 지연의 **99% 이상이 순수 신경망 forward**다. 즉 후처리 최적화로는 개선 여지가 없고, 모델·연산장치 자체가 병목이다.

---

## 5. 분석

1. **10 Hz(100 ms) 불가**: 실제 배포 모델 `yolov8l-worldv2.pt` 는 8코어 best-case에서도 ≈ 1.4 s/frame(0.71 FPS)로 목표의 **약 14배 느리다**. 노드 기본 모델 `yolov8n.pt` 조차 단일 최고 기록 178 ms(5.6 FPS)로 100 ms를 넘는다.
2. **스레드 증설 효과 제한적**: 1→8 스레드로 `yolov8l` 은 3.2 s→1.4 s(약 2.3배)로 개선되나 여전히 10 Hz와 한 자릿수 배율 차이. nano는 1→8에서 오히려 평균이 비슷(경합·하이퍼스레딩 한계).
3. **GPU 부재가 근본 원인**: PyTorch는 CUDA 빌드지만 사용 가능한 GPU가 없어 CPU로 폴백. GPU(예: RTX 계열)에서는 동일 모델이 통상 10–50배 빨라 10 Hz가 쉽게 달성된다.
4. **설계상 노드는 연속 10 Hz가 아니다**: `yolov8_node.py` 는 **on-demand `DetectBox` 액션 서버**로, 매 프레임이 아니라 **goal당 1회만** 추론한다(주석/코드 확인). 따라서 현재 파이프라인에서 10 Hz 연속 추론은 요구 사항이 아니며, 박스 1회 검출에 `yolov8l-worldv2` 기준 **약 1.4–3 s** 가 소요된다고 이해하면 된다.

---

## 6. 결론

- **CPU(i7-10610U)에서 YOLOv8의 10 Hz 연속 추론은 어떤 테스트 모델로도 달성 불가능하다.**
- 실제 배포 구성(`yolov8l-worldv2.pt`, imgsz 640)의 1회 검출 지연은 **약 1.4 s(8코어) ~ 3.2 s(1코어, 부하시)** 다.
- 다만 현 노드는 on-demand 1회 추론 설계이므로 10 Hz 자체가 운영 요구는 아니다.

## 7. 권고 (10 Hz가 실제로 필요할 경우)

1. **GPU 사용** — `device:='cuda'` + GPU 장착. 가장 확실하고 직접적인 해결책.
2. **모델 경량화** — 박스 검출이 목적이면 오픈보캐 `yolov8l-worldv2`(90 MB) 대신 COCO 파인튜닝 `yolov8n/s`(6–26 MB) 사용. CPU에서도 수 배 빠름.
3. **입력 해상도 축소** — `image_size` 파라미터를 640→320/416 으로 낮추면 연산량이 약 2.4–4배 감소(정확도 trade-off). 단 CPU에서 box-world 모델은 여전히 10 Hz 미달.
4. **추론 가속 백엔드** — ONNX Runtime / OpenVINO 로 export(`yolo export format=openvino`). Intel CPU에서 OpenVINO는 통상 2–4배 가속.
5. **스레드 명시 설정** — CPU 멀티스레드를 쓰려면 노드에서 `torch.set_num_threads(N)` 을 명시(현재는 ultralytics가 1로 고정).

---

## 8. 재현 방법

```bash
cd /home/openarmx/TR-Works/kkw/China/Yolo/Yolov8
VENV_PY=./yolov8_env/bin/python
ROOT=/home/openarmx/TR-Works/kkw/China

# 운영 기본값(1스레드)
$VENV_PY scripts/benchmark_yolov8_cpu.py --threads 1 --imgsz 640 --conf 0.10 \
  --models "$ROOT/3d_detect_ws/yolov8n.pt" "$ROOT/yolov8s-world.pt" "$ROOT/yolov8l-worldv2.pt"

# 전체 코어(best-case)
$VENV_PY scripts/benchmark_yolov8_cpu.py --threads 8 --imgsz 640 --conf 0.10 \
  --models "$ROOT/3d_detect_ws/yolov8n.pt" "$ROOT/yolov8s-world.pt" "$ROOT/yolov8l-worldv2.pt"
```

> 신뢰도를 높이려면 로봇 스택(RViz/컨트롤러/AnyDesk/브라우저)을 종료한 무부하 상태에서 재측정 권장. 단 본 검증의 결론(10 Hz 불가)은 best-case(min) 기준으로도 변하지 않는다.
