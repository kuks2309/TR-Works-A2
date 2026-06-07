# Pick 시퀀스 v2 타이밍 기록 (2026-06-07)

## 대상
`experiments/ptp_pick_seq_v2_left.py` (`--side`로 좌/우 공용), 오른팔 래퍼
`experiments/ptp_pick_seq_v2_right.py` — 단일 중앙캠 box pick 자동 시퀀스
(검출 → 중간자세 → 접근 → 하강 → 80% 파지 → 상승 → dwell → release → INIT).

## 측정 환경
- 오른팔(붉은 박스). 튜닝: rate 60°/s, descend 1.3s, init 1.5s, settle 0.2s,
  dwell 3.0s(고정/사용자 지시), approach_z 0.87, descend_z 0.75.
- 계측: 스크립트 내장 lap(`_time.monotonic`) + IK 계산시간 + `/usr/bin/time -v`(벽시계/CPU/RSS).
- IK: 자세 자유(position-only) + pitch 상한(≤45°). 자연 슬랜티드 파지(P≈−29°).

## 측정 결과 (대표 1회, 파지 성공)

### IK(Inverse Kinematics) 계산 시간 — 셋업 단계, lap TOTAL 에 미포함
| 단계 | 시간 |
| --- | --- |
| 접근 | 27 ms |
| 하강 | 30 ms |
| 상승 | 25 ms |
| **합** | **82 ms** |

### 단계별(lap) — 동작 구간
| 단계 | 시간 | 비중 |
| --- | --- | --- |
| 2b_중간자세(forward) | 0.70 s | 6.1% |
| 2c_접근(forward) | 0.88 s | 7.8% |
| 4b_하강(JTC) | 1.52 s | 13.4% |
| 5_파지(비차단) | ~1.0 s | ~9% |
| 6b_상승(forward) | ~0.7 s | ~6% |
| 7_dwell(고정) | 3.0 s | 26.5% |
| 9_INIT 복귀(JTC) | ~2.0 s | ~18% |
| 스위치 4회 합 | ~0.12 s | ~1% |
| **TOTAL** | **11.32 s** | (실동작 8.32 s + dwell 3.0 s) |

### 벽시계 / 자원
- 벽시계(`time -v`): **15.42 s** = 셋업(urdf 수신·모델빌드·대기) ~4.1 s + IK 0.08 s + 동작 11.32 s
- CPU: **25%** (작업 평균 — 모션 ramp/JTC/dwell의 sleep 대기로 낮음)
- 최대 RSS: ~**100 MB**

### 점유율
| 기준 | IK 점유율 |
| --- | --- |
| 동작 TOTAL(11.32 s) | **0.72%** |
| 벽시계(15.42 s) | **0.53%** |
→ **IK 는 전체의 1% 미만, 병목 아님.**

## 최적화 이력 (튜닝 전 → 후)
| 시점 | TOTAL | 핵심 변경 |
| --- | --- | --- |
| 초기 | 24.1 s | — |
| grasp 비차단 | 14.6→12.3 s | 그리퍼 결과 대기(`stall_timeout` 10 s) 제거 → −9.5 s |
| JTC 단축 | 12.3→9.84 s | 하강 5→1.3 s, INIT 6→1.5 s + 워밍업 0.1 s |
| 현재(중간자세 추가) | 11.3 s | 테이블 충돌 방지 중간자세(+1.8 s 비용) |
| IK 방식 | 2.6 s → **82 ms** | ori-min(수직 우선) 제거 → 자연 슬랜티드 자유 IK 복귀 (−2.5 s, 약 32배) |

## 결론 / 병목
- **IK 계산은 무시 가능(82 ms, <1%).** 추가 IK 최적화 여지 없음.
- 사이클 시간 지배 요소: **고정 dwell 3 s(26%) + 셋업 ~4 s + 모션(하강 1.5 s · INIT 2 s · 중간자세 0.7 s · 접근 0.9 s)**.
- 추가 단축 레버:
  1. dwell 3→2 s (사용자 지시값) → ~−1 s
  2. forward rate 60→90°/s → 중간자세/접근/상승 ~−0.5 s
  3. **상주 노드화**(매 실행 rclpy 재시작·urdf 재수신 대신 데몬) → 셋업 ~4 s 제거 (가장 큼)

## 참조
- 스크립트: `experiments/ptp_pick_seq_v2_left.py` (`--side=left|right`), 래퍼 `..._right.py`
- 설정: `experiments/pick_seq_config_{left,right}.yaml`
- 관련 커밋: `e164d43` (feat(pick) 좌/우 단일캠 pick 자동 시퀀스 v2)
