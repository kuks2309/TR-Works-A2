# Grasp 기준자세 모델 + INIT 등록 (2026-06-07)

대상: `experiments/ptp_pick_seq_v2_left.py` (`--side`로 좌/우), 오른팔 단일캠 pick 시퀀스.

## 증상

far-right(박스 |Y| ≳ 0.16) 구석에서 오른팔 파지가 반복 실패(빈 손). 자세 패턴:
- 뒤틀림(yaw −73°), 수직화(pitch −10°) 등 — 자유(position-only) IK(Inverse Kinematics)가 자세를 자유롭게 풀어
  far-right 에서 손목이 뒤틀리거나 하강이 수직으로 드리프트 → 그리퍼가 박스를 못 감쌈.
- 성공 구역(Y −0.07~−0.155)은 P≈−29° 자연 슬랜티드로 잘 잡힘.

## 원인

1. **자세를 자유 IK(Inverse Kinematics)로 매번 새로 풀어** 위치별 "좋은 파지 기울기"가 보장되지 않음.
   far-right 는 더 큰 기울기(tilt)가 필요한데 자유 IK 는 그걸 못 맞추거나 뒤틀린 해를 고름.
2. 핸드 가이드 실측: 사용자가 손으로 만든 "잡기 좋은" 자세의 기울기가 **위치에 따라 16°~58°**.
   기존 상한 `PITCH_MAX=45°` 가 far-right 의 좋은 각도(~50°)를 오히려 걸러냄.

## 수정

1. **PITCH_MAX 45° → 55°** (핸드 가이드 시연 ~50° 수용).
2. **grasp 기준자세 모델** (`experiments/build_grasp_reference_model.py` → `right_grasp_reference_model.yaml`):
   - 핸드 가이드로 6점(박스 위치 ↔ 좋은 자세) 수집 → `experiments/right_grasp_reference_dataset.yaml`.
   - 평면 회귀 `tilt(X,|Y|) = 135.6·X + 25.9·|Y| − 7.86` (RMSE 4.8°, 자세 측지각 평균 4.7°).
   - 목표 자세 `R = rpyToMatrix(180°, −tilt, yaw0)` (roll 180, yaw≈0).
3. **시퀀스 배선** (`solve_pick_refmodel`): 박스 (X,Y) → 모델 tilt → R, **접근·하강 모두 그 R 로 6D IK**
   (자세 구속) → "접근 자세 = 파지 자세"(수직화/뒤틀림 제거). 상승=접근 자세 복귀. 기본 경로,
   `--optimal` 시 구 후보탐색 폴백. 왼팔은 yaw 부호반전(미러).
4. **INIT 등록 + 좌우 미러**: 핸드 가이드 자세를 INIT 으로 등록(테이블 충돌 회피 상향). 미러 규칙
   실증(FK(Forward Kinematics) perr=0mm): **j4 만 동일, 나머지 부호반전** `s=(-1,-1,-1,+1,-1,-1,-1)`.
5. **자유 구동(핸드 가이드) 도구·정책**: `experiments/arm_freedrive.py` + 작업 지시서
   `docs/work_instructions/2026-06-07_arm_freedrive_handguide.md` (홈에서만 ON / 해제 시 홈 복귀).

## 검증

- DRY + 실로봇: 중앙~중간 구역(예 박스 X0.26/|Y|0.13) tilt 32.7°, 접근/하강 err 0.0/0.1mm, **파지 성공**
  (finger 0.0122m, 단단히 물림). IK(Inverse Kinematics) 33~40ms.
- 좌우 INIT FK(Forward Kinematics) 미러 대칭 검증(위치 Y 반전, roll/yaw 반전, pitch 동일).

## 남은 한계 (재발 방지 관찰)

- **극단 far-right 코너**(박스 X≈0.36, Y≈−0.21)는 모델 tilt 48.8°(도달 err 0)에도 빈 손 발생.
  자세가 아니라 **그 구석의 도달/접촉 한계 또는 descend_z(0.75 고정) 부적합** 가능성 → 추가 데이터/
  위치별 z 보정 필요. 성공 구역은 안정적.
- 기준자세 모델은 6점 기반 → far/극단 구역은 외삽이라 정확도 낮음. 점 추가 시 재적합.

## 10회 반복 테스트 (2026-06-07, `experiments/run_pick_tests.py`, 결과 `pick_test_results.yaml`)

성공 **7/10**.
- 성공(7): 박스 X 0.36~0.40, |Y| 0.08~0.19, tilt 46.8~50.9, finger 0.0104~0.0134(단단히 물림).
- 실패#7: 검출 0(일시적) → 러너에 **검출 재시도(최대 3회)** 추가로 보완.
- 실패#8 (0.386, −0.064) tilt48.2, 실패#10 (0.402, −0.091) tilt51.1: 둘 다 finger=0.0089(빈 손 닫힘).
  → **far+central(최대 도달 + |Y|≤0.09)** 에 실패가 몰림. 인접 far+lateral(|Y|≈0.11)은 성공(#4,#5,#9).

분석: far+central 은 (1) 팔 최대 신전으로 무적분 droop 이 가장 크고, (2) 기준자세 모델 학습점이 가장
적고 적합이 가장 약한 구역(모델 빌드시 central 점 #5 잔차 −7.5° 로 최대). → 보완 후보:
  (a) far+central 기준자세 점 2~3개 핸드 가이드 추가 → 모델 재적합(정공법),
  (b) 시퀀스에 빈손 감지 시 재파지(재하강+재폐) 1회 재시도,
  (c) far 도달 시 descend droop 보정(하강 z 살짝 낮춤) — 실측 검증 후.
