# 주요 관찰사항 및 개선 권장사항

## A. Solver 관련 발견

### A-1. MoveIt KDL IK가 7-DOF redundant 팔에 사용
- **위치:** `openarmx_bimanual_moveit_config/config/kinematics.yaml`
- **문제:** KDL은 redundant manipulator에서 수렴률이 낮은 것으로 알려짐
- **권장:** TRAC-IK 또는 Pick-IK 플러그인으로 교체 시 성공률 개선 기대
- **참고:** `kdl_kinematics_plugin/KDLKinematicsPlugin` → `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`

### A-2. IK timeout 5 ms가 매우 공격적
- **위치:** `openarmx_bimanual_moveit_config/config/kinematics.yaml` (`kinematics_solver_timeout: 0.005`)
- **문제:** 실시간 의도이나 KDL 수렴에는 빠듯, 실패 시 fallback 없음
- **권장:** 50 ms로 완화하거나, MoveIt의 `kinematics_solver_attempts` 명시적 설정

### A-3. Pseudo-inverse 안정 SVD 비활성화
- **위치:** `openarmx_gravity_comp/src/dynamics.cpp` (`use_stable_svd` 하드코딩 false)
- **문제:** 특이점 근처에서 자코비안 폭주 가능
- **권장:** DLS (Damped Least Squares) damping 항 추가 또는 stable SVD 활성화

### A-4. dynamics.cpp 코드 중복
- **위치:**
  - `openarmx_gravity_comp/src/dynamics.cpp`
  - `openarmx_teleop_bimanual/src/dynamics.cpp`
- **문제:** 동일 KDL 래퍼 클래스가 두 패키지에 중복 존재
- **권장:** 별도 공유 라이브러리 패키지(`openarmx_dynamics`)로 추출

### A-5. VR IK 솔버가 바이너리 전용
- **위치:** `openarmx_arm_driver.OpenArmTeleopController` (install 디렉토리에만 존재)
- **문제:** 소스 비공개 → 디버깅/튜닝 제약
- **권장:** 소스 공개 여부 확인 또는 자체 Jacobian 기반 differential IK 구현 검토

## B. 워크스페이스 구조 발견

### B-1. 빌드 21일 경과
- **위치:** `openarmx_ws/install/`
- **권장:** `colcon build` 재실행 후 새 분석 (2026-05-14 기준)

### B-2. `docs/` 폴더가 분석 전 비어있었음
- **위치:** `/home/openarmx/TR-Works/kkw/China/openarmx_ws/docs/`
- **상태:** 본 분석 이전에는 비어있었고, `code_review/`, `cyclo_control/` 폴더만 존재
- **조치:** 본 분석으로 `analysis_2026-05-14/` 폴더 생성

### B-3. `lerobot_teleoperator_openarmx_leader_ros2` 위치 불명
- **참조:** VLA 코드에서 사용
- **상태:** src 디렉토리에 없음 — 외부 의존성 또는 미빌드 패키지일 가능성
- **권장:** `openarmx_vla.repos`에 추가 여부 확인

### B-4. 프로젝트 리브랜딩 흔적
- 예전 `openarm` 파일명이 현재 `openarmx`와 함께 존재 (특히 preview_bringup의 런치 파일)
- **권장:** 정합성을 위해 정리

## C. 라이선스 / 사용 발견

### C-1. CC BY-NC-SA 4.0 비상업용 라이선스
- **영향:** 상업적 사용 불가
- **권장:** 상업화 검토 시 라이선스 협상 필요

### C-2. 다국어 문서 (EN/CN/KR)
- 모든 주요 README가 3개 언어 제공
- 모터 매니저는 JP/RU 추가
- **참고:** 새 패키지 추가 시 다국어 정책 유지 검토

## D. 통합 / 운영 관련

### D-1. CAN 드라이버가 사전 컴파일된 DEB
- **위치:** `openarmx-can_1.0.0_{amd64,arm64}.deb`
- **영향:** colcon build 이전에 dpkg 설치 필수, 설치 누락 시 빌드 실패
- **권장:** 설치 스크립트 자동화 또는 setup.sh 제공

### D-2. 그리퍼 액션 명령 staggered
- **위치:** `openarmx_gripper_panel` (dual gripper 모드에서 right → left 5ms 간격)
- **이유:** 동시 명령 시 충돌 회피 추정
- **권장:** 주석 또는 문서로 의도 명확화

### D-3. JTC 모드에서만 그리퍼 spawner 분리
- **위치:** `openarmx_bringup/launch/openarmx.bimanual.launch.py`
- **상태:** forward_position 모드는 그리퍼 통합 (8 DOF 명령에 포함)
- **참고:** 컨트롤러 모드 전환 시 그리퍼 제어 인터페이스가 바뀜

## E. 후속 작업 후보

1. **TRAC-IK / Pick-IK 마이그레이션 PoC** — 양팔 IK 성공률 측정
2. **dynamics 공유 라이브러리화** — 중복 제거
3. **`openarmx_arm_driver` 소스 확보** — VR IK 튜닝 가능성 확보
4. **재빌드 + 회귀 테스트** — 21일 경과 빌드 갱신
5. **OMPL planner 명시적 설정** — Pilz 외 RRT-Connect 등 추가 검토
6. **CI/CD 도입 검토** — 빌드 자동화 및 회귀 감지
