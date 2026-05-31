# openarmx_tools

`openarmx_tools`는 OpenArmX의 도구 모음 패키지로, 주로 **디버깅, 티칭, 파라미터 튜닝 및 빠른 연동 테스트**에 사용됩니다.  
이 디렉터리의 각 서브 패키지는 독립적으로 빌드 및 사용할 수 있으며, RViz 시각화 제어부터 궤적 녹화/재생까지 일반적인 엔지니어링 요구사항을 포괄합니다.

> ⚠️ 본 디렉터리의 도구를 사용해 연동 테스트나 티칭을 수행하기 전에, 로봇 컨트롤러가 정상적으로 실행되어 있는지 먼저 확인하시기 바랍니다.

## 🧰 포함된 도구

1. `openarmx_joint_slider_panel`
- RViz2 관절 슬라이더 패널 (듀얼암 + 듀얼 그리퍼).
- 빠른 자세 조정, 시연, 연동 테스트에 적합합니다.
- 분할 스텝 실행을 지원하여 큰 점프로 인한 모션 충격을 줄여줍니다.

2. `openarmx_gripper_panel`
- RViz2 그리퍼 제어 패널.
- 좌측 그리퍼, 우측 그리퍼 또는 듀얼 그리퍼 동기 제어를 지원합니다.
- `GripperCommand` action 기반으로 명령을 전송합니다.

3. `openarmx_kp_kd_panel`
- RViz2의 KP/KD 파라미터 조절 패널.
- 팔/그리퍼에 대해 실시간으로 스티프니스와 댐핑을 조정할 수 있습니다.
- 좌/우 단일 팔 또는 듀얼암 모드를 지원하며, 실기 파라미터 튜닝에 적합합니다.

4. `openarmx_teach`
- 궤적 티칭 도구 (녹화 + 재생).
- `/joint_states`에서 YAML 궤적을 녹화하고, 듀얼암 및 그리퍼 컨트롤러로 재생합니다.
- 관절 필터링, 속도 스케일링, 그리퍼 동기화 전략을 지원합니다.

## 🚀 권장 사용 순서 (일반적인 실기 워크플로우)

1. 로봇 하위 계층 실행 (bringup/moveit, 컨트롤러 온라인 상태)
2. `openarmx_kp_kd_panel`로 적절한 스티프니스 조정
3. `openarmx_joint_slider_panel` 또는 `openarmx_gripper_panel`로 동작 연동 테스트 수행
4. `openarmx_teach`로 궤적 녹화 및 재생 검증 수행

> ✅ 위 순서대로 진행하시면, "토픽은 보이는데 동작하지 않는" 문제의 디버깅 비용을 크게 줄일 수 있습니다.

## 🔧 빠른 빌드

워크스페이스 루트 디렉터리에서 실행합니다.

```bash
colcon build --packages-select \
  openarmx_joint_slider_panel \
  openarmx_gripper_panel \
  openarmx_kp_kd_panel \
  openarmx_teach
source install/setup.bash
```

## 📚 문서 진입점

- `openarmx_joint_slider_panel/README_kr.md`
- `openarmx_gripper_panel/README_kr.md`
- `openarmx_kp_kd_panel/README_kr.md`
- `openarmx_teach/README_kr.md`

## ⚖️ 라이선스

본 디렉터리의 각 서브 패키지는 저장소 내에 명시된 라이선스를 따릅니다 (각 서브 패키지의 `LICENSE` 및 `README` 참조).
