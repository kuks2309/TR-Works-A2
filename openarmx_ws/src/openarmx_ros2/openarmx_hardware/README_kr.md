# openarmx_hardware 설명

[English](README.md) | [简体中文](README_CN.md) | 한국어

OpenArmX V10(Robstride 모터 버전)을 위한 ROS 2 `hardware_interface::SystemInterface` 플러그인입니다. 하위 계층의 CAN 통신을 위해 `openarmx_can`에 의존합니다.

## 기능 개요
- 플러그인 클래스 `openarmx_hardware/OpenArmX_v10HW` (`openarmx_hardware.xml` 참조)는 `ros2_control` 하드웨어 블록에서 직접 참조할 수 있습니다.
- 7관절 매니퓰레이터 + 선택적 그리퍼(기본 활성화)를 지원합니다. 관절 이름은 자동 생성됩니다: `openarmx_<prefix>joint{1..7}` 및 `openarmx_<prefix>finger_joint1`.
- MIT 모션 컨트롤 모드(기본값)와 CiA402 CSP 모드를 지원하며, 각 관절별로 KP/KD를 설정할 수 있습니다.
- 동적 파라미터 노드 `openarmx_<prefix>hardware_params`가 `kp_joint1..8`, `kd_joint1..8`을 노출하여 런타임에 조정할 수 있습니다 (8번째는 그리퍼).
- 기본적으로 CAN Socket을 사용하며 (CAN-FD 선택 가능), Robstride 모터 모델과 CAN ID 매핑이 내장되어 있고, 모터 방향 계수는 -1.0으로 통일되어 있습니다.
- 그리퍼는 관절 변위 0–0.044 m와 모터 라디안 0–1.0472 사이를 매핑합니다.

## 빌드
의존성: ROS 2(rclcpp, rclcpp_lifecycle, hardware_interface, pluginlib)와 `openarmx_can`.

```bash
colcon build --packages-select openarmx_hardware
source install/setup.bash
```

**주의:** openarmx_hardware는 openarmx_can에 의존하므로, openarmx_can을 먼저 빌드한 후 openarmx_hardware를 빌드해야 합니다.


## ros2_control에서 사용
URDF/xacro의 하드웨어 설정에서 플러그인을 선언합니다.

```xml
<ros2_control name="openarmx" type="system">
  <hardware>
    <plugin>openarmx_hardware/OpenArmX_v10HW</plugin>
    <param name="can_interface">can0</param>
    <param name="arm_prefix"></param>        <!-- 듀얼암의 경우 left_ / right_ 로 설정 가능 -->
    <param name="hand">true</param>          <!-- 그리퍼 활성화 여부 -->
    <param name="can_fd">false</param>       <!-- 버스가 지원하는 경우 CAN-FD 활성화 가능 -->
    <param name="control_mode">mit</param>   <!-- mit(기본값) 또는 csp -->
  </hardware>
  <!-- 여기에 관절 정의를 추가합니다 -->
</ros2_control>
```

하드웨어 파라미터 (hardware 태그):
- `can_interface` (문자열, 기본값 `can0`): CAN 포트 이름.
- `arm_prefix` (문자열, 기본값 빈 문자열): 관절 이름 접두사로, 다중 매니퓰레이터 구분에 사용됩니다.
- `hand` (불리언, 기본값 `true`): 그리퍼 포함 여부.
- `can_fd` (불리언, 기본값 `false`): CAN-FD로 초기화할지 여부.
- `control_mode` (문자열, 기본값 `mit`): `mit`은 모션 제어를 수행하고, `csp`는 CiA402 위치 참조를 전송합니다.

런타임 KP/KD 파라미터 (노드 `openarmx_<prefix>hardware_params`):
- 기본값: KP = `[50, 50, 50, 50, 10, 10, 10, 50]`, KD = `[2.5, 2.5, 2.5, 2.5, 0.5, 0.5, 0.5, 2.5]`.
- 파라미터 조정 예시: `ros2 param set openarmx_hardware_params kp_joint1 80.0`.

## 동작 특성
- 활성화 시: 콜백 모드를 STATE로 전환하고, 모터를 활성화한 뒤 홈 포지션으로 복귀시킵니다 (그리퍼도 초기 위치로 복귀). CSP 모드에서는 먼저 속도/전류 제한을 설정한 후 활성화합니다.
- 상태 인터페이스: 각 관절에 대해 위치/속도/토크를 내보냅니다. 그리퍼의 속도와 토크는 현재 0으로 채워집니다.
- 명령 인터페이스: 각 관절에 대해 위치/속도/토크를 내보냅니다. MIT는 모션 제어로 전송하고, CSP는 목표 위치 프레임을 전송합니다.
- 디버그 출력: 활성화 후 처음 약 50회의 `read()`에서 원시 모터 값과 퍼블리시되는 관절 값을 출력하여 방향/영점 편차 확인에 도움을 줍니다.

## 비고
- 방향 계수는 -1로 고정되어 있으므로, 관절 부호 방향에 주의해 주시기 바랍니다. 그리퍼는 기본적으로 Robstride(RS00)를 사용하며 CAN ID는 0x08입니다. 하드웨어가 다른 경우 상위 단계에서 적절히 조정해야 합니다.

## 라이선스

본 저작물은 Creative Commons 저작자표시-비영리-동일조건변경허락 4.0 국제 라이선스 (CC BY-NC-SA 4.0)에 따라 이용 허락됩니다.

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

자세한 내용은 [LICENSE_kr.md](LICENSE) 파일을 참조하시거나 다음 링크를 방문하시기 바랍니다: http://creativecommons.org/licenses/by-nc-sa/4.0/

## 작성자

- **Zhang Li** (张力)
- 회사: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 웹사이트: https://openarmx.com/

## 버전

**현재 버전**: 1.0.0

---

## 📞 문의

### Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
**Chengdu Changshu Robotics Co., Ltd.**

| 연락처 | 정보 |
|---------|------|
| 📧 이메일 | openarmrobot@gmail.com |
| 📱 전화/WeChat | +86-17746530375 |
| 🌐 공식 웹사이트 | <https://openarmx.com/> |
| 📍 주소 | 천진 경제기술개발구 서구 신예팔가 11호 화성기계공장 |
| 👤 담당자 | Mr. Wang |
