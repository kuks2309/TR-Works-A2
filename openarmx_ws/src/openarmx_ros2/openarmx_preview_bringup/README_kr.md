# openarmx_preview_bringup

## 패키지 소개

`openarmx_preview_bringup`은 독립적인 래퍼 bringup(wrapper bringup) 패키지로, 본 로봇의 관절 제어 플러그인을 실행하여 사용자가 손쉽게 활용할 수 있도록 합니다.

본 패키지를 실행하면 항상 RViz2가 함께 기동되며, 다음 표시 항목이 사전 구성됩니다.

- `openarmx_joint_slider_panel/JointSliderPanel`
- `/robot_description`으로부터의 `RobotModel` 표시

본 패키지는 기존 bringup 패키지를 변경하지 않으므로, 기존 시스템에 직접 연결하여 연동 테스트, 시연, 시각화 검증 용도로 적합합니다.

## 시뮬레이션 모드 (OpenArmX)

```bash
source install/setup.bash
ros2 launch openarmx_preview_bringup openarmx.bimanual.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=true
```

## 실기 모드 (OpenArm)

```bash
source install/setup.bash
ros2 launch openarmx_preview_bringup openarmx.bimanual.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=false
```

또한 다음 대체 launch 파일명도 지원합니다.

- `openarmx.preview.bimanual.launch.py`
- `openarm.preview.bimanual.launch.py`



## 라이선스

본 저작물은 Creative Commons 저작자표시-비영리-동일조건변경허락 4.0 국제 라이선스 (CC BY-NC-SA 4.0)에 따라 이용 허락됩니다.

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

자세한 내용은 [LICENSE_kr.md](LICENSE_kr.md) 파일을 참조하시거나 다음 링크를 방문하시기 바랍니다: http://creativecommons.org/licenses/by-nc-sa/4.0/

## 작성자

- **Li QingRan** (李青燃)
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
