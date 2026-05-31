# OpenArmX 관절 슬라이더 패널

## 패키지 소개

`openarmx_joint_slider_panel`은 RViz2 패널 플러그인으로, 슬라이더를 통해 OpenArmX 듀얼암 로봇의 관절과 그리퍼를 직접 제어합니다.

본 플러그인은 디버깅, 시연, 빠른 연동 테스트 시나리오를 대상으로 합니다. RViz2 내에서 듀얼암 관절과 그리퍼의 목표값을 조정할 수 있으며, 분할 실행 메커니즘을 통해 큰 폭의 목표값 변화를 작은 스텝 명령으로 분해하여 보다 부드럽고 제어 가능한 모션을 얻을 수 있습니다.

## 기능 특성

- 16개의 슬라이더를 제공합니다: 좌/우 매니퓰레이터 관절(7 + 7) 및 좌/우 그리퍼.
- 패널이 시작되면 자동으로 한 번 `/joint_states` 동기화를 수행합니다.
- `Sync From /joint_states`를 지원하여, 언제든지 현재 로봇 자세를 기준으로 슬라이더 값을 다시 초기화할 수 있습니다.
- 슬라이더를 드래그하면 곧바로 명령이 전송되며, 별도의 "적용" 버튼을 누를 필요가 없습니다.
- 백그라운드 스레드 분할 실행을 지원합니다.
  - `Joint Step`: 사이클당 매니퓰레이터의 최대 스텝(mrad/cycle)을 제어합니다.
  - `Gripper Step`: 사이클당 그리퍼의 최대 스텝(mm/cycle)을 제어합니다.
  - 큰 폭의 슬라이더 점프는 자동으로 여러 회의 작은 스텝 명령으로 분할됩니다.
- 프리뷰 모델은 활성화되지 않습니다 (Preview model disabled).
- `Home` 버튼을 누르면 듀얼암과 그리퍼의 목표값을 0으로 설정하고, 분할 모드로 홈 포지션 복귀를 실행합니다.
- 전방향 위치 제어(forward position) 백엔드만 지원합니다.
  - `/left_forward_position_controller/commands`
  - `/right_forward_position_controller/commands`

## 빌드

```bash
cd ~/openarmx_ws2
colcon build --packages-select openarmx_joint_slider_panel
source install/setup.bash
```

## RViz2 사용 방법

1. 로봇 시스템을 실행합니다 (`demo.launch.py` / `demo_sim.launch.py` / bringup).
2. RViz2를 엽니다.
3. `Panels` -> `Add New Panel` -> `openarmx_joint_slider_panel/JointSliderPanel`로 진입합니다.
4. `Sync From /joint_states`를 클릭합니다.
5. 원하는 부드러움 정도에 따라 `Joint Step`과 `Gripper Step`을 설정합니다.
6. 슬라이더를 드래그하면, 로봇이 분할 명령에 따라 직접 모션을 실행합니다.
7. `Home` 버튼을 클릭하면 분할 방식으로 홈 포지션 복귀를 실행합니다.

**또는 openarmx_preview_bringup을 사용해 원클릭으로 실행하실 수 있습니다.**

## 일반적인 컨트롤러 토픽 매핑

- `/left_forward_position_controller/commands`
- `/right_forward_position_controller/commands`


## 라이선스

본 저작물은 Creative Commons 저작자표시-비영리-동일조건변경허락 4.0 국제 라이선스 (CC BY-NC-SA 4.0)에 따라 이용 허락됩니다.

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

자세한 내용은 [LICENSE_kr.md](LICENSE_kr.md) 파일을 참조하시거나 다음 링크를 방문하시기 바랍니다: http://creativecommons.org/licenses/by-nc-sa/4.0/

## 작성자

- **Li QingRan** (李青燃)
- 회사: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 웹사이트: https://openarmx.com/

## 버전

1.0.0

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
