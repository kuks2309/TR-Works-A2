# 관절 KP/KD 제어 패널

[English](README.md) | [简体中文](README_CN.md) | 한국어

### 개요
`openarmx_kp_kd_panel`은 RViz2 패널 플러그인으로, 슬라이더를 통해 로봇의 8개 관절(7축 매니퓰레이터 + 그리퍼)의 KP(스티프니스)와 KD(댐핑)를 실시간으로 조정합니다. 슬라이더 값은 모터 종류에 따라 자동으로 실제 파라미터 범위로 매핑되며, ROS 2 파라미터 서비스를 통해 하드웨어에 일괄 전송됩니다.

### 주요 특성
- 매니퓰레이터/그리퍼 분리 슬라이더: 매니퓰레이터(Joint 1-7)와 그리퍼(Joint 8)에 각각 독립된 KP/KD 슬라이더와 "기본값 복원" 버튼이 있습니다
- 자동 범위 매핑: RS04/RS03/RS00 모터의 KP/KD 상한값이 자동 환산되어, 비율을 수동으로 계산할 필요가 없습니다
- 다중 제어 모드: 우측 팔, 좌측 팔, 또는 듀얼암을 선택할 수 있으며, 각각 `/openarmx_right_hardware_params`와 `/openarmx_left_hardware_params`에 연결됩니다
- 실시간 상태 피드백: 상태 바에 연결 상태, 시뮬레이션 감지 결과, 적용 성공 여부가 표시됩니다
- 설정 영속화: 슬라이더와 제어 모드가 RViz 설정에 기록되며, RViz를 다시 열면 자동으로 복원됩니다
- 적응형 UI: 패널을 스크롤할 수 있어 작은 해상도에 적합합니다. 버튼/안내 색상으로 상태를 구분합니다

### 모터별 매핑 범위
| 관절 | 모터 종류 | KP 매핑 범위 | KD 매핑 범위 |
| --- | --- | --- | --- |
| Joint 1-2 | RS04 | 0–5000 | 0–100 |
| Joint 3-4 | RS03 | 0–5000 | 0–100 |
| Joint 5-8 | RS00 | 0–500  | 0–5 |

슬라이더 범위: 매니퓰레이터 KP 0–1000, 매니퓰레이터 KD 0–100; 그리퍼 KP 0–1000, 그리퍼 KD 0–100. 패널에는 매핑 후의 실제 값이 실시간으로 표시됩니다.

### 기본값 (패널의 "기본값 복원" 버튼)
- 매니퓰레이터 KP: 10  → RS04/RS03=50.0, RS00=5.0
- 매니퓰레이터 KD: 3   → RS04/RS03=3.0, RS00=0.15
- 그리퍼 KP: 100 → RS00=50.0
- 그리퍼 KD: 50  → RS00=2.5

### 빌드 및 설치
본 패키지는 표준 ROS 2 ament 플러그인이며, `rclcpp`, `rviz_common`, `rviz_rendering`, `pluginlib`, `Qt5 Widgets`에 의존합니다. 워크스페이스에서 다음을 실행합니다.
```bash
colcon build --packages-select openarmx_kp_kd_panel
```
빌드 후 워크스페이스를 source(예: `source install/setup.bash`)하면 RViz에서 플러그인을 로드할 수 있습니다.

### 사용 절차
1) **하드웨어 노드 확인**: 실제 하드웨어 파라미터 노드가 실행 중인지 확인합니다  
   - 우측 팔: `/openarmx_right_hardware_params`  
   - 좌측 팔: `/openarmx_left_hardware_params`  
   패널이 서비스 준비 상태를 감지합니다. 한쪽만 연결된 경우 "부분 연결" 안내가 표시됩니다.
2) **RViz에 패널 추가**: `Panels` → `Add New Panel` → `openarmx_kp_kd_panel` → `KpKdPanel` 선택.
3) **제어 모드 선택**: 우측 팔 / 좌측 팔 / 듀얼암. 모드 전환 시 파라미터 서버를 재확인합니다.
4) **슬라이더 드래그 및 매핑 값 확인**: 상단 라벨에 각 관절의 매핑된 KP/KD 수치가 실시간으로 표시됩니다.
5) **적용**: "모든 관절에 KP/KD 적용" 버튼을 클릭하면 8축 파라미터가 일괄 전송됩니다. 상태 바 색상으로 성공/실패/대기 상태가 안내됩니다.
6) **시뮬레이션 모드 안내**: `fake_hardware`가 감지되면(controller_manager는 있으나 *_hardware_params가 없을 때), 버튼이 자동으로 비활성화되며 KP/KD 사용 불가 안내가 표시됩니다.

### 안전 주의사항
- 진동과 소음을 방지하기 위해 작은 KP/KD 값부터 단계적으로 상향 조정하시기 바랍니다
- 듀얼암 모드에서는 양쪽이 동일한 슬라이더 값을 동시에 전송합니다
- 서비스가 연결되지 않은 경우, 패널이 적용을 차단하고 팝업으로 안내합니다

### 커맨드라인 등가 예시
```bash
# Joint1 KP를 100으로 설정 (우측 팔 예시)
ros2 param set /openarmx_right_hardware_params kp_joint1 100.0

# 우측 팔의 모든 파라미터 조회
ros2 param list /openarmx_right_hardware_params
```

## 작성자

- **Wei Lindong** (魏林栋)
- 회사: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 웹사이트: https://openarmx.com/

## 버전

v1.0.0

## 라이선스

본 저작물은 Creative Commons 저작자표시-비영리-동일조건변경허락 4.0 국제 라이선스 (CC BY-NC-SA 4.0)에 따라 이용 허락됩니다.

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

자세한 내용은 [LICENSE_kr.md](LICENSE) 파일을 참조하시거나 다음 링크를 방문하시기 바랍니다: http://creativecommons.org/licenses/by-nc-sa/4.0/

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
