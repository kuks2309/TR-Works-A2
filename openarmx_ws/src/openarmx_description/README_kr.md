# OpenArmX Description

[English](README.md) | [简体中文](README_CN.md) | 한국어

[![ROS 2](https://img.shields.io/badge/ROS-2-blue.svg)](https://docs.ros.org/en/humble/index.html)

OpenArmX 로봇 플랫폼의 완전한 URDF 기술 패키지로, ROS 2 시뮬레이션과 제어를 위한 상세한 키네매틱스, 다이내믹스, 시각화 모델을 제공합니다.

## 개요

`openarmx_description` 패키지는 OpenArmX 로봇의 완전한 기계 구조, 키네매틱스, 다이내믹스, 시각적 표현을 정의하는 URDF(Unified Robot Description Format) 파일을 포함합니다. 본 패키지는 ROS 2 환경에서 로봇 시각화, 모션 플래닝, 시뮬레이션 및 하드웨어 제어의 기반이 됩니다.

## 기능 특성

- **완전한 로봇 모델**: 정확한 키네매틱스와 다이내믹스 파라미터를 갖춘 OpenArmX 로봇의 완전한 URDF 기술
- **모듈러 아키텍처**: 매니퓰레이터, 본체, 엔드 이펙터 컴포넌트의 독립된 기술로 유연한 로봇 구성 지원
- **충돌 형상**: 안전한 모션 플래닝과 장애물 회피를 위한 상세한 충돌 메시
- **시각화 모델**: 사실적인 3D 시각화를 위한 고품질 STL/DAE 메시
- **ROS 2 제어 통합**: 시뮬레이션과 실제 하드웨어를 위해 사전 구성된 ros2_control 하드웨어 인터페이스
- **듀얼암 지원**: 듀얼암 로봇 구성을 기본 지원
- **다양한 로봇 변형**: 서로 다른 매니퓰레이터 타입(v10)과 엔드 이펙터(OpenArmX Hand) 지원
- **파라미터화된 설정**: 키네매틱스, 다이내믹스, 관절 제한을 위한 YAML 기반 설정 파일

## 패키지 구조

```
openarmx_description/
├── CMakeLists.txt              # CMake 빌드 설정
├── package.xml                 # ROS 2 패키지 매니페스트
├── LICENSE                     # Apache 2.0 라이선스
├── config/                     # 로봇 파라미터 설정
│   ├── arm/                    # 매니퓰레이터별 파라미터
│   │   └── v10/                # v10 매니퓰레이터 설정 파일
│   │       ├── inertials.yaml          # 링크 질량 및 관성 속성
│   │       ├── joint_limits.yaml       # 관절 위치/속도/토크 제한
│   │       ├── kinematics.yaml         # DH 파라미터 및 변환
│   │       ├── kinematics_link.yaml    # 링크 좌표계 정의
│   │       └── kinematics_offset.yaml  # 관절 영점 오프셋 캘리브레이션
│   ├── body/                   # 본체/몸체 파라미터
│   │   └── v10/                # v10 본체 설정
│   └── hand/                   # 엔드 이펙터 파라미터
│       └── openarmx_hand/      # OpenArmX Hand 그리퍼 설정
├── launch/                     # 시각화 launch 파일
│   └── display_openarmx.launch.py  # RViz에서 로봇 시각화 실행
├── meshes/                     # 3D 메시 파일 (STL/DAE)
│   ├── arm/                    # 매니퓰레이터 링크 메시
│   │   └── v10/
│   │       ├── collision/      # 단순화된 충돌 형상
│   │       └── visual/         # 상세 시각화 메시
│   ├── body/                   # 본체/몸체 메시
│   │   └── v10/
│   │       ├── collision/
│   │       └── visual/
│   └── ee/                     # 엔드 이펙터 메시
│       └── openarmx_hand/
│           ├── collision/
│           └── visual/
├── rviz/                       # RViz 설정 파일
│   ├── arm_only.rviz          # 단일 팔 시각화 설정
│   └── bimanual.rviz          # 듀얼암 시각화 설정
└── urdf/                       # URDF/Xacro 기술 파일
    ├── arm/                    # 매니퓰레이터 URDF 컴포넌트
    │   ├── openarmx_arm.xacro        # 메인 매니퓰레이터 기술
    │   └── openarmx_macro.xacro      # 매니퓰레이터 xacro 매크로
    ├── body/                   # 본체/몸체 URDF 컴포넌트
    │   ├── openarmx_body.xacro       # 메인 본체 기술
    │   └── openarmx_body_macro.xacro # 본체 xacro 매크로
    ├── ee/                     # 엔드 이펙터 URDF 컴포넌트
    │   ├── openarmx_hand.xacro       # OpenArmX Hand 기술
    │   ├── openarmx_hand_macro.xacro # Hand xacro 매크로
    │   ├── openarmx_hand_arguments.xacro  # Hand 파라미터
    │   └── ee_with_one_link.xacro    # 범용 엔드 이펙터 연결
    ├── robot/                  # 전체 로봇 어셈블리
    │   ├── openarmx_robot.xacro      # 범용 로봇 매크로
    │   ├── v10.urdf.xacro            # v10 로봇 변형
    │   └── openarmx_bimanual_sim.urdf # 사전 생성된 듀얼암 URDF
    └── ros2_control/           # ROS 2 제어 설정
        ├── openarmx.ros2_control.xacro         # 단일 팔 제어 설정
        └── openarmx.bimanual.ros2_control.xacro # 듀얼암 제어 설정
```

## 설치

### 사전 요구사항

- ROS 2 (Humble 이상)
- Python 3.10+
- `xacro` 패키지
- `joint_state_publisher_gui` 패키지
- `rviz2` 패키지

### 소스에서 빌드

```bash
# ROS 2 워크스페이스로 이동
cd ~/openarmx_ws/src

# 저장소 클론
git clone https://github.com/openarmx-arm/openarmx_description.git

# 패키지 빌드
colcon build --packages-select openarmx_description

# 환경 설정
source install/setup.bash
```

## 사용 방법

### RViz에서 로봇 시각화

기본 설정으로 로봇 시각화를 실행합니다.

```bash
ros2 launch openarmx_description display_openarmx.launch.py arm_type:=v10 bimanual:=true
```

#### 실행 파라미터

- `arm_type` (필수): 시각화할 매니퓰레이터 타입
  - `v10`: 7 자유도 OpenArmX v10 매니퓰레이터

- `ee_type` (기본값: `openarmx_hand`): 엔드 이펙터 타입
  - `openarmx_hand`: OpenArmX Hand 그리퍼
  - `none`: 엔드 이펙터 없음

- `bimanual` (필수: `false`): 듀얼암 설정 활성화
  - `true`: 두 매니퓰레이터를 갖는 듀얼암 로봇 로드
  - `false`: 단일 매니퓰레이터만 로드


### Xacro에서 URDF 생성

검사를 위해 xacro 파일을 URDF로 변환합니다.

```bash
xacro $(ros2 pkg prefix openarmx_description)/share/openarmx_description/urdf/robot/v10.urdf.xacro \
    arm_type:=v10 ee_type:=openarmx_hand bimanual:=true > /home/openarmx/openarmx/src/openarmx_description/urdf/robot/openarmx_robot.urdf
```

## 지원하는 로봇 구성

### 매니퓰레이터 타입

| 타입 | 자유도 | 설명 |
|------|--------|------|
| v10  | 7      | OpenArmX v10 - 7 자유도 협동 매니퓰레이터, 하중 5kg |

### 엔드 이펙터

| 타입 | 설명 |
|------|------|
| openarmx_hand | 위치 제어를 갖춘 평행 그리퍼 |
| none | 엔드 이펙터 없음 (플랜지만) |

### 구성 모드

- **단일 팔**: 하나의 매니퓰레이터에 옵션 엔드 이펙터
- **듀얼암**: 동기 제어를 갖는 듀얼암 시스템

## 설정 파일

본 패키지는 로봇 파라미터를 정의하기 위해 YAML 설정 파일을 사용합니다.

- **`inertials.yaml`**: 각 링크의 질량, 질량 중심, 관성 텐서
- **`joint_limits.yaml`**: 각 관절의 위치, 속도, 토크 제한
- **`kinematics.yaml`**: 전방 키네매틱스 파라미터 (DH 규약)
- **`kinematics_link.yaml`**: 링크 간 변환 정의
- **`kinematics_offset.yaml`**: 관절 영점 캘리브레이션 오프셋

## ROS 2 제어 통합

본 패키지는 사전 구성된 ros2_control 하드웨어 인터페이스를 포함합니다.

- **위치 컨트롤러**: 관절 궤적 제어
- **속도 컨트롤러**: 직접 속도 명령
- **토크 컨트롤러**: 토크 기반 제어
- **그리퍼 컨트롤러**: 엔드 이펙터 제어

시뮬레이션의 경우 `use_fake_hardware:=true`, 실제 하드웨어 제어의 경우 `use_fake_hardware:=false`를 사용합니다.

## 개발

### 새로운 로봇 변형 추가

1. 설정 디렉터리 생성: `config/arm/your_variant/`
2. 필요한 YAML 파일 추가: `inertials.yaml`, `joint_limits.yaml` 등
3. 메시 생성: `meshes/arm/your_variant/{collision,visual}/`
4. xacro 파일 생성: `urdf/robot/your_variant.urdf.xacro`
5. 새 변형을 지원하도록 launch 파일 업데이트

### 로봇 파라미터 수정

`config/` 디렉터리의 YAML 파일을 편집하여 로봇 파라미터를 조정할 수 있습니다. 수정 후 워크스페이스를 다시 빌드합니다.

```bash
colcon build --packages-select openarmx_description
```

## 트러블슈팅

### RViz에 로봇 모델이 표시되지 않는 경우
- 패키지의 환경이 올바르게 설정되어 있는지 확인합니다: `source install/setup.bash`
- `arm_type` 파라미터가 사용 가능한 설정과 일치하는지 확인합니다
- URDF 생성을 검증합니다: `ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro path/to/file.xacro)"`

### 메시 파일이 로드되지 않는 경우
- xacro 파일의 메시 경로가 `package://` URI 스킴을 사용하는지 확인합니다
- 메시가 설치되어 있는지 확인합니다: `install/openarmx_description/share/openarmx_description/meshes/`

### 시뮬레이션에서 관절 제한이 위반되는 경우
- `config/arm/*/joint_limits.yaml`의 제한을 확인하고 조정합니다
- 컨트롤러가 설정에서 관절 제한을 준수하는지 확인합니다

## 기여

기여를 환영합니다! 다음 가이드라인을 따라주시기 바랍니다.

1. 저장소를 포크
2. 기능 브랜치 생성
3. 명확한 커밋 메시지로 변경
4. 다양한 구성으로 충분히 테스트
5. Pull Request 제출

## 라이선스

본 저작물은 Creative Commons 저작자표시-비영리-동일조건변경허락 4.0 국제 라이선스 (CC BY-NC-SA 4.0)에 따라 이용 허락됩니다.

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

자세한 내용은 [LICENSE_kr.md](LICENSE) 파일을 참조하시거나 다음 링크를 방문하시기 바랍니다: http://creativecommons.org/licenses/by-nc-sa/4.0/

## 작성자

- **Zhang Li** (张力)
- 회사: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 웹사이트: https://openarmx.com/

## 버전

**현재 버전**: 6.0.0

## 감사의 말

본 패키지는 OpenArmX 로봇 플랫폼 생태계의 일부이며, 협동 로봇 분야의 연구 및 산업 응용을 위해 개발되었습니다.

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
