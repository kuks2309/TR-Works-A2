# OpenARM Xacro 및 v10 설정 변수 설명

본 문서는 `openarmx_arm.xacro`에 등장하는 "변수"(Xacro 매크로 파라미터/속성)가 어디서 오는지, 각각 무엇을 의미하는지, 그리고 `config/arm/v10` 아래의 YAML 설정과의 관계 및 데이터 흐름을 설명합니다.

## 개요: 데이터 흐름과 호출 관계

- 진입 모델: `urdf/robot/v10.urdf.xacro`는 `xacro.load_yaml(...)`을 사용해 v10의 YAML 설정(관절 제한, 키네매틱스, 관성 등)을 읽어, 이 딕셔너리를 파라미터로 상위 매크로 `openarmx_robot`에 전달합니다.
- 로봇 매크로: `urdf/robot/openarmx_robot.xacro`가 다시 매니퓰레이터 매크로 `openarmx_arm`(및 본체/엔드 이펙터 매크로)을 호출하여 해당 설정을 전달합니다.
- 매니퓰레이터 매크로: `urdf/arm/openarmx_arm.xacro` 내부에서 보조 매크로(`openarmx-kinematics`, `openarmx-kinematics-link`, `openarmx-inertials`, `openarmx-limits`)를 통해 YAML의 수치 값을 URDF의 `<origin>`, `<inertial>`, `<limit>` 등의 요소로 작성합니다.

간략 도식:

```
[v10.urdf.xacro]
  ├─ load_yaml(joint_limits.yaml/inertials.yaml/kinematics.yaml/kinematics_link.yaml/kinematics_offset.yaml)
  └─ openarmx_robot(..., joint_limits=..., inertials=..., kinematics=..., kinematics_link=..., kinematics_offset=...)
       └─ openarmx_arm(... 동일한 파라미터명 ...)
            ├─ openarmx-kinematics(kinematics["jointN"].kinematic [+ offset]에서 유래)
            ├─ openarmx-kinematics-link(kinematics_link["linkN"].kinematic에서 유래)
            ├─ openarmx-inertials(inertials["linkN"]에서 유래) 
            └─ openarmx-limits(joint_limits["jointN"].limit에서 유래)
```

## 주요 파일 인덱스 (v10)

- `urdf/robot/v10.urdf.xacro`: 모델 진입점. YAML을 로드하여 `openarmx_robot`에 파라미터를 전달합니다.
- `urdf/robot/openarmx_robot.xacro`: 최상위 로봇 매크로. 매니퓰레이터/본체/엔드 이펙터 매크로를 호출합니다.
- `urdf/arm/openarmx_arm.xacro`: 매니퓰레이터 매크로 (link0~link7, joint1~joint7 정의).
- `urdf/arm/openarmx_macro.xacro`: 공통 매크로 제공 (관성, 제한, 키네매틱스, 비주얼/충돌 정렬 등).
- `config/arm/v10/*.yaml`: 매니퓰레이터 v10의 파라미터 라이브러리.
  - `joint_limits.yaml`, `kinematics.yaml`, `kinematics_link.yaml`, `kinematics_offset.yaml`, `inertials.yaml`.

## openarmx_arm.xacro 매크로 파라미터와 의미

`openarmx_arm`의 주요 파라미터 (상위에서 전달):

- `arm_type`: 매니퓰레이터 모델(예: `v10`). 메시 경로 등에 사용됩니다.
- `arm_prefix`: 매니퓰레이터 접두사(예: `left_`/`right_`/빈 문자열). 링크와 관절의 명명에 사용됩니다 (최종 접두사는 `openarmx_<arm_prefix>`이며, `no_prefix`로 비활성화 가능).
- `no_prefix`: `true`인 경우 `openarmx_` 접두사를 붙이지 않습니다.
- `description_pkg`: 메시 파일이 위치한 패키지 (기본값 `openarmx_description`).
- `connected_to`, `xyz`, `rpy`: 전체 매니퓰레이터를 고정 관절로 부모 링크(예: 본체)에 연결하고, 조립 포즈를 설정합니다.
- `joint_limits`: `joint_limits.yaml`에서 유래한 딕셔너리. `<limit>`(각도 상/하한, 속도, 토크)에 사용됩니다.
- `inertials`: `inertials.yaml`에서 유래한 딕셔너리. `<inertial>`(질량 중심, 질량, 관성 텐서)에 사용됩니다.
- `kinematics`: `kinematics.yaml`에서 유래한 딕셔너리. `<joint>/<origin>`(부모-자식 간 명목 형상)에 사용됩니다.
- `kinematics_link`: `kinematics_link.yaml`에서 유래한 딕셔너리. `<visual>/<collision>/<origin>`(메시 정렬)에 사용됩니다.
- `kinematics_offset`: `kinematics_offset.yaml`에서 유래한 딕셔너리. 선택적 추가 오프셋으로, `kinematics`에 누적 적용됩니다 (듀얼암 조립 또는 캘리브레이션에 주로 사용).

추가 내부 변수:

- `reflect`: `arm_prefix`에 따라 자동 설정되는 미러 계수 (우측 팔 `+1`, 좌측 팔 `-1`). 대칭 Y축 스케일링/일부 자세 및 축 방향에 사용되며, 좌/우 팔의 형상/관성/축 방향이 미러링되도록 보장합니다.
- `limit_offset_joint2`: 듀얼암의 경우 `joint2` 제한의 오프셋 (우측 팔 `+π/2`, 좌측 팔 `-π/2`). 듀얼암 조립 각도가 적절해지도록 합니다.

## config/arm/v10 아래 YAML의 필드와 용도

- `joint_limits.yaml` (관절 제한 및 성능)
  - `jointN.limit.lower/upper`: 각도 상/하한 (라디안).
  - `jointN.limit.velocity`: 최대 각속도 (rad/s).
  - `jointN.limit.effort`: 최대 토크/힘 (Nm).
  - Xacro에서 `openarmx-limits` 매크로를 통해 `<limit>`에 작성되며, `reflect`와 (선택적으로) `offset`(예: joint2의 좌우 오프셋)을 고려합니다.

- `kinematics.yaml` (관절 명목 형상)
  - `jointN.kinematic.x/y/z/roll/pitch/yaw`: 부모 link에서 해당 관절 원점까지의 포즈 (미터/라디안).
  - Xacro에서 `openarmx-kinematics` 매크로를 통해 각 `<joint>/<origin>`에 작성됩니다. `kinematics_offset`이 제공된 경우 `offset`을 `kinematics`에 더한 후 적용됩니다.

- `kinematics_link.yaml` (link 메시 정렬)
  - `linkN.kinematic.x/y/z/roll/pitch/yaw`: `<visual>/<collision>`의 `<origin>`에만 사용됩니다. CAD 메시와 link 좌표계의 정렬을 보장하며, 관절 토폴로지는 변경하지 않습니다.
  - Xacro에서 `openarmx-kinematics-link` 매크로를 통해 각 link의 비주얼과 충돌 형상에 적용됩니다.

- `kinematics_offset.yaml` (명목 형상 추가 보정)
  - `jointN.kinematic_offset.x/y/z/roll/pitch/yaw`: 필요 시 `kinematics`에 누적 적용됩니다 (예: v10 듀얼암의 `joint2.roll ≈ π/2`).

- `inertials.yaml` (각 link의 관성 속성)
  - `linkN.origin.x/y/z/roll/pitch/yaw`: link 원점 기준 질량 중심의 포즈.
  - `linkN.mass`: 질량 (kg).
  - `linkN.inertia.xx/xy/xz/yy/yz/zz`: 관성 텐서 요소.
  - Xacro에서 `openarmx-inertials` 매크로를 통해 `<inertial>`에 작성되며, Y 방향은 `reflect`에 따라 미러 처리됩니다.

## 좌/우 팔 미러링과 듀얼암 차이

- 미러링 (`reflect`): 우측 팔은 `+1`, 좌측 팔은 `-1`. 영향:
  - 비주얼/충돌 메시 스케일의 Y축 부호 (메시 미러링).
  - 관성에서 질량 중심 `origin.y`의 부호.
  - 일부 관절 축 방향과 말단 관절의 축 회전 방향 (예: joint7의 `axis xyz="0 ±1 0"`).
- 듀얼암 오프셋:
  - `joint2`에 `limit_offset_joint2` (±π/2)를 적용하여 좌/우 팔의 조립 포즈 범위를 구분합니다.
  - 듀얼암 모드에서는 일반적으로 `kinematics_offset`도 함께 전달하여 특정 관절 포즈에 고정 회전을 추가합니다 (예: `joint2.roll`).

## 수정 및 검증 권장사항

- 수정 위치: 소스 패키지 아래에서 편집하는 것을 우선 권장합니다 (`install/` 디렉터리가 아닌).
  - YAML: `src/openarmx_description/config/arm/v10/*.yaml`
  - Xacro: `src/openarmx_description/urdf/**/*.xacro`
- 빌드 및 확인:
  - 빌드: `colcon build --symlink-install`
  - 환경: `source install/setup.bash`
  - RViz 표시: `ros2 launch openarmx_description display_openarmx.launch.py arm_type:=v10 ee_type:=openarmx_hand bimanual:=false`
- 작은 팁:
  - 메시 정렬만 조정하려면 `kinematics_link.yaml`을 수정합니다.
  - 관절 영점/링크 길이를 조정하려면 `kinematics.yaml`을 수정합니다 (필요 시 `kinematics_offset.yaml`과 함께 미세 캘리브레이션).
  - 하중 및 동역학을 조정하려면 `inertials.yaml`을 수정합니다.
  - 제한/속도/토크 성능을 조정하려면 `joint_limits.yaml`을 수정합니다.

## 참고 (주요 구현 위치)

- 진입과 로딩: `urdf/robot/v10.urdf.xacro`
- 로봇 매크로: `urdf/robot/openarmx_robot.xacro`
- 매니퓰레이터 매크로: `urdf/arm/openarmx_arm.xacro`
- 공통 매크로: `urdf/arm/openarmx_macro.xacro`
- 파라미터 라이브러리: `config/arm/v10/*.yaml`


## 코드 위치와 라인 번호 (빠른 위치 확인용)

- `urdf/arm/openarmx_arm.xacro`
  - `urdf/arm/openarmx_arm.xacro:6` `openarmx_arm` 매크로 파라미터 목록 정의.
  - `urdf/arm/openarmx_arm.xacro:8` 명명 접두사 `prefix` 계산 (`arm_prefix`와 `no_prefix` 결합).
  - `urdf/arm/openarmx_arm.xacro:10` 우측 팔 `reflect=+1`; `urdf/arm/openarmx_arm.xacro:14` 좌측 팔 `reflect=-1`.
  - `urdf/arm/openarmx_arm.xacro:18` 부모 링크에 마운트하는 경우 고정 관절 생성; `urdf/arm/openarmx_arm.xacro:22` 해당 고정 관절의 조립 포즈 `xyz/rpy` 설정.
  - `urdf/arm/openarmx_arm.xacro:26` link0 비주얼/충돌/관성 생성; `urdf/arm/openarmx_arm.xacro:28` link1 동일.
  - `urdf/arm/openarmx_arm.xacro:31` `joint1`은 `kinematics`로 `<origin>` 작성; `urdf/arm/openarmx_arm.xacro:35` `joint1` 제한 (오프셋 지원).
  - `urdf/arm/openarmx_arm.xacro:41` `limit_offset_joint2` 기본값 0; `urdf/arm/openarmx_arm.xacro:44` 우측 팔은 `+pi/2`로 설정; `urdf/arm/openarmx_arm.xacro:48` 좌측 팔은 `-pi/2`로 설정.
  - `urdf/arm/openarmx_arm.xacro:52` `joint2`는 `kinematics_offset`과 `reflect`를 참조; `urdf/arm/openarmx_arm.xacro:56` `joint2` 제한에 좌우 오프셋 누적.
  - `urdf/arm/openarmx_arm.xacro:62`/`72`/`82`/`92`/`102` 각 관절은 `kinematics`로 `<origin>` 작성 (예: `joint3`은 `:62`).
  - `urdf/arm/openarmx_arm.xacro:105` `joint7`의 축 방향은 `reflect` 사용 (`axis xyz="0 ±1 0"`).

- `urdf/arm/openarmx_macro.xacro`
  - `urdf/arm/openarmx_macro.xacro:4` 매크로 `openarmx-inertials` 정의; `urdf/arm/openarmx_macro.xacro:12` `<inertial>/<origin>`에서 `y`에 `reflect` 곱셈; `urdf/arm/openarmx_macro.xacro:17` `<inertia>` 요소.
  - `urdf/arm/openarmx_macro.xacro:24` 매크로 `link_with_sc`; `urdf/arm/openarmx_macro.xacro:31` 비주얼 메시 경로와 스케일 (`reflect` 포함); `urdf/arm/openarmx_macro.xacro:37` 충돌 메시 경로; `urdf/arm/openarmx_macro.xacro:41` `openarmx-inertials` 호출.
  - `urdf/arm/openarmx_macro.xacro:91` 매크로 `openarmx-limits`; `urdf/arm/openarmx_macro.xacro:92` `limits` 읽기; `urdf/arm/openarmx_macro.xacro:109` `<limit>` 출력.
  - `urdf/arm/openarmx_macro.xacro:116` 매크로 `openarmx-kinematics`; `urdf/arm/openarmx_macro.xacro:121` offset 포함 `<origin>`; `urdf/arm/openarmx_macro.xacro:127` offset 없는 `<origin>`.
  - `urdf/arm/openarmx_macro.xacro:131` 매크로 `openarmx-kinematics-link`; `urdf/arm/openarmx_macro.xacro:133` link의 `<origin>`.

- `urdf/robot/v10.urdf.xacro`
  - `urdf/robot/v10.urdf.xacro:36` `joint_limits.yaml` 로드; `urdf/robot/v10.urdf.xacro:37` `inertials.yaml` 로드; `urdf/robot/v10.urdf.xacro:38` `kinematics.yaml` 로드; `urdf/robot/v10.urdf.xacro:39` `kinematics_link.yaml` 로드; `urdf/robot/v10.urdf.xacro:40` `kinematics_offset.yaml` 로드.

- `urdf/robot/openarmx_robot.xacro`
  - `urdf/robot/openarmx_robot.xacro:73` 좌측 팔이 `openarmx_arm` 호출; `urdf/robot/openarmx_robot.xacro:85` `kinematics_offset` 명시적 전달.
  - `urdf/robot/openarmx_robot.xacro:88` 우측 팔이 `openarmx_arm` 호출; `urdf/robot/openarmx_robot.xacro:100` `kinematics_offset` 명시적 전달.
  - `urdf/robot/openarmx_robot.xacro:149` 단일 팔이 `openarmx_arm` 호출; `urdf/robot/openarmx_robot.xacro:156`/`157`/`158` 각각 `kinematics`/`kinematics_link`/`inertials` 전달 (`kinematics_offset` 미전달).

## 관절 축 방향 개요 (openarmx_arm.xacro 기준)

```
joint1: axis = 0 0 1
joint2: axis = -1 0 0
joint3: axis = 0 0 1
joint4: axis = 0 1 0
joint5: axis = 0 0 1
joint6: axis = 1 0 0
joint7: axis = 0 (reflect) 0   # reflect: 우측 팔 +1, 좌측 팔 -1
```
