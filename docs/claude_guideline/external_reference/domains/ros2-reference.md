# 외부 참조 — ROS2 Add-on (External Reference — ROS2)

> 코어 [handling.md](../handling.md) 의 도메인 sub-file. 코어 §1~12 규칙(특히 §3 source 분리)에 ROS2 특화 taxonomy·경로·인용 형식·추정 사례·grep 을 더한다.

**트리거**: `package.xml`, `rclpy`/`rclcpp` import, `.launch.py`, `rcl_interfaces`, `ament_python`/`ament_cmake` 빌드 타입, REP 인용, sensor driver(livox/velodyne/realsense) 참조.

## 1. 매뉴얼 종류 taxonomy

| 종류 | 다루는 정보 | 예 |
| --- | --- | --- |
| **sensor datasheet** (벤더) | LiDAR/IMU/camera 의 물리 spec(scan rate, FoV, range, accuracy) | `Livox MID-360 DataSheet`, `Bosch BMI088 DataSheet` |
| **벤더 ROS2 driver 문서** | 토픽 이름, frame_id 권장, QoS 권장, 파라미터 default | `livox_ros2_driver` README, `realsense-ros` docs |
| **REP** (Robot Enhancement Proposal) | ROS 표준 명세 — 좌표계, TF tree, IMU frame_id, 단위 | REP-103, REP-105, REP-145 |
| **ROS2 design docs** | DDS/QoS 정책, executor 모델, lifecycle | design.ros2.org |
| **메시지 spec** | `sensor_msgs/PointCloud2`, `sensor_msgs/Imu` 등 정의 | docs.ros.org / interfaces |
| **시뮬레이터 bridge 문서** | Gazebo sensor plugin, `ros_gz_bridge` 토픽 매핑 | gazebosim.org docs |

## 2. 보관 경로

- 벤더 sensor datasheet: `references/<vendor>/<sensor>/` (예: `references/livox/mid-360/`)
- 벤더 driver 문서 사본: `references/<vendor>/<sensor>/driver-readme.md`
- REP / design doc: URL 우선 + `references/standards/ros/rep-<N>/` 사본 (변경 추적용)
- 알고리즘 논문: `references/papers/<topic>/`

## 3. 인용 형식

```
[Livox MID-360 DataSheet v1.2, Table 3, page 8](references/livox/mid-360/datasheet.pdf)
[REP-105, Section "Frame Authority", accessed 2026-05-21](https://www.ros.org/reps/rep-0105.html)
[livox_ros2_driver README v1.0.1, "Parameters" section](references/livox/mid-360/driver-readme.md)
```

## 4. 흔한 추정 단정 사례

대표 위반 시퀀스: 벤더 ROS2 드라이버 default 파라미터를 datasheet spec 으로 비약 → "드라이버 default `frequency` = datasheet TYP scan rate" 단정 → "datasheet 위반" 거짓 결론 → 사용자가 LiDAR datasheet 확인 시 spec 안쪽 정상.

**핵심**: "벤더 ROS2 driver default ≠ sensor datasheet TYP / Min-Max spec ≠ REP / design doc 권장".

기타:

- "REP-105 가 base_link 를 base frame 으로 권장" 단정 (실제 REP 는 `base_link`/`base_footprint` 모두 허용)
- "Gazebo bridge 가 PointCloud2 per-point timestamp 발행" 추정 → 실측 미발행 → SLAM deskew 비활성 → 드리프트
- IMU vendor datasheet 의 mounting axis 를 REP-145 (x=forward, y=left, z=up) 와 일치한다고 단정
- 패키지 README 의 QoS 권장값을 ROS2 design doc spec 으로 비약

## 5. 1차 source 확인 절차

- 센서 datasheet PDF 다운로드 (코어 §11 절차)
- 드라이버 소스 코드 default 값 직접 인용 (file:line)
- 드라이버 README 는 보조 자료 — datasheet 와 별도 검증 항목
- REP / design doc 은 공식 URL 우선, 변경 가능 부분은 `accessed YYYY-MM-DD` 함께
- 시뮬레이터 동작은 `ros2 topic info -v <토픽>` / `ros2 topic echo` 실측 의무

## 6. 자체 점검 grep

```bash
TARGET=<분석 대상 .md>

# ROS2 인용 형식 (sensor datasheet / REP)
grep -oE "\[(Livox|Velodyne|Ouster|Hesai|Bosch|InvenSense)[^]]*DataSheet[^]]*page [0-9]+\]" $TARGET
grep -oE "\[REP-[0-9]+[^]]*\]" $TARGET

# 드라이버 default 인용 시 "datasheet spec 아님" 명시
grep -nE "default|기본값" $TARGET | grep -iE "frequency|rpm|scan_rate|rate" | grep -vE "driver default|datasheet 아님|소스 코드"

# REP / accessed 일자
grep -oE "accessed [0-9]{4}-[0-9]{2}-[0-9]{2}" $TARGET
```
