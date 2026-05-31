# ROS2 / 임베디드 작업 규칙

ROS2 워크스페이스 + 임베디드 / 외부 드라이버 결합 환경에 적용되는 도메인 SSOT. ROS2 프로젝트에서는 다른 가이드라인보다 먼저 읽는다.

프로젝트 고유 값(빌드 옵션 변형, 네트워크 설정, 패키지 경로, IP 등)은 본 문서가 아닌 워크스페이스 루트 `CLAUDE.md` 또는 [`local/`](local/) 의 override 문서에 둔다.

## 빌드 명령

- 워크스페이스 루트에서 `colcon build` 사용. `--symlink-install` 옵션은 src 원본 사용 규칙과 직결되므로 임의 제거 금지.
- 단일 패키지 빌드: `colcon build --packages-select <pkg> --symlink-install`.
- 빌드 후 반드시 `source install/setup.bash`. 새 패키지 추가 / 노드명 변경 시 재 source 필수.
- 프로젝트별 추가 cmake 인자(예: `-DROS_EDITION=...`, `-DHUMBLE_ROS=...`)는 워크스페이스 루트 `CLAUDE.md` 에서 정의한다.

## 설정 파일은 src 원본 사용

- rviz, yaml, JSON 등 설정 파일은 `install/share/...` 에 복사하지 않는다. **항상 src 폴더 원본을 수정**한다.
- launch 에서 경로 참조 시 `--symlink-install` 의존 또는 `get_package_share_directory()` 결과가 src 원본을 가리키도록 유지.

## COLCON_IGNORE 정책

- 의도적으로 빌드 제외된 디렉토리(ROS1 잔존, 외부 도구 빌드 트리, 미완성 패키지 등)는 IGNORE 제거 금지.
- 외부 빌드 시스템(예: PlatformIO 의 `.pio/`)은 colcon 빌드 대상이 아니다.

## 외부 드라이버 read-only 보호

- 외부 공식 저장소에서 가져온 vendored 코드는 수정 금지, config / launch overlay 만 변경 허용.
- 시스템 설치 라이브러리(`/usr/local/lib` 등)는 재컴파일·교체 금지.
- 수정이 불가피한 경우 wrapper 패키지 또는 launch overlay 로 처리.
- 구체 경로 목록은 [github.md](github.md) "Read-only 외부 vendored 저장소 가드" 와 워크스페이스 루트 `CLAUDE.md` / [`local/`](local/) 에서 정의한다.

## 임베디드 / 시리얼 함정 (사전 경고)

다음은 재현 빈도 높은 함정으로, 작업 시작 전 검토한다.

1. **시리얼 포트 점유 충돌**: `pio device monitor`, `cat /dev/ttyACM0`, 시리얼 브리지 노드 등이 동시에 같은 장치를 열 수 없다. 플래시 / 모니터 / 브리지 중 하나만 활성화한다 ([workflow.md](workflow.md) "펌웨어 다운로드 절차").
2. **장치 권한**: `/dev/ttyUSB*`, `/dev/ttyACM*` 접근 권한. `dialout` 그룹 가입 또는 udev 규칙 필요.
3. **Baud rate 불일치**: 펌웨어 빌드 설정과 호스트 측 시리얼 파라미터의 일치 필수. 프로젝트별 baud / framing 은 모듈 CLAUDE.md 에 명시한다.
4. **리셋 / 재연결 순서**: USB serial 재연결 시점에 보드 reset loop 또는 silent crash 가 발생할 수 있음. 호스트 측 자동 reconnect 동작과 충돌하지 않도록 수동 reset 시 호스트 노드 stop 권장.
5. **빌드 환경 함정**: PlatformIO `pio run` 만 입력하면 `[platformio]` 의 `default_envs` 가 빌드된다. 양산 펌웨어는 반드시 `pio run -e <env_name> -t upload` 로 명시한다.

## 패키지 종류별 주의사항

| 패키지 종류 | 빌드 시스템 | 주의 |
|---|---|---|
| C++ | `ament_cmake` | `CMakeLists.txt` 의 `install(TARGETS ...)` 누락 시 설치 안 됨 |
| Python | `ament_python` | `setup.py` 의 `entry_points` 와 `package.xml` 동기화 필수 |
| 임베디드 | 외부 (예: PlatformIO) | colcon 영향 없음. 별도 빌드 사이클 |

## 모듈 CLAUDE.md 와의 관계

루트 / 본 가이드라인은 **워크스페이스 공통**(빌드 / launch / 토픽 / QoS / source / 외부 드라이버 경계). 모듈 CLAUDE.md 는 **하드웨어 핀맵·상수·외부 빌드 명령** 등 모듈 고유 규칙. 충돌 시 모듈 우선.
