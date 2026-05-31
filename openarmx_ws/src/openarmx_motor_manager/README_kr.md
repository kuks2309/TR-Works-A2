# OpenArmX Motor Manager

[English](README.md) | [简体中文](README_CN.md) | 한국어

OpenArmX 다중 로봇 모터 관리 시스템 - PySide6 기반의 그래픽 듀얼암 로봇 제어 도구입니다.

## 개요

`openarmx_motor_manager`는 OpenArmX 듀얼암 로봇을 관리하고 제어하기 위한 데스크톱 애플리케이션입니다. 본 시스템은 여러 로봇의 동시 관리를 지원하며, 모터 제어, 상태 모니터링, CAN 인터페이스 관리 등의 작업을 수행하기 위한 직관적인 그래픽 인터페이스를 제공합니다.

## 기능 특성

### 다중 로봇 관리
- 여러 듀얼암 로봇의 동시 연결 및 관리 지원
- 각 로봇을 독립된 탭으로 표시
- CAN 인터페이스 자동 감지 및 페어링 (can0-can1, can2-can3, ...)
- CAN 채널 수동 설정 지원

### CAN 인터페이스 관리
- 모든 CAN 인터페이스의 원클릭 활성화/비활성화
- 실제 CAN 인터페이스 자동 감지 (가상 인터페이스 필터링)
- CAN 인터페이스 상태와 비트레이트 조회
- sudo 비밀번호 자동 입력 지원

### 모터 제어
- **모든 모터 활성화** - 듀얼암의 모든 모터 일괄 활성화
- **모든 모터 비활성화** - 듀얼암의 모든 모터 일괄 정지
- **홈 포지션 복귀** - 모든 모터를 영점 위치로 복귀
- **영점 설정** - 현재 위치를 모터 영점으로 설정
- **단일 모터 테스트** - MIT 모드에서 단일 모터 정밀 제어
- **모든 모터 테스트** - 간단한 모션 테스트 실행

### 모터 상태 모니터링
- 모터 상태 실시간 표시 (위치, 속도, 토크, 온도)
- 모드 상태 표시 (Motor 모드/Reset 모드/Cali 모드)
- 결함 상태 모니터링

### 다국어 지원
- 중국어 (zh_CN)
- English (en_US)
- 日本語 (ja_JP)
- Русский (ru_RU)

## 프로젝트 구조

```
openarmx_motor_manager/
├── GUI_MultiRobot.py          # 프로그램 진입점
├── __init__.py                # 패키지 초기화
├── requirements.txt           # 의존성 목록
├── config/
│   ├── config.yaml            # 설정 파일
│   ├── config_manager.py      # 설정 매니저
│   └── script_finder.py       # 스크립트 파인더
├── ui/
│   ├── MainUI_MultiRobot.py   # 메인 인터페이스
│   ├── RobotPage.py           # 로봇 제어 페이지
│   ├── RobotWorker.py         # 워커 스레드
│   ├── SingleMotorTestDialog.py  # 단일 모터 테스트 다이얼로그
│   ├── SettingsDialog.py      # 설정 다이얼로그
│   ├── ConfigDialog.py        # 구성 다이얼로그
│   ├── translations.yaml      # 다국어 번역 파일
│   ├── ui/                    # Qt Designer UI 파일
│   │   ├── MainUI.ui
│   │   ├── TestMotorUI.ui
│   │   ├── ui_MainUI.py
│   │   └── ui_TestMotorUI.py
│   └── texture/               # 아이콘 리소스
│       ├── icon.ico
│       └── icon.png
├── utils/
│   └── can_detector.py        # CAN 인터페이스 디텍터
└── scripts/                   # 커맨드라인 스크립트
    ├── en_all_can.py          # 모든 CAN 인터페이스 활성화
    ├── dis_all_can.py         # 모든 CAN 인터페이스 비활성화
    ├── en_all_motors.py       # 모든 모터 활성화
    ├── dis_all_motors.py      # 모든 모터 정지
    ├── check_motor_status.py  # 모터 상태 확인
    ├── control_motor_gohome.py  # 모터 홈 포지션 복귀
    ├── set_motor_zero.py      # 영점 설정
    ├── test_motor_one_CSP.py  # 단일 모터 CSP 모드 테스트
    ├── test_motor_one_MIT.py  # 단일 모터 MIT 모드 테스트
    ├── test_motor_one_by_one.py  # 모터 순차 테스트
    └── test_motor_all_random.py  # 모든 모터 랜덤 테스트
```

## 설치

### 의존성

```bash
pip install -r requirements.txt
```

주요 의존성:
- PySide6 >= 6.5.0
- PyYAML >= 6.0
- openarmx_arm_driver >= 1.1.5
- python-can >= 4.0.0

### 시스템 요구사항
- Linux 운영체제 (CAN 인터페이스 지원 필요)
- Python 3.8+
- CAN 하드웨어 장치 (예: USB-CAN 어댑터)

## 사용 방법

### GUI 애플리케이션 실행

```bash
cd /path/to/openarmx_motor_manager
python3 GUI_MultiRobot.py
```

### 빠른 시작

1. **CAN 인터페이스 활성화**
   - 메뉴바 → CAN → CAN 인터페이스 활성화
   - 최초 사용 시 sudo 비밀번호 입력 필요

2. **로봇 추가**
   - 메뉴바 → 로봇 → 로봇 추가
   - 자동 설정 또는 CAN 채널 수동 설정 선택
   - 듀얼암 로봇 하나를 제어하려면 최소 2개의 CAN 인터페이스가 필요합니다

3. **모터 제어**
   - 로봇 페이지에서 모터 제어 버튼 사용
   - 출력 영역에서 작업 결과 확인

### 커맨드라인 스크립트

커맨드라인 스크립트를 직접 사용하여 작업을 수행할 수도 있습니다.

```bash
# 모든 CAN 인터페이스 활성화
python scripts/en_all_can.py

# 모든 모터 활성화
python scripts/en_all_motors.py

# 모터 상태 확인
python scripts/check_motor_status.py

# 모터 홈 포지션 복귀
python scripts/control_motor_gohome.py

# 모든 모터 정지
python scripts/dis_all_motors.py

# 모든 CAN 인터페이스 비활성화
python scripts/dis_all_can.py
```

## 설정 설명

설정 파일은 `config/config.yaml`에 위치하며, 다음 설정을 포함합니다.

```yaml
version: 2.0.0
first_run: false              # 최초 실행 여부
language: zh_CN               # 인터페이스 언어
sudo_password: ""             # sudo 비밀번호 (평문 저장, 보안 주의)
last_can_channels:            # 이전에 사용된 CAN 채널
  right: can0
  left: can1
scripts:                      # MoveIt 스크립트 경로
  moveit_sim: ""
  moveit_can: ""
```

## 안전 주의사항

단일 모터 테스트 기능 사용 시, 반드시 다음 사항에 유의하시기 바랍니다.

1. 모터가 견고하게 설치되어 있고 주변에 사람이 없는지 확인합니다
2. 조작자는 비상 정지 버튼 위에 손을 대기시킨 채로 작업합니다
3. 초기 테스트 시 파라미터 값을 최대값의 10% 미만으로 설정합니다
4. 이상 상황 발견 시 즉시 비상 정지 버튼을 누릅니다

## API 의존성

본 시스템은 `openarmx_arm_driver` 패키지를 기반으로 하며, 주로 다음 기능을 사용합니다.

- `Robot` - 듀얼암 로봇 제어 클래스
- `get_all_can_interfaces()` - 모든 CAN 인터페이스 조회
- `get_available_can_interfaces()` - 활성화된 CAN 인터페이스 조회
- `enable_can_interface()` - CAN 인터페이스 활성화
- `disable_can_interface()` - CAN 인터페이스 비활성화
- `check_can_interface_type()` - 인터페이스 타입 확인 (실제/가상)
- `verify_can_interface()` - 인터페이스 상태 검증

## 작성자

- **Wei Lindong** (魏林栋)
- 회사: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 웹사이트: https://openarmx.com/

## 버전

v2.0.0

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
