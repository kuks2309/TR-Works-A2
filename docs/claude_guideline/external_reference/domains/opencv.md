# 외부 참조 — OpenCV Add-on (External Reference — OpenCV / Computer Vision)

> 코어 [handling.md](../handling.md) 의 도메인 sub-file. 코어 §1~12 규칙(특히 §3 source 분리)에 OpenCV 특화 taxonomy·경로·인용 형식·추정 사례·grep 을 더한다.

**트리거**: `cv2.`/`cv::` import, `findChessboardCorners`, `calibrateCamera`, `Mat`, `imread`, BGR/RGB 변환, distortion 모델, 카메라 calibration 작업.

## 1. 매뉴얼 종류 taxonomy

| 종류 | 다루는 정보 | 예 |
| --- | --- | --- |
| **OpenCV 공식 docs** | API reference, 함수 시그니처, default 파라미터, 버전별 차이 | docs.opencv.org/4.x/ |
| **OpenCV 소스 코드** | 구현체, 실제 default, 알고리즘 변형 | github.com/opencv/opencv `modules/<module>/src/<file>.cpp` |
| **알고리즘 원문 논문** | 알고리즘 의도, 가정, 권장 파라미터 범위 | Zhang 1999 (calibration), Lucas & Kanade 1981 (optical flow) |
| **카메라 calibration spec** | distortion 모델 정의 (Brown-Conrady 5/8-param, fisheye) | OpenCV `calib3d` docs |
| **카메라 매뉴얼** (벤더) | sensor type, 렌즈 종류, intrinsic 추정값 | RealSense D435i datasheet |

## 2. 보관 경로

- OpenCV 공식 docs (버전 명시 의무): `references/opencv/<version>/` (예: `references/opencv/4.8.0/`)
- 알고리즘 논문: `references/papers/<topic>/<author-year>.pdf`
- 카메라 매뉴얼: `references/<vendor>/<camera>/`

## 3. 인용 형식

```
[OpenCV 4.8.0 docs, cv::calibrateCamera, "Detailed Description"](references/opencv/4.8.0/calib3d_calibrateCamera.html)
[Zhang 1999 "A Flexible New Technique for Camera Calibration", IEEE PAMI 22(11), p.1330](references/papers/calibration/zhang-1999.pdf)
[Intel RealSense D435i DataSheet v1.3, §4.2, page 18](references/intel/realsense-d435i/datasheet.pdf)
```

## 4. 흔한 추정 단정 사례

- OpenCV 함수 default 파라미터를 algorithm spec 으로 비약 (예: `findChessboardCorners` flags default 를 Zhang 원문 권장으로 단정)
- BGR vs RGB channel order 추정 단정 (`cv::Mat` type 미확인)
- OpenCV 3.x ↔ 4.x API 차이 미인용 (예: `cv2.findContours` 반환값 변경)
- 5-param Brown-Conrady distortion 모델을 fisheye 렌즈에 그대로 적용 (fisheye 는 `cv::fisheye::calibrate` 별도)
- `imread` default `IMREAD_COLOR`(3채널 BGR)를 alpha 채널 보존으로 추정
- 알고리즘 원문 미참조로 OpenCV 함수 동작을 "표준 알고리즘" 단정

## 5. 1차 source 확인 절차

- **OpenCV 버전 명시 의무** — major.minor.patch (예: 4.8.0). API 가 버전마다 다름
- 공식 docs 함수 page (URL + 버전 + accessed 일자)
- 소스 코드 file:line 직접 인용 (`modules/calib3d/src/calibration.cpp:LNNNN`)
- 알고리즘 원문 논문 인용 (저자·학회·연도) — OpenCV 채택 변형과 원문 권장의 차이 명시
- 카메라 sensor 의존 작업은 카메라 datasheet 별도 인용
- 실측: 체크보드 calibration 의 reprojection error 측정

## 6. 자체 점검 grep

```bash
TARGET=<분석 대상 .md>

# OpenCV 인용 (버전 명시 의무)
grep -oE "\[OpenCV [0-9]+\.[0-9]+\.[0-9]+[^]]*\]" $TARGET

# OpenCV default 인용 시 "algorithm spec 아님" 또는 소스 file:line 명시
grep -nE "OpenCV.*default|cv2\.|cv::" $TARGET | grep -vE "algorithm spec 아님|소스|github.com/opencv|file:line"

# 알고리즘 원문 인용
grep -oE "\[[A-Z][a-z]+ [12][0-9]{3}[^]]*\]" $TARGET

# distortion 모델 / 렌즈 종류 분리 (fisheye vs pinhole)
grep -nE "calibrateCamera|fisheye|distortion" $TARGET | grep -E "pinhole|fisheye|omnidir|Brown-Conrady"
```
