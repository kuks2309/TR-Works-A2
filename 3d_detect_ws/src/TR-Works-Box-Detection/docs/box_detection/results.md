# 평면 검출 실측 결과 — 5 bags / 2026-05-06

`analyze_planes` 5프레임 평균(mid-time 등간격), distance threshold = 1 cm, RANSAC 2000 반복. RealSense optical frame 기준 (`+y=down`), normal은 `+y` 방향으로 통일.

> 평면 방정식 형태: `ax + by + cz + d = 0` (≈ 단위 normal). 박스 높이 = `|d_box_top - d_ground|`.

---

## 1. center 카메라 (`d435_center_depth_optical_frame`)

| Bag | Ground (책상) | Box-Top | 박스 높이 (Δ\|d\|) |
|---|---|---|---|
| big_box_a            | `+0.0045 x +0.8120 y +0.5836 z −0.7819 = 0` | `+0.0014 x +0.7891 y +0.6142 z −0.4503 = 0` | **0.332 m** |
| medium_box_narrow_a  | `+0.0006 x +0.8037 y +0.5950 z −0.8110 = 0` | `+0.0027 x +0.8023 y +0.5969 z −0.5206 = 0` | **0.290 m** |
| medium_box_wide_a    | `+0.0012 x +0.8017 y +0.5977 z −0.8171 = 0` | `+0.0053 x +0.7806 y +0.6251 z −0.5425 = 0` | **0.275 m** |
| short_box_narrow_a   | `+0.0035 x +0.8091 y +0.5877 z −0.8025 = 0` | `−0.0098 x +0.7847 y +0.6198 z −0.5691 = 0` | **0.233 m** |
| short_box_wide_a     | `+0.0032 x +0.8128 y +0.5825 z −0.7834 = 0` | `−0.0144 x +0.7856 y +0.6186 z −0.5669 = 0` | **0.217 m** |

## 2. upper 카메라 (`d435_center_upper_depth_optical_frame`)

| Bag | Ground (책상) | Box-Top | 박스 높이 (Δ\|d\|) |
|---|---|---|---|
| big_box_a            | (검출 실패 — 박스 윗면이 시야 대부분, RANSAC 수직 평면을 잡음) | — | — |
| medium_box_narrow_a  | `+0.0032 x +0.8678 y +0.4970 z −0.6816 = 0` | `−0.0176 x +0.8503 y +0.5261 z −0.4175 = 0` | 0.264 m |
| medium_box_wide_a    | `+0.0007 x +0.8587 y +0.5124 z −0.6992 = 0` | `+0.0084 x +0.8259 y +0.5638 z −0.4446 = 0` | 0.255 m |
| short_box_narrow_a   | `+0.0046 x +0.8660 y +0.4999 z −0.6863 = 0` | `−0.0062 x +0.8421 y +0.5392 z −0.4595 = 0` | 0.227 m |
| short_box_wide_a     | `+0.0043 x +0.8682 y +0.4962 z −0.6801 = 0` | `+0.0299 x +0.8285 y +0.5593 z −0.4704 = 0` | 0.210 m |

---

## 3. 일관성 검증

### 3.1 ground 평면의 d 분포 (카메라 위치 = 일정해야 함)
- center 카메라: `|d|` ≈ 0.78 ~ 0.82 m → 변동폭 ~4 cm. 책상이 카메라에서 약 80 cm 떨어져 있음 (RealSense 정밀도 한계 내 일관).
- upper 카메라: `|d|` ≈ 0.68 m → 변동폭 ~1 cm. 매우 일관.

### 3.2 박스 높이의 시나리오별 단조성
**center 카메라 결과** 기준:
```
big(33.2) > medium_narrow(29.0) > medium_wide(27.5) > short_narrow(23.3) > short_wide(21.7)   [cm]
```
시나리오 이름과 일관 (`big > medium > short`). 같은 카테고리 내 narrow/wide 차이는 1 cm 내외 — 박스 종류 자체의 미세 차이로 추정.

### 3.3 양 카메라 비교
같은 박스에 대해 center vs upper의 박스 높이 차이:

| Bag | center | upper | Δ |
|---|---|---|---|
| medium_box_narrow_a | 29.0 | 26.4 | 2.6 cm |
| medium_box_wide_a | 27.5 | 25.5 | 2.0 cm |
| short_box_narrow_a | 23.3 | 22.7 | 0.6 cm |
| short_box_wide_a | 21.7 | 21.0 | 0.7 cm |

박스가 클수록 카메라간 차이가 커짐. 추정 원인: upper 카메라는 박스를 더 가파른 각도로 봐서 윗면 가장자리 픽셀이 적게 잡힘 → 평면 추정에 inlier 분포 편향.

---

## 4. 측정 조건 메모

- bag 기록 일자: 2026-05-06
- 카메라: D435 두 대 (center 시리얼 `818312070932`, upper `819612070814`)
- 카메라 셋업: 로봇 작업 자세에서 책상을 위/측면에서 내려다보는 시야
- bag duration 6 ~ 10 초 (시나리오별)
- 분석 시점: bag 시작 0%/25%/50%/75%/100% 의 5프레임 평균

값을 다른 좌표계로 옮기려면 `base_link → d435_center_*_link` 정적 TF가 필요합니다 (외부 robot URDF에서 관리).
