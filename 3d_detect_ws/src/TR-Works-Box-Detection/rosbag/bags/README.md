# Bag Naming Convention

`box_pick_and_place` 시나리오 ros2 bag 파일 명명 규칙 + 폴더 구조.

> 전체 기록·재생 워크플로우는 [HOWTO.md](HOWTO.md) 참조.

## 폴더 구조 (날짜별 분리)

```
bags/
├── HOWTO.md, README.md, qos_override.yaml   ← 루트 (공통 문서·설정)
├── 20260504/   1-camera 시절 데이터 (구, 보존용)
└── 20260506/   2-camera 표준 (현, 새 bag은 여기로)
```

새 bag은 항상 **그 날짜 폴더 안에** 기록. 카메라 구성이나 시스템이 크게 바뀌면 새 날짜 폴더 생성.

## Suffix 의미

- `_a` → **앉은 자세에서 박스 잡기**
  (시작부터 앉은 상태로 pick 동작 수행)

- `_b` → **서있는 자세 → 앉기 → 박스 잡기**
  (정자세에서 출발하여 앉기 동작 후 pick 수행)

## 카메라 구성 (현재 표준: 2대)

- `d435_center` (시리얼 `818312070932`)
- `d435_center_upper` (시리얼 `819612070814`, 2026-05-04 추가)

## 기록 명령 (표준)

```bash
source /home/tc/Project/TR-Works-Dev/kkw/TR-Works_ros2_ws/install/setup.bash && \
unset FASTRTPS_DEFAULT_PROFILES_FILE && \
cd /home/tc/Project/TR-Works-Dev/bags/<날짜폴더> && \
ros2 bag record -a \
  --qos-profile-overrides-path ../qos_override.yaml \
  --max-cache-size 1073741824 \
  -o <scenario>_<a|b>
```

각 옵션의 의미는 [HOWTO.md §2.3](HOWTO.md) 참조.

## bag 목록

### `20260506/` — 2-camera (현 표준)

| 시나리오 | _a | _b | 비고 |
|---|---|---|---|
| big_box | ✅ | (미작업) | _a 2026-05-06 재기록 |
| medium_box_narrow | ✅ | (미작업) | _a 2026-05-06 재기록 |
| medium_box_wide | ✅ | (미작업) | _a 2026-05-06 재기록 |
| short_box_narrow | ✅ | ✅ | 2026-05-04 작업분 (이동됨) |
| short_box_wide | ✅ | ✅ | 2026-05-04 작업분 (이동됨) |

### `20260504/` — 1-camera (구, 보존만)

| 시나리오 | _a | _b | 비고 |
|---|---|---|---|
| big_box | ✅ | ✅ | `d435_center_upper` 없음 |
| medium_box_narrow | ✅ | ✅ | (← medium_box_short_*) |
| medium_box_wide | ✅ | ✅ | (← medium_box_*) |

> `medium_box_*`/`medium_box_short_*` 옛 bag은 시나리오 이름 변경에 따라 폴더명만 rename.
> 내부 db3 파일명은 옛 이름 유지 (`medium_box_short_a_0.db3` 등) — `ros2 bag info/play` 정상 동작.
> 1-camera 시절 데이터라 `d435_center_upper` 토픽 없음.
