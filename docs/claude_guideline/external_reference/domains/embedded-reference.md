# 외부 참조 — 임베디드 Add-on (External Reference — Embedded)

> 코어 [handling.md](../handling.md) 의 도메인 sub-file. 코어 §1~12 규칙(특히 §3 source 분리)에 임베디드 특화 taxonomy·경로·인용 형식·추정 사례·grep 을 더한다.

**트리거**: `__attribute__((interrupt))`, `ISR(`, `NVIC_`, `IRQHandler`, FreeRTOS API, STM32 HAL 매크로, `.ld` linker script, register-level access, `volatile` 빈출, MCU datasheet / Family Manual 참조.

## 1. 매뉴얼 종류 taxonomy

| 종류 | 다루는 정보 | 예 |
| --- | --- | --- |
| **silicon datasheet** | 전기 특성, 핀맵, 패키지, Operation Conditions | `STM32F4 DataSheet`, `Infineon AURIX TC3xx DataSheet` |
| **User / Family / Reference Manual** | peripheral register-level 동작, SR(service request)/IRQ 라우팅, DMA 토폴로지 | `STM32F4 Reference Manual`, `AURIX TC3xx Family Manual` |
| **SDK / HAL 문서** | API 시그니처, sample, default 매크로 | `STM32 HAL UM`, `iLLD AURIX SDK manual` |
| **Application Note** | 응용 사례, 권장 회로, 권장 파라미터 | `AN2867 (STM32 oscillator)` |
| **Errata** | 알려진 silicon 버그, 회피 절차 | `STM32F4 Errata`, `AURIX Errata sheet` |

## 2. 보관 경로

`references/<vendor>/<mcu>/` (예: `references/st/stm32f407/`, `references/infineon/tc375/`).

## 3. 인용 형식 (코어 §2 강제 형식 적용)

```
[STM32F4 Reference Manual RM0090 Rev 19, Table 134, page 543](references/st/stm32f407/RM0090.pdf)
[AURIX TC3xx Family Manual v2.0.0, §10.3.2, page 214](references/infineon/tc375/family_manual.pdf)
```

## 4. 흔한 추정 단정 사례

- 벤더 SDK 의 `<PERIPHERAL>_<PARAM>_MAX` 매크로를 silicon datasheet spec 으로 비약
- `volatile` 단독 사용을 SDK docstring "권장" 만 보고 race 가드 충분으로 단정
- HAL default ADC sampling time 을 datasheet 권장 운영점으로 비약
- Errata 미확인 → 알려진 silicon 버그 회피 누락
- "TYP = 권장값" 비약 (코어 §3.1)

## 5. 1차 source 확인 절차

- silicon datasheet — 전기 특성 단정 시 의무
- User / Family / Reference Manual — register / peripheral / DMA 단정 시 의무
- 두 종류 모두 다운로드 권장: `<vendor>_<mcu>_DataSheet_vX.Y.pdf` + `<vendor>_<mcu>_ReferenceManual_vX.Y.pdf`
- Errata 별도 다운로드 + 작업 전 검토 의무

## 6. 자체 점검 grep

```bash
TARGET=<분석 대상 .md>

# 임베디드 인용 형식 (DataSheet / Reference Manual / Family Manual)
grep -oE "\[(STM32|AURIX|ESP32|nRF|MSP430|PIC|AVR|ATmega)[^]]*\b(DataSheet|Reference Manual|Family Manual|User Manual)[^]]*page [0-9]+\]" $TARGET

# SDK 매크로 인용 시 "silicon spec 아님" 명시 여부
grep -nE "_MAX|_MIN|#define" $TARGET | grep -vE "silicon|device|SDK 권장|datasheet 아님"

# Errata 검토 흔적
grep -E "Errata|errata" $TARGET
```
