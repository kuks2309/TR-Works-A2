# 2026-06-07 — JTC(Joint Trajectory Controller) 명령 시 "덜덜덜" 떨림 근본원인 진단 (진단만, 코드 변경 없음)

## 증상

JTC(Joint Trajectory Controller)로 명령을 내리면 관절이 "덜덜덜" 떤다(고주파 버즈). 정지(hold) 상태에서도 떨림이 지속된다.

## 검토한 데이터 (rosbag)

`experiments/rosbags/` (603MB, git 제외). 분석 스크립트 2종 신규 작성:
- [analyze_jtc_jitter.py](../../experiments/analyze_jtc_jitter.py) — controller_state(reference/feedback/error/output) 디시리얼라이즈, 관절별 방향반전율·지배주파수·추종오차.
- [analyze_jointstates_tremor.py](../../experiments/analyze_jointstates_tremor.py) — `/joint_states`(고속) 속도·토크 스펙트럼 + 루프 타이밍(헤더 stamp dt).

대상 bag: `grav_hil_left_g090_081112`(최종, 중력보상 HIL), `jog_verify_20260607_043459`, `jog_20260607_034347`.

## 핵심 측정값

### 1) 정지 중에도 모든 관절이 떤다 — 위치는 고정인데 속도가 진동
`/joint_states` hold 구간:
- 정지 관절 위치 p2p = **0.022°(=엔코더 1카운트)** 인데 속도는 **±6~13°/s**, 부호반전 **~38회/초**(샘플레이트의 약 절반).
- 즉 떨림 주파수가 샘플레이트(68Hz)보다 높아 위치 샘플로는 못 잡고 속도/토크로만 드러나는 **소진폭·고주파(>34Hz) 버즈**. effort(토크) FFT(Fast Fourier Transform)에서 손목(j5~7) 등 10~30Hz 성분 확인.
- jog(중력보상 OFF)·grav(중력보상 ON) 모두 동일 → **중력보상 탓 아님, 모터 제어/양자화 문제**.

### 2) 정적 처짐(static droop) — 무적분 증거
controller_state `reference − feedback` err_mean **2~5°** (예: jog_verify right_joint4 −4.9°, right_joint5 −2.4°). 적분항 없어 중력/마찰 미보상.

### 3) 제어루프가 느리고 지터 큼 — 기록 누락 아님(헤더 타임스탬프로 확정)
| 토픽 | 설정 | 실제(헤더 stamp dt 중앙값) | 지터 |
|---|---|---|---|
| `/joint_states`(루프율) | 100Hz(10ms) | jog **14.7ms(68Hz)**, grav **18.1ms(55Hz)** | p99 24ms, 간헐 49~338ms |
| controller_state | 50Hz(20ms) | **28~36ms(28~35Hz)** | p99 40ms |

헤더 stamp(실제 루프 시각)와 bag 수신 시각 일치 → ros2_control 업데이트 루프 자체가 100Hz를 못 채우고 지터가 큼.

## 원인 (코드 근거)

모터 구동: [v10_simple_hardware.cpp:572-591](../../openarmx_ws/src/openarmx_ros2/openarmx_hardware/src/v10_simple_hardware.cpp#L572-L591) — MIT 임피던스 모드
```
τ = KP·(pos_cmd − pos) + KD·(vel_cmd − vel) + tau_ff
```
- KP/KD 팩토리값([:178-179](../../openarmx_ws/src/openarmx_ros2/openarmx_hardware/src/v10_simple_hardware.cpp#L178-L179)): KP=`{50,50,50,50,10,10,10}`, KD=`{2.5,2.5,2.5,2.5,0.5,0.5,0.5}`.
- JTC(Joint Trajectory Controller)는 `command_interfaces:[position]`만 발행 → **vel_cmd=0** → KD 항 = `−KD·vel`(측정속도 순수 댐핑).
- 측정 vel 은 엔코더 양자화(0.022°)를 미분한 값이라 노이즈 큼 → **KD가 그 속도 노이즈를 증폭 → 토크 채터 → 한계진동(limit cycle).** 이것이 hold 중 떨림의 직접 원인(적분 없는 MIT PD(Proportional-Derivative) + 양자화).

루프 저속 원인: [v10_simple_hardware.cpp:479-483](../../openarmx_ws/src/openarmx_ros2/openarmx_hardware/src/v10_simple_hardware.cpp#L479-L483) — `read()`가 매 사이클 `refresh_all()`+`recv_all()`로 CAN(Controller Area Network) 요청-응답을 **동기 블로킹** 수행 → 루프를 55~68Hz로 끌어내림. 추종 거칠음·안정여유 감소로 떨림을 악화(직접 원인 아님).

## 정리

1. **(주원인) MIT PD(Proportional-Derivative) 한계진동**: 무적분 KP/KD + 엔코더 양자화 + KD가 속도노이즈 증폭 → 소진폭 고주파 토크 채터. hold 중에도 떨림.
2. **(악화) 호스트 루프 저속·지터**: 동기 CAN(Controller Area Network) recv 로 100Hz→55~68Hz, 지터 최대 수백 ms.
3. **(별개) 정적 처짐 2~5°**: 무적분. 2026-06-07 `open_loop_control:true`(controllers.yaml)는 "jog 풀림"만 막았고 떨림·처짐은 미해결.

## 수정

진단만 — 코드 변경 없음. 분석 스크립트 2종만 추가.

## 해결 방향 (미실행, 사용자 지시 대기)

1. **모터 게인 재튜닝**: KP 소폭↓ 또는 **속도 피드백 저역통과 필터**(KD가 보는 양자화 노이즈 제거). `experiments/joint_gain_tuner.py` 활용.
2. **제어루프율 복구**: `read()`의 `recv_all()`를 별도 수신 스레드로 분리(비동기) → update 루프 100Hz+ 회복. 먼저 루프 타이밍 프로파일링.
3. **속도 피드포워드**: JTC(Joint Trajectory Controller)에 velocity interface 추가 → vel_cmd 공급으로 `KD·(vel_cmd−vel)` 채터 완화.

## 재발 방지

- 떨림/추종 이슈 분석 시 controller_state 35Hz 로는 고주파 떨림을 못 잡는다 → `/joint_states` 속도/토크 + 헤더 stamp dt 로 루프율·고주파를 본다(저속 토픽의 reversal 은 양자화 dither 와 혼동 주의).
- 루프율 의심 시 bag 수신 dt 가 아니라 **헤더 stamp dt** 로 실제 루프 시각을 확인(기록 누락과 구분).
</content>
