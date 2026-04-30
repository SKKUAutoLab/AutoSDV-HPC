# ECU (RA6M5) 소스 코드 최적화 계획

**작성일:** 2026-03-26
**대상:** RA6M5_ECU/adc_rtos 전체 소스 코드
**빌드 환경:** Renesas e2 studio / FSP (FreeRTOS)
**참조:** ZCU 최적화 계획 `.claude/zcu-optimization-plan.md`

---

## 아키텍처 요약

```
FreeRTOS (RA6M5)
├─ Blinky Thread (메인) — HW 초기화, LED 토글, 1초 루프
├─ PotentiometerTask (Steering_Zone 전용, 50ms) — ADC → 조향 모터 제어
├─ SteeringStatusTask (Steering_Zone 전용, 300ms) — CAN 0x113 현재 각도 송신
├─ canfd0_callback (ISR) — CAN 수신: 0x111(조향), 0x123(속도)
└─ R_AGT0_Interrupt (ISR) — 2초 주기 워치독
```

**하드웨어 Zone별 역할:**
- Steering_Zone: 조향 모터 + 포텐셔미터 + CAN 0x111 수신/0x113 송신
- Front_Left / Front_Right / Rear_Left / Rear_Right: 구동 모터 + CAN 0x123 수신

---

## 파일 구조

```
RA6M5_ECU/adc_rtos/src/
├── blinky_thread_entry.c  ← 메인 태스크, CAN 콜백, 워치독, 조향 제어
├── hardware.c             ← Motor_Control, ADC, PWM
├── hardware.h             ← 하드웨어 추상화, enum
├── network.c              ← CANFD 초기화
└── motor_test.c           ← 테스트 모드 (비활성)
```

---

## 이슈 목록

### E1 — 워치독 타임아웃 6초 → 400ms로 단축 + 조향 워치독 신설 [P0 — 안전] ✅ (2026-04-29)

**파일:** `blinky_thread_entry.c:296-323`, `hal_data.c:15-18`, `ecu_config.h` (신규), `configuration.xml` (사용자가 e2 studio에서 직접 변경)

**문제:**
```c
void R_AGT0_Interrupt(timer_callback_args_t *p_args) {
    if(motor_stop == 0) { motor_stop++; }      // 2초 후
    else if(motor_stop == 1) { motor_stop++; }  // 4초 후
    else if(motor_stop == 2) { Motor_Stop(); }   // 6초 후 정지
}
```
Timer0 주기 2초 x 3단계 = **6초** 동안 모터가 무제어 상태로 동작. 1/5 스케일 차량에 위험.
또한 조향 zone은 워치독 자체가 없어 HPC가 죽으면 마지막 angle 명령이 무한 hold (Motor_Stop 은 g_timer3 PWM duty=0 만 처리).

**해결 (실 적용):**
- AGT period: 2s → **200ms** (configuration.xml `g_timer0` `period=200`, `unit_period_msec`)
- hal_data.c `period_counts`: 0x10000 → **0x199A** (LOCO 32.768kHz × 200ms ≒ 6554)
- 카운터 임계값: 3단계 → **2단계** (총 timeout = 200ms × 2 = **400ms**)
- 새 헤더 `ecu_config.h` 도입: `WATCHDOG_AGT_PERIOD_MS=200`, `WATCHDOG_TIMEOUT_TICKS=2`
- `R_AGT0_Interrupt` 를 zone 별로 분기:
  - **Steering_Zone**: 별도 카운터 `steering_motor_stop` 사용 → 발동 시 `angle = 0` + `Motor_Stop()` (조향 PWM 즉시 차단 + PotentiometerTask 자동 센터 복귀)
  - **구동 zone (Front/Rear Left/Right)**: 기존 `motor_stop` 사용 → 발동 시 `Motor_Stop()` (구동 PWM duty=0)
- `canfd0_callback`: zone 별로 카운터 리셋 분기 (조향 메시지 수신 시만 `steering_motor_stop=0`, 구동 메시지 수신 시만 `motor_stop=0`)

**최종 인터럽트 핸들러:**
```c
void R_AGT0_Interrupt(timer_callback_args_t *p_args) {
    FSP_PARAMETER_NOT_USED (p_args);
#if Hardware == Steering_Zone
    if (steering_motor_stop < WATCHDOG_TIMEOUT_TICKS) steering_motor_stop++;
    else { angle = 0; Motor_Stop(); }
#else
    if (motor_stop < WATCHDOG_TIMEOUT_TICKS) motor_stop++;
    else { Motor_Stop(); }
#endif
}
```

**검증 절차 (사용자 진행):**
1. e2 studio 에서 configuration.xml AGT g_timer0 period 200ms 로 변경 → Generate Project Content
2. 빌드 + 펌웨어 플래시 (모든 zone)
3. 차량 정상 주행 시작 후 HPC 강제 종료 (`pkill -9 ros2` 등)
4. 약 400ms 이내 구동 정지 + 조향 센터 복귀 확인

---

### E2 — 전역 변수 ISR↔Task 경쟁 [P1] ✅ (2026-04-29)

**파일:** `blinky_thread_entry.c` (PotentiometerTask)

**문제:**
- `motor_stop` (volatile int): ISR(canfd0_callback)에서 리셋, ISR(R_AGT0_Interrupt)에서 증가, Task에서 읽기
- `g_can_steering_rx_frame`, `g_can_rx_frame`: ISR에서 쓰기, Task에서 읽기 → struct tearing 가능
- `angle`: Task에서 쓰기/읽기

단일 코어이므로 ISR은 원자적이지만, Task 간 공유는 선점에 의해 데이터 손상 가능.

**해결 (실 적용):**
- `motor_stop`/`steering_motor_stop` 은 `volatile int` 단일 워드 → 자체 atomic 으로 OK
- `PotentiometerTask` 에서 `g_can_steering_rx_frame` 접근 시 critical section 으로 보호 + 로컬 스냅샷 사용:
```c
can_frame_t local_steering_frame;
taskENTER_CRITICAL();
local_steering_frame = g_can_steering_rx_frame;
taskEXIT_CRITICAL();

if (local_steering_frame.id == CAN_STEERING_ID) {
    angle = (int)local_steering_frame.data[3] - MAX_STEERING_STEP;
    angle = -angle;
}
```
- 구동 zone 의 `g_can_rx_frame` 은 ISR 안에서 즉시 `Motor_Control` 호출만 하고 Task 가 읽지 않으므로 별도 보호 불필요

---

### E3 — 조향 캘리브레이션 실패 처리 없음 [P1] ✅ (2026-04-29)

**파일:** `blinky_thread_entry.c` (Steering_Auto_Calibration)

**문제:**
캘리브레이션 중 ADC 읽기 실패 시 예외 처리 없음. 좌/우 최대값이 동일하면 나눗셈에서 0으로 나누기 가능.

**해결 (실 적용):**
- 좌/우 평균값 차이가 `STEER_CALIB_MIN_RANGE_MV` (300mV) 미만이면 sanity check 실패로 간주, 기본값 (`STEER_CALIB_DEFAULT_RANGE_MV` 500mV) 으로 폴백
- 센터 역전 check 도 기본값 폴백 (역전 시 LEFT < CENTER, RIGHT > CENTER 보호)
- 센터 복귀 단계는 이미 40-tick (=2초) timeout 구조였음 → 그대로 유지
- 평균 계산 시 i=1~49 → **i=0~49 (50개 모두)** 로 수정 (E-N3 함께 처리)

---

### E4 — CAN ID / 바이트 매핑 하드코딩 [P2] (CAN ID 부분 ✅ 2026-04-29)

**파일:** `blinky_thread_entry.c`, `ecu_config.h` (신규)

**문제:**
```c
if(p_args->frame.id == 0x111) ...        // 조향
if(p_args->frame.id == 0x123) ...        // 구동
steering_status_frame.id = 0x113;        // 상태 송신
if(Hardware == Front_Left) Motor_Control(data[0], data[3]);
if(Hardware == Front_Right) Motor_Control(data[4], data[7]);
```
- CAN ID 가 4군데 산재 (canfd0_callback x2, SteeringStatusTask, PotentiometerTask)
- Zone 별 CAN 바이트 매핑이 switch-case 로 하드코딩

**해결 (CAN ID 부분 — 실 적용):**
새 헤더 `ecu_config.h` 에 정의:
```c
#define CAN_STEERING_ID  0x111
#define CAN_SPEED_ID     0x123
#define CAN_STATUS_ID    0x113
```
→ `blinky_thread_entry.c` 의 4군데 모두 define 으로 교체. 이후 CAN ID 변경 시 헤더 한 곳만 수정.

**해결 (바이트 매핑 부분 — 미적용, 향후 작업):**
설정 헤더 `ecu_config.h` 에 Zone 별 바이트 오프셋 정의:
```c
#define MOTOR_DIR_OFFSET  0  // 또는 4
#define MOTOR_SPD_OFFSET  3  // 또는 7
```
ZCU의 zcu_config.h와 동일 패턴.

---

### E5 — Task 매초 재생성 (메모리 누수) [P1] ✅ (2026-04-29)

**파일:** `blinky_thread_entry.c` (blinky_thread_entry)

**문제:**
```c
while(1) {
    xTaskCreate(PotentiometerTask, ...);    // 매초 새 태스크 생성
    xTaskCreate(SteeringStatusTask, ...);   // 매초 새 태스크 생성
    vTaskDelay(1000);
}
```
태스크가 완료되지 않으면 매초 새로 생성 → FreeRTOS 힙 고갈 → 크래시.

**해결 (실 적용):**
태스크 생성을 부팅 시 1회만 (Steering_Auto_Calibration 직후) 실행되도록 while 루프 밖으로 이동.
이제 while(1) 루프는 LED 토글 + vTaskDelay 만 수행.
구동 zone 은 별도 task 가 없으므로 변경 영향 없음.

---

## ZCU 연계 이슈 (ZCU 코드도 수정 필요)

### Z1 — 조향 클램핑 범위 불일치 [P0 — ZCU 수정]

**파일:** `S32G3_ZCU/include/zcu_config.h`

**문제:**
ZCU: `MIN_STEERING=-15, MAX_STEERING=15`
ECU: angle 범위 = `-7 ~ +7`, CAN data[3] = `angle + 7` = `0 ~ 14`

ZCU에서 15를 보내면 → `(15+7) & 0xFF = 22` → ECU에서 14 초과 = 예측 불가.

**수정:** `zcu_config.h`에서:
```c
#define MIN_STEERING  (-7)
#define MAX_STEERING  (7)
```

### Z2 — StatusPublisher 종료 메시지 경로 오류 [P1 — ZCU 수정]

**파일:** `S32G3_ZCU/src/status_module/status_publisher.cpp`

**문제:**
`sendShutdownMessages()`가 CAN 0x113으로 종료 메시지 전송.
하지만 ECU는 0x113을 **수신하지 않음** (송신만 함).

ECU가 수신하는 ID: 0x111(조향), 0x123(속도).
`ControlSubscriber::sendShutdownMessages()`가 이미 0x111/0x123으로 올바르게 전송 중.

**수정:** `StatusPublisher::sendShutdownMessages()`의 CAN 전송 제거:
```cpp
void StatusPublisher::sendShutdownMessages() {
    // ECU는 0x113을 수신하지 않음
    // 종료 시 ControlSubscriber가 0x111/0x123으로 정지 명령 전송
}
```

---

## 코드 재검토 후 추가 발견 이슈 (2026-03-27)

| # | 심각도 | 문제 | 파일 | 상태 |
|---|--------|------|------|------|
| E-N1 | 중간 | R_ADC_ScanStart 중복 호출 (ADC_Setting + Potentiometer_Read 양쪽) | hardware.c | ✅ 2026-04-29 — ADC_Setting 에서 ScanStart 제거, Potentiometer_Read 만 시작 |
| E-N2 | 중간 | network.c에서 strlen()으로 CAN 바이너리 데이터 길이 체크 (NULL 없으면 오버플로) | network.c | ✅ 2026-04-29 — strlen 제거, dlc 인자 + NULL 체크로 변경 |
| E-N3 | 낮음 | 캘리브레이션 for(i=1; i<50) — 첫 샘플 건너뜀 (의도/버그 불명확) | blinky_thread_entry.c | ✅ 2026-04-29 — i=0~49 (50개 모두) 평균 사용 |

---

## 실행 우선순위

| 순서 | 이슈 | 대상 | 우선순위 | 상태 |
|------|------|------|----------|------|
| 1 | **Z1** 조향 클램핑 범위 수정 (-15→-7) | ZCU | P0 | ❌ |
| 2 | **Z2** StatusPublisher 종료 메시지 수정 | ZCU | P1 | ❌ |
| 3 | **E1** 워치독 타임아웃 단축 (6초→400ms) + 조향 워치독 신설 | ECU | P0 | ✅ 2026-04-29 |
| 4 | **E5** Task 재생성 방지 | ECU | P1 | ✅ 2026-04-29 |
| 5 | **E-N2** strlen → dlc 사용 | ECU | P1 | ✅ 2026-04-29 |
| 6 | **E3** 캘리브레이션 실패 처리 | ECU | P1 | ✅ 2026-04-29 |
| 7 | **E2** 전역 변수 보호 | ECU | P1 | ✅ 2026-04-29 |
| 8 | **E-N1** ADC ScanStart 중복 제거 | ECU | P2 | ✅ 2026-04-29 |
| 9 | **E4** Zone 설정 외부화 (CAN ID 부분) | ECU | P2 | ✅ 2026-04-29 |
| 9b | **E4** Zone 설정 외부화 (바이트 매핑 부분) | ECU | P2 | ❌ (사용자 결정으로 보류) |
| 10 | **E-N3** 캘리브레이션 첫 샘플 확인 | ECU | P2 | ✅ 2026-04-29 |

---

## 빌드 환경 참고

- ECU 코드는 **Renesas e2 studio + FSP** 환경에서 빌드
- 이 프로젝트에서 직접 빌드 불가 (cross-compile 환경 필요)
- 코드 수정은 여기서 하고, e2 studio에서 빌드 + 플래시

---

## 성공 기준

1. 워치독 타임아웃 6초 → 1초 이하
2. ZCU 종료 시 ECU가 1초 내 자동 정지
3. Task 메모리 누수 제거
4. 조향 범위 ZCU-ECU 일관성 확보
