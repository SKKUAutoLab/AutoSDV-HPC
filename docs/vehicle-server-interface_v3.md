# 차량(HPC) ↔ Server PC 인터페이스 합의안

| 항목 | 내용 |
|---|---|
| 상태 | **Draft v3 (서버 검토 반영, HPC 최종 확인 대기)** |
| 작성일 | 2026-09-17 |
| 기준 버전 | MediaMTX v1.21.0, Mosquitto 2.1 (MQTT v5), HPC GStreamer 1.24.2 |
| 대상 | 전방 카메라 영상 스트리밍, 차량 상태/진단 정보 전송 |

## 변경 이력

| 버전 | 내용 |
|---|---|
| v1 | 서버 측 초안 |
| v2 | HPC 1차 검토: 640×480 확정, ID 고정값, Diagnostics 요약 구조, 시간 동기화 미요구 |
| v3 | 서버 검토 반영: ID 형식 규칙 수정, Retain 정책 정리, `camera/{camera_id}/status` 제거 → `streamer/status`(별도 MQTT 연결)로 대체, STALE 층위 구분, `zones` 규칙, `status` payload·`seq` 규칙, HPC GStreamer 확인 결과 반영 |

## 0. 요약

> 차량은 H.264(Constrained Baseline, 키프레임 1~2초)로 **WHIP `POST http://100.74.147.75:8889/autosdv/{vehicle_id}/{camera_id}/whip`** 에 송출합니다.
> STUN/TURN은 쓰지 않습니다.
> 상태와 진단은 **MQTT v5 `100.74.147.75:1883`**, Keep Alive 5초, [9장](#9-mqtt-broker-정보)의 토픽·QoS·Retain 표를 따릅니다.
> 스트리밍 오류는 streaming 프로세스가 별도 MQTT 연결로 발행하는 `streamer/status`의 `state`와 `error_code`로 표현합니다.

```
[차량 HPC]
  ├─ streaming 프로세스 ──WebRTC (WHIP)──▶ MediaMTX (Server PC) ──WebRTC (WHEP)──▶ 브라우저 (Dashboard)
  │        └──────────────MQTT v5 (streamer/status)──┐
  └─ gateway 프로세스 ────MQTT v5 (status, diagnostics)──▶ Mosquitto ─▶ Backend (FastAPI) ─▶ TimescaleDB / Dashboard
                              (모든 구간 Tailscale 경유)
```

### 시그널링 방식에 대한 전제

서버는 MediaMTX를 쓰므로 **표준 WHIP(HTTP)** 로 시그널링합니다.
따라서 WebSocket 기반 시그널링, 자체 SDP/ICE JSON 스키마, 별도 차량 등록 절차는 **정의하지 않습니다.**

| 단계 | HTTP 요청 | 내용 |
|---|---|---|
| 세션 생성 | `POST` | 본문에 SDP Offer → `201 Created`, 본문에 SDP Answer, `Location` 헤더에 세션 URL |
| ICE 추가 | `PATCH {Location}` | Trickle ICE (선택) |
| 세션 종료 | `DELETE {Location}` | |

---

## 1. WebRTC 연결 구조

**결정: 서버의 MediaMTX를 거치는 구조**

- 연결 흐름: 차량 → (WHIP) → MediaMTX → (WHEP) → 브라우저
- 차량은 보는 사람 수와 관계없이 **서버로 한 번만** 송출합니다.
- MediaMTX는 **영상을 다시 인코딩하지 않고 그대로 전달**합니다. 따라서 차량이 보낸 코덱과 프로파일을 브라우저가 그대로 재생할 수 있어야 합니다([4장](#4-영상-코덱) 참고).

## 2. Signaling 주소와 스키마

**결정: WHIP 표준 사용, WebSocket 없음**

| 항목 | 값 |
|---|---|
| WHIP URL | `http://100.74.147.75:8889/autosdv/{vehicle_id}/{camera_id}/whip` |
| 1단계 URL | `http://100.74.147.75:8889/autosdv/a0000/camera_01/whip` |
| 요청 | `POST`, `Content-Type: application/sdp`, 본문은 SDP Offer |
| 응답 | `201 Created`, 본문은 SDP Answer, `Location` 헤더에 세션 URL |
| ICE 추가 | `PATCH {Location}` (trickle). 후보를 모두 모은 뒤 한 번에 Offer를 보내도 됨 |
| 종료 | `DELETE {Location}` |
| 인증 | `Authorization: Bearer {user}:{pass}`. 차량 계정은 `autosdv/` 경로에만 publish 허용. 계정 정보는 서버 설치 후 별도 전달 |
| 차량 등록 | 별도 절차 없음. **경로 이름이 곧 식별자**이고, 온라인 여부는 MQTT로 판단 |

- 연결은 **Tailscale 경유**이고, WireGuard가 암호화하므로 HTTP로 운용합니다.
- HPC 송출: GStreamer 1.24.2의 `whipclientsink` 사용 (확인 완료).
- 인증: `whipclientsink`의 `auth-token` 속성에 `{user}:{pass}`를 넣으면 `Bearer` 헤더로 전달됩니다. MediaMTX는 `Bearer user:pass` 형식을 사용자 이름과 비밀번호로 해석합니다 (v1.21.0 소스 확인).

## 3. SDP 생성 주체

**결정: 차량이 Offer를 만들고 MediaMTX가 Answer를 반환**

- WHIP 표준이 이 방식입니다.
- 브라우저도 WHEP로 똑같이 Offer를 보내며, 이 부분은 서버 쪽에서 처리합니다.

## 4. 영상 코덱

**결정: H.264**

| 항목 | 값 | 이유 |
|---|---|---|
| 코덱 | H.264 | 하드웨어 인코딩(`nvv4l2h264enc`, 확인 완료), 모든 브라우저에서 재생 가능 |
| 프로파일 | **Constrained Baseline (B-frame 없음)** | B-frame이 들어가면 브라우저가 재생하지 못함 (MediaMTX 문서에 명시) |
| 키프레임 간격 | **1~2초** | 대시보드를 새로 연 사람은 다음 키프레임부터 영상이 보임 |
| SPS/PPS | **키프레임마다 반복 삽입** | 중간에 접속한 사람도 디코딩 가능. 예: `nvv4l2h264enc insert-sps-pps=1` |

VP8은 사용하지 않습니다.

## 5. 영상 제약 조건

**결정: 초기값 동의, 비트레이트 상한 조정**

| 항목 | HPC 초기안 | 합의값 |
|---|---|---|
| 해상도 | 640×480 | **640×480** (Camera 01 실제 입력 해상도, 4:3) |
| 프레임레이트 | 최대 30fps | 최대 30fps |
| 비트레이트 | 300kbps~2.5Mbps | **300kbps~1.5Mbps로 시작**, 필요하면 상한을 2.5Mbps까지 상향 |

640×480 30fps는 약 1.5Mbps면 화질이 충분합니다.

## 6. STUN/TURN 설정

**결정: 사용하지 않음 (Tailscale 사용)**

| 위치 | 설정 |
|---|---|
| 차량 | ICE 서버 목록을 **비워 둠** |
| 서버 (MediaMTX) | `webrtcAdditionalHosts: [100.74.147.75, 192.168.0.45]` |
| 서버 (MediaMTX) | `webrtcIPsFromInterfacesList: [tailscale0, enp2s0]`: `docker0` 같은 쓸모없는 주소가 연결 후보에 들어가지 않게 함 |

| 포트 | 용도 |
|---|---|
| 8889/tcp | WHIP/WHEP HTTP |
| 8189/udp | 영상 데이터 (WebRTC ICE) |

- Tailscale 인터페이스는 서버 방화벽에서 이미 전체 허용이라 따로 열 필요가 없습니다.
- 나중에 Tailscale 없이 배포하게 되면 이 항목은 다시 논의합니다.

## 7. MediaStream 식별 규칙

**결정: MediaMTX 경로로 식별**

- 경로 형식: `autosdv/{vehicle_id}/{camera_id}`
- 1단계 고정값: `vehicle_id=a0000`, `camera_id=camera_01`
- 1단계 MediaMTX 경로: `autosdv/a0000/camera_01`
- 경로 하나에 영상 트랙 하나만 보내므로, Dashboard는 **Track ID를 쓰지 않습니다.**
- `vehicle_id`는 MQTT 토픽의 `vehicle_id`와 **반드시 같은 문자열**이어야 합니다.
- ID 형식(`vehicle_id`, `camera_id`, `zone_id`): **영문 소문자, 숫자, `_`, `-`만 사용**
  - MediaMTX 경로 규칙(`^[0-9a-zA-Z_\-/\.]+$`)과 MQTT 토픽 규칙을 모두 만족합니다.

## 8. WebRTC 재연결 규칙

| 항목 | 값 |
|---|---|
| 연결 끊김 판단 | ICE/PeerConnection 상태가 `failed`면 즉시, `disconnected`가 **5초** 넘게 지속되면 끊김으로 봄. WHIP `POST`가 실패해도 마찬가지 |
| 재시도 간격 | 1 → 2 → 4 → 8 → 최대 10초, ±20% 랜덤 지연. 30초 이상 안정적으로 유지되면 간격 초기화 |
| 재연결 절차 | 이전 세션에 `DELETE`(실패해도 무시) → **새 PeerConnection 생성** → 새 `POST` |
| `session_id` | 재연결을 시도할 때마다 **차량이 새 UUID를 생성**해서 로그와 `streamer/status`에 기록 |
| 서버 동작 | `overridePublisher: true`로 설정해서, 새 세션이 들어오면 남아 있던 이전 세션을 대체 |

## 9. MQTT Broker 정보

### 접속 정보

| 항목 | 값 |
|---|---|
| 주소:포트 | `100.74.147.75:1883` (Tailscale) |
| 프로토콜 | MQTT **v5** |
| 인증 | 사용자 `vehicle`, 비밀번호는 별도 전달 (두 프로세스가 같은 계정 사용) |
| TLS | 사용하지 않음 (Tailscale이 암호화) |
| Keep Alive | **5초**. 브로커는 1.5배인 약 7.5초 안에 끊김을 감지하고 LWT를 발행하므로, 10초 OFFLINE 기준을 맞출 수 있음 |
| 세션 | Clean Start, Session Expiry 0 |

### MQTT 연결 (프로세스별 분리)

| 프로세스 | Client ID | 발행 토픽 | LWT |
|---|---|---|---|
| gateway | `autosdv-{vehicle_id}-gateway` (1단계: `autosdv-a0000-gateway`) | `status`, `diagnostics`, `event` | `status` ← `offline` |
| streaming | `autosdv-{vehicle_id}-streamer` (1단계: `autosdv-a0000-streamer`) | `streamer/status` | `streamer/status` ← `{"schema_version":"1.0","online":false}` |

- **같은 Client ID로 접속하면 기존 연결이 끊기므로** 두 프로세스는 반드시 서로 다른 Client ID를 사용합니다.
- 각 프로세스는 자기 연결에 LWT를 설정합니다. 한쪽 프로세스만 죽어도 서버가 구분할 수 있습니다.

### 토픽 / QoS / Retain

| 토픽 | 발행 | QoS | Retain | 주기 |
|---|---|---|---|---|
| `autosdv/{vehicle_id}/status` | gateway | 1 | ✅ | 연결 직후 `online`, 종료 시 `offline`. **정상 종료할 때도 `offline`을 직접 발행** (LWT는 정상 종료 시 발행되지 않음) |
| `autosdv/{vehicle_id}/diagnostics` | gateway | 0 | ❌ | 1Hz |
| `autosdv/{vehicle_id}/event` | gateway | 1 | ❌ | 이벤트 발생 시 |
| `autosdv/{vehicle_id}/streamer/status` | streaming | 1 | ✅ | 상태가 바뀔 때 + 5초마다. **정상 종료할 때도 `online:false`를 직접 발행** |

현재 브로커 ACL은 `vehicle` 계정에 다음을 허용하므로, 위 토픽을 그대로 쓸 수 있습니다.

- 발행: `autosdv/#`
- 구독: `autosdv/+/cmd/#` (서버 → 차량 명령용, 추후 사용)

### `status` payload

- JSON이 아닌 **UTF-8 문자열 `online` / `offline`**
- 이유: LWT는 연결할 때 미리 등록하는 메시지라 발행 시점의 시각 등을 넣을 수 없음

### Retain 정책

Retain은 브로커가 토픽별로 **마지막 메시지 1개를 저장해 두었다가, 나중에 구독한 쪽에 즉시 전달**하는 기능입니다.

| 토픽 | Retain | 이유 |
|---|---|---|
| `status` | ✅ | 연결·종료 시에만 발행됩니다. Retain이 없으면 차량이 먼저 접속한 뒤 서버가 구독했을 때 `online`을 알 수 없습니다 |
| `streamer/status` | ✅ | LWT(`online:false`)는 한 번만 발행됩니다. Retain이 없으면 그 이후 구독한 서버는 스트리머가 죽었다는 사실을 알 수 없습니다 |
| `diagnostics` | ❌ | 1Hz로 계속 오므로 1초 안에 최신 값을 받습니다. Retain하면 오래된 값이 최신처럼 보일 위험만 생깁니다 |
| `event` | ❌ | 일회성 이벤트이므로, 나중에 구독한 쪽에 과거 이벤트를 다시 보내면 안 됩니다 |

- **서버 판정 규칙:** retain 플래그가 켜진 채 전달된 메시지는 "마지막으로 알려진 값"으로만 표시하고, **신선도(STALE/OFFLINE) 타이머를 갱신하지 않습니다.**
  - 브로커가 재시작되면 LWT 없이 이전 `online`이 남아 있을 수 있기 때문입니다.
- 차량 ID를 폐기할 때는 해당 retain 토픽에 **빈 payload를 retain으로 발행**해서 저장된 메시지를 지웁니다.

## 10. Diagnostics JSON 스키마

**결정: 공통 틀은 서버가 제안, 세부 필드와 zone 매핑은 HPC가 제안**

### 예시 구조

```json
{
  "schema_version": "1.0",
  "vehicle_id": "a0000",
  "seq": 1234,
  "ts_ms": 1789000000000,
  "overall": "OK",
  "zones": [
    { "zone_id": "zone_1", "label": "FL ZCU", "status": "OK" },
    { "zone_id": "zone_2", "label": "FR ZCU", "status": "WARN" },
    { "zone_id": "zone_3", "label": "RL ZCU", "status": "OK" },
    { "zone_id": "zone_4", "label": "RR ZCU", "status": "STALE" }
  ],
  "cameras": [
    { "camera_id": "camera_01", "status": "OK" }
  ]
}
```

### 공통 규칙

| 항목 | 규칙 |
|---|---|
| 시간 | `ts_ms`는 HPC 시스템 시간의 epoch 밀리초(정수). 1단계에서는 시간 동기화를 요구하지 않으며 서버의 상태 판정에 사용하지 않음 |
| 상태 값 | `OK` \| `WARN` \| `ERROR` \| `STALE`. ROS `diagnostic_msgs` 레벨 0/1/2/3과 1:1 대응 |
| 필드 이름 | snake_case, 단위가 있는 값은 이름에 단위 포함 (예: `temp_c`, `speed_mps`) |
| `seq` | 메시지마다 1씩 증가 (유실 감지용). **프로세스가 재시작하면 0부터 다시 시작**하고, 서버는 `seq`가 줄어들면 차량 재시작으로 해석 |
| 버전 | 필드 추가는 minor 버전 증가, 서버는 모르는 필드를 무시. 필드 삭제·의미 변경은 major 버전 증가 |

합의가 끝난 스키마는 서버 저장소의 `schemas/`에 **JSON Schema로 저장하고 단일 원본**으로 씁니다.

### AutoSDV Dashboard 요약 필드

1단계에서는 `/ivn/diagnostics/status`의 전체 `DiagnosticArray`를 보내지 않고,
현재 `autosdv_gui`가 표시하는 상태만 요약해서 보냅니다.

| JSON 필드 | AutoSDV 기준 |
|---|---|
| `overall` | `IVN/Summary`의 상태 |
| `zones[].status` | 해당 ZCU의 `Network`와 `Runtime` 상태 중 우선순위가 높은 상태 |
| `cameras[].status` | `hardware_id=camera-image_01`인 진단 상태 (**카메라 입력** 상태) |

상태 우선순위는 AutoSDV GUI와 동일하게 `ERROR > STALE > WARN > OK`를 사용합니다.
`zones[].zone_id`와 표시 위치는 다음과 같이 고정합니다.

| `zone_id` | `label` | 위치 |
|---|---|---|
| `zone_1` | `FL ZCU` | Front Left |
| `zone_2` | `FR ZCU` | Front Right |
| `zone_3` | `RL ZCU` | Rear Left |
| `zone_4` | `RR ZCU` | Rear Right |

### `zones` 규칙

- **항상 4개를 모두 보냅니다.** 특정 ZCU 정보가 없으면 항목을 빼지 않고 `status: "STALE"`로 보냅니다.
  - 그래야 서버가 "ZCU 정보 없음"과 "메시지 형식 오류"를 구분할 수 있습니다.
- 서버는 `zone_id`를 키로 사용하고, `label`은 표시용으로만 씁니다.

원본 진단의 `items`와 `values`는 1단계 전송 대상에서 제외합니다.

## 11. 온라인 및 STALE 기준

**결정: HPC 초기안 동의, 판단 방식은 아래와 같음**

### STALE의 두 층위

| 구분 | 판정 주체 | 의미 | 우선순위 |
|---|---|---|---|
| **payload 안의 `STALE`** | 차량 | 차량 내부에서 진단 소스가 끊김 (예: 특정 ZCU 진단 미수신) | 차량 규칙 `ERROR > STALE > WARN > OK` |
| **서버의 STALE / OFFLINE** | 서버 | MQTT 메시지나 영상 자체가 서버에 오지 않음 | 마지막으로 받은 값(`ERROR` 포함)보다 **항상 우선** |

**Dashboard 표시 우선순위:** 서버 OFFLINE > 서버 STALE > payload 상태 값(차량 규칙 적용)

### 서버 판정 기준

| 대상 | 기준 | 서버 판단 방법 |
|---|---|---|
| Gateway OFFLINE | 10초 | `status=offline`(LWT 포함)을 받거나, gateway가 발행하는 메시지(`status`, `diagnostics`, `event`)가 **10초간 하나도 오지 않음** |
| 진단 STALE | 3초 | `diagnostics` 메시지가 3초간 오지 않음 |
| Streamer OFFLINE | 10초 | `streamer/status`에서 `online:false`(LWT 포함)를 받거나, `streamer/status`가 **10초간 오지 않음** (5초 주기 2회 누락) |
| 영상 STALE | 3초 | MediaMTX API에서 경로가 준비 상태가 아니거나 수신 바이트가 3초간 늘지 않음. 브라우저에서도 프레임이 3초간 갱신되지 않으면 STALE로 표시 |

- **서버는 메시지를 받은 시각을 기준으로** 판단합니다. 차량 시계가 틀어져도 오판하지 않기 위해서입니다. `ts_ms`는 차량이 기록한 참고 시각이며 상태 판정에는 사용하지 않습니다.
- retain으로 전달된 메시지는 타이머를 갱신하지 않습니다 ([9장 Retain 정책](#retain-정책)).

### 카메라 관련 상태의 표시 규칙

| 출처 | 의미 | Dashboard 표시 |
|---|---|---|
| `streamer/status`의 `streams[].state` + MediaMTX | **스트리밍(인코딩·WHIP 송출)** 상태 | 영상 타일의 **주 상태** |
| `diagnostics.cameras[].status` | **카메라 입력(센서/이미지 토픽)** 상태 | 영상 타일의 **배지** |

입력 상태가 `ERROR`이면 스트리밍 `error_code: CAMERA_UNAVAILABLE`과 같은 의미로 봅니다.

## 12. 오류 표현 방식

**결정: streaming 프로세스의 `streamer/status` 하나로 표현**

시그널링이 HTTP이므로 **WHIP HTTP 응답 코드를 공통 오류 코드로 변환**해서 `streamer/status`에 기록합니다.

### `streamer/status` 메시지

토픽: `autosdv/{vehicle_id}/streamer/status` (QoS 1, Retain)

```json
{
  "schema_version": "1.0",
  "vehicle_id": "a0000",
  "online": true,
  "seq": 42,
  "ts_ms": 1789000000000,
  "streams": [
    {
      "camera_id": "camera_01",
      "path": "autosdv/a0000/camera_01",
      "session_id": "3f6c1a2e-8f0b-4c1e-9a55-2b7d0e6f4a10",
      "state": "STREAMING",
      "error_code": null,
      "width": 640,
      "height": 480,
      "fps": 30,
      "bitrate_kbps": 1200
    }
  ]
}
```

LWT payload (연결 시 등록):

```json
{ "schema_version": "1.0", "online": false }
```

- `streams`는 카메라가 늘어날 경우를 대비한 배열입니다. 1단계에서는 `camera_01` 하나만 들어갑니다.
- `seq` 규칙은 [10장](#공통-규칙)과 같습니다.

### `streams[].state`

| 값 | 의미 |
|---|---|
| `STARTING` | 파이프라인 시작 중 |
| `CONNECTING` | WHIP 요청 또는 ICE 연결 진행 중 |
| `STREAMING` | 송출 중 |
| `RECONNECTING` | 재연결 대기 중 (backoff) |
| `STOPPED` | 의도적으로 중지 |
| `ERROR` | 복구 불가, 사람의 조치가 필요 |

### `streams[].error_code`

| 값 | 원인 |
|---|---|
| `AUTH_FAILED` | WHIP 401 / 403 |
| `PATH_REJECTED` | WHIP 404 |
| `SDP_REJECTED` | WHIP 400 / 406 / 415 |
| `SERVER_ERROR` | WHIP 5xx |
| `SIGNALING_TIMEOUT` | HTTP 요청 타임아웃 또는 연결 실패 |
| `ICE_FAILED` | ICE 연결 실패 |
| `CAMERA_UNAVAILABLE` | 카메라 입력 없음 |
| `ENCODER_ERROR` | 인코더 오류 |

정상 상태에서는 `error_code`가 `null`입니다.

---

## HPC 측 확인 결과

| # | 항목 | 결과 |
|---|---|---|
| 1 | GStreamer 버전 1.22 이상 여부 | **확인 완료: 1.24.2** |
| 2 | WHIP 송출 요소 사용 가능 여부 | **확인 완료: `whipclientsink` 지원** |
| 3 | 하드웨어 H.264 인코더 사용 가능 여부 | **확인 완료: `nvv4l2h264enc` 지원** |
| 4 | 전방 카메라 원본 해상도 | **확인 완료: 640×480 (4:3)** |
| 5 | 시간 기준 | **확정: `ts_ms`는 HPC 시스템 시간, 판정은 서버 수신 시각** |
| 6 | Diagnostics 요약 스키마 | **HPC 제안 완료: 10장 AutoSDV Dashboard 요약 필드 참고** |

## HPC 측 최종 확인 요청 (v3 변경분)

| # | 항목 | 장 |
|---|---|---|
| 1 | `camera/{camera_id}/status` 제거, `streamer/status` 구조와 LWT payload | 9, 12 |
| 2 | 프로세스별 Client ID와 LWT 설정 | 9 |
| 3 | `diagnostics` Retain 해제 | 9 |
| 4 | `status` payload 문자열 형식, `seq` 재시작 규칙 | 9, 10 |
| 5 | `zones` 4개 항상 전송 | 10 |
| 6 | WHIP 인증 방식 (`auth-token={user}:{pass}`) | 2 |

## 서버 측 후속 작업 (합의 후)

1. 토픽 규약과 JSON Schema를 `schemas/`에 커밋
2. MediaMTX 설치: 인증 계정, `webrtcAdditionalHosts`, `webrtcIPsFromInterfacesList`, `overridePublisher`, 방화벽 반영
3. 테스트 WHIP 송출로 서버 쪽 수신·브라우저 재생 검증
