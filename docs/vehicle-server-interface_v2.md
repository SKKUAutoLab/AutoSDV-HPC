# 차량(HPC) ↔ Server PC 인터페이스 합의안

| 항목 | 내용 |
|---|---|
| 상태 | **Draft (HPC 1차 검토 반영, 서버 측 확인 대기)** |
| 작성일 | 2026-09-17 |
| 기준 버전 | MediaMTX v1.21.0, Mosquitto 2.1 (MQTT v5) |
| 대상 | 전방 카메라 영상 스트리밍, 차량 상태/진단 정보 전송 |

## 0. 요약

> 차량은 H.264(Constrained Baseline, 키프레임 1~2초)로 **WHIP `POST http://100.74.147.75:8889/autosdv/{vehicle_id}/{camera_id}/whip`** 에 송출합니다.
> STUN/TURN은 쓰지 않습니다.
> 상태와 진단은 **MQTT v5 `100.74.147.75:1883`**, Keep Alive 5초, [9장](#9-mqtt-broker-정보)의 토픽·QoS·Retain 표를 따릅니다.
> 오류는 camera status의 `state`와 `error_code`로 통일합니다.

```
[차량 HPC] ──WebRTC (WHIP)──▶ MediaMTX (Server PC) ──WebRTC (WHEP)──▶ 브라우저 (Dashboard)
     │
     └──────MQTT v5─────────▶ Mosquitto ─▶ Backend (FastAPI) ─▶ TimescaleDB / Dashboard
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
| 요청 | `POST`, `Content-Type: application/sdp`, 본문은 SDP Offer |
| 응답 | `201 Created`, 본문은 SDP Answer, `Location` 헤더에 세션 URL |
| ICE 추가 | `PATCH {Location}` (trickle). 후보를 모두 모은 뒤 한 번에 Offer를 보내도 됨 |
| 종료 | `DELETE {Location}` |
| 인증 | HTTP `Authorization` 헤더. 차량 계정은 `autosdv/` 경로에만 publish 허용. 계정 정보는 서버 설치 후 별도 전달 |
| 차량 등록 | 별도 절차 없음. **경로 이름이 곧 식별자**이고, 온라인 여부는 MQTT로 판단 |

- 연결은 **Tailscale 경유**이고, WireGuard가 암호화하므로 HTTP로 운용합니다.
- **HPC 확인 필요:** GStreamer를 쓴다면 **1.22 이상**과 `whipclientsink`(gst-plugins-rs)가 필요합니다. JetPack 기본 패키지에는 없을 수 있습니다.
  - 확인 명령: `gst-inspect-1.0 whipclientsink`

## 3. SDP 생성 주체

**결정: 차량이 Offer를 만들고 MediaMTX가 Answer를 반환**

- WHIP 표준이 이 방식입니다.
- 브라우저도 WHEP로 똑같이 Offer를 보내며, 이 부분은 서버 쪽에서 처리합니다.

## 4. 영상 코덱

**결정: H.264 우선**

| 항목 | 값 | 이유 |
|---|---|---|
| 코덱 | H.264 | 하드웨어 인코딩, 모든 브라우저에서 재생 가능 |
| 프로파일 | **Constrained Baseline (B-frame 없음)** | B-frame이 들어가면 브라우저가 재생하지 못함 (MediaMTX 문서에 명시) |
| 키프레임 간격 | **1~2초** | 대시보드를 새로 연 사람은 다음 키프레임부터 영상이 보임 |
| SPS/PPS | **키프레임마다 반복 삽입** | 중간에 접속한 사람도 디코딩 가능. 예: `nvv4l2h264enc insert-sps-pps=1` |

- VP8은 Thor에서 H.264 하드웨어 인코딩이 안 될 때만 대안으로 씁니다. 소프트웨어 인코딩이라 CPU 부하가 큽니다.
- **HPC 확인 필요:** `gst-inspect-1.0 nvv4l2h264enc`

## 5. 영상 제약 조건

**결정: 초기값 동의, 아래 두 가지 조정 제안**

| 항목 | HPC 초기안 | 서버 제안 |
|---|---|---|
| 해상도 | 640×480 | **640×480으로 확정** (Camera 01 실제 입력 해상도) |
| 프레임레이트 | 최대 30fps | 동의 |
| 비트레이트 | 300kbps~2.5Mbps | **상한 1.5Mbps로 시작**, 필요하면 2.5Mbps까지 상향 |

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
- `vehicle_id`, `camera_id` 권장 형식: 영문 소문자, 숫자, `-`만 사용

## 8. WebRTC 재연결 규칙

| 항목 | 값 |
|---|---|
| 연결 끊김 판단 | ICE/PeerConnection 상태가 `failed`면 즉시, `disconnected`가 **5초** 넘게 지속되면 끊김으로 봄. WHIP `POST`가 실패해도 마찬가지 |
| 재시도 간격 | 1 → 2 → 4 → 8 → 최대 10초, ±20% 랜덤 지연. 30초 이상 안정적으로 유지되면 간격 초기화 |
| 재연결 절차 | 이전 세션에 `DELETE`(실패해도 무시) → **새 PeerConnection 생성** → 새 `POST` |
| `session_id` | 재연결을 시도할 때마다 **차량이 새 UUID를 생성**해서 로그와 MQTT camera status에 기록 |
| 서버 동작 | `overridePublisher: true`로 설정해서, 새 세션이 들어오면 남아 있던 이전 세션을 대체 |

## 9. MQTT Broker 정보

### 접속 정보

| 항목 | 값 |
|---|---|
| 주소:포트 | `100.74.147.75:1883` (Tailscale) |
| 프로토콜 | MQTT **v5** 권장 |
| 인증 | 사용자 `vehicle`, 비밀번호는 별도 전달 |
| TLS | 사용하지 않음 (Tailscale이 암호화) |
| Client ID | `autosdv-{vehicle_id}-{component}` (1단계: `autosdv-a0000-gateway`). **같은 ID로 접속하면 기존 연결이 끊기므로** 연결마다 고유해야 함 |
| Keep Alive | **5초**. 브로커는 1.5배인 약 7.5초 안에 끊김을 감지하고 LWT를 발행하므로, 10초 OFFLINE 기준을 맞출 수 있음 |
| 세션 | Clean Start, Session Expiry 0 |

### 토픽 / QoS / Retain

| 토픽 | QoS | Retain | 주기 |
|---|---|---|---|
| `autosdv/{vehicle_id}/status` (`online`/`offline`, LWT) | 1 | ✅ | 연결·종료 시. **정상 종료할 때도 `offline`을 직접 발행** (LWT는 정상 종료 시 발행되지 않음) |
| `autosdv/{vehicle_id}/diagnostics` | 0 | ✅ | 1Hz |
| `autosdv/{vehicle_id}/camera/{camera_id}/status` | 1 | ✅ | 상태가 바뀔 때 + 5초마다 |
| `autosdv/{vehicle_id}/event` | 1 | ❌ | 이벤트 발생 시 |

현재 브로커 ACL은 `vehicle` 계정에 다음을 허용하므로, 위 토픽을 그대로 쓸 수 있습니다.

- 발행: `autosdv/#`
- 구독: `autosdv/+/cmd/#` (서버 → 차량 명령용, 추후 사용)

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
    {
      "zone_id": "zone_1",
      "label": "FL ZCU",
      "status": "WARN"
    }
  ],
  "cameras": [
    {
      "camera_id": "camera_01",
      "status": "OK"
    }
  ]
}
```

### 공통 규칙

| 항목 | 규칙 |
|---|---|
| 시간 | `ts_ms`는 HPC 시스템 시간의 epoch 밀리초(정수). 1단계에서는 시간 동기화를 요구하지 않으며 서버의 상태 판정에 사용하지 않음 |
| 상태 값 | `OK` \| `WARN` \| `ERROR` \| `STALE`. ROS `diagnostic_msgs` 레벨 0/1/2/3과 1:1 대응 |
| 필드 이름 | snake_case, 단위는 이름에 포함 (예: `temp_c`, `speed_mps`) |
| `seq` | 메시지마다 1씩 증가 (유실 감지용) |
| 버전 | 필드 추가는 minor 버전 증가, 서버는 모르는 필드를 무시. 필드 삭제·의미 변경은 major 버전 증가 |

합의가 끝난 스키마는 서버 저장소의 `schemas/`에 **JSON Schema로 저장하고 단일 원본**으로 씁니다.

### AutoSDV Dashboard 요약 필드

1단계에서는 `/ivn/diagnostics/status`의 전체 `DiagnosticArray`를 보내지 않고,
현재 `autosdv_gui`가 표시하는 상태만 요약해서 보냅니다.

| JSON 필드 | AutoSDV 기준 |
|---|---|
| `overall` | `IVN/Summary`의 상태 |
| `zones[].status` | 해당 ZCU의 `Network`와 `Runtime` 상태 중 우선순위가 높은 상태 |
| `cameras[].status` | `hardware_id=camera-image_01`인 진단 상태 |

상태 우선순위는 AutoSDV GUI와 동일하게 `ERROR > STALE > WARN > OK`를 사용합니다.
`zones[].zone_id`와 표시 위치는 다음과 같이 고정합니다.

| `zone_id` | `label` | 위치 |
|---|---|---|
| `zone_1` | `FL ZCU` | Front Left |
| `zone_2` | `FR ZCU` | Front Right |
| `zone_3` | `RL ZCU` | Rear Left |
| `zone_4` | `RR ZCU` | Rear Right |

원본 진단의 `items`와 `values`는 1단계 전송 대상에서 제외합니다.

## 11. 온라인 및 STALE 기준

**결정: HPC 초기안 동의, 판단 방식은 아래와 같이 제안**

| 대상 | 기준 | 서버 판단 방법 |
|---|---|---|
| Gateway OFFLINE | 10초 | `status=offline`(LWT 포함)을 받거나, 해당 차량에서 **어떤 메시지도 10초간 오지 않음** |
| 진단 STALE | 3초 | `diagnostics` 메시지가 3초간 오지 않음 |
| 영상 STALE | 3초 | MediaMTX API에서 경로가 준비 상태가 아니거나 수신 바이트가 3초간 늘지 않음. 브라우저에서도 프레임이 3초간 갱신되지 않으면 STALE로 표시 |

- **서버는 메시지를 받은 시각을 기준으로** 판단합니다. 차량 시계가 틀어져도 오판하지 않기 위해서입니다. `ts_ms`는 차량이 기록한 참고 시각이며 상태 판정에는 사용하지 않습니다.
- **표시 우선순위:** OFFLINE > STALE > 실제 상태 값

## 12. 오류 표현 방식

**결정: camera status 하나로 통일**

시그널링이 HTTP이므로 **WHIP HTTP 응답 코드를 공통 오류 코드로 변환**해서 MQTT camera status에 기록합니다.

### camera status 메시지

토픽: `autosdv/{vehicle_id}/camera/{camera_id}/status`

```json
{
  "schema_version": "1.0",
  "ts_ms": 1789000000000,
  "session_id": "3f6c1a2e-8f0b-4c1e-9a55-2b7d0e6f4a10",
  "state": "STREAMING",
  "error_code": null,
  "width": 640,
  "height": 480,
  "fps": 30,
  "bitrate_kbps": 1200
}
```

### `state`

| 값 | 의미 |
|---|---|
| `STARTING` | 파이프라인 시작 중 |
| `CONNECTING` | WHIP 요청 또는 ICE 연결 진행 중 |
| `STREAMING` | 송출 중 |
| `RECONNECTING` | 재연결 대기 중 (backoff) |
| `STOPPED` | 의도적으로 중지 |
| `ERROR` | 복구 불가, 사람의 조치가 필요 |

### `error_code`

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

## HPC 측 확인 요청 사항

| # | 항목 | 확인 방법 |
|---|---|---|
| 1 | GStreamer 버전 1.22 이상 여부 | `gst-launch-1.0 --version` |
| 2 | WHIP 송출 요소 사용 가능 여부 | `gst-inspect-1.0 whipclientsink` |
| 3 | 하드웨어 H.264 인코더 사용 가능 여부 | `gst-inspect-1.0 nvv4l2h264enc` |
| 4 | 전방 카메라 원본 해상도 | **확인 완료: 640×480 (4:3)** |
| 5 | 시간 기준 | **확정: `ts_ms`는 HPC 시스템 시간, 판정은 서버 수신 시각** |
| 6 | Diagnostics 요약 스키마 | **HPC 제안 완료: 10장 AutoSDV Dashboard 요약 필드 참고** |

## 서버 측 후속 작업 (합의 후)

1. 토픽 규약과 JSON Schema를 `schemas/`에 커밋
2. MediaMTX 설치: 인증 계정, `webrtcAdditionalHosts`, `overridePublisher`, 방화벽 반영
3. 테스트 WHIP 송출로 서버 쪽 수신·브라우저 재생 검증
