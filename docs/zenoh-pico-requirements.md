# Zenoh-Pico 모드 구현 전 결정 사항

> 초기 설계 검토 기록이다. 현재 Linux 구현·필수 라이브러리 패치·운영 방법은
> [Pico 운영 문서](zenoh-pico-linux.md)를 참조한다. 아래 미검증 표시는 조사 당시 기준이다.

사용자 확정 범위: 현재 Linux ZCU에서 구현·비교하며 M코어 이식은 보류한다. 아래 M코어 관련 항목은 추후 참고용이며 현재 필수 요구가 아니다.

조사일: 2026-09-07. 공식 `zenoh-pico` **1.10.0**, commit `96006957fddef401c20c8c2d813c2a630b666974`의 소스와 현재 인접 `S32G3_ZCU` 저장소를 읽었다. 아래는 설계 검토이며 Pico ↔ bridge 통신 시험이나 M 코어 실행 결과가 아니다.

## 확인한 사실

| 항목 | 확인 결과 및 근거 |
|---|---|
| 플랫폼·전송 | Pico는 native C 라이브러리이며 Unix의 TCP 및 UDP unicast를 지원한다. FreeRTOS-Plus-TCP, Zephyr 등도 지원하지만 이것만으로 S32G3 M 코어 BSP가 지원된다는 의미는 아니다. [공식 README](https://github.com/eclipse-zenoh/zenoh-pico/blob/1.10.0/README.md) |
| UDP 신뢰성 | 1.10.0 수신 구현의 RELIABLE 경로에도 재전송 신뢰성 구현 전이라는 TODO가 있으며 현재 단조 증가 sequence만 보장한다. UDP에서 `RELIABLE` 선택만으로 손실 복구를 보장할 수 없다. [unicast rx.c](https://github.com/eclipse-zenoh/zenoh-pico/blob/1.10.0/src/transport/unicast/rx.c) |
| 분할·재조립 | 기본 `FRAG_MAX_SIZE=4096`, `BATCH_UNICAST_SIZE=2048`. `FRAG_MAX_SIZE`는 **수신 재조립** 한도다. 송신은 확장 가능한 fragmentation buffer를 사용하므로 43,200바이트 이미지 송신이 4096바이트 기본값 때문에 곧바로 불가능하다는 해석은 잘못이다. 이미지 CDR에는 픽셀 외 헤더도 포함된다. [CMake](https://github.com/eclipse-zenoh/zenoh-pico/blob/1.10.0/CMakeLists.txt), [수신 설정 설명](https://github.com/eclipse-zenoh/zenoh-pico/blob/1.10.0/docs/config.rst), [송신 구현](https://github.com/eclipse-zenoh/zenoh-pico/blob/1.10.0/src/transport/common/tx.c) |
| DSCP | Pico 1.10.0 UDP endpoint 설정은 `iface/tout/join`, TCP는 `tout`만 정의한다. Rust Zenoh의 `#dscp=...`를 Pico에 그대로 넘기는 설계는 사용할 수 없다. [UDP 설정](https://github.com/eclipse-zenoh/zenoh-pico/blob/1.10.0/include/zenoh-pico/link/config/udp.h), [TCP 설정](https://github.com/eclipse-zenoh/zenoh-pico/blob/1.10.0/include/zenoh-pico/link/config/tcp.h) |
| 실행 task | 1.10.0의 multithread 설정에서는 세션 생성 시 executor가 read·lease·keepalive 등을 자동 처리한다. 기존 개별 start-read/start-lease API는 deprecated다. Single-thread에서는 `zp_spin_once`를 주기적으로 호출해야 한다. [옵션](https://github.com/eclipse-zenoh/zenoh-pico/blob/1.10.0/include/zenoh-pico/api/types.h), [API](https://github.com/eclipse-zenoh/zenoh-pico/blob/1.10.0/include/zenoh-pico/api/primitives.h) |
| 초기 연결 | client `z_open`은 연결 가능한 transport가 있어야 성공한다. exit-on-failure 옵션만으로 라우터 없는 초기 실행이 성공하지 않는다. [공식 설정 설명](https://github.com/eclipse-zenoh/zenoh-pico/blob/1.10.0/docs/config.rst) |
| 포팅 경계 | platform profile에서 시스템 함수와 TCP/UDP 구현을 구성할 수 있다. RTOS·socket stack·시간·스레드·메모리 계층을 구체적으로 선정해야 한다. [공식 플랫폼 문서](https://github.com/eclipse-zenoh/zenoh-pico/blob/1.10.0/docs/platforms.rst) |

현재 ZCU의 `CMakeLists.txt`는 `fastdds/cyclonedds/zenoh`만 선택한다. `src/backend/zenoh`는 zenoh-cpp/zenoh-c, Fast-CDR 및 rmw_zenoh 전용 key/attachment를 사용한다. Pico 브릿지 모드에서는 전용 backend를 추가하고 브릿지용 key 및 CDR 계약을 사용해야 한다. 기존 rmw_zenoh 규약을 그대로 복사하면 안 된다. Fast-CDR는 직렬화 라이브러리이며 FastDDS transport를 제거한다는 이유만으로 Linux에서 제거할 필요는 없다.

## 권장 결정안

1. **단계 구분:** 먼저 S32G3 A 코어 Linux에서 기존 서비스와 동일한 `zenoh_bridge_pico` 모드를 구현한다. M 코어는 별도 RTOS 포팅 단계로 정의한다. Linux 실행파일의 zenoh-c를 Pico로 교체한 것만으로 M 코어 준비 완료로 판정하지 않는다.
2. **버전 고정:** Pico·브릿지·Zenoh 라이브러리 버전 및 build flags를 실험 manifest에 기록한다. 동일 `1.10.0`은 출발 후보이며 브릿지와의 CDR·key·fragmentation 호환 검증을 대신하지 않는다.
3. **전송:** 기본 상호운용 시험은 TCP로 먼저 수행한다. UDP 카메라는 별도 실험 축으로 둔다. 제어 전달 요구가 재전송이면 TCP를 우선하고, 주기 최신 명령의 지연·유효기간 요구라면 UDP와 sequence/age/watchdog를 함께 설계한다. 애플리케이션 처리 완료가 요구되면 TCP에서도 별도 ACK가 필요하다. 각 모드의 비교에서는 TCP/UDP 차이를 기록하고 섞어서 미들웨어 효과로 해석하지 않는다.
4. **버퍼:** batch는 경로 MTU를 고려해 정한다. 1500 MTU에서 2048 기본 batch를 그대로 사용하면 IP fragmentation이 생길 수 있으므로 예를 들어 1400 이하를 시작 후보로 잡고 패킷 캡처로 확인한다. Pico 수신 최대 메시지가 제어만인지 이미지도 포함하는지에 따라 FRAG_MAX_SIZE를 정한다. 송신 이미지 크기만 보고 수신 버퍼를 무조건 키우지 않는다. 이미지 최대 직렬화 크기, 순간 heap, HPC 재조립 한도도 함께 측정한다.
5. **DSCP:** Linux 시험은 포트별 OS egress marking 또는 작은 socket 계층 확장 중 하나를 선택한다. M 코어는 선택한 네트워크 stack의 IP TOS/DSCP 지원으로 별도 구현한다. 세션/포트를 카메라와 제어로 나누면 flow 분류가 쉬우나 세션 수가 RAM과 CPU를 늘리므로 함께 기록한다.
6. **실행·복구:** Linux는 자동 executor를 사용하고 callback은 짧게 유지한다. M 코어는 executor task와 cooperative spin 중 하나를 선택해 스택 크기, 우선순위, 주기, 최장 callback 시간을 정한다. 초기 라우터 부재·연결 단절·브릿지 재시작 각각에 대해 재연결 및 pub/sub 재선언과 허용 복구 시간을 시험한다.
7. **M 코어 범위:** 정확한 보드·M 코어·RTOS·TCP/IP stack·RAM/flash 예산, Ethernet 직접 소유 또는 A 코어 IPC 경유 여부, 포함할 센서/제어 서비스와 최대 payload를 먼저 확정한다. 카메라 수신 코드·Linux 장치 API·C++ STL·Fast-CDR 등 현재 Linux 서비스 의존성을 모두 M 코어로 가져가는 것을 기본값으로 삼지 않는다. M 코어용 bounded CDR 인코더/디코더가 필요하면 Linux와 golden byte 검증을 공유한다.

## 구현 후 통과해야 할 실험

- `Image`, `DiagnosticArray`, `MotionCommand` 실제 CDR에 대해 양방향 byte/field 일치, 경계 크기, 잘못된 길이 거부를 확인한다. 대형 이미지는 43,200바이트 픽셀과 최대 계획 해상도 모두 시험한다.
- 동일 workload·토픽 수·Hz·payload·MTU·전송·DSCP에서 ZCU CPU/RSS/PSS/스레드, HPC 내부 노드와 router/bridge의 합계 CPU/메모리, wire bytes와 packet count를 측정한다. M 코어에서는 RSS 대신 heap 최고점, task stack 여유, 정적 RAM/flash, deadline miss를 측정한다.
- sequence 기반 drop/중복/재정렬, p50/p95/p99/max 지연, frame age, 장시간 메모리 증가를 기록한다. 단방향 지연은 시간 동기화 정확도를 먼저 측정한다.
- 다중 ZCU 동시 이미지 송신 중 제어 지연, 손실/재정렬 주입, link down/up, bridge/router 재시작, ZCU 부팅 순서 변경을 시험한다.
- Pico가 가볍다는 일반 설명과 별개로 **서비스 전체**가 기존 zenoh-c보다 자원을 적게 쓰는지 측정한다. 서비스별 세션·buffer·직렬화 복사 비용이 결과를 바꿀 수 있다.

미검증: Pico 1.10.0 ↔ 현재 HPC bridge 1.10.0 실통신, 실제 DSCP marking, 4대 ZCU 부하, M 코어 BSP/RTOS 포팅. 이 문서는 위 항목의 성공을 주장하지 않는다.
