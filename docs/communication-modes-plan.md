# 네 가지 통신 모드 구현·실험 검토

> 이 문서는 초기 구현 계획이다. 2026-09-10 현재 공통 전환과 CycloneDDS·Zenoh·Bridge·Linux Pico가 구현되었다. 최신 Bridge 범위와 검증 한계는 [구현 기록](zenoh-bridge-validation-2026-09-10.md)을 참조한다. Linux Pico 구현은 [Pico 운영 문서](zenoh-pico-linux.md)를 참조한다.
검토일: 2026-09-07. 사용자 범위 확정 반영: ADAS_load/BE_load 이주 제외(삭제 허용, 이번 문서 갱신에서 삭제하지 않음), Linux Pico 구현 포함, M코어 이식 보류. 상태: 구현 전 범위·결정사항 검토. 이 문서는 권장 설계이며 모드 구현 완료를 의미하지 않는다.

## 1. 모드 정의

| 모드 ID (제안) | HPC ROS 노드 | HPC 통신 중계 | ZCU 구현 | IVN 프로토콜 |
| --- | --- | --- | --- | --- |
| `cyclonedds` | `rmw_cyclonedds_cpp` | 없음 | CycloneDDS | DDS/RTPS |
| `zenoh` | `rmw_zenoh_cpp` | rmw_zenoh용 router | zenoh-c/cpp + rmw_zenoh 규약 | Zenoh |
| `zenoh_bridge` | `rmw_cyclonedds_cpp`, loopback 격리 | `zenoh-bridge-ros2dds` 카메라·제어 분리 | zenoh-c/cpp + ros2dds 규약 | Zenoh |
| `zenoh_bridge_pico` | `rmw_cyclonedds_cpp`, loopback 격리 | 위와 같은 bridge | zenoh-pico + ros2dds 규약 | Zenoh |

세 번째와 네 번째 모드는 같은 HPC 구성을 공유한다. Pico를 HPC RMW로 선택하는 방식이 아니다. 네 모드를 별도 앱으로 복제하지 않고 기존 ZCU `include/transport/transport.hpp` 경계를 유지한다. ZCU의 빌드 시 백엔드 선택과 사용자 실행 시 모드 선택을 구분한다.

현재 모드 선택은 HPC `src/launch_pkg/launch_pkg/rmw_profile.py`와 ZCU `scripts/select_middleware.sh`로 분리돼 있다. ZCU CMake는 fastdds/cyclonedds/zenoh를 지원하며 bridge/pico 분기는 없다. HPC도 pico 모드와 시스템 전체 전환 진입점은 없다. 양쪽 기본값은 FastDDS다.

## 2. 먼저 정할 범위

- 기본 이주 대상: 이미지 5개, MotionCommand 제어, DiagnosticArray 헬스, HPC GUI/인지/판단/안전감시/진단 노드.
- 사용자 확정: `../ADAS_load`, `../BE_load`는 이주·검증 대상에서 제외한다. 삭제는 허용됐으며, 향후 부하 실험이 필요하면 별도 프로그램을 새로 구현한다. 이번 문서 갱신에서는 실제 삭제를 수행하지 않았다.
- 기존 부하 프로그램의 트래픽을 네 모드의 기본 비교에 포함하지 않는다. 향후 새 부하 발생기를 사용하는 실험은 구현·부하량·프로토콜을 별도로 기록한다.
- `/topic_status_signal`은 ZCU CMake에서 `ZCU_ENABLE_LEGACY_STATUS_PUBLISHER=OFF`이며 켜는 경로도 FastDDS 전용이다. 실제 기능 요구가 있으면 네 모드 지원을 추가하고, 없으면 필수 검증 토픽에서 제외한다.
- FastCDR 직렬화 라이브러리를 사용하는 것은 FastDDS 통신 런타임을 사용하는 것과 다르다. 완전 이주의 필수 판정은 실행 프로세스/통신 경로 기준으로 하고, 생성 코드/직렬화 라이브러리 제거는 Pico·M코어 이식 요구와 별도로 결정한다.
- 권장: FastDDS는 전환 검증 동안 명시적 레거시 복구 모드로만 보존하고, 기본값과 정식 실험 모드는 요청한 네 가지로 전환한다. 코드까지 삭제하는 시점은 기능 동등성 검증 이후로 둔다.

## 3. 모드별 구현 항목

### CycloneDDS

기존 HPC 프로파일, ZCU CycloneDDS-CXX 백엔드를 재사용한다.

필요 작업:

1. GUI를 거치지 않는 launch/CLI/systemd 및 별도 native 프로그램을 포함해 실제 RMW/백엔드 적용을 점검한다.
2. ROS 토픽·DDS 타입·IDL/CDR 레이아웃의 단일 기준을 정하고 Image, MotionCommand, DiagnosticArray 동등성을 검증한다.
3. Domain ID, HPC/ZCU 주소, participant index, unicast discovery 범위, 메시지 크기, DSCP를 노드 수와 일치시킨다.
4. FastDDS 기본값/서비스/부하 프로그램 잔존 경로를 정리한다. 빌드 의존성과 실제 실행 의존성을 따로 확인한다.

결정: 완전 이주에 부하 프로그램과 legacy status 포함 여부, 기본 Domain ID, 최대 노드 수, 멀티캐스트 정책, 고정 participant index 유지 범위.

### Zenoh

HPC는 rmw_zenoh, ZCU는 현재 zenoh-c/cpp 백엔드다. 일반 Zenoh put만으로 rmw_zenoh와 호환되는 것은 아니다. rmw_zenoh의 keyexpr, attachment, liveliness 규약을 함께 맞춘다.

필요 작업:

1. 설치된 rmw_zenoh/zenoh-c/cpp 버전 조합과 규약을 고정한다. 현재 코드의 타입 해시 상수 및 liveliness 포맷에 대한 회귀 검증을 둔다.
2. Domain ID 변경, 모든 메시지 타입, ROS service/action을 쓰는 HPC 내부 경로, GUI/ROS CLI를 검증한다.
3. ZCU 카메라/제어 세션을 분리하고 정적 endpoint를 사용한다. HPC 내부 peer 연결 및 SHM 사용은 실제 연결/캡처로 확인하고 실험 조건에 기록한다.
4. router 시작 전 노드 기동, router 재시작, 링크 단절 후 재연결을 검증한다.

결정: UDP/TCP 실험 조건, 내부 SHM 사용 여부, 설치 패키지 고정 방법, 프로토콜 변경 시 호환성 검사 범위.

근거: [rmw_zenoh Jazzy README](https://github.com/ros2/rmw_zenoh/blob/jazzy/README.md), [공식 설계](https://github.com/ros2/rmw_zenoh/blob/jazzy/docs/design.md). 현재 Jazzy 브랜치 문서가 설치된 구버전과 동일함을 가정하지 않는다.

### Zenoh-DDS Bridge

필요 작업:

1. ZCU에 ros2dds wire 규약을 추가한다: 기본 namespace에서 선행 `/`를 제거한 토픽 keyexpr, DDS encapsulated CDR. 기존 rmw_zenoh 모드는 보존한다.
2. 기존 rmw_zenoh attachment/liveliness 생성과 GraphProbe 파싱을 브릿지 모드에서 분리한다. 헬스의 producer 상태·실제 데이터 신선도를 주 판정으로 두고, DDS publisher 존재를 원격 ZCU 생존으로 취급하지 않는다.
3. HPC와 bridge에 같은 loopback DDS 프로파일을 강제한다. `run_zenoh_bridge.sh`가 상속된 `CYCLONEDDS_URI`를 우선하는 기존 문제를 수정한다. discovery 관련 환경변수도 전환 시 명시적으로 관리한다.
4. allow 목록은 요구 토픽 및 방향으로 제한한다. 외부 service/action이 실제 필요한 경우만 추가한다. 기본 7447 카메라, 7448 제어·헬스 분리를 유지한다.
5. 원격 토큰 없이 로컬 구독자로 route를 만드는 정책이면, 구독자 준비 전 송신 데이터 유실과 구독자 제거/재시작 시 route 생성을 시험한다.
6. bridge ROS domain/ROS distro/실행 버전을 검증한다. domain은 Zenoh key에 자동 격리를 제공하지 않으므로 동시 다중 실험은 별도 namespace 또는 포트/네트워크로 분리한다.

검토에서 확인한 차이:

- BEST_EFFORT·depth 1 로컬 이미지 구독자로 만든 bridge writer가 실제로 RELIABLE·depth 1이었다. QoS는 이름이나 주석이 아닌 실제 endpoint 정보로 판정한다.
- 제어 bridge의 `reliable_routes_blocking: true` 설명은 제어가 RELIABLE이라는 전제지만 현재 safety supervisor 발행 QoS는 BEST_EFFORT·depth 1이다.
- ros2dds MP/MS liveliness 토큰만으로 원격 ZCU의 ROS 노드 이름이 복원되지 않는다. 생성된 endpoint는 bridge 노드 이름으로 등록된다.

근거: [ros2dds 1.10.0](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/tree/1.10.0), [QoS 변환](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/blob/1.10.0/zenoh-plugin-ros2dds/src/qos_helpers.rs), [ROS graph 등록](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/blob/1.10.0/zenoh-plugin-ros2dds/src/ros_discovery.rs).

### Zenoh-DDS Bridge + Zenoh-Pico

권장 단계:

1. 현재 S32G3 Linux에서 zenoh-pico C API 백엔드를 구현해 일반 zenoh-c 브릿지 모드와 같은 메시지·HPC 설정·하드웨어로 비교한다.
2. Linux에서 이미지·제어·헬스의 기능 동등성과 자원 사용을 검증한다.
3. 사용자 확정: M코어 이식은 현재 범위에서 제외한다. 아래 M코어 관련 항목은 추후 논의용 참고이며 이번 구현의 선결 조건이나 완료 조건이 아니다.

검토/결정 항목:

- 대상 SoC/보드와 정확한 M코어, RTOS/bare-metal, Ethernet 직접 접근 또는 A코어 경유 IPC, CAN 접근 방법.
- RAM/flash/스택/힙 상한, 동적 할당 허용 여부, 세션 수와 버퍼 합계.
- 같은 Image/CDR를 유지할 최대 직렬화 크기와 FPS, fragmentation/reassembly 제한. UDP batch 크기와 최대 애플리케이션 메시지 크기는 별개다.
- 제어/헬스는 작은 payload지만 DiagnosticArray의 문자열/배열은 가변 길이다. 요소 수와 문자열 길이에 상한이 필요하다.
- 현재 ZCU는 C++17, FastCDR, Linux pthread 및 장치/파일 시스템에 의존한다. Pico 라이브러리 교체만으로 이러한 의존성이 사라지지 않는다.
- Pico 1.10.0 UDP의 `RELIABLE`은 재전송 보장이 아니다. 최신 주기 명령 UDP와 timeout/sequence 정책을 유지할지, TCP 기반 제어로 통일할지 결정한다. 모드마다 다른 전송을 쓰고 이를 미들웨어 단독 효과로 해석하지 않는다.
- Pico 1.10.0 기본 수신 재조립 상한은 4096바이트, unicast batch는 2048바이트다. 송신 이미지 43,200바이트가 곧바로 불가능하다는 뜻은 아니다. 송신 peak heap과 실제 수신 메시지 상한을 따로 검증한다.
- DSCP endpoint 설정이 zenoh-c와 같다고 가정하지 않는다. Pico의 해당 버전/플랫폼 socket 코드 또는 Linux tc 규칙으로 마킹을 구현하고 캡처한다.
- M코어에서 이미지까지 처리할지는 RAM/네트워크/DMA 예산으로 결정한다. Linux 단계에서는 네 모드의 동일 서비스 비교를 먼저 수행하고, M코어에서 제어·헬스만 가능하다면 축소된 서비스 범위를 별도 결과로 명시한다.

세부 primary-source 조사: [Zenoh-Pico 검토](zenoh-pico-requirements.md).

## 4. 공통 모드 전환

제안 인터페이스: `select_communication_mode <cyclonedds|zenoh|zenoh_bridge|zenoh_bridge_pico>` (아직 구현되지 않은 명령).

전환은 프로세스 재시작 방식으로 구현한다. 실행 중 ROS context의 RMW만 바꾸는 방식은 사용하지 않는다.

1. 선택한 모드의 HPC 패키지·ZCU 4대 바이너리·설정·버전·포트 준비 여부를 먼저 확인한다.
2. 구동 명령을 중립화하고 기존 앱/중계 프로세스를 정상 종료한다. 기존 ROS daemon과 실험 소유 QoS 정책을 정리한다.
3. 각 장치에 모드 설정을 적용한다. ROS/RMW/profile/domain, middleware-specific environment와 DSCP 정책을 일관되게 설정한다.
4. 필요 router/bridge와 HPC 수신 노드를 기동하고 준비를 확인한 다음 ZCU를 시작한다. 고정 sleep만으로 준비 판정하지 않는다.
5. 4대 모두의 적용 모드/버전, 실제 이미지·헬스·제어 경로를 확인한 뒤 선택 완료로 표시한다.
6. 일부 실패 시 혼합 모드를 완료 상태로 표시하지 않는다. 구동은 비활성 상태로 유지하고 실패 장치를 명시한다. 자동 rollback 여부는 결정사항이다.
7. 마지막 성공 모드를 재부팅 후 복원한다. 런타임 선택 파일과 미리 빌드된 바이너리를 사용해 모드 전환 때마다 빌드하지 않는다.

외부 프로세스 하나가 자기 부모 shell의 환경을 바꿀 수 없으므로, 선택 명령이 서비스/GUI 재기동까지 소유하거나 이후 실행기가 영속 모드 파일을 읽도록 해야 한다. GUI에 모드 선택을 넣어도 외부 전환 도구를 호출하고 GUI/ROS context를 재시작하는 구조가 필요하다.

구현 재사용 지점: HPC `rmw_profile.py`, router/bridge launcher, `network_qos/autosdv_dscp_qos.py`; ZCU CMake 백엔드 분기, `Transport` 인터페이스, `select_middleware.sh`, zone별 설정, systemd health 실행.

## 5. 실험 전에 통일할 정책

| 항목 | 권장 출발점 / 결정 필요 |
| --- | --- |
| 이미지 | 현재 동일 입력/인코딩/FPS/최대 크기, 최신 프레임 우선; 요청 QoS와 실제 QoS 모두 기록 |
| 연속 제어 | 현재 50 ms 주기/BEST_EFFORT/depth 1을 기준선으로 유지하되, 수신 timeout 및 오래된/중복 명령 처리 정책 확정 |
| 신뢰성 | ROS/DDS QoS, Zenoh transport reliability, congestion drop/block은 서로 다른 설정으로 기록. RELIABLE로 바꾸는 실험은 네 모드에 일관되게 정의 |
| 헬스 | 현재 2 Hz 기준, zone별 heartbeat 신선도 사용. graph 존재만으로 정상 판정 금지 |
| MTU | IVN 동일 MTU, 현재 1400 batch/message 출발점; IP fragmentation 여부를 실제 캡처로 확인 |
| DSCP/TSN | 카메라 CS5, 제어 CS6 기준. 헬스 공유 포트를 유지할지 3번째 포트로 분리할지 결정; 실제 TOS/큐/GCL 기록 |
| 내부 SHM | 실험 전체에서 고정하거나 별도 인자로 명시. 내부 최적화 변경과 wire 변경 효과 혼동 금지 |
| 부하 | idle/정상/포화/손실·단절 조건을 분리하고 부하 발생기 구현·대역폭을 명시 |

RELIABLE 전송이 명령의 유효시간을 보장하지는 않는다. MotionCommand는 현재 frame_id와 구동 값만 갖고 있으므로, source timestamp/validity를 추가할지 수신측 timeout으로 제한할지 결정한다. 전자는 네 모드와 양쪽 메시지 정의를 함께 바꾸며 기존 CAN frame_id 연결도 점검한다.

## 6. 검증·측정 완료 조건

- 기능: 전체 이미지 토픽, 실제 MotionCommand, DiagnosticArray payload 동등성. 필요한 status/service/action만 추가.
- 모드: 4대 일치, 재부팅 복원, A→B 및 B→A 전환, 누락 바이너리/설정 실패 처리.
- 격리: bridge 계열 서비스 IVN에 DDS/RTPS 없음, native Zenoh에 bridge 없음, Cyclone 모드에 Zenoh 중계 잔존 없음.
- 복구: ZCU 재시작, HPC router/bridge 재시작, 케이블 단절, 구독자 늦은 시작/재시작.
- 실시간성: 이미지/제어 p50/p95/p99/max 지연, jitter, drop/duplicate/out-of-order, stale command 및 정지 반응.
- 자원: ZCU gateway+health 합계와 HPC 노드+router/bridge 합계 CPU, RSS/PSS, thread/FD/socket 수, peak/steady-state 메모리. Pico는 버퍼/heap/stack high-water와 code/flash 크기도 측정.
- 네트워크: payload goodput과 실제 wire bitrate, 제어/발견/재전송 오버헤드, DSCP, fragmentation, 링크별 경로.
- 시간: 서로 다른 장치의 monotonic timestamp는 직접 뺄 수 없다. PTP 등 동기화 오차를 기록하거나 RTT/동일 호스트 구간을 사용한다. 기존 frame_id 계측을 활용하되 wraparound를 처리한다.
- 반복성: 동일 데이터셋/seed/해상도/FPS/토픽·노드 수/CPU governor/affinity/빌드 옵션, warmup, 반복 횟수, 실행 순서, 버전·설정 hash를 저장한다.
- 합격 임계값: 허용 제어 지연/timeout, 이미지 FPS·손실, CPU/RAM 상한, 복구 시간을 구현 전에 수치로 확정한다. 현재 정보만으로 임의 값을 합격 기준으로 만들지 않는다.

2026-09-07 직전 리뷰의 확인 범위: 관련 Python 26개 통과. DDS domain 93, loopback 임시 포트에서 Image(43,200-byte 픽셀), DiagnosticArray Zenoh→DDS 및 MotionCommand DDS→Zenoh 직렬화 일치, allow 밖 토픽 수신 0건. 이는 합성 클라이언트의 HPC 검증이며 ZCU 4대 실기 또는 Pico 성능 검증이 아니다. `../middleware-migration.md`의 과거 실기 검증 기록도 이번 검토에서 재실행한 결과와 구분한다.

## 7. 권장 구현 순서와 미확정 입력

순서: 공통 메시지/실험 계약 확정 → Cyclone 완전 이주 및 전환 기준선 → 기존 Zenoh 통합 → zenoh_bridge ZCU 구현 및 HPC 수정 → 같은 Linux 보드의 zenoh_bridge_pico → 네 모드 자동 검증·비교. M코어 이식은 보류한다.

확정된 입력:

1. ADAS_load/BE_load 이주 제외, 삭제 허용. 향후 필요하면 새 부하 프로그램 구현.
2. Pico는 현재 Linux ZCU에서 구현. M코어/RTOS 이식은 현재 작업 범위에서 제외.

남은 결정:

1. 제어 지연, 명령 만료, 통신 단절 시 정지 동작/시간 및 복구 정책.
2. 이미지 FPS/최대 크기/허용 손실과 데이터 신선도, 자원 측정/상한.
3. legacy status 필요 여부, CLI/GUI 전환 UX, FastDDS 레거시 보존 기간, 헬스 포트 분리 여부는 기존 권고안을 유지하되 미확정으로 둔다.

## 8. 제어·이미지·자원 기준을 정하는 방법

현재 코드 사실(배포 장치 실측값 아님):

- HPC safety supervisor의 출력 주기는 50ms(20Hz). `_command_callback`은 명령만 저장하고 `_control_tick`은 생성/수신 시각 만료 검사 없이 마지막 값을 재발행한다.
- MotionCommand의 frame_id는 기존 카메라/CAN 지연 계측 식별자다. 제어 송신마다 증가하는 sequence 또는 생성 timestamp로 가정하면 안 된다.
- ZCU control callback은 수신 명령을 CAN으로 전달하며 이 경로에 명령 생성 시각 기반 만료 검사는 없다.
- ECU 소스 주석은 200ms×2=400ms watchdog을 설명하지만, 실제 callback은 counter<2일 때 증가만 하고 다음 callback의 else에서 정지한다. 주기가 실제 200ms라면 마지막 reset 대비 약 400~600ms에 정지 함수가 호출될 수 있다. 배포 타이머/펌웨어와 실제 구동 정지 시간은 별도 검증해야 한다.
- 카메라 송신 cam.py/webcam.py는 640×480, JPEG quality 80, 기본 목표 30FPS이며 65,000바이트를 넘는 JPEG는 송신하지 않는다. 실제 FPS와 프레임 크기는 입력·장치 부하에 따라 달라진다.
- 이전 브릿지 시험의 43,200바이트 픽셀은 합성 검증 데이터였다. 실제 640×480 JPEG 프레임의 고정 크기가 아니다.

제어에서 구분할 네 시간:

| 시간 | 정의 | 용도 |
| --- | --- | --- |
| 전달 지연 | HPC 명령 publish→ZCU callback 또는 ECU 수신. 구간을 고정 | 통신 모드 성능 비교 |
| 명령 나이/만료 | 원래 명령 생성→현재 적용 시점 | 계속 재전송되는 오래된 명령 차단 |
| 수신 공백 timeout | 마지막 유효 명령 수신 후 새 유효 명령이 없는 시간 | 링크/중계/송신 프로세스 장애 감지 |
| 정지 반응 시간 | 장애 발생→정지 출력, 필요하면 물리적 정지까지 | 전체 시스템 요구 검증 |

50ms 주기에 timeout 100/150/200ms는 각각 약 2/3/4주기 공백에 해당하는 설명용 예다. 짧으면 정상 jitter에도 정지하고 길면 오래된 구동이 유지된다. 어떤 값도 아직 승인된 운용 기준이 아니다. 먼저 액추에이터를 구동하지 않는 시험에서 전달 지연 분포와 최대 수신 공백을 측정하고, 최대 속도·허용 이동거리·기계적 감속 시간으로 운용 한도를 결정한다. 일정 속도에서 통신 지연 동안 추가 이동거리는 v×t이며 이는 전체 제동거리가 아니다.

정지 동작(속도 0/제동/PWM 차단, 조향 유지/중립)과 자동 재개/수동 재활성화도 시간과 별개로 결정한다. HPC에서만 정지 메시지를 보내는 설계는 링크 단절을 처리하지 못하므로 ZCU/ECU의 로컬 감시가 필요하다. 원래 제어 source의 정지, HPC 재발행의 지속, ZCU 수신 정지, CAN 정지를 각각 시험한다.

이미지 권장 출발점(미확정): 현재 640×480 JPEG 품질 80, 5개 스트림, 스트림당 목표 30FPS 유지. JPEG 최대 65,000바이트 제한과 초과 프레임 drop을 유지하되 drop 원인을 camera/ZCU/middleware/HPC 단계별로 계측한다. 비교용 반복 재생 데이터는 동일하게 사용한다. 프레임 크기는 평균·p95·최대, 수신은 FPS·연속 유실 길이·프레임 나이를 기록한다. DDS 메시지 타입은 현재 Image지만 data는 JPEG이므로 raw RGB 크기로 대역폭을 산정하지 않는다.

payload 대역폭 = 평균 프레임 바이트×FPS×스트림 수×8. 설명용 평균 40,000바이트×30×5는 48Mbps, 상한 65,000바이트×30×5는 78Mbps다. CDR·전송·Ethernet·재전송 오버헤드는 제외한 값이다. 1개→5개 스트림과 15→30FPS 단계 시험으로 병목을 확인한다. 최대 크기는 JPEG 크기와 CDR를 포함한 직렬화 크기를 따로 기록한다.

자원 상한은 현재 임의의 MB/%를 고정하기보다 같은 workload에서 네 모드의 baseline/steady/peak를 먼저 비교한다. ZCU별 gateway+health의 PSS/RSS·CPU·thread 수와 HPC 노드+router/bridge 합계를 기록한다. CPU는 1코어 100% 기준인지 전체 코어 정규화인지 명시한다. Linux Pico 시험에는 M코어 flash/RTOS stack 예산을 합격 기준으로 요구하지 않는다. 가용 CPU/RAM에서 OS와 다른 서비스의 peak 및 여유분을 뺀 값이 통신 예산이다. 같은 기능과 FPS를 유지하는지 먼저 확인하고 자원 감소율을 해석한다.
