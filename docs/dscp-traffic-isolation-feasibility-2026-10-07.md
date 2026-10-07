# Communication mode별 DSCP 분리 구현 가능성 검토

검토일: 2026-10-07. 실행 코드, 설정, 네트워크 규칙은 변경하지 않았다. 이 문서는 소스 및 공식 문서에 근거한 설계 검토이며, 구현 완료나 패킷 단위 검증 결과가 아니다.

## 판정

1안(제어 payload CS6=48, 카메라 payload CS5=40, Health payload CS1=8, 기타 미들웨어 CS0=0)은 **네 모드 모두 현 설정/포트 마킹만으로 엄밀하게 보장할 수 없다.** 그러나 **미들웨어 송신부와 packetization까지 수정하면 구현 가능한 설계**다. 원천적으로 불가능하다는 판정과 구분해야 한다. 2안(관련 노드의 통신 전체를 제어 CS6, 카메라 CS5, Health CS0)은 **역할별 프로세스·세션·소켓 분리와 양방향 정책 변경으로 네 모드 모두 구현 가능**하다.

| 모드 | 1안: 설정/포트 마킹만 | 1안: 필요한 구현 범위 | 2안 |
|---|---|---|---|
| CycloneDDS | 엄밀 보장 불가 | DDS writer/sample 역할 추적, RTPS DATA/DATA_FRAG와 discovery/HEARTBEAT/ACKNACK 등 분리 packetization, UDP 송신 단위 DSCP | 역할별 프로세스/실제 domain transport 및 소켓 분리 후 전체 역할 트래픽 마킹 |
| Zenoh (ROS rmw_zenoh 경로) | 엄밀 보장 불가 | ROS 토픽 의미→Zenoh 메시지 클래스 전달, 클래스별 배치/fragment 분리, Rust Zenoh 및 router 송신부 마킹 | 제어/카메라/Health별 실제 IVN 세션·router 경로 분리 |
| Zenoh-bridge | 엄밀 보장 불가 | bridge의 ROS↔Zenoh 변환에서 의미 전달, bridge가 사용하는 Zenoh 송신부와 ZCU zenoh-c 경로 수정 | 역할별 bridge/router 세션, 포트, allow 목록 및 ZCU 세션 분리 |
| zenoh-pico | 엄밀 보장 불가 | pico network message→batch/fragment→POSIX UDP send 경로에 클래스 전달; HPC Rust Zenoh/bridge도 함께 수정 | pico 세션별 역할 분리 및 양방향 세션 소켓/포트 마킹 |

범위는 HPC–S32G3 사이 유선 IVN이다. loopback DDS 등 로컬 통신은 스위치 큐 검증 대상과 구별한다. payload 패킷의 필수 IP/UDP/직렬화/fragment 헤더까지 CS0로 분리한다는 의미는 아니다. 별개의 discovery, subscription, lease/keepalive, liveliness, ACK/NACK/heartbeat 메시지는 CS0로 보내는 의미다. 데이터 재전송은 해당 payload 클래스에 속하고, 재전송 요청 등은 CS0에 속한다.

## 왜 포트만으로 1안을 만족할 수 없는가

DSCP는 IP 패킷 헤더의 하나의 필드다. 하나의 패킷에 제어 데이터와 미들웨어 housekeeping이 함께 있으면 같은 패킷에 CS6와 CS0을 동시에 부여할 수 없다. 패킷을 만들기 전에 분리해야 한다. 분리한 이후 OS가 다시 병합하는 TCP 경로도 고려해야 한다. [RFC 2474](https://www.rfc-editor.org/rfc/rfc2474)

현재 코드의 포트 분류와 baseline 마킹도 요구사항과 다르다. HPC `network_qos/autosdv_dscp_qos.py`는 IPv4 baseline CS1 및 Zenoh 계열 7448 CS6 규칙을 사용한다. `scripts/communication_host.py`는 ZCU 카메라 목적지 포트를 CS5로 지정한다. 따라서 현 카메라 역방향 미들웨어 통신까지 CS5로 일관되게 분리됐다고 볼 수 없으며, 1안에서는 baseline을 CS0으로 바꾸고 payload만 선택적으로 승격해야 한다. 로컬 근거: [HPC DSCP 정책](../network_qos/autosdv_dscp_qos.py), [communication mode 호스트 설정](../scripts/communication_host.py).

## CycloneDDS

검토 기준은 HPC 설치 CycloneDDS 0.10.5, rmw_cyclonedds 2.2.4이다. Cyclone 공식 trace 예시는 application DATA와 piggyback HEARTBEAT를 같은 xpack에 넣어 전송한다. user-data UDP 포트라고 해서 application payload만 들어간다고 판단할 수 없다. discovery 포트만 CS0으로 설정해도 HEARTBEAT/ACKNACK 등 데이터 경로의 관리 통신이 남는다. [CycloneDDS trace 설명](https://cyclonedds.io/content/guides/tracefile.html)

`TransportPriority` 및 `Channels/DiffServField` 이름만 보고 stock CycloneDDS에서 즉시 가능한 기능이라고 판단하면 안 된다. 0.10.5의 channel 설정은 `DDS_HAS_NETWORK_CHANNELS` 조건부 코드이며, 빌드 파일은 이를 소스에 남아 있는 기능으로 열거한다. 표준 생성 `features.h.in`에도 해당 기능 정의가 없다. `TransportPriority` QoS API 존재는 네트워크 DSCP 자동 매핑 지원의 증거가 아니다. [0.10.5 CMake](https://github.com/eclipse-cyclonedds/cyclonedds/blob/0.10.5/src/CMakeLists.txt), [feature 정의](https://github.com/eclipse-cyclonedds/cyclonedds/blob/0.10.5/src/ddsrt/include/dds/features.h.in), [조건부 channel 설정](https://github.com/eclipse-cyclonedds/cyclonedds/blob/0.10.5/src/core/ddsi/include/dds/ddsi/ddsi_cfgelems.h)

또한 남아 있는 UDP DiffServ 코드는 `m_diffserv`를 그대로 IPv4 `IP_TOS`에 전달한다. 이 코드에 6-bit DSCP 수치 48을 그대로 넣으면 CS6에 필요한 TOS 192와 같지 않다. 실제 마킹 API가 6-bit DSCP인지 전체 TOS인지 구분해야 한다. [0.10.5 UDP 구현](https://github.com/eclipse-cyclonedds/cyclonedds/blob/0.10.5/src/core/ddsi/src/ddsi_udp.c)

1안의 구현 지점은 DDS 송신 메시지/pack 구성과 UDP 송신이다. 일반 토픽 DATA를 모두 payload로 취급하면 built-in discovery DATA도 잘못 분류하므로 writer/entity 및 토픽 의미를 식별해야 한다. 클래스가 바뀌면 xpack을 flush하고, 별도 heartbeat와 ACKNACK은 CS0으로 처리한다. fragment와 재전송에도 원래 sample의 클래스를 유지해야 한다. 로컬 ROS RMW QoS 조절만으로 이 불변조건이 생기지는 않는다.

2안에서는 S32G3 `src/backend/cyclonedds/transport.cpp`의 프로세스당 static domain 초기화와 `src/main.cpp`의 카메라/제어 공존을 먼저 분리해야 한다. 단지 ROS 노드 이름이나 DDS participant 객체를 나누는 것만으로 독립 송신 소켓을 얻는다고 가정하면 안 된다. Cyclone의 공유 discovery/transport 구조도 고려해야 한다. [공식 discovery 설명](https://github.com/eclipse-cyclonedds/cyclonedds/blob/master/docs/manual/config/discovery-behavior.rst)

## Rust Zenoh / zenoh-c / ROS bridge

HPC 설치 `zenohcConfigVersion.cmake`의 native C library 버전은 1.8.0이다. 저장소 문서에는 과거 zenoh-c 1.6.2 설치 예시도 있으므로 문서 예시와 설치 버전을 동일시하지 않는다. bridge 및 pico 설치 스크립트 기준은 1.10.0이다. ZCU 실제 배포 바이너리 버전은 이번 검토에서 원격 확인하지 않았다.

Zenoh에는 endpoint `#dscp=...` 설정이 있으나 **소켓/링크 전체**의 DSCP다. 1.8.0 UDP 구현은 connect/listen 시 `set_dscp`를 호출한다. 해당 링크의 discovery/declare/keepalive가 자동 CS0이 되는 기능이 아니다. [Zenoh endpoint 설명](https://github.com/eclipse-zenoh/zenoh/blob/1.8.0/DEFAULT_CONFIG.json5), [1.8.0 UDP 구현](https://github.com/eclipse-zenoh/zenoh/blob/1.8.0/io/zenoh-links/zenoh-link-udp/src/unicast.rs)

1.8.0 universal link 송신 루프는 배치를 `link.send_batch`로 보내고, idle timeout에는 같은 link에서 `KeepAlive`를 `link.send`로 보낸다. 따라서 priority별 endpoint를 나누고 각 endpoint DSCP를 지정하는 것만으로도 1안은 충족되지 않는다. [1.8.0 link 송신](https://github.com/eclipse-zenoh/zenoh/blob/1.8.0/io/zenoh-transport/src/unicast/universal/link.rs)

Zenoh priority는 Zenoh 내부 스케줄링 클래스이며 DSCP 숫자와 별개다. 이름이 `Control`이라고 해서 차량 제어 payload를 뜻하지도 않는다. Frame에는 복수 network message가 들어갈 수 있고, express/배치 설정은 packetization에 영향을 주지만 전체 housekeeping의 CS0 분류를 대신하지 않는다. [Zenoh QoS 규격](https://spec.zenoh.io/spec/1.0.0/concepts/qos.html), [Frame 규격](https://spec.zenoh.io/spec/1.0.0/wire/frame-format.html)

1안에서는 application payload 의미를 송신 단계까지 전달하고, 한 UDP datagram에는 하나의 클래스만 넣도록 해야 한다. `PUSH`도 liveliness/graph 토큰 등 middleware 용도일 수 있으므로 무조건 application payload로 간주해서는 안 된다. 단순 wire offset 필터는 압축, fragment, 배치, key-expression mapping에 따라 오분류할 수 있다. router/bridge가 메시지를 재전송하면 거기서 클래스가 보존되거나 다시 계산되어야 한다. native ROS Zenoh와 bridge가 다른 Zenoh 버전을 링크한다면 패치/검증 대상도 별개다.

2안에서는 현재 공유 7448을 제어 전용과 Health 전용 실제 세션/리스너 경로로 분리하고, router gossip/autoconnect 등으로 다시 합쳐지지 않게 해야 한다. ZCU 카메라는 이미 개별 세션 구조지만 HPC↔ZCU 양방향 전체 역할 트래픽 규칙이 필요하다. 각 역할의 discovery/keepalive도 해당 역할 CS를 받는 것은 2안에서 허용되는 차이다. 카메라 시작이 제어 세션 연결 성공을 기다리는 현재 시작 순서도 분리해야 한다. 세션뿐 아니라 시작·재시작·오류 전파의 수명주기를 독립시켜야 Q6 차단이 다른 역할을 멈추지 않는다.

## zenoh-pico

1.10.0 소스의 `src/transport/common/tx.c`는 network message batching, express 처리, fragment 생성, 최종 `_z_link_send_wbuf`를 연결한다. 상위 message 의미를 잃기 전에 클래스 정보를 지정하고, 배치 혼합 금지 및 모든 fragment의 동일 클래스 보존을 구현할 위치가 있다. [pico 1.10.0 TX 소스](https://github.com/eclipse-zenoh/zenoh-pico/blob/1.10.0/src/transport/common/tx.c)

검토한 1.10.0 소스에서 `dscp`, `DSCP`, `IP_TOS`, `TCLASS` 구현은 검색되지 않았다. Rust Zenoh endpoint DSCP 기능이 pico에서도 지원된다고 가정하면 안 된다. 2안은 OS의 역할별 소켓/포트 정책으로 구현할 수 있으며, 1안은 pico POSIX link send에 datagram별 마킹을 추가해야 한다. HPC 반대편의 Rust Zenoh/bridge도 동일 정책을 구현해야 한다. [pico POSIX UDP 구현](https://github.com/eclipse-zenoh/zenoh-pico/blob/1.10.0/src/link/transport/udp/udp_posix.c)

## 권장 구현 경계 및 검증

1안을 우선한다면 네 모드의 의미는 유지하면서 **유선 IVN 전송은 UDP로 제한하고 transport 송신부를 확장하는 설계**가 가장 명확하다. per-datagram ancillary `IP_TOS` 또는 명확히 분리한 class socket 경로를 사용하고, shared socket에 메시지마다 `setsockopt`를 변경하는 경쟁 조건을 피한다. 별도 class socket을 사용한다면 source IP/port가 바뀌어 기존 세션의 peer 식별을 깨뜨리지 않도록 설계해야 한다. 현재 세션의 source identity를 유지하는 `sendmsg` ancillary 방식이 이 점에서는 우선 검토 대상이다. 현재 UDP 연결의 reliability 특성까지 임의로 바꾸지는 않아야 한다. 이 문장의 구현 선택은 위 source 분석에 기반한 설계 제안이며 검증된 patch는 아니다.

TCP까지 일반화하려면 application write 경계와 실제 segment 경계가 다르며 ACK/재전송/커널 keepalive까지 다뤄야 하므로 설계가 더 커진다. 필수 TCP ACK header와 standalone ACK 패킷의 범위도 정의해야 한다. 단순히 `TCP_NODELAY` 또는 express만 켜서 엄밀한 class 분리를 증명할 수는 없다. 모든 모드를 custom application UDP로 우회하면 기존 Cyclone/Zenoh 비교의 의미가 바뀌므로 여기서는 그 방식을 기본안으로 삼지 않는다.

구현 후 승인 기준은 네 모드 각각 양방향 패킷 캡처에서 의미별 DSCP를 검증하는 것이다. 정상 주기 데이터만 아니라 초기 discovery, 구독/해제, idle keepalive, reconnect, camera fragment, 데이터 재전송 및 middleware retransmission 요청을 포함한다. Q6 차단 상태에서도 CS0 관리 통신과 Health/카메라 경로가 살아 있고, 차단 해제 후 제어 경로가 복구되는지 확인한다. 2안에서는 제어 세션 관리 패킷도 Q6에 들어가므로 Q6 차단 시 제어 세션 단절 자체는 허용되지만 Health/카메라로 전파되어서는 안 된다.
