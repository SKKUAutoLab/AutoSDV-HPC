# Zenoh-DDS Bridge 환경 (AUTOSDV_RMW=zenoh_bridge)

미들웨어 비교 실험의 **세 번째 환경**이다. ZCU는 DDS 없이 zenoh로만 보내고, HPC
안에서는 CycloneDDS로 도는 구성을 `zenoh-plugin-ros2dds`의 standalone 브릿지
(`zenoh-bridge-ros2dds`)로 만든다. **브릿지는 HPC에만 있다.**

2026-09-10: ZCU `zenoh_bridge` 백엔드와 ros2dds 발행자 광고를 추가했다. 운영 전환은 GUI의
Communication mode 또는 공통 CLI를 사용한다. 아래 초기 조사 기록과 실기 검증은
[Bridge 구현·검증 기록](zenoh-bridge-validation-2026-09-10.md)으로 구분한다.

```
ZCU (순수 zenoh 클라이언트)  ──zenoh/IVN──▶  HPC: zenoh-bridge-ros2dds
                                                        │ DDS (loopback)
                                                        ▼
                                            HPC 노드 (rmw_cyclonedds)
```

기존 두 환경과의 관계:

| 환경 | HPC 노드 | 와이어 | ZCU |
| --- | --- | --- | --- |
| CycloneDDS | rmw_cyclonedds, 10.0.0.10 | RTPS | cyclonedds 백엔드 |
| Zenoh | rmw_zenoh, 라우터 경유 | zenoh | zenoh 백엔드 (rmw_zenoh 규약) |
| **Zenoh-DDS Bridge** | **rmw_cyclonedds, 127.0.0.1** | **zenoh** | **zenoh 백엔드 (ros2dds 규약)** |

세 환경 모두 ZCU 쪽 전송 설정(client 모드, 10.0.0.10의 카메라별 17451~17459 및 제어 7448, `#dscp`)은
같은 모양이고, 바뀌는 건 keyexpr과 프레이밍뿐이다.

---

## 1. ZCU가 지켜야 할 규약

브릿지의 zenoh 쪽은 **평범한 zenoh 클라이언트**를 1급으로 받는다. 상대가 브릿지일
필요도, ROS일 필요도 없다 (upstream 통합 테스트 `zenoh-test-ros2dds/tests/
topic_pub_sub.rs`가 정확히 이 경우를 검증한다). 계약은 두 줄이다:

```
keyexpr  = ROS 토픽명에서 선행 '/'만 뗀 것       예: /image_01_raw -> image_01_raw
payload  = DDS 직렬화 샘플 = 캡슐화된 CDR        ZCU가 지금 만드는 것 그대로
```

발행은 그 keyexpr에 `put`, 수신은 같은 keyexpr에 `declare_subscriber`다.
`docs/examples/zcu_zenoh_client.c`가 zenoh-c만으로 된 최소 구현이다.

### 지금 zenoh 백엔드에서 바뀌는 것

| | 현재 (rmw_zenoh 규약) | 이 환경 (ros2dds 규약) |
| --- | --- | --- |
| keyexpr | `<domain>/<topic>/<type>/<RIHS01 hash>` | `<topic>` |
| attachment | (seq, source_timestamp, gid) **필수** | **없음** — payload만 |
| liveliness | `@ros2_lv/...` 토큰 필수 | ros2dds MP 토큰 사용 (§1.1) |
| payload | 캡슐화 XCDRv1 | 그대로 |
| 세션 설정 | client, 카메라별 17451~17459/제어 7448, `#dscp` | 그대로 |

CDR payload와 Zenoh 세션은 재사용한다. 타입 해시는 data key 대신 광고 토큰의
USER_DATA로 전달하며, 광고 형식은 ros2dds 1.10.0에 맞춰 검증한다.

### 1.1 발행자 광고와 구독 활성화

현재 ZCU는 카메라와 Health publisher의 수명 동안 ros2dds 1.10.0 형식의
Zenoh liveliness 토큰을 유지한다. 생성 성공 시 광고하고 파괴 시 회수한다.

```
@/<session_zid>/@ros2_lv/MP/image_01_raw/sensor_msgs§msg§Image/:0:0:0,1:typehash=RIHS01_…;
@/<session_zid>/@ros2_lv/MP/ivn§zcu_health/diagnostic_msgs§msg§DiagnosticArray/:1:0:0,10:typehash=RIHS01_…;
```

`§`는 토픽·타입 안의 `/`를 치환한 값이다. QoS는 keyless, reliability,
durability, history kind/depth, USER_DATA 순서다. 카메라는 BEST_EFFORT/KEEP_LAST 1,
Health는 RELIABLE/KEEP_LAST 10이며 모두 VOLATILE이다. 타입 해시도 전달한다.

브리지는 광고만으로 DDS writer를 생성해 GUI 구독 전에 발견되도록 한다.
HPC 구독자가 생기면 Zenoh→DDS 데이터 전달을 활성화한다. 마지막 로컬 구독자가
없어져도 원격 광고가 살아 있으면 DDS writer를 유지한다.

광고가 회수되고 로컬 구독자도 없으면 writer가 제거된다. 로컬 구독자가 남아 있으면
writer가 유지될 수 있으므로, 실제 카메라 Health는 기존 producer hook과 메시지
신선도 검사도 계속 사용한다. 발행자 광고는 프레임이 정상 도착한다는 보증이 아니다.
ZCU 원래 노드 이름을 복원하는 기능은 아니며 DDS endpoint는 HPC bridge 이름으로 보인다.

[발행자 광고 구현과 검증 기록](zenoh-bridge-announcement-2026-09-10.md).

---

## 2. HPC 쪽 구성물

| 파일 | 역할 |
| --- | --- |
| `cyclonedds_bridge_local.xml` | HPC 노드와 브릿지가 공유하는 loopback 전용 DDS 프로파일 |
| `zenoh_bridge_camera_ivn.json5` | 카메라 브릿지 (UDP 17451/17453/17455/17457/17459) |
| `zenoh_bridge_control_ivn.json5` | 제어 브릿지 (7448) |
| `scripts/install_zenoh_bridge.sh` | 브릿지 바이너리 내려받기 |
| `scripts/run_zenoh_bridge.sh` | 브릿지 기동 (`camera` / `control` / `all`) |
| `docs/examples/zcu_zenoh_client.c` | ZCU 쪽 최소 참조 구현 |
| `launch_pkg.rmw_profile` | `AUTOSDV_RMW=zenoh_bridge` 추가 |
| `network_qos/autosdv_dscp_qos.py` | `--middleware zenoh_bridge` 추가 |

---

## 3. 설계 판단과 근거

### 3.1 플로우마다 브릿지를 따로 띄운다

zenoh 세션 하나 = 링크 하나 = 5-tuple 하나다. 카메라와 제어를 한 브릿지에 태우면
keyexpr이 zenoh 페이로드 안에 있어서 tc도 SJA1110도 둘을 구분할 수 없다. 네이티브
zenoh 환경이 라우터 포트를 7447(카메라)·7448(제어·헬스)로 나눈 것과 같은 이유로,
브릿지도 두 개를 띄우고 `allow` 목록을 서로 겹치지 않게 준다. ZCU는 지금처럼 세션을
두 개 연다.

`scouting.gossip`은 네이티브 zenoh 설정과 달리 **끈다**. 켜 두면 두 브릿지가 공통
ZCU를 통해 서로를 발견해 두 플로우가 한 링크로 합쳐질 수 있다.

### 3.2 DDS는 loopback에 가둔다

`cyclonedds_bridge_local.xml`은 다른 프로파일들과 정확히 반대다. 다른 환경은
10.0.0.10을 고정하지만 여기서는 127.0.0.1만 남기고 멀티캐스트를 끈다. 와이어에
RTPS가 한 패킷이라도 나가면 브릿지를 우회하게 되고, 그게 이 실험이 분리하려는 바로
그 변인이다. `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`를 그대로 푼 형태이고,
rmw_cyclonedds와 zenoh-plugin-ros2dds가 같은 설정에서 생성하는 것과 같다.

브릿지 프로세스도 CycloneDDS를 내장하므로 같은 파일을 `CYCLONEDDS_URI`로 받는다.
`run_zenoh_bridge.sh`가 이걸 export한다.

이미지 노드의 participant index 고정(20–24)은 여기서 **뺐다**. 그 고정은 와이어의
RTPS 목적지 포트를 결정적으로 만들어 DSCP를 걸기 위한 것이었는데, 이 환경에는
와이어에 RTPS가 없다. 그래서 모든 노드가 프로파일 하나를 공유하고,
`rmw_profile.profile_filename()`이 어떤 이름을 받든 이 파일 하나로 보낸다.

### 3.3 HPC가 router, ZCU가 client

네이티브 zenoh 환경과 같다. ZCU는 client라 브릿지 없이는 세션을 못 열고, 그래서
`connect.timeout_ms`를 넉넉히 준 뒤 유닛의 `Restart=on-failure`에 맡기는 방식도
그대로 쓸 수 있다.

### 3.4 DDS 도메인은 고정하지 않는다

`plugins.ros2dds.domain`을 비워 두면 플러그인이 `$ROS_DOMAIN_ID`(없으면 0)를 쓴다.
HPC 노드와 같은 규칙이다. 여기에 `0`을 박아 두면 누군가 도메인을 바꿔 노드를 띄운
날 브릿지가 아무것도 라우팅하지 않으면서 에러도 내지 않는다.

### 3.5 `batch_size: 1400`

네이티브 zenoh 환경과 같은 값이다. 이게 없으면 44 KB 이미지가 데이터그램 하나 +
IP fragment 약 30조각이 되어 RTPS `maxMessageSize=1400`과 비교 가능한 조건이
아니게 된다. 카메라는 ZCU가 송신 측이므로 ZCU 세션에도 같은 값이 필요하다.

### 3.6 DSCP는 그대로 재사용한다

카메라는 ZCU 쪽 endpoint의 `#dscp=0xa0`이, 제어는 HPC의 nftables 규칙(7448 egress
→ CS6)이 담당한다. 두 메커니즘 모두 네이티브 zenoh 환경과 동일해서
`--middleware zenoh_bridge`가 만드는 룰셋은 `--middleware zenoh`와 바이트 단위로
같다 (테스트로 고정해 두었다).

`#dscp` 값이 이름과 달리 IPv4 TOS 바이트에 그대로 쓰인다는 점도 zenoh 1.10에서
다시 확인했다 (§5).

### 3.7 `ROS_DISTRO`

브릿지는 `ROS_DISTRO`로 `ros_discovery_info`의 레이아웃을 고른다. 안 주면 iron으로
가정하고 경고만 남기므로 `run_zenoh_bridge.sh`가 `jazzy`를 export한다.

---

## 4. 설치와 운영

```sh
AutoSDV-HPC/scripts/install_zenoh_bridge.sh     # 최초 1회
```

`zenoh-plugin-ros2dds 1.10.0`의 aarch64 glibc standalone 빌드를
`tools/zenoh-bridge-ros2dds/`에 놓는다. 버전은 고정이다 — HPC와 보드 4대가 같은
zenoh 프로토콜을 써야 하는데 "latest"면 다음 릴리스에서 조용히 깨진다.

ZCU마다 해당 zone 번호로 먼저 빌드한다. zenoh-c/cpp와 Fast CDR가 필요하다.

```sh
cmake -S . -B build-zenoh_bridge -DZCU_MIDDLEWARE=zenoh_bridge \
  -DZCU_ZONE_ID=1 -DCMAKE_PREFIX_PATH=/usr/local -DBUILD_TESTING=ON
cmake --build build-zenoh_bridge -j2
ctest --test-dir build-zenoh_bridge --output-on-failure
```

HPC에서 모든 장치를 함께 전환한다. GUI 실행 중에는 Communication mode에서
`Zenoh–DDS Bridge`를 선택하고 Check → Apply한다.

```sh
./scripts/select_communication_mode.sh check zenoh_bridge
./scripts/select_communication_mode.sh apply zenoh_bridge
```

공통 전환기가 HPC 브리지를 먼저 시작하고 ZCU 실행 파일을 교체·기동한다.
기존 `select_middleware.sh`는 개별 장치용 이전 도구이며 이 모드 전환에는 사용하지 않는다.
ZCU 발행자 광고로 DDS writer를 미리 준비하며, 실제 데이터 전달은 HPC 구독 시 활성화한다.

---

## 5. 초기 격리 환경 검증 기록

HPC 한 대에서 확인했다. 도메인 0(운용 중인 시스템)은 건드리지 않았다.

**순수 zenoh 클라이언트 ↔ 브릿지** (`docs/examples/zcu_zenoh_client.c`, DDS·브릿지·
liveliness 없음, 브릿지는 HPC에 하나):

| 항목 | 결과 |
| --- | --- |
| ZCU(zenoh) → HPC(CycloneDDS) | 수신됨 |
| HPC(CycloneDDS) → ZCU(zenoh) | 수신됨 |
| 브릿지가 만든 route | `Route Subscriber (Zenoh:topic_status_signal -> ROS:/topic_status_signal)`, `Route Publisher (ROS:/topic_control_signal -> Zenoh:topic_control_signal)` |

**브릿지 자체** (도메인 90 ↔ 91):

| 항목 | 결과 |
| --- | --- |
| `zenoh-bridge-ros2dds --version` | v1.10.0 |
| 카메라 (`/image_0N_raw`), 제어, 상태 양방향 | 전달됨 |
| allow 목록 밖 토픽 (`/parameter_events`) | 라우팅 안 됨 |
| 카메라 링크 egress TOS / 제어 링크 | `0xa0` (CS5) / `0x00` |
| `render_ruleset(zenoh_bridge)` == `render_ruleset(zenoh)` | 동일 |

이 절은 ZCU 구현 전 격리 환경의 기록이다. 현재의 실기 결과와 방향별 allow 목록은
[2026-09-10 검증 기록](zenoh-bridge-validation-2026-09-10.md)을 참조한다.

---

## 6. 구현된 ZCU 백엔드

`-DZCU_MIDDLEWARE=zenoh_bridge`는 기존 Zenoh 세션과 CDR 코드를 재사용하고
`ZCU_ZENOH_BRIDGE`로 wire 규약을 선택한다. plain topic key를 사용하며
rmw_zenoh attachment와 토큰은 발행하지 않고 ros2dds MP 토큰을 발행한다. 네이티브 Zenoh 빌드는
기존 규약을 유지한다.

Health는 fresh producer hook을 사용한다. ZCU의 GraphProbe는 HPC의 DDS discovery를 조회하지 않으므로
ZCU Health 값의 `publisher_discovered=false`가 정상이며 `publisher_configured`와 프레임 신선도로
판정한다. 선택 기능인 `monitor_camera_payload=true`는 카메라별 별도 세션(17451~17459)을
필요할 때만 열어 직접 샘플을 센다. 기본값에서는 추가 이미지 구독을 만들지 않는다.

HPC allow 목록은 로컬 DDS endpoint 기준으로 카메라 bridge는 이미지 구독자만,
control bridge는 제어 발행자와 Health 구독자만 허용한다. 현재 비활성인
`topic_status_signal`과 service/action은 전달하지 않는다.

## 7. 알려진 차이

- **ZCU 원래 노드 이름은 `ros2 node list`에 나타나지 않는다.** 발행 endpoint는
  HPC bridge 노드 아래에 보인다. MP 토큰만으로 원래 노드 이름을 복원하지 않는다.
- **DDS writer 발견과 실제 데이터 전달은 구분한다.** 광고로 writer를 미리 만들고
  로컬 구독 시 데이터 전달을 활성화한다. ros2dds는 카메라의 BEST_EFFORT 광고를
  받아도 DDS writer를 RELIABLE로 생성하므로 실험 시 실제 QoS를 기록해야 한다.
- **제어와 헬스가 7448을 공유한다.** CS6가 헬스까지 덮는다. 네이티브 zenoh 환경과
  같은 타협이라 비교 가능성은 유지되지만, 분리하려면 양쪽에 포트가 하나 더 필요하다.
- **`ivn_diagnostics`의 그래프 질의 결과가 달라질 수 있다.** 카메라·제어 zenoh 망을
  갈라 두었고 ZCU가 그래프에 없으므로 entry 수를 실기에서 다시 재야 한다.
