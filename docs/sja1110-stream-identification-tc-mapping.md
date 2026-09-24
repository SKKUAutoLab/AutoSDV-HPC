# SJA1110 Stream Identification 및 TC 할당을 위한 HPC·ZCU 트래픽 매핑

현재 HPC·ZCU 코드의 IVN 설정 기준이다. HPC 주소는 `10.0.0.10`, ZCU1~4 주소는 각각 `10.0.0.2`, `.3`, `.4`, `.5`이다. ZCU는 `pfe0`를 쓰고, HPC는 `10.0.0.10`이 할당된 인터페이스를 선택한다. TC 매핑은 스위치에 이미 구현되어 있으며, 카메라별 Stream Identification 규칙은 별도로 설정해야 한다.

| 구분 | IP·UDP 포트 할당 | DSCP (IPv4 TOS, ECN=0) | 목표 TC |
| --- | --- | --- | --- |
| 카메라, Cyclone DDS | ZCU → HPC `10.0.0.10`; 목적지 `7451/7453/7455/7457/7459` | CS5 = 40 (`0xA0`) | TC5 |
| 카메라, Zenoh·Bridge·Pico | ZCU → HPC `10.0.0.10`; 카메라별 목적지 `17451/17453/17455/17457/17459` | CS5 = 40 (`0xA0`) | TC5 |
| 카메라, Fast DDS | ZCU 출발지 `61000`; 목적지 포트는 DDS 참가자에 따라 결정 | CS5 = 40 (`0xA0`) | TC5 |
| 제어, Cyclone DDS | HPC → ZCU; 목적지 `7421`, HPC 출발지 포트는 가변 | CS6 = 48 (`0xC0`) | TC6 |
| 제어, Zenoh·Bridge·Pico | HPC → ZCU; **출발지 `7448`**, ZCU 목적지 포트는 가변 | CS6 = 48 (`0xC0`) | TC6 |
| 제어, Fast DDS | HPC → ZCU; HPC 출발지 포트와 ZCU DDS 목적지 포트는 참가자에 따라 결정 | CS6 = 48 (`0xC0`) | TC6 |
| 그 밖의 HPC 송신 IPv4 | HPC → ZCU; DDS discovery 등 | CS1 = 8 (`0x20`) | TC1 |

Cyclone DDS와 Zenoh 계열 카메라의 HPC 목적지 포트는 아래처럼 개별 할당된다. Zenoh 포트는 HPC의 로컬 DDS 참가자 포트 범위와 충돌하지 않도록 분리했다.

| 송신 보드 | 카메라 토픽 | Cyclone DDS 목적지 | Zenoh 계열 목적지 |
| --- | --- | --- | --- |
| ZCU1 `10.0.0.2` | `image_01_raw`, `image_02_raw` | `7451`, `7453` | `17451`, `17453` |
| ZCU2 `10.0.0.3` | `image_03_raw` | `7455` | `17455` |
| ZCU3 `10.0.0.4` | `image_04_raw` | `7457` | `17457` |
| ZCU4 `10.0.0.5` | `image_05_raw` | `7459` | `17459` |

SJA1110에서 카메라는 **ZCU 출발지 IP + HPC 목적지 UDP 포트**로 개별 식별할 수 있다. Zenoh 계열은 카메라별로 별도 세션도 사용한다. 스위치의 Stream Identification 규칙은 아직 별도로 설정해야 하며, DSCP는 기존 **TC 분류**에 계속 사용한다. Zenoh의 `7448`에는 제어와 상태 정보가 함께 흐르므로 포트만으로 제어 토픽을 단독 식별할 수는 없다.

ZCU → HPC의 Zenoh `7448` 트래픽은 현재 DSCP 0이다. Fast DDS의 ZCU 제어 송신용 고정 출발지 포트는 `61002`이다. 위 CS1 기본 규칙은 **HPC 송신 방향**에 적용되며 ZCU의 나머지 송신 트래픽까지 CS1이라는 뜻은 아니다. DSCP 비교 시에는 ECN 하위 2비트를 제외한 상위 6비트를 사용한다.

Fast DDS는 ZCU에 기존 구현으로 남아 있지만, HPC의 통합 통신 모드 전환 대상은 Cyclone DDS·Zenoh·Bridge·Pico이다.

구현 근거: [HPC DSCP 규칙](../network_qos/autosdv_dscp_qos.py), [HPC 통신 모드와 ZCU 카메라 포트 규칙](../scripts/communication_host.py), [HPC QoS 정책](../network_qos/README.md), ZCU의 `include/zcu_config.h`, `systemd/zcu-camera-dscp.service`, `systemd/zcu-camera-dscp-cyclonedds@.service`.
