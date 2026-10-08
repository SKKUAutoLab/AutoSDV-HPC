# SJA1110 Stream Identification 및 TC 할당을 위한 HPC·ZCU 트래픽 매핑

현재 HPC·ZCU 코드의 IVN 설정 기준이다. HPC 주소는 `10.0.0.10`, ZCU1~4 주소는 각각 `10.0.0.2`, `.3`, `.4`, `.5`이다. ZCU는 `pfe0`를 쓰고, HPC는 `10.0.0.10`이 할당된 인터페이스를 선택한다.

## 분류 정책 (2안: 역할 단위)

| 역할 | 포함 범위 | DSCP (IPv4 TOS) | 목표 TC |
| --- | --- | --- | --- |
| 제어 | MotionCommand와 제어 세션·노드의 discovery, 구독, 세션 유지 | CS6 = 48 (`0xC0`) | TC6 |
| 카메라 | 영상과 카메라 세션·노드의 discovery, 구독, 세션 유지 | CS5 = 40 (`0xA0`) | TC5 |
| Health 및 그 밖의 모든 트래픽 | ZCU health, HPC 진단(ping 포함), SSH, OTA 등 | CS0 = 0 (`0x00`) | TC0 |

정책은 **양방향**에 적용된다. 예를 들어 제어 세션에서 ZCU가 HPC로 보내는 구독 선언과 keepalive도 CS6이다. HPC에는 모든 송신을 다시 표시하는 기본 규칙이 없다. 그런 규칙은 프로세스가 소켓에 직접 설정한 표시를 지운다.

## 표시 방법

| 위치 | 방법 |
| --- | --- |
| ZCU (모든 모드) | 역할마다 프로세스를 나눈다. `zcu-camera.service`와 `zcu-control.service`가 `libautosdv_dscp_shim.so`를 `LD_PRELOAD`로 올리고 `AUTOSDV_IP_TOS`(0xa0/0xc0)를 넘긴다. shim은 그 프로세스의 모든 UDP 소켓에 IP_TOS를 설정한다. `zcu-health@` 유닛은 shim이 없어서 CS0이다. zenoh-c는 엔드포인트의 `#dscp`로 같은 값을 한 번 더 설정한다. |
| HPC, CycloneDDS | GUI가 띄우는 노드마다 같은 shim을 붙인다(`nodes.yaml`의 `traffic_class`). safety supervisor는 `control`, 카메라 노드 다섯 개는 `camera`이고, 나머지 노드는 CS0이다. |
| HPC, Zenoh·Bridge·Pico | 라우터나 브리지가 IVN 송신을 대신한다. 흐름마다 HPC 포트가 다르므로 nftables가 출발지 포트로 표시한다(`network_qos/autosdv_dscp_qos.py`). |

## 포트 할당

| 흐름 | Cyclone DDS (ZCU participant index) | Zenoh·Bridge·Pico (HPC 포트) | DSCP |
| --- | --- | --- | --- |
| 카메라 ZCU→HPC 데이터 | 목적지 = HPC 영상 노드 사용자 포트 `7451/7453/7455/7457/7459` | 목적지 `17451/17453/17455/17457/17459` | CS5 |
| 카메라 세션의 HPC→ZCU 송신 | 영상 노드 프로세스 (shim) | 출발지 `1745x` | CS5 |
| 제어 HPC→ZCU 데이터 | 목적지 ZCU `7425` (`zcu_control`, index 7) | 출발지 `7448` | CS6 |
| 제어 세션의 ZCU→HPC 송신 | `zcu_control` 프로세스 (shim) | 목적지 `7448` | CS6 |
| Health ZCU→HPC | `zcu_health_node` (index 6, `7422/7423`) | 목적지 `7449` | CS0 |
| ZCU 카메라 프로세스 수신 포트 | `zcu_camera` (index 5, `7420/7421`) | — | — |

카메라는 Zone 1이 image_01(웹캠)과 image_02, Zone 2~4가 각각 image_03~05를 보낸다.

| 송신 보드 | 카메라 토픽 | Cyclone DDS 목적지 | Zenoh 계열 목적지 |
| --- | --- | --- | --- |
| ZCU1 `10.0.0.2` | `image_01_raw`, `image_02_raw` | `7451`, `7453` | `17451`, `17453` |
| ZCU2 `10.0.0.3` | `image_03_raw` | `7455` | `17455` |
| ZCU3 `10.0.0.4` | `image_04_raw` | `7457` | `17457` |
| ZCU4 `10.0.0.5` | `image_05_raw` | `7459` | `17459` |

## 남는 한계

- Zenoh 계열에서 세션을 맺을 때의 INIT/OPEN 패킷도 그 세션의 등급으로 나간다. 2안은 세션 단위로 분류하므로 이는 정책에 맞는 동작이다.
- ARP 같은 비IP 프레임은 DSCP가 없어서 스위치 포트의 기본 우선순위를 따른다.
- SJA1110의 GCL은 출력 포트마다 적용된다. GM에서 ZCU 쪽 포트만 Q6을 닫으면 HPC→ZCU 방향만 막힌다(2026-10-07 실측).

구현 근거: [HPC DSCP 규칙](../network_qos/autosdv_dscp_qos.py), [HPC 통신 모드 호스트](../scripts/communication_host.py), [GUI 노드 정의](../src/autosdv_gui_pkg/config/nodes.yaml), ZCU의 `include/zcu_config.h`, `systemd/zcu-camera.service`, `systemd/zcu-control.service`, `src/dscp_shim/dscp_shim.c`.
