# Zenoh Bridge 구현·실기 검증 — 2026-09-10

> 이 문서는 최초 Bridge 구현의 검증 기록이다. 이후 발행자 광고를 추가해 미구독 상태의 발견·Health 판정을 개선했다. [후속 구현·검증](zenoh-bridge-announcement-2026-09-10.md)을 참조한다.

ZCU 4대의 `zenoh_bridge` 백엔드를 구현·빌드·배포했다. HPC 내부는 CycloneDDS
(loopback), ZCU↔HPC는 Zenoh로 통신한다. 기존 GUI Communication mode의
Check → Apply로 전체 장치를 전환할 수 있다. 최종 선택은 `ready / zenoh_bridge`이며
검증 GUI와 자식 노드는 종료했다. 당시 Pico는 미구현 상태로 사전 검사에서 차단됐다. 이후 Linux Pico는
[별도 구현](zenoh-pico-linux.md)으로 추가됐다.

## 구현

- 인접 `S32G3_ZCU` 저장소의 CMake에 `zenoh_bridge`를 추가했다. 기존 Zenoh
  백엔드를 재사용하고 `ZCU_ZENOH_BRIDGE`로 plain topic key를 선택한다.
  encapsulated CDR는 유지하고 rmw_zenoh attachment/liveliness는 생략한다.
- 기본 Health는 fresh producer hook으로 카메라 상태를 판단한다.
  원격 ROS graph가 없으므로 `publisher_discovered=false`가 정상이다.
  선택형 payload monitoring은 필요할 때 카메라 세션(7447)을 별도로 연다.
- HPC는 기존 zenoh-bridge-ros2dds 1.10.0을 카메라(7447)와 제어·Health(7448)로
  나눠 실행한다. DDS는 모든 HPC 노드와 브리지에서 동일한 loopback 설정을 쓴다.
- allow 목록은 로컬 DDS endpoint 기준으로 이미지 구독자, Health 구독자,
  제어 발행자만 허용한다. 현재 비활성인 status 및 service/action은 제외했다.
- 기존 GUI/Manual Drive 코드는 이번에 변경하지 않았다. 공통 모드 전환과
  환경 상속 경로에 새 ZCU 빌드를 연결하고 실제 적용을 검증했다.

기본 namespace의 wire 형식은 [ros2dds 1.10.0](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/tree/1.10.0)의
plain Zenoh 연동 규약을 사용한다. Domain ID는 Zenoh key에 포함되지 않으므로
동시 독립 실험에는 별도 namespace/port/network 격리가 필요하다.

## 실기 검증

실제 MainWindow·ModeDialog·NodeProcessManager를 **offscreen Qt**로 실행했다.
실제 ROS 자식 프로세스, ZCU 4대, 현재 카메라 입력을 사용했으며 목업은 사용하지 않았다.
화면을 사람이 직접 조작한 검사와는 구분한다.

| 경로 | 관측 결과 |
|---|---|
| CycloneDDS → Bridge | HPC+ZCU 4대 사전 검사·적용·실행 파일 검증 통과 |
| Health → 진단 → GUI | zone 1–4 Runtime 수신 및 all components healthy 표시 |
| 이미지 01–05 → GUI | 각 20–21프레임, 실제 QImage 640×480 |
| GPU YOLO → GUI | DetectionArray 130개, YOLO preview 20프레임 |
| Manual Drive → HPC | manual_cmd 21개, 실제 CycloneDDS RMW 로드 |
| Supervisor → Bridge → ZCU CAN | 제어 출력 249개; 네 보드 각각 정지 CAN 10프레임 |
| 브리지 서비스 재시작 | 기존 GUI에서 제어·YOLO 수신 복구, 각 카메라 preview 20–22프레임 |
| GUI ModeDialog | Bridge Check → Apply 성공, 5대 ready; drive/motion 자동 복원 제외 |
| 새 GUI 프로세스 | Health·5개 영상·YOLO 재수신, Lane/Path/Motion 기동·CycloneDDS 로드 |
| 허용 목록 밖 이미지 06 | CDR 샘플 30개 Zenoh 발행, DDS 수신 0; 동시에 실제 image 05는 210개 수신 |
| 선택형 ZCU Health payload monitor | control 세션에서 probe 생성 후 camera 세션으로 3초간 90개 수신 |
| 기존 native Zenoh 회귀 | 네 보드 재빌드, GUI 영상·Health·YOLO·Manual 메시지 및 Zenoh RMW 확인 |

CAN 값은 `111#00000007`, `123#0000000000000000`이다. 검증 하네스에서
heartbeat source를 `none`으로 고정해 실제 Supervisor를 비활성 상태로 유지했다.
Manual 노드의 메시지 생성과 정지 제어 경로를 확인했으며 비영 명령·실제 주행은
시험하지 않았다. 운영 GUI의 heartbeat/승인 코드는 변경하지 않았다.

새 GUI 실행에서는 Manual Drive를 정지한 뒤 Lane/Path/Motion을 시작했다.
이는 프로세스 기동과 통신 경로 검사이며 인식·경로 추종 정확도 판정은 아니다.
ModeDialog 적용과 새 GUI 수신을 각각 확인했으며, `main()`의 exec 재시작과
이전 노드 전체 자동 복원을 한 번에 실행한 검사는 아니다.

최종 Bridge 재적용 후 12초 구독 검사에서는 이미지 01–05가 각각 359개 수신됐고,
네 zone의 Health sequence도 계속 증가했다. 이때 GUI와 제어 노드가 종료되어
CAN RX가 관측되지 않았으며, 기존 Health 정책은 CAN 항목을 ERROR로 보고했다.
이는 최종 상태에 실제로 포함된 진단이며, 수신 성공을 모든 구성 요소의 정상 상태와
같은 뜻으로 취급하지 않았다. GUI 운용 검사 중에는 all components healthy를 확인했다.

## 전송과 QoS

30초 AF_PACKET IVN 캡처에서 Zenoh UDP 7447/7448을 확인했다. HPC↔ZCU IPv4 UDP
RTPS 패킷은 0, IP 단편은 0이었다. 관측 구간의 결과이며 장시간 무유실 보장은 아니다.

- ZCU 이미지 egress: TOS 0xa0(CS5).
- HPC 제어 egress: 정상 전송 중 TOS 0xc0(CS6).
- HPC 카메라 링크 응답: TOS 0x20. ZCU Health/제어 링크 응답: 0x00.
- 캡처에는 서비스 재시작도 포함되어 있으며 재연결 초기 일부 7448 패킷은 0x00이다.
  QoS watcher의 규칙 적용 전 패킷까지 전부 CS6라고 주장하지 않는다.

실제 DDS endpoint 조회 결과는 다음과 같다. 브리지의 이미지 writer는 로컬
BEST_EFFORT 구독자로 route를 만들더라도 RELIABLE로 나타났다. 비교 실험 시
이 차이를 기록해야 한다. `reliable_routes_blocking`은 제어 QoS를 바꾸지 않는다.

| HPC 발행 endpoint | Reliability | Depth |
|---|---|---|
| zenoh_bridge_camera / image_01_raw | RELIABLE | 1 |
| zenoh_bridge_control / ivn/zcu_health | RELIABLE | 10 |
| safety_supervisor_node·autosdv_gui / topic_control_signal | BEST_EFFORT | 1 |

ZCU gateway와 Health의 실제 `/proc/PID/maps`에서 libzenohc를 확인하고
libddsc/libfastrtps/libfastdds가 없음을 확인했다. HPC GUI 자식 노드에서는
librmw_cyclonedds_cpp.so를 확인했다. 배포한 8개 소스 파일의 SHA256은 네 보드와
로컬 저장소가 일치한다.

## 재현·검증 기록

```sh
./scripts/select_communication_mode.sh check zenoh_bridge
./scripts/select_communication_mode.sh apply zenoh_bridge
```

GUI가 실행 중이면 GUI의 Communication mode에서 Check → Apply한다.
ZCU 빌드 명령은 [운영 문서](zenoh-dds-bridge.md)에 있다.

- 공통 전환·GUI·QoS pytest: **89개 통과**.
- ZCU 4대 × Bridge/native Zenoh 빌드: 모두 성공.
- 각 빌드의 CTest: health policy/keyexpr 2개 통과, loopback router가 없는
  transport smoke 1개 skip. 실기 연결은 위 GUI·CAN 검사로 별도 확인했다.
- 보드 시계 차이에 따른 make clock-skew 경고가 있다. 변경 파일의 컴파일·링크,
  실행 바이너리 해시와 실기 수신으로 검증했으며 보드 시계는 변경하지 않았다.
- 최종 수신 기록의 첫 JSON 저장은 DiagnosticStatus.level의 bytes 표현 때문에
  실패했다. 검증 스크립트에서 숫자로 변환한 후 재실행했다. 운영 코드 오류는 아니다.

원시 관측, 모드 적용 로그, 빌드 결과와 최종 상태는
[검증 JSON](validation/zenoh-bridge-2026-09-10.json)에 보관했다.

이미지 설정은 기존 640×480 JPEG/30 FPS 조건을 유지했고 자원 상한을 추가하지 않았다.
지연 실측·운용 한도 결정·장시간 부하·CANoe 재검증·주행·Pico 구현은 수행하지 않았다.
저장 상태의 `data_path_verified=false`는 공통 전환기가 프로세스 준비만 판정한다는
기존 정책이며, 실제 경로 검증 결과는 이 문서와 JSON에 별도로 기록한다.
