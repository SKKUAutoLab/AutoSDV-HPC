# Bridge 발행자 광고 — 2026-09-10

ZCU 4대에 ros2dds 발행자 광고를 구현·배포했다. GUI가 카메라를 구독하지 않아도
HPC DDS에서 발행자를 발견하며, 연결·생산 상태가 정상인 카메라는 초록색으로 표시된다.
HPC 진단 판정이나 GUI 색상 코드를 완화하지 않았다.

## 변경

인접 S32G3_ZCU 저장소의 `ros_keyexpr.cpp/.hpp`에 ros2dds 1.10.0 MP 토큰 생성 함수를
추가했다. ImagePublisher와 DiagnosticPublisher가 Zenoh publisher 생성 후 토큰을
선언하며, publisher 객체 파괴 시 토큰도 회수한다. session ZID로 발행 세션을 구분한다.
카메라 5개와 Health 발행자만 광고하며, 별도 가상 카메라나 상시 이미지 구독자는 없다.

- 토픽과 ROS 타입의 `/`는 `§`로 치환한다.
- QoS는 keyless, VOLATILE, KEEP_LAST를 사용한다. 카메라 BEST_EFFORT/depth 1,
  Health RELIABLE/depth 10을 광고한다.
- 기존 타입 해시를 USER_DATA의 `typehash=RIHS01_...;`로 전달한다.
- 기존 plain topic CDR payload, 7447/7448 분리, 이미지 설정은 유지했다.
- Native Zenoh는 기존 rmw_zenoh 광고를 사용한다. Bridge 광고는 Bridge 빌드에만 적용한다.

규약 및 수명 처리는 [ros2dds 1.10.0 liveliness 구현](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/blob/1.10.0/zenoh-plugin-ros2dds/src/liveliness_mgt.rs)과
[원격 발행자 route 관리](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/blob/1.10.0/zenoh-plugin-ros2dds/src/routes_mgr.rs#L309)를 기준으로 했다.

## 실기 결과

사용자의 기존 GUI와 image_01 구독을 유지했다. 별도 진단 관찰 노드가 실제
`/ivn/diagnostics/status`를 수신하고 GUI의 `_graph_diagnostic_states`에 적용했다.
화면 클릭·스크린샷 검사가 아니라 실제 GUI와 같은 메시지 및 색상 판정 함수의 검증이다.
시험 중 주행은 비활성 상태였으며 제어 값을 입력하지 않았다.

| 단계 | 관측 |
|---|---|
| 수정 전 | image_01 publisher만 발견. 미구독 image_02–05 publisher는 없음 |
| 광고 적용 후 | image_02–05 구독자 0개인데도 publisher 각 1개, 카메라 5개 모두 ok |
| 5개 임시 이미지 구독 | 각 117–120프레임 수신, 모두 ok |
| 임시 구독 제거 | publisher 각 1개 유지, 미구독 카메라도 모두 ok |
| 미구독 ZCU4 gateway 종료 | image_05 publisher 0개, image_05 error |
| ZCU4 gateway 재시작 | image_05 publisher 재생성, ok 복구 |
| HPC 브리지 서비스 재시작 | 5개 publisher 자동 재발견, 모든 카메라 ok 및 Runtime 정상 |
| 새 관찰 노드로 최종 확인 | image_01만 구독 중, 5개 publisher 발견, 전체 진단 level 0 |

미구독 image_05의 IVN 유입을 5초 캡처했을 때 ZCU4→HPC:7447은 2패킷/120바이트였고
300바이트 초과 패킷은 0이었다. 해당 구간에서 광고를 위해 영상 전체를 계속 수신하지
않음을 확인했다. 이것은 제한된 관측 결과이며 장시간 성능·지연 측정은 아니다.

DDS에서 조회한 카메라 타입 해시는 기존 Image RIHS01 해시와 일치했다. 실제 writer
QoS는 ros2dds 정책에 따라 RELIABLE/depth 1이다. Health 타입 해시도 확인했다.

## 검증 및 최종 상태

- 로컬 C++ keyexpr 검사를 Native Zenoh/Bridge 각각 실행해 통과했다.
- ZCU 4대에서 Bridge와 Native Zenoh 빌드 모두 성공했다.
- 각 빌드의 CTest는 health policy와 keyexpr 2개 통과, loopback router 없는
  transport smoke 1개 skip이다. 실제 연결은 위 수명·구독 검증으로 확인했다.
- 다섯 변경 소스 파일은 로컬과 네 ZCU의 SHA256이 일치한다.
- 네 ZCU의 실행 바이너리는 공통 전환기의 Bridge verify를 통과했다.
- 상태는 `ready / zenoh_bridge`이며 사용자의 GUI와 기존 image_01 구독을 유지했다.

[원시 검증 기록](validation/zenoh-bridge-announcement-2026-09-10.json).
ZCU 변경 전 파일은 각 보드 `/tmp/pre-announcement-sources.tgz`, 이전 Bridge 실행 파일은
`/tmp/pre-announcement-zcu`와 `/tmp/pre-announcement-health`에 임시 백업했다.

## 판정 범위

광고는 발행 객체의 존재를 나타내며 정상 프레임 도착을 보장하지 않는다. 센서 입력이
끊기거나 프로세스가 멈추면 기존 producer hook과 Health 신선도 검사가 상태에 반영된다.
이번 검사는 gateway의 정상 종료·재시작과 브리지 재시작이다. 전원 차단·케이블 분리 시
Zenoh lease 만료까지의 시간은 측정하지 않았다.

원격 광고가 사라져도 로컬 DDS 구독자가 남아 있으면 bridge writer가 유지될 수 있다.
따라서 실제 장애 판정에는 기존 Health 검사가 계속 필요하다. 원래 ZCU ROS 노드 이름을
복원하지 않으며 DDS endpoint는 HPC bridge 노드 아래에 나타난다.
