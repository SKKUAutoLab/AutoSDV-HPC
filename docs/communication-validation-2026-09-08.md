# 공통 통신 모드 실기 검증 — 2026-09-08

HPC `10.0.0.10`과 ZCU `10.0.0.2`–`.5`에서 FastDDS → CycloneDDS →
Zenoh → CycloneDDS 전환을 실행했다. 세 번 모두 5개 장치의 서비스 기동,
ZCU 실행 파일 해시 확인, 상태 저장이 성공했다. 최종 모드는 **CycloneDDS**이며
GUI는 종료 상태로 유지했다. 검증 프로그램은 제어 명령을 발행하지 않았다.

## 수신 검증

각 이미지에 기존 participant 20–24 프로파일을 적용한 별도 ROS 프로세스를 사용했다.
20초 동안 BEST_EFFORT 구독으로 수신했고, 각 스트림의 첫 프레임을 OpenCV로
디코딩했다. 아래 값은 시작·발견 시간을 포함한 메시지 수로, 정밀 성능 벤치마크는 아니다.

| 조건 | image 01 | 02 | 03 | 04 | 05 | Health |
|---|---:|---:|---:|---:|---:|---:|
| CycloneDDS 최초 전환 | 598 | 598 | 597 | 599 | 597 | 161 |
| Zenoh 전환 | 601 | 600 | 600 | 601 | 601 | 160 |
| CycloneDDS 복귀 + ZCU 재부팅 | 598 | 601 | 601 | 601 | 600 | 160 |

모두 약 30 FPS, 메시지 크기 필드 640×480, encoding `jpeg`이며 디코딩 결과는
480×640×3이었다. Health에는 네 zone 모두 포함됐고 카메라·런타임 상태는 OK였다.
CAN RX는 기존 FastDDS 기준 측정과 동일하게 네 zone 모두 미관측 ERROR였다.
Zenoh QoS 서비스 수정·재시작 후 image 01을 별도로 다시 측정해 601개를 수신했다.
첨부 JSON의 Zenoh image 01 상세 샘플은 이 재측정 값이다.

## QoS와 수정 사항

- ZCU의 최소 Linux 이미지에 `ldd`가 없어 사전 점검이 실패했다.
  `/lib/ld-linux-aarch64.so.1 --list` 대체 경로를 추가해 실제 라이브러리 오류는
  그대로 차단하면서 네 보드 점검을 통과시켰다.
- HPC QoS가 존재하지 않는 `enp4s0`에 규칙을 만들어 패킷 카운터가 0이었다.
  `10.0.0.10`을 소유한 인터페이스를 검사하고 현재 장치의 `enP2p1s0`를
  QoS 명령에 명시하도록 수정했다. 주소를 찾을 수 없으면 사전 점검에서 차단한다.
- HPC의 실제 IVN 포트에서 패킷을 캡처했다. 두 모드 모두 ZCU 카메라 트래픽이
  CS5(DSCP 40)였다. CycloneDDS는 목적지 포트 7451/7453/7455/7457/7459,
  Zenoh는 7447이었다.
- 수정 후 HPC→ZCU의 Zenoh 출발지 포트 7448 패킷은 네 보드 모두 CS6(48),
  7447 패킷은 CS1(8)이었다. 이는 해당 포트의 패킷 분류 확인이며 주행 명령의
  수신·CAN 변환 검증을 의미하지 않는다. CycloneDDS 제어 CS6 실패킷 검증은 미실시다.

## 전환·복원·GUI

- Zenoh 실행 중 Bridge/Pico Apply를 요청했을 때 누락된 ZCU 빌드로 사전 차단됐다.
  기존 Zenoh 서비스와 저장 상태는 유지됐다.
- ZCU 네 대를 실제 재부팅했다. boot ID 변경, CycloneDDS 자동 기동,
  선택 빌드와 실행 파일 해시 일치, 이미지·Health 수신 복구를 확인했다.
- HPC는 서비스 정지 → 휘발성 `/run/autosdv-communication-mode` 제거 → 서비스
  기동으로 저장 상태 복원을 확인했다. HPC 자체 재부팅은 실시하지 않았다.
- GUI의 프로파일 로더가 저장된 CycloneDDS를 읽고 `rmw_cyclonedds_cpp`와
  `cyclonedds_ivn.xml`을 선택하는 것을 확인했다. 실제 화면 버튼 조작과 GUI
  프로세스 재시작은 미검증이다. 앞선 세션에서는 offscreen Qt로 실제 보드
  Check 동작 및 모드별 Apply 활성화 조건을 확인했다.

## 남은 범위와 자동 테스트

Bridge/Pico ZCU 백엔드는 빌드가 없어 데이터 경로 검증이 불가능하다.
제어/CAN 전달, 제어 운용 한도, 지연·만료·정지 동작, 장시간 자원 측정은 별도 검증이
필요하다. 따라서 저장 상태의 `data_path_verified`는 `false`로 유지했다.
CycloneDDS의 native DDS endpoint에서 ROS type-hash metadata 누락 경고가 발생했지만
이미지·Health 수신은 성공했다. 이 경고와 서비스의 H.264 환경 변수에도 실제 메시지가
JPEG인 점은 후속 백엔드 검토 항목이며, 이번에 이미지 설정을 변경하지 않았다.

프로파일 단위 테스트가 실제 장치의 저장 모드에 영향을 받는 문제를 발견해 임시 상태
파일로 격리했다. 수정 후 **44개 pytest 테스트 통과**, ROS·workspace 환경을 source한
상태에서 launch_pkg의 communication_mode CTest도 통과했다. 변경은 Python 어댑터와
테스트이며 별도 C++ 재빌드는 필요하지 않았다.

측정 상세: [수신·패킷·최종 장치 상태 JSON](validation/communication-2026-09-08.json).


## GUI 경고 후속 수정

같은 날짜의 GUI 경고 보고 후 두 원인을 수정하고 ZCU 네 대에 적용했다.

- `ROS_LOCALHOST_ONLY=0`도 rcl에서 deprecation 경고를 발생시켰다. 공통 모드
  프로파일이 해당 값을 빈 문자열로 덮어쓰도록 변경했다. 외부에서 `1`을 상속한
  경우에도 새 discovery 설정으로 동작하고 경고가 없는 것을 확인했다.
- native CycloneDDS endpoint의 USER_DATA가 비어 있어 ROS type hash가 INVALID로
  조회됐다. 이미지/Health writer와 제어/image-monitor reader QoS에
  `typehash=RIHS01_…;` 메타데이터를 추가했다. 기존 Zenoh backend의 해시 상수를
  재사용했으며 HPC의 ROS 생성 type-description JSON 세 종류와 일치함을 확인했다.
  포맷 근거는 [Jazzy rmw_dds_common 구현](https://github.com/ros2/rmw_dds_common/blob/jazzy/rmw_dds_common/src/qos.cpp)이다.
- 수정 전 INVALID였던 이미지 writer 5개, Health writer 4개, 제어 reader 4개가
  모두 예상 해시와 정확히 일치했다. 경고 로그 억제 옵션은 사용하지 않았다.
- ZCU별 gateway/health 빌드 후 공통 CycloneDDS Apply로 재기동했다. 보드의 시간
  차이 때문에 make clock-skew 경고가 있었으나 수정 소스 컴파일·링크 성공과
  실제 실행 endpoint의 메타데이터 변경을 확인했다. 보드 시계는 변경하지 않았다.
- 수정 후 20초 수신: 이미지 01–05 각각 601/573/601/601/601개, Health 160개.
  모든 이미지의 JPEG 디코딩 성공, 여섯 수신 프로세스의 stderr는 모두 비어 있었다.
  단위 테스트 44개와 communication_mode CTest도 통과했다.

앞서 기록한 type-hash 경고는 이 후속 수정으로 해결됐다. 제어/CAN 운용 검증 범위는
확장하지 않았으며, 최종 모드는 CycloneDDS다. GUI는 다시 실행하면 수정된 환경을 읽는다.
[해시와 수신 측정 상세](validation/cyclonedds-typehash-fix-2026-09-08.json).


## GUI Health·카메라 미수신 후속 수정

GUI 실제 경로 검증에서 프로세스는 RUNNING이고 카메라 publisher도 발견됐지만
Health 본문은 미수신, preview sequence는 1(소스 설정으로 증가한 값)이었다.
ZCU 서비스가 장시간 실행된 상태에서 단독 기본 프로파일 검사도 Health 0건이었다.
HPC 프로파일의 AllowMulticast만 false로 바꾼 대조 검사에서는 ZCU를 재시작하지
않고 10초에 Health 80건을 수신했다. 기존 IVN 프로파일의 멀티캐스트 의존성이
현재 네트워크에서 정상 전달을 보장하지 못하는 것이 확인됐다. 특정 스위치의
필터/IGMP 설정까지 원인을 확정한 것은 아니다.

공통·카메라 CycloneDDS 프로파일 6개에 AllowMulticast=false를 적용하고,
HPC 내부 프로세스 발견을 위해 10.0.0.10도 명시적 peer로 추가했다.
공통 프로파일은 ParticipantIndex=auto(최대 19), 카메라는 기존 20–24를 유지한다.
유니캐스트 강제 설정은 [CycloneDDS 공식 설명](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html)에 따른다.
launch_pkg 재빌드와 설치 프로파일 내용 확인을 완료했다. ZCU 바이너리나 서비스를
이번 수정 검증 중 재시작하지 않았다.

검증은 offscreen Qt로 실제 MainWindow와 NodeProcessManager를 생성하고 실제
ROS 자식 프로세스·물리 ZCU를 연결해 진행했다. 카메라 1개 검사에서 30초 동안
preview sequence가 799까지 증가하고 4개 zone의 Runtime·Health가 표시됐다.
GUI를 종료·재생성한 다음 5개 카메라를 시작하고 preview 체크박스를 순서대로
선택했다. 각 카메라에서 20프레임 이상과 GUI 표시용 QImage 640×480을 확인했다.
최초 브리지 기동을 포함한 camera 01 준비는 7.44초, 이후 02–05는 0.70/0.81/
0.91/0.81초였다. 고정 5초 대기 검사는 최초 브리지 준비보다 짧아 실패했으므로,
실제 프레임·이미지 조건으로 대기하고 통과 여부를 확인했다. 별도 OpenCV 창은
끄고 GUI 내 미리보기 경로를 검사했다. 주행·motion planner는 시작하지 않았다.

[실제 GUI 경로 측정·로그](validation/gui-unicast-2026-09-08.json).
검사 후 GUI와 자식 노드는 종료했고 CycloneDDS 상태는 유지했다.
회귀 테스트 45개 및 communication_mode CTest 통과, git diff --check 통과.
앞선 단독 수신 검사는 GUI 재실행·내부 노드 경로까지 보증하지 못했으며,
이 후속 검사로 Health·카메라 미리보기 경로를 추가 확인했다.
