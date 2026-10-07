# Linux Zenoh-Pico 모드

`zenoh_bridge_pico`는 ZCU의 Zenoh C 런타임을 zenoh-pico C API로 교체한다.
HPC GUI·Manual Drive·인식/계획·진단 노드는 CycloneDDS를 사용하고, HPC의
zenoh-bridge-ros2dds가 ZCU 통신을 변환한다. Linux 구현이며 M코어/RTOS 포팅은 포함하지 않는다.

```text
ZCU 카메라 01–05 → Pico → Zenoh UDP :7447 → HPC Bridge → CycloneDDS → GUI/인식
ZCU Health       → Pico → Zenoh UDP :7448 → HPC Bridge → CycloneDDS → 진단/GUI
ZCU CAN          ← Pico ← Zenoh UDP :7448 ← HPC Bridge ← CycloneDDS ← Supervisor/Manual 경로
```

gateway의 카메라·제어와 별도 Health 프로세스는 각각 세션을 연다. 기존 Bridge와
같은 endpoint·카메라 조건(640×480 JPEG, 30 FPS)을 유지한다. Pico는 Rust endpoint의
`#dscp` 설정을 제거하고 Linux tc로 카메라 egress CS5를 적용한다. HPC 제어 CS6는
기존 QoS 관리자가 적용한다. TCP는 격리 상호운용 시험에서 별도로 검증한다.

## 설치와 전환

인접 `S32G3_ZCU` 저장소에서, 각 ZCU에 해당 저장소와 `patches/`를 함께 배포한다.
라이브러리 교체 전 gateway/Health 서비스를 종료하고 GUI도 종료한다.

```sh
# 각 ZCU에서 실행. zone_1에는 1, zone_2에는 2, ... 를 사용한다.
cd /home/root/Workspace/S32G3_ZCU
systemctl stop zcu.service zcu-health@zone_1.service
scripts/install_zenoh_pico.sh
cmake -S . -B build-zenoh_bridge_pico \
  -DZCU_MIDDLEWARE=zenoh_bridge_pico -DZCU_ZONE_ID=1 -DBUILD_TESTING=ON
cmake --build build-zenoh_bridge_pico -j2
ctest --test-dir build-zenoh_bridge_pico --output-on-failure
```

네 보드의 빌드가 준비되면 HPC에서 공통 전환기를 실행한다.

```sh
./scripts/select_communication_mode.sh check zenoh_bridge_pico
./scripts/select_communication_mode.sh apply zenoh_bridge_pico
```

GUI에서는 Communication mode → **Zenoh–DDS Bridge + Pico** → Check → Apply를
사용한다. 기존 공통 전환 경로에 연결되므로 별도 Pico GUI를 실행하지 않는다.
HPC의 RMW는 `rmw_cyclonedds_cpp`이며, Pico는 ZCU 런타임이다. 사전 검사는
**gateway와 Health 모두** 동적 `libzenohpico.so`에 연결됐는지 확인한다.
전환기의 `data_path_verified=false`는 프로세스 준비만 판정한다는 기존 정책이다.
실제 경로 검증은 별도 기록으로 남긴다. ZCU의 과거 `select_middleware.sh` 대신
이 공통 전환기를 사용한다.

## 고정 빌드와 필수 패치

- zenoh-pico 1.10.0, commit `96006957fddef401c20c8c2d813c2a630b666974`.
- 공식 archive SHA256: `2638ee27c76a235940606a28ecc64757965617f0eb269d4b3609db7fbb0759b0`.
- Release, shared library, multithread executor, auto reconnect, liveliness 및 unstable API 사용.
- fixed endpoint client. Scouting, multicast transport, UDP multicast link 비활성.
- `BATCH_UNICAST_SIZE=1400`, `FRAG_MAX_SIZE=131072`. 후자는 선택형 이미지 payload
  monitor의 **수신 재조립** 한도이며 서비스 전체 RAM 상한이 아니다.
- Linux 애플리케이션의 C++17·Fast-CDR·pthread 의존성은 유지한다. ZCU Pico
  실행 파일에 zenoh-c·CycloneDDS·FastDDS transport를 링크하지 않는다.

설치 스크립트가 archive 해시 확인 후
`S32G3_ZCU/patches/zenoh-pico-1.10.0-fragment-reliability.patch`를 적용한다.
원본 라이브러리로 대체하면 안 된다.

원본 `src/transport/common/tx.c`의 `__unsafe_z_serialize_zenoh_fragment`는
`reliability == Z_RELIABILITY_RELIABLE`이라는 bool을 reliability enum 자리에 전달한다.
이 버전은 `RELIABLE=0`, `BEST_EFFORT=1`이므로 fragment의 신뢰성 비트가 반대로
기록된다. 송신 sequence가 다른 채널에 반영되어 이후 작은 Health나 광고 회수가
누락될 수 있다. 해당 인자를 `reliability` 그대로 전달하도록 한 줄 수정했다.
이는 AutoSDV 로컬 패치이며 upstream 릴리스에 반영됐다고 주장하지 않는다.

실제 재현에서는 큰 BEST_EFFORT 이미지와 RELIABLE Health를 같은 세션으로 보내면
Health 60개 중 1개만 수신했다. 패치 후 같은 시험에서 TCP·UDP 모두 이미지
각 크기 20개와 Health 60개를 수신했다. 세션 분리로 증상만 피하는 대신 원인을 수정했다.

## 메시지와 발견

- plain ROS topic key + encapsulated XCDRv1 CDR를 사용한다. `rmw_zenoh` 전용
  key/attachment를 사용하지 않는다. `Image`, `DiagnosticArray`, `MotionCommand`를 지원한다.
- Image/Health publisher 생성 시 기존 Bridge와 동일한 ros2dds MP liveliness 광고를
  선언하고 파괴 시 회수한다. 타입 해시와 QoS를 포함해 HPC DDS 발견에 반영한다.
- 광고는 발행 객체의 존재를 나타낸다. 실제 정상 판정은 기존 producer heartbeat와
  Health 신선도 검사도 사용한다. GUI 이미지 구독 없이도 연결 상태를 확인한다.
- MotionCommand는 CDR 20바이트만 디코딩하고, 잘못된 길이/encapsulation은 CAN
  callback에 전달하지 않는다. 제어 지연·만료·운용 한도는 이번에 새로 정하지 않는다.
- 선택형 Health 이미지 monitor는 별도 카메라 세션에서 payload를 수신한다.

## 격리 회귀 시험 재현

HPC에서 Pico 설치 prefix를 예를 들어 `/tmp/autosdv-pico-install`로 지정해 빌드한 뒤,
인접 ZCU 저장소를 `build-pico-host`에 구성한다. Fast-CDR는 ROS Jazzy의 것을 사용한다.

```sh
# S32G3_ZCU에서
scripts/install_zenoh_pico.sh /tmp/zenoh-pico-1.10.0.tar.gz /tmp/autosdv-pico-install
cmake -S . -B build-pico-host -DZCU_MIDDLEWARE=zenoh_bridge_pico \
  -DCMAKE_PREFIX_PATH='/tmp/autosdv-pico-install;/opt/ros/jazzy' -DBUILD_TESTING=ON
cmake --build build-pico-host --target pico_interop_test -j2
```

HPC 저장소에서 별도 터미널의 격리 bridge를 띄운다.

```sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DISTRO=jazzy CYCLONEDDS_URI="file://$PWD/config/dds/cyclonedds_bridge_local.xml" \
  tools/zenoh-bridge-ros2dds/zenoh-bridge-ros2dds -c docs/examples/pico_interop.json5
# 다른 터미널에서도 위 두 setup 파일을 source한 뒤
python3 docs/examples/pico_interop.py
```

이 시험은 domain **93**, loopback **27447** 전용이다. 정수 경계값 제어 명령을
사용하므로 운영 bridge 설정으로 바꾸지 않는다. C++ probe도 이 endpoint만 허용한다.
이미지 payload 0/43,200/65,503바이트의 전체 값·헤더, Health 필드, 양방향 제어,
잘못된 CDR 거부, 분할 이미지와 Health 혼합 발행을 확인한다. 시험 종료 후 격리
bridge는 Ctrl-C로 종료한다. 일반 CTest의 transport smoke skip은 이 검사를 대신하지 않는다.

## 판정 범위

기능 구현과 실측 성능 비교는 구분한다. 자원 상한은 고정하지 않았으며 CPU/RAM 절감,
지연 분포, 장시간 무유실, 부하·손실 주입, M코어 이식 완료를 주장하지 않는다.
Pico 1.10.0의 UDP RELIABLE 선택만으로 재전송 복구를 보장할 수 없다. 기존 UDP
조건을 유지한 기능 시험이며 제어 운용 한도는 추후 실측 후 확정한다.

## 2026-09-10 실기 검증

ZCU 4대에 빌드·배포하고 공통 CLI와 GUI ModeDialog Check → Apply를 실행했다.
최종 선택 모드는 `ready / zenoh_bridge_pico`다. 검증 GUI 및 자식 노드는 종료했다.
GUI는 실제 MainWindow를 offscreen Qt로 실행했으며, 실제 ROS 프로세스와 물리
ZCU·카메라를 사용했다. 사람이 데스크톱에서 클릭한 검사와는 구분한다.

| 검사 | 결과 |
|---|---|
| 격리 CDR 혼합 발행 | TCP·UDP 각각 이미지 payload 0/43,200/65,503바이트 각 20개, Health 60개 수신 |
| 격리 제어 | int32 경계값·uint16 frame_id 일치, 잘못된 길이 및 encapsulation callback 0회 |
| 초기 bridge 부재 | TCP 연결 재시도 로그 이후 연결·동일 CDR 시험 통과 |
| 카메라 미구독 | 5개 DDS publisher 발견, 구독자 0개, GUI 카메라 상태 전부 ok |
| 5개 이미지 동시 구독 | 4초 관찰에서 카메라별 119–120프레임 수신 |
| 구독 해제 | 5개 publisher 및 정상 Health 유지 |
| ZCU4 gateway 종료·재시작 | image_05 publisher 제거·error, 재시작 후 publisher·ok 복구 |
| HPC bridge 재시작 | 미구독 publisher 광고 복구, 활성 GUI의 영상·YOLO·제어 수신 복구 |
| GUI 5개 preview | 각각 20–22프레임, 실제 QImage 640×480 |
| YOLO / Manual | DetectionArray 93개, manual_cmd 21개, 출력 제어 1,280개 관측 |
| Lane / Path / Motion | 실제 프로세스 기동 및 CycloneDDS RMW 로드 확인 |
| CAN 정지 전달 | 네 ZCU 각각 10프레임: `111#00000007`, `123#0000000000000000` |
| GUI Pico Check → Apply | 5대 적용 성공, 주행 관련 drive/motion 자동 복원 제외 유지 |
| 적용 후 새 GUI | 영상·Health·YOLO·Manual 재수신, 실행 중 drive_node의 CycloneDDS 로드 확인 |
| 선택형 Pico 이미지 monitor | ZCU2에서 image_05 payload 3초/90개 수신, 마지막 활동 시각 갱신 |
| IVN 패킷 | 활성 구독을 포함한 30초: Zenoh UDP 7447/7448, RTPS 0개, IP 단편 0개 |

카메라 egress는 CS5(0xa0), HPC 제어 egress는 CS6(0xc0)를 확인했다. 표의 프레임
수는 해당 관측 구간의 결과이며 FPS·무유실 보장이나 제어 지연 실측 결과가 아니다.
Pico의 UDP 송신 신뢰성 패치를 적용해도 UDP 재전송 보장이 추가되는 것은 아니다.

실제 ZCU gateway/Health의 `/proc/PID/maps`에서 `libzenohpico.so`를 확인했고
zenoh-c·CycloneDDS·FastDDS transport는 없었다. 네 보드의 패치된 라이브러리 SHA256은
`e07908ad8f784cf45715a6f57e319beaff140ba295ae2031a726fca0bc8f9879`로 일치했다.
배포 소스 해시도 로컬과 일치한다. HPC와 네 보드에서 최종 실행 바이너리 검증을 통과했다.

- 공통 모드·GUI ModeDialog·QoS pytest: 30개 통과.
- HPC Pico 빌드 및 네 ZCU 빌드 성공. 각 CTest는 health policy/keyexpr 2개 통과,
  loopback router 없는 transport smoke 1개 skip. 실제 통신은 위 시험으로 별도 검증했다.
- 보드의 기존 시계 차이로 make clock-skew 경고가 있으며, 시계는 변경하지 않았다.
  변경 파일의 컴파일·링크, 배포 해시와 실제 수신으로 검증했다.

시험 하네스는 heartbeat source를 `none`으로 고정했다. 비영 제어·실제 주행·CANoe
재검증은 수행하지 않았다. GUI 종료 후 정지 명령 발행도 끝나므로 기존 Health의 CAN
수신 항목이 ERROR가 될 수 있다. 운영 GUI의 상태 판정·승인 코드를 완화하지 않았다.
ModeDialog 적용과 새 GUI 기동을 각각 검증했으며 `main()`의 exec 및 모든 노드의
자동 복원을 한 번에 검증했다고 주장하지 않는다.

[원시 검증 기록](validation/zenoh-pico-linux-2026-09-10.json),
[격리 상호운용 결과](validation/zenoh-pico-interop-2026-09-10.json).
