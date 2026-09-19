# Zenoh GUI 경로 검증 — 2026-09-09

최종 상태: HPC와 ZCU 4대 모두 `ready / zenoh`. 검증용 GUI와 자식 노드는 종료했다.
운영 코드 변경 없이 설치된 환경을 검증했다. 지연 실측은 수행하지 않았다.

## 방법과 확인 결과

실제 MainWindow, NodeProcessManager, ModeDialog를 offscreen Qt에서 실행하고
실제 ROS 자식 프로세스와 ZCU 4대를 연결했다. 목업 메시지나 모의 ZCU는 사용하지
않았다. 화면을 사람이 직접 조작한 검사와는 구분한다.

| 경로 | 결과 |
|---|---|
| CycloneDDS → Zenoh | 공통 CLI 적용, 5개 장치 commit/실행 파일 검증 통과 |
| ZCU Health → 진단 노드 → GUI | 4개 zone의 Runtime·Health 표시 확인 |
| 카메라 01–05 → GUI 미리보기 | 각각 21프레임, 실제 QImage 640×480 확인 |
| 카메라 → GPU YOLO → GUI 미리보기 | DetectionArray 146개, YOLO preview 22프레임 확인 |
| Manual Drive | manual_cmd 20개 수신, 실제 librmw_zenoh_cpp.so 로드 확인 |
| Safety Supervisor | control 출력 250개 수신, 관측 출력은 모두 정지 값 |
| ZCU CAN | 네 보드 각각 10프레임 수신. 0x111/0x123 정지 값 확인 |
| Zenoh 라우터 재시작 | 기존 GUI·노드의 결과/제어 수신 복구 및 카메라별 preview 21프레임 재확인 |
| GUI ModeDialog | Check → Apply를 실제 5개 장치 대상으로 실행, Zenoh 재적용 성공 |
| 제어 노드 자동 복원 제외 | 재적용 시 restore 목록에서 drive/motion_planner 제외 확인 |
| 새 GUI 프로세스 실행 | 4개 Health, 카메라별 20–21프레임, YOLO 결과 148개·preview 20프레임 재확인 |
| Lane / Path Planner / Motion Planner | GUI에서 기동 성공 및 실제 Zenoh 라이브러리 로드 확인 |

초기 실행 중 camera 5개, YOLO, 진단, Supervisor, Manual Drive, preview bridge 등
10개 프로세스의 `/proc/PID/maps`에서 Zenoh RMW 라이브러리를 확인했다.
두 번째 실행에서는 Manual Drive 정지 후 자율주행 관련 노드를 시작해 통신 라이브러리를
확인했다. 동시에 수동·자율 제어 노드를 실행하지 않았다.

CAN 캡처 값은 `111#00000007`(중립 조향), `123#0000000000000000`(속도 0)이다.
현재 GUI는 Manual Drive 시작 시 source를 manual로 자동 선택한다. 검증 하네스는
실제 Supervisor를 비활성 상태로 유지하도록 heartbeat의 requested source를 none으로
고정했다. 따라서 이 검사는 Manual Drive의 메시지 생성·Zenoh 사용 및 정지 출력 경로를
확인한 것이며, 주행 승인 상태의 비영(非零) 명령 중재·주행 동작까지 검증한 것은 아니다.
첫 하네스는 source가 none으로 유지될 것이라는 과거 GUI 동작을 가정해 assertion이
실패했다. 현재 동작을 확인해 하네스의 heartbeat를 비활성으로 고정한 후 재검증했다.
운영 GUI의 승인/제어 동작은 변경하지 않았다.

## 범위의 한계

- Lane/Path/Motion 노드는 기동·Zenoh 사용 확인이다. 차선 인식과 실제 경로 추종의
  정확도, 특정 주행 상황의 전체 자율주행 결과는 판정하지 않았다.
- ModeDialog의 Zenoh 재적용과 새 GUI 프로세스에서 재수신을 각각 확인했다.
  `main()`의 자동 exec 재시작과 이전 노드 목록 자동 복원 전체를 직접 실행한 검사는 아니다.
- CANoe로 확인한 CycloneDDS 제어 전달은 사용자의 검증 결과다. 이번 Zenoh 검사는
  정지 CAN 프레임까지 확인했으며 CANoe/실제 주행 검사를 대체하지 않는다.
- 지연, 운용 한도, 장시간 부하/성능 실험, Bridge/Pico 구현은 이번 범위에 포함하지 않았다.
  저장 상태의 `data_path_verified`는 기존 정책대로 false를 유지한다.

회귀 검증: launch_pkg, autosdv_gui_pkg(게임패드 테스트 포함), network_qos의
pytest **84개 통과**. 최종 5개 장치의 Zenoh verify를 다시 실행해 모두 통과했다.

[측정·프로세스·모드 적용·CAN 로그](validation/zenoh-gui-2026-09-09.json).
