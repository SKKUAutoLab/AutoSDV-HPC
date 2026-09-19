# Xbox 레이싱 게임 조작 조사

조사일: 2026-09-08. 범위: 공식 기본 조작표와 현재 AutoSDV 구현 비교.
이번 작업에서는 주행 코드를 변경하지 않았다.

## RB를 누르게 구현했던 이유

AutoSDV의 RB는 운전자가 버튼을 놓으면 비상정지를 걸기 위해 구현자가
추가한 주행 허용 조건이다. 사용자 요구나 레이싱 게임 조작표에서 가져온
배치가 아니다. 현재 `GamepadGate`는 RB 해제를 E-Stop으로 처리한다.

## 공식 자료에서 확인한 배치

| 게임 / 프리셋 | RT | LT | 왼쪽 스틱 | RB | 주요 기타 버튼 |
| --- | --- | --- | --- | --- | --- |
| Forza Motorsport 5 Xbox One | 가속 | 브레이크 | 조향 | 뒤 보기 | A 핸드브레이크, B 기어 올림, X 기어 내림, LB 클러치 |
| F1 25 Xbox 조작표 | 가속 | 브레이크/후진 | 좌우 조향 | 무전/음성 명령 | A 기어 올림·클러치, X 기어 내림, B MFD |
| Need for Speed Heat 자동 변속 기본값 | 가속 | 브레이크 | 조향 | 상호작용 | A 니트로, X 핸드브레이크, Y 카메라 |
| Need for Speed Heat 수동 변속 기본값 | 가속 | 브레이크 | 조향 | 기어 올림·상호작용 | LB 기어 내림, A 니트로, X 핸드브레이크 |

출처:

- [Xbox Live Forza Motorsport 5 공식 매뉴얼](https://dlassets-ssl.xboxlive.com/public/content/f952d7d3-cd26-47e6-aaf3-8114c0b02eb3/GameManual/11ae5f04-90ba-4198-a32c-785da367fb5e/en-IE/index.html#12): Controls 절의 HTML 버튼 아이콘 클래스와 기능을 대조했다. FM5에 한정한 결과이며 최신 Motorsport/Horizon의 전체 버튼 배치가 같다고 확장하지 않는다. 이 문서는 LT를 Brake로만 표기한다.
- [EA F1 25 Xbox Driving Controls](https://www.ea.com/es/games/f1/f1-25/f1-25-controls-hub/driving-controls-controller-xbox): 공식 페이지의 검색 색인에서 조작표 확인. 직접 열면 본문이 누락되는 경우가 있어, 현재 게임 실행 화면을 직접 확인한 결과와는 구분한다.
- [EA Need for Speed Heat Xbox One Text Manual](https://www.ea.com/able/resources/need-for-speed/need-for-speed-heat/xbox-one/text-manual): CONTROLS 절이 자동/수동 변속의 기본 프리셋을 명시한다. LT는 이 문서에 Brake로 표기되므로 후진 전환 조건까지 추정하지 않았다.

이 조작표들에는 RB를 계속 눌러야 일반 주행이 허용된다는 조건이 없다.
이는 조사한 기본 프리셋에 대한 결론이며 모든 게임·사용자 설정에 대한
보편적 단정은 아니다. 게임의 핸드브레이크는 AutoSDV의 정지 상태가
유지되는 E-Stop과 다른 기능이다.

## 버튼 배치 외의 차이

[EA F1 25 접근성 설명](https://www.ea.com/able/resources/f1-25)은 조향,
스로틀, 브레이크 각각의 데드존과 감도 보정을 제공한다고 명시한다.
[Forza Motorsport 7 컨트롤러 설정](https://support.forzamotorsport.net/hc/en-us/articles/360005421793-FM7-Controller-Settings)도
스틱·트리거의 이동 범위에서 0/최대 입력을 판정하는 데드존을 설명한다.
따라서 아날로그 입력량을 살리는 것이 게임 같은 조작감을 위한 핵심이다.
게임별 조향 보정·속도 의존성·후진 진입 규칙은 동일하다고 가정하지 않는다.

현재 저장소 코드에서 확인한 동작:

- `src/autosdv_gui_pkg/autosdv_gui_pkg/gamepad.py`: 트리거 10%, 스틱 20%를 넘으면 입력을 bool로 바꾼다. 누른 정도를 주행 노드에 전달하지 않는다.
- `src/interfaces_pkg/msg/DriveInput.msg`: 전진·후진·좌우 입력이 bool이다.
- `src/data_collection_pkg/data_collection_pkg/drive_node.py`: 방향 입력에 따라 속도와 조향을 일정한 비율로 누적한다. 스틱을 중립으로 돌려도 기존 조향값은 유지된다. 후진 입력은 속도를 음수 방향으로 누적하므로 별도의 브레이크/후진 전환 상태는 없다.

## AutoSDV에 적용할 제안 — 아직 미구현

1. 일반 주행에서 RB 유지 조건을 없앤다. GUI Manual Drive 시작과 중립 입력 확인으로 진입한다.
2. RT 입력량을 가속 명령, LT 입력량을 제동 명령으로 사용한다. 후진은 정지 후 명시적인 전환 조건을 별도로 정의한다.
3. 왼쪽 스틱 위치를 목표 조향각으로 사용하고, 중립이면 목표를 중앙으로 복귀시킨다. 실제 조향 명령에는 기존 한계값과 적절한 변화율 제한을 적용한다.
4. 비상정지 버튼과 연결 해제 시 정지를 유지한다. 비상정지의 버튼 선택은 AutoSDV용 설계로 명시하고 특정 게임의 핸드브레이크와 동일시하지 않는다.

RB 조건 삭제만으로는 2·3의 조작감이 구현되지 않는다. 이를 적용하려면
아날로그 값을 전달할 메시지와 DriveController의 입력 처리도 함께 바꿔야 한다.
