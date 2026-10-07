# 통신 설정

- `dds/`: Fast DDS·Cyclone DDS의 XML 프로파일
- `zenoh/`: Zenoh 세션·라우터·DDS 브리지의 JSON5 설정

카메라별 프로파일은 `src/launch_pkg/config/`에서 관리합니다.
`launch_pkg`를 빌드하면 공통 설정도 기존과 동일하게
`install/launch_pkg/share/launch_pkg/config/`에 설치됩니다.

설정 경로를 변경한 뒤에는 `launch_pkg`를 다시 빌드하세요.
통신 모드 관리 도구를 설치한 환경에서는 다음 명령으로 경로 변경을 반영합니다.

```bash
sudo python3 scripts/install_communication_mode.py hpc --workspace "$PWD"
```
