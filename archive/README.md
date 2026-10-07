# 아카이브

`bev_perception_pkg/`에는 `bev_node`와 `svm_compositor_node`의 소스,
보정 설정, launch 파일을 원형 그대로 보관합니다.

이 디렉토리의 `COLCON_IGNORE`는 아카이브 전체를 기본 colcon 빌드에서 제외합니다.
패키지를 복원하려면 워크스페이스 루트에서 다음 명령을 실행하세요.

```bash
mv archive/bev_perception_pkg src/
colcon build --packages-select bev_perception_pkg
source install/local_setup.bash
ros2 launch bev_perception_pkg bev.launch.py
```
