#!/usr/bin/env bash
# setup_tracing_overlay.sh — ROS2 Humble의 핵심 패키지를 tracepoint 켠 상태로 overlay 재빌드.
#
# 배경:
#   Ubuntu의 ros-humble-* apt 패키지는 LTTng tracepoint가 컴파일 타임에 제거된 채로 배포되어,
#   `ros2 run tracetools status`가 "Tracing disabled"로 나옴. 기존 apt 설치는 그대로 두고,
#   별도 overlay workspace에 tracepoint enabled로 핵심 패키지만 재빌드한다.
#
# 일상 운영:
#   overlay를 source하지 않으면 apt 환경 그대로 → 영향 0.
#   측정 시에만 `source ~/ros2_tracing_overlay/install/setup.bash` 후 진행.
#
# 소요:
#   빌드 시간 30~50분, 디스크 ~5 GB.
#
# 사용:
#   bash Tools/tracing/setup_tracing_overlay.sh
#   OVERLAY_DIR=~/my_overlay bash Tools/tracing/setup_tracing_overlay.sh

set -euo pipefail

# bash 가드 — zsh로 실행되면 setup.bash가 BASH_SOURCE 등에서 깨짐
if [ -z "${BASH_VERSION:-}" ]; then
    echo "[ERROR] 이 스크립트는 bash로 실행되어야 합니다." >&2
    echo "        실행: bash Tools/tracing/setup_tracing_overlay.sh" >&2
    echo "        (zsh로 직접 실행하지 마세요. shebang은 #!/usr/bin/env bash)" >&2
    exit 1
fi

OVERLAY_DIR="${OVERLAY_DIR:-${HOME}/ros2_tracing_overlay}"
ROS_DISTRO="humble"

echo "=========================================="
echo " ROS2 tracing overlay setup"
echo "=========================================="
echo " Overlay dir:  ${OVERLAY_DIR}"
echo " ROS distro:   ${ROS_DISTRO}"
echo ""

# ── 사전 점검 ──────────────────────────────────────────────
if [ ! -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
    echo "[ERROR] /opt/ros/${ROS_DISTRO}/setup.bash not found. ROS2 Humble 설치 필요." >&2
    exit 1
fi

for pkg in liblttng-ust-dev lttng-tools python3-vcstool python3-colcon-common-extensions; do
    if ! dpkg -s "${pkg}" >/dev/null 2>&1; then
        echo "[ERROR] ${pkg} not installed. 'sudo apt install ${pkg}'" >&2
        exit 1
    fi
done

# ── workspace 생성 ────────────────────────────────────────
mkdir -p "${OVERLAY_DIR}/src"
cd "${OVERLAY_DIR}"

# ── 패키지 목록 (tracepoint를 emit하는 핵심) ───────────────
cat > tracing.repos << 'EOF'
repositories:
  ros2/ros2_tracing:
    type: git
    url: https://github.com/ros2/ros2_tracing.git
    version: humble
  ros2/rcl:
    type: git
    url: https://github.com/ros2/rcl.git
    version: humble
  ros2/rclcpp:
    type: git
    url: https://github.com/ros2/rclcpp.git
    version: humble
  ros2/rclpy:
    type: git
    url: https://github.com/ros2/rclpy.git
    version: humble
  ros2/rmw_fastrtps:
    type: git
    url: https://github.com/ros2/rmw_fastrtps.git
    version: humble
EOF

# ── clone ─────────────────────────────────────────────────
if [ -d src/ros2 ] && [ "$(ls -A src/ros2 2>/dev/null)" ]; then
    echo "[INFO] src/ 이미 존재. 기존 clone 유지."
else
    echo "[INFO] vcs import 중..."
    vcs import src < tracing.repos
fi

# ── 빌드 ──────────────────────────────────────────────────
echo ""
echo "[INFO] colcon build 시작 (30~50분 소요)..."
echo ""
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# --allow-overriding: overlay가 apt(underlay) 패키지를 의도적으로 대체 (tracepoint 활성화 빌드)
# -DBUILD_TESTING=OFF: rcl_yaml_param_parser 등의 test 의존성(osrf_testing_tools_cpp) 회피 — production 빌드
# -DTRACETOOLS_DISABLED=OFF: 명시적으로 tracepoint 컴파일
colcon build --symlink-install \
    --allow-overriding rcl rcl_action rcl_lifecycle rcl_yaml_param_parser \
                       rclcpp rclcpp_action rclcpp_components rclcpp_lifecycle \
                       rclpy \
                       rmw_fastrtps_cpp rmw_fastrtps_dynamic_cpp rmw_fastrtps_shared_cpp \
                       tracetools \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
                 -DBUILD_TESTING=OFF \
                 -DTRACETOOLS_DISABLED=OFF

# ── 검증 ──────────────────────────────────────────────────
echo ""
echo "=========================================="
echo " 빌드 완료. 검증 중..."
echo "=========================================="
# shellcheck disable=SC1091
source "${OVERLAY_DIR}/install/setup.bash"

echo ""
echo "[Check 1] tracetools status:"
ros2 run tracetools status

echo ""
echo "[Check 2] librclcpp.so → lttng-ust link 확인:"
if ldd "${OVERLAY_DIR}/install/rclcpp/lib/librclcpp.so" 2>/dev/null | grep -q lttng; then
    ldd "${OVERLAY_DIR}/install/rclcpp/lib/librclcpp.so" | grep lttng
    echo "  ✓ rclcpp가 lttng-ust에 link됨"
else
    echo "  ✗ rclcpp가 여전히 lttng-ust에 link 안 됨 — 빌드 실패 가능성"
    exit 2
fi

echo ""
echo "=========================================="
echo " Overlay 준비 완료"
echo "=========================================="
echo " 측정 시 매번 source 해야 함:"
echo "   source ${OVERLAY_DIR}/install/setup.bash"
echo ""
echo " AutoSDV workspace도 overlay 위에서 재빌드 필요:"
echo "   cd ~/Projects/AutoSDV/Laptop_HPC"
echo "   source ${OVERLAY_DIR}/install/setup.bash"
echo "   colcon build --packages-select camera_perception_pkg decision_making_pkg launch_pkg"
echo ""
echo " 이후 측정:"
echo "   source install/setup.bash"
echo "   NUM_RUNS=1 bash Tools/tracing/run_trace_session.sh S1"
echo "=========================================="
