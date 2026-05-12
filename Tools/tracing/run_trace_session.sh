#!/usr/bin/env bash
# AutoSDV HPC ROS2 tracing — 4-config × 3 runs 자동 실행 스크립트
#
# 동작:
#   각 config(S1..S4)마다 3회씩 ros2 launch main_traced.launch.py 를 5분간 실행,
#   완료 후 SIGINT로 종료 → LTTng 세션 플러시 → /tmp/autosdv_traces/<session>/ 에 저장.
#
# 사전 준비 (HPC에서):
#   sudo apt install -y \
#     ros-humble-ros2trace ros-humble-tracetools ros-humble-tracetools-launch \
#     ros-humble-tracetools-analysis lttng-tools liblttng-ust-dev \
#     python3-babeltrace python3-bt2
#   ros2 run tracetools status   # "Tracing enabled" 확인
#   cd ~/Projects/AutoSDV/Laptop_HPC && colcon build --packages-select \
#     camera_perception_pkg decision_making_pkg launch_pkg
#
# 사용:
#   cd ~/Projects/AutoSDV/Laptop_HPC
#   source install/setup.bash
#   bash Tools/tracing/run_trace_session.sh             # S1~S4 × 3회
#   bash Tools/tracing/run_trace_session.sh S2          # S2만 3회
#   RUN_DURATION_SEC=300 bash Tools/tracing/run_trace_session.sh  # 명시적 길이
#
# 각 config 사이에는 TSN/CBS 설정을 사용자가 수동으로 바꿔야 함 (자동화 범위 밖).
# 스크립트가 config 시작 전에 "config X 준비 끝났으면 Enter" 프롬프트로 멈춤.

set -euo pipefail

# ── 설정 ──────────────────────────────────────────────────
RUN_DURATION_SEC="${RUN_DURATION_SEC:-300}"   # 5분 (plan §5)
NUM_RUNS="${NUM_RUNS:-3}"                      # config당 3회 (plan §5)
TRACE_BASE="${AUTOSDV_TRACE_BASE:-/tmp/autosdv_traces}"
LAUNCH_PKG="launch_pkg"
LAUNCH_FILE="main_traced.launch.py"

# config 식별자 → 사람에게 보여줄 설명
declare -A CONFIG_DESC=(
    [S1]="BE baseline (TSN off, 5-cam 30fps)"
    [S2]="TAS only (TSN on, 5-cam 30fps)"
    [S3]="TAS + CBS manual (TSN on, 5-cam 30fps)"
    [S4]="TAS + CBS proposed (TSN on, 5-cam 30fps)"
)

# 인자로 특정 config만 받을 수도 있음
if [ $# -gt 0 ]; then
    CONFIGS=("$@")
else
    CONFIGS=(S1 S2 S3 S4)
fi

# ── 사전 점검 ──────────────────────────────────────────────
echo "=========================================="
echo " AutoSDV HPC tracing — run_trace_session"
echo "=========================================="
echo " 측정 길이:    ${RUN_DURATION_SEC}s ($((RUN_DURATION_SEC/60))분) × ${NUM_RUNS}회"
echo " Trace base:   ${TRACE_BASE}"
echo " Configs:      ${CONFIGS[*]}"
echo ""

if ! command -v ros2 >/dev/null 2>&1; then
    echo "[ERROR] ros2 command not found. source /opt/ros/humble/setup.bash 먼저." >&2
    exit 1
fi

if ! command -v lttng >/dev/null 2>&1; then
    echo "[ERROR] lttng command not found. apt install lttng-tools 필요." >&2
    exit 1
fi

# ★ Tracing이 실제로 활성화되어 있는지 검증 (Humble apt 빌드는 기본 disabled)
tracing_status="$(ros2 run tracetools status 2>&1 | head -1 || true)"
if ! echo "${tracing_status}" | grep -q "Tracing enabled"; then
    echo "[ERROR] ros2_tracing이 비활성 상태입니다: '${tracing_status}'" >&2
    echo "        Tools/tracing/setup_tracing_overlay.sh 로 overlay 빌드 후" >&2
    echo "        'source ~/ros2_tracing_overlay/install/setup.bash' 한 다음 다시 실행하세요." >&2
    exit 1
fi
echo "[OK] ros2_tracing enabled"

mkdir -p "${TRACE_BASE}"

# ── 본 루프 ───────────────────────────────────────────────
for cfg in "${CONFIGS[@]}"; do
    desc="${CONFIG_DESC[$cfg]:-unknown config}"
    echo ""
    echo "=========================================="
    echo " Config: ${cfg}  —  ${desc}"
    echo "=========================================="
    echo " 이 config에 맞는 TSN/CBS 설정을 적용하셨습니까?"
    echo " (예: S1=BE면 taprio/cbs off, S2~S4면 적절히 on)"
    read -r -p " 준비됐으면 Enter, 건너뛰려면 's' Enter: " ans
    if [ "${ans}" = "s" ]; then
        echo " → ${cfg} skip"
        continue
    fi

    for run in $(seq 1 "${NUM_RUNS}"); do
        session="autosdv_hpc_${cfg}_run$(printf '%02d' "${run}")"
        echo ""
        echo "── Run ${run}/${NUM_RUNS}  session=${session} ──"
        echo "   $(date '+%F %T') 시작, ${RUN_DURATION_SEC}s 후 자동 SIGINT"

        # 백그라운드로 launch 시작
        ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" \
            session_name:="${session}" \
            trace_base_path:="${TRACE_BASE}" \
            > "${TRACE_BASE}/${session}.stdout.log" \
            2> "${TRACE_BASE}/${session}.stderr.log" &
        LAUNCH_PID=$!

        # 정확히 RUN_DURATION_SEC 만큼 sleep 후 SIGINT
        sleep "${RUN_DURATION_SEC}"

        echo "   $(date '+%F %T') SIGINT → launch_pid=${LAUNCH_PID}"
        kill -INT "${LAUNCH_PID}" 2>/dev/null || true

        # graceful exit 최대 30초 대기 — motion_planner shutdown handler가 0.5s 정도 잡음
        for _ in $(seq 30); do
            kill -0 "${LAUNCH_PID}" 2>/dev/null || break
            sleep 1
        done

        # 아직 살아있으면 SIGTERM 후 5초, 그래도 안 죽으면 SIGKILL
        if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
            echo "   [WARN] launch가 SIGINT에 안 죽음 → SIGTERM"
            kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
            sleep 5
        fi
        if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
            echo "   [WARN] SIGTERM도 안 통함 → SIGKILL"
            kill -9 "${LAUNCH_PID}" 2>/dev/null || true
            sleep 2
        fi

        # LTTng 세션이 자동 stop 안 됐을 수 있으니 강제 stop + destroy (★ flush 보장)
        if lttng list 2>/dev/null | grep -q "${session}"; then
            echo "   [INFO] LTTng session 수동 stop/destroy"
            lttng stop "${session}" 2>/dev/null || true
            lttng destroy "${session}" 2>/dev/null || true
        fi

        wait "${LAUNCH_PID}" 2>/dev/null || true
        sleep 2

        # 결과 요약
        if [ -d "${TRACE_BASE}/${session}/ust" ]; then
            sz=$(du -sh "${TRACE_BASE}/${session}" | cut -f1)
            csv_cnt=$(find "${TRACE_BASE}/${session}/csv" -name "*.csv" 2>/dev/null | wc -l)
            echo "   ✓ saved: ${TRACE_BASE}/${session} (${sz}, csv files=${csv_cnt})"
        else
            echo "   ✗ FAIL: ${TRACE_BASE}/${session}/ust not found — see stderr.log"
        fi

        # config 내부 run 사이 짧은 휴식 (DDS discovery 안정화)
        sleep 5
    done
done

echo ""
echo "=========================================="
echo " 전체 완료. 분석은 Tools/tracing/analyze_traces.ipynb 참고."
echo "=========================================="
