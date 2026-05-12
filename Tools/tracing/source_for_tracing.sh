# source_for_tracing.sh — 측정용 ROS2 환경 source helper (bash/zsh 양쪽 동작)
#
# 이 파일은 *source*해야 합니다 (실행 X). 한 줄로 apt → overlay → AutoSDV 순으로 source.
#
# 사용:
#   source Tools/tracing/source_for_tracing.sh
#
# 또는 .zshrc/.bashrc에 alias 등록:
#   alias autosdv-trace='source ~/update/AutoSDV-HPC/Tools/tracing/source_for_tracing.sh'

# ─── sourced인지 확인 ─────────────────────────────────────
# bash: BASH_SOURCE[0] != $0 면 sourced
# zsh: ZSH_EVAL_CONTEXT에 "file" 포함되면 sourced
_is_sourced=0
if [ -n "${BASH_VERSION:-}" ]; then
    [ "${BASH_SOURCE[0]}" != "${0}" ] && _is_sourced=1
elif [ -n "${ZSH_VERSION:-}" ]; then
    case "${ZSH_EVAL_CONTEXT:-}" in *:file*) _is_sourced=1 ;; esac
fi

if [ "${_is_sourced}" -ne 1 ]; then
    echo "[ERROR] 이 스크립트는 source 해야 합니다 (실행 X):" >&2
    echo "        source Tools/tracing/source_for_tracing.sh" >&2
    return 1 2>/dev/null || exit 1
fi
unset _is_sourced

# ─── 셸별 setup 파일 확장자 선택 ──────────────────────────
if [ -n "${ZSH_VERSION:-}" ]; then
    _ext="zsh"
elif [ -n "${BASH_VERSION:-}" ]; then
    _ext="bash"
else
    echo "[ERROR] bash 또는 zsh 만 지원" >&2
    return 1
fi

# ─── ROS2 setup은 nounset-safe하지 않으므로 -u 잠시 해제 ───
_prev_opts="$(set +o | tr '\n' ';')"
set +u 2>/dev/null

# ─── 1. apt underlay ─────────────────────────────────────
ROS_DISTRO="${ROS_DISTRO:-humble}"
if [ -f "/opt/ros/${ROS_DISTRO}/setup.${_ext}" ]; then
    # shellcheck disable=SC1090
    . "/opt/ros/${ROS_DISTRO}/setup.${_ext}"
else
    echo "[ERROR] /opt/ros/${ROS_DISTRO}/setup.${_ext} 없음" >&2
    eval "${_prev_opts}"
    return 1
fi

# ─── 2. tracing overlay (★ 핵심) ──────────────────────────
OVERLAY_DIR="${OVERLAY_DIR:-${HOME}/ros2_tracing_overlay}"
if [ -f "${OVERLAY_DIR}/install/setup.${_ext}" ]; then
    # shellcheck disable=SC1090
    . "${OVERLAY_DIR}/install/setup.${_ext}"
else
    echo "[WARN] ${OVERLAY_DIR}/install/setup.${_ext} 없음 — overlay 건너뜀" >&2
    echo "       먼저 'bash Tools/tracing/setup_tracing_overlay.sh' 실행." >&2
fi

# ─── 3. AutoSDV workspace (있으면) ────────────────────────
for _candidate in \
    "${AUTOSDV_DIR:-}" \
    "${HOME}/update/AutoSDV-HPC" \
    "${HOME}/Projects/AutoSDV/Laptop_HPC" \
    "${PWD}"
do
    [ -z "${_candidate}" ] && continue
    if [ -f "${_candidate}/install/setup.${_ext}" ]; then
        # shellcheck disable=SC1090
        . "${_candidate}/install/setup.${_ext}"
        echo "[OK] AutoSDV workspace sourced: ${_candidate}"
        break
    fi
done
unset _candidate

# ─── 원래 shell 옵션 복원 ─────────────────────────────────
eval "${_prev_opts}" 2>/dev/null
unset _prev_opts _ext

# ─── 결과 요약 ────────────────────────────────────────────
echo ""
echo "── 측정 환경 점검 ──"
echo "  tracetools prefix : $(ros2 pkg prefix tracetools 2>/dev/null || echo '???')"
echo "  rclcpp prefix     : $(ros2 pkg prefix rclcpp 2>/dev/null || echo '???')"
_status="$(ros2 run tracetools status 2>&1 | head -1)"
echo "  tracetools status : ${_status}"
if echo "${_status}" | grep -q "Tracing enabled"; then
    echo "  → OK, 측정 가능"
else
    echo "  ✗ Tracing 미활성. overlay source 확인 필요."
fi
unset _status
