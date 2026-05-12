#!/usr/bin/env bash
# package_for_analysis.sh — 측정 결과를 분석 가능한 형태(parquet/csv)로 가공해서 tar.gz로 묶음.
#
# 이유:
#   LTTng CTF binary는 babeltrace2 필요 → dev PC에 없으면 못 읽음.
#   HPC에서 frame_id_join.py(bt2 사용)로 미리 parquet 산출 → 그것만 보내면 dev PC에서 분석 가능.
#
# 사용:
#   bash Tools/tracing/package_for_analysis.sh \
#       autosdv_hpc_S1_run01 autosdv_hpc_S1_run02 autosdv_hpc_S1_run03
#
# 또는 인자 없이 (S1~S4의 run01..03 자동 탐색):
#   bash Tools/tracing/package_for_analysis.sh

set -euo pipefail

TRACE_BASE="${TRACE_BASE:-/tmp/autosdv_traces}"
OUT_DIR="${OUT_DIR:-${PWD}/trace_analysis_$(date +%Y%m%d_%H%M%S)}"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 인자 없으면 자동 탐색
if [ $# -eq 0 ]; then
    mapfile -t SESSIONS < <(
        find "${TRACE_BASE}" -maxdepth 1 -type d -name 'autosdv_hpc_*' \
        | xargs -I {} basename {} | sort
    )
    if [ "${#SESSIONS[@]}" -eq 0 ]; then
        echo "[ERROR] ${TRACE_BASE}에 autosdv_hpc_* 세션 없음" >&2
        exit 1
    fi
else
    SESSIONS=("$@")
fi

echo "=========================================="
echo " AutoSDV trace 패키징"
echo "=========================================="
echo " Trace base:  ${TRACE_BASE}"
echo " 출력 디렉토리: ${OUT_DIR}"
echo " 세션 수:     ${#SESSIONS[@]}"
echo "   ${SESSIONS[*]}"
echo ""

mkdir -p "${OUT_DIR}"

# 전체 manifest
{
    echo "=== Package manifest ==="
    echo "host: $(hostname)"
    echo "date: $(date -Iseconds)"
    echo "kernel: $(uname -r)"
    echo "ros2: $(ros2 --version 2>/dev/null || echo 'unknown')"
    echo "overlay: ${HOME}/ros2_tracing_overlay"
    echo "sessions:"
    printf '  - %s\n' "${SESSIONS[@]}"
} > "${OUT_DIR}/manifest.txt"

# 세션별 가공
for session in "${SESSIONS[@]}"; do
    session_dir="${TRACE_BASE}/${session}"
    if [ ! -d "${session_dir}" ]; then
        echo "[WARN] ${session_dir} 없음 — skip"
        continue
    fi

    echo ""
    echo "── ${session} ──"
    out_sub="${OUT_DIR}/${session}"
    mkdir -p "${out_sub}"

    # 1) CSV 복사 (작음, 사람이 직접 확인 가능)
    if [ -d "${session_dir}/csv" ]; then
        cp -r "${session_dir}/csv" "${out_sub}/"
        csv_lines=$(wc -l "${session_dir}/csv"/*.csv 2>/dev/null | tail -1 | awk '{print $1}')
        echo "   CSV files copied (${csv_lines} total lines)"
    fi

    # 2) frame_id_join.py로 parquet 산출 (★ 핵심)
    if [ -d "${session_dir}/ust" ]; then
        echo "   frame_id_join 실행..."
        if python3 "${SCRIPT_DIR}/frame_id_join.py" "${session_dir}" \
                   --out "${out_sub}/t_hpc.parquet" > "${out_sub}/join.log" 2>&1; then
            echo "   ✓ t_hpc.parquet 생성"
        else
            echo "   ✗ frame_id_join 실패 — join.log 확인"
            tail -5 "${out_sub}/join.log" | sed 's/^/        /'
        fi
    else
        echo "   [WARN] ust/ 없음 — frame_id_join 스킵"
    fi

    # 3) 세션 metadata
    {
        echo "session: ${session}"
        echo "trace_size: $(du -sh "${session_dir}" | cut -f1)"
        echo "ust_size: $(du -sh "${session_dir}/ust" 2>/dev/null | cut -f1 || echo 'N/A')"
        echo "csv_size: $(du -sh "${session_dir}/csv" 2>/dev/null | cut -f1 || echo 'N/A')"
        echo "csv_lines: ${csv_lines:-N/A}"
        echo "csv_files:"
        find "${session_dir}/csv" -name "*.csv" -printf '  - %f (%s bytes)\n' 2>/dev/null
    } > "${out_sub}/manifest.txt"

    # 4) 로그 복사 (디버깅용)
    for log in "${session_dir}.stdout.log" "${session_dir}.stderr.log"; do
        [ -f "${log}" ] && cp "${log}" "${out_sub}/"
    done
done

# tar.gz로 묶기
TAR_OUT="${OUT_DIR}.tar.gz"
tar czf "${TAR_OUT}" -C "$(dirname "${OUT_DIR}")" "$(basename "${OUT_DIR}")"
SIZE=$(du -sh "${TAR_OUT}" | cut -f1)

echo ""
echo "=========================================="
echo " 완료"
echo "=========================================="
echo " 출력 디렉토리: ${OUT_DIR}"
echo " tarball:       ${TAR_OUT} (${SIZE})"
echo ""
echo " ── 다음 단계 ──"
echo " dev PC로 전송:"
echo "   scp ${TAR_OUT} <user>@<dev-pc>:/home/osb/Projects/AutoSDV/"
echo " 또는:"
echo "   git LFS / 클라우드 스토리지 업로드"
echo "   파일 사이즈 ${SIZE} 면 git LFS 적절"
