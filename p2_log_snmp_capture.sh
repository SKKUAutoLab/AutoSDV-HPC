#!/usr/bin/env bash
set -euo pipefail

NODE="${NODE:-/ethernet_image_publisher_node_1}"
OUT_ROOT="${OUT_ROOT:-/home/autolab/update/P2_log}"
RUN_ID="${RUN_ID:-}"
LOG_PATH="${LOG_PATH:-}"
STATE_DIR="${STATE_DIR:-${OUT_ROOT}/.p2_log_snmp_state}"
BE_STATS_PATH="${BE_STATS_PATH:-${AUTOSDV_BE_STATS_PATH:-/tmp/autosdv_be_rx_stats.csv}}"
ETH_STATS_IFACE="${ETH_STATS_IFACE:-}"
STATE_FILE=""
OUT_DIR=""
DURATION=""
RUN_ACTIVE=0

usage() {
  cat <<'EOF'
Usage: p2_log_snmp_capture.sh start|stop|run [options]

Wraps P2 ROS parameter toggling with /proc/net/snmp snapshots.

Commands:
  start   Save proc_net_snmp.before.txt, set p2_log_path, then enable P2 log.
  stop    Save proc_net_snmp.after.txt, then disable P2 log.
  run     start, sleep for --duration, then stop.

Options:
  --node NODE          ROS 2 node name (default: /ethernet_image_publisher_node_1)
  --out-root DIR      Root directory for generated run directories
                      (default: /home/autolab/update/P2_log)
  --out DIR           Existing/specific run directory
  --run-id ID         Run directory name under --out-root
                      (default: p2_YYMMDD_HHMMSS)
  --log-path FILE     P2 CSV path passed to p2_log_path
                      (default: OUT_DIR/hpc_image_receive_p2.csv)
  --be-stats-path FILE
                      BE subscriber snapshot CSV to capture before/after
                      (default: AUTOSDV_BE_STATS_PATH or /tmp/autosdv_be_rx_stats.csv)
  --no-be-stats       Do not capture BE subscriber snapshot CSV
  --eth-stats-iface IFACE
                      Optional interface for ethtool -S before/after capture
                      (default: disabled)
  --no-eth-stats      Do not capture ethtool -S before/after
  --duration SEC      Required for run command. Accepts sleep(1) durations
                      such as 300, 300s, 5m.
  -h, --help          Show this help

Environment variables with the same names as the defaults are also accepted:
NODE, OUT_ROOT, RUN_ID, LOG_PATH, STATE_DIR, BE_STATS_PATH, ETH_STATS_IFACE.
EOF
}

node_state_file() {
  local safe_node
  safe_node="$(printf '%s' "${NODE}" | tr '/: ' '___')"
  printf '%s/%s.state' "${STATE_DIR}" "${safe_node}"
}

require_ros2() {
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 command not found. Source the ROS 2 and workspace setup first." >&2
    exit 1
  fi
}

validate_duration() {
  if [[ -z "${DURATION}" ]]; then
    echo "--duration SEC is required for run." >&2
    exit 2
  fi

  if [[ ! "${DURATION}" =~ ^[0-9]+([.][0-9]+)?([smhd])?$ ]]; then
    echo "Invalid --duration: ${DURATION} (use e.g. 300, 300s, 5m)" >&2
    exit 2
  fi

  if [[ "${DURATION}" =~ ^0+([.]0+)?([smhd])?$ ]]; then
    echo "--duration must be greater than zero." >&2
    exit 2
  fi
}

make_run_paths() {
  if [[ -z "${OUT_DIR}" ]]; then
    if [[ -z "${RUN_ID}" ]]; then
      RUN_ID="p2_$(date +%y%m%d_%H%M%S)"
    fi
    OUT_DIR="${OUT_ROOT}/${RUN_ID}"
  fi

  if [[ -z "${LOG_PATH}" ]]; then
    LOG_PATH="${OUT_DIR}/hpc_image_receive_p2.csv"
  fi

  mkdir -p "${OUT_DIR}" "${STATE_DIR}"
  STATE_FILE="$(node_state_file)"
}

capture_snmp() {
  local phase="$1"
  local target="${OUT_DIR}/proc_net_snmp.${phase}.txt"

  if [[ ! -r /proc/net/snmp ]]; then
    echo "Cannot read /proc/net/snmp" >&2
    exit 1
  fi

  cat /proc/net/snmp > "${target}"
  date --iso-8601=ns > "${OUT_DIR}/proc_net_snmp.${phase}.timestamp.txt"
  echo "${target}"
}

capture_be_stats() {
  local phase="$1"
  local target="${OUT_DIR}/be_rx_stats.${phase}.csv"

  if [[ -z "${BE_STATS_PATH}" ]]; then
    return 0
  fi

  if [[ ! -r "${BE_STATS_PATH}" ]]; then
    echo "BE stats snapshot not readable: ${BE_STATS_PATH}" >&2
    return 0
  fi

  cp "${BE_STATS_PATH}" "${target}"
  date --iso-8601=ns > "${OUT_DIR}/be_rx_stats.${phase}.timestamp.txt"
  echo "${target}"
}

capture_eth_stats() {
  local phase="$1"
  local target="${OUT_DIR}/ethtool_stats.${phase}.txt"
  local error_target="${OUT_DIR}/ethtool_stats.${phase}.error.txt"

  if [[ -z "${ETH_STATS_IFACE}" ]]; then
    return 0
  fi

  if ! command -v ethtool >/dev/null 2>&1; then
    echo "ethtool command not found; skipping ${ETH_STATS_IFACE} stats." >&2
    return 0
  fi

  if ! ethtool -S "${ETH_STATS_IFACE}" > "${target}" 2> "${error_target}"; then
    echo "Failed to capture ethtool stats for ${ETH_STATS_IFACE}: ${error_target}" >&2
    rm -f "${target}"
    return 0
  fi

  rm -f "${error_target}"
  date --iso-8601=ns > "${OUT_DIR}/ethtool_stats.${phase}.timestamp.txt"
  echo "${target}"
}

write_be_stats_delta() {
  local before="${OUT_DIR}/be_rx_stats.before.csv"
  local after="${OUT_DIR}/be_rx_stats.after.csv"
  local target="${OUT_DIR}/be_rx_stats.delta.csv"

  if [[ ! -r "${before}" || ! -r "${after}" ]]; then
    return 0
  fi

  awk -F, '
    BEGIN {
      OFS=",";
      print "topic","received_delta","sequence_gap_delta","invalid_magic_delta","received_before","received_after","sequence_gap_before","sequence_gap_after","invalid_magic_before","invalid_magic_after";
    }
    NR == FNR {
      if (FNR > 1) {
        recv[$1] = $2;
        gap[$1] = $3;
        bad[$1] = $4;
      }
      next;
    }
    FNR > 1 {
      topic = $1;
      before_recv = (topic in recv) ? recv[topic] : 0;
      before_gap = (topic in gap) ? gap[topic] : 0;
      before_bad = (topic in bad) ? bad[topic] : 0;
      print topic, $2 - before_recv, $3 - before_gap, $4 - before_bad, before_recv, $2, before_gap, $3, before_bad, $4;
    }
  ' "${before}" "${after}" > "${target}"
}

write_metadata_start() {
  {
    echo "node=${NODE}"
    echo "out_dir=${OUT_DIR}"
    echo "log_path=${LOG_PATH}"
    echo "be_stats_path=${BE_STATS_PATH}"
    echo "eth_stats_iface=${ETH_STATS_IFACE}"
    if [[ -n "${DURATION}" ]]; then
      echo "requested_duration=${DURATION}"
    fi
    echo "start_time=$(date --iso-8601=ns)"
  } > "${OUT_DIR}/p2_log_snmp.metadata"
}

write_metadata_stop() {
  echo "stop_time=$(date --iso-8601=ns)" >> "${OUT_DIR}/p2_log_snmp.metadata"
}

write_state() {
  {
    printf 'NODE=%q\n' "${NODE}"
    printf 'OUT_ROOT=%q\n' "${OUT_ROOT}"
    printf 'OUT_DIR=%q\n' "${OUT_DIR}"
    printf 'LOG_PATH=%q\n' "${LOG_PATH}"
    printf 'BE_STATS_PATH=%q\n' "${BE_STATS_PATH}"
    printf 'ETH_STATS_IFACE=%q\n' "${ETH_STATS_IFACE}"
  } > "${STATE_FILE}"
}

read_state() {
  STATE_FILE="$(node_state_file)"
  if [[ ! -f "${STATE_FILE}" ]]; then
    echo "No active P2/SNMP state for node ${NODE}: ${STATE_FILE}" >&2
    echo "Pass --out if this stop should use a specific run directory." >&2
    exit 2
  fi
  # shellcheck disable=SC1090
  source "${STATE_FILE}"
}

cmd_start() {
  require_ros2
  make_run_paths

  capture_snmp before >/dev/null
  capture_be_stats before >/dev/null
  capture_eth_stats before >/dev/null

  ros2 param set "${NODE}" p2_log_path "${LOG_PATH}"
  ros2 param set "${NODE}" p2_log_enabled true
  write_metadata_start

  write_state
  echo "P2 log enabled on ${NODE}"
  echo "P2 CSV: ${LOG_PATH}"
  echo "SNMP before: ${OUT_DIR}/proc_net_snmp.before.txt"
  if [[ -r "${OUT_DIR}/be_rx_stats.before.csv" ]]; then
    echo "BE stats before: ${OUT_DIR}/be_rx_stats.before.csv"
  fi
  if [[ -r "${OUT_DIR}/ethtool_stats.before.txt" ]]; then
    echo "ethtool stats before: ${OUT_DIR}/ethtool_stats.before.txt"
  fi
}

cmd_stop() {
  require_ros2

  if [[ -n "${OUT_DIR}" ]]; then
    mkdir -p "${OUT_DIR}" "${STATE_DIR}"
    STATE_FILE="$(node_state_file)"
  else
    read_state
  fi

  ros2 param set "${NODE}" p2_log_enabled false
  write_metadata_stop
  capture_snmp after >/dev/null
  capture_be_stats after >/dev/null
  capture_eth_stats after >/dev/null
  write_be_stats_delta
  rm -f "${STATE_FILE}"

  echo "P2 log disabled on ${NODE}"
  echo "SNMP after: ${OUT_DIR}/proc_net_snmp.after.txt"
  if [[ -r "${OUT_DIR}/be_rx_stats.after.csv" ]]; then
    echo "BE stats after: ${OUT_DIR}/be_rx_stats.after.csv"
  fi
  if [[ -r "${OUT_DIR}/be_rx_stats.delta.csv" ]]; then
    echo "BE stats delta: ${OUT_DIR}/be_rx_stats.delta.csv"
  fi
  if [[ -r "${OUT_DIR}/ethtool_stats.after.txt" ]]; then
    echo "ethtool stats after: ${OUT_DIR}/ethtool_stats.after.txt"
  fi
}

cmd_run() {
  validate_duration

  cmd_start
  RUN_ACTIVE=1
  trap 'status=$?; trap - EXIT INT TERM; if [[ "${RUN_ACTIVE}" == "1" ]]; then echo "Stopping P2 log after interrupted run..." >&2; cmd_stop || true; fi; exit "${status}"' EXIT INT TERM

  sleep "${DURATION}"
  RUN_ACTIVE=0
  trap - EXIT INT TERM
  cmd_stop
}

if [[ $# -lt 1 ]]; then
  usage >&2
  exit 2
fi

COMMAND="$1"
shift

while [[ $# -gt 0 ]]; do
  case "$1" in
    --node)
      NODE="$2"
      shift 2
      ;;
    --out-root)
      OUT_ROOT="$2"
      shift 2
      ;;
    --out)
      OUT_DIR="$2"
      shift 2
      ;;
    --run-id)
      RUN_ID="$2"
      shift 2
      ;;
    --log-path)
      LOG_PATH="$2"
      shift 2
      ;;
    --be-stats-path)
      BE_STATS_PATH="$2"
      shift 2
      ;;
    --no-be-stats)
      BE_STATS_PATH=""
      shift
      ;;
    --eth-stats-iface)
      ETH_STATS_IFACE="$2"
      shift 2
      ;;
    --no-eth-stats)
      ETH_STATS_IFACE=""
      shift
      ;;
    --duration)
      DURATION="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${COMMAND}" in
  start)
    cmd_start
    ;;
  stop)
    cmd_stop
    ;;
  run)
    cmd_run
    ;;
  -h|--help)
    usage
    ;;
  *)
    echo "Unknown command: ${COMMAND}" >&2
    usage >&2
    exit 2
    ;;
esac
