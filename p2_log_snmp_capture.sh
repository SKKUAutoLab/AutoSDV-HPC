#!/usr/bin/env bash
set -euo pipefail

NODE="${NODE:-/ethernet_image_publisher_node_1}"
OUT_ROOT="${OUT_ROOT:-/home/autolab/update/P2_log}"
RUN_ID="${RUN_ID:-}"
LOG_PATH="${LOG_PATH:-}"
STATE_DIR="${STATE_DIR:-${OUT_ROOT}/.p2_log_snmp_state}"
STATE_FILE=""
OUT_DIR=""
DURATION=""

usage() {
  cat <<'EOF'
Usage: p2_log_snmp_capture.sh start|stop|run [options]

Wraps P2 ROS parameter toggling with /proc/net/snmp snapshots.

Commands:
  start   Save proc_net_snmp.before.txt, set p2_log_path, then enable P2 log.
  stop    Save proc_net_snmp.after.txt, then disable P2 log.
  run     start, sleep for --duration seconds, then stop.

Options:
  --node NODE          ROS 2 node name (default: /ethernet_image_publisher_node_1)
  --out-root DIR      Root directory for generated run directories
                      (default: /home/autolab/update/P2_log)
  --out DIR           Existing/specific run directory
  --run-id ID         Run directory name under --out-root
                      (default: p2_YYMMDD_HHMMSS)
  --log-path FILE     P2 CSV path passed to p2_log_path
                      (default: OUT_DIR/hpc_image_receive_p2.csv)
  --duration SEC      Required for run command
  -h, --help          Show this help

Environment variables with the same names as the defaults are also accepted:
NODE, OUT_ROOT, RUN_ID, LOG_PATH, STATE_DIR.
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

write_metadata_start() {
  {
    echo "node=${NODE}"
    echo "out_dir=${OUT_DIR}"
    echo "log_path=${LOG_PATH}"
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
  write_metadata_start

  ros2 param set "${NODE}" p2_log_path "${LOG_PATH}"
  ros2 param set "${NODE}" p2_log_enabled true

  write_state
  echo "P2 log enabled on ${NODE}"
  echo "P2 CSV: ${LOG_PATH}"
  echo "SNMP before: ${OUT_DIR}/proc_net_snmp.before.txt"
}

cmd_stop() {
  require_ros2

  if [[ -n "${OUT_DIR}" ]]; then
    mkdir -p "${OUT_DIR}" "${STATE_DIR}"
    STATE_FILE="$(node_state_file)"
  else
    read_state
  fi

  capture_snmp after >/dev/null
  ros2 param set "${NODE}" p2_log_enabled false
  write_metadata_stop
  rm -f "${STATE_FILE}"

  echo "P2 log disabled on ${NODE}"
  echo "SNMP after: ${OUT_DIR}/proc_net_snmp.after.txt"
}

cmd_run() {
  if [[ -z "${DURATION}" ]]; then
    echo "--duration SEC is required for run." >&2
    exit 2
  fi

  cmd_start
  sleep "${DURATION}"
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
