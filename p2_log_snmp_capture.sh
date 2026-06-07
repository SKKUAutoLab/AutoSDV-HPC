#!/usr/bin/env bash
set -euo pipefail

NODE="${NODE:-}"
CAMERA_NODES="${CAMERA_NODES:-/ethernet_image_publisher_node_1=image_01_raw /ethernet_image_publisher_node_2=image_02_raw /ethernet_image_publisher_node_3=image_03_raw /ethernet_image_publisher_node_4=image_04_raw /ethernet_image_publisher_node_5=image_05_raw}"
OUT_ROOT="${OUT_ROOT:-/home/autolab/update/P2_log}"
RUN_ID="${RUN_ID:-}"
P2_RUN_START_NS="${P2_RUN_START_NS:-}"
LOG_PATH="${LOG_PATH:-}"
FRAME_LOG_DIR="${FRAME_LOG_DIR:-}"
STATE_DIR="${STATE_DIR:-${OUT_ROOT}/.p2_log_snmp_state}"
ETH_STATS_IFACE="${ETH_STATS_IFACE:-}"
ADAS_LOG="${ADAS_LOG:-auto}"
ADAS_CONTROL_PATH="${ADAS_CONTROL_PATH:-/tmp/autosdv_adas_p2_control.env}"
ADAS_SEQUENCE_LOG_DIR="${ADAS_SEQUENCE_LOG_DIR:-}"
STATE_FILE=""
OUT_DIR=""
DURATION=""
RUN_ACTIVE=0

usage() {
  cat <<'EOF'
Usage: p2_log_snmp_capture.sh start|stop|run [options]

Wraps P2 ROS parameter toggling with /proc/net/snmp snapshots.

Commands:
  start   Save proc_net_snmp.before.txt, configure P2 parameters, then enable P2 log.
  stop    Save proc_net_snmp.after.txt, then disable P2 log.
  run     start, sleep for --duration, then stop.

Options:
  --node NODE          Record only one ROS 2 node. Without this option, all
                      camera bridge nodes in --camera-nodes are recorded.
  --camera-nodes SPEC Space-separated node=topic entries for multi-camera mode
                      (default: ethernet_image_publisher_node_1..5)
  --out-root DIR      Root directory for generated run directories
                      (default: /home/autolab/update/P2_log)
  --out DIR           Existing/specific run directory
  --run-id ID         Run directory name under --out-root
                      (default: p2_YYMMDD_HHMMSS)
  --run-start-ns NS   Optional shared monotonic start timestamp override
                      (default: generated with time.monotonic_ns())
  --log-path FILE     P2 CSV path passed to p2_log_path
                      (single-node default: OUT_DIR/hpc_image_receive_p2.csv)
  --frame-log-dir DIR Directory for per-camera frame_id CSV files
                      (multi-camera default: OUT_DIR/frame_id)
  --eth-stats-iface IFACE
                      Optional interface for ethtool -S before/after capture
                      (default: disabled)
  --no-eth-stats      Do not capture ethtool -S before/after
  --adas-log MODE     ADAS seq logging: auto, on, or off (default: auto)
  --adas-control-path FILE
                      Control file watched by the running ADAS subscriber
                      (default: /tmp/autosdv_adas_p2_control.env)
  --adas-sequence-log-dir DIR
                      Directory for per-ADAS-topic seq CSV files
                      (default: OUT_DIR/adas_load_sequence)
  --no-adas-log       Disable ADAS seq logging
                      Requires adas_load_subscriber running with the same
                      --p2-control-path.
  --duration SEC      Required for run command. Accepts sleep(1) durations
                      such as 300, 300s, 5m.
  -h, --help          Show this help

Environment variables with the same names as the defaults are also accepted:
NODE, CAMERA_NODES, OUT_ROOT, RUN_ID, P2_RUN_START_NS, LOG_PATH,
FRAME_LOG_DIR, STATE_DIR, ETH_STATS_IFACE, ADAS_LOG, ADAS_CONTROL_PATH,
ADAS_SEQUENCE_LOG_DIR.
EOF
}

node_state_file() {
  local state_key safe_node
  if [[ -n "${NODE}" ]]; then
    state_key="${NODE}"
  else
    state_key="all_camera_nodes"
  fi
  safe_node="$(printf '%s' "${state_key}" | tr '/: ' '___')"
  printf '%s/%s.state' "${STATE_DIR}" "${safe_node}"
}

safe_log_name() {
  printf '%s' "$1" | tr '/: ' '___'
}

parse_camera_entry() {
  local entry="$1"
  if [[ "${entry}" != *=* ]]; then
    echo "Invalid --camera-nodes entry: ${entry} (expected node=topic)" >&2
    exit 2
  fi
  CAMERA_ENTRY_NODE="${entry%%=*}"
  CAMERA_ENTRY_TOPIC="${entry#*=}"
  if [[ -z "${CAMERA_ENTRY_NODE}" || -z "${CAMERA_ENTRY_TOPIC}" ]]; then
    echo "Invalid --camera-nodes entry: ${entry} (empty node/topic)" >&2
    exit 2
  fi
}

require_ros2() {
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 command not found. Source the ROS 2 and workspace setup first." >&2
    exit 1
  fi
}

PARAM_SET_PIDS=()
PARAM_SET_LABELS=()

param_set_bg() {
  local node="$1"
  local param="$2"
  local value="$3"

  ros2 param set "${node}" "${param}" "${value}" &
  PARAM_SET_PIDS+=("$!")
  PARAM_SET_LABELS+=("${node} ${param}")
}

wait_param_sets() {
  local status=0
  local index

  for index in "${!PARAM_SET_PIDS[@]}"; do
    if ! wait "${PARAM_SET_PIDS[${index}]}"; then
      echo "ros2 param set failed: ${PARAM_SET_LABELS[${index}]}" >&2
      status=1
    fi
  done

  PARAM_SET_PIDS=()
  PARAM_SET_LABELS=()

  if [[ "${status}" != "0" ]]; then
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

monotonic_ns() {
  python3 -c 'import time; print(time.monotonic_ns())'
}

make_run_paths() {
  if [[ -z "${OUT_DIR}" ]]; then
    if [[ -z "${RUN_ID}" ]]; then
      RUN_ID="p2_$(date +%y%m%d_%H%M%S)"
    fi
    OUT_DIR="${OUT_ROOT}/${RUN_ID}"
  elif [[ -z "${RUN_ID}" ]]; then
    RUN_ID="$(basename "${OUT_DIR}")"
  fi

  if [[ -n "${NODE}" && -z "${LOG_PATH}" ]]; then
    LOG_PATH="${OUT_DIR}/hpc_image_receive_p2.csv"
  fi
  if [[ -z "${NODE}" && -n "${LOG_PATH}" ]]; then
    echo "--log-path is only valid with --node single-node mode. Use --frame-log-dir for multi-camera mode." >&2
    exit 2
  fi
  if [[ -z "${NODE}" && -z "${FRAME_LOG_DIR}" ]]; then
    FRAME_LOG_DIR="${OUT_DIR}/frame_id"
  fi
  if [[ "${ADAS_LOG}" != "off" && -z "${ADAS_SEQUENCE_LOG_DIR}" ]]; then
    ADAS_SEQUENCE_LOG_DIR="${OUT_DIR}/adas_load_sequence"
  fi
  if [[ -z "${P2_RUN_START_NS}" ]]; then
    P2_RUN_START_NS="$(monotonic_ns)"
  fi

  mkdir -p "${OUT_DIR}" "${STATE_DIR}"
  if [[ -n "${FRAME_LOG_DIR}" ]]; then
    mkdir -p "${FRAME_LOG_DIR}"
  fi
  if [[ "${ADAS_LOG}" != "off" && -n "${ADAS_SEQUENCE_LOG_DIR}" ]]; then
    mkdir -p "${ADAS_SEQUENCE_LOG_DIR}"
  fi
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

write_metadata_start() {
  {
    if [[ -n "${NODE}" ]]; then
      echo "node=${NODE}"
    else
      echo "camera_nodes=${CAMERA_NODES}"
      echo "frame_log_dir=${FRAME_LOG_DIR}"
    fi
    echo "run_id=${RUN_ID}"
    echo "out_dir=${OUT_DIR}"
    echo "run_start_ns=${P2_RUN_START_NS}"
    if [[ -n "${LOG_PATH}" ]]; then
      echo "log_path=${LOG_PATH}"
    fi
    echo "eth_stats_iface=${ETH_STATS_IFACE}"
    echo "adas_log=${ADAS_LOG}"
    echo "adas_control_path=${ADAS_CONTROL_PATH}"
    echo "adas_sequence_log_dir=${ADAS_SEQUENCE_LOG_DIR}"
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
    printf 'CAMERA_NODES=%q\n' "${CAMERA_NODES}"
    printf 'OUT_ROOT=%q\n' "${OUT_ROOT}"
    printf 'RUN_ID=%q\n' "${RUN_ID}"
    printf 'OUT_DIR=%q\n' "${OUT_DIR}"
    printf 'P2_RUN_START_NS=%q\n' "${P2_RUN_START_NS}"
    printf 'LOG_PATH=%q\n' "${LOG_PATH}"
    printf 'FRAME_LOG_DIR=%q\n' "${FRAME_LOG_DIR}"
    printf 'ETH_STATS_IFACE=%q\n' "${ETH_STATS_IFACE}"
    printf 'ADAS_LOG=%q\n' "${ADAS_LOG}"
    printf 'ADAS_CONTROL_PATH=%q\n' "${ADAS_CONTROL_PATH}"
    printf 'ADAS_SEQUENCE_LOG_DIR=%q\n' "${ADAS_SEQUENCE_LOG_DIR}"
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

write_adas_control() {
  local enabled="$1"

  if [[ "${ADAS_LOG}" == "off" ]]; then
    return 0
  fi
  if [[ "${ADAS_LOG}" != "auto" && "${ADAS_LOG}" != "on" ]]; then
    echo "Invalid --adas-log: ${ADAS_LOG} (expected auto, on, or off)" >&2
    exit 2
  fi

  local control_dir tmp_path
  control_dir="$(dirname "${ADAS_CONTROL_PATH}")"
  mkdir -p "${control_dir}"
  tmp_path="${ADAS_CONTROL_PATH}.$$"

  if [[ "${enabled}" == "1" ]]; then
    {
      echo "enabled=1"
      echo "run_id=${RUN_ID}"
      echo "run_start_ns=${P2_RUN_START_NS}"
      echo "sequence_log_dir=${ADAS_SEQUENCE_LOG_DIR}"
      echo "updated_at=$(date --iso-8601=ns)"
    } > "${tmp_path}"
  else
    {
      echo "enabled=0"
      echo "run_id=${RUN_ID}"
      echo "run_start_ns=${P2_RUN_START_NS}"
      echo "sequence_log_dir="
      echo "updated_at=$(date --iso-8601=ns)"
    } > "${tmp_path}"
  fi
  mv "${tmp_path}" "${ADAS_CONTROL_PATH}"
}

cmd_start() {
  require_ros2
  make_run_paths

  capture_snmp before >/dev/null
  capture_eth_stats before >/dev/null

  if [[ -n "${NODE}" ]]; then
    param_set_bg "${NODE}" p2_log_run_id "${RUN_ID}"
    param_set_bg "${NODE}" p2_log_start_ns "${P2_RUN_START_NS}"
    param_set_bg "${NODE}" p2_log_path "${LOG_PATH}"
    wait_param_sets
    param_set_bg "${NODE}" p2_log_enabled true
    wait_param_sets
  else
    local manifest="${OUT_DIR}/frame_id_logs.csv"
    printf 'node,topic,path\n' > "${manifest}"
    local entry node topic log_path safe_topic
    for entry in ${CAMERA_NODES}; do
      parse_camera_entry "${entry}"
      node="${CAMERA_ENTRY_NODE}"
      topic="${CAMERA_ENTRY_TOPIC}"
      safe_topic="$(safe_log_name "${topic}")"
      log_path="${FRAME_LOG_DIR}/${safe_topic}.csv"
      param_set_bg "${node}" p2_log_run_id "${RUN_ID}"
      param_set_bg "${node}" p2_log_start_ns "${P2_RUN_START_NS}"
      param_set_bg "${node}" p2_log_path "${log_path}"
      printf '%s,%s,%s\n' "${node}" "${topic}" "${log_path}" >> "${manifest}"
    done
    wait_param_sets

    for entry in ${CAMERA_NODES}; do
      parse_camera_entry "${entry}"
      node="${CAMERA_ENTRY_NODE}"
      param_set_bg "${node}" p2_log_enabled true
    done
    wait_param_sets
  fi
  write_adas_control 1
  write_metadata_start

  write_state
  if [[ -n "${NODE}" ]]; then
    echo "P2 log enabled on ${NODE}"
    echo "P2 CSV: ${LOG_PATH}"
  else
    echo "P2 log enabled on camera nodes"
    echo "P2 CSV directory: ${FRAME_LOG_DIR}"
    echo "P2 CSV manifest: ${OUT_DIR}/frame_id_logs.csv"
  fi
  if [[ "${ADAS_LOG}" != "off" ]]; then
    echo "ADAS seq CSV directory: ${ADAS_SEQUENCE_LOG_DIR}"
    echo "ADAS control file: ${ADAS_CONTROL_PATH}"
  fi
  echo "SNMP before: ${OUT_DIR}/proc_net_snmp.before.txt"
  if [[ -r "${OUT_DIR}/ethtool_stats.before.txt" ]]; then
    echo "ethtool stats before: ${OUT_DIR}/ethtool_stats.before.txt"
  fi
}

cmd_stop() {
  require_ros2

  if [[ -n "${OUT_DIR}" ]]; then
    mkdir -p "${OUT_DIR}" "${STATE_DIR}"
    if [[ -z "${NODE}" && -z "${FRAME_LOG_DIR}" ]]; then
      FRAME_LOG_DIR="${OUT_DIR}/frame_id"
    fi
    STATE_FILE="$(node_state_file)"
    if [[ -f "${STATE_FILE}" ]]; then
      # shellcheck disable=SC1090
      source "${STATE_FILE}"
    fi
  else
    read_state
  fi

  if [[ -n "${NODE}" ]]; then
    param_set_bg "${NODE}" p2_log_enabled false
    wait_param_sets
  else
    local entry node
    for entry in ${CAMERA_NODES}; do
      parse_camera_entry "${entry}"
      node="${CAMERA_ENTRY_NODE}"
      param_set_bg "${node}" p2_log_enabled false
    done
    wait_param_sets
  fi
  write_adas_control 0
  write_metadata_stop
  capture_snmp after >/dev/null
  capture_eth_stats after >/dev/null
  rm -f "${STATE_FILE}"

  if [[ -n "${NODE}" ]]; then
    echo "P2 log disabled on ${NODE}"
  else
    echo "P2 log disabled on camera nodes"
    echo "P2 CSV directory: ${FRAME_LOG_DIR}"
  fi
  echo "SNMP after: ${OUT_DIR}/proc_net_snmp.after.txt"
  if [[ -r "${OUT_DIR}/ethtool_stats.after.txt" ]]; then
    echo "ethtool stats after: ${OUT_DIR}/ethtool_stats.after.txt"
  fi
  if [[ "${ADAS_LOG}" != "off" ]]; then
    echo "ADAS seq CSV directory: ${ADAS_SEQUENCE_LOG_DIR}"
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
    --camera-nodes)
      CAMERA_NODES="$2"
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
    --run-start-ns|--p2-run-start-ns)
      P2_RUN_START_NS="$2"
      shift 2
      ;;
    --log-path)
      LOG_PATH="$2"
      shift 2
      ;;
    --frame-log-dir)
      FRAME_LOG_DIR="$2"
      shift 2
      ;;
    --eth-stats-iface)
      ETH_STATS_IFACE="$2"
      shift 2
      ;;
    --no-eth-stats)
      ETH_STATS_IFACE=""
      shift
      ;;
    --adas-log)
      ADAS_LOG="$2"
      shift 2
      ;;
    --no-adas-log)
      ADAS_LOG="off"
      shift
      ;;
    --adas-control-path)
      ADAS_CONTROL_PATH="$2"
      shift 2
      ;;
    --adas-sequence-log-dir|--adas-seq-log-dir)
      ADAS_SEQUENCE_LOG_DIR="$2"
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
