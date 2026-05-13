#!/usr/bin/env bash
# realtime-tune.sh — HPC(Laptop) 측정/실행 직전 노이즈 제거 토글
# 사용법:
#   sudo ./realtime-tune.sh on        # 튜닝 적용
#   sudo ./realtime-tune.sh off       # 원복 (대부분은 재부팅으로도 원복됨)
#   ./realtime-tune.sh status         # 현재 상태 확인 (sudo 불필요)
#
# 영향 범위:
#   - rfkill (WiFi/BT/WWAN)
#   - USB autosuspend
#   - bluetooth/ModemManager/snapd/packagekit/unattended-upgrades/cups/avahi
#   - GNOME: gnome-remote-desktop, tracker3 indexer
#   - CPU governor → performance, power-profiles-daemon 정지
#   - swap off (RAM 여유 있을 때만)

set -u

LOG_TAG="[realtime-tune]"

log() { echo "${LOG_TAG} $*"; }
warn() { echo "${LOG_TAG} WARN: $*" >&2; }

need_root() {
    if [[ $EUID -ne 0 ]]; then
        echo "${LOG_TAG} '$1' 는 root 권한이 필요합니다. sudo 로 다시 실행하세요." >&2
        exit 1
    fi
}

is_gnome() {
    [[ "${XDG_CURRENT_DESKTOP:-}" == *GNOME* ]] || pgrep -x gnome-shell >/dev/null 2>&1
}

# ---------- ON ----------
apply_on() {
    need_root on

    log "1) rfkill block all (WiFi/BT/WWAN OFF)"
    rfkill block all || warn "rfkill 실패"

    log "2) USB autosuspend 끄기 (control=on)"
    for f in /sys/bus/usb/devices/*/power/control; do
        [[ -w "$f" ]] && echo on > "$f" 2>/dev/null || true
    done

    log "3) 백그라운드 서비스 정지"
    local services=(
        bluetooth
        ModemManager
        snapd snapd.socket
        packagekit
        unattended-upgrades
        cups cups-browsed
        avahi-daemon
    )
    for svc in "${services[@]}"; do
        systemctl stop "$svc" 2>/dev/null && log "   stopped: $svc" || true
    done

    if is_gnome; then
        log "   (GNOME 감지) gnome-remote-desktop / tracker3 정지"
        systemctl stop gnome-remote-desktop 2>/dev/null || true
        # tracker3 는 user 세션 데몬 — sudo 로는 못 끄므로 호출자 UID 로 실행
        local invoker_uid="${SUDO_UID:-$(id -u)}"
        if command -v tracker3 >/dev/null 2>&1 && [[ "$invoker_uid" != "0" ]]; then
            sudo -u "#${invoker_uid}" tracker3 daemon -t 2>/dev/null || true
        fi
    fi

    log "4) CPU governor → performance"
    if command -v cpupower >/dev/null 2>&1; then
        cpupower frequency-set -g performance >/dev/null || warn "cpupower 실패"
    else
        for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
            [[ -w "$g" ]] && echo performance > "$g" 2>/dev/null || true
        done
    fi
    systemctl stop power-profiles-daemon 2>/dev/null && log "   stopped: power-profiles-daemon" || true
    systemctl stop tlp 2>/dev/null && log "   stopped: tlp" || true

    log "5) Swap off (사용 중인 swap < 여유 RAM 일 때만)"
    local mem_avail_kb swap_used_kb
    mem_avail_kb=$(awk '/MemAvailable/{print $2}' /proc/meminfo)
    swap_used_kb=$(awk '/SwapTotal/{t=$2} /SwapFree/{f=$2} END{print t-f}' /proc/meminfo)
    if (( swap_used_kb < mem_avail_kb )); then
        swapoff -a && log "   swapoff OK (사용 중이던 ${swap_used_kb} KB → RAM 으로)"
    else
        warn "swap 사용량($swap_used_kb KB) 이 여유 RAM($mem_avail_kb KB) 보다 큼 — swapoff 건너뜀"
    fi

    log "완료. 측정/실행 끝나면 './realtime-tune.sh off' 또는 재부팅으로 원복."
}

# ---------- OFF ----------
apply_off() {
    need_root off

    log "1) rfkill unblock all"
    rfkill unblock all || true

    log "2) USB autosuspend 자동 복귀 (control=auto)"
    for f in /sys/bus/usb/devices/*/power/control; do
        [[ -w "$f" ]] && echo auto > "$f" 2>/dev/null || true
    done

    log "3) 백그라운드 서비스 재시작"
    local services=(
        bluetooth
        ModemManager
        snapd snapd.socket
        packagekit
        unattended-upgrades
        cups cups-browsed
        avahi-daemon
    )
    for svc in "${services[@]}"; do
        systemctl start "$svc" 2>/dev/null && log "   started: $svc" || true
    done

    if is_gnome; then
        systemctl start gnome-remote-desktop 2>/dev/null || true
        # tracker3 는 다음 파일 접근 시 자동 기동, 별도 명령 불필요
    fi

    log "4) CPU governor → powersave/schedutil (배포판 기본 복귀)"
    systemctl start power-profiles-daemon 2>/dev/null || true
    # power-profiles-daemon 가 governor 를 자동으로 잡지 못하면 schedutil 로
    if command -v cpupower >/dev/null 2>&1; then
        cpupower frequency-set -g schedutil >/dev/null 2>&1 \
            || cpupower frequency-set -g powersave >/dev/null 2>&1 || true
    fi

    log "5) Swap on"
    swapon -a 2>/dev/null || true

    log "원복 완료."
}

# ---------- STATUS ----------
show_status() {
    echo "${LOG_TAG} === Status ==="

    echo "[rfkill]"
    rfkill 2>/dev/null || echo "  (rfkill not installed)"

    echo "[CPU governor]"
    if compgen -G "/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor" >/dev/null; then
        awk 'BEGIN{c=""} {if($0!=c){print "  "$0; c=$0}}' \
            /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
    else
        echo "  (cpufreq not available)"
    fi

    echo "[Services]"
    for svc in bluetooth ModemManager snapd packagekit unattended-upgrades \
               cups avahi-daemon power-profiles-daemon tlp gnome-remote-desktop; do
        local state
        state=$(systemctl is-active "$svc" 2>/dev/null)
        [[ -z "$state" ]] && state="n/a"
        printf "  %-28s %s\n" "$svc" "$state"
    done

    echo "[Swap]"
    awk '/^Swap(Total|Free):/{print "  "$0}' /proc/meminfo

    echo "[USB autosuspend (control values, 처음 10개)]"
    local i=0
    for f in /sys/bus/usb/devices/*/power/control; do
        [[ -r "$f" ]] || continue
        printf "  %s = %s\n" "$f" "$(cat "$f")"
        (( ++i >= 10 )) && { echo "  ..."; break; }
    done
}

# ---------- main ----------
case "${1:-}" in
    on)     apply_on ;;
    off)    apply_off ;;
    status) show_status ;;
    *)
        cat <<EOF
Usage: $0 {on|off|status}

  on      튜닝 적용 (sudo 필요)
  off     원복 (sudo 필요)
  status  현재 상태 출력 (sudo 불필요)
EOF
        exit 1
        ;;
esac
