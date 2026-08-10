#!/bin/bash
# Pi 5 power-safety helper for benchmark runs (run on the Pi HOST OS, not in
# a container). Context: repeated abrupt power-offs during benchmarks with no
# throttle flag and low temperature — see docs/optimize/pi_benchmark_guide.md
# §8.1. This script (a) pins CPU clocks for stable measurements, optionally
# capped below max to cut peak current draw, (b) enables persistent journald
# so kernel logs survive a power loss, and (c) samples the PMIC's real 5V
# input rail (EXT5V_V) — the signal that actually shows a supply brownout,
# unlike `measure_volts core` which the PMIC holds stable until the very end.
#
# Usage:
#   ./pi_power_guard.sh setup [MAX_MHZ]   # governor=performance (+ freq cap), persistent journal
#   ./pi_power_guard.sh monitor [LOGFILE] # 1Hz sampler: temp, throttled, EXT5V, core V/A
#   ./pi_power_guard.sh status            # print current governor/freq/temp/throttle/EXT5V
#   ./pi_power_guard.sh restore           # governor=ondemand, uncap freq
#
# Typical benchmark flow (see guide §7): `setup 2000` first (capped — all
# presets under the same cap keep FP32/FP16/INT8 comparison valid), `monitor`
# in a separate tmux pane during the whole run.

set -u

CPUFREQ_GLOB=/sys/devices/system/cpu/cpu[0-9]*/cpufreq

die() { echo "[pi_power_guard] ERROR: $*" >&2; exit 1; }

require_pi() {
    [ -d /sys/devices/system/cpu/cpu0/cpufreq ] || die "no cpufreq sysfs — is this the Pi host OS?"
}

set_governor() {
    local gov=$1
    for d in $CPUFREQ_GLOB; do
        echo "$gov" | sudo tee "$d/scaling_governor" >/dev/null || die "failed to set governor on $d"
    done
    echo "[pi_power_guard] governor=$gov on all cores"
}

set_max_freq_khz() {
    local khz=$1
    for d in $CPUFREQ_GLOB; do
        echo "$khz" | sudo tee "$d/scaling_max_freq" >/dev/null || die "failed to set max freq on $d"
    done
    echo "[pi_power_guard] scaling_max_freq=${khz} kHz on all cores"
}

pmic_line() {
    # Single named channel, e.g. EXT5V_V. Falls back to N/A when vcgencmd or
    # the channel is unavailable (non-Pi5 firmware).
    local out
    out=$(vcgencmd pmic_read_adc "$1" 2>/dev/null | tr -s ' ' | sed 's/^ //')
    echo "${out:-N/A}"
}

cmd_setup() {
    require_pi
    local max_mhz="${1:-}"

    set_governor performance
    if [ -n "$max_mhz" ]; then
        set_max_freq_khz "$(( max_mhz * 1000 ))"
    else
        # explicit uncap so a previous capped run doesn't leak into this one
        local hw_max
        hw_max=$(cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq)
        set_max_freq_khz "$hw_max"
    fi

    # Persistent journal: keep kernel logs across a hard power loss
    if [ ! -d /var/log/journal ]; then
        sudo mkdir -p /var/log/journal
        sudo systemctl restart systemd-journald
        echo "[pi_power_guard] persistent journal enabled (/var/log/journal)"
    fi
    if grep -qE '^\s*Storage=volatile' /etc/systemd/journald.conf 2>/dev/null; then
        echo "[pi_power_guard] WARNING: journald.conf has Storage=volatile — change to auto/persistent or logs will NOT survive a power loss"
    fi

    cmd_status
}

cmd_restore() {
    require_pi
    set_governor ondemand
    set_max_freq_khz "$(cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq)"
}

cmd_status() {
    require_pi
    echo "governor:  $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
    echo "max freq:  $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq) kHz"
    echo "cur freq:  $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq) kHz"
    echo "temp:      $(vcgencmd measure_temp 2>/dev/null || echo N/A)"
    echo "throttled: $(vcgencmd get_throttled 2>/dev/null || echo N/A)"
    echo "5V input:  $(pmic_line EXT5V_V)"
    echo "core:      $(pmic_line VDD_CORE_V) $(pmic_line VDD_CORE_A)"
}

cmd_monitor() {
    local log="${1:-$HOME/power_log.txt}"
    echo "[pi_power_guard] sampling 1Hz to $log (Ctrl+C to stop)"
    echo "[pi_power_guard] watch EXT5V_V: healthy ~5.0-5.2V; dips toward 4.6V right before a crash = supply brownout"
    while true; do
        echo "$(date +%T) $(vcgencmd measure_temp 2>/dev/null) $(vcgencmd get_throttled 2>/dev/null) $(pmic_line EXT5V_V) $(pmic_line VDD_CORE_V) $(pmic_line VDD_CORE_A)"
        sleep 1
    done | tee -a "$log"
}

case "${1:-}" in
    setup)   cmd_setup "${2:-}" ;;
    monitor) cmd_monitor "${2:-}" ;;
    status)  cmd_status ;;
    restore) cmd_restore ;;
    *)
        sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
        exit 1
        ;;
esac
