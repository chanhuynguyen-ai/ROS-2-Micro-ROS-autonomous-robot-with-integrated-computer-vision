#!/usr/bin/env bash
#
# Khởi động micro-ROS agent (ESP32 <-> ROS2) đúng MỘT instance duy nhất.
#
# Vấn đề đã fix: script cũ ở ~/start_agent_rpi5.sh gọi `docker run -it --rm ...`
# không có --name và không có guard singleton. Chạy script ở nhiều terminal =
# nhiều container agent cùng open() /dev/ttyUSB0 -> tranh chấp cổng Serial ->
# ESP32 reset/mất session XRCE-DDS -> subscriber /cmd_vel bị hủy đăng ký.
# Ngoài ra `-it` khiến container chết theo terminal đã khởi động nó.
#
# Cách fix: agent giờ là service trong docker-compose.prod.yml (container_name
# cố định => Compose tự đảm bảo singleton, chạy detached, restart unless-stopped).
# Script này chỉ dọn instance cũ rồi gọi Compose.
#
# Dùng:
#   ./start_agent_rpi5.sh            # dọn agent cũ + start, in log gần nhất
#   ./start_agent_rpi5.sh --follow   # như trên, rồi bám log (giống hành vi cũ)
#   ./start_agent_rpi5.sh --stop     # dừng hẳn (không bị restart policy bật lại)
#   ./start_agent_rpi5.sh --status   # chỉ xem trạng thái, không đụng gì

set -euo pipefail

# readlink -f: script thường được gọi qua symlink ~/start_agent_rpi5.sh, khi đó
# dirname của BASH_SOURCE là /home/pi chứ không phải scripts/ trong repo.
SELF="$(readlink -f "${BASH_SOURCE[0]}")"
REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$SELF")/.." && pwd)}"
COMPOSE_FILE="$REPO_DIR/docker-compose.prod.yml"
SERVICE="micro_ros_agent"
CONTAINER="micro_ros_agent"
AGENT_IMAGE="microros/micro-ros-agent"
DEV="${MICRO_ROS_DEV:-/dev/ttyUSB0}"
RELEASE_TIMEOUT=10

log()  { printf '\033[1;34m[agent]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[agent]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[agent]\033[0m %s\n' "$*" >&2; exit 1; }

compose() {
  if docker compose version >/dev/null 2>&1; then
    docker compose -f "$COMPOSE_FILE" "$@"
  elif command -v docker-compose >/dev/null 2>&1; then
    docker-compose -f "$COMPOSE_FILE" "$@"
  else
    die "Không tìm thấy 'docker compose' lẫn 'docker-compose'."
  fi
}

# Mọi container sinh ra từ image micro-ros-agent, bất kể tag hay tên ngẫu nhiên
# (adoring_cannon, tender_neumann, ...), cộng với container tên cố định.
agent_containers() {
  local all named
  # Gán riêng + `|| true`: dưới `set -o pipefail`, docker lỗi (vd. thiếu quyền
  # docker.sock) sẽ làm cả pipeline non-zero và `set -e` giết script giữa chừng.
  all="$(docker ps -a --format '{{.ID}} {{.Image}}' 2>/dev/null || true)"
  named="$(docker ps -aq --filter "name=^/${CONTAINER}$" 2>/dev/null || true)"
  {
    if [ -n "$all" ]; then
      printf '%s\n' "$all" \
        | awk -v img="$AGENT_IMAGE" 'index($2, img) == 1 { print $1 }'
    fi
    if [ -n "$named" ]; then
      printf '%s\n' "$named"
    fi
  } | sort -u
}

# Ai đang giữ cổng Serial. Trả về rỗng nếu không dò được (thiếu quyền/thiếu tool).
serial_holders() {
  local fuser_cmd=""
  if sudo -n true >/dev/null 2>&1 && command -v fuser >/dev/null 2>&1; then
    fuser_cmd="sudo -n fuser"
  elif command -v fuser >/dev/null 2>&1; then
    fuser_cmd="fuser"
  else
    return 0
  fi
  $fuser_cmd "$DEV" 2>/dev/null | tr -s ' ' '\n' | grep -E '^[0-9]+$' || true
}

show_status() {
  # Liệt kê theo ID chứ không theo tên: container do `docker run` rời tạo ra
  # mang tên ngẫu nhiên (adoring_cannon, ...) nên lọc theo tên sẽ bỏ sót.
  local ids count
  ids="$(agent_containers)"
  count="$(printf '%s' "$ids" | grep -c . || true)"
  log "Container agent hiện có: $count"
  if [ -n "$ids" ]; then
    # shellcheck disable=SC2086
    docker inspect $ids \
      --format '  {{.Name}}  {{.Config.Image}}  running={{.State.Running}}' 2>/dev/null || true
  fi
  if [ "$count" -gt 1 ]; then
    warn "CẢNH BÁO: $count container agent cùng tồn tại — chúng tranh chấp $DEV."
    warn "Chạy '$0' để dọn về đúng 1 instance."
  fi
  log "Tiến trình giữ $DEV:"
  local pids
  pids="$(serial_holders)"
  if [ -z "$pids" ]; then
    log "  (không có, hoặc không dò được — cần quyền root)"
  else
    # shellcheck disable=SC2086
    ps -o pid,comm,args -p $(echo "$pids" | tr '\n' ',' | sed 's/,$//') || true
  fi
}

remove_agents() {
  local ids
  ids="$(agent_containers)"
  [ -z "$ids" ] && return 0
  log "Dọn $(echo "$ids" | wc -l) container agent cũ..."
  # shellcheck disable=SC2086
  docker rm -f $ids >/dev/null
}

# Chờ kernel nhả cổng Serial sau khi container bị xóa.
wait_serial_released() {
  local i=0
  while [ "$i" -lt "$RELEASE_TIMEOUT" ]; do
    pgrep -x micro_ros_agent >/dev/null 2>&1 || return 0
    sleep 1
    i=$((i + 1))
  done
  warn "Sau ${RELEASE_TIMEOUT}s vẫn còn tiến trình micro_ros_agent:"
  pgrep -ax micro_ros_agent >&2 || true
  return 1
}

# Cổng bị giữ bởi thứ KHÁC agent. Không tự kill: đó là quyết định của người dùng.
# (Đã kiểm chứng: container yahboomtechnology/ros-humble KHÔNG mở /dev/ttyUSB0 —
# nó chỉ chạy joy teleop. Xung đột của nó là ở /cmd_vel, xem deployment_guide §5.2.)
assert_serial_free() {
  local pids
  pids="$(serial_holders)"
  [ -z "$pids" ] && return 0
  warn "$DEV đang bị tiến trình khác giữ:"
  # shellcheck disable=SC2086
  ps -o pid,comm,args -p $(echo "$pids" | tr '\n' ',' | sed 's/,$//') >&2 || true
  warn "Kiểm tra xem có container nào khác (vd. yahboomtechnology/ros-humble)"
  warn "đang mở cổng này không: docker ps"
  die "Dừng lại — start agent lúc này sẽ gây tranh chấp đúng như lỗi cũ."
}

main() {
  local action="${1:-start}"

  [ -f "$COMPOSE_FILE" ] || die "Không thấy $COMPOSE_FILE"

  case "$action" in
    --status)
      show_status
      return 0
      ;;
    --stop)
      # `docker compose stop` để restart policy không bật lại.
      log "Dừng service $SERVICE..."
      compose stop "$SERVICE" >/dev/null 2>&1 || true
      remove_agents
      log "Đã dừng."
      return 0
      ;;
    start|--follow|-f) ;;
    *) die "Tham số không hợp lệ: $action (dùng --follow | --stop | --status)" ;;
  esac

  [ -e "$DEV" ] || die "Không thấy $DEV — kiểm tra cáp USB ESP32 (lsusb: CP210x)."

  remove_agents
  wait_serial_released || die "Không nhả được $DEV. Kiểm tra thủ công rồi chạy lại."
  assert_serial_free

  log "Khởi động $SERVICE qua Compose (detached, singleton)..."
  compose up -d "$SERVICE"

  sleep 2
  if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" != "true" ]; then
    warn "Container $CONTAINER không chạy. Log:"
    docker logs --tail 30 "$CONTAINER" 2>&1 | sed 's/^/  /' >&2 || true
    die "Khởi động thất bại."
  fi

  log "OK — agent đang chạy:"
  docker ps --filter "name=^/${CONTAINER}$" \
    --format '  {{.Names}}  {{.Image}}  {{.Status}}'
  log "Log gần nhất:"
  docker logs --tail 10 "$CONTAINER" 2>&1 | sed 's/^/  /'

  if [ "$action" = "--follow" ] || [ "$action" = "-f" ]; then
    log "Bám log (Ctrl-C để thoát — container VẪN chạy nền):"
    exec docker logs -f "$CONTAINER"
  fi

  log "Xem log:  docker logs -f $CONTAINER"
  log "Dừng hẳn: $0 --stop"
}

main "$@"
