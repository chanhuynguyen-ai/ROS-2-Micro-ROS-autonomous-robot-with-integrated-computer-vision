#!/usr/bin/env bash
#
# Khởi động container Yahboom ROS2 Humble (joy teleop) — bản đã làm cứng.
#
# Thay cho ~/ros2_humble.sh cũ:
#   docker run -it --privileged=true --net=host --env="DISPLAY" ... \
#     -v /tem/.X11-unix:/tmp/.X11-unix ... yahboomtechnology/ros-humble:4.1.2 \
#     /bin/bash /root/1.sh
#
# Ba lỗi của bản cũ, đã fix ở đây:
#   1. Không --name, không --rm  -> mỗi lần chạy đẻ thêm container tên ngẫu
#      nhiên (hopeful_fermat, trusting_sammet, ...) và không bao giờ được dọn.
#   2. Typo `-v /tem/.X11-unix`  -> Docker tự tạo thư mục RỖNG /tem/.X11-unix
#      trên host rồi mount vào /tmp/.X11-unix trong container => X11 forwarding
#      hỏng im lặng suốt (đó là lý do GUI không lên, exit 255).
#   3. `xhost +` mở toang access control cho MỌI client.
#
# CẢNH BÁO QUAN TRỌNG — xung đột /cmd_vel, KHÔNG phải xung đột Serial:
#   Image này có sẵn ROS_DOMAIN_ID=20 (trùng domain của AVS) và chạy --net=host.
#   /root/1.sh -> supervisor -> [program:ChassisServer] -> run_handle.sh
#     -> ros2 launch yahboomcar_ctrl yahboomcar_joy_launch.py
#     -> chỉ có 2 node: yahboom_joy + joy_node.
#   Nó KHÔNG mở /dev/ttyUSB0. Nhưng nó PUBLISH /cmd_vel từ tay cầm, cạnh tranh
#   trực tiếp với stack tự hành trên đúng topic mà ESP32 subscribe.
#   Script chỉ CẢNH BÁO chứ không chặn — teleop thường được dùng làm override
#   thủ công/E-stop, chặn nó có thể nguy hiểm hơn là để chạy.
#
# Dùng:
#   ./start_ros2_humble_rpi5.sh            # dọn instance cũ + chạy (foreground)
#   ./start_ros2_humble_rpi5.sh --status   # xem trạng thái, không đụng gì
#   ./start_ros2_humble_rpi5.sh --stop     # dừng instance đang chạy
#   ./start_ros2_humble_rpi5.sh --prune    # xoá luôn các container đã Exited

set -euo pipefail

CONTAINER="yahboom_ros"
IMAGE="${YAHBOOM_IMAGE:-yahboomtechnology/ros-humble:4.1.2}"
IMAGE_REPO="yahboomtechnology/ros-humble"
AVS_CONTAINER="avs_perception_container"

log()  { printf '\033[1;34m[yahboom]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[yahboom]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[yahboom]\033[0m %s\n' "$*" >&2; exit 1; }

# Liệt kê theo image chứ không theo tên: container bản cũ mang tên ngẫu nhiên.
# Gán riêng + `|| true` vì `set -o pipefail` sẽ giết script nếu docker lỗi.
yahboom_containers() {
  local all
  all="$(docker ps -a --format '{{.ID}} {{.Image}} {{.State}}' 2>/dev/null || true)"
  local want_state="${1:-any}"
  if [ -n "$all" ]; then
    printf '%s\n' "$all" | awk -v img="$IMAGE_REPO" -v st="$want_state" '
      index($2, img) == 1 && (st == "any" || $3 == st) { print $1 }'
  fi
}

container_running() {
  [ "$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" = "true" ]
}

show_status() {
  local ids count
  ids="$(yahboom_containers)"
  count="$(printf '%s' "$ids" | grep -c . || true)"
  log "Container Yahboom hiện có: $count"
  if [ -n "$ids" ]; then
    # shellcheck disable=SC2086
    docker inspect $ids \
      --format '  {{.Name}}  {{.Config.Image}}  running={{.State.Running}} exit={{.State.ExitCode}}' \
      2>/dev/null || true
  fi
  if container_running "$AVS_CONTAINER"; then
    log "Stack AVS: ĐANG CHẠY (control_node cũng đang lái /cmd_vel)"
  else
    log "Stack AVS: không chạy"
  fi
}

# Chỉ xoá container ĐANG CHẠY (bắt buộc để đảm bảo singleton).
# Container đã Exited có thể chứa thay đổi người dùng tự sửa bên trong nên
# không tự xoá — chỉ báo và để `--prune` quyết định.
remove_running() {
  local ids
  ids="$(yahboom_containers running)"
  if [ -n "$ids" ]; then
    log "Dọn $(printf '%s' "$ids" | grep -c . || true) container Yahboom đang chạy..."
    # shellcheck disable=SC2086
    docker rm -f $ids >/dev/null
  fi
  local dead
  dead="$(yahboom_containers exited)"
  local n
  n="$(printf '%s' "$dead" | grep -c . || true)"
  if [ "$n" -gt 0 ]; then
    warn "Còn $n container Yahboom đã Exited (rác từ bản script cũ)."
    warn "Xoá bằng: $0 --prune"
  fi
}

warn_cmd_vel_conflict() {
  container_running "$AVS_CONTAINER" || return 0
  warn "─────────────────────────────────────────────────────────────"
  warn "$AVS_CONTAINER đang chạy: control_node đang publish lệnh lái."
  warn "Container này sẽ chạy joy_node + yahboom_joy, cũng publish /cmd_vel"
  warn "trên cùng ROS_DOMAIN_ID=20 => HAI nguồn lệnh tranh nhau lái xe."
  warn "Chỉ tiếp tục nếu bạn CHỦ Ý dùng tay cầm để override thủ công."
  warn "─────────────────────────────────────────────────────────────"
}

main() {
  local action="${1:-start}"

  case "$action" in
    --status) show_status; return 0 ;;
    --stop)
      log "Dừng container Yahboom..."
      local ids
      ids="$(yahboom_containers running)"
      if [ -z "$ids" ]; then
        log "Không có container nào đang chạy."
      else
        # shellcheck disable=SC2086
        docker rm -f $ids >/dev/null
        log "Đã dừng."
      fi
      return 0
      ;;
    --prune)
      local dead
      dead="$(yahboom_containers exited)"
      if [ -z "$dead" ]; then
        log "Không có container Exited nào để xoá."
      else
        log "Xoá $(printf '%s' "$dead" | grep -c . || true) container đã Exited:"
        # shellcheck disable=SC2086
        docker inspect $dead --format '  {{.Name}} exit={{.State.ExitCode}}' 2>/dev/null || true
        # shellcheck disable=SC2086
        docker rm $dead >/dev/null
        log "Xong."
      fi
      return 0
      ;;
    start) ;;
    *) die "Tham số không hợp lệ: $action (dùng --status | --stop | --prune)" ;;
  esac

  remove_running
  warn_cmd_vel_conflict

  # X11: chỉ nới quyền cho client local, thay vì `xhost +` mở cho toàn mạng.
  # Bỏ qua khi chạy qua SSH không có DISPLAY (GUI vốn không dùng được).
  local x11_args=()
  if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
    xhost +local:root >/dev/null 2>&1 || warn "xhost thất bại — GUI có thể không lên."
    # /tmp/.X11-unix — KHÔNG phải /tem (typo của bản cũ).
    x11_args=(--env "DISPLAY=$DISPLAY" --env QT_X11_NO_MITSHM=1
              -v /tmp/.X11-unix:/tmp/.X11-unix)
  else
    warn "DISPLAY chưa set — chạy không GUI (rviz/rqt sẽ không mở được)."
  fi

  # Chỉ mount thiết bị thực sự tồn tại, tránh Docker tự tạo thư mục rỗng
  # rồi mount đè — đúng cái bẫy đã làm hỏng X11 ở bản cũ.
  local dev_args=()
  [ -d /dev/input ]   && dev_args+=(-v /dev/input:/dev/input)
  [ -e /dev/video0 ]  && dev_args+=(-v /dev/video0:/dev/video0)

  log "Khởi động $CONTAINER (Ctrl-C hoặc exit để thoát, container tự xoá)..."
  exec docker run -it --rm \
    --name "$CONTAINER" \
    --privileged=true \
    --net=host \
    --security-opt apparmor:unconfined \
    "${x11_args[@]}" \
    "${dev_args[@]}" \
    "$IMAGE" /bin/bash /root/1.sh
}

main "$@"
