#!/usr/bin/env bash
# Lite3 robot package installer, the ROBOT_SETUP hook of Molly's script/setup.sh.
#
#   lite3_setup.sh check            motion host, its state reports, GStreamer / OpenCV
#   lite3_setup.sh install          apt packages, then what to change on the motion host
#   lite3_setup.sh network          the network.toml change for this computer
#   lite3_setup.sh network --apply  make that change on the motion host over ssh (with a backup)
#
# The motion host only reports its state to the computers listed in ~/jy_exe/conf/network.toml
# (Deep Robotics "SDK Deploy" 1.3.2). Nothing on the robot is changed without --apply.
# Env: MOTION_HOST (ROBOT_IP, else 192.168.1.120), MOTION_USER (ysc), LOCAL_PORT (43897),
#      ROS_DISTRO (humble), REALSENSE=1 (also install the RealSense driver)
set -euo pipefail

MOTION_HOST="${MOTION_HOST:-${ROBOT_IP:-192.168.1.120}}"
MOTION_USER="${MOTION_USER:-ysc}"
LOCAL_PORT="${LOCAL_PORT:-43897}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
SUDO=$([[ $EUID -eq 0 ]] && echo "" || echo sudo)

APT_PACKAGES=(
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav python3-opencv
    python3-yaml python3-numpy iputils-ping iproute2
    "ros-$ROS_DISTRO-xacro" "ros-$ROS_DISTRO-robot-state-publisher"
)

local_ip() {
    ip -4 route get "$MOTION_HOST" 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p' | head -n1
}

# Count the motion host's datagrams on LOCAL_PORT for 3 s ("busy" when the driver has the port).
state_rate() {
    python3 - "$LOCAL_PORT" <<'PY'
import socket, sys, time
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.bind(('0.0.0.0', int(sys.argv[1])))
except OSError:
    print('busy')
    sys.exit(0)
sock.settimeout(0.2)
count, end = 0, time.monotonic() + 3.0
while time.monotonic() < end:
    try:
        sock.recvfrom(65535)
        count += 1
    except socket.timeout:
        pass
print(count // 3)
PY
}

check_gstreamer() {
    command -v gst-inspect-1.0 >/dev/null || { echo "gst-inspect-1.0 missing"; return 1; }
    gst-inspect-1.0 rtspsrc >/dev/null 2>&1 || { echo "rtspsrc missing (plugins-good)"; return 1; }
    if gst-inspect-1.0 nvv4l2decoder >/dev/null 2>&1; then
        echo "jetson decoder"
    elif gst-inspect-1.0 avdec_h264 >/dev/null 2>&1; then
        echo "software decoder"
    else
        echo "no H.264 decoder (gstreamer1.0-libav)"
        return 1
    fi
}

check_opencv() {
    python3 -c 'import cv2, sys; i = cv2.getBuildInformation(); \
sys.exit(0 if any(l.strip().startswith("GStreamer:") and "YES" in l for l in i.splitlines()) else 1)' \
        2>/dev/null || { echo "OpenCV without GStreamer (pip opencv-python? remove it)"; return 1; }
    echo "OpenCV with GStreamer"
}

do_check() {
    local problems=() notes=() rate gst cv
    if ping -c1 -W1 "$MOTION_HOST" >/dev/null 2>&1; then
        notes+=("motion host $MOTION_HOST up")
    else
        problems+=("motion host $MOTION_HOST down")
    fi
    rate=$(state_rate || echo 0)
    case "$rate" in
        busy) notes+=("port $LOCAL_PORT in use (driver running)") ;;
        0) problems+=("no state on port $LOCAL_PORT: lite3_setup.sh network") ;;
        *) notes+=("state ${rate}/s") ;;
    esac
    if gst=$(check_gstreamer); then notes+=("$gst"); else problems+=("$gst"); fi
    if cv=$(check_opencv); then notes+=("$cv"); else problems+=("$cv"); fi
    if [[ ${#problems[@]} -gt 0 ]]; then
        echo "${problems[*]}"
        return 1
    fi
    echo "${notes[*]}"
}

do_install() {
    $SUDO apt-get update
    $SUDO apt-get install -y --no-install-recommends "${APT_PACKAGES[@]}"
    if [[ "${REALSENSE:-0}" == 1 ]]; then
        $SUDO apt-get install -y --no-install-recommends "ros-$ROS_DISTRO-realsense2-camera"
    fi
    do_network
}

do_network() {
    local me
    me=$(local_ip || true)
    if [[ -z "$me" ]]; then
        echo "no route to the motion host $MOTION_HOST: connect this computer to the robot first"
        return 1
    fi
    if [[ "${1:-}" != "--apply" ]]; then
        cat <<EOF
On the motion host (ssh $MOTION_USER@$MOTION_HOST, Deep Robotics' default password is a single
quote), add this computer to ~/jy_exe/conf/network.toml, keeping the existing entries:

    ips = ['192.168.1.103', '$me']
    ports = [43897, $LOCAL_PORT]

then restart the robot. Or let this script do it (it keeps a backup):

    $0 network --apply
EOF
        return 0
    fi
    echo "adding $me:$LOCAL_PORT to ~/jy_exe/conf/network.toml on $MOTION_HOST"
    ssh "$MOTION_USER@$MOTION_HOST" python3 - "$me" "$LOCAL_PORT" <<'PY'
import os, re, shutil, sys, time
path = os.path.expanduser('~/jy_exe/conf/network.toml')
ip, port = sys.argv[1], int(sys.argv[2])
text = open(path).read()
ips_line = re.search(r'^ips\s*=\s*\[(.*?)\]\s*$', text, re.M)
ports_line = re.search(r'^ports\s*=\s*\[(.*?)\]\s*$', text, re.M)
if not ips_line or not ports_line:
    sys.exit(f'{path}: no "ips = [...]" / "ports = [...]" lines, edit it by hand')
ips = [v.strip().strip('\'"') for v in ips_line.group(1).split(',') if v.strip()]
ports = [int(v) for v in ports_line.group(1).split(',') if v.strip()]
if ip in ips and ports[ips.index(ip)] == port:
    print(f'{ip}:{port} is already listed')
    sys.exit(0)
if ip in ips:
    ports[ips.index(ip)] = port
else:
    ips.append(ip)
    ports.append(port)
backup = f'{path}.{time.strftime("%Y%m%d%H%M%S")}.bak'
shutil.copy(path, backup)
text = text.replace(ips_line.group(0), 'ips = [' + ', '.join(f"'{v}'" for v in ips) + ']')
text = text.replace(ports_line.group(0), 'ports = [' + ', '.join(str(v) for v in ports) + ']')
open(path, 'w').write(text)
print(f'{path} updated (backup {backup}); restart the robot')
PY
}

case "${1:-check}" in
    check) do_check ;;
    install) do_install ;;
    network) shift; do_network "$@" ;;
    *) sed -n '2,13p' "$0"; exit 1 ;;
esac
