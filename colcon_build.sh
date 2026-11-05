#!/bin/bash
# Workspace bauen – immer über dieses Skript, nicht mit "colcon build" in einem normalen Terminal.
#
#   ./colcon_build.sh                                   # alles
#   ./colcon_build.sh --packages-select slop_base       # weitere Argumente gehen an colcon build
#
# Warum: In normalen Terminals stehen Dinge in der Umgebung, die den Build verderben:
#   - ~/ros2_ws (aus ~/.bashrc): colcon trägt jeden beim Bauen gesourcten Workspace fest als Unterlage in
#     install/setup.bash ein → slop-bot_ws würde danach immer die Pakete von ros2_ws mitladen.
#   - ~/ethercat_venv/bin vorn im PATH (VS Code-Terminal): CMake nimmt dessen Python ohne catkin_pkg.
#   - /opt/MVS/lib/aarch64 im LD_LIBRARY_PATH: alte libusb des Hikrobot-SDK.
# Deshalb läuft colcon hier in einer leeren Umgebung, nur mit /opt/ros/humble.
#
# Außerdem legt das Skript die lokalen Anpassungen der Fremdpakete an, falls sie fehlen (z. B. nach einem
# frischen "git submodule update --init --recursive"): package.xml des Livox-Treibers, der FAST-LIO-Patch und
# COLCON_IGNORE in realsense-ros.
set -e
WS="$(cd "$(dirname "$0")" && pwd)"
cd "$WS"

# livox_ros_driver2 bringt nur package_ROS2.xml mit (package.xml ist dort per .gitignore ausgeschlossen,
# sein build.sh kopiert sie sonst – das build.sh NICHT benutzen, es löscht build/ und install/ des Workspace).
LIVOX="src/third_party/livox_ros_driver2"
if [ -f "$LIVOX/package_ROS2.xml" ] && [ ! -f "$LIVOX/package.xml" ]; then
    sed 's|\(.*<depend>libpcl-all-dev</depend>\)|\1\n  <depend>livox_sdk2</depend>|' \
        "$LIVOX/package_ROS2.xml" > "$LIVOX/package.xml"
    echo "colcon_build: $LIVOX/package.xml aus package_ROS2.xml angelegt (+ Abhängigkeit livox_sdk2)"
fi

# FAST-LIO: lokale Änderungen aus src/third_party/patches/fast_lio.patch (pcl_ros → tf2_ros, C++17) anwenden, falls
# sie noch fehlen (frisch geklontes Submodule). Schon angewendet → nichts tun.
FASTLIO="src/third_party/FAST_LIO"
PATCH="$WS/src/third_party/patches/fast_lio.patch"
if [ -f "$FASTLIO/package.xml" ] && [ -f "$PATCH" ] && git -C "$FASTLIO" apply --check "$PATCH" 2>/dev/null; then
    git -C "$FASTLIO" apply "$PATCH"
    echo "colcon_build: $PATCH auf $FASTLIO angewendet"
fi

# Nicht gebrauchte Teile von realsense-ros überspringen (Beschreibung/URDF und RGBD-Plugin)
for p in realsense2_description realsense2_rgbd_plugin; do
    d="src/third_party/realsense-ros/$p"
    if [ -d "$d" ] && [ ! -f "$d/COLCON_IGNORE" ]; then
        touch "$d/COLCON_IGNORE"
        echo "colcon_build: $d/COLCON_IGNORE angelegt"
    fi
done

exec env -i HOME="$HOME" USER="$USER" LOGNAME="${LOGNAME:-$USER}" TERM="${TERM:-xterm}" LANG=C.UTF-8 \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    bash -c 'source /opt/ros/humble/setup.bash && exec colcon build "$@"' colcon_build "$@"
