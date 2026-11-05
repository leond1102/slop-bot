#!/bin/bash
# Alle hängengebliebenen Slop-Bot-Prozesse beenden (Lidar-Treiber, Kamera, FAST-LIO, RViz, Antrieb, Reste von ros2 launch ...).
#
#   ./stop_all.sh            # zeigt, was beendet würde, fragt nach, beendet dann
#   ./stop_all.sh --list     # nur anzeigen
#   ./stop_all.sh --yes      # ohne Rückfrage beenden
#
# Beendet NUR Prozesse dieses Nutzers, deren Programm aus diesem Workspace (install/) oder aus den ROS-Paketen kommt, die der
# Roboter startet (twist_mux, robot_state_publisher, robot_localization, rviz2, joint_state_publisher). Nichts anderes (kein
# VS Code, keine fremden ROS-Programme, kein pkill nach Namen). Reihenfolge: Sensoren, LIO, RViz, Tastatur -> Sicherheit,
# Modell, EKF -> zuletzt der Antrieb (Endstufen aus) und die ros2-Wrapper.
# Jeder Schritt: SIGINT (wie Strg+C), nach 8 s SIGTERM, nach 3 s SIGKILL.
#
# Echte Zombies ("<defunct>") lassen sich nicht beenden, sie verschwinden, wenn ihr Elternprozess endet; das Script beendet
# deshalb deren Eltern, sofern sie dazugehören, und meldet übrig gebliebene.
# Programme, die als root laufen (sudo-Wrapper von with_netcaps), kann es nicht beenden; sie enden mit ihrem Kindprozess.
set -u
ME=$(id -un)
WS="$(cd "$(dirname "$0")" && pwd)"
MODE=ask
for a in "$@"; do
    case "$a" in
        --list) MODE=list ;;
        --yes|-y) MODE=yes ;;
        -h|--help) sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "Unbekannte Option: $a (siehe --help)"; exit 1 ;;
    esac
done

# Regex je Stufe (gegen die Kommandozeile). STOP_ALL_EXTRA: zusätzliches Muster für Stufe 1 (nur für Tests).
ROS='/opt/ros/humble/lib/'
INSTALL="$WS/install/"
STAGE2="(${INSTALL}(livox_ros_driver2|fast_lio|realsense2_camera|slop_lio|slop_teleop|slop_localization|slop_calibration)/|${ROS}(rviz2|joint_state_publisher)|fastlio_mapping${STOP_ALL_EXTRA:+|$STOP_ALL_EXTRA})"
STAGE3="(${INSTALL}slop_safety/|${ROS}(twist_mux|robot_state_publisher|robot_localization)/)"
STAGE4="(${INSTALL}slop_base/|bin/ros2 (launch|run) (slop_|tf2_ros|livox|realsense)|with_netcaps)"

select_pids() {   # select_pids <regex>: PIDs dieses Nutzers (außer diesem Script, dem Elternprozess und dem awk, das die Muster trägt)
    ps -eo pid=,user=,args= | awk -v me="$ME" -v self="$$" -v par="$PPID" -v re="$1" \
        '$2 == me && $1 != self && $1 != par && $0 !~ /-v re=/ && $0 !~ /stop_all\.sh/ && substr($0, index($0, $3)) ~ re {print $1}'
}

show() {   # show <titel> <pids...>
    local title=$1; shift
    [ $# -eq 0 ] && return
    echo "$title"
    ps -o pid=,stat=,etime=,args= -p "$(IFS=,; echo "$*")" 2>/dev/null | cut -c1-150 | sed 's/^/  /'
}

mapfile -t P2 < <(select_pids "$STAGE2")
mapfile -t P3 < <(select_pids "$STAGE3")
mapfile -t P4 < <(select_pids "$STAGE4")
ALL=("${P2[@]}" "${P3[@]}" "${P4[@]}")

zombies=$(ps -eo pid=,ppid=,stat=,args= | awk '$3 ~ /^Z/ {print "  PID " $1 " (Eltern " $2 "): " substr($0, index($0, $4))}')

if [ "${#ALL[@]}" -eq 0 ]; then
    echo "Nichts zu beenden: es läuft kein Slop-Bot-Prozess."
    [ -n "$zombies" ] && { echo "Zombies (verschwinden mit ihrem Elternprozess):"; echo "$zombies"; }
    exit 0
fi

show "1) Sensoren, LIO, RViz, Tastatur:" "${P2[@]}"
show "2) Sicherheit, Modell, EKF:" "${P3[@]}"
show "3) Antrieb und ros2-Wrapper:" "${P4[@]}"
[ -n "$zombies" ] && { echo "Zombies (verschwinden mit ihrem Elternprozess):"; echo "$zombies"; }
[ "$MODE" = list ] && exit 0
if [ "$MODE" = ask ]; then
    read -r -p "Diese ${#ALL[@]} Prozesse beenden? [j/N] " answer
    [[ "$answer" =~ ^[jJyY]$ ]] || { echo "Abgebrochen."; exit 1; }
fi

stop() {   # stop <pid>: SIGINT, SIGTERM, SIGKILL mit Wartezeit; Prozess selbst und seine Kinder
    local pid=$1 i
    kill -0 "$pid" 2>/dev/null || return 0
    kill -INT "$pid" 2>/dev/null
    for i in $(seq 1 16); do kill -0 "$pid" 2>/dev/null || return 0; sleep 0.5; done
    echo "  $pid reagiert nicht auf SIGINT – SIGTERM"
    kill -TERM "$pid" 2>/dev/null
    for i in $(seq 1 6); do kill -0 "$pid" 2>/dev/null || return 0; sleep 0.5; done
    echo "  $pid reagiert nicht auf SIGTERM – SIGKILL"
    kill -KILL "$pid" 2>/dev/null
}

stop_stage() {   # stop_stage <regex>: Stufe frisch auswählen (Kinder sind inzwischen evtl. schon weg) und beenden
    local pids pid
    mapfile -t pids < <(select_pids "$1")
    for pid in "${pids[@]}"; do [ -n "$pid" ] && stop "$pid"; done
}

echo "Beende ..."
stop_stage "$STAGE2"
stop_stage "$STAGE3"
stop_stage "$STAGE4"

left=$(for p in $(select_pids "($STAGE2|$STAGE3|$STAGE4)"); do ps -o pid=,args= -p "$p"; done | cut -c1-150)
if [ -n "$left" ]; then
    echo "Noch übrig (vermutlich root-Prozesse oder Zombies):"; echo "$left" | sed 's/^/  /'
    exit 1
fi
echo "Alles beendet."
