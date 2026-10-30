#!/usr/bin/env bash
# The most accurate map: record the mapping drive, then rebuild it with Cartographer at full quality.
# docs/mapping.md has the whole method (calibration first, how to drive, how to check the result).
#
#   bash ~/slam-robot/scripts/map_offline.sh record cse_floor   # while mapping (robot running): Ctrl+C to stop
#   bash ~/slam-robot/scripts/map_offline.sh build  cse_floor   # afterwards: robot stopped, takes 10-30 min
#   bash ~/slam-robot/scripts/map_offline.sh list
#
# record  saves /scan /odom /imu /tf_static and the robot model (URDF) to ~/bags/<name>/
# build   runs cartographer_offline_node with config/cartographer_mapping_hq.lua and writes
#         ~/maps/<name>.pbstream (for slam_mode:=localization) + ~/maps/<name>.pgm/.yaml (picture, report)
set -eo pipefail

BAGS=${BAGS:-$HOME/bags}
MAPS=${MAPS:-$HOME/maps}
cmd=${1:-}
name=${2:-}

die() { echo "ERROR: $*" >&2; exit 1; }
need_name() { [[ "$name" =~ ^[A-Za-z0-9_-]+$ ]] || die "give a map name (letters, digits, _ and - only), e.g. cse_floor"; }

ros_env() {
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "$HOME/ros2_ws/install/setup.bash"
    fi
}

config_dir() {
    local share
    share=$(ros2 pkg prefix lidar_robot 2>/dev/null)/share/lidar_robot/config
    [ -f "$share/cartographer_mapping_hq.lua" ] || share="$(cd "$(dirname "$0")/.." && pwd)/ros2_ws/src/lidar_robot/config"
    [ -f "$share/cartographer_mapping_hq.lua" ] || die "cartographer_mapping_hq.lua not found: rebuild the workspace (colcon build)"
    echo "$share"
}

case "$cmd" in
record)
    need_name
    ros_env
    out="$BAGS/$name"
    [ -e "$out" ] && die "$out exists already: pick another name or delete it (rm -r $out)"
    mkdir -p "$BAGS"
    ros2 topic list 2>/dev/null | grep -qx /scan || die "no /scan topic: start the robot first (sudo systemctl start robot)"
    urdf_tmp=$(mktemp)
    if ros2 param get /robot_state_publisher robot_description --hide-type > "$urdf_tmp" 2>/dev/null \
            && grep -q '<robot' "$urdf_tmp"; then
        echo "robot model saved (the map builder needs where the LiDAR sits on the robot)"
    else
        rm -f "$urdf_tmp"; urdf_tmp=""
        echo "no robot_state_publisher (urdf:=false?): the LiDAR position comes from /tf_static in the recording"
    fi
    echo "Recording to $out  (drive the mapping route now; Ctrl+C when back at the start)"
    echo "Tips: 0.15 m/s, slow turns, come back to where you started, keep people out of the way."
    trap 'echo; echo "stopping…"' INT
    ros2 bag record -o "$out" /scan /odom /imu /tf_static || true
    trap - INT
    [ -d "$out" ] || die "nothing recorded"
    [ -n "$urdf_tmp" ] && mv "$urdf_tmp" "$out/robot.urdf"
    du -sh "$out" | awk '{print "recorded " $1}'
    ros2 bag info "$out" 2>/dev/null | grep -E "Duration|Topic: /(scan|odom)" || true
    echo "Next (robot can stay on, but stop the service for speed):"
    echo "  sudo systemctl stop robot && bash $0 build $name"
    ;;
build)
    need_name
    ros_env
    bag="$BAGS/$name"
    [ -d "$bag" ] || die "no recording $bag (record it first)"
    if systemctl is-active --quiet robot 2>/dev/null; then
        echo "NOTE: the robot service is running: the build will be slower and the Pi hotter."
        echo "      Stop it first for the best result: sudo systemctl stop robot"
    fi
    cfg=$(config_dir)
    mkdir -p "$MAPS"
    state="$MAPS/$name.pbstream"
    args=(-configuration_directory "$cfg" -configuration_basenames cartographer_mapping_hq.lua
          -bag_filenames "$bag" -save_state_filename "$state")
    if [ -f "$bag/robot.urdf" ]; then
        args+=(-urdf_filenames "$bag/robot.urdf" -use_bag_transforms=false)
    fi
    echo "Building $state from $bag with the high-quality settings (this takes a while) …"
    start=$(date +%s)
    # the recording's topics are already /scan and /odom, the names Cartographer reads
    ros2 run cartographer_ros cartographer_offline_node "${args[@]}" 2>&1 \
        | grep -vE "^\[INFO\].*(Inserted submap|Remaining work items)" || true
    [ -s "$state" ] || die "no map was written (see the messages above)"
    ros2 run cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename "$state" \
        -map_filestem "$MAPS/$name" -resolution 0.05 >/dev/null 2>&1 \
        || echo "(could not write the .pgm picture; the .pbstream is what the robot needs)"
    echo "Done in $(( ($(date +%s) - start) / 60 )) min:"
    ls -lh "$state" "$MAPS/$name".pgm "$MAPS/$name".yaml 2>/dev/null || true
    echo
    echo "Use it: put this line in ~/robot.env (nano ~/robot.env), then sudo systemctl restart robot"
    echo "  ROBOT_ARGS=esp32_port:=/dev/ttyAMA0 lidar_port:=/dev/ttyUSB0 slam_mode:=localization map:=$state"
    echo "Then check it: app → Map → Measure three known lengths (docs/mapping.md, step 6)."
    ;;
list)
    ls -1d "$BAGS"/*/ 2>/dev/null | sed 's|/$||' | while read -r b; do
        echo "$(basename "$b")  $(du -sh "$b" | cut -f1)  $( [ -f "$MAPS/$(basename "$b").pbstream" ] && echo "-> map built" )"
    done
    ;;
*)
    sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
