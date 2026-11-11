#!/usr/bin/env bash
#
# Copyright 2026 Sanaullah
# SPDX-License-Identifier: Apache-2.0
#
# One-shot bring-up for FFW-SG2 navigation in Gazebo simulation.
# Starts each component in order, waits until it's actually ready, then
# starts the next. A single Ctrl+C shuts everything down cleanly.
#
# Run INSIDE the ai_worker container:
#     ./run_nav_sim.sh              # SLAM mapping (default)
#     ./run_nav_sim.sh map          # use the saved map + AMCL
#     ./run_nav_sim.sh slam teleop  # SLAM + also open the GUI teleop
#     ./run_nav_sim.sh map teleop   # saved map + GUI teleop
#
MODE="${1:-slam}"            # slam | map
TELEOP="${2:-}"              # teleop | (empty)


# --- gz-transport loopback fix (server <-> gui/create discovery) ---
export GZ_IP=127.0.0.1

# --- make sure ROS is sourced even when run non-interactively ---
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash

TELEOP_SCRIPT=/root/ros2_ws/src/ai_worker/ffw_teleop/ffw_teleop/mobile_gui_teleop.py
LOGDIR=/tmp/nav_sim_logs
mkdir -p "$LOGDIR"
PIDS=()

# ------------------------------------------------------------------ #
log() { echo -e "\033[1;36m[bringup]\033[0m $*"; }
warn() { echo -e "\033[1;33m[bringup]\033[0m $*"; }

cleanup() {
    echo
    log "shutting down (this takes a few seconds)..."
    # ask each launch to stop gracefully, newest first
    for ((i=${#PIDS[@]}-1; i>=0; i--)); do
        kill -INT "${PIDS[$i]}" 2>/dev/null
    done
    sleep 3
    # hard-kill anything that lingers (gz is the usual offender)
    pkill -9 -f "gz sim"            2>/dev/null
    pkill -9 -f "ffw_sg2_follower"  2>/dev/null
    pkill -9 -f "navigation.launch" 2>/dev/null
    pkill -9 -f parameter_bridge    2>/dev/null
    pkill -9 -f rviz2               2>/dev/null
    pkill -9 -f mobile_gui_teleop   2>/dev/null
    log "all components stopped."
    exit 0
}
trap cleanup INT TERM

# clear any stale sim processes from a previous run so we start clean
clean_stale() {
    if pgrep -f "gz sim" >/dev/null 2>&1 || pgrep -f navigation.launch >/dev/null 2>&1; then
        warn "found running sim/nav processes from a previous session - clearing them."
        pkill -9 -f "gz sim" 2>/dev/null
        pkill -9 -f "ffw_sg2_follower" 2>/dev/null
        pkill -9 -f "navigation.launch" 2>/dev/null
        pkill -9 -f parameter_bridge 2>/dev/null
        pkill -9 -f rviz2 2>/dev/null
        sleep 2
    fi
}

wait_for_controllers() {
    log "waiting for Gazebo + the robot's controllers to come up..."
    for _ in $(seq 1 60); do
        if ros2 control list_controllers 2>/dev/null \
                | grep -qE "swerve_drive_controller.*active"; then
            log "controllers active - robot is spawned."
            return 0
        fi
        sleep 2
    done
    warn "controllers not active after ~2 min; continuing anyway (check $LOGDIR/gazebo.log)."
}

wait_for_topic() {  # $1 = topic, $2 = timeout seconds
    local topic="$1" t="${2:-30}"
    log "waiting for $topic ..."
    if timeout "$t" ros2 topic echo "$topic" --once >/dev/null 2>&1; then
        log "$topic is publishing."
    else
        warn "no message on $topic within ${t}s; continuing (check the logs)."
    fi
}

# ------------------------------------------------------------------ #
clean_stale

# [1] Gazebo + SG2 + controllers + lidar bridge + sensor RViz
log "[1/3] launching Gazebo (FFW-SG2)..."
ros2 launch ffw_bringup ffw_sg2_follower_ai_gazebo.launch.py \
    > "$LOGDIR/gazebo.log" 2>&1 &
PIDS+=($!)
wait_for_controllers
wait_for_topic /scan 25

# [2] Nav2 (+ SLAM or saved-map AMCL) + navigation RViz
if [ "$MODE" = "map" ]; then
    USE_SLAM=false
    log "[2/3] launching Nav2 with SAVED MAP + AMCL..."
else
    USE_SLAM=true
    log "[2/3] launching Nav2 with SLAM mapping..."
fi
ros2 launch ffw_navigation navigation.launch.py \
    ${MAP_YAML:+map:=$MAP_YAML} \
    use_slam:=$USE_SLAM use_sim_time:=true rviz:=true \
    > "$LOGDIR/nav2.log" 2>&1 &
PIDS+=($!)
if [ "$USE_SLAM" = "true" ]; then
    wait_for_topic /map 40
else
    log "set the initial pose in RViz with '2D Pose Estimate' so AMCL can localize."
fi

# [3] optional GUI teleop
if [ "$TELEOP" = "teleop" ]; then
    if [ -f "$TELEOP_SCRIPT" ]; then
        log "[3/3] launching GUI teleop..."
        python3 "$TELEOP_SCRIPT" > "$LOGDIR/teleop.log" 2>&1 &
        PIDS+=($!)
        warn "don't send a Nav2 goal while teleoping - both publish /cmd_vel."
    else
        warn "teleop script not found at $TELEOP_SCRIPT - skipping."
    fi
fi

echo
log "everything is up."
log "  logs:  $LOGDIR/{gazebo,nav2,teleop}.log"
if [ "$USE_SLAM" = "true" ]; then
    log "  drive (teleop or RViz goals) to build the map, then save it with:"
    log "    ros2 run nav2_map_server map_saver_cli -f \\"
    log "      /root/ros2_ws/src/ai_worker/ffw_navigation/maps/map"
else
    log "  send goals in RViz with '2D Goal Pose' (or 'Nav2 Goal')."
fi
log "  press Ctrl+C HERE to shut the whole stack down."
echo

# stay alive until Ctrl+C; cleanup() handles teardown
wait
