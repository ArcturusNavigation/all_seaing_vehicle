#!/usr/bin/env bash
# Cleanly stop all ROS 2 processes of the current user, escalating only when needed,
# so no nodes are left hanging between runs (stale nodes duplicate topics/TF and
# hold serial ports).
#
# Order:
#   1. SIGINT to `ros2 launch` / `ros2 bag` processes: launch forwards the shutdown to
#      its nodes, which run their normal cleanup (e.g. flushing logs).
#   2. SIGINT to any remaining node process (anything started with `--ros-args`, or
#      `ros2 run`).
#   3. SIGTERM, then SIGKILL, to whatever is still alive after each grace period.
# Nodes must not rely on step 1/2 for correctness: the run logger also recovers files
# left behind by processes killed with SIGKILL.
#
# Usage: kill_ros_nodes.sh [-t SECONDS] [--sim] [--daemon] [-n]
#   -t SECONDS  grace period for each step (default 5)
#   --sim       also stop Gazebo (gz sim) and RViz
#   --daemon    also stop the ros2 CLI daemon
#   -n          dry run: only list what would be signalled
#   -h          show this help

set -u

GRACE=5
KILL_SIM=false
STOP_DAEMON=false
DRY_RUN=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        -t) GRACE="$2"; shift 2 ;;
        --sim) KILL_SIM=true; shift ;;
        --daemon) STOP_DAEMON=true; shift ;;
        -n) DRY_RUN=true; shift ;;
        -h|--help) sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done

# This script and its ancestors (e.g. a `bash -c "...ros2 launch...; kill_ros_nodes.sh"`
# wrapper) must never be signalled, even if their command line matches a pattern
EXCLUDE=" "
p=$$
while [[ -n "$p" && "$p" -gt 1 ]]; do
    EXCLUDE+="$p "
    p=$(ps -o ppid= -p "$p" | tr -d ' ')
done

# PIDs (of this user, excluding EXCLUDE) whose command line matches an ERE
pids_matching() {
    local pid
    for pid in $(pgrep -u "$USER" -f -- "$1"); do
        [[ "$EXCLUDE" == *" $pid "* ]] || echo "$pid"
    done
}

launch_pids() { pids_matching 'ros2 (launch|bag) '; }
node_pids() {
    {
        pids_matching ' --ros-args( |$)'
        pids_matching 'ros2 run '
        pids_matching 'ros2 (launch|bag) '
        if $KILL_SIM; then
            pids_matching 'gz sim|ruby.*gz|rviz2'
        fi
    } | sort -u
}

# Send a signal to a list of PIDs and wait until they exit or the grace period ends
signal_and_wait() {
    local sig="$1"; shift
    local pids=("$@")
    [[ ${#pids[@]} -eq 0 ]] && return
    echo "Sending SIG$sig to ${#pids[@]} process(es):"
    ps -o pid=,args= -p "$(IFS=,; echo "${pids[*]}")" | cut -c1-120 | sed 's/^/  /'
    $DRY_RUN && return
    kill -s "$sig" "${pids[@]}" 2>/dev/null
    for ((i = 0; i < GRACE * 10; i++)); do
        local alive=0
        for p in "${pids[@]}"; do kill -0 "$p" 2>/dev/null && alive=1 && break; done
        [[ $alive -eq 0 ]] && return
        sleep 0.1
    done
}

mapfile -t L < <(launch_pids)
signal_and_wait INT "${L[@]}"

for sig in INT TERM KILL; do
    mapfile -t N < <(node_pids)
    [[ ${#N[@]} -eq 0 ]] && break
    signal_and_wait "$sig" "${N[@]}"
done

# Persist the log buffers of any node that didn't shut down cleanly, and end the run so
# the next launch logs into a new run folder (see all_seaing_common/run_logger.py)
if ! $DRY_RUN; then
    python3 -m all_seaing_common.run_logger --recover --end-run 2>/dev/null \
        || echo "WARNING: run logger recovery failed (is the workspace sourced?)" >&2
fi

if $STOP_DAEMON && ! $DRY_RUN; then
    ros2 daemon stop >/dev/null 2>&1 && echo "Stopped ros2 daemon"
fi

mapfile -t LEFT < <(node_pids)
if [[ ${#LEFT[@]} -gt 0 ]] && ! $DRY_RUN; then
    echo "WARNING: ${#LEFT[@]} process(es) still running (owned by another user or unkillable):" >&2
    ps -o pid=,args= -p "$(IFS=,; echo "${LEFT[*]}")" | cut -c1-120 >&2
    exit 1
fi
echo "All ROS 2 processes stopped."
