#!/usr/bin/env bash
# ============================================================
# Start grayscale camera perception:
#   - AprilTag target -> rt/target_state
#   - bright camera ball -> rt/cam_ball_state
#   - optional fuser -> rt/ball_state
#
# This launches _launch.sh with the grayscale/IR V4L2 defaults so a
# single process owns the GREY node. Pass --with-fuser to also start ball_fuser.py.
# Override the camera node/rate via env: V4L2_DEVICE / V4L2_FOURCC / V4L2_FPS /
# BALL_BRIGHT_MAX_HZ.
# ============================================================
set -e
cd "$(dirname "$0")/../../.."

with_fuser=0
args=()
for arg in "$@"; do
    case "$arg" in
        --with-fuser)
            with_fuser=1
            ;;
        *)
            args+=("$arg")
            ;;
    esac
done

# Grayscale/IR camera defaults (merged here from the former run_apriltag_gray_ball.sh).
gray_args=(
    --camera-profile gray-ir
    --color-backend v4l2
    --v4l2-device "${V4L2_DEVICE:-/dev/video3}"
    --v4l2-fourcc "${V4L2_FOURCC:-GREY}"
    --v4l2-fps "${V4L2_FPS:-30}"
    --ball-bright
    --ball-bright-max-hz "${BALL_BRIGHT_MAX_HZ:-0}"
)

if [[ "$with_fuser" -eq 1 ]]; then
    camera_pid=""
    fuser_pid=""
    shutting_down=0

    cleanup() {
        local status=$?
        if [[ "$shutting_down" -eq 1 ]]; then
            return "$status"
        fi
        shutting_down=1

        echo
        echo "[run_gray.sh] Stopping managed perception processes..."
        for pid in "${camera_pid:-}" "${fuser_pid:-}"; do
            if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
                # Children are started with setsid, so -PID targets the whole
                # process group (bash/conda/python and multiprocessing workers).
                kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
            fi
        done

        sleep 1
        for pid in "${camera_pid:-}" "${fuser_pid:-}"; do
            if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
                kill -KILL "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
            fi
        done

        wait "${camera_pid:-}" 2>/dev/null || true
        wait "${fuser_pid:-}" 2>/dev/null || true
        echo "[run_gray.sh] Done."
        return "$status"
    }
    trap cleanup INT TERM EXIT

    fuser_log="${FUSER_LOG:-/tmp/ball_fuser.log}"
    echo "[run_gray.sh] Starting fuser... log=$fuser_log"
    : > "$fuser_log"
    setsid bash onboard/perception/run_ball_fuser.sh > "$fuser_log" 2>&1 &
    fuser_pid=$!
    echo "[run_gray.sh] fuser pid=$fuser_pid pgid=$fuser_pid"

    echo "[run_gray.sh] Starting AprilTag/bright-ball camera..."
    setsid bash onboard/perception/camera/_launch.sh \
        "${gray_args[@]}" "${args[@]}" &
    camera_pid=$!
    echo "[run_gray.sh] camera pid=$camera_pid pgid=$camera_pid"

    echo "[run_gray.sh] Press Ctrl+C to stop camera and fuser."
    wait -n "$camera_pid" "$fuser_pid"
    exit $?
fi

exec bash onboard/perception/camera/_launch.sh \
    "${gray_args[@]}" "${args[@]}"
