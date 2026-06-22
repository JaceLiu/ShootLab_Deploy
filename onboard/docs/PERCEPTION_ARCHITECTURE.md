# Perception Architecture

This document describes the runtime architecture for grayscale camera target
detection, camera ball detection, lidar ball detection, and final ball fusion.

## Runtime Pipeline

```mermaid
flowchart TD
    grayCamera["RealSense D435 GREY UVC /dev/video3 @30Hz"] --> cameraOwner["single camera owner: target_ball_detector.py"]
    cameraOwner --> april["AprilTag target detection"]
    cameraOwner --> bright["bright_ball worker: latest frame only"]
    lowstate["rt/lowstate waist joints"] --> april
    lowstate --> bright
    april -->|"target pos b"| targetTopic["rt/target_state"]
    bright -->|"raw far camera ball"| camBall["rt/cam_ball_state"]

    lidar["Livox MID360 reflectivity"] --> lidarDetector["lidar/ball_detector.py"]
    lidarDetector -->|"raw near lidar ball"| lidarBall["rt/lidar_ball_state"]

    camBall --> fuser["ball_fuser.py source priority"]
    lidarBall --> fuser
    fuser --> kf["CenterKalmanFilter"]
    kf --> finalBall["rt/ball_state"]
    finalBall --> policy["policy runtime"]
```

## Topic Ownership

- `rt/target_state` is published by the camera AprilTag service.
- `rt/cam_ball_state` is a raw camera ball observation topic.
- `rt/lidar_ball_state` is a raw lidar ball observation topic.
- `rt/ball_state` is published only by `onboard/perception/ball_fuser.py`.

The final ball source priority is `lidar > camera`. The chosen raw observation
feeds one final `CenterKalmanFilter`, so camera far-range detections and lidar
near-range detections share the same smoothed output used by policy.

## Grayscale Camera Performance Rules

- Keep the V4L2 `GREY` frame as a single-channel image for AprilTag and
  bright-ball detection. Convert to BGR only for preview or recording.
- Use latest-frame semantics for worker handoff. The bright-ball worker queue is
  bounded to one frame and drops stale frames when detection falls behind.
- Keep preview optional and throttled. `--preview-max-hz` limits MJPEG encoding,
  which can otherwise steal time from detection.
- Keep terminal output throttled. `--status-hz` avoids per-frame flush jitter.
- Use `--profile-timing` to print mean/p95 stage timing over
  `--profile-window` frames.

## Launch

Recommended full-system bring-up:

```bash
bash tools/start_tmux_layout.sh
```

The tmux layout keeps camera, lidar, policy, dashboard, and the single final
fuser in separate panes. This is the preferred real-robot structure because
`rt/ball_state` has exactly one publisher.

Run grayscale camera target + bright-ball perception:

```bash
bash onboard/perception/camera/run_gray.sh
```

Run lidar raw ball detection:

```bash
bash onboard/perception/lidar/run.sh --show --base-y-bias 0.05 --dds-topic rt/lidar_ball_state
```

Run the final ball fuser separately:

```bash
bash onboard/perception/run_ball_fuser.sh
```

Do not start more than one fuser. `run_gray.sh --with-fuser` remains useful for
camera-only debugging, but do not use it together with the tmux fuser pane or a
manual `run_ball_fuser.sh`.

Monitor the final policy-facing topic:

```bash
python tools/check_ball_state.py
```

Profiling example:

```bash
bash onboard/perception/camera/run_gray.sh \
    --profile-timing \
    --profile-window 30 \
    --preview-max-hz 10 \
    --status-hz 4
```

Hardware validation should confirm `rt/target_state`, `rt/cam_ball_state`,
`rt/lidar_ball_state`, and final `rt/ball_state` are all published at the
expected rates before policy is started.
