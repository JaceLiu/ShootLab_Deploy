# SDK2 Bridge

This directory contains the minimal bridge scaffold for Orin-friendly real-robot deployment:

- `cpp_bridge_main.cpp`: C++ low-level I/O bridge skeleton with real `unitree_sdk2` LowState/LowCmd wiring
- `idl/policy_bridge.idl`: shared DDS schema for Python policy <-> C++ bridge
- `CMakeLists.txt`: minimal CMake scaffold
- `python/deploy_bridge_py.py`: Python SDK2 bridge prototype for immediate end-to-end validation
- `python/deploy_policy.py`: Python policy-only entrypoint
- `README_zh.md`: Chinese overview
- `VALIDATION.md`: staged validation steps

## Responsibilities

### C++ bridge
- Subscribe `rt/lowstate` with official `unitree_sdk2`
- Publish `rt/policy_bridge_state`
- Subscribe `rt/policy_bridge_cmd`
- Publish `rt/lowcmd`
- Fall back to damping if the Python command stream is stale

### Python policy node
- Subscribe `rt/policy_bridge_state`
- Run FSM + policy inference
- Publish `rt/policy_bridge_cmd`
- Continue consuming `rt/ball_state` / `rt/target_state` unchanged

## Topic contract

### `rt/policy_bridge_state`
- `tick`
- `q[29]`
- `dq[29]`
- `imu_quat_wxyz[4]`
- `imu_gyro[3]`
- `remote_raw[24]`

### `rt/policy_bridge_cmd`
- `q_des[29]`
- `kp[29]`
- `kd[29]`
- `seq`
- `timestamp_us`
- `request_damping`
- `exit_requested`

## Expected integration steps

1. Build the C++ bridge against `unitree_sdk2`. `bridge/CMakeLists.txt` already:
   - discovers `unitree_sdk2`
   - discovers `idlc`
   - generates C bridge DDS types when `idlc` is available
2. Run the Python policy process with:

```bash
python bridge/python/deploy_policy.py
```

4. Build the C++ bridge:

```bash
cmake -S bridge -B bridge/build
cmake --build bridge/build -j2
```

4. For immediate testing before switching to the C++ bridge runtime, run the Python bridge prototype:

```bash
python bridge/python/deploy_bridge_py.py
```

5. Replace the prototype with `bridge/build/cpp_bridge_main` on the robot computer.

## Current implementation status

- `LowState` subscription: wired in C++ via official `unitree_sdk2`
- `LowCmd` publication: wired in C++ via official `unitree_sdk2`
- `BridgeStateMsg` / `BridgeCmdMsg` schema: defined
- Python bridge prototype: runnable now
- C++ bridge transport: wired with CycloneDDS C API and generated bridge message types

## Current environment note

On this machine, `idlc` is available but the `idlc -l c++` generator plugin is missing. `CMakeLists.txt` therefore uses `idlc -l c` and the C bridge DDS types from `bridge/build/generated/` to keep the bridge fully buildable.

## Safety model

- The C++ bridge owns the final `LowCmd` publication.
- If Python commands stop arriving within the configured timeout, send damping.
- `request_damping` or `exit_requested` from Python should also trigger damping immediately.

## Initial validation scope

Only validate these modes first:
- `PassiveMode`
- `FixedPose`
- `LocoMode`

Keep heavier policies disabled until mode-switch latency and command freshness are stable.
