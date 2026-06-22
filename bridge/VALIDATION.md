# SDK2 Bridge Validation

## Stage 1: Bridge connectivity

Goal: confirm the bridge can read `rt/lowstate` and the Python policy node can receive `rt/policy_bridge_state`.

Checks:
- `python bridge/python/deploy_policy.py` prints `Successfully connected to bridge state.`
- `remote_raw` changes when the physical remote buttons are pressed
- `q` / `dq` / IMU values update continuously

For immediate validation before switching to the C++ bridge, use:

```bash
python bridge/python/deploy_bridge_py.py
```

## Stage 2: Passive and FixedPose

Goal: validate safe command ownership and fallback behavior.

Procedure:
1. Start bridge (`bridge/python/deploy_bridge_py.py` first, then `bridge/build/cpp_bridge_main`)
2. Start Python policy node
3. Press `Start` to enter `FixedPose`
4. Press `Select` or `F1`

Expected:
- `Start` switches to `FixedPose`
- `Select` / `F1` immediately produce damping/passive behavior
- If the Python node is killed, the bridge times out and sends damping automatically

## Stage 3: LocoMode

Goal: verify that removing Python from low-level I/O reduces switching delay and runtime jitter.

Procedure:
1. Repeat `FixedPose -> LocoMode` transitions
2. Record timing for:
   - button edge to policy transition
   - bridge command freshness
   - lowcmd send loop timing

Expected:
- `R1+A` enters `LocoMode` without multi-second lag
- `Select` stops quickly
- No prolonged stale-command gaps

## Stage 4: Optional heavier policies

Only after Stage 3 is stable, re-enable:
- `FreeKick`
- `BeyondMimic`
- `BeyondMimicMJ`
- `AMP`

These should be validated one at a time, with `PassiveMode` / damping fallback confirmed after each run.
