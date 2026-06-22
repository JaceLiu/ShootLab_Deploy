# SDK2 Bridge

`bridge/` 目录用于承载与“低层 SDK2 I/O 和 Python 策略解耦”相关的全部内容，避免将 bridge 逻辑分散到 `common/` 和 `deploy_real/` 下。

## 目录结构

- `cpp_bridge_main.cpp`: C++ bridge，可直接连接 `unitree_sdk2` 的 `rt/lowstate` / `rt/lowcmd`
- `python/bridge_state_dds.py`: Python 侧 `BridgeState` DDS helper
- `python/bridge_cmd_dds.py`: Python 侧 `BridgeCmd` DDS helper
- `python/policy_runtime.py`: Python policy/FSM 运行时
- `python/deploy_policy.py`: Python policy-only 入口
- `python/deploy_bridge_py.py`: 过渡用 Python bridge 原型
- `idl/policy_bridge.idl`: C/C++ bridge DDS schema
- `VALIDATION.md`: 分阶段验证步骤
- `WIFI_zh.md`: Wi‑Fi 排查与快速连接说明

## 运行方式

### 1. 过渡验证版

终端 1：

```bash
python bridge/python/deploy_bridge_py.py
```

终端 2：

```bash
python bridge/python/deploy_policy.py
```

### 2. C++ bridge 版

编译：

```bash
cmake -S bridge -B bridge/build
cmake --build bridge/build -j2
```

运行：

终端 1：

```bash
BRIDGE_NETWORK_INTERFACE=eth0 bridge/build/cpp_bridge_main
```

终端 2：

```bash
python bridge/python/deploy_policy.py
```

## 当前状态

- C++ bridge 已接通 `unitree_sdk2` 的 `LowState` / `LowCmd`
- `BridgeStateMsg` / `BridgeCmdMsg` 已通过 CycloneDDS C codegen 接入 C++ bridge
- Python policy-only 入口已从 `deploy_real/` 抽离到 `bridge/python/`
- 机载 perception DDS 仍保留现有逻辑，由 Python policy 侧继续消费

## 设计原则

- `deploy_real/` 只保留原始真机部署逻辑和共享配置
- bridge 专属逻辑统一放进 `bridge/`
- `common/` 不再放 bridge 专用消息定义，避免“公共模块”不断膨胀
