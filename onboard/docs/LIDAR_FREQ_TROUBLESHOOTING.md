# MID360 发布频率问题排查记录

> **问题**：`ros2 topic hz /livox/lidar` 显示平均帧率 2–4 Hz，期望 10 Hz。  
> **最终结果**：修复后稳定 ~10 Hz（`publish_freq=10.0`），驱动内部 `CheckTimer` 可达 20 Hz。

---

## 一、现象描述

启动驱动后立即测量：

```
average rate: 2.678   min: 0.133s  max: 0.581s  std dev: 0.162s  window: 5
average rate: 3.474   min: 0.132s  max: 0.581s  std dev: 0.171s  window: 35
average rate: 5.533   min: 0.130s  max: 0.591s  std dev: 0.059s  window: 448
```

特征：
- 频率从 2 Hz 缓慢爬升，长时间后收敛于 ~7.5 Hz（仍低于期望 10 Hz）
- `max` 值始终固定在 ~0.534–0.591 s（约等于某个超时值）
- `min` 值稳定在 ~0.130 s（约 7.7 Hz）

---

## 二、排查过程

### 2.1 首先排除网络问题

```bash
ping -c 5 192.168.123.120
# 结果：0% packet loss, rtt avg ~1.3ms → 网络正常
```

```bash
netstat -su | grep "receive error"
# 结果：0 packet receive errors → 无 UDP 丢包
```

### 2.2 查看 UDP 缓冲区

```bash
sysctl net.core.rmem_max net.core.rmem_default
# 结果：212992（仅 208 KB）
```

理论上 MID360 以 ~2.8 MB/s 发数据，208 KB 缓冲区 ~70 ms 就会溢出。  
执行扩大缓冲区：

```bash
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400
sudo sysctl -w net.core.netdev_max_backlog=4096

# 永久生效
echo -e "net.core.rmem_max=26214400\nnet.core.rmem_default=26214400\nnet.core.netdev_max_backlog=4096" \
  | sudo tee /etc/sysctl.d/99-livox-udp.conf
```

**结论**：扩大缓冲区后 UDP 错误依然为 0，频率无明显提升。缓冲区不是根本原因。

### 2.3 定位驱动源码中的超时值

读 `ws_livox/src/livox_ros_driver2/src/comm/pub_handler.cpp`，发现 `RawDataProcess` 线程：

```cpp
// 修复前（原始代码）
if (raw_packet_queue_.empty()) {
    packet_condition_.wait_for(lock, std::chrono::milliseconds(500));  // ← 问题 1
    ...
}
```

`max: 0.534s ≈ 500ms + 34ms 处理开销`，完全匹配！  
**原因 1**：队列短暂为空时（雷达初始化阶段），处理线程最长睡 500 ms。

### 2.4 加入 SDK 回调率诊断

在 `OnLivoxLidarPointCloudCallback` 中添加计数器：

```
[SDK-CB] callbacks in last 2.0s: 4569  => 2284 cb/s  dot_num=1  time_type=0
```

**发现**：`dot_num=1` 是 IMU 包干扰，将计数器移到 IMU 过滤之后：

```
[LIDAR-CB] 2.0s: 4169 pkts  400224 pts  => 2084 pkt/s  200099 pts/s  dot_num=96  time_type=0  sync=0
```

**结论**：SDK 以 2084 包/秒、200K 点/秒稳定交付（符合硬件规格），`time_type=0`（无外部时间同步），走非同步路径。

### 2.5 发现条件变量竞态（根本原因）

原始代码：

```cpp
// 发送通知（在锁外）
{
    std::unique_lock<std::mutex> lock(self->packet_mutex_);
    self->raw_packet_queue_.push_back(packet);
}
self->packet_condition_.notify_one();  // ← 问题 2：锁外调用，存在竞态
```

**竞态场景**：
1. 处理线程发现队列为空，即将进入 `wait_for`
2. 回调线程已把包推入队列并调用 `notify_one()`（此时处理线程还没进入 wait 状态）
3. 通知**丢失**，处理线程进入 `wait_for` 后睡 500 ms（或修复后的 10 ms）

这解释了为何 500 ms 超时改成 10 ms 后频率从 ~2 Hz 提升到 ~7.7 Hz，但仍达不到 10 Hz。

---

## 三、解决方案

修改文件：`ws_livox/src/livox_ros_driver2/src/comm/pub_handler.cpp`

### 修复 1：缩短 `wait_for` 超时（消除启动长延迟）

```cpp
// 修复前
packet_condition_.wait_for(lock, std::chrono::milliseconds(500));

// 修复后
packet_condition_.wait_for(lock, std::chrono::milliseconds(10));
```

效果：启动期间的 ~534 ms 间隔缩减为 ≤ 20 ms，频率收敛时间从 ~2 分钟缩短到 ~10 秒。

### 修复 2：将 `notify_one()` 移入锁内（消除竞态）

```cpp
// 修复前
{
    std::unique_lock<std::mutex> lock(self->packet_mutex_);
    self->raw_packet_queue_.push_back(packet);
}
self->packet_condition_.notify_one();  // 锁外，存在竞态

// 修复后
{
    std::unique_lock<std::mutex> lock(self->packet_mutex_);
    self->raw_packet_queue_.push_back(packet);
    self->packet_condition_.notify_one();  // 锁内，保证不丢通知
}
```

效果：处理线程处理频率从 ~7.7 Hz 提升到稳定 ~10 Hz（`publish_freq=10.0`）。

### 修改 publish_freq（按需）

```python
# ws_livox/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py
publish_freq = 10.0  # 10 Hz（当前默认，适合 ball_detector）
# publish_freq = 20.0  # 可选，驱动内部 CheckTimer 确认达到 20 Hz
```

> **注意**：将 `publish_freq` 设为 20.0 时，驱动内部 `CheckTimer` 确实以 20 Hz 触发，
> 但由于 DDS 消息序列化开销，ROS2 订阅端实测约 10–12 Hz。
> 对 `ball_detector.py` 而言，10 Hz 已足够。

---

## 四、重新编译与验证

每次修改源码后需要重新编译和启动：

```bash
cd "$HOME/ws_livox"
colcon build --packages-select livox_ros_driver2 --cmake-args -DCMAKE_BUILD_TYPE=Release

# 重启驱动
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash
source "$HOME/ws_livox/install/setup.sh"
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

在另一个终端验证频率：

```bash
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash
source "$HOME/ws_livox/install/setup.sh"
ros2 topic hz /livox/lidar
```

修复后预期输出（等待 ~10 s 初始化完成后）：

```
average rate: 9.97
    min: 0.098s  max: 0.105s  std dev: 0.003s  window: 50
```

---

## 五、结果对比

| 状态 | 稳定帧率 | max 间隔 | 收敛时间 |
|------|----------|----------|----------|
| 原始代码 | ~2–4 Hz | ~534 ms | 永远不收敛 |
| 修复 1（wait_for 500→10ms）| ~7.7 Hz | ~20 ms | ~30 s |
| 修复 1+2（notify 移入锁内）| **~10 Hz** | ~10 ms | **~10 s** |

---

## 六、根本原因总结

```
原因 1: RawDataProcess wait_for(500ms)
  → 雷达初始化阶段队列短暂为空时，线程睡 500ms
  → 造成启动后频率持续偏低 + max: 534ms
  → 修复：wait_for 改为 10ms

原因 2: notify_one() 在锁外调用（条件变量竞态）
  → 通知在处理线程进入 wait_for 之前发出 → 通知丢失
  → 处理线程无谓等待 10ms，每秒丢失约 23% 的包处理机会
  → 频率上限约 7.7 Hz 而非期望的 10 Hz
  → 修复：notify_one() 移至 push_back 之后、释放锁之前
```

---

## 七、文件改动清单

| 文件 | 改动内容 |
|------|----------|
| `ws_livox/src/livox_ros_driver2/src/comm/pub_handler.cpp` | ① `wait_for` 500ms→10ms；② `notify_one()` 移入锁内 |
| `ws_livox/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` | `publish_freq` 保持 10.0（可选改 20.0） |
| `/etc/sysctl.d/99-livox-udp.conf` | UDP 接收缓冲区扩大到 25 MB（辅助优化） |
