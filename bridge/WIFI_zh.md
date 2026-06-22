# G1 Wi-Fi 排查与快速连接

本文档用于说明：

- 如何查看 G1 当前能扫描到的 Wi‑Fi
- 如何快速切换到指定 Wi‑Fi
- 如何验证主机与 G1 是否已经处于同一个可互访的无线网络
- 在 bridge 架构下，哪些流量该走 `wlan0`，哪些仍然应该走 `eth0`

## 1. 最短命令清单

### 在 G1 上连接手机热点

```bash
sudo nmcli device wifi connect "Xiaomi_14" password "<热点密码>" ifname wlan0
ip addr show wlan0
```

### 在主机上 SSH 到 G1

假设上一步看到 G1 的 `wlan0` 地址是 `10.79.215.11`：

```bash
ssh unitree@10.79.215.11
```

### 在 G1 上启动 bridge

终端 1：

```bash
# cb
cd /home/unitree/yichao/RoboMimic_Deploy
BRIDGE_NETWORK_INTERFACE=eth0 bridge/build/cpp_bridge_main
```

终端 2：

```bash
# pp
cd /home/unitree/yichao/RoboMimic_Deploy
python bridge/python/deploy_policy.py
```

### 如果 C++ bridge 还没编译过

```bash
cd /home/unitree/yichao/RoboMimic_Deploy
cmake -S bridge -B bridge/build
cmake --build bridge/build -j2
```

## 2. 先理解两条网络链路

在当前 bridge 部署方案下，通常存在两条独立链路：

1. **主机 <-> G1 机载电脑**
   - 可走 `wlan0`
   - 主要用于 `ssh`、远程启动 bridge / policy 进程

2. **G1 机载电脑 <-> 机器人底层控制器**
   - 通常仍走 `eth0`
   - 用于 `rt/lowstate` / `rt/lowcmd`

因此，即使你希望“通过 Wi‑Fi 远程登录 G1”，bridge 本身大多数情况下仍应继续使用：

```bash
BRIDGE_NETWORK_INTERFACE=eth0 bridge/build/cpp_bridge_main
```

不要轻易把 bridge 的控制网卡改成 `wlan0`。

## 3. 查看当前活动连接

在 G1 上执行：

```bash
nmcli connection show --active
```

如果你只想快速查看 `wlan0` 当前正在连接的 Wi‑Fi 名称，推荐直接用：

```bash
nmcli -g GENERAL.CONNECTION device show wlan0
```

也可以用下面这条只筛出当前正在使用的 SSID：

```bash
nmcli -t -f ACTIVE,SSID dev wifi | grep '^yes:'
```

查看接口地址：

```bash
ifconfig
```

或者：

```bash
ip addr show wlan0
ip addr show eth0
```

重点关注：

- `wlan0`：G1 当前无线地址
- `eth0`：G1 与机器人底层控制器的有线地址（通常是 `192.168.123.x`）

## 4. 扫描当前可见 Wi‑Fi

在 G1 上：

```bash
sudo nmcli device wifi rescan ifname wlan0
nmcli -f IN-USE,SSID,BSSID,SIGNAL,SECURITY device wifi list ifname wlan0
```

说明：

- `IN-USE` 列里 `*` 表示当前正在连接的 Wi‑Fi
- `SSID` 是热点名
- `SECURITY` 可判断是否需要密码

## 5. 快速连接到 Wi‑Fi.HK via HKU（无密码）

如果该网络允许直接接入：

```bash
sudo nmcli device wifi connect "Wi-Fi.HK via HKU" ifname wlan0
```

连接后检查：

```bash
nmcli connection show --active
ip addr show wlan0
```

## 6. 快速连接到 Xiaomi_14（手机热点）

先扫描确认 SSID 名称确实是 `Xiaomi_14`，然后：

```bash
sudo nmcli device wifi connect "Xiaomi_14" password "<热点密码>" ifname wlan0
```

说明：

- 请把 `<热点密码>` 替换成你当前手机热点的实际密码
- 不建议把个人热点密码直接写入仓库文档

连接后检查：

```bash
nmcli connection show --active
ip addr show wlan0
```

## 7. 如果提示没有权限切换网络

例如：

```text
Not authorized to control networking
```

说明当前用户没有权限修改 NetworkManager 配置。

可使用：

```bash
sudo nmcli device wifi connect "<SSID>" password "<PASSWORD>" ifname wlan0
```

## 7. 如何确认主机和 G1 是否真的能通过 Wi‑Fi 互访

假设在 G1 上看到：

```text
wlan0 = 10.79.215.11
```

在主机上先确认自己也连接到了同一个 Wi‑Fi，然后：

```bash
ping 10.79.215.11
ssh unitree@10.79.215.11
```

### 如果同网段但仍然无法互访

比如：

- 主机能拿到同网段地址
- 但 `ping` 不通
- `ip neigh` 显示 `FAILED`

这通常说明：

- AP / 热点开启了客户端隔离
- 网络策略不允许同网段设备互访

此时：

- `ssh` 到 G1 会失败
- 不是 G1 的 `sshd` 一定有问题
- 更可能是无线网络本身限制了客户端互访

## 8. 一个推荐的工作方式

如果目标是“无线远程操作 G1”，推荐流程是：

1. 主机和 G1 都连接到同一个可互访 Wi‑Fi
2. 主机通过 `ssh` 登录到 G1 的 `wlan0` 地址
3. 在 G1 上启动：

```bash
BRIDGE_NETWORK_INTERFACE=eth0 bridge/build/cpp_bridge_main
python bridge/python/deploy_policy.py
```

这样：

- Wi‑Fi 只负责远程登录和管理
- `eth0` 继续承担机器人底层实时控制链路

## 9. 常用排查命令汇总

G1 上：

```bash
nmcli connection show --active
sudo nmcli device wifi rescan ifname wlan0
nmcli -f IN-USE,SSID,BSSID,SIGNAL,SECURITY device wifi list ifname wlan0
ip addr show wlan0
systemctl status ssh
ss -ltnp | grep :22
```

主机上：

```bash
ip addr show wlo1
ping <g1_wifi_ip>
ip route get <g1_wifi_ip>
ip neigh | grep <g1_wifi_ip>
ssh unitree@<g1_wifi_ip>
```
