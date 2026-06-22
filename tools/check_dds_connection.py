"""
DDS 连接诊断脚本

用法:
    python tools/check_dds_connection.py [网卡名]
    python tools/check_dds_connection.py enp109s0

若不传参数，从 deploy_real/config/real.yaml 读取网卡名。
"""

import sys
import time
import subprocess
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent.absolute()))

# ── 读取网卡名 ────────────────────────────────────────────────────────────────

if len(sys.argv) > 1:
    net_interface = sys.argv[1]
else:
    import yaml
    config_path = Path(__file__).parent.parent / "deploy_real" / "config" / "real.yaml"
    with open(config_path) as f:
        net_interface = yaml.safe_load(f)["net"]

print(f"网卡: {net_interface}")

# ── 1. 检查网卡是否存在 ────────────────────────────────────────────────────────

print("\n[1/3] 检查网卡...")
result = subprocess.run(["ip", "addr", "show", net_interface], capture_output=True, text=True)
if result.returncode != 0:
    print(f"  错误: 找不到网卡 '{net_interface}'")
    print("  可用网卡:")
    subprocess.run(["ip", "-o", "link", "show"])
    sys.exit(1)

# 提取 IP
ip_line = [l.strip() for l in result.stdout.splitlines() if "inet " in l]
if ip_line:
    ip_addr = ip_line[0].split()[1]
    print(f"  IP 地址: {ip_addr}")
    if not ip_addr.startswith("192.168.123."):
        print(f"  警告: IP 不在 192.168.123.x 子网，DDS 可能无法发现机器人")
    else:
        print(f"  子网: 正确 (192.168.123.x)")
else:
    print(f"  警告: 网卡 '{net_interface}' 未分配 IP 地址")
    print(f"  请执行: sudo ip addr add 192.168.123.100/24 dev {net_interface}")

# ── 2. Ping 机器人 ─────────────────────────────────────────────────────────────

ROBOT_IP = "192.168.123.161"
print(f"\n[2/3] Ping 机器人 ({ROBOT_IP})...")
result = subprocess.run(["ping", "-c", "3", "-W", "1", ROBOT_IP], capture_output=True, text=True)
if result.returncode == 0:
    # 提取延迟
    for line in result.stdout.splitlines():
        if "avg" in line or "rtt" in line:
            print(f"  {line.strip()}")
    print(f"  Ping: 正常")
else:
    print(f"  Ping 失败: 无法到达 {ROBOT_IP}")
    print(f"  请确认:")
    print(f"    - 网线已连接")
    print(f"    - 机器人已开机")
    print(f"    - 本机 IP 在 192.168.123.x 子网")

# ── 3. DDS 订阅测试 ────────────────────────────────────────────────────────────

print(f"\n[3/3] DDS 订阅测试 (5秒)...")

try:
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowStateHG
except ImportError:
    print("  错误: 未安装 unitree_sdk2py，请先安装")
    sys.exit(1)

msg_count = 0
last_tick = None
imu_samples = []

def handler(msg: LowStateHG):
    global msg_count, last_tick
    msg_count += 1
    last_tick = msg.tick
    if len(imu_samples) < 3:
        q = msg.imu_state.quaternion
        imu_samples.append(q)
        print(f"  收到第 {msg_count} 条: tick={msg.tick}  quat=({q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f})")

ChannelFactoryInitialize(0, net_interface)
sub = ChannelSubscriber("rt/lowstate", LowStateHG)
sub.Init(handler, 10)

for i in range(5):
    time.sleep(1)
    print(f"  已等待 {i+1}s，收到 {msg_count} 条消息...")
    if msg_count > 10:
        break

# ── 结果汇总 ──────────────────────────────────────────────────────────────────

print("\n" + "=" * 50)
if msg_count > 0:
    rate = msg_count / min(5, (msg_count / 50 + 0.1))  # 粗略估算
    print(f"DDS 连接正常")
    print(f"  收到消息数: {msg_count}")
    print(f"  最后 tick:  {last_tick}")
    print(f"  可以运行真机部署: python deploy_real/deploy_real.py")
else:
    print(f"DDS 连接失败，未收到任何消息")
    print(f"  排查建议:")
    print(f"    1. sudo ufw disable  (临时关闭防火墙)")
    print(f"    2. 确认 IP 在 192.168.123.x 子网")
    print(f"    3. 确认网线连接到机器人网口（非交换机）")
    print(f"    4. 确认 Domain ID 为 0（G1 默认）")
