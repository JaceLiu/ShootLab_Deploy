#!/usr/bin/env bash
# 在 G1 上快速切换已保存的 Wi‑Fi 配置（NetworkManager / nmcli）。
# 用法见文末或: ./wifi-switch.sh help
#
# HKU 若已设为静态 IP，只需能 connection up 对应配置文件即可。
# Xiaomi_14 静态 IP：先 DHCP 连一次，用 `ip -4 route show dev "$WIFI_IFACE"` 看网关，
# 再填下面的 XIAOMI_IPV4_* 并执行: ./wifi-switch.sh setup-xiaomi-static

set -euo pipefail

WIFI_IFACE="${WIFI_IFACE:-wlan0}"
# NetworkManager 连接名（常与 SSID 相同；若不对，用 list 子命令或 nmcli -t -f NAME connection show 查看）
HKU_CONN="${HKU_NM_CONNECTION:-HKU}"
XIAOMI_CONN="${XIAOMI_NM_CONNECTION:-Xiaomi_14}"

# 手机热点静态 IPv4（按你实际热点网段修改；未设置时不要运行 setup-xiaomi-static）
# 典型小米/安卓热点：192.168.43.x/24，网关多为 192.168.43.1（仍以本机 DHCP 时 `ip route` 为准）
XIAOMI_IPV4_ADDR="${XIAOMI_IPV4_ADDR:-}"
XIAOMI_IPV4_GW="${XIAOMI_IPV4_GW:-}"
XIAOMI_IPV4_DNS="${XIAOMI_IPV4_DNS:-8.8.8.8}"

usage() {
	cat <<'EOF'
用法:
  ./wifi-switch.sh hku|hk              连接到 HKU（默认 nmcli 连接名: HKU）
  ./wifi-switch.sh xiaomi|mi           连接到 Xiaomi_14
  ./wifi-switch.sh setup-xiaomi-static  将 Xiaomi_14 档案改为静态 IPv4（需已设置 XIAOMI_IPV4_*）
  ./wifi-switch.sh status              当前 WLAN 连接与地址
  ./wifi-switch.sh list                列出所有 NetworkManager 连接名

环境变量（可选）:
  WIFI_IFACE          默认 wlan0
  HKU_NM_CONNECTION   HKU 的 nmcli 连接名
  XIAOMI_NM_CONNECTION Xiaomi 的 nmcli 连接名
  XIAOMI_IPV4_ADDR    如 192.168.43.100/24
  XIAOMI_IPV4_GW      如 192.168.43.1
  XIAOMI_IPV4_DNS     默认 8.8.8.8

一次性配置静态 IP 示例:
  export XIAOMI_IPV4_ADDR='192.168.43.100/24'
  export XIAOMI_IPV4_GW='192.168.43.1'
  ./wifi-switch.sh setup-xiaomi-static
  ./wifi-switch.sh xiaomi
EOF
}

reexec_as_root_if_needed() {
	local -a argv=("$@")
	local sub="${1:-}"
	case "$sub" in
	hku | hk | HKU | xiaomi | mi | Xiaomi_14 | setup-xiaomi-static)
		if [[ "${EUID:-}" -ne 0 ]]; then
			exec sudo env \
				"WIFI_IFACE=$WIFI_IFACE" \
				"HKU_NM_CONNECTION=${HKU_NM_CONNECTION-}" \
				"XIAOMI_NM_CONNECTION=${XIAOMI_NM_CONNECTION-}" \
				"XIAOMI_IPV4_ADDR=${XIAOMI_IPV4_ADDR-}" \
				"XIAOMI_IPV4_GW=${XIAOMI_IPV4_GW-}" \
				"XIAOMI_IPV4_DNS=${XIAOMI_IPV4_DNS:-8.8.8.8}" \
				"$0" "${argv[@]}"
		fi
		;;
	esac
}

cmd_up() {
	local conn="$1"
	nmcli device connect "$WIFI_IFACE" >/dev/null 2>&1 || true
	if nmcli connection up "$conn" ifname "$WIFI_IFACE"; then
		echo "已激活: $conn (接口 $WIFI_IFACE)"
		nmcli -g IP4.ADDRESS device show "$WIFI_IFACE" 2>/dev/null || ip -4 addr show dev "$WIFI_IFACE"
	else
		echo "激活失败: $conn" >&2
		echo "提示: 运行 './wifi-switch.sh list' 查看实际连接名，并用 HKU_NM_CONNECTION / XIAOMI_NM_CONNECTION 覆盖。" >&2
		return 1
	fi
}

cmd_setup_xiaomi_static() {
	if [[ -z "$XIAOMI_IPV4_ADDR" || -z "$XIAOMI_IPV4_GW" ]]; then
		echo "请先设置 XIAOMI_IPV4_ADDR 与 XIAOMI_IPV4_GW（见 ./wifi-switch.sh help）" >&2
		exit 1
	fi
	nmcli connection modify "$XIAOMI_CONN" \
		ipv4.method manual \
		ipv4.addresses "$XIAOMI_IPV4_ADDR" \
		ipv4.gateway "$XIAOMI_IPV4_GW" \
		ipv4.dns "$XIAOMI_IPV4_DNS" \
		ipv4.ignore-auto-dns no
	echo "已把 \"$XIAOMI_CONN\" 设为静态 IPv4: $XIAOMI_IPV4_ADDR gateway $XIAOMI_IPV4_GW"
	echo "请执行: ./wifi-switch.sh xiaomi"
}

cmd_status() {
	echo "接口: $WIFI_IFACE"
	nmcli -g GENERAL.CONNECTION device show "$WIFI_IFACE" 2>/dev/null || true
	ip -4 addr show dev "$WIFI_IFACE"
	ip -4 route show dev "$WIFI_IFACE" || true
}

cmd_list() {
	nmcli -t -f NAME,TYPE,DEVICE connection show | grep -E ':802-11-wireless:|wifi' || nmcli connection show
}

main() {
	reexec_as_root_if_needed "$@"
	local sub="${1:-}"
	case "$sub" in
	hku | hk | HKU)
		cmd_up "$HKU_CONN"
		;;
	xiaomi | mi | Xiaomi_14)
		cmd_up "$XIAOMI_CONN"
		;;
	setup-xiaomi-static)
		cmd_setup_xiaomi_static
		;;
	status)
		cmd_status
		;;
	list)
		cmd_list
		;;
	help | -h | --help | "")
		usage
		;;
	*)
		echo "未知子命令: $sub" >&2
		usage >&2
		exit 1
		;;
	esac
}

main "$@"
