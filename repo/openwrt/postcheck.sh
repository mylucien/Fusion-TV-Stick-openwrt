#!/bin/sh
# 校验 OpenWrt 编译出来的 .ko 能不能被目标内核 insmod。
#
# 两条硬指标，任一条不满足都是"装不上"，默认让构建失败（STRICT=1）：
#
#   1. .gnu.linkonce.this_module 段大小 == 内核里 sizeof(struct module)。
#      它完全由目标内核 .config 决定（TRACING / PERF_EVENTS / LIVEPATCH /
#      BPF 等），差一个字节 insmod 就拒绝：
#
#        module dvb_core: .gnu.linkonce.this_module section size must match
#                         the kernel's built struct module size at run time
#
#      在 an7581 这台设备上是 704。
#
#   2. vermagic 与设备内核**完全一致**。原来只比后缀
#      (" SMP mod_unload aarch64")，那样 6.18.99、6.18.55-rc1 编出来的
#      都会被判通过，而实际 insmod 会报 "version magic ... should be ..."。
#
# 环境变量：
#   KO_MDIR             .ko 所在目录，默认当前目录
#   OBJDUMP             objdump（交叉工具链的，在主机上执行）
#   EXPECT_MODULE_SIZE  期望的 struct module 大小，默认 704；0 = 不比对
#   EXPECT_VERMAGIC     期望的 vermagic 全串，默认 6.18.55 SMP mod_unload aarch64
#                       置空 = 不比对
#   STRICT              1（默认）不符就让构建失败；0 只打印警告
#
# 想临时放过一次（比如故意用别的内核树试编）：
#   STRICT=0 sh openwrt/postcheck.sh
#   EXPECT_MODULE_SIZE=0 EXPECT_VERMAGIC= sh openwrt/postcheck.sh

set -u

KO_MDIR="${KO_MDIR:-.}"
OBJDUMP="${OBJDUMP:-objdump}"
EXPECT="${EXPECT_MODULE_SIZE:-704}"
EXPECT_VM="${EXPECT_VERMAGIC-6.18.55 SMP mod_unload aarch64}"
STRICT="${STRICT:-1}"

rc=0

fail() {
	echo "   !! $*"
	[ "$STRICT" = "1" ] && rc=1
	return 0
}

for m in dvb-core dvb-usb dvb-usb-smit; do
	f="$KO_MDIR/$m.ko"
	if [ ! -f "$f" ]; then
		echo "!! 缺少 $m.ko"
		rc=1
		continue
	fi

	# 不依赖 strings(1)：grep -a 直接扫二进制里的 .modinfo
	vm=$(grep -a -o 'vermagic=[0-9A-Za-z._+ ~]*' "$f" 2>/dev/null |
		head -1 | sed 's/^vermagic=//')

	hex=$("$OBJDUMP" -h "$f" 2>/dev/null |
		awk '$2 == ".gnu.linkonce.this_module" { print $3; exit }')

	if [ -n "${hex:-}" ]; then
		size=$((0x$hex))
		printf '   %-14s vermagic="%s"  this_module=%s\n' "$m" "${vm:-?}" "$size"
		if [ "$EXPECT" != "0" ] && [ "$size" != "$EXPECT" ]; then
			fail "$m.ko: struct module = $size，目标设备是 $EXPECT"
			echo "      这个内核的 .config 与目标设备不同，模块装不上。"
		fi
	else
		printf '   %-14s vermagic="%s"  this_module=?\n' "$m" "${vm:-?}"
		echo "   (objdump 读不出段表，跳过大小比对)"
	fi

	if [ -n "$EXPECT_VM" ]; then
		if [ -z "${vm:-}" ]; then
			fail "$m.ko: 读不到 vermagic"
		elif [ "$vm" != "$EXPECT_VM" ]; then
			fail "$m.ko: vermagic \"$vm\" 与期望 \"$EXPECT_VM\" 不一致"
		fi
	fi
done

if [ "$rc" != "0" ]; then
	echo "!! 产物校验失败：这些 .ko 装不进目标设备。"
	echo "   确认用的是本设备(an7581/6.18.55)那套配置的源码树；"
	echo "   确实要放过：STRICT=0 sh openwrt/postcheck.sh"
fi

exit $rc
