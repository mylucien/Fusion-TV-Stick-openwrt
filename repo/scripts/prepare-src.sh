#!/usr/bin/env bash
# Copy the DVB / RC sources of the target kernel into ./kern so they can be
# built out-of-tree as standalone modules.
#
# Usage:  KERNEL_DIR=/path/to/linux-6.18.52 [NO_RC=1] ./scripts/prepare-src.sh
set -euo pipefail

KERNEL_DIR="${KERNEL_DIR:?set KERNEL_DIR to the prepared kernel tree}"
NO_RC="${NO_RC:-1}"
cd "$(dirname "$0")/.."
ROOT="$PWD"

if [ ! -f "$KERNEL_DIR/Makefile" ]; then
	echo "!! $KERNEL_DIR does not look like a kernel tree" >&2
	exit 1
fi

KVER=$(make -s -C "$KERNEL_DIR" kernelversion 2>/dev/null || echo unknown)
echo ">> kernel version : $KVER"

echo ">> cleaning kern/"
rm -rf kern
mkdir -p kern

echo ">> copying dvb-core"
cp -a "$KERNEL_DIR/drivers/media/dvb-core" kern/
echo ">> copying dvb-usb"
cp -a "$KERNEL_DIR/drivers/media/usb/dvb-usb" kern/

# dvb-usb.h 里 #include "dvb-pll.h"，而 6.18 已把 dvb-pll.h 挪到 dvb-frontends，
# 树外编译时同目录找不到，补一份进来（只是原型声明，不需要 dvb-pll.c）。
echo ">> copying dvb-pll.h from dvb-frontends"
cp -a "$KERNEL_DIR/drivers/media/dvb-frontends/dvb-pll.h" kern/dvb-usb/

if [ "$NO_RC" = "0" ]; then
	echo ">> copying rc (NO_RC=0)"
	cp -a "$KERNEL_DIR/drivers/media/rc" kern/
fi

# ---- verify every object referenced by the Makefile actually exists -------
missing=0
check() {
	local f="$1"
	if [ ! -f "kern/$f" ]; then
		echo "   MISSING: kern/$f"
		missing=$((missing + 1))
	fi
}

for f in dvbdev.c dmxdev.c dvb_demux.c dvb_ca_en50221.c dvb_frontend.c \
         dvb_net.c dvb_ringbuffer.c; do
	check "dvb-core/$f"
done
for f in dvb-usb-init.c dvb-usb-urb.c dvb-usb-i2c.c dvb-usb-dvb.c \
         dvb-usb-firmware.c usb-urb.c dvb-usb.h dvb-usb-common.h dvb-pll.h; do
	check "dvb-usb/$f"
done
if [ "$NO_RC" = "0" ]; then
	check "dvb-usb/dvb-usb-remote.c"
	check "rc/rc-main.c"
	check "rc/rc-ir-raw.c"
fi

if [ "$missing" -ne 0 ]; then
	echo
	echo "!! $missing expected source file(s) missing."
	echo "   The kernel tree differs from the layout this Makefile was written for."
	echo "   Compare kern/dvb-core/Makefile and kern/dvb-usb/Makefile with ours"
	echo "   and adjust dvb-core-objs / dvb-usb-objs accordingly."
	exit 1
fi

# ---- sanity: things that silently break module loading --------------------
cfg="$KERNEL_DIR/.config"
if [ -f "$cfg" ]; then
	echo ">> relevant kernel config:"
	grep -E '^CONFIG_(SMP|MODVERSIONS|PREEMPT|PREEMPT_NONE|PREEMPT_VOLUNTARY|PREEMPT_DYNAMIC|LOCALVERSION|MODULE_SIG|MODULE_SIG_FORCE|DVB|RC_CORE|INPUT|LEDS_TRIGGERS)=' "$cfg" | sed 's/^/     /' || true
	if grep -q '^CONFIG_MODVERSIONS=y' "$cfg"; then
		echo
		echo "!! CONFIG_MODVERSIONS=y : external modules need the kernel's"
		echo "   Module.symvers, which 'modules_prepare' does not produce."
		echo "   Build inside the real OpenWrt buildroot (or its SDK) instead."
	fi
fi

echo
echo ">> ok. next:"
echo "   make KERNEL_DIR=$KERNEL_DIR ARCH=arm64 CROSS_COMPILE=aarch64-openwrt-linux- [NO_RC=1]"
