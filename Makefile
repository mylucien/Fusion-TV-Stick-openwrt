# SPDX-License-Identifier: GPL-2.0
#
# dvb-usb-smit -- SMIT iCast DTMB/DVB-C USB dongle (29df:0001)
#
# 把本文件放到 OpenWrt / PonWrt 源码树的
#     package/kernel/dvb-usb-smit/Makefile
# 并把本仓库 clone 到
#     package/kernel/dvb-usb-smit/repo/
#
# 详见 openwrt/README.md
#
# 要点：dvb-core / dvb-usb 由本包自带编译，目标内核不需要
# CONFIG_MEDIA_SUPPORT / CONFIG_DVB_CORE（固件里根本没有这两个）。

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=dvb-usb-smit
PKG_RELEASE:=1
PKG_LICENSE:=GPL-2.0
PKG_MAINTAINER:=lucien

# 仓库位置。默认是本包目录下的 repo/ 子目录。
DVB_SMIT_SRC ?= $(CURDIR)/repo

# 1 = 不编 rc-core（驱动没有 .rc，遥控走空实现；设备也没红外）
NO_RC ?= 1

# kbuild 的 M= 构建本身是并行安全的
PKG_BUILD_PARALLEL:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/dvb-usb-smit
  SUBMENU:=Video Support
  TITLE:=SMIT iCast DTMB/DVB-C USB dongle
  # @!PACKAGE_kmod-dvb-core：本包自带一个同名 dvb-core.ko，和 OpenWrt 自带的
  # kmod-dvb-core 同时选中必然互相覆盖。用互斥让 menuconfig 里二选一；
  # 这个符号不存在时（精简树里没这个包）Kconfig 视作 n，!n = y，不影响。
  DEPENDS:=@USB_SUPPORT +kmod-usb-core @!PACKAGE_kmod-dvb-core
  FILES:= \
	$(PKG_BUILD_DIR)/dvb-core.ko \
	$(PKG_BUILD_DIR)/dvb-usb.ko \
	$(PKG_BUILD_DIR)/dvb-usb-smit.ko
  AUTOLOAD:=$(call AutoLoad,30,dvb-core dvb-usb dvb-usb-smit)
endef

define KernelPackage/dvb-usb-smit/description
 SMIT iCast (29df:0001) DTMB/DVB-C USB dongle driver, ported to airoha/an7581.

 dvb-core.ko / dvb-usb.ko 也由本包编出来，因此目标内核不需要
 CONFIG_DVB_CORE。注意**不要**同时选 OpenWrt 自带的 kmod-dvb-core：
 两个包会各自装一个同名 dvb-core.ko，后装的覆盖先装的。
endef

define Build/Prepare
	@test -d "$(DVB_SMIT_SRC)/src" || { \
		echo "!! 找不到驱动源码: $(DVB_SMIT_SRC)"; \
		echo "   把本仓库 clone 到 package/kernel/dvb-usb-smit/repo/"; \
		exit 1; }
	rm -rf $(PKG_BUILD_DIR)
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) $(DVB_SMIT_SRC)/. $(PKG_BUILD_DIR)/
	rm -rf $(PKG_BUILD_DIR)/.git
	# 从 $(LINUX_DIR) 里抠出 dvb-core / dvb-usb 源码到 kern/，
	# 顺便检查目标内核的文件清单和本 Makefile 的 obj 列表是否对得上
	KERNEL_DIR="$(LINUX_DIR)" NO_RC="$(NO_RC)" \
		bash $(PKG_BUILD_DIR)/scripts/prepare-src.sh
endef

define Build/Compile
	# 走仓库自带的 kbuild 包装（它自己会 make -C $(LINUX_DIR) M=...）；
	# KBUILD_MODPOST_WARN=1：dvb-core/dvb-usb 是同一批 co 出来的模块，
	# 内核的 Module.symvers 里没有它们的符号，modpost 报未解析是预期的
	+$(MAKE) -C $(PKG_BUILD_DIR) $(PKG_JOBS) \
		KERNEL_DIR="$(LINUX_DIR)" \
		ARCH="$(LINUX_KARCH)" \
		CROSS_COMPILE="$(TARGET_CROSS)" \
		NO_RC="$(NO_RC)" \
		KBUILD_MODPOST_WARN=1 \
		all
	# 把两条硬指标打出来（struct module 大小、vermagic）
	KO_MDIR="$(PKG_BUILD_DIR)" OBJDUMP="$(TARGET_CROSS)objdump" \
		sh $(PKG_BUILD_DIR)/openwrt/postcheck.sh
endef

$(eval $(call KernelPackage,dvb-usb-smit))
