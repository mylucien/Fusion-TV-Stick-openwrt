// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SMIT iCast DTMB/DVC driver
 *
 * Copyright (C) 2026 Xiaodong Ni <nxiaodong520@gmail.com>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/moduleparam.h>
#include <linux/usb.h>
#include <linux/workqueue.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 16, 0)
#include <dvb_frontend.h>
#else
#include <media/dvb_frontend.h>
#endif

#include "dvb-usb.h"
#include "icast-fe.h"
#include "smit.h"

//#define CONFIG_DYNAMIC_DEBUG

#define USB_VID_SMIT							0x29DF
#define USB_PID_SMIT_ICAST				0x0001

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

/* 热芯片跳过初始化时拿不到 tuner 信息，dtmb 只能取默认值。
 * 必须默认 1（双模 ops）：它同时声明 SYS_DVBT + SYS_DVBC_ANNEX_A，
 * 是单模 ops 的超集，取错也不会少功能。
 * 冷启动路径会被 smit_init 里的实际检测覆盖。 */
static struct icast_fe_config icast_config = {
	.dtmb = 1,
};

/* ---- 调试开关（默认全关，不影响正常行为） ----------------------------
 *
 * trace         : 打印 smit_init 每条命令的耗时
 * skip_wake     : probe 时跳过状态判定，直接跑初始化序列
 *                 （用来复现"热芯片重跑 init 会把自己打死"）
 * reset_on_exit : 卸载时做硬件复位，强制下一次 probe 走冷启动路径
 * diag_probe    : probe 时打印设备描述符和 vendor control 请求的应答
 * autorecover   : 命令通道连续失败时自动复位芯片重新枚举（默认开）
 * settle_ms     : 唤醒成功后等多久再发第一条命令（让应用固件启动完）
 * reset_on_cold : 冷芯片（module_init 探活超时）时是否立刻复位。默认开。
 *                 关掉它可以省掉实测 108s 的重枚举，让 probe 用 wake 顶上；
 *                 但那条路径是"ping 已经把 OUT FIFO 弄脏、还不复位就去发
 *                 唤醒包"，真机上从来没验证过（历史上冷启动能过，都是
 *                 ping 后面跟着一次复位）。所以默认保持现场已知行为，
 *                 这个参数留给冷启动实验。
 */
bool smit_trace;
bool smit_autorecover = true;
static bool smit_skip_wake;
static bool smit_reset_on_exit;
static bool smit_diag_probe;
static bool smit_reset_on_cold = true;

/* 唤醒包应答之后、固件真正能处理业务命令之前的空窗。
 * 实测撞过一次：wake answered 0ms，紧接着 560ms 内 cmd0 仍超时，
 * 整套 init 全 -110 并触发自愈复位，然后循环。300ms 是保守值。 */
static int smit_settle_ms = 300;

/* probe / smit_init 执行期间为 true，smit_note() 看到它就拒绝触发自愈。
 * 详见 smit.c 里 smit_note() 的说明：这段时间的工作由 probe 自己兜底。 */
bool smit_in_probe;

module_param_named(trace, smit_trace, bool, 0644);
MODULE_PARM_DESC(trace, "print per-command timing of the init sequence");
module_param_named(skip_wake, smit_skip_wake, bool, 0644);
MODULE_PARM_DESC(skip_wake, "skip the wake-up and run the init sequence directly");
module_param_named(reset_on_exit, smit_reset_on_exit, bool, 0644);
MODULE_PARM_DESC(reset_on_exit, "hardware-reset the chip on disconnect (forces a cold next probe)");
module_param_named(diag_probe, smit_diag_probe, bool, 0644);
MODULE_PARM_DESC(diag_probe, "dump descriptors and vendor control replies during probe");
module_param_named(autorecover, smit_autorecover, bool, 0644);
MODULE_PARM_DESC(autorecover, "reset the chip to re-enumerate when the command channel dies");
module_param_named(settle_ms, smit_settle_ms, int, 0644);
MODULE_PARM_DESC(settle_ms, "wait this long after the wake-up before the first command");
module_param_named(reset_on_cold, smit_reset_on_cold, bool, 0644);
MODULE_PARM_DESC(reset_on_cold, "reset the chip when module_init finds it cold (default on; 0 = let probe wake it, unverified on real hardware)");

static int icast_frontend_attach(struct dvb_usb_adapter *adap)
{
	/* Frontend and bridge live in the same module, so call it directly
	 * instead of dvb_attach()/symbol_request(). One less .ko to load. */
	if ((adap->fe_adap[0].fe = icast_fe_attach(adap, &icast_config)) != NULL) {
		return 0;
	}
	info("not attached iCast usb device");
	return -ENODEV;
}

static struct usb_device_id icast_id_table[] = {
	{USB_DEVICE(USB_VID_SMIT, USB_PID_SMIT_ICAST)},
	{}
};

MODULE_DEVICE_TABLE(usb, icast_id_table);

static struct dvb_usb_device_properties icast_properties = {
	.num_adapters = 1,

	.adapter = {
		{
		.num_frontends = 1,
		.fe = {{
			/* 这里原来声明了 DVB_USB_ADAP_HAS_PID_FILTER，但驱动从来没
			 * 实现过 .pid_filter / .pid_filter_ctrl —— 声明的是个不存在
			 * 的能力。后果在 dvb-usb-init.c 里：设备以 Full-speed 枚举时
			 * （caps 有 HAS_PID_FILTER）会走 pid_filtering=1 分支，
			 * max_feed_count = pid_filter_count，而后者没赋值就是 0；
			 * 再往下 demux.filternum = 0 —— 节点出来了却一个 filter 都
			 * 开不了。去掉这个声明后，Full-speed 会被明确拒绝
			 * （"cannot be run on USB1.1 port"），High-speed 行为不变
			 * （pid_filtering=0，走软件 demux，max_feed_count=255）。 */
			.caps = 0,
			.frontend_attach  = icast_frontend_attach,
			.stream = {
				.type = USB_BULK,
				.count = 8,
				.endpoint = SMIT_BULK_ENDPOINT_STREAM,
				.u = {
					.bulk = {
						.buffersize = 8192,
					}
				}
			},
		}},	
		},
	},
	.num_device_descs = 1,
	.devices = {
		{
			.name = "iCast DVB USB DONGLE adapter",
		  .warm_ids = {&icast_id_table[0], NULL},
		},
	}
};

/* 唤醒序列 "FE 00 10 00" 只对"刚上电、固件还停在等待握手状态"的冷芯片有效。
 *
 * 必须只在确认为冷芯片之后才发：真机对照实验里，热芯片（已经初始化过一次）
 * 收到这个包不但不应答，还会让之后的命令全部超时 —— 它本身就是毒药。
 * 所以调用顺序是先 smit_set(0x82) 探活，探不到才允许发唤醒包。
 *
 * 冷芯片唤醒实测：第一次就应答（0 ms）；热芯片重试 9 次 / 6270 ms 全超时，
 * 所以 1s 预算足够，没必要拉长。
 */
#define SMIT_WAKE_BUDGET_MS	1000
#define SMIT_WAKE_TIMEOUT_MS	500
#define SMIT_WAKE_BUF_SIZE	512
#define SMIT_DIAG_BUF_SIZE	64

static int smit_wake(struct usb_device *udev)
{
	const u8 wake[4] = { 0xFE, 0x00, 0x10, 0x00 };
	unsigned long t0 = jiffies, deadline;
	int size, ret = 0, attempt = 0;
	u8 *buf;

	buf = kmalloc(SMIT_WAKE_BUF_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	deadline = t0 + msecs_to_jiffies(SMIT_WAKE_BUDGET_MS);

	for (;;) {
		int pipe = usb_sndbulkpipe(udev, SMIT_BULK_ENDPOINT_CMD_SEND);

		attempt++;

		memcpy(buf, wake, sizeof(wake));
		ret = usb_bulk_msg(udev, pipe, buf, sizeof(wake), &size,
				   SMIT_WAKE_TIMEOUT_MS);
		if (!ret) {
			pipe = usb_rcvbulkpipe(udev, SMIT_BULK_ENDPOINT_CMD_RECEIVE);
			ret = usb_bulk_msg(udev, pipe, buf, SMIT_WAKE_BUF_SIZE,
					   &size, SMIT_WAKE_TIMEOUT_MS);
			/* 回包长度不确定：设备给的数据比缓冲区多只会报
			 * EOVERFLOW，这时包其实已经收到了，不算失败 */
			if (ret == -EOVERFLOW)
				ret = 0;
		}

		if (!ret) {
			dev_info(&udev->dev,
				 "wake answered after %d attempt(s), %u ms\n",
				 attempt, jiffies_to_msecs(jiffies - t0));
			kfree(buf);
			return 0;
		}

		/* 端点 stall 或超时都可能是"还没准备好"；
		 * 其它错误（设备没了/被关掉了）重试没有意义 */
		if (ret != -EPIPE && ret != -ETIMEDOUT)
			break;

		/* 只有真 stall(-EPIPE) 才值得付 clear_halt 的代价。
		 * 原来这里是每轮无条件清两个端点，而 usb_clear_halt 内部发的是
		 * CLEAR_FEATURE 控制请求、超时用 USB_CTRL_SET_TIMEOUT（5s）：
		 * 芯片连 control 都不应答时每轮固定烧 10s，1s 的预算形同废纸
		 * （实测死透的芯片能把这里拖到几十秒）。超时的时候端点并没有
		 * halt，清它纯属自找 5s。现在也只清真正出错的那个端点。 */
		/* 预算用完就直接放弃。这个判断必须在 clear_halt **之前**：
		 * 否则最后一轮 EPIPE 会先白烧一次 CLEAR_FEATURE（芯片不应答
		 * 控制请求时 5s），然后才发现预算早就没了。smit.c 里同类问题
		 * 已经修过，这里之前漏了。 */
		if (time_after_eq(jiffies, deadline))
			break;

		if (ret == -EPIPE)
			usb_clear_halt(udev, pipe);

		msleep(100);
	}

	dev_info(&udev->dev,
		 "wake gave up after %d attempt(s) / %u ms, last error %d\n",
		 attempt, jiffies_to_msecs(jiffies - t0), ret);
	kfree(buf);
	return ret;
}

/* 采集冷/热芯片的描述符与 vendor control 应答（diag_probe=1）。
 *
 * 缓冲区必须用 kmalloc，不能是栈数组：hcd_map_urb_for_dma() 对栈上的
 * transfer_buffer 直接 WARN_ONCE 并返回 -EAGAIN，请求根本发不出去。
 * 真机实拍（diag_probe=1）：
 *   usb 1-1: diag: bcdUSB=0200 ...
 *   transfer buffer is on stack
 *   usb 1-1: diag: ctrl req=a0 val=0000 -> -11
 *   ...
 * 五个请求全是 -11，一个字节都没到设备。所以之前"vendor control 请求
 * 冷热完全一致、零区分度"的结论是假的 —— 那些 -11 是主机侧映射失败，
 * 不是设备的答复。smit_ping 当年踩过同一个坑（见上面的注释）。 */
static void smit_diag(struct usb_device *udev)
{
	const struct usb_device_descriptor *d = &udev->descriptor;
	static const struct { u8 type; u8 req; u16 val; u16 idx; u16 len; } t[] = {
		{ 0xC0, 0xA0, 0x0000, 0, 64 },
		{ 0xC0, 0x01, 0x0000, 0, 8  },
		{ 0xC0, 0x81, 0x0000, 0, 64 },
		{ 0xC0, 0x82, 0x0000, 0, 64 },
		{ 0xC0, 0xFF, 0x0000, 0, 64 },
	};
	u8 *buf;
	int i, ret;

	buf = kmalloc(SMIT_DIAG_BUF_SIZE, GFP_KERNEL);
	if (!buf) {
		dev_warn(&udev->dev, "diag: no memory, skipped\n");
		return;
	}

	dev_info(&udev->dev,
		 "diag: bcdUSB=%04x bcdDevice=%04x id=%04x:%04x cfg=%u cls=%02x/%02x/%02x iMan=%u iProd=%u iSer=%u speed=%d\n",
		 d->bcdUSB, d->bcdDevice, d->idVendor, d->idProduct,
		 d->bNumConfigurations, d->bDeviceClass, d->bDeviceSubClass,
		 d->bDeviceProtocol, d->iManufacturer, d->iProduct,
		 d->iSerialNumber, udev->speed);

	for (i = 0; i < ARRAY_SIZE(t); i++) {
		int n = 0;

		memset(buf, 0, SMIT_DIAG_BUF_SIZE);
		ret = usb_control_msg(udev,
			usb_rcvctrlpipe(udev, 0), t[i].req, t[i].type,
			t[i].val, t[i].idx, buf, t[i].len, 300);
		if (ret > 0)
			n = min(ret, 16);
		dev_info(&udev->dev, "diag: ctrl req=%02x val=%04x -> %d %*ph\n",
			 t[i].req, t[i].val, ret, n, buf);
	}

	if (d->iProduct) {
		ret = usb_string(udev, d->iProduct, buf, SMIT_DIAG_BUF_SIZE - 1);
		if (ret > 0) {
			buf[ret] = 0;
			dev_info(&udev->dev, "diag: iProduct=\"%s\"\n", buf);
		}
	}

	kfree(buf);
}

/* 探针：初始化序列的第一条命令本来就是 "req 0x82 读 1 字节" 的只读查询。
 * 热芯片 0ms 就回 0，冷芯片必然超时（-ETIMEDOUT）。
 *
 * 必须走 smit_set() 而不是手搓 usb_bulk_msg：直接拿栈缓冲区调 usb_bulk_msg
 * 实测会返回 -EAGAIN(-11)，看起来像错误其实根本不是设备给的答案，
 * 会把热芯片误判成冷芯片。smit_set 内部用 kmalloc 缓冲区，行为可靠。
 */
static int smit_ping(struct usb_device *udev)
{
	u8 cmd0[1] = { 0x01 };

	return smit_set(udev, 0x82, cmd0, 1, 0);
}

/* ping 的返回值怎么解读，决定要不要给芯片发唤醒包。
 *
 * 只有"芯片完全没回话"（超时 / 端点 stall）才算冷。其余 errno 一律不能当冷：
 *   -ENOMEM          : smit_usb_bulk_msg 里 kmalloc 失败，主机侧就没发出去
 *   -EPROTO          : 总线协议错，常见于线材/供电抖一下
 *   -EAGAIN / -EINTR : 映射失败或被信号打断
 * 误判的代价是不对称的：唤醒包对热芯片是毒药，会把固件打死、只有 vendor
 * reset 能救（而 reset 在有的批次上要十几分钟）；对一颗其实冷着的芯片，
 * 晚一次唤醒只是浪费一次机会。所以宁可漏判，不可误判。
 */
static bool smit_ping_is_cold(int ret)
{
	return ret == -ETIMEDOUT || ret == -EPIPE;
}

/* dvb_usb_device_init 失败时要不要补一刀复位：只有总线级故障值得。
 * attach / 内存 / adapter 注册失败这类主机侧问题，复位芯片解决不了。 */
static bool smit_is_bus_fault(int ret)
{
	return ret == -ETIMEDOUT || ret == -EPIPE || ret == -EPROTO;
}

/* ---- 复位代价判定 -----------------------------------------------------
 *
 * 同一行 smit_reset() 在两支同型号棒子上代价差两个数量级：
 * 实测一支 6.5s 回到总线，另一支 806s。所以复位不能当常规手段用，
 * 但也删不得 —— 在"便宜"的那支上它是 6.5s 的有效自愈。
 *
 * 判据只能靠实测：复位之后 SMIT_VERDICT_MS 内没有一次成功 probe，
 * 就认定这支设备的复位是"昂贵"的，本次加载内不再对它复位。
 * 端口（devpath）重新枚举后不变，用它当键；devnum 会变，不能用。
 *
 * reset_costly_ports 可以人工钉死（例如开机时就跳过复位），
 * 内核日志里那条 "marking it reset-costly on <port>" 就是要写进去的值。
 */
#define SMIT_VERDICT_MS		20000
#define SMIT_VERDICT_MAX	4
#define SMIT_PORT_LEN		12

/* 0444：这是个"写错就静默失效"的安全阀，读得到就够了。
 * 留成 0644 的话，运行时 echo 进去会和 probe / 自愈里的解析并发，
 * 没有锁，读到半截字符串的后果是"匹配不上 -> 复位照发"，
 * 而那可能是一支要等十几分钟的棒子。改它请重新 insmod（热芯片 0.1s）。 */
static char smit_costly_ports[64];
module_param_string(reset_costly_ports, smit_costly_ports,
		    sizeof(smit_costly_ports), 0444);
MODULE_PARM_DESC(reset_costly_ports,
	"comma separated ports (e.g. 1-1,3-1) where resetting the chip costs minutes");

static struct {
	char port[SMIT_PORT_LEN];
	bool costly;
} smit_verdict[SMIT_VERDICT_MAX];

static char smit_pending_port[SMIT_PORT_LEN];
static bool smit_pending_ok;
static struct delayed_work smit_reset_watch;

/* 端口名 = "%d-%s" (busnum, devpath)，和 sysfs / kmsg 里的 "1-1"、"3-1" 一致。
 * 别直接用 udev->devpath：它只是端口路径（本机两支棒子都是 "1"），
 * 拿它当键会让不同控制器上同端口的设备撞成同一个键 —— 实测踩过：
 * pin 表写 1-1 匹配不上 devpath 的 "1"，复位照发。 */
static void smit_portname(struct usb_device *udev, char *out, size_t len)
{
	snprintf(out, len, "%d-%s", udev->bus->busnum,
		 udev->devpath ? udev->devpath : "?");
}

/* 人工钉死的端口表：逗号分隔，允许前后空格。
 *
 * 尾随空格必须一起算掉：原来只跳前导空格，token 长度把尾随空格也数进去了，
 * 于是 "1-1 ,3-1" 里 1-1 匹配失败、"1-1, 3-1 " 里 3-1 匹配失败 —— 而这个参数
 * 是绕开分钟级复位的唯一开关，写错一个空格就静默失效、复位照发。
 * （用 strlen(port) 当长度是另一条线：它不含空格，所以只修剪 token 侧就够。） */
static bool smit_port_pinned_costly(const char *port)
{
	const char *p = smit_costly_ports;
	int n = strlen(port);

	while (*p) {
		const char *start, *end;

		while (*p == ',' || *p == ' ' || *p == '\t' || *p == '\n')
			p++;
		start = p;
		while (*p && *p != ',')
			p++;
		end = p;
		while (end > start && (end[-1] == ' ' || end[-1] == '\t' ||
				       end[-1] == '\n' || end[-1] == '\r'))
			end--;
		if (end - start == n && !strncmp(start, port, n))
			return true;
	}
	return false;
}

static int smit_verdict_slot(const char *port, bool create)
{
	int i, free_slot = -1;

	for (i = 0; i < SMIT_VERDICT_MAX; i++) {
		if (smit_verdict[i].port[0] &&
		    !strncmp(smit_verdict[i].port, port, SMIT_PORT_LEN - 1))
			return i;
		if (!smit_verdict[i].port[0] && free_slot < 0)
			free_slot = i;
	}
	return create ? free_slot : -1;
}

static void smit_verdict_set(const char *port, bool costly)
{
	int i = smit_verdict_slot(port, true);

	if (i < 0)		/* 表满：最简单的处理是丢掉最老的判断 */
		i = 0;
	strncpy(smit_verdict[i].port, port, SMIT_PORT_LEN - 1);
	smit_verdict[i].port[SMIT_PORT_LEN - 1] = 0;
	smit_verdict[i].costly = costly;
}

/* true = 这支设备上复位代价是分钟级，别再复位它 */
static bool smit_reset_is_costly(struct usb_device *udev)
{
	char port[SMIT_PORT_LEN];
	int i;

	smit_portname(udev, port, sizeof(port));
	if (smit_port_pinned_costly(port))
		return true;
	i = smit_verdict_slot(port, false);
	return i >= 0 && smit_verdict[i].costly;
}

/* 模块加载时在总线上找到的、确认已初始化的芯片的身份。
 * devnum 每次重新枚举都会变，所以它同时充当"这个芯片还是不是当年那个"的凭据。
 *
 * 但 devnum 只在**一条总线**内唯一：两个 USB 控制器上各挂一个同型号设备时
 * 可能出现相同的 devnum，只看它就会把另一个冷芯片误判成热芯片（跳过
 * 唤醒和初始化，节点出来了前端全哑）。所以连 busnum 一起记。 */
static int smit_warm_devnum = -1;
static int smit_warm_busnum = -1;

/* 复位是最后手段，也可能是死循环的入口：复位 -> 重新枚举 -> probe 又失败 ->
 * 再复位，每轮 7s。每次 probe 成功都会把计数清零；连续 SMIT_RECOVER_MAX_CONSEC
 * 次复位都没换来一次成功就放弃，把设备留在那儿等用户处理，
 * 总比每 7 秒重置一次强。 */
#define SMIT_RECOVER_MAX_CONSEC	3

static unsigned smit_reset_attempts;

static bool smit_reset_allowed(void)
{
	if (smit_reset_attempts >= SMIT_RECOVER_MAX_CONSEC) {
		pr_warn("dvb_usb_smit: %u resets without a successful probe, giving up\n",
			smit_reset_attempts);
		return false;
	}
	smit_reset_attempts++;
	return true;
}

/* 复位之后盯着它多久回总线：回来 = 便宜，不回来 = 昂贵（本次加载不再复位） */
static void smit_reset_watch_fn(struct work_struct *work)
{
	if (smit_pending_ok) {
		pr_info("dvb_usb_smit: %s probed ok after the reset, reset stays cheap here\n",
			smit_pending_port);
		return;
	}

	smit_verdict_set(smit_pending_port, true);
	pr_warn("dvb_usb_smit: %s did not come back within %d ms after a reset, marking it reset-costly on %s (no further resets this boot; add reset_costly_ports=%s to skip it at insmod)\n",
		smit_pending_port, SMIT_VERDICT_MS, smit_pending_port, smit_pending_port);
}

static DECLARE_DELAYED_WORK(smit_reset_watch, smit_reset_watch_fn);

/* 所有复位都走这里：先问"这支设备负担得起吗"，再决定发不发。
 * 返回 true 表示确实发了复位。 */
static bool smit_try_reset(struct usb_device *udev, const char *why)
{
	char port[SMIT_PORT_LEN];

	smit_portname(udev, port, sizeof(port));

	if (smit_reset_is_costly(udev)) {
		dev_warn(&udev->dev,
			 "%s, but a reset on %s is known to cost minutes; leaving it alone\n",
			 why, port);
		return false;
	}
	if (!smit_reset_allowed())
		return false;

	dev_warn(&udev->dev, "%s, resetting chip (%s)\n", why, port);

	snprintf(smit_pending_port, sizeof(smit_pending_port), "%s", port);
	smit_pending_ok = false;
	smit_reset(udev);
	/* 必须用 mod_ 而不是 schedule_：schedule_delayed_work 对已经排队的
	 * work 直接返回 false，不会重新计时。两次复位相隔不到 20s 时，
	 * 第二次复位的观察窗口会被第一次的残留计时截短，慢一点的设备
	 * 就会被误判成"昂贵"——而错误判定的后果是以后再也不复位它。 */
	mod_delayed_work(system_wq, &smit_reset_watch,
			 msecs_to_jiffies(SMIT_VERDICT_MS));
	return true;
}

/* ---- 芯片状态判定（两个遗留问题的共同解）-----------------------------
 *
 * 目标：热芯片（已经初始化过一次）不要走"复位 -> 重新枚举 7s -> 再 init 7s"。
 *
 * 已被证伪的手段：
 *   - 设备描述符 / 字符串 / vendor control 请求：冷热完全一样
 *     （bcdDevice=0200、iProduct="DONGLE"、几个 0xC0 请求一律 -11），零区分度。
 *
 * 探活（ping 0x82）现在放在两处，含义不同：
 *   - module_init：usb_register 之前先问一句。有应答就记下 devnum，probe 看到
 *     同一个 devnum 直接注册（~0s）。这时还没注册 usb_driver，就算要复位也
 *     不会引起任何 probe，不可能循环。
 *   - probe：拿不到凭据时（重新枚举 / 拔插 / reset 之后）再问一次。
 *
 * 曾经踩过的坑，现在的代码只是绕开、没有根除：早期在 probe 里探活之后
 * **立刻发唤醒包**，冷芯片收不到应答时探针的 5 个字节还留在它的 OUT FIFO 里，
 * 唤醒包 FE 00 10 00 被拼成畸形命令，握手必然失败（实测 4s / 10 次重发都冲
 * 不掉），于是 probe 失败 -> 复位 -> 重枚举 -> 再探测 -> 再污染，无限循环。
 * smit_flush_in() 只抽 IN，清不掉 OUT 的这一截。
 *
 * **未验证**：冷芯片在 ping 弄脏 FIFO 之后、不复位直接 wake 到底能不能成功。
 * 我们记录到的两次冷启动里，一次是 8.06s（insmod 时设备还没枚举完，
 * module_init 根本没找到设备，也就没 ping），另一次 ping 后面跟着一次复位
 * （reset_on_cold 那条分支，节点 130.2s 才出来）。"ping 过 + 不复位 + wake"
 * 这个组合没有数据，所以 reset_on_cold 默认仍然是开。
 */
static int smit_pick_device(struct usb_device *udev, void *data)
{
	struct usb_device **found = data;

	if (udev->descriptor.idVendor == USB_VID_SMIT &&
	    udev->descriptor.idProduct == USB_PID_SMIT_ICAST) {
		*found = usb_get_dev(udev);
		return 1;	/* 找到就停 */
	}
	return 0;
}

/* ---- 运行时自愈 -------------------------------------------------------
 *
 * 老驱动最讨厌的毛病是"跑一阵子突然不工作了，必须拔掉 USB 重新插"。
 * 三个结构性原因都在命令通道上：没有互斥（两个调用者的包交错）、
 * stall 后从不 clear_halt（之后每次传输都立刻 -EPIPE）、轮询重试没有时间上限
 * （一条 ioctl 卡几十秒）。前两个在 smit.c 里修了，这里补最后一环：
 *
 * 连续 SMIT_RECOVER_STREAK 次事务在传输层失败，就当芯片真的卡死了，
 * 发一次硬件复位让它重新枚举。复位后 device 会 disconnect 再 probe，
 * 因为这里把 smit_warm_devnum 清成 -1，probe 会走完整的冷启动流程
 * （唤醒 + 初始化），节点自己回来 —— 不需要人拔插。
 *
 * 必须放在 workqueue 里做：复位会触发 disconnect -> dvb_usb_device_exit，
 * 如果在 frontend ioctl 的上下文里同步做，等于在自己还没退出的时候就
 * 把自己的设备拆掉，风险太大。异步做就是把这件事交还给 USB 核心。
 */
static struct usb_device *smit_recover_udev;
static atomic_t smit_recover_queued = ATOMIC_INIT(0);

static void smit_recover_work_fn(struct work_struct *work)
{
	struct usb_device *udev = smit_recover_udev;

	smit_recover_udev = NULL;

	if (udev) {
		if (udev->state != USB_STATE_NOTATTACHED) {
			pr_warn("dvb_usb_smit: command channel dead, attempting a recovery reset\n");
			/* 复位后 devnum 会变，下一次 probe 因此不会命中"热芯片复用"，
			 * 而是走完整冷启动。 */
			smit_warm_devnum = -1;
			smit_warm_busnum = -1;
			/* 所有复位走同一个入口，代价判定才管得住这里。
			 * 原来这里直接 smit_reset()：钉死成 reset_costly_ports 的
			 * 端口照样被复位，而那支棒子实测要 806s（十三分钟）才回
			 * 总线，且软复位救不回来（必须物理断电）——等于用十三分钟
			 * 换一个还是不通的设备。拒绝复位时把话说明白，让人去断电，
			 * 比静默熬着强。 */
			if (!smit_try_reset(udev, "command channel dead"))
				pr_warn("dvb_usb_smit: not resetting this port; power-cycle the dongle to recover\n");
		}
		usb_put_dev(udev);
	}
	atomic_set(&smit_recover_queued, 0);
}

static DECLARE_WORK(smit_recover_work, smit_recover_work_fn);

void smit_schedule_recovery(struct usb_device *udev)
{
	if (!smit_autorecover)
		return;
	if (atomic_xchg(&smit_recover_queued, 1))
		return;		/* 已经排了一个，别叠 */

	usb_get_dev(udev);
	smit_recover_udev = udev;
	if (!schedule_work(&smit_recover_work)) {
		smit_recover_udev = NULL;
		usb_put_dev(udev);
		atomic_set(&smit_recover_queued, 0);
	}
}

static int smit_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	int ret;

	/* 从这里到 out: 之间所有命令的传输层失败都不许触发自愈复位，
	 * 否则会和下面 probe 自己的复位逻辑打架 —— 详见 smit_note()。 */
	smit_in_probe = true;

	if (smit_diag_probe)
		smit_diag(udev);

	if (smit_warm_devnum == (int)udev->devnum &&
	    smit_warm_busnum == (int)udev->bus->busnum) {
		dev_info(&udev->dev,
			 "chip is already initialized (bus %d devnum %d), skipping wake + init\n",
			 udev->bus->busnum, udev->devnum);
		smit_warm_devnum = -1;	/* 只复用一次 */
		smit_warm_busnum = -1;
		goto register_device;
	}

	if (smit_skip_wake)
		goto do_init;

	/* 先探活，再决定要不要发唤醒包。
	 *
	 * 唤醒包 { FE 00 10 00 } 只对"还在 bootloader 里的冷芯片"有意义，
	 * 对已经初始化过的热芯片是**毒药**：它不应答，而且之后的命令全部超时
	 * （实测两支棒子一模一样：2 次尝试 / 1130~1140ms / 最后 -110）。
	 * 而这颗芯片在整机里是 VBUS 常供的（xhci-mtk 没有 vbus regulator），
	 * 所以 USB 掉线重连、驱动 unbind/bind 这些场景里芯片**一直是热的** ——
	 * 旧代码在这些场景里每次都发唤醒包、每次都中毒，然后靠复位收场：
	 * 便宜的那支付 6.5s，贵的那支实测 806s。
	 *
	 * 判据就用初始化序列的第一条命令（req 0x82 只读查询）：热芯片 0ms 就回。
	 * 没应答时先抽干 IN 再问一次，避免把上一条探针的残留应答当成这次的答案。 */
	ret = smit_ping(udev);
	/* 只在"没应答"时重问一次：先抽干 IN，避免把上一条探针的残留当成
	 * 这一次的答案。主机侧的瞬时错误（-ENOMEM 之类）重问也没有意义，
	 * 它们不构成"冷"的证据，见 smit_ping_is_cold()。 */
	if (smit_ping_is_cold(ret)) {
		smit_flush_in(udev);
		ret = smit_ping(udev);
	}

	if (ret == -ENODEV || ret == -ESHUTDOWN) {
		dev_info(&udev->dev,
			 "device is going away (%d), leaving it to re-enumerate\n",
			 ret);
		goto out;
	}

	if (!smit_ping_is_cold(ret)) {
		/* 有应答（哪怕是协议级怪数据），或者错误根本不是"没回话"那一类：
		 * 按热芯片处理，只注册、不动芯片 —— 唤醒包对热芯片是毒药。
		 * 拿不到 tuner 信息，dtmb 用默认的双模配置。 */
		dev_info(&udev->dev,
			 "chip looks alive (ping %d), skipping wake + init\n",
			 ret);
		goto register_device;
	}

	/* 冷路径：唤醒包要发几次。
	 * 实测 USB 重新授权/重新枚举之后，芯片要过一两秒才"听得懂"唤醒包；
	 * 只发一次的旧逻辑会立刻判死然后复位（复位在贵的批次上是分钟级）。
	 * 这里给 4 次机会（每次 500ms 超时 + 200ms 间隔，最坏 ~4.5s），
	 * 每次都先抽干 IN，免得读到上一次的残留。 */
	{
		int i;

		for (i = 0; i < 4; i++) {
			if (i)
				msleep(200);
			smit_flush_in(udev);
			ret = smit_wake(udev);
			if (!ret || (ret != -EPIPE && ret != -ETIMEDOUT))
				break;
			dev_info(&udev->dev, "wake attempt %d failed (%d), retrying\n",
				 i + 1, ret);
		}
	}
	if (ret) {
		/* 唤醒包也不理：芯片卡在既不在应用模式、又听不懂唤醒的中间态。
		 * 只能靠硬件复位，但那代价在有的批次上是分钟级，所以交给
		 * smit_try_reset() 判定，它会记住这支设备负担不负担得起。
		 *
		 * 但 -ENODEV / -ESHUTDOWN 是另一回事：设备正在复位/重枚举，
		 * 这时候再发一次复位只会把重枚举又拖一遍。直接放手，
		 * 等 USB core 在设备重新连上之后再 probe 一次。 */
		if (ret == -ENODEV || ret == -ESHUTDOWN) {
			dev_info(&udev->dev,
				 "device is going away (%d), leaving it to re-enumerate\n",
				 ret);
			goto out;
		}
		/* 只有"总线真的挂了"才值得复位。smit_wake 里 kmalloc 失败会返回
		 * -ENOMEM，那种情况一个字节都没发出去，复位纯属白搭（在贵的批次上
		 * 是分钟级），还可能把一颗好芯片重启掉。 */
		if (!smit_is_bus_fault(ret)) {
			dev_info(&udev->dev,
				 "wake failed with %d, not a bus fault; leaving the chip alone\n",
				 ret);
			goto out;
		}
		if (smit_try_reset(udev, "device not responding to the wake packet"))
			dev_info(&udev->dev, "device not responding (%d), reset to re-enumerate\n",
				 ret);
		goto out;
	}

	/* 唤醒包只是让芯片从 bootloader 跳到应用固件，它回的 ACK 只代表
	 * "这条唤醒命令收到了"，不代表"固件已经能处理业务命令"。
	 * 实测撞过这个空窗：wake answered 0ms 之后 560ms 内 cmd0 仍然超时，
	 * 整套 init 全 -110，然后触发自愈复位、再撞、开始循环。
	 * 固件启动时间在软复位后尤其不稳，所以这里留一个可调的 settle 间隔。
	 * 代价只在冷路径上（插拔 / 复位后），热复用完全不受影响。 */
	/* settle_ms 是 0644 可写的，负数会变成天文数字：msleep 的入参是
	 * unsigned int，-1 就是 4294967295ms（约 49 天）。顺手夹一下。 */
	if (smit_settle_ms > 0)
		msleep(smit_settle_ms > 5000 ? 5000 : (unsigned int)smit_settle_ms);
do_init:
	ret = smit_init(udev, &icast_config.dtmb);
	if (smit_is_transport_err(ret)) {
		dev_err(&udev->dev, "init failed at USB level: %d\n", ret);
		/* 设备正在消失时不要补一刀复位，理由同上：复位只会把重枚举
		 * 再拖一遍。wake 分支已经这么处理了，init 这里漏了。 */
		if (ret == -ENODEV || ret == -ESHUTDOWN) {
			dev_info(&udev->dev, "device is going away (%d)\n", ret);
			goto out;
		}
		/* 实验模式下不要复位：否则同一次 insmod 里会不断
		 * 复位→重枚举→再失败，变成死循环。
		 * 复位只对总线级故障有意义：-ENOMEM 这类主机侧失败
		 * 一个字节都没发出去，复位纯属白搭。 */
		if (!smit_skip_wake && smit_is_bus_fault(ret))
			smit_try_reset(udev, "init sequence failed at bus level");
		goto out;
	}
	if (ret)
		/* 协议级错误（设备回了怪数据）上游一直是忽略的，
		 * 这里只记录，免得把本来能用的设备判死 */
		dev_info(&udev->dev, "init sequence reported %d, continuing\n", ret);

register_device:
	ret = dvb_usb_device_init(intf, &icast_properties, THIS_MODULE, NULL,
				  adapter_nr);
	if (ret == 0) {
		/* 一次成功的 probe 说明链路是好的，把自愈计数清零 */
		smit_reset_attempts = 0;
		/* 也是"复位代价判定"的正样本：复位后这么久之内能 probe 成功，
		 * 说明这支设备的复位是便宜的（smit_reset_watch_fn 据此下结论）。
		 * 必须核对端口：pending 是单槽的，A 复位后紧跟着 B 又复位，
		 * 随后 A probe 成功 —— 原来会把 B 的判决也刷成"便宜"，
		 * 而那可能是要等十几分钟的那支。 */
		if (smit_pending_port[0]) {
			char port[SMIT_PORT_LEN];

			smit_portname(udev, port, sizeof(port));
			if (!strncmp(smit_pending_port, port, SMIT_PORT_LEN - 1))
				smit_pending_ok = true;
		}
		goto out;
	}

	dev_err(&udev->dev, "dvb_usb_device_init failed: %d\n", ret);
	/* 这里失败多半是主机侧的事（frontend attach 失败、内存不够、
	 * adapter/demux 注册失败），复位芯片解决不了，却要付分钟级的代价。
	 * 只有总线级故障才值得补这一刀。 */
	if (smit_is_bus_fault(ret))
		smit_try_reset(udev, "dvb_usb_device_init failed at bus level");
out:
	smit_in_probe = false;
	return ret;
}

static void smit_disconnect(struct usb_interface *intf)
{
	struct usb_device *udev = interface_to_usbdev(intf);

	/* 设备走了，"这是个热芯片"的凭据就作废。
	 * devnum 是 USB 核心回收复用的：留着它的话，下一次插上来的冷芯片
	 * 完全可能拿到同一个号，probe 就会误判成热芯片、跳过唤醒和初始化，
	 * 节点出来了但前端全哑（每条命令都超时），再靠自愈绕一圈才恢复。
	 * 自愈路径上本来自己会清，但外部拔插不会，所以必须在这里清。 */
	smit_warm_devnum = -1;
	smit_warm_busnum = -1;

	dvb_usb_device_exit(intf);

	/* 默认不复位芯片。
	 * 以前这里复位是为了"让下一次 insmod 的第一次 probe 是干净的"，代价是
	 * 设备重新枚举 7s；现在 probe 会先探活，热芯片直接复用，重新枚举纯属浪费。
	 * 需要验证冷启动路径时用 reset_on_exit=1 手动复位。 */
	if (smit_reset_on_exit)
		dev_info(&udev->dev, "reset on exit: %d\n", smit_reset(udev));
}

static struct usb_driver smit_driver = {
	.name = KBUILD_MODNAME,
	.probe = smit_probe,
	.disconnect = smit_disconnect,
	.id_table = icast_id_table,
};

/* 重启/关机不再需要任何动作：probe 探测到热芯片会直接复用，不再有
 * "开机第一次 probe 失败"这回事，reboot_notifier 已删除。 */

static int __init smit_module_init(void)
{
	struct usb_device *udev = NULL;
	int result, ret;

	/* 加载时总线上可能已经挂着 dongle（insmod 重载、开机时开机脚本加载）。
	 * 趁还没注册 usb_driver 先问一次芯片状态，见 smit_probe 上面的说明：
	 *   - 热芯片：记下 devnum，probe 直接注册，不再复位重枚举
	 *   - 冷芯片：探针弄脏了 FIFO，就地复位（此时没有 driver，不会引发 probe） */
	usb_for_each_dev(&udev, smit_pick_device);
	if (udev) {
		/* 探针失败在这里是**预期**结果（冷芯片本来就该超时），
		 * 不能让它记进链路失败计数，更不能顺势触发自愈复位。 */
		smit_in_probe = true;
		ret = smit_ping(udev);
		smit_in_probe = false;
		if (ret == -ENOMEM) {
			/* 主机侧 kmalloc 失败，完全没碰到设备。既不能当"热芯片"
			 * 复用（没验证过），也不能当"冷芯片"去复位（芯片可能是好的，
			 * 白搭 7s 重枚举）。放过去，让 probe 走完整的唤醒+init。 */
			pr_info("dvb_usb_smit: ping out of memory, leaving it to probe\n");
		} else if (!smit_is_transport_err(ret)) {
			smit_warm_devnum = udev->devnum;
			smit_warm_busnum = udev->bus->busnum;
			pr_info("dvb_usb_smit: chip already initialized (bus %d devnum %d), will reuse it\n",
				udev->bus->busnum, udev->devnum);
		} else {
			int i;

			/* 冷芯片：探针可能弄脏了 FIFO，复位一次换来干净的重枚举。
			 * 但这只在 reset_on_cold=1 时才做 —— 复位代价实测 6.5s~806s
			 * （同一行代码，同一型号，离散度 100 倍以上），而 probe 对
			 * 冷芯片走 wake(最多 4 次) + init 只要 ~3s，完全不需要复位。
			 * 真机实测：冷启动（整机断电上电）时本分支复位 1-1，节点从
			 * 18.0s 被拖到 130.2s 才出现，其中 108s 全花在等重枚举。
			 * 默认交给 usb_register 之后的 probe（它先 ping 再决定唤醒）。 */
			if (smit_reset_on_cold &&
			    smit_try_reset(udev, "cold chip, cleaning the FIFO")) {
				/* 复位是"发完命令才重启"，实测要 ~0.8s 才真正从
				 * 总线上消失。如果这时就去 usb_register，probe 会撞上
				 * 一个正在重启的设备（-ENODEV），再触发一次复位。
				 * 等它断开再注册：重连时会自动 probe 到干净的冷芯片。 */
				for (i = 0; i < 30; i++) {
					if (udev->state == USB_STATE_NOTATTACHED)
						break;
					msleep(50);
				}
			} else {
				char port[SMIT_PORT_LEN];

				smit_portname(udev, port, sizeof(port));
				pr_info("dvb_usb_smit: cold chip (ping %d) on %s, will let probe wake it\n",
					ret, port);
			}
		}
		usb_put_dev(udev);
	}

	result = usb_register(&smit_driver);
	if (result) {
		err("usb_register failed. Error number %d", result);
		return result;
	}
	return 0;
}

static void __exit smit_module_exit(void)
{
	/* 顺序很重要：先摘驱动，之后不会再有 probe/ioctl 进来排新的自愈任务。
	 * 反过来（先 cancel 再 deregister）留着一个口子：cancel 返回之后到
	 * deregister 之间还可能排进一个新 work，它会在模块代码已经卸载之后
	 * 才执行 —— 直接跳进已释放的代码段。 */
	usb_deregister(&smit_driver);

	/* cancel_work_sync 返回 true 表示任务是被取消的，fn 根本没跑，
	 * 那它持有的 usb 引用就得我们自己还。不还的话这个 usb_device 的
	 * 引用计数永远多一份：设备拔掉后结构体释放不掉，smit_recover_queued
	 * 也停在 1。正在执行的情况它会等 fn 跑完，fn 自己会 usb_put_dev。 */
	if (cancel_work_sync(&smit_recover_work)) {
		if (smit_recover_udev) {
			usb_put_dev(smit_recover_udev);
			smit_recover_udev = NULL;
		}
		atomic_set(&smit_recover_queued, 0);
	}

	/* 复位代价判定那条定时器只在内存里下结论，不持有 usb 引用，
	 * 但必须在模块代码消失之前取消，否则 20s 后会跳进已卸载的段。 */
	cancel_delayed_work_sync(&smit_reset_watch);
}

module_init(smit_module_init);
module_exit(smit_module_exit);

MODULE_DESCRIPTION("SMIT iCast dvb driver");
MODULE_AUTHOR("Xiaodong Ni <nxiaodong520@gmail.com>");
MODULE_LICENSE("GPL");

