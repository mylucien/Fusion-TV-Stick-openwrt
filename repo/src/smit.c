// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SMIT iCast DTMB/DVC driver
 *
 * Copyright (C) 2026 Xiaodong Ni <nxiaodong520@gmail.com>
 */

#include "dvb-usb.h"
#include "smit.h"
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/moduleparam.h>
#include <linux/version.h>

/* 单次 bulk 传输的超时。设备正常工作时应答是 0ms 级的，
 * 这个值只在链路出问题时才会被真正等满。 */
#define SMIT_BULK_TIMEOUT_MS	500

/* 冲刷 IN 管道时用的超时：只要读不到东西就认为已经干净了 */
#define SMIT_DRAIN_TIMEOUT_MS	40

/* 完成标志轮询的总预算 = 基础值 + checkcount*10ms，且不超过上限。
 * 原来是纯次数上限（checkcount 最大 300），每次重试含两个 500ms 超时，
 * 链路一坏就能把一个 ioctl 卡几十秒，用户看到的就是"整个 dvb 死住了"。 */
#define SMIT_CHECK_BASE_MS	1000
#define SMIT_CHECK_MAX_MS	6000

/* 连续多少次"整条事务"在传输层失败就认为芯片卡死，触发自愈复位 */
#define SMIT_RECOVER_STREAK	3

/* 用 u8 而不是 char：aarch64 上 char 默认无符号，比较刚好能碰对，
 * 但 x86 上 char 有符号，data[i] == 0x80 会变成 -128 == 128，永远为假，
 * "完成标志"判断直接失效。目标平台虽然只有 aarch64，也没理由留这种地雷。 */
struct st_smit_pack {
	u8 tag;
	u8 byte2;
	u8 req;
	u8 datalen;
	u8 data[1000];
} typedef SMIT_PACK;

/* 必须用 kmalloc 的缓冲区转一手，不能把调用方的 buffer 直接交给 hcd：
 * hcd_map_urb_for_dma() 对栈上的 transfer_buffer 会 WARN_ONCE 并返回
 * -EAGAIN（object_is_on_stack，与 VMAP_STACK 无关，只要主机控制器用 DMA
 * 就一定命中）。真机实测：直接传栈数组时 usb_bulk_msg 返回 -11，
 * 请求根本没发出去，看起来像"设备没应答"，把热芯片误判成冷芯片。 */
int smit_usb_bulk_msg(struct usb_device *usb_dev, unsigned int pipe, void *data, int len, int *actual_length, int timeout)
{
  void *buf;
  int ret;

  buf = (void *)kmalloc(len, GFP_KERNEL);
  if ( !buf )
    return -ENOMEM;
  memcpy(buf, data, len);
  ret = usb_bulk_msg(usb_dev, pipe, buf, (unsigned int)len, actual_length, timeout);
  /* IN 方向的数据是 URB 直接 DMA 进 buf 的。设备回的数据比缓冲区多时
   * 内核只报 -EOVERFLOW，包其实已经收到了，actual_length 也有效
   * （usb_start_wait_urb 无条件回填），这时候必须照样拷回去。
   * 原来只在 ret >= 0 时拷，EOVERFLOW 就会把**自己发出去的请求包**
   * 当成设备的应答交给调用方解析。 */
  if (ret >= 0 || ret == -EOVERFLOW)
  {
    if (usb_pipein(pipe))
      memcpy(data, buf, len);
  }
  kfree(buf);
  return ret;
}

/* ---- 命令通道 ---------------------------------------------------------
 *
 * 一条业务命令 = "往 0x01 写 5+N 字节，再从 0x82 读回一包"，两步构成一个事务。
 * 这两步必须在一次独占访问里完成：两个调用者一旦并发（比如播放器在轮询
 * FE_READ_STATUS，同时另一个进程在调 FE_SET_FRONTEND），OUT 包会交错，
 * 读回的是对方的应答，tag/长度对不上，之后整条通道就永久错位。
 * 真机上的表现就是"跑一段时间不定时挂死，拔插才好"。
 */
static DEFINE_MUTEX(smit_cmd_lock);

/* 上一次事务在传输层出过问题（超时 / stall / tag 错位），说明 IN 管道里
 * 可能还躺着半包或者上一条命令的旧应答。下一次事务开始前先冲干净，
 * 否则会一直读到错位的数据。 */
static bool smit_resync_pending;

static atomic_t smit_err_streak = ATOMIC_INIT(0);

/* 计数器用原子量：read_status 是每 100ms 一次的高频调用，同一 fd 上多个
 * 线程并发 ioctl 时，普通 long 的 ++ 是 read-modify-write，会丢更新，
 * 让 stats 数字比实际偏小 —— 而这几个数字正是排查"挂在哪一层"的唯一依据。 */
static atomic_long_t smit_stat_ok = ATOMIC_LONG_INIT(0);
static atomic_long_t smit_stat_timeout = ATOMIC_LONG_INIT(0);
static atomic_long_t smit_stat_epipe = ATOMIC_LONG_INIT(0);
static atomic_long_t smit_stat_resync = ATOMIC_LONG_INIT(0);
static atomic_long_t smit_stat_recover = ATOMIC_LONG_INIT(0);

int smit_reset(struct usb_device *udev)
{
  return usb_control_msg(udev, usb_sndctrlpipe(udev, 0), 160, 64, 0, 0, 0, 0, 500);
}

/* 端点 stall（-EPIPE）之后必须 clear_halt，否则之后每一次传输都会立刻
 * 返回 -EPIPE，谁也救不回来 —— 这是"驱动挂死只能拔插"最典型的成因。
 *
 * 但 clear_halt 的代价极高：它是一条 CLEAR_FEATURE 控制请求，不管端点
 * 到底有没有真的 halt 都会发（见 ch9/message.c 的注释），超时用
 * USB_CTRL_SET_TIMEOUT = 5s，芯片连 control 都不应答时必然走满。
 * 所以只在"确实还会再重试一次"时才清，且只清出错的那个端点。
 */
static int smit_bulk_transfer(struct usb_device *udev, unsigned int pipe,
			      void *data, int len, int *actual_len)
{
	int ret, actual_length = 0, tries = 0;

	for (;;) {
		ret = smit_usb_bulk_msg(udev, pipe, data, len,
					&actual_length, SMIT_BULK_TIMEOUT_MS);
		if (!ret)
			break;
		/* 设备给的数据比缓冲区多只会报 EOVERFLOW，包其实已经收到了 */
		if (ret == -EOVERFLOW) {
			ret = 0;
			break;
		}
		if (ret != -EPIPE)
			break;
		atomic_long_inc(&smit_stat_epipe);
		/* 最后一次失败之后再清一遍 halt 没有任何意义：
		 * 反正不会再发了，只是白等 5s（原来会在这里多烧一整个超时）。 */
		if (++tries >= 2)
			break;
		usb_clear_halt(udev, pipe);
	}

	if (ret == -ETIMEDOUT) {
		atomic_long_inc(&smit_stat_timeout);
		smit_resync_pending = true;
	}

	if (actual_len)
		*actual_len = actual_length;
	return ret;
}

static int smit_bulk_out(struct usb_device *udev, void *data, int len)
{
	return smit_bulk_transfer(udev,
		usb_sndbulkpipe(udev, SMIT_BULK_ENDPOINT_CMD_SEND),
		data, len, NULL);
}

static int smit_bulk_in(struct usb_device *udev, void *data, int len, int *actual_len)
{
	return smit_bulk_transfer(udev,
		usb_rcvbulkpipe(udev, SMIT_BULK_ENDPOINT_CMD_RECEIVE),
		data, len, actual_len);
}

/* 把 IN 管道里残留的包全部读掉，让下一条命令从干净的边界开始。
 * 只在事务之间调用（持有 smit_cmd_lock），不会吃掉在飞的应答。
 *
 * 这里**不能**无条件 smit_clear_halt()：usb_clear_halt() 内部是两条
 * CLEAR_FEATURE 控制请求，超时用的是 USB_CTRL_GET_TIMEOUT（5s），而芯片
 * 不应答时它一定走满 5s。实测：init 里 cmd3 花 5580ms、cmd4 花 10630ms，
 * 正好是「1 次 clear_halt + 一次 560ms 的 IN 超时」和「2 次 + 560ms」。
 * 触发 resync 的原因是**传输超时**，端点并没有 halt，清 halt 纯属自找 5s。
 * 真 stall(-EPIPE) 的路径在 smit_bulk_in/out 里已经单独处理了。 */
static void smit_drain_in(struct usb_device *udev)
{
	u8 *buf;
	int i, actual_length, ret;

	smit_resync_pending = false;

	buf = kmalloc(1024, GFP_KERNEL);
	if (!buf)
		return;

	for (i = 0; i < 32; i++) {
		ret = usb_bulk_msg(udev,
			usb_rcvbulkpipe(udev, SMIT_BULK_ENDPOINT_CMD_RECEIVE),
			buf, 1024, &actual_length, SMIT_DRAIN_TIMEOUT_MS);
		if (ret == -EOVERFLOW)
			continue;	/* 收到了，继续抽 */
		if (ret)
			break;		/* 超时 / stall：认为已经空了 */
	}
	kfree(buf);
	atomic_long_inc(&smit_stat_resync);
}

static void smit_prepare(struct usb_device *udev)
{
	if (smit_resync_pending)
		smit_drain_in(udev);
}

/* 给 probe 用的公开版本：判完冷热、重发唤醒包之前先把 IN 抽干净。
 * 必须持有 smit_cmd_lock 之外调用 —— probe 阶段没有并发 ioctl，
 * 也不会有 in-flight 的应答，所以这里不需要加锁。 */
void smit_flush_in(struct usb_device *udev)
{
	smit_drain_in(udev);
}

/* 公共入口统一在这里判定链路健康度：连续 N 次事务在传输层失败，
 * 说明芯片已经卡死，交给上层复位重新枚举（不用拔插）。
 * 协议级错误码（-1xxx，设备回了怪数据）不算链路故障，不触发复位。 */
static void smit_note(struct usb_device *udev, int ret)
{
	if (ret >= 0) {
		atomic_long_inc(&smit_stat_ok);
		atomic_set(&smit_err_streak, 0);
		return;
	}
	/* -ENOMEM 是主机侧分配失败，跟链路没关系。把它算成链路故障的话，
	 * 连续几次内存紧张就能把好芯片复位掉；module_init 里的探活更糟：
	 * ping 返回 -ENOMEM 会被当成"冷芯片"，白复位一次再等 7s 重枚举。 */
	if (!smit_is_transport_err(ret) || ret == -ENODEV ||
	    ret == -ESHUTDOWN || ret == -ENOMEM)
		return;

	/* probe / smit_init 正在跑的时候不能触发自愈。
	 * 冷启动的 init 有二十多条命令，芯片启动慢一点就会连续几条超时，
	 * 这时如果让 workqueue 插进来复位，等于把还没注册完的设备拆掉：
	 * probe 后续命令全 -ENODEV -> probe 失败 -> 又复位一次 -> 循环。
	 * init 期间的失败由 probe 自己统一判断，不归这里管。 */
	if (smit_in_probe)
		return;

	if (atomic_inc_return(&smit_err_streak) >= SMIT_RECOVER_STREAK) {
		atomic_set(&smit_err_streak, 0);
		atomic_long_inc(&smit_stat_recover);
		smit_schedule_recovery(udev);
	}
}

static int __smit_check(struct usb_device *udev, int checkcount)
{
  unsigned int budget = SMIT_CHECK_BASE_MS + (unsigned int)checkcount * 10;
  unsigned long deadline;
  int cnt;
  int ret;

  if (budget > SMIT_CHECK_MAX_MS)
    budget = SMIT_CHECK_MAX_MS;
  deadline = jiffies + msecs_to_jiffies(budget);

  cnt = 0;

  while ( 1 )
  {
    SMIT_PACK smit = {.tag = 0x01, .byte2 = 0x0, .req = 0xA0, .datalen = 0x01, .data[0] = 0x01};

    /* 每次尝试都可能等满两个超时，所以预算在循环头上判，不能只在末尾判 */
    if ( time_after_eq(jiffies, deadline) )
      break;

    ret = smit_bulk_out(udev, &smit, 5);
    if ( ret )
    {
      return ret;
    }
    ret = smit_bulk_in(udev, &smit, 1000, NULL);
    if ( ret )
    {
      return ret;
    }
    if ( smit.tag != 1 )
    {
      smit_resync_pending = true;
      return -1003;
    }
    if ( smit.data[1] == 0x80 )
    {
      return 0;
    }
    if (smit.data[1] )
    {
      return -1006;
    }
    ++cnt;
    if ( checkcount <= 0 )
      return 0;
    if ( cnt > checkcount )
      break;
    msleep((unsigned int)(cnt / 2 + 3));
  }
  /* 归到协议级错误码区间（< -1000），别和 USB 传输层的 errno 混在一起：
   * 调用方靠这个区间判断是该重试/复位，还是照旧忽略。 */
  return -(1000 + checkcount);
}

static int __smit_set(struct usb_device *udev, u8 req, u8 *data, unsigned int len, int need_complete_flag)
{
  int ret, actual_length;
  SMIT_PACK smit = {.tag = 0x01, .byte2 = 0x0, .req = req, .datalen = (u8)len};

  /* 真正有效的防线。原来写的是 len > sizeof(smit.data)，而 len 是 u8
   * 参数：最大 255，永远不可能大于 1000，这个检查形同虚设。
   * 有两处硬上限必须同时挡住：
   *   - data[1000] 的物理大小
   *   - datalen 字段是 u8，超过 255 会被静默截断成另一个长度 */
  if (len > sizeof(smit.data) || len > 0xff)
    return -EINVAL;

  smit_prepare(udev);

  actual_length = 0;

  memcpy(smit.data, data, len);

  ret = smit_bulk_out(udev, &smit, len + 4);
  if ( !ret )
  {
    ret = smit_bulk_in(udev, &smit, 1000, &actual_length);
    if ( ret )
    {
      return ret;
    }
    if (smit.tag != 1 )
    {
      smit_resync_pending = true;
      return -1004;
    }
    if (smit.req == 0x80 && smit.data[1] == 0x80 )
    {
        return 0;
    }
    if ( (smit.datalen + 4) >= actual_length || smit.data[(u8)smit.datalen] != 0x80 || smit.data[(u8)smit.datalen + 3] != 0x80 )
    {
      if ( need_complete_flag )
      {
        /* 原先这里先 msleep(100) 再去轮询完成标志，一整套初始化序列要付
         * 10 次 = 1s 的固定开销。smit_check 自己就是从 3ms 起步的退避轮询，
         * 直接进去等更划算，芯片真有活干也等得住。 */
        return __smit_check(udev, need_complete_flag);
      }
    }
    return 0;
  }
  return ret;
}

static int __smit_tuner_stat(struct usb_device *udev, u8 *out, int checkcount)
{
  int ret, dlen, copy;
  int actual_length = 0;
  SMIT_PACK smit = {.tag = 0x01, .byte2 = 0x0, .req = 0x81, .datalen = 0x01, .data[0] = 0x01};

  smit_prepare(udev);

  ret = smit_bulk_out(udev, &smit, 5);
  if ( ret )
  {
    return ret;
  }

  ret = smit_bulk_in(udev, &smit, 900, &actual_length);
  if ( ret )
  {
    return ret;
  }
  if ( smit.tag != 1 )
  {
    smit_resync_pending = true;
    return -1003;
  }

  dlen = (int)(u8)smit.datalen;

  /* datalen 是设备自己报的，不能盲信：设备若报 32 而实际只回了 8 字节，
   * 按 datalen 拷贝就是把传感器没写过的字节当应答交给调用方，调用方还会
   * 读到 buf[29] 这种偏移。按实际收到的长度收紧。
   *
   * 收紧要按 actual_length - 4 算，不是 actual_length：IN 包里前面还有
   * 4 字节包头（tag / byte2 / req / datalen），有效载荷最多
   * actual_length - 4 字节。原来钳到 actual_length，每次都会多拷最多
   * 4 个字节 —— 那 4 个字节并不脏（SMIT_PACK 用的是指定初始化器，C 标准
   * 保证未列出的成员全部清零；smit_usb_bulk_msg 也是整包 memcpy 进
   * kmalloc 缓冲、DMA 只覆盖收到的部分，再整包拷回），错的是长度算错。 */
  copy = dlen;
  if (actual_length > 4) {
    if (copy > actual_length - 4)
      copy = actual_length - 4;
  } else {
    copy = 0;
  }
  memcpy(out, smit.data, copy);

  /* 完成标志的两个 0x80 是紧跟在**设备自报的载荷**后面的，所以定位必须
   * 用 dlen，不能用钳位后的 copy，否则偏移就整体错了。 */
  if ( ( dlen >= actual_length || smit.data[dlen] != 0x80 || smit.data[dlen + 3] != 0x80 ) && checkcount > 0)
  {
      int chk = __smit_check(udev, checkcount);

      /* 原来这个返回值直接丢掉了：完成标志轮询本身在传输层挂掉时
       * （芯片已经不回话）外面拿到的仍是"成功"，这次失败也就永远
       * 不计入 err_streak，自愈不会启动。 */
      if (chk < 0 && smit_is_transport_err(chk))
        return chk;
  }

  return copy;
}

/* 故障注入，只用来验证自愈链路：>0 时每条命令直接返回 -ETIMEDOUT，不碰硬件。
 * 设成 3 再调两次 FE_READ_STATUS（每次两条命令）就能凑满连续失败阈值，
 * 观察芯片是否会自动复位重新枚举、节点是否自己回来。默认 0，不影响正常行为。
 * 权限 0600：只有 root 能写，避免普通进程随手把链路"弄坏"。 */
static int smit_fault_test;
module_param_named(fault_test, smit_fault_test, int, 0600);
MODULE_PARM_DESC(fault_test, "test hook only: inject N transport failures");

static bool smit_fault_hit(struct usb_device *udev)
{
	if (smit_fault_test <= 0)
		return false;
	smit_fault_test--;
	smit_note(udev, -ETIMEDOUT);
	return true;
}

int smit_set(struct usb_device *udev, u8 req, u8 *data, unsigned int len, int need_complete_flag)
{
  int ret;

  if (smit_fault_hit(udev))
    return -ETIMEDOUT;

  mutex_lock(&smit_cmd_lock);
  ret = __smit_set(udev, req, data, len, need_complete_flag);
  mutex_unlock(&smit_cmd_lock);

  smit_note(udev, ret);
  return ret;
}

int smit_tuner_stat(struct usb_device *udev, u8 *out, int checkcount)
{
  int ret;

  if (smit_fault_hit(udev))
    return -ETIMEDOUT;

  mutex_lock(&smit_cmd_lock);
  ret = __smit_tuner_stat(udev, out, checkcount);
  mutex_unlock(&smit_cmd_lock);

  smit_note(udev, ret);
  return ret;
}

/* 下面这一串初始化命令的返回值上游一直是静默忽略的，设备回怪数据
 * （-1003/-1004/-1006）也能继续跑。为了让 probe 能区分"设备回了怪数据"
 * 和"USB 传输本身挂了"，这里把第一个负返回值记下来带出去，
 * 但流程照旧全部执行，不提前退出。
 *
 * 唯一例外是传输层失败（-110/-71/-19 这种）：芯片已经完全不回话了，
 * 剩下二十多条命令发出去也是每条白等一次超时（实测每条 560ms，二十条
 * 就是十几秒），然后 probe 才失败。这种情况下直接跳过剩余命令，
 * 让 probe 尽快返回去走复位重试。协议级错误（-1xxx）照旧继续跑。
 *
 * 记录错误时传输层错误优先：如果先记下的是 -1003（协议），后面链路
 * 彻底断了，只看"第一个负值"就会把 -110 全吞掉 —— 结果是六条命令
 * 全部白跑，返回 -1003，probe 判成"非传输错误，继续"，注册一个死设备。
 */
#define SMIT_DO(expr) do {						\
	int __r = 0;							\
	unsigned long __t0 = jiffies;					\
	if (!(err < 0 && smit_is_transport_err(err))) {			\
		__r = (expr);						\
		if (__r < 0 && (err >= 0 ||				\
		    (!smit_is_transport_err(err) &&			\
		     smit_is_transport_err(__r))))			\
			err = __r;					\
		if (smit_trace)						\
			pr_info("dvb_usb_smit:  %-46s %5ums r=%d\n", #expr,	\
				jiffies_to_msecs(jiffies - __t0), __r);	\
	} else if (smit_trace) {					\
		pr_info("dvb_usb_smit:  %-46s  skipped (link dead, %d)\n", \
			#expr, err);					\
	}								\
} while (0)

int smit_init(struct usb_device *udev, u8 *dtmb)
{
  int err = 0;
  int i;
  int n = 0;		/* cmd14 应答长度，0 表示没读到 */
  unsigned long t_all = jiffies;
  u8 cmd0[1]  = { 0x01 };
  u8 cmd3[10] = { 0x01, 0x92, 0x07, 0x00, 0x00, 0x01, 0x00, 0x41, 0x00, 0x01};
  u8 cmd4[9]  = { 0x01, 0x90, 0x02, 0x00, 0x01, 0x9F, 0x80, 0x10, 0x00 };
  u8 cmd5[37] = { 0x01, 0x90, 0x02, 0x00, 0x01, 0x9F, 0x80, 0x11, 0x1C, 0x00, 0x01, 0x00, 0x41, 0x00, 0x02, 0x00,
                  0x41, 0x00, 0x03, 0x00, 0x41, 0x00, 0x24, 0x00, 0x41, 0x00, 0x40, 0x00, 0x41, 0x00, 0x96, 0x10,
                  0x01, 0x00, 0x20, 0x00, 0x41 };
  u8 cmd6[10] = { 0x01, 0x92, 0x07, 0x00, 0x00, 0x96, 0x10, 0x01, 0x00, 0x03 };
  u8 cmd7[17] = { 0x01, 0x90, 0x02, 0x00, 0x03, 0x9F, 0x9A, 0x00, 0x08, 0x53, 0x4D, 0x69, 0x54, 0x5A, 0x42, 0x4A, 0x4C };
  u8 cmd8[10] = { 0x01, 0x92, 0x07, 0x00, 0x00, 0x96, 0x10, 0x01, 0x00, 0x04 };
  u8 cmd10[10] = { 0x01, 0x92, 0x07, 0x00, 0x00, 0x03, 0x00, 0x41, 0x00, 0x04 };
  u8 cmd11[9] = { 0x01, 0x90, 0x02, 0x00, 0x04, 0x9F, 0x80, 0x30, 0x00 };
  u8 cmd12[10] = { 0x01, 0x92, 0x07, 0x00, 0x00, 0x96, 0x10, 0x01, 0x00, 0x05 };
  u8 cmd9[32] = {
  	0x01, 0x90, 0x02, 0x00, 0x03, 0x9F, 0x9A, 0x07,
  	0x17,	0x00, 0x00, 0x14, 0x00, 0x03, 0x00, 0x10,
  	0xB8,	0xF3, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  	0x00,	0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00
  }; 
  u8 cmd14[20] = { 0x01, 0x90, 0x02, 0x00, 0x03, 0x9F, 0x9A, 0x07, 0x0B, 0x07, 0x00, 0x08, 0x10, 0x07, 0x00, 0x04, 0x47, 0x44, 0x48, 0x57};
  /* 必须清零。下面靠 strstr(buf + 18, "DTMB:Y") 判断双模/单模，而 strstr
   * 要求 NUL 结尾：设备正常最多回 255 字节（datalen 是 u8），但一旦
   * smit_tuner_stat 全部失败，这块栈数组就是未初始化的垃圾，strstr 会
   * 一路读到 1000 字节之外 —— 内核栈越界读，轻则把随机内存当成字符串，
   * 重则踩到 guard page 直接 oops。清零之后最坏也只是退化成单模。 */
  u8 buf[1000] = { 0, };

  SMIT_DO(smit_set(udev, 0x82, cmd0, 1, 100));
  SMIT_DO(smit_tuner_stat(udev, buf, 0));

  SMIT_DO(smit_set(udev, 0xA0, cmd3, 10, 0));

  SMIT_DO(smit_set(udev, 0xA0, cmd4, 9, 100));
  SMIT_DO(smit_tuner_stat(udev, buf, 0));

  cmd4[7] = 0x12;
  SMIT_DO(smit_set(udev, 0xA0, cmd4, 9, 100));
  SMIT_DO(smit_tuner_stat(udev, buf, 0));

  SMIT_DO(smit_set(udev, 0xA0, cmd5, 37, 100));
  SMIT_DO(smit_tuner_stat(udev, buf, 0));

  cmd3[5] = 0x02;
  cmd3[9] = 0x02;
  SMIT_DO(smit_set(udev, 0xA0, cmd3, 10, 0));

  cmd4[4] = 0x02;
  cmd4[7] = 0x20;
  SMIT_DO(smit_set(udev, 0xA0, cmd4, 9, 100));
  SMIT_DO(smit_tuner_stat(udev, buf, 0));
  SMIT_DO(smit_tuner_stat(udev, buf, 0));
  
  SMIT_DO(smit_set(udev, 0xA0, cmd6, 10, 1));

  SMIT_DO(smit_set(udev, 0xA0, cmd7, 17, 100));
  SMIT_DO(smit_tuner_stat(udev, buf, 100));
  SMIT_DO(smit_tuner_stat(udev, buf, 0));

  SMIT_DO(smit_set(udev, 0xA0, cmd8, 10, 300));
  SMIT_DO(smit_tuner_stat(udev, buf, 0));

  SMIT_DO(smit_set(udev, 0xA0, cmd14, 20, 100));

  /* cmd14 的应答里带 tuner 的能力串（"DTMB:Y" / "DTMB:N"），位置在
   * data+18 之后。这里有个隐蔽的前提：buf 是整个初始化序列复用的同一个
   * 数组，如果这条 tuner_stat 被跳过（链路已死）或者自己失败了，里面
   * 留着的是**上一条命令（cmd8）的应答**，strstr 必然匹配不到，于是
   * 双模芯片被判成单模，而 probe 只会打一句"continuing"，毫无提示。
   *
   * 处理办法：先清零，再只认明确读到的内容 —— 读到 "DTMB:N" 才降级成
   * 单模，读到 "DTMB:Y" 才升成双模，读不到就重试一次（命令通道丢一包
   * 是有的），仍然读不到才保持传入的默认值（双模 ops 是单模的超集，
   * 判错时代价最小）。 */
  for (i = 0; i < 2; i++)
  {
    if (i)
      SMIT_DO(smit_set(udev, 0xA0, cmd14, 20, 100));

    memset(buf, 0, sizeof(buf));
    SMIT_DO((n = smit_tuner_stat(udev, buf, 0)));
    if (n > 18 && (strstr((char *)buf + 18, "DTMB:Y") ||
                   strstr((char *)buf + 18, "DTMB:N")))
      break;
    n = 0;
  }

  if (n > 18)
  {
    if (strstr((char *)buf + 18, "DTMB:N"))
      *dtmb = 0;
    else if (strstr((char *)buf + 18, "DTMB:Y"))
      *dtmb = 1;
  }

  if (!*dtmb)
  {
    SMIT_DO(smit_set(udev, 0xA0, cmd10, 10, 0));
    
    SMIT_DO(smit_set(udev, 0xA0, cmd11, 9, 0));
    SMIT_DO(smit_tuner_stat(udev, buf, 0));
    SMIT_DO(smit_tuner_stat(udev, buf, 0));
    
    SMIT_DO(smit_set(udev, 0xA0, cmd12, 10, 100));
    SMIT_DO(smit_tuner_stat(udev, buf, 0));
  }
  else if (n <= 18)
  {
    /* tuner 信息两次都没读到，只能按默认的双模走：双模 ops 是单模的
     * 超集，注册不会出错。但 cmd10..cmd12 是单模（DVB-C 专用）的初始化，
     * 双模芯片本来也不发，这里一起跳过。
     *
     * 万一这真是块单模芯片、又恰好两次都丢了回包，DVB-C 那边就没做过
     * 初始化 —— 这是没有单模硬件时消不掉的取舍，所以至少要让它在 dmesg
     * 里可见。
     *
     * 但要分清两种"读不到"：链路在此之前就断了（传输层错误），
     * SMIT_DO 后面全是 skipped，n 自然是 0，这跟"读到了但认不出"是两回事。
     * 前者已经由 probe 统一处理（会去复位），这里再 warn 一次只会把
     * 排查方向带偏。 */
    if (err < 0 && smit_is_transport_err(err))
      pr_info("dvb_usb_smit: link failed (%d) before tuner info, "
              "keeping dual mode\n", err);
    else
      pr_warn("dvb_usb_smit: tuner mode unreadable (%d), assuming dual mode; "
              "DVB-C-only init was skipped\n", n);
  }

  /* 这条 cmd9 本身是"调谐"命令，初始化阶段频点全是 0，发出去芯片既不会
   * 调谐也不会回完成标志 —— 实测在这里白等 4390ms 后拿到 -1100（重试用尽）。
   * 上游同样忽略它的结果，所以只发不收完成标志。 */
  SMIT_DO(smit_set(udev, 0xA0, cmd9, 32, 0));
  SMIT_DO(smit_tuner_stat(udev, buf, 20));
  
  if (smit_trace)
    pr_info("dvb_usb_smit:  init sequence took %ums\n",
            jiffies_to_msecs(jiffies - t_all));

  return err;
}

#undef SMIT_DO

/* /sys/module/dvb_usb_smit/parameters/stats —— 链路健康度快照。
 * 以后再出现"不定时挂死"，先读这个就能看出挂在哪一层：
 *   timeout 在涨 -> 芯片不再应答（业务层卡住）
 *   epipe   在涨 -> 端点 stall 过，已被 clear_halt 救回来
 *   resync  在涨 -> 出现过错位，已被冲刷修正
 *   recover 在涨 -> 触发过自愈复位（芯片重新枚举过）
 * 四个都不动而画面临时断 -> 问题在 TS 数据流，不在这条命令通道。
 */
static int smit_stats_get(char *buffer, const struct kernel_param *kp)
{
	return sysfs_emit(buffer,
		"ok=%ld timeout=%ld epipe=%ld resync=%ld recover=%ld err_streak=%d\n",
		atomic_long_read(&smit_stat_ok),
		atomic_long_read(&smit_stat_timeout),
		atomic_long_read(&smit_stat_epipe),
		atomic_long_read(&smit_stat_resync),
		atomic_long_read(&smit_stat_recover),
		atomic_read(&smit_err_streak));
}

static const struct kernel_param_ops smit_stats_ops = {
	.get = smit_stats_get,
};

module_param_cb(stats, &smit_stats_ops, NULL, 0444);
MODULE_PARM_DESC(stats, "read-only link health counters");
