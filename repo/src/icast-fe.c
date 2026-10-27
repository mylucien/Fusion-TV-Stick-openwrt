// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SMIT iCast DTMB/DVC driver
 *
 * Copyright (C) 2026 Xiaodong Ni <nxiaodong520@gmail.com>
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 16, 0)
#include <dvb_frontend.h>
#else
#include <media/dvb_frontend.h>
#endif

#include "dvb-usb.h"
#include "smit.h"
#include "icast-fe.h"

//#define CONFIG_DYNAMIC_DEBUG

struct icast_state {
	struct dvb_usb_device *dev;
  struct dvb_frontend frontend;
  u32 frequency;
  u32 symbol_rate;
  enum fe_modulation	modulation;
  u8 tuned;		/* set_frontend 成功过，priv 里的参数才可信 */
  u8 lock;
  u8 strength;
  u16 snr;
};

/* 测试钩子：强制 read_status 上报的锁状态。
 *
 * 用来在没有信号源（或者没法远程拔天线）的情况下走完
 * "锁定 -> 失锁 -> 自己重新锁定" 这条链路：
 *    0   正常，完全不改变行为（默认）
 *   >0   无条件上报"已锁定"
 *   <0   无条件上报"未锁定"
 *
 * 它只改最后填进 *status 的结论，命令通道和 buf 解析照旧，所以测的是
 * 内核状态机 + 驱动的重调谐路径（这正是"失锁后频率被污染"出事的地方），
 * 不是驱动自身的收发逻辑。和 fault_test 一样，只为验证而存在：
 * 权限 0600，只有 root 能改，默认 0 时行为与未加钩子完全一致。 */
static int smit_fake_lock;
module_param_named(fake_lock, smit_fake_lock, int, 0600);
MODULE_PARM_DESC(fake_lock, "test hook only: >0 force lock, <0 force unlock, 0 off");

static int icast_fe_init(struct dvb_frontend *fe)
{
	struct icast_state *priv = fe->demodulator_priv;
	
	priv->frequency = 0;
	priv->symbol_rate = 0;
	/* 重新打开前端后，第一次读状态之前 read_signal_strength / read_snr
	 * 会用到这几个值。留着上一次的锁状态就等于"还没读就报告旧信号"，
	 * 播放器可能据此认为信号还在。 */
	priv->tuned = 0;
	priv->lock = 0;
	priv->strength = 0;
	priv->snr = 0;

	return 0;
}

static void icast_fe_release(struct dvb_frontend *fe)
{
	struct icast_state *state = fe->demodulator_priv;

	kfree(state);
}

/* 只写调用方传进来的 props，绝不去碰 fe->dtv_property_cache。
 *
 * 内核在这两条路径上的处理完全不同：
 *   FE_GET_FRONTEND(FE_GET_PROPERTY)：dvb_get_frontend() 特意先 memcpy
 *     一份副本再交给驱动，注释写明就是防"驱动乱写缓存"；
 *   状态事件：dvb_frontend_add_event() 直接把真实 cache 传进来
 *     （每次锁定状态变化都会调，不需要应用参与）。
 *
 * 原来这里无视 props、直接写真实 cache，等于把副本机制绕了过去：
 * 应用读到的还是旧值，而 cache.frequency 被悄悄改写成设备侧的数值。
 * 而 cache.frequency 正是 swzigzag 重调谐的基准
 * （dvb_frontend_swzigzag_autotune: c->frequency += lnb_drift）：
 * 它被改成 kHz 之后，失锁后每次重试都往几百 kHz 上调，再也锁不上 ——
 * 表现就是"画面卡住不恢复"。 */
static int icast_fe_get_frontend(struct dvb_frontend *fe, struct dtv_frontend_properties *props)
{
  struct icast_state *priv = fe->demodulator_priv;

  /* 只在真的调谐成功过之后上报，单位统一是 Hz（与 set_frontend 入参一致）。
   * 一次都没调谐就返回 0 字段，让核心保留它自己缓存的值。 */
  if (priv->tuned) {
    props->frequency = priv->frequency;
    props->symbol_rate = priv->symbol_rate;
    props->modulation = priv->modulation;
  }

  return 0;
}

/* do-while(0) 包起来，否则这四条语句在 if/else 里展开就只剩第一条受控，
 * 是个典型的静默错误。参数也统一加括号，避免传表达式时被优先级坑。 */
#define DWORD_TO_BYTES(dword, bytes, position) do {			\
  (bytes)[position]     = (u8)((dword) & 0xFF);				\
  (bytes)[position + 1] = (u8)(((dword) >> 8) & 0xFF);			\
  (bytes)[position + 2] = (u8)(((dword) >> 16) & 0xFF);			\
  (bytes)[position + 3] = (u8)(((dword) >> 24) & 0xFF);			\
} while (0)

static int icast_fe_set_frontend(struct dvb_frontend *fe)
{
  u8 cmd9[32] = {
  	0x01, 0x90, 0x02, 0x00, 0x03, 0x9F, 0x9A, 0x07,
  	0x17,	0x00, 0x00, 0x14, 0x00, 0x03, 0x00, 0x10,
  	0xB8,	0xF3, 0x03, 0x00, 0xDB, 0x1A, 0x00, 0x00,
  	0x40,	0x00, 0x00, 0x00, 0x08, 0x01, 0x00, 0x00
  }; 
  u8 buf[900];
  u32 frequency, symbol_rate;
  struct icast_state *priv = fe->demodulator_priv;
  unsigned long t0 = jiffies;
  int ret;

	frequency = (u32)(fe->dtv_property_cache.frequency / 1000);
  DWORD_TO_BYTES(frequency, cmd9, 16);
  
  if ( fe->dtv_property_cache.delivery_system == SYS_DVBC_ANNEX_A)
  {
		symbol_rate = (u32)(fe->dtv_property_cache.symbol_rate / 1000);
    DWORD_TO_BYTES(symbol_rate, cmd9, 20);
    
    switch ( fe->dtv_property_cache.modulation )
    {
      case QAM_16:
      	DWORD_TO_BYTES(16, cmd9, 24);
        break;
      case QAM_32:
      	DWORD_TO_BYTES(32, cmd9, 24);
        break;
      case QAM_64:
      	DWORD_TO_BYTES(64, cmd9, 24);
        break;
      case QAM_128:
      	DWORD_TO_BYTES(128, cmd9, 24);
        break;
      case QAM_256:
      	DWORD_TO_BYTES(256, cmd9, 24);
        break;
      default:
        break;
    }
  }
  else
  {
  	DWORD_TO_BYTES(0, cmd9, 20);
    DWORD_TO_BYTES(0, cmd9, 24);
    DWORD_TO_BYTES(8, cmd9, 28);
  }

  /* 失败必须往上报，而且不能更新 priv：原来把返回值丢掉、永远 return 0，
   * priv 里的状态会被改成"已调到这个频点"，之后 get_frontend 就会把这个
   * 假状态报给上层。
   *
   * 注意这里说的是 priv 被写脏，不是"应用会看到调谐成功" —— 应用本来就
   * 拿不到这个返回值：DTV_TUNE 只是把 fepriv->state 置成 FESTATE_RETUNE
   * 就返回 0，真正的 set_frontend 是状态线程后来调的。
   *
   * 但"哪些失败值得上报"要分清，因为返回负值在核心里是很重的惩罚：
   * dvb_frontend_swzigzag_autotune() 见到 set_frontend < 0 就把状态置成
   * FESTATE_ERROR，而 FESTATE_ERROR 之后：
   *   - swzigzag 的所有分支都不再匹配 -> 再也不会自动重新调谐；
   *   - FE_READ_STATUS ioctl 直接短路返回 0（不调驱动），应用永远看到"没锁"；
   * 只有应用重新下发调谐才能脱身。所以：
   *   - 传输层错误（-110/-71/-19…）：链路死了，上报，让状态机停手，
   *     自愈复位在后台重枚举，重开后重新走 init；
   *   - -1006：芯片明确回了错误状态（参数不被接受），原地重试没有意义；
   *   - -(1000+checkcount)：芯片收下了命令，只是没在预算内回完成标志。
   *     无信号频点、"正在锁"都可能这样（init 阶段调谐到 0 Hz 实测白等
   *     4390ms 后拿到 -1100 就是这种）。把它当成失败会让一次失锁永久
   *     卡死，天线插回来也不会自己恢复 —— 所以按"命令已下发、状态未知"
   *     处理，上报 0 让状态机继续轮询和重调谐。 */
  ret = smit_set(priv->dev->udev, 0xA0u, cmd9, 32, 100);

  /* 传输层错误先自己重发一次，别急着把状态机推到 ERROR。
   *
   * 命令通道偶发丢一包是常态（smit_init 那边就记过"丢一包是有的"）。
   * 一次孤立的 -ETIMEDOUT 如果直接上抛，核心会进 FESTATE_ERROR 再也不
   * 重调谐；而紧接着线程里的 read_status 是成功的，err_streak 反而被清零，
   * 自愈也不会启动 —— 结果链路明明是好的，前端却永久停在"没锁"，
   * 只能等应用重新下发调谐。这正好是"画面不动、但不拔插也能好"的反面。
   *
   * 重发是安全的：smit_bulk_transfer() 在超时时已经把 smit_resync_pending
   * 置上了，下一次事务开头的 smit_prepare() 会先把 IN 管道里可能残留的
   * 半包/迟到应答抽干净，再从干净的边界重新发。
   *
   * 设备已经不在（-ENODEV / -ESHUTDOWN）不重发：那是热插拔事件，
   * 交给上层和自愈处理，多打一次只会白等。 */
  if (ret < 0 && smit_is_transport_err(ret) &&
      ret != -ENODEV && ret != -ESHUTDOWN) {
    if (smit_trace)
      pr_info("dvb_usb_smit:  set_frontend %u Hz: transport error %d, "
              "resending once\n", fe->dtv_property_cache.frequency, ret);
    ret = smit_set(priv->dev->udev, 0xA0u, cmd9, 32, 100);
  }

  if (ret < 0) {
    if (smit_is_transport_err(ret) || ret == -1006)
      return ret;
    if (smit_trace)
      pr_info("dvb_usb_smit:  set_frontend %u Hz: no completion flag (%d), "
              "treat as accepted\n", fe->dtv_property_cache.frequency, ret);
  }

  /* 状态回读失败不算调谐失败：调谐命令已经下去了，这里返回错误只会
   * 让应用以为没调成而反复重试。真正的失败由 set 那一步暴露。 */
  smit_tuner_stat(priv->dev->udev, buf, 2);

  priv->frequency = fe->dtv_property_cache.frequency;
  priv->symbol_rate = fe->dtv_property_cache.symbol_rate;
  priv->modulation = fe->dtv_property_cache.modulation;
  priv->tuned = 1;

  /* 调谐是排查"失锁后能不能自己恢复"的唯一观测点：重调谐时核心到底
   * 拿了多少 Hz 去下发，只有这里看得见。频率单位被污染（474 MHz 变成
   * 474 kHz）在这种日志里一眼就能认出来。 */
  if (smit_trace)
    pr_info("dvb_usb_smit:  set_frontend %s %u Hz sr=%u r=%d %ums\n",
            fe->dtv_property_cache.delivery_system == SYS_DVBC_ANNEX_A ?
              "DVBC" : "DVBT",
            fe->dtv_property_cache.frequency,
            fe->dtv_property_cache.symbol_rate, ret,
            jiffies_to_msecs(jiffies - t0));

  return 0;
}

static int icast_fe_get_tune_settings(struct dvb_frontend *fe,
			      struct dvb_frontend_tune_settings *fesettings)
{
	fesettings->min_delay_ms = 800;
	fesettings->step_size = 0;
	fesettings->max_drift = 0;
	return 0;
}

static int icast_fe_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
  struct icast_state *priv = fe->demodulator_priv;
  u8 cmd12[20] = {
  	0x01, 0x90, 0x02, 0x00, 0x03, 0x9F, 0x9A, 0x07,
  	0x0B, 0x01, 0x00, 0x08, 0x00, 0x05, 0x00, 0x04,
  	0x47, 0x53, 0x54, 0x41
  };
  u8 buf[900];
  int ok = 0;

  /* 先把状态明确置 0。原来这条失败分支直接 return 0 而没写 *status，
   * 上层拿到的是 dvb_frontend_ioctl 栈上的未初始化变量：里面可能刚好带着
   * FE_HAS_LOCK 位，于是一次读失败之后播放器会一直以为"还锁着"，
   * 画面对着不动的画面永远不恢复，看起来就像驱动挂了。 */
  *status = 0;

  /* 命令发出失败就直接按"没锁"处理，没必要再等一次 tuner_stat 的
   * 超时预算（芯片不应答时那是 560ms 起，最坏几秒）。
   * 但**不能提前 return**：fake_lock 钩子必须对所有分支都成立，否则
   * 换一种固件行为（比如无信号时 cmd12 完不成）钩子就静默失效，
   * 测出来的"恢复正常"是假的。 */
  if (smit_set(priv->dev->udev, 0xA0u, cmd12, 20, 50) < 0)
    ok = 0;
  else
    ok = (smit_tuner_stat(priv->dev->udev, buf, 5) == 32);

  /* 读回的长度不是 32（或者命令就没发出去）就认为这次读取失败，锁状态
   * 未知，按"没锁"上报。不这么做的话 priv->lock 会停在上一次的值，
   * read_signal_strength / read_snr 会继续返回旧的强度和信噪比，
   * 看起来像是信号还在。 */
  priv->lock = ok ? (u8)buf[29] : 0;
  if (smit_fake_lock)
    priv->lock = smit_fake_lock > 0 ? 1 : 0;

  if ( priv->lock == 0 )
  {
    *status = 0;
  }
  else
  {
    /* 设备只给一个"锁"标志。锁定意味着载波、同步、Viterbi 都已经收敛，
     * 所以这几个位一起点亮：个别应用只看 FE_HAS_CARRIER / FE_HAS_SYNC，
     * 只报 SIGNAL|LOCK 会被它们当成"没锁"。 */
    *status = FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI |
              FE_HAS_SYNC | FE_HAS_LOCK;

    /* 这里原来还会用 buf[16..19] 覆盖 priv->frequency，但那个字段的
     * 单位是设备侧的（下发时 set_frontend 明确 /1000 转成 kHz），而
     * priv->frequency 在 set_frontend 里存的是 Hz。两种单位混在一个
     * 字段里，再经 get_frontend 写回核心的 cache.frequency，就会把
     * swzigzag 重调谐的基准从 474 MHz 变成 474 kHz，失锁之后永远
     * 调不回来。这个字段只保留一个单位（Hz）、一个写入方
     * （set_frontend），设备回报值不再参与。 */
    if (ok) {
      priv->snr = (u8)buf[30];
      priv->strength = (u8)buf[31];
    }
  }
  return 0;
}

/* 设备回报的是 0..100 的百分比，DVB 要求 0..65535。
 *
 * 原来那串魔数（90070618176225 >> 37 再减一个修正项）就是编译器把
 * x * 65535 / 100 展开的结果，x 在 0..100 时完全正确，但超过 100
 * 就会溢出：x=101 算出 66190，赋给 u16 截断成 654；x=255 变成 36042。
 * 也就是"信号强度一下掉到几乎为 0"，应用会当成失锁。
 * 直接写乘除更好读，再加 clamp 把异常值挡在 100。 */
static u16 icast_pct_to_u16(u8 v)
{
	u32 x = v > 100 ? 100 : v;

	return (u16)(x * 0xFFFF / 100);
}

static int icast_fe_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
  struct icast_state *priv = fe->demodulator_priv;

  *strength = 0;
  if ( priv->lock )
    *strength = icast_pct_to_u16(priv->strength);
  return 0;
}

static int icast_fe_read_snr(struct dvb_frontend *fe, u16 *snr)
{
  struct icast_state *priv = fe->demodulator_priv;

  *snr = 0;
  if ( priv->lock )
    *snr = icast_pct_to_u16(priv->snr);
  return 0;
}

static const struct dvb_frontend_ops icast_dual_mode_fe_ops = {
	.delsys = { SYS_DVBT, SYS_DVBC_ANNEX_A },
	.info = {
		.name = "iCast DTMB/DVBC USB demodulator",
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0)
		.frequency_min = 10000000,
		.frequency_max = 862000000,
		.frequency_stepsize = 10000,
#else
		.frequency_min_hz	= 10 * MHz,
		.frequency_max_hz	= 862 * MHz,
		.frequency_stepsize_hz	= 10 * kHz,
#endif
		.caps =
			FE_CAN_FEC_AUTO |
			FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO
	},

	.release = icast_fe_release,
	.init = icast_fe_init,
	.set_frontend = icast_fe_set_frontend,
	.get_tune_settings = icast_fe_get_tune_settings,
	.get_frontend = icast_fe_get_frontend,
	.read_status = icast_fe_read_status,
	.read_signal_strength = icast_fe_read_signal_strength,
	.read_snr = icast_fe_read_snr,
};

static const struct dvb_frontend_ops icast_single_mode_fe_ops = {
	.delsys = { SYS_DVBC_ANNEX_A },
	.info = {
		.name = "iCast DVBC USB demodulator",
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0)
		.frequency_min = 10000000,
		.frequency_max = 862000000,
		.frequency_stepsize = 10000,
#else
		.frequency_min_hz	= 10 * MHz,
		.frequency_max_hz	= 862 * MHz,
		.frequency_stepsize_hz	= 10 * kHz,
#endif
		.caps =
			FE_CAN_FEC_AUTO |
			FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO
	},

	.release = icast_fe_release,
	.init = icast_fe_init,
	.set_frontend = icast_fe_set_frontend,
	.get_tune_settings = icast_fe_get_tune_settings,
	.get_frontend = icast_fe_get_frontend,
	.read_status = icast_fe_read_status,
	.read_signal_strength = icast_fe_read_signal_strength,
	.read_snr = icast_fe_read_snr,
};

struct dvb_frontend *icast_fe_attach(struct dvb_usb_adapter *adap, const struct icast_fe_config *config)
{
	struct icast_state *priv = NULL;

	priv = kzalloc(sizeof(struct icast_state), GFP_KERNEL);
	if (priv == NULL)
		return NULL;
	
  if (config->dtmb)
  {
  	memcpy(&priv->frontend.ops, &icast_dual_mode_fe_ops, sizeof(struct dvb_frontend_ops));
  }
  else
  {
  	memcpy(&priv->frontend.ops, &icast_single_mode_fe_ops, sizeof(struct dvb_frontend_ops));
  }
	
	priv->frontend.demodulator_priv = priv;
	priv->dev = adap->dev;
	
	return &priv->frontend;
}

/* Module metadata lives in icast-smit.c: both files are linked into the same
 * dvb-usb-smit.ko, and a second MODULE_LICENSE() would duplicate .modinfo. */

