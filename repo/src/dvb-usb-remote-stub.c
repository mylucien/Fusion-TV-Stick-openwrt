// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal replacement for drivers/media/usb/dvb-usb/dvb-usb-remote.c
 *
 * Why this exists
 * ---------------
 * The DVB-USB library always calls dvb_usb_remote_init()/dvb_usb_remote_exit()
 * from dvb-usb-init.c. Upstream implements them in dvb-usb-remote.c, which
 * pulls in rc-core (rc_allocate_device, rc_register_device, ...) and the input
 * core (input_allocate_device, input_event, ...).
 *
 * Router / ONT firmware usually ships without CONFIG_RC_CORE and often without
 * CONFIG_INPUT, so the stock dvb-usb.ko cannot link or load there.
 *
 * The iCast dongle never fills in props.rc, so upstream's
 * dvb_usb_remote_init() would bail out at its very first check anyway:
 *
 *	if (d->props.rc.legacy.rc_map_table && d->props.rc.legacy.rc_query)
 *		... = DVB_RC_LEGACY;
 *	else if (d->props.rc.core.rc_codes)
 *		... = DVB_RC_CORE;
 *	else
 *		return 0;		<-- we always land here
 *
 * So stubbing both entry points is behaviourally identical for this driver,
 * and it removes the rc-core + input-core dependency entirely.
 */

#include "dvb-usb-common.h"

/*
 * 注意：6.18 起 dvb_usb_disable_rc_polling 已由 dvb-usb-init.c 无条件定义
 * （它同时是 module_param），这里不能再定义，否则 ld -r 报 multiple definition。
 */

int dvb_usb_remote_init(struct dvb_usb_device *d)
{
	d->state &= ~DVB_USB_STATE_REMOTE;
	deb_info("rc support compiled out (NO_RC=1)\n");
	return 0;
}

int dvb_usb_remote_exit(struct dvb_usb_device *d)
{
	d->state &= ~DVB_USB_STATE_REMOTE;
	return 0;
}
