/*
 *  Bluetooth Software UART Qualcomm protocol
 *
 *  HCI_IBS (HCI In-Band Sleep) is Qualcomm's power management
 *  protocol extension to H4.
 *
 *  Copyright (C) 2007 Texas Instruments, Inc.
 *  Copyright (c) 2010, 2012 The Linux Foundation. All rights reserved.
 *
 *  Acknowledgements:
 *  This file is based on hci_ll.c, which was...
 *  Written by Ohad Ben-Cohen <ohad@bencohen.org>
 *  which was in turn based on hci_h4.c, which was written
 *  by Maxim Krasnyansky and Marcel Holtmann.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2
 *  as published by the Free Software Foundation
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/tty.h>
#include <asm/ioctls.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "hci_uart.h"
#include "btqca.h"

/* HCI_IBS protocol messages */
#define HCI_IBS_SLEEP_IND	0xFE
#define HCI_IBS_WAKE_IND	0xFD
#define HCI_IBS_WAKE_ACK	0xFC
#define HCI_MAX_IBS_SIZE	10

/* Controller states */
#define STATE_IN_BAND_SLEEP_ENABLED	1
#define QCA_DROP_VENDOR_EVENT		2

#define IBS_WAKE_RETRANS_TIMEOUT_MS	100
#define IBS_TX_IDLE_TIMEOUT_MS		2000
#define BAUDRATE_SETTLE_TIMEOUT_MS	300
#define CMD_TRANS_TIMEOUT_MS		100

/* Which SoC sits on the other end of the line discipline.  Without serdev
 * there is no device to bind to, so the board's device tree is consulted:
 * SDM6xx/MSM8998 trees describe the WCN3990 (for its power driver) as a
 * node compatible with "qca,wcn3990".
 */
static enum qca_btsoc_type qca_soc_type_from_dt(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "qca,wcn3990");
	if (!np)
		return QCA_ROME;

	of_node_put(np);
	return QCA_WCN3990;
}

/* HCI_IBS transmit side sleep protocol states */
enum tx_ibs_states {
	HCI_IBS_TX_ASLEEP,
	HCI_IBS_TX_WAKING,
	HCI_IBS_TX_AWAKE,
};

/* HCI_IBS receive side sleep protocol states */
enum rx_states {
	HCI_IBS_RX_ASLEEP,
	HCI_IBS_RX_AWAKE,
};

/* HCI_IBS transmit and receive side clock state vote */
enum hci_ibs_clock_state_vote {
	HCI_IBS_VOTE_STATS_UPDATE,
	HCI_IBS_TX_VOTE_CLOCK_ON,
	HCI_IBS_TX_VOTE_CLOCK_OFF,
	HCI_IBS_RX_VOTE_CLOCK_ON,
	HCI_IBS_RX_VOTE_CLOCK_OFF,
};

struct qca_data {
	struct hci_uart *hu;
	struct sk_buff *rx_skb;
	struct sk_buff_head txq;
	struct sk_buff_head tx_wait_q;	/* HCI_IBS wait queue	*/
	spinlock_t hci_ibs_lock;	/* HCI_IBS state lock	*/
	u8 tx_ibs_state;	/* HCI_IBS transmit side power state*/
	u8 rx_ibs_state;	/* HCI_IBS receive side power state */
	bool tx_vote;		/* Clock must be on for TX */
	bool rx_vote;		/* Clock must be on for RX */
	struct timer_list tx_idle_timer;
	u32 tx_idle_delay;
	struct timer_list wake_retrans_timer;
	u32 wake_retrans;
	struct workqueue_struct *workqueue;
	struct work_struct ws_awake_rx;
	struct work_struct ws_awake_device;
	struct work_struct ws_rx_vote_off;
	struct work_struct ws_tx_vote_off;
	unsigned long flags;
	enum qca_btsoc_type soc_type;
	struct completion drop_ev_comp;

	/* For debugging purpose */
	u64 ibs_sent_wacks;
	u64 ibs_sent_slps;
	u64 ibs_sent_wakes;
	u64 ibs_recv_wacks;
	u64 ibs_recv_slps;
	u64 ibs_recv_wakes;
	u64 vote_last_jif;
	u32 vote_on_ms;
	u32 vote_off_ms;
	u64 tx_votes_on;
	u64 rx_votes_on;
	u64 tx_votes_off;
	u64 rx_votes_off;
	u64 votes_on;
	u64 votes_off;
};

/* Ask the UART driver to keep its clocks on (or let them go) via the
 * generic tty power-management ioctls.  On MSM the high-speed UART driver
 * implements them as a runtime-PM reference (MSM_ENABLE_UART_CLOCK is
 * TIOCPMGET); after the last put the port autosuspends once idle and the
 * RX wakeup interrupt injects an IBS wake indication when the controller
 * talks again.  Drivers without the ioctl simply keep their clocks on.
 *
 * This sleeps, so it is called from the IBS workqueue with the lock
 * dropped, never from serial_clock_vote() itself.
 */
static void qca_serial_clock(struct hci_uart *hu, bool on)
{
	struct tty_struct *tty = hu->tty;
	int err;

	if (!tty->ops->ioctl)
		return;

	/* From qca_close() the hdev may already be gone and, when the tty is
	 * being released, the port already shut down (-EIO): the UART driver
	 * resets its reference count on the next open, so that is fine.
	 */
	err = tty->ops->ioctl(tty, on ? TIOCPMGET : TIOCPMPUT, 0);
	if (err && err != -ENOIOCTLCMD && err != -ENOTTY && err != -EIO)
		BT_ERR("%s: UART clock %s request failed (%d)",
		       tty->name, on ? "on" : "off", err);
	else
		BT_DBG("%s: UART clock %s (%d)", tty->name,
		       on ? "on" : "off", err);
}

/* serial_clock_vote needs to be called with the ibs lock held.
 * Returns 1 when the combined vote turned on, -1 when it turned off and
 * 0 when nothing changed; the caller applies the change with
 * qca_serial_clock() once the lock is released.
 */
static int serial_clock_vote(unsigned long vote, struct hci_uart *hu)
{
	struct qca_data *qca = hu->priv;
	unsigned int diff;

	bool old_vote = (qca->tx_vote | qca->rx_vote);
	bool new_vote;

	switch (vote) {
	case HCI_IBS_VOTE_STATS_UPDATE:
		diff = jiffies_to_msecs(jiffies - qca->vote_last_jif);

		if (old_vote)
			qca->vote_off_ms += diff;
		else
			qca->vote_on_ms += diff;
		return 0;

	case HCI_IBS_TX_VOTE_CLOCK_ON:
		qca->tx_vote = true;
		qca->tx_votes_on++;
		new_vote = true;
		break;

	case HCI_IBS_RX_VOTE_CLOCK_ON:
		qca->rx_vote = true;
		qca->rx_votes_on++;
		new_vote = true;
		break;

	case HCI_IBS_TX_VOTE_CLOCK_OFF:
		qca->tx_vote = false;
		qca->tx_votes_off++;
		new_vote = qca->rx_vote | qca->tx_vote;
		break;

	case HCI_IBS_RX_VOTE_CLOCK_OFF:
		qca->rx_vote = false;
		qca->rx_votes_off++;
		new_vote = qca->rx_vote | qca->tx_vote;
		break;

	default:
		BT_ERR("Voting irregularity");
		return 0;
	}

	if (new_vote == old_vote)
		return 0;

	BT_DBG("Vote serial clock %s(%s)", new_vote ? "true" : "false",
	       vote ? "true" : "false");

	diff = jiffies_to_msecs(jiffies - qca->vote_last_jif);

	if (new_vote) {
		qca->votes_on++;
		qca->vote_off_ms += diff;
	} else {
		qca->votes_off++;
		qca->vote_on_ms += diff;
	}
	qca->vote_last_jif = jiffies;

	return new_vote ? 1 : -1;
}

/* Builds and sends an HCI_IBS command packet.
 * These are very simple packets with only 1 cmd byte.
 */
static int send_hci_ibs_cmd(u8 cmd, struct hci_uart *hu)
{
	int err = 0;
	struct sk_buff *skb = NULL;
	struct qca_data *qca = hu->priv;

	BT_DBG("hu %p send hci ibs cmd 0x%x", hu, cmd);

	skb = bt_skb_alloc(1, GFP_ATOMIC);
	if (!skb) {
		BT_ERR("Failed to allocate memory for HCI_IBS packet");
		return -ENOMEM;
	}

	/* Assign HCI_IBS type */
	*skb_put(skb, 1) = cmd;

	skb_queue_tail(&qca->txq, skb);

	return err;
}

static void qca_wq_awake_device(struct work_struct *work)
{
	struct qca_data *qca = container_of(work, struct qca_data,
					    ws_awake_device);
	struct hci_uart *hu = qca->hu;
	unsigned long retrans_delay;
	int clock;

	BT_DBG("hu %p wq awake device", hu);

	spin_lock(&qca->hci_ibs_lock);

	/* Vote for serial clock */
	clock = serial_clock_vote(HCI_IBS_TX_VOTE_CLOCK_ON, hu);

	spin_unlock(&qca->hci_ibs_lock);

	/* Bring the UART up before the wake goes out */
	if (clock > 0)
		qca_serial_clock(hu, true);

	spin_lock(&qca->hci_ibs_lock);

	/* Send wake indication to device */
	if (send_hci_ibs_cmd(HCI_IBS_WAKE_IND, hu) < 0)
		BT_ERR("Failed to send WAKE to device");

	qca->ibs_sent_wakes++;

	/* Start retransmit timer */
	retrans_delay = msecs_to_jiffies(qca->wake_retrans);
	mod_timer(&qca->wake_retrans_timer, jiffies + retrans_delay);

	spin_unlock(&qca->hci_ibs_lock);

	/* Actually send the packets */
	hci_uart_tx_wakeup(hu);
}

static void qca_wq_awake_rx(struct work_struct *work)
{
	struct qca_data *qca = container_of(work, struct qca_data,
					    ws_awake_rx);
	struct hci_uart *hu = qca->hu;
	int clock;

	BT_DBG("hu %p wq awake rx", hu);

	spin_lock(&qca->hci_ibs_lock);

	clock = serial_clock_vote(HCI_IBS_RX_VOTE_CLOCK_ON, hu);

	spin_unlock(&qca->hci_ibs_lock);

	if (clock > 0)
		qca_serial_clock(hu, true);

	spin_lock(&qca->hci_ibs_lock);

	qca->rx_ibs_state = HCI_IBS_RX_AWAKE;

	/* Always acknowledge device wake up,
	 * sending IBS message doesn't count as TX ON.
	 */
	if (send_hci_ibs_cmd(HCI_IBS_WAKE_ACK, hu) < 0)
		BT_ERR("Failed to acknowledge device wake up");

	qca->ibs_sent_wacks++;

	spin_unlock(&qca->hci_ibs_lock);

	/* Actually send the packets */
	hci_uart_tx_wakeup(hu);
}

static void qca_wq_serial_rx_clock_vote_off(struct work_struct *work)
{
	struct qca_data *qca = container_of(work, struct qca_data,
					    ws_rx_vote_off);
	struct hci_uart *hu = qca->hu;
	int clock;

	BT_DBG("hu %p rx clock vote off", hu);

	spin_lock(&qca->hci_ibs_lock);

	clock = serial_clock_vote(HCI_IBS_RX_VOTE_CLOCK_OFF, hu);

	spin_unlock(&qca->hci_ibs_lock);

	if (clock < 0)
		qca_serial_clock(hu, false);
}

static void qca_wq_serial_tx_clock_vote_off(struct work_struct *work)
{
	struct qca_data *qca = container_of(work, struct qca_data,
					    ws_tx_vote_off);
	struct hci_uart *hu = qca->hu;
	int clock;

	BT_DBG("hu %p tx clock vote off", hu);

	spin_lock(&qca->hci_ibs_lock);

	/* Run HCI tx handling unlocked */
	hci_uart_tx_wakeup(hu);

	/* Now that message queued to tty driver, vote for tty clocks off.
	 * It is up to the tty driver to pend the clocks off until tx done.
	 */
	clock = serial_clock_vote(HCI_IBS_TX_VOTE_CLOCK_OFF, hu);

	spin_unlock(&qca->hci_ibs_lock);

	if (clock < 0)
		qca_serial_clock(hu, false);
}

static void hci_ibs_tx_idle_timeout(unsigned long arg)
{
	struct hci_uart *hu = (struct hci_uart *)arg;
	struct qca_data *qca = hu->priv;
	unsigned long flags;

	BT_DBG("hu %p idle timeout in %d state", hu, qca->tx_ibs_state);

	spin_lock_irqsave_nested(&qca->hci_ibs_lock,
				 flags, SINGLE_DEPTH_NESTING);

	switch (qca->tx_ibs_state) {
	case HCI_IBS_TX_AWAKE:
		/* TX_IDLE, go to SLEEP */
		if (send_hci_ibs_cmd(HCI_IBS_SLEEP_IND, hu) < 0) {
			BT_ERR("Failed to send SLEEP to device");
			break;
		}
		qca->tx_ibs_state = HCI_IBS_TX_ASLEEP;
		qca->ibs_sent_slps++;
		queue_work(qca->workqueue, &qca->ws_tx_vote_off);
		break;

	case HCI_IBS_TX_ASLEEP:
	case HCI_IBS_TX_WAKING:
		/* Fall through */

	default:
		BT_ERR("Spurrious timeout tx state %d", qca->tx_ibs_state);
		break;
	}

	spin_unlock_irqrestore(&qca->hci_ibs_lock, flags);
}

static void hci_ibs_wake_retrans_timeout(unsigned long arg)
{
	struct hci_uart *hu = (struct hci_uart *)arg;
	struct qca_data *qca = hu->priv;
	unsigned long flags, retrans_delay;
	bool retransmit = false;

	BT_DBG("hu %p wake retransmit timeout in %d state",
		hu, qca->tx_ibs_state);

	spin_lock_irqsave_nested(&qca->hci_ibs_lock,
				 flags, SINGLE_DEPTH_NESTING);

	switch (qca->tx_ibs_state) {
	case HCI_IBS_TX_WAKING:
		/* No WAKE_ACK, retransmit WAKE */
		retransmit = true;
		if (send_hci_ibs_cmd(HCI_IBS_WAKE_IND, hu) < 0) {
			BT_ERR("Failed to acknowledge device wake up");
			break;
		}
		qca->ibs_sent_wakes++;
		retrans_delay = msecs_to_jiffies(qca->wake_retrans);
		mod_timer(&qca->wake_retrans_timer, jiffies + retrans_delay);
		break;

	case HCI_IBS_TX_ASLEEP:
	case HCI_IBS_TX_AWAKE:
		/* Fall through */

	default:
		BT_ERR("Spurrious timeout tx state %d", qca->tx_ibs_state);
		break;
	}

	spin_unlock_irqrestore(&qca->hci_ibs_lock, flags);

	if (retransmit)
		hci_uart_tx_wakeup(hu);
}

/* Initialize protocol */
static int qca_open(struct hci_uart *hu)
{
	struct qca_data *qca;

	BT_DBG("hu %p qca_open", hu);

	qca = kzalloc(sizeof(struct qca_data), GFP_ATOMIC);
	if (!qca)
		return -ENOMEM;

	skb_queue_head_init(&qca->txq);
	skb_queue_head_init(&qca->tx_wait_q);
	spin_lock_init(&qca->hci_ibs_lock);
	qca->workqueue = create_singlethread_workqueue("qca_wq");
	if (!qca->workqueue) {
		BT_ERR("QCA Workqueue not initialized properly");
		kfree(qca);
		return -ENOMEM;
	}

	INIT_WORK(&qca->ws_awake_rx, qca_wq_awake_rx);
	INIT_WORK(&qca->ws_awake_device, qca_wq_awake_device);
	INIT_WORK(&qca->ws_rx_vote_off, qca_wq_serial_rx_clock_vote_off);
	INIT_WORK(&qca->ws_tx_vote_off, qca_wq_serial_tx_clock_vote_off);

	qca->hu = hu;
	qca->soc_type = qca_soc_type_from_dt();
	init_completion(&qca->drop_ev_comp);

	/* Assume we start with both sides asleep -- extra wakes OK */
	qca->tx_ibs_state = HCI_IBS_TX_ASLEEP;
	qca->rx_ibs_state = HCI_IBS_RX_ASLEEP;

	/* clocks actually on, but we start votes off */
	qca->tx_vote = false;
	qca->rx_vote = false;
	qca->flags = 0;

	qca->ibs_sent_wacks = 0;
	qca->ibs_sent_slps = 0;
	qca->ibs_sent_wakes = 0;
	qca->ibs_recv_wacks = 0;
	qca->ibs_recv_slps = 0;
	qca->ibs_recv_wakes = 0;
	qca->vote_last_jif = jiffies;
	qca->vote_on_ms = 0;
	qca->vote_off_ms = 0;
	qca->votes_on = 0;
	qca->votes_off = 0;
	qca->tx_votes_on = 0;
	qca->tx_votes_off = 0;
	qca->rx_votes_on = 0;
	qca->rx_votes_off = 0;

	hu->priv = qca;

	init_timer(&qca->wake_retrans_timer);
	qca->wake_retrans_timer.function = hci_ibs_wake_retrans_timeout;
	qca->wake_retrans_timer.data = (u_long)hu;
	qca->wake_retrans = IBS_WAKE_RETRANS_TIMEOUT_MS;

	init_timer(&qca->tx_idle_timer);
	qca->tx_idle_timer.function = hci_ibs_tx_idle_timeout;
	qca->tx_idle_timer.data = (u_long)hu;
	qca->tx_idle_delay = IBS_TX_IDLE_TIMEOUT_MS;

	BT_DBG("HCI_UART_QCA open, tx_idle_delay=%u, wake_retrans=%u",
	       qca->tx_idle_delay, qca->wake_retrans);

	return 0;
}

static void qca_debugfs_init(struct hci_dev *hdev)
{
	struct hci_uart *hu = hci_get_drvdata(hdev);
	struct qca_data *qca = hu->priv;
	struct dentry *ibs_dir;
	umode_t mode;

	if (!hdev->debugfs)
		return;

	ibs_dir = debugfs_create_dir("ibs", hdev->debugfs);

	/* read only */
	mode = S_IRUGO;
	debugfs_create_u8("tx_ibs_state", mode, ibs_dir, &qca->tx_ibs_state);
	debugfs_create_u8("rx_ibs_state", mode, ibs_dir, &qca->rx_ibs_state);
	debugfs_create_u64("ibs_sent_sleeps", mode, ibs_dir,
			   &qca->ibs_sent_slps);
	debugfs_create_u64("ibs_sent_wakes", mode, ibs_dir,
			   &qca->ibs_sent_wakes);
	debugfs_create_u64("ibs_sent_wake_acks", mode, ibs_dir,
			   &qca->ibs_sent_wacks);
	debugfs_create_u64("ibs_recv_sleeps", mode, ibs_dir,
			   &qca->ibs_recv_slps);
	debugfs_create_u64("ibs_recv_wakes", mode, ibs_dir,
			   &qca->ibs_recv_wakes);
	debugfs_create_u64("ibs_recv_wake_acks", mode, ibs_dir,
			   &qca->ibs_recv_wacks);
	debugfs_create_bool("tx_vote", mode, ibs_dir, &qca->tx_vote);
	debugfs_create_u64("tx_votes_on", mode, ibs_dir, &qca->tx_votes_on);
	debugfs_create_u64("tx_votes_off", mode, ibs_dir, &qca->tx_votes_off);
	debugfs_create_bool("rx_vote", mode, ibs_dir, &qca->rx_vote);
	debugfs_create_u64("rx_votes_on", mode, ibs_dir, &qca->rx_votes_on);
	debugfs_create_u64("rx_votes_off", mode, ibs_dir, &qca->rx_votes_off);
	debugfs_create_u64("votes_on", mode, ibs_dir, &qca->votes_on);
	debugfs_create_u64("votes_off", mode, ibs_dir, &qca->votes_off);
	debugfs_create_u32("vote_on_ms", mode, ibs_dir, &qca->vote_on_ms);
	debugfs_create_u32("vote_off_ms", mode, ibs_dir, &qca->vote_off_ms);

	/* read/write */
	mode = S_IRUGO | S_IWUSR;
	debugfs_create_u32("wake_retrans", mode, ibs_dir, &qca->wake_retrans);
	debugfs_create_u32("tx_idle_delay", mode, ibs_dir,
			   &qca->tx_idle_delay);
}

/* Flush protocol data */
static int qca_flush(struct hci_uart *hu)
{
	struct qca_data *qca = hu->priv;

	BT_DBG("hu %p qca flush", hu);

	skb_queue_purge(&qca->tx_wait_q);
	skb_queue_purge(&qca->txq);

	return 0;
}

/* Close protocol */
static int qca_close(struct hci_uart *hu)
{
	struct qca_data *qca = hu->priv;

	BT_DBG("hu %p qca close", hu);

	spin_lock(&qca->hci_ibs_lock);

	serial_clock_vote(HCI_IBS_VOTE_STATS_UPDATE, hu);

	spin_unlock(&qca->hci_ibs_lock);

	skb_queue_purge(&qca->tx_wait_q);
	skb_queue_purge(&qca->txq);
	del_timer(&qca->tx_idle_timer);
	del_timer(&qca->wake_retrans_timer);
	destroy_workqueue(qca->workqueue);

	/* No more vote-off work will run; return the UART clock reference
	 * if the last vote left it on.
	 */
	if (qca->tx_vote || qca->rx_vote)
		qca_serial_clock(hu, false);

	qca->hu = NULL;

	kfree_skb(qca->rx_skb);

	hu->priv = NULL;

	kfree(qca);

	return 0;
}

/* Called upon a wake-up-indication from the device.
 */
static void device_want_to_wakeup(struct hci_uart *hu)
{
	unsigned long flags;
	struct qca_data *qca = hu->priv;

	BT_DBG("hu %p want to wake up", hu);

	spin_lock_irqsave(&qca->hci_ibs_lock, flags);

	qca->ibs_recv_wakes++;

	switch (qca->rx_ibs_state) {
	case HCI_IBS_RX_ASLEEP:
		/* Make sure clock is on - we may have turned clock off since
		 * receiving the wake up indicator awake rx clock.
		 */
		queue_work(qca->workqueue, &qca->ws_awake_rx);
		spin_unlock_irqrestore(&qca->hci_ibs_lock, flags);
		return;

	case HCI_IBS_RX_AWAKE:
		/* Always acknowledge device wake up,
		 * sending IBS message doesn't count as TX ON.
		 */
		if (send_hci_ibs_cmd(HCI_IBS_WAKE_ACK, hu) < 0) {
			BT_ERR("Failed to acknowledge device wake up");
			break;
		}
		qca->ibs_sent_wacks++;
		break;

	default:
		/* Any other state is illegal */
		BT_ERR("Received HCI_IBS_WAKE_IND in rx state %d",
		       qca->rx_ibs_state);
		break;
	}

	spin_unlock_irqrestore(&qca->hci_ibs_lock, flags);

	/* Actually send the packets */
	hci_uart_tx_wakeup(hu);
}

/* Called upon a sleep-indication from the device.
 */
static void device_want_to_sleep(struct hci_uart *hu)
{
	unsigned long flags;
	struct qca_data *qca = hu->priv;

	BT_DBG("hu %p want to sleep", hu);

	spin_lock_irqsave(&qca->hci_ibs_lock, flags);

	qca->ibs_recv_slps++;

	switch (qca->rx_ibs_state) {
	case HCI_IBS_RX_AWAKE:
		/* Update state */
		qca->rx_ibs_state = HCI_IBS_RX_ASLEEP;
		/* Vote off rx clock under workqueue */
		queue_work(qca->workqueue, &qca->ws_rx_vote_off);
		break;

	case HCI_IBS_RX_ASLEEP:
		/* Fall through */

	default:
		/* Any other state is illegal */
		BT_ERR("Received HCI_IBS_SLEEP_IND in rx state %d",
		       qca->rx_ibs_state);
		break;
	}

	spin_unlock_irqrestore(&qca->hci_ibs_lock, flags);
}

/* Called upon wake-up-acknowledgement from the device
 */
static void device_woke_up(struct hci_uart *hu)
{
	unsigned long flags, idle_delay;
	struct qca_data *qca = hu->priv;
	struct sk_buff *skb = NULL;

	BT_DBG("hu %p woke up", hu);

	spin_lock_irqsave(&qca->hci_ibs_lock, flags);

	qca->ibs_recv_wacks++;

	switch (qca->tx_ibs_state) {
	case HCI_IBS_TX_AWAKE:
		/* Expect one if we send 2 WAKEs */
		BT_DBG("Received HCI_IBS_WAKE_ACK in tx state %d",
		       qca->tx_ibs_state);
		break;

	case HCI_IBS_TX_WAKING:
		/* Send pending packets */
		while ((skb = skb_dequeue(&qca->tx_wait_q)))
			skb_queue_tail(&qca->txq, skb);

		/* Switch timers and change state to HCI_IBS_TX_AWAKE */
		del_timer(&qca->wake_retrans_timer);
		idle_delay = msecs_to_jiffies(qca->tx_idle_delay);
		mod_timer(&qca->tx_idle_timer, jiffies + idle_delay);
		qca->tx_ibs_state = HCI_IBS_TX_AWAKE;
		break;

	case HCI_IBS_TX_ASLEEP:
		/* Fall through */

	default:
		BT_ERR("Received HCI_IBS_WAKE_ACK in tx state %d",
		       qca->tx_ibs_state);
		break;
	}

	spin_unlock_irqrestore(&qca->hci_ibs_lock, flags);

	/* Actually send the packets */
	hci_uart_tx_wakeup(hu);
}

/* Enqueue frame for transmittion (padding, crc, etc) may be called from
 * two simultaneous tasklets.
 */
static int qca_enqueue(struct hci_uart *hu, struct sk_buff *skb)
{
	unsigned long flags = 0, idle_delay;
	struct qca_data *qca = hu->priv;

	BT_DBG("hu %p qca enq skb %p tx_ibs_state %d", hu, skb,
	       qca->tx_ibs_state);

	/* Prepend skb with frame type */
	memcpy(skb_push(skb, 1), &bt_cb(skb)->pkt_type, 1);

	/* Don't go to sleep in middle of patch download or
	 * Out-Of-Band(GPIOs control) sleep is selected.
	 */
	if (!test_bit(STATE_IN_BAND_SLEEP_ENABLED, &qca->flags)) {
		skb_queue_tail(&qca->txq, skb);
		return 0;
	}

	spin_lock_irqsave(&qca->hci_ibs_lock, flags);

	/* Act according to current state */
	switch (qca->tx_ibs_state) {
	case HCI_IBS_TX_AWAKE:
		BT_DBG("Device awake, sending normally");
		skb_queue_tail(&qca->txq, skb);
		idle_delay = msecs_to_jiffies(qca->tx_idle_delay);
		mod_timer(&qca->tx_idle_timer, jiffies + idle_delay);
		break;

	case HCI_IBS_TX_ASLEEP:
		BT_DBG("Device asleep, waking up and queueing packet");
		/* Save packet for later */
		skb_queue_tail(&qca->tx_wait_q, skb);

		qca->tx_ibs_state = HCI_IBS_TX_WAKING;
		/* Schedule a work queue to wake up device */
		queue_work(qca->workqueue, &qca->ws_awake_device);
		break;

	case HCI_IBS_TX_WAKING:
		BT_DBG("Device waking up, queueing packet");
		/* Transient state; just keep packet for later */
		skb_queue_tail(&qca->tx_wait_q, skb);
		break;

	default:
		BT_ERR("Illegal tx state: %d (losing packet)",
		       qca->tx_ibs_state);
		kfree_skb(skb);
		break;
	}

	spin_unlock_irqrestore(&qca->hci_ibs_lock, flags);

	return 0;
}

static int qca_ibs_sleep_ind(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct hci_uart *hu = hci_get_drvdata(hdev);

	BT_DBG("hu %p recv hci ibs cmd 0x%x", hu, HCI_IBS_SLEEP_IND);

	device_want_to_sleep(hu);

	kfree_skb(skb);
	return 0;
}

static int qca_ibs_wake_ind(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct hci_uart *hu = hci_get_drvdata(hdev);

	BT_DBG("hu %p recv hci ibs cmd 0x%x", hu, HCI_IBS_WAKE_IND);

	device_want_to_wakeup(hu);

	kfree_skb(skb);
	return 0;
}

static int qca_ibs_wake_ack(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct hci_uart *hu = hci_get_drvdata(hdev);

	BT_DBG("hu %p recv hci ibs cmd 0x%x", hu, HCI_IBS_WAKE_ACK);

	device_woke_up(hu);

	kfree_skb(skb);
	return 0;
}

#define QCA_IBS_SLEEP_IND_EVENT \
	.type = HCI_IBS_SLEEP_IND, \
	.hlen = 0, \
	.loff = 0, \
	.lsize = 0, \
	.maxlen = HCI_MAX_IBS_SIZE

#define QCA_IBS_WAKE_IND_EVENT \
	.type = HCI_IBS_WAKE_IND, \
	.hlen = 0, \
	.loff = 0, \
	.lsize = 0, \
	.maxlen = HCI_MAX_IBS_SIZE

#define QCA_IBS_WAKE_ACK_EVENT \
	.type = HCI_IBS_WAKE_ACK, \
	.hlen = 0, \
	.loff = 0, \
	.lsize = 0, \
	.maxlen = HCI_MAX_IBS_SIZE

static int qca_recv_event(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct hci_uart *hu = hci_get_drvdata(hdev);
	struct qca_data *qca = hu->priv;

	if (test_bit(QCA_DROP_VENDOR_EVENT, &qca->flags)) {
		struct hci_event_hdr *hdr = (void *)skb->data;

		/* For the WCN3990 the vendor command for a baudrate change
		 * isn't sent as synchronous HCI command, because the
		 * controller sends the corresponding vendor event with the
		 * new baudrate. The event is received and properly decoded
		 * after changing the baudrate of the host port. It needs to
		 * be dropped, otherwise it can be misinterpreted as
		 * response to a later firmware download command (also a
		 * vendor command).
		 */

		if (hdr->evt == HCI_VENDOR_PKT)
			complete(&qca->drop_ev_comp);

		kfree_skb(skb);

		return 0;
	}

	return hci_recv_frame(hdev, skb);
}

static const struct h4_recv_pkt qca_recv_pkts[] = {
	{ H4_RECV_ACL,             .recv = hci_recv_frame    },
	{ H4_RECV_SCO,             .recv = hci_recv_frame    },
	{ H4_RECV_EVENT,           .recv = qca_recv_event    },
	{ QCA_IBS_WAKE_IND_EVENT,  .recv = qca_ibs_wake_ind  },
	{ QCA_IBS_WAKE_ACK_EVENT,  .recv = qca_ibs_wake_ack  },
	{ QCA_IBS_SLEEP_IND_EVENT, .recv = qca_ibs_sleep_ind },
};

static int qca_recv(struct hci_uart *hu, const void *data, int count)
{
	struct qca_data *qca = hu->priv;

	if (!test_bit(HCI_UART_REGISTERED, &hu->flags))
		return -EUNATCH;

	qca->rx_skb = h4_recv_buf(hu->hdev, qca->rx_skb, data, count,
				  qca_recv_pkts, ARRAY_SIZE(qca_recv_pkts));
	if (IS_ERR(qca->rx_skb)) {
		int err = PTR_ERR(qca->rx_skb);
		BT_ERR("%s: Frame reassembly failed (%d)", hu->hdev->name, err);
		qca->rx_skb = NULL;
		return err;
	}

	return count;
}

static struct sk_buff *qca_dequeue(struct hci_uart *hu)
{
	struct qca_data *qca = hu->priv;

	return skb_dequeue(&qca->txq);
}

static uint8_t qca_get_baudrate_value(int speed)
{
	switch (speed) {
	case 9600:
		return QCA_BAUDRATE_9600;
	case 19200:
		return QCA_BAUDRATE_19200;
	case 38400:
		return QCA_BAUDRATE_38400;
	case 57600:
		return QCA_BAUDRATE_57600;
	case 115200:
		return QCA_BAUDRATE_115200;
	case 230400:
		return QCA_BAUDRATE_230400;
	case 460800:
		return QCA_BAUDRATE_460800;
	case 500000:
		return QCA_BAUDRATE_500000;
	case 921600:
		return QCA_BAUDRATE_921600;
	case 1000000:
		return QCA_BAUDRATE_1000000;
	case 2000000:
		return QCA_BAUDRATE_2000000;
	case 3000000:
		return QCA_BAUDRATE_3000000;
	case 3200000:
		return QCA_BAUDRATE_3200000;
	case 3500000:
		return QCA_BAUDRATE_3500000;
	default:
		return QCA_BAUDRATE_115200;
	}
}

static int qca_set_baudrate(struct hci_dev *hdev, uint8_t baudrate)
{
	struct hci_uart *hu = hci_get_drvdata(hdev);
	struct qca_data *qca = hu->priv;
	struct sk_buff *skb;
	unsigned long timeout;
	u8 cmd[] = { 0x01, 0x48, 0xFC, 0x01, 0x00 };

	if (baudrate > QCA_BAUDRATE_3200000)
		return -EINVAL;

	cmd[4] = baudrate;

	skb = bt_skb_alloc(sizeof(cmd), GFP_ATOMIC);
	if (!skb) {
		BT_ERR("Failed to allocate memory for baudrate packet");
		return -ENOMEM;
	}

	/* Assign commands to change baudrate and packet type. */
	memcpy(skb_put(skb, sizeof(cmd)), cmd, sizeof(cmd));
	bt_cb(skb)->pkt_type = HCI_COMMAND_PKT;

	skb_queue_tail(&qca->txq, skb);
	hci_uart_tx_wakeup(hu);

	/* Wait for the baudrate change request to be handed to the tty
	 * (HCI_UART_SENDING covers the write work) and to leave the FIFO.
	 */
	timeout = jiffies + msecs_to_jiffies(BAUDRATE_SETTLE_TIMEOUT_MS);
	while ((!skb_queue_empty(&qca->txq) ||
		test_bit(HCI_UART_SENDING, &hu->tx_state)) &&
	       time_before(jiffies, timeout))
		usleep_range(100, 200);

	tty_wait_until_sent(hu->tty, msecs_to_jiffies(CMD_TRANS_TIMEOUT_MS));

	/* Give the controller time to process the request.  ROME needs the
	 * full settle time; the WCN3990 answers at the new rate instead.
	 */
	if (qca->soc_type == QCA_WCN3990)
		msleep(10);
	else
		msleep(BAUDRATE_SETTLE_TIMEOUT_MS);

	return 0;
}

static int qca_set_speed(struct hci_uart *hu, unsigned int speed)
{
	struct qca_data *qca = hu->priv;
	struct hci_dev *hdev = hu->hdev;
	bool wcn3990 = qca->soc_type == QCA_WCN3990;
	int ret;

	/* Disable flow control for wcn3990 to deassert RTS while
	 * changing the baudrate of chip and host.
	 */
	if (wcn3990) {
		hci_uart_set_flow_control(hu, true);
		reinit_completion(&qca->drop_ev_comp);
		set_bit(QCA_DROP_VENDOR_EVENT, &qca->flags);
	}

	BT_INFO("%s: Set UART speed to %d", hdev->name, speed);
	ret = qca_set_baudrate(hdev, qca_get_baudrate_value(speed));
	if (ret) {
		BT_ERR("%s: Failed to change the baud rate (%d)", hdev->name,
		       ret);
		goto out;
	}

	/* The WCN3990 answers at the new speed the moment RTS is asserted,
	 * so reconfigure the port and restore flow control in one go.
	 */
	if (wcn3990)
		hci_uart_set_baudrate_flow_control(hu, speed);
	else
		hci_uart_set_baudrate(hu, speed);

out:
	if (wcn3990) {
		if (ret)
			hci_uart_set_flow_control(hu, false);

		/* Wait for the controller to send the vendor event
		 * for the baudrate change command.
		 */
		if (!ret &&
		    !wait_for_completion_timeout(&qca->drop_ev_comp,
						 msecs_to_jiffies(100))) {
			BT_ERR("%s: Failed to change controller baudrate",
			       hdev->name);
			ret = -ETIMEDOUT;
		}

		clear_bit(QCA_DROP_VENDOR_EVENT, &qca->flags);
	}

	return ret;
}

static int qca_setup(struct hci_uart *hu)
{
	struct hci_dev *hdev = hu->hdev;
	struct qca_data *qca = hu->priv;
	unsigned int speed, qca_baudrate = QCA_BAUDRATE_115200;
	u32 soc_ver = 0;
	int ret;

	/* Patch downloading has to be done without IBS mode */
	clear_bit(STATE_IN_BAND_SLEEP_ENABLED, &qca->flags);

	/* Enable controller to do both LE scan and BR/EDR inquiry
	 * simultaneously.
	 */
	set_bit(HCI_QUIRK_SIMULTANEOUS_DISCOVERY, &hdev->quirks);

	/* Setup initial baudrate */
	speed = 0;
	if (hu->init_speed)
		speed = hu->init_speed;
	else if (hu->proto->init_speed)
		speed = hu->proto->init_speed;

	if (speed)
		hci_uart_set_baudrate(hu, speed);

	if (qca->soc_type == QCA_WCN3990) {
		/* The SoC has already been powered and sent its power-on
		 * pulse by whoever attached the line discipline (there is
		 * no serdev here to do it in-kernel, and the UART must be
		 * reopened after the pulse anyway).  It answers the version
		 * request at the initial speed; everything after that runs
		 * at the operational speed.
		 */
		BT_INFO("%s: setting up wcn3990", hdev->name);

		/* The NVM carries a placeholder address; the board must
		 * provide the real one (mgmt Set Public Address), as with
		 * mainline's local-bd-address property.
		 */
		set_bit(HCI_QUIRK_INVALID_BDADDR, &hdev->quirks);

		ret = qca_read_soc_version(hdev, &soc_ver);
		if (ret)
			return ret;
	} else {
		BT_INFO("%s: ROME setup", hdev->name);
	}

	/* Setup user speed if needed */
	speed = 0;
	if (hu->oper_speed)
		speed = hu->oper_speed;
	else if (qca->soc_type == QCA_WCN3990)
		speed = 3200000;
	else if (hu->proto->oper_speed)
		speed = hu->proto->oper_speed;

	if (speed) {
		ret = qca_set_speed(hu, speed);
		if (ret)
			return ret;

		qca_baudrate = qca_get_baudrate_value(speed);
	}

	if (qca->soc_type != QCA_WCN3990) {
		ret = qca_read_soc_version(hdev, &soc_ver);
		if (ret)
			return ret;
	}

	BT_INFO("%s: QCA controller version 0x%08x", hdev->name, soc_ver);

	/* Setup patch / NVM configurations */
	ret = qca_uart_setup(hdev, qca_baudrate, qca->soc_type, soc_ver);
	if (!ret) {
		set_bit(STATE_IN_BAND_SLEEP_ENABLED, &qca->flags);
		qca_debugfs_init(hdev);
	} else if (ret == -ENOENT) {
		/* No patch/nvm-config found, run with original fw/config */
		ret = 0;
	} else if (ret == -EAGAIN) {
		/*
		 * Userspace firmware loader will return -EAGAIN in case no
		 * patch/nvm-config is found, so run with original fw/config.
		 */
		ret = 0;
	}

	/* Setup bdaddr */
	if (qca->soc_type == QCA_WCN3990)
		hu->hdev->set_bdaddr = qca_set_bdaddr;
	else
		hu->hdev->set_bdaddr = qca_set_bdaddr_rome;

	return ret;
}

static struct hci_uart_proto qca_proto = {
	.id		= HCI_UART_QCA,
	.name		= "QCA",
	.manufacturer	= 29,
	.init_speed	= 115200,
	.oper_speed	= 3000000,
	.open		= qca_open,
	.close		= qca_close,
	.flush		= qca_flush,
	.setup		= qca_setup,
	.recv		= qca_recv,
	.enqueue	= qca_enqueue,
	.dequeue	= qca_dequeue,
};

int __init qca_init(void)
{
	return hci_uart_register_proto(&qca_proto);
}

int __exit qca_deinit(void)
{
	return hci_uart_unregister_proto(&qca_proto);
}
