/*
 *  Bluetooth supports for Qualcomm Atheros chips
 *
 *  Copyright (c) 2015 The Linux Foundation. All rights reserved.
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
#include <linux/module.h>
#include <linux/firmware.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "btqca.h"

#define VERSION "0.2"

/* How long to wait for the final patch segment's acknowledgement when the
 * download mode says the controller skips events.  Mainline expects the
 * last segment to be acked anyway; the WCN3990 firmware on chef (crbtfw21,
 * patch 0x0002) never answers it, even given 3 s, so a timeout here is
 * not an error.  The window starts when the segment is handed to the
 * driver, i.e. it also covers the ~0.4 s the 123 KB patch takes on the
 * wire at 3.2 Mbaud, leaving the controller ~0.6 s to apply the patch
 * before the NVM follows.
 */
#define QCA_LAST_SEG_TIMEOUT	msecs_to_jiffies(1000)

int qca_read_soc_version(struct hci_dev *hdev, u32 *soc_version)
{
	struct sk_buff *skb;
	struct edl_event_hdr *edl;
	struct rome_version *ver;
	char cmd;
	int err = 0;

	BT_DBG("%s: QCA Version Request", hdev->name);

	cmd = EDL_PATCH_VER_REQ_CMD;
	skb = __hci_cmd_sync_ev(hdev, EDL_PATCH_CMD_OPCODE, EDL_PATCH_CMD_LEN,
				&cmd, HCI_VENDOR_PKT, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		err = PTR_ERR(skb);
		BT_ERR("%s: Reading QCA version information failed (%d)",
		       hdev->name, err);
		return err;
	}

	if (skb->len != sizeof(*edl) + sizeof(*ver)) {
		BT_ERR("%s: QCA Version size mismatch len %d", hdev->name,
		       skb->len);
		err = -EILSEQ;
		goto out;
	}

	edl = (struct edl_event_hdr *)(skb->data);
	if (!edl) {
		BT_ERR("%s: QCA TLV with no header", hdev->name);
		err = -EILSEQ;
		goto out;
	}

	if (edl->cresp != EDL_CMD_REQ_RES_EVT ||
	    edl->rtype != EDL_APP_VER_RES_EVT) {
		BT_ERR("%s: QCA Wrong packet received %d %d", hdev->name,
		       edl->cresp, edl->rtype);
		err = -EIO;
		goto out;
	}

	ver = (struct rome_version *)(edl->data);

	BT_INFO("%s: QCA Product:0x%08x Patch:0x%04x ROM:0x%04x SOC:0x%08x",
		hdev->name, le32_to_cpu(ver->product_id),
		le16_to_cpu(ver->patch_ver), le16_to_cpu(ver->rome_ver),
		le32_to_cpu(ver->soc_id));

	/* QCA chipset version can be decided by patch and SoC
	 * version, combination with upper 2 bytes from SoC
	 * and lower 2 bytes from patch will be used.
	 */
	*soc_version = (le32_to_cpu(ver->soc_id) << 16) |
			(le16_to_cpu(ver->rome_ver) & 0x0000ffff);
	if (*soc_version == 0)
		err = -EILSEQ;

out:
	kfree_skb(skb);
	if (err)
		BT_ERR("%s: QCA Failed to get version (%d)", hdev->name, err);

	return err;
}
EXPORT_SYMBOL_GPL(qca_read_soc_version);

static int qca_send_reset(struct hci_dev *hdev)
{
	struct sk_buff *skb;
	int err;

	BT_DBG("%s: QCA HCI_RESET", hdev->name);

	skb = __hci_cmd_sync(hdev, HCI_OP_RESET, 0, NULL, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		err = PTR_ERR(skb);
		BT_ERR("%s: QCA Reset failed (%d)", hdev->name, err);
		return err;
	}

	kfree_skb(skb);

	return 0;
}

int qca_send_pre_shutdown_cmd(struct hci_dev *hdev)
{
	struct sk_buff *skb;
	int err;

	BT_DBG("%s: QCA pre shutdown cmd", hdev->name);

	skb = __hci_cmd_sync(hdev, QCA_PRE_SHUTDOWN_CMD, 0, NULL,
			     HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		err = PTR_ERR(skb);
		BT_ERR("%s: QCA preshutdown command failed (%d)", hdev->name,
		       err);
		return err;
	}

	kfree_skb(skb);

	return 0;
}
EXPORT_SYMBOL_GPL(qca_send_pre_shutdown_cmd);

/* Walk the tags of one NVM block and patch the host-side settings in. */
static void qca_nvm_update_tags(struct rome_config *config, u8 *data,
				u32 length)
{
	struct tlv_type_nvm *tlv_nvm;
	u16 tag_id, tag_len;
	u32 idx = 0;

	while (idx + sizeof(*tlv_nvm) <= length) {
		tlv_nvm = (struct tlv_type_nvm *)(data + idx);

		tag_id = le16_to_cpu(tlv_nvm->tag_id);
		tag_len = le16_to_cpu(tlv_nvm->tag_len);

		if (idx + sizeof(*tlv_nvm) + tag_len > length) {
			BT_ERR("NVM tag %u overruns block", tag_id);
			return;
		}

		/* Update NVM tags as needed */
		switch (tag_id) {
		case EDL_TAG_ID_HCI:
			if (tag_len < 3)
				break;

			/* HCI transport layer parameters
			 * enabling software inband sleep
			 * onto controller side.
			 */
			tlv_nvm->data[0] |= 0x80;

			/* UART Baud Rate */
			tlv_nvm->data[2] = config->user_baud_rate;

			break;

		case EDL_TAG_ID_DEEP_SLEEP:
			if (tag_len < 1)
				break;

			/* Sleep enable mask
			 * enabling deep sleep feature on controller.
			 */
			tlv_nvm->data[0] |= 0x01;

			break;
		}

		idx += sizeof(*tlv_nvm) + tag_len;
	}
}

static void qca_tlv_check_data(struct rome_config *config,
			       const struct firmware *fw)
{
	u8 *data;
	u32 type_len, type, block_type, block_len;
	u32 idx, length;
	struct tlv_type_hdr *tlv;
	struct tlv_type_patch *tlv_patch;

	tlv = (struct tlv_type_hdr *)fw->data;

	type_len = le32_to_cpu(tlv->type_len);
	type = type_len & 0x000000ff;
	length = (type_len >> 8) & 0x00ffffff;

	BT_DBG("TLV Type\t\t : 0x%x", type);
	BT_DBG("Length\t\t : %d bytes", length);

	config->dnld_mode = QCA_SKIP_EVT_NONE;
	config->dnld_type = QCA_SKIP_EVT_NONE;

	switch (config->type) {
	case TLV_TYPE_PATCH:
		tlv_patch = (struct tlv_type_patch *)tlv->data;

		/* For Rome version 1.1 to 3.1, all segment commands
		 * are acked by a vendor specific event (VSE).
		 * For Rome >= 3.2 and WCN3990, the download mode field
		 * indicates if VSE is skipped by the controller.
		 */
		config->dnld_mode = tlv_patch->download_mode;
		config->dnld_type = config->dnld_mode;

		BT_DBG("Total Length           : %d bytes",
		       le32_to_cpu(tlv_patch->total_size));
		BT_DBG("Patch Data Length      : %d bytes",
		       le32_to_cpu(tlv_patch->data_length));
		BT_DBG("Signing Format Version : 0x%x",
		       tlv_patch->format_version);
		BT_DBG("Signature Algorithm    : 0x%x",
		       tlv_patch->signature);
		BT_DBG("Download mode          : 0x%x",
		       tlv_patch->download_mode);
		BT_DBG("Product ID             : 0x%04x",
		       le16_to_cpu(tlv_patch->product_id));
		BT_DBG("Rom Build Version      : 0x%04x",
		       le16_to_cpu(tlv_patch->rom_build));
		BT_DBG("Patch Version          : 0x%04x",
		       le16_to_cpu(tlv_patch->patch_version));
		BT_DBG("Patch Entry Address    : 0x%x",
		       le32_to_cpu(tlv_patch->entry));
		break;

	case TLV_TYPE_NVM:
		/* The firmware buffer is modified in place, as mainline does;
		 * request_firmware() hands out a private writable copy.
		 */
		data = (u8 *)tlv->data;

		if (type == TLV_TYPE_NVM) {
			qca_nvm_update_tags(config, data, length);
			break;
		}

		if (type != TLV_TYPE_NVM_UNIFIED) {
			BT_ERR("Unexpected NVM TLV type 0x%x", type);
			break;
		}

		/* Unified NVM: a sequence of (header, block) pairs. */
		idx = 0;
		while (idx + sizeof(*tlv) <= length) {
			tlv = (struct tlv_type_hdr *)(data + idx);
			type_len = le32_to_cpu(tlv->type_len);
			block_type = type_len & 0x000000ff;
			block_len = (type_len >> 8) & 0x00ffffff;

			if (idx + sizeof(*tlv) + block_len > length) {
				BT_ERR("NVM block 0x%x overruns file",
				       block_type);
				break;
			}

			BT_DBG("NVM block type 0x%x, %d bytes", block_type,
			       block_len);

			if (block_type == TLV_TYPE_NVM)
				qca_nvm_update_tags(config, (u8 *)tlv->data,
						    block_len);

			idx += sizeof(*tlv) + block_len;
		}
		break;

	default:
		BT_ERR("Unknown TLV type %d", config->type);
		break;
	}
}

static int qca_tlv_send_segment(struct hci_dev *hdev, int seg_size,
				const u8 *data, enum qca_tlv_dnld_mode mode,
				bool last)
{
	struct sk_buff *skb;
	struct edl_event_hdr *edl;
	struct tlv_seg_resp *tlv_resp;
	u8 cmd[MAX_SIZE_PER_TLV_SEGMENT + 2];
	bool skip = (mode == QCA_SKIP_EVT_VSE_CC || mode == QCA_SKIP_EVT_VSE);
	int err = 0;

	cmd[0] = EDL_PATCH_TLV_REQ_CMD;
	cmd[1] = seg_size;
	memcpy(cmd + 2, data, seg_size);

	if (skip && !last)
		return __hci_cmd_send(hdev, EDL_PATCH_CMD_OPCODE, seg_size + 2,
				      cmd);

	skb = __hci_cmd_sync_ev(hdev, EDL_PATCH_CMD_OPCODE, seg_size + 2, cmd,
				HCI_VENDOR_PKT,
				skip ? QCA_LAST_SEG_TIMEOUT : HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		err = PTR_ERR(skb);
		if (skip && err == -ETIMEDOUT) {
			BT_DBG("%s: no ack for last segment in skip mode",
			       hdev->name);
			return 0;
		}
		BT_ERR("%s: QCA Failed to send TLV segment (%d)", hdev->name,
		       err);
		return err;
	}

	if (skb->len != sizeof(*edl) + sizeof(*tlv_resp)) {
		BT_ERR("%s: QCA TLV response size mismatch", hdev->name);
		err = -EILSEQ;
		goto out;
	}

	edl = (struct edl_event_hdr *)(skb->data);
	if (!edl) {
		BT_ERR("%s: TLV with no header", hdev->name);
		err = -EILSEQ;
		goto out;
	}

	tlv_resp = (struct tlv_seg_resp *)(edl->data);
	if (edl->cresp != EDL_CMD_REQ_RES_EVT ||
	    edl->rtype != EDL_TVL_DNLD_RES_EVT || tlv_resp->result != 0x00) {
		BT_ERR("%s: QCA TLV with error stat 0x%x rtype 0x%x (0x%x)",
		       hdev->name, edl->cresp, edl->rtype, tlv_resp->result);
		err = -EIO;
	}

out:
	kfree_skb(skb);

	return err;
}

/* Feed the HCI core the command complete it never got, so that its command
 * credit is restored and the command timeout is cancelled.
 */
static int qca_inject_cmd_complete_event(struct hci_dev *hdev)
{
	struct hci_event_hdr *hdr;
	struct hci_ev_cmd_complete *evt;
	struct sk_buff *skb;

	skb = bt_skb_alloc(sizeof(*hdr) + sizeof(*evt) + 1, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	hdr = (struct hci_event_hdr *)skb_put(skb, sizeof(*hdr));
	hdr->evt = HCI_EV_CMD_COMPLETE;
	hdr->plen = sizeof(*evt) + 1;

	evt = (struct hci_ev_cmd_complete *)skb_put(skb, sizeof(*evt));
	evt->ncmd = 1;
	evt->opcode = cpu_to_le16(QCA_HCI_CC_OPCODE);

	*skb_put(skb, 1) = QCA_HCI_CC_SUCCESS;

	bt_cb(skb)->pkt_type = HCI_EVENT_PKT;

	return hci_recv_frame(hdev, skb);
}

static int qca_download_firmware(struct hci_dev *hdev,
				 struct rome_config *config)
{
	const struct firmware *fw;
	const u8 *segment;
	u32 type_len, length;
	int ret, remain, i = 0;

	BT_INFO("%s: QCA Downloading %s", hdev->name, config->fwname);

	ret = request_firmware(&fw, config->fwname, &hdev->dev);
	if (ret) {
		BT_ERR("%s: QCA Failed to request file: %s (%d)", hdev->name,
		       config->fwname, ret);
		return ret;
	}

	if (fw->size < sizeof(struct tlv_type_hdr)) {
		BT_ERR("%s: %s too small", hdev->name, config->fwname);
		ret = -EINVAL;
		goto out;
	}

	type_len = le32_to_cpu(((struct tlv_type_hdr *)fw->data)->type_len);
	length = (type_len >> 8) & 0x00ffffff;
	if (fw->size - sizeof(struct tlv_type_hdr) != length) {
		BT_ERR("%s: %s size %zu does not match header %u",
		       hdev->name, config->fwname, fw->size, length);
		ret = -EINVAL;
		goto out;
	}

	qca_tlv_check_data(config, fw);

	segment = fw->data;
	remain = fw->size;
	while (remain > 0) {
		int segsize = min(MAX_SIZE_PER_TLV_SEGMENT, remain);

		BT_DBG("%s: Send segment %d, size %d", hdev->name, i++,
		       segsize);

		remain -= segsize;
		ret = qca_tlv_send_segment(hdev, segsize, segment,
					   config->dnld_mode, remain == 0);
		if (ret)
			goto out;

		segment += segsize;
	}

	/* Latest qualcomm chipsets are not sending a command complete event
	 * for every fw packet sent. They only respond with a vendor specific
	 * event for the last packet. This optimization in the chip will
	 * decrease the BT in initialization time. Here we will inject a command
	 * complete event to avoid a command timeout error message.
	 */
	if (config->dnld_type == QCA_SKIP_EVT_VSE_CC ||
	    config->dnld_type == QCA_SKIP_EVT_VSE)
		ret = qca_inject_cmd_complete_event(hdev);

out:
	release_firmware(fw);

	return ret;
}

int qca_set_bdaddr_rome(struct hci_dev *hdev, const bdaddr_t *bdaddr)
{
	struct sk_buff *skb;
	u8 cmd[9];
	int err;

	cmd[0] = EDL_NVM_ACCESS_SET_REQ_CMD;
	cmd[1] = 0x02; 			/* TAG ID */
	cmd[2] = sizeof(bdaddr_t);	/* size */
	memcpy(cmd + 3, bdaddr, sizeof(bdaddr_t));
	skb = __hci_cmd_sync_ev(hdev, EDL_NVM_ACCESS_OPCODE, sizeof(cmd), cmd,
				HCI_VENDOR_PKT, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		err = PTR_ERR(skb);
		BT_ERR("%s: QCA Change address command failed (%d)",
		       hdev->name, err);
		return err;
	}

	kfree_skb(skb);

	return 0;
}
EXPORT_SYMBOL_GPL(qca_set_bdaddr_rome);

int qca_uart_setup(struct hci_dev *hdev, uint8_t baudrate,
		   enum qca_btsoc_type soc_type, u32 soc_ver)
{
	struct rome_config config;
	int err;
	u8 rom_ver = 0;

	BT_DBG("%s: QCA setup on UART", hdev->name);

	config.user_baud_rate = baudrate;

	/* Download rampatch file */
	config.type = TLV_TYPE_PATCH;
	if (soc_type == QCA_WCN3990) {
		/* Firmware files to download are based on ROM version.
		 * ROM version is derived from last two bytes of soc_ver.
		 */
		rom_ver = ((soc_ver & 0x00000f00) >> 0x04) |
			    (soc_ver & 0x0000000f);
		snprintf(config.fwname, sizeof(config.fwname),
			 "qca/crbtfw%02x.tlv", rom_ver);
	} else {
		snprintf(config.fwname, sizeof(config.fwname),
			 "qca/rampatch_%08x.bin", soc_ver);
	}

	err = qca_download_firmware(hdev, &config);
	if (err < 0) {
		BT_ERR("%s: QCA Failed to download patch (%d)", hdev->name,
		       err);
		return err;
	}

	/* Give the controller some time to get ready to receive the NVM */
	msleep(10);

	/* Download NVM configuration */
	config.type = TLV_TYPE_NVM;
	if (soc_type == QCA_WCN3990)
		snprintf(config.fwname, sizeof(config.fwname),
			 "qca/crnv%02x.bin", rom_ver);
	else
		snprintf(config.fwname, sizeof(config.fwname),
			 "qca/nvm_%08x.bin", soc_ver);

	err = qca_download_firmware(hdev, &config);
	if (err < 0) {
		BT_ERR("%s: QCA Failed to download NVM (%d)", hdev->name,
		       err);
		return err;
	}

	/* Perform HCI reset */
	err = qca_send_reset(hdev);
	if (err < 0) {
		BT_ERR("%s: QCA Failed to run HCI_RESET (%d)", hdev->name,
		       err);
		return err;
	}

	BT_INFO("%s: QCA setup on UART is completed", hdev->name);

	return 0;
}
EXPORT_SYMBOL_GPL(qca_uart_setup);

int qca_set_bdaddr(struct hci_dev *hdev, const bdaddr_t *bdaddr)
{
	bdaddr_t bdaddr_swapped;
	struct sk_buff *skb;
	int err;

	/* Unlike the HCI convention, this vendor command takes the address
	 * most significant byte first (verified: sending it unswapped gives
	 * a reversed address).
	 */
	baswap(&bdaddr_swapped, (bdaddr_t *)bdaddr);

	skb = __hci_cmd_sync_ev(hdev, EDL_WRITE_BD_ADDR_OPCODE, 6,
				&bdaddr_swapped, HCI_VENDOR_PKT,
				HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		err = PTR_ERR(skb);
		BT_ERR("%s: QCA Change address cmd failed (%d)", hdev->name,
		       err);
		return err;
	}

	kfree_skb(skb);

	return 0;
}
EXPORT_SYMBOL_GPL(qca_set_bdaddr);

MODULE_AUTHOR("Ben Young Tae Kim <ytkim@qca.qualcomm.com>");
MODULE_DESCRIPTION("Bluetooth support for Qualcomm Atheros family ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
