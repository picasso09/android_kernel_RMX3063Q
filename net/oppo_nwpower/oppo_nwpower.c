/***********************************************************
** Copyright (C), 2009-2019, OPPO Mobile Comm Corp., Ltd.
** VENDOR_EDIT
** File: - oppo_nwpower.c
** Description: BugID:2120730, Add for classify glink wakeup services and count IPA wakeup.
**
** Version: 1.0
** Date : 2019/07/31
** Author: Asiga@PSW.NW.DATA
**
** ------------------ Revision History:------------------------
** <author> <data> <version > <desc>
** Asiga 2019/07/31 1.0 build this module
** Asiga 2019/11/12 1.1 Adapt for MTK.
****************************************************************/
#include <linux/types.h>
#include <linux/module.h>
#include <linux/atomic.h>
#include <linux/skbuff.h>
#include <linux/err.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <net/oppo_nwpower.h>

#define CHANNEL_SUM            60
#define IP_SUM                 100

static atomic_t qmi_wakeup_hook_boot = ATOMIC_INIT(0);
static u64 channel_wakeup_times[CHANNEL_SUM][4] = {{0}};

static void oppo_ipv4_hook_work_callback(struct work_struct *work);
static void oppo_print_qmi_wakeup(bool unsl);
static void oppo_print_ipa_wakeup(bool unsl);
static void oppo_reset_qmi_wakeup(void);
static void oppo_reset_ipa_wakeup(void);
static int oppo_nwpower_send_to_user(int msg_type,char *msg_data, int msg_len);
static void oppo_nwpower_netlink_rcv(struct sk_buff *skb);
static int oppo_nwpower_netlink_rcv_msg(struct sk_buff *skb, struct nlmsghdr *nlh);
static int oppo_nwpower_netlink_init(void);
static void oppo_nwpower_netlink_exit(void);

extern atomic_t ipa_first_msg;
extern atomic_t ipa_wakeup_hook_boot;

struct oppo_ipv4_hook_struct oppo_ipv4_hook = {
	.ipa_wakeup = {0},
};
DECLARE_WORK(oppo_ipv4_hook_work, oppo_ipv4_hook_work_callback);

//Netlink
enum{
	NW_POWER_ANDROID_PID                    = 0x11,
	NW_POWER_BOOT_MONITOR                   = 0x12,
	NW_POWER_STOP_MONITOR                   = 0x13,
	NW_POWER_STOP_MONITOR_UNSL              = 0x14,
	NW_POWER_UNSL_MONITOR                   = 0x15,
};
static DEFINE_MUTEX(netlink_mutex);
static u32 oppo_nwpower_pid = 0;
static struct sock *oppo_nwpower_sock;
//TOP5 QMI + TOP5 IPA + Glink
static u64 unsl_msg[11] = {0};

static void oppo_ipv4_hook_work_callback(struct work_struct *work) {
	int i;
	int j;
	u32 count;
	u64 temp_sort;
	bool handle = false;
	for (i = 0; i < IP_SUM; ++i) {
		if (oppo_ipv4_hook.addr == (oppo_ipv4_hook.ipa_wakeup[i] & 0xFFFFFFFF)) {
			count = (oppo_ipv4_hook.ipa_wakeup[i] & 0xFFFC00000000000) >> 50;
			oppo_ipv4_hook.ipa_wakeup[i] = (u64)(++count) << 50 | oppo_ipv4_hook.ipa_wakeup[i];
			handle = true;
			break;
		} else if (oppo_ipv4_hook.ipa_wakeup[i] == 0) {
			count = 1 << 18 | from_kuid_munged(&init_user_ns, oppo_ipv4_hook.uid);
			oppo_ipv4_hook.ipa_wakeup[i] = (u64)count << 32 | oppo_ipv4_hook.addr;
			handle = true;
			break;
		}
	}
	if (!handle) {
		count = 1 << 18 | from_kuid_munged(&init_user_ns, oppo_ipv4_hook.uid);
		oppo_ipv4_hook.ipa_wakeup[99] = (u64)count << 32 | oppo_ipv4_hook.addr;
	}
	//Insert sort
	for (i = 1; i < IP_SUM; ++i) {
		temp_sort = oppo_ipv4_hook.ipa_wakeup[i];
		count = (temp_sort & 0xFFFC00000000000) >> 50;
		j = i - 1;
		while (j >=0 && count > ((oppo_ipv4_hook.ipa_wakeup[j] & 0xFFFC00000000000) >> 50)) {
			oppo_ipv4_hook.ipa_wakeup[j+1] = oppo_ipv4_hook.ipa_wakeup[j];
			--j;
		}
		oppo_ipv4_hook.ipa_wakeup[j+1] = temp_sort;
	}
	printk("[oppo_nwpower] IPAWakeup: IP: %d, UID: %d.",
		oppo_ipv4_hook.addr, from_kuid_munged(&init_user_ns, oppo_ipv4_hook.uid));
	atomic_set(&ipa_first_msg, 0);
}

extern void oppo_match_qmi_wakeup(int passageway, int channel) {
	int i;
	bool isNew = true;
	if (atomic_read(&qmi_wakeup_hook_boot) == 1) {
		for (i = 0; i < CHANNEL_SUM; ++i) {
			if (channel_wakeup_times[i][0] == 1 && channel_wakeup_times[i][1] == passageway && channel_wakeup_times[i][2] == channel) {
				isNew = false;
				channel_wakeup_times[i][3] += 1;
				if (passageway == 3 && (channel == 0 || channel == 1)) {
					printk("[oppo_nwpower] QmiWakeup: ChannelID: %d(IPA), PassagewayID: %d, Count: %d",
						channel_wakeup_times[i][2], channel_wakeup_times[i][1], channel_wakeup_times[i][3]);
				} else {
					printk("[oppo_nwpower] QmiWakeup: ChannelID: %d, PassagewayID: %d, Count: %d",
						channel_wakeup_times[i][2], channel_wakeup_times[i][1], channel_wakeup_times[i][3]);
				}
				break;
			}
		}
		if (isNew) {
			for (i = 0; i < CHANNEL_SUM; ++i) {
				if (channel_wakeup_times[i][0] == 0) {
					channel_wakeup_times[i][0] = 1;
					channel_wakeup_times[i][1] = passageway;
					channel_wakeup_times[i][2] = channel;
					channel_wakeup_times[i][3] = 1;
					break;
				}
			}
		}
	}
}

static void oppo_print_qmi_wakeup(bool unsl) {
	u64 temp[5][4] = {{0}};
	u64 max_count = 0;
	u64 max_count_id = 0;
	int j;
	int i;
	int k;
	for (j = 0; j < 5; ++j) {
		for (i = 0; i < CHANNEL_SUM; ++i) {
			if (channel_wakeup_times[i][0] == 1 && channel_wakeup_times[i][3] > max_count) {
				max_count = channel_wakeup_times[i][3];
				max_count_id = i;
			}
		}
		for (k = 0;k < 4; ++k) {
			temp[j][k] = channel_wakeup_times[max_count_id][k];
		}
		max_count = 0;
		channel_wakeup_times[max_count_id][3] = 0;
		if (unsl) {
			if (temp[j][3] > 0) unsl_msg[j] = temp[j][2] << 32 | ((u32)temp[j][1] << 16 | (u16)temp[j][3]);
		}
		if (temp[j][3] > 0) printk("[oppo_nwpower] QmiWakeupMax[%d]: ChannelID: %d, PassagewayID: %d, Count: %d",
			j, temp[j][2], temp[j][1], temp[j][3]);
	}
}

static void oppo_print_ipa_wakeup(bool unsl) {
	int i;
	u32 count;
	for (i = 0; i < 5;++i) {
		if (unsl) {
			unsl_msg[5+i] = oppo_ipv4_hook.ipa_wakeup[i];
		}
		count = (oppo_ipv4_hook.ipa_wakeup[i] & 0xFFFC00000000000) >> 50;
		if (count > 0) {
			printk("[oppo_nwpower] IPAWakeupMAX[%d]: IP: %d, UID: %d, Count: %d",
				i, oppo_ipv4_hook.ipa_wakeup[i] & 0xFFFFFFFF,
				(oppo_ipv4_hook.ipa_wakeup[i] & 0x3FFFF00000000) >> 32, count);
		}
	}
}

static void oppo_reset_qmi_wakeup() {
	int i;
	for (i = 0; i < CHANNEL_SUM; ++i) {
		if (channel_wakeup_times[i][0] == 1) {
			channel_wakeup_times[i][3] = 0;
		}
	}
}

static void oppo_reset_ipa_wakeup() {
	int i;
	for (i = 0; i < IP_SUM; ++i) {
		oppo_ipv4_hook.ipa_wakeup[i] = 0;
	}
}

extern void oppo_nwpower_hook_on(bool normal) {
	//if own Netlink is ok, ignore sla
	if (normal && oppo_nwpower_pid > 0) {
		return;
	}
	atomic_set(&qmi_wakeup_hook_boot, 1);
	atomic_set(&ipa_wakeup_hook_boot, 1);
}

extern void oppo_nwpower_hook_off(bool normal, bool unsl) {
	//if own Netlink is ok, ignore sla
	if (normal && oppo_nwpower_pid > 0) {
		return;
	}
	atomic_set(&qmi_wakeup_hook_boot, 0);
	atomic_set(&ipa_wakeup_hook_boot, 0);

	oppo_print_qmi_wakeup(unsl);
	oppo_reset_qmi_wakeup();

	oppo_print_ipa_wakeup(unsl);
	oppo_reset_ipa_wakeup();

	if (unsl) {
		oppo_nwpower_send_to_user(NW_POWER_UNSL_MONITOR, (char*)unsl_msg, sizeof(unsl_msg));
		memset(unsl_msg, 0x0, sizeof(unsl_msg));
	}
}

static int oppo_nwpower_send_to_user(int msg_type,char *msg_data, int msg_len) {
	int ret = 0;
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	if (oppo_nwpower_pid == 0) {
		printk("[oppo_nwpower] netlink: oppo_nwpower_pid = 0.\n");
		return -1;
	}
	skb = alloc_skb(NLMSG_SPACE(msg_len), GFP_ATOMIC);
	if (skb == NULL) {
		printk("[oppo_nwpower] netlink: alloc_skb failed.\n");
		return -2;
	}
	nlh = nlmsg_put(skb, 0, 0, msg_type, NLMSG_ALIGN(msg_len), 0);
	if (nlh == NULL) {
		printk("[oppo_nwpower] netlink: nlmsg_put failed.\n");
		nlmsg_free(skb);
		return -3;
	}
	nlh->nlmsg_len = NLMSG_HDRLEN + NLMSG_ALIGN(msg_len);
	if(msg_data != NULL) {
		memcpy((char*)NLMSG_DATA(nlh), msg_data, msg_len);
	}
	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;
	ret = netlink_unicast(oppo_nwpower_sock, skb, oppo_nwpower_pid, MSG_DONTWAIT);
	if(ret < 0) {
		printk(KERN_ERR "[oppo_nwpower] netlink: netlink_unicast failed, ret = %d.\n",ret);
		return -4;
	}
	return 0;
}

static void oppo_nwpower_netlink_rcv(struct sk_buff *skb) {
	mutex_lock(&netlink_mutex);
	netlink_rcv_skb(skb, &oppo_nwpower_netlink_rcv_msg);
	mutex_unlock(&netlink_mutex);
}

static int oppo_nwpower_netlink_rcv_msg(struct sk_buff *skb, struct nlmsghdr *nlh) {
	int ret = 0;
	switch (nlh->nlmsg_type) {
		case NW_POWER_ANDROID_PID:
			oppo_nwpower_pid = NETLINK_CB(skb).portid;
			printk("[oppo_nwpower] netlink: oppo_nwpower_pid = %d.\n",oppo_nwpower_pid);
			break;
		case NW_POWER_BOOT_MONITOR:
			oppo_nwpower_hook_on(false);
			printk("[oppo_nwpower] netlink: hook_on.\n");
			break;
		case NW_POWER_STOP_MONITOR:
			oppo_nwpower_hook_off(false, false);
			printk("[oppo_nwpower] netlink: hook_off.\n");
			break;
		case NW_POWER_STOP_MONITOR_UNSL:
			oppo_nwpower_hook_off(false, true);
			printk("[oppo_nwpower] netlink: hook_off_unsl.");
			break;
		default:
			return -EINVAL;
	}
	return ret;
}

static int oppo_nwpower_netlink_init(void) {
	struct netlink_kernel_cfg cfg = {
		.input = oppo_nwpower_netlink_rcv,
	};
	oppo_nwpower_sock = netlink_kernel_create(&init_net, NETLINK_OPPO_NWPOWERSTATE, &cfg);
	return oppo_nwpower_sock == NULL ? -ENOMEM : 0;
}

static void oppo_nwpower_netlink_exit(void) {
	netlink_kernel_release(oppo_nwpower_sock);
	oppo_nwpower_sock = NULL;
}

static int __init oppo_nwpower_init(void) {
	int ret = 0;
	ret = oppo_nwpower_netlink_init();
	if (ret < 0) {
		printk("[oppo_nwpower] netlink: failed to init netlink.\n");
	} else {
		printk("[oppo_nwpower] netlink: init netlink successfully.\n");
	}
	return ret;
}

static void __exit oppo_nwpower_fini(void) {
	oppo_nwpower_netlink_exit();
}

module_init(oppo_nwpower_init);
module_exit(oppo_nwpower_fini);