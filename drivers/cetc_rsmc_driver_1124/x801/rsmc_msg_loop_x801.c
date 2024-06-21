/*
#include <securec.h>
#include <cetc_platform/log/cetc_log.h>
*/
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netlink.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/semaphore.h>
#include <linux/skbuff.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <net/sock.h>
#include <net/netlink.h>
#include <uapi/linux/netlink.h>
#include <linux/types.h>
#include <net/net_namespace.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <net/genetlink.h>

#include "../module_type.h"
#include "rsmc_msg_loop_x801.h"
#include "rsmc_spi_x801.h"
#include "rsmc_device_x801.h"

#ifdef CETCLOG_TAG
#undef CETCLOG_TAG
#endif
#define CETCLOG_TAG RSMC_MSG_LOOP
//CETCLOG_REGIST();
MODULE_LICENSE("GPL");

enum channel_status {
	NETLINK_MSG_LOOP_EXIT = 0,
	NETLINK_MSG_LOOP_INIT,
};

/*
#ifdef NETLINK_RSMC
#undef NETLINK_RSMC
#endif
#define NETLINK_RSMC 43
*/

#define GENLMSG_DATA(glh)	((char *)(NLMSG_DATA(glh) + GENL_HDRLEN))
#define NLA_DATA(nla)		((char *)((char*)(nla) + NLA_HDRLEN))
#define RSMC_HEART_BEAT_TIMEOUT msecs_to_jiffies(5000)
#define RSMC_SOC_HEART_BEAT_TIMEOUT msecs_to_jiffies(5000)
#define RSMC_SOC_SEND_HEART_INTERVAL msecs_to_jiffies(2000)
#define MAX_BIN_LEN 100

#define DTS_NODE_CETC_RSMC "cetc_rsmc"
#define DTS_PROP_RSMC_ENABLE "rsmc_enable"
#define RSMC_GEML_MSG_CMD 1
#define RMSC_GENE_NAME "rsmcgenl"

static DEFINE_MUTEX(g_recv_mtx);
static DEFINE_MUTEX(g_send_mtx);

/* module IDs defined for each module */
enum install_model {
	RSMC_DEVICE = 0,
	MODEL_NUM,
};

#define MAX_NOR_MSG_LEN 100
#define MAX_FST_MSG_LEN 50
#define MAX_SPI_TRANS_MSG_LEN 50

int rsmc_netlink_recv(struct sk_buff *skb, struct genl_info *info);

struct genl_ops rsmc_genl_ops = {
	.cmd = RSMC_GEML_MSG_CMD,
	.flags = 0,
	.doit = rsmc_netlink_recv,
	.dumpit = NULL,
};

static struct genl_family rsmc_genl_family = {
	.name = "rsmcgenl",
	.version = 1,
	.maxattr = 4,
	.ops = &rsmc_genl_ops,
	.n_ops = 1,
};

/* The context of the message handle */
struct handle_ctx {
	/* Netlink socket fd */
	struct sock *nlfd;

	/* Save user space progress pid when user space netlink registering. */
	unsigned int native_pid;

	/* Tasks for send messages. */
	struct task_struct *up_task;

	/* Tasks for receive messages. */
	struct task_struct *dn_task;

	/* Channel status */
	int chan_state;

	/* Semaphore of the message sent */
	struct semaphore up_sema;

	spinlock_t up_lock;

	struct semaphore dn_inter_sema;

	spinlock_t dn_inter_lock;

	spinlock_t dn_spi_lock;

	/* Message processing callback functions */
	msg_process *mod_cb[MODEL_NUM];

	struct timer_list heartbeat_timer;

	volatile unsigned long native_jiffies;

	volatile unsigned long soc_rx_jiffies;

	volatile unsigned long soc_tx_jiffies;

	volatile bool heartbeat_running;

	volatile struct list_head *dn_inter_list[MAX_NOR_MSG_LEN + 4]; // add buffer 4

	volatile int dn_inter_end;

	volatile int dn_inter_start;

	/* Message send to x801 */
	volatile struct list_head *dn_spi_list[MAX_NOR_MSG_LEN + 4]; // add buffer 4

	volatile int dn_spi_end;

	volatile int dn_spi_start;

	volatile struct list_head *up_list[MAX_NOR_MSG_LEN + 4]; // add buffer 4

	volatile int up_end;

	volatile int up_start;
};

static struct handle_ctx g_nl_ctx = {0};

/* mesage map entry index */
enum map_index {
	MAP_KEY_INDEX = 0,
	MAP_VALUE_INDEX,
	MAP_ENTITY_NUM,
};

struct model_map {
	enum install_model model;
	model_reg *reg;
	model_unreg *unreg;
};

const static struct model_map module_init_map[MODEL_NUM] = {
	{RSMC_DEVICE, rsmc_device_reg_x801, rsmc_device_unreg_x801},
};

/* Message mapping table for external modules */
const static u16 cmd_module_map[][MAP_ENTITY_NUM] = {
	{CMD_DN_INIT_REQ, RSMC_DEVICE},
	{CMD_DN_ANY_REQ, RSMC_DEVICE},
	{CMD_DN_DTX_REQ, RSMC_DEVICE},
	{CMD_INTER_INIT_REQ, RSMC_DEVICE},
	{CMD_INTER_HB_TIMER_REQ, RSMC_DEVICE},
	{CMD_INTER_START_SPI_TIMER, RSMC_DEVICE},
	{CMD_INTER_STOP_SPI_TIMER, RSMC_DEVICE},
	{CMD_INTER_WAIT_STA_TIMEOUT, RSMC_DEVICE},
};

u32 get_rsmc_spi_status(void)
{
	u32 status = get_spi_status();
	return status;
}

void rsmc_nl_up_msg_notify(struct msg_head *msg)
{
	struct msg_entity *p = NULL;
	u32 msg_len;
	int msg_idx, ret;

	if (msg == NULL)
		return;
	cetclog_info("%s: enter, type:%d,module:%d,len:%d",
			__func__, msg->type, msg->module, msg->len);
	if (g_nl_ctx.chan_state != NETLINK_MSG_LOOP_INIT) {
		cetclog_err("%s: module not inited", __func__);
		return;
	}
	msg_len = sizeof(struct list_head) + msg->len;
	p = kmalloc(msg_len, GFP_ATOMIC);
	if (p == NULL) {
		cetclog_err("%s: kmalloc failed", __func__);
		return;
	}
	ret = memcpy_s(&p->msg, msg_len, msg, msg->len);
	if (ret != EOK) {
		cetclog_err("%s: memcpy_s fail", __func__);
		kfree(p);
		return;
	}
	spin_lock_bh(&g_nl_ctx.up_lock);
	msg_idx = g_nl_ctx.up_end;
	if (msg_idx >= MAX_NOR_MSG_LEN || msg_idx < 0) {
		spin_unlock_bh(&g_nl_ctx.up_lock);
		cetclog_err("%s: idx err", __func__);
		kfree(p);
		return;
	}
	if ((g_nl_ctx.up_start + MAX_NOR_MSG_LEN - msg_idx) % MAX_NOR_MSG_LEN == 1) {
		spin_unlock_bh(&g_nl_ctx.up_lock);
		cetclog_err("%s: msg overlap", __func__);
		kfree(p);
		return;
	}
	g_nl_ctx.up_list[msg_idx] = (struct list_head *)p;
	msg_idx = (msg_idx + 1) % MAX_NOR_MSG_LEN;
	g_nl_ctx.up_end = msg_idx;
	barrier();
	if (g_nl_ctx.up_end != msg_idx)
		cetclog_err("%s: idx not new", __func__);
	spin_unlock_bh(&g_nl_ctx.up_lock);

	up(&g_nl_ctx.up_sema);

	cetclog_info("%s: exit", __func__);
}

void rsmc_nl_dn_msg_notify_spi(struct msg_head *msg)
{
	struct msg_entity *p = NULL;
	u32 msg_len;
	int msg_idx;
	int ret;

	if (msg == NULL)
		return;
	cetclog_info("%s: enter, type:%d,module:%d,len:%d",
		__func__, msg->type, msg->module, msg->len);
	if (g_nl_ctx.chan_state != NETLINK_MSG_LOOP_INIT) {
		cetclog_err("%s: module not inited", __func__);
		return;
	}
	msg_len = sizeof(struct list_head) + msg->len;
	p = kmalloc(msg_len, GFP_ATOMIC);
	if (p == NULL) {
		cetclog_err("%s: kmalloc failed", __func__);
		return;
	}
	ret = memcpy_s(&p->msg, msg_len, msg, msg->len);
	if (ret != EOK) {
		kfree(p);
		return;
	}

	spin_lock(&g_nl_ctx.dn_spi_lock);
	msg_idx = g_nl_ctx.dn_spi_end;
	if (msg_idx >= MAX_NOR_MSG_LEN || msg_idx < 0) {
		spin_unlock(&g_nl_ctx.dn_spi_lock);
		cetclog_err("%s: idx err", __func__);
		kfree(p);
		return;
	}
	if ((g_nl_ctx.dn_spi_start + MAX_NOR_MSG_LEN - msg_idx) % MAX_NOR_MSG_LEN == 1) {
		rsmc_clear_dn_spi_msg_list();
		spin_unlock(&g_nl_ctx.dn_spi_lock);
		cetclog_err("%s: msg overlap", __func__);\
		kfree(p);
		return;
	}
	g_nl_ctx.dn_spi_list[msg_idx] = (struct list_head *)p;
	msg_idx = (msg_idx + 1) % MAX_NOR_MSG_LEN;
	g_nl_ctx.dn_spi_end = msg_idx;
	barrier();
	if (g_nl_ctx.dn_spi_end != msg_idx)
		cetclog_err("%s: idx not new", __func__);
	spin_unlock(&g_nl_ctx.dn_spi_lock);
	up_spi_sema();

	cetclog_info("%s: exit", __func__);
}

void rsmc_nl_dn_msg_notify_inter(struct msg_head *msg)
{
	struct msg_entity *p = NULL;
	u32 msg_len;
	int msg_idx;
	int ret;

	if (msg == NULL)
		return;
	cetclog_info("%s: enter, type:%d,module:%d,len:%d",
		__func__, msg->type, msg->module, msg->len);
	if (g_nl_ctx.chan_state != NETLINK_MSG_LOOP_INIT) {
		cetclog_err("%s: module not inited", __func__);
		return;
	}
	msg_len = sizeof(struct list_head) + msg->len;
	p = kmalloc(msg_len, GFP_ATOMIC);
	if (p == NULL) {
		cetclog_err("%s: kmalloc failed", __func__);
		return;
	}
	ret = memcpy_s(&p->msg, msg_len, msg, msg->len);
	if (ret != EOK) {
		kfree(p);
		return;
	}

	spin_lock(&g_nl_ctx.dn_inter_lock);
	msg_idx = g_nl_ctx.dn_inter_end;
	if (msg_idx >= MAX_NOR_MSG_LEN || msg_idx < 0) {
		spin_unlock(&g_nl_ctx.dn_inter_lock);
		cetclog_err("%s: idx err", __func__);
		kfree(p);
		return;
	}
	if ((g_nl_ctx.dn_inter_start + MAX_NOR_MSG_LEN - msg_idx) % MAX_NOR_MSG_LEN == 1) {
		rsmc_clear_dn_inter_msg_list();
		spin_unlock(&g_nl_ctx.dn_inter_lock);
		cetclog_err("%s: msg overlap", __func__);
		kfree(p);
		return;
	}
	g_nl_ctx.dn_inter_list[msg_idx] = (struct list_head *)p;
	msg_idx = (msg_idx + 1) % MAX_NOR_MSG_LEN;
	g_nl_ctx.dn_inter_end = msg_idx;
	barrier();
	if (g_nl_ctx.dn_inter_end != msg_idx)
		cetclog_err("%s: idx not new", __func__);
	spin_unlock(&g_nl_ctx.dn_inter_lock);
	up(&g_nl_ctx.dn_inter_sema);

	cetclog_info("%s: exit", __func__);
}

void rsmc_nl_dn_msg_notify_inter_nonblock(struct msg_head *msg)
{
	struct msg_entity *p = NULL;
	u32 msg_len;
	int msg_idx;
	int ret;

	if (msg == NULL)
		return;
	cetclog_info("%s: enter, type:%d,module:%d,len:%d",
		__func__, msg->type, msg->module, msg->len);
	if (g_nl_ctx.chan_state != NETLINK_MSG_LOOP_INIT) {
		cetclog_err("%s: module not inited", __func__);
		return;
	}
	msg_len = sizeof(struct list_head) + msg->len;
	p = kmalloc(msg_len, GFP_ATOMIC);
	if (p == NULL) {
		cetclog_err("%s: kmalloc failed", __func__);
		return;
	}
	ret = memcpy_s(&p->msg, msg_len, msg, msg->len);
	if (ret != EOK) {
		kfree(p);
		return;
	}

	msg_idx = g_nl_ctx.dn_inter_end;
	if (msg_idx >= MAX_NOR_MSG_LEN || msg_idx < 0) {
		cetclog_err("%s: idx err", __func__);
		kfree(p);
		return;
	}
	if ((g_nl_ctx.dn_inter_start + MAX_NOR_MSG_LEN - msg_idx) % MAX_NOR_MSG_LEN == 1) {
		rsmc_clear_dn_inter_msg_list();
		cetclog_err("%s: msg overlap", __func__);
		kfree(p);
		return;
	}
	g_nl_ctx.dn_inter_list[msg_idx] = (struct list_head *)p;
	msg_idx = (msg_idx + 1) % MAX_NOR_MSG_LEN;
	g_nl_ctx.dn_inter_end = msg_idx;
	barrier();
	if (g_nl_ctx.dn_inter_end != msg_idx)
		cetclog_err("%s: idx not new", __func__);
	up(&g_nl_ctx.dn_inter_sema);

	cetclog_info("%s: exit", __func__);
}

void rsmc_nl_dn_msg_dipatch(struct msg_head *msg)
{
	if (msg == NULL) {
		cetclog_err("%s: nl msg null", __func__);
		return;
	}
	if ((msg->type == CMD_DN_ANY_REQ) || (msg->type == CMD_DN_DTX_REQ))
		rsmc_nl_dn_msg_notify_spi(msg);
	else
		rsmc_nl_dn_msg_notify_inter(msg);
}

void rsmc_clear_dn_inter_msg_list(void)
{
	int ret, i;
	for (i = 0; i < MAX_NOR_MSG_LEN; i++) {
		struct msg_entity *msg = (struct msg_entity *)g_nl_ctx.dn_inter_list[i];
		if (msg != NULL)
			kfree(msg);
	}
	ret = memset_s(g_nl_ctx.dn_inter_list, sizeof(struct list_head *) * (MAX_NOR_MSG_LEN + 4),
		0, sizeof(struct list_head *) * MAX_NOR_MSG_LEN);
	if (ret != EOK)
		cetclog_err("%s: dn_list memset_s fail", __func__);
	g_nl_ctx.dn_inter_start = 0;
	g_nl_ctx.dn_inter_end = 0;
}

void rsmc_clear_dn_spi_msg_list(void)
{
	int ret, i;
	for (i = 0; i < MAX_NOR_MSG_LEN; i++) {
		struct msg_entity *spi_msg = (struct msg_entity *)g_nl_ctx.dn_spi_list[i];
		if (spi_msg != NULL)
			kfree(spi_msg);
	}
	ret = memset_s(g_nl_ctx.dn_spi_list, sizeof(struct list_head *) * (MAX_NOR_MSG_LEN + 4),
		0, sizeof(struct list_head *) * MAX_NOR_MSG_LEN);
	if (ret != EOK)
		cetclog_err("%s: dn_spi_list memset_s fail", __func__);
	g_nl_ctx.dn_spi_start = 0;
	g_nl_ctx.dn_spi_end = 0;
}

void rsmc_process_cmd_x801(struct msg_head *cmd)
{
	int i;

	if (cmd == NULL)
		return;
	cetclog_info("%s: enter, type:%d,module:%d",
		__func__, cmd->type, cmd->module);

	if (cmd->module != MODULE_TYPE_KNL) {
		cetclog_err("%s: module not kernel %d", __func__, cmd->module);
		return;
	}
	for (i = 0; i < sizeof(cmd_module_map) / (sizeof(u16) * MAP_ENTITY_NUM); i++) {
		if (cmd_module_map[i][MAP_KEY_INDEX] != cmd->type)
			continue;
		if (g_nl_ctx.mod_cb[cmd_module_map[i][MAP_VALUE_INDEX]] == NULL)
			break;
		g_nl_ctx.mod_cb[cmd_module_map[i][MAP_VALUE_INDEX]](cmd);
		break;
	}

	cetclog_info("%s: exit", __func__);
}

static char get_crc(char *data, int len)
{
	int i;
	char crc = 0;
	for (i = 0; i < len; i++)
		crc ^= data[i];
	return crc;
}

static void send_heartbeat_to_chip(void)
{
	char cmd[MAX_BIN_LEN] = {0};
	struct any_msg hrt_msg = {0};
	int cmd_len, len;

	cetclog_info("%s: enter", __func__);
	hrt_msg.head.type = CMD_DN_ANY_REQ;
	hrt_msg.head.module = MODULE_TYPE_KNL;
	hrt_msg.head.len = sizeof(struct any_msg);
	cmd_len = sprintf_s(cmd, MAX_BIN_LEN, "HRT,%u", jiffies_to_msecs(jiffies));
	len = sprintf_s(hrt_msg.data, sizeof(hrt_msg.data), "$%s*%.2x\r\n", cmd, get_crc(cmd, cmd_len));
	if (len <= 0) {
		cetclog_info("%s: sprintf fail", __func__);
		return;
	}
	rsmc_nl_dn_msg_notify_spi((struct msg_head *)&hrt_msg);
}

void send_up_soc_err(void)
{
	struct any_msg up_msg = {0};
	up_msg.head.type = CMD_UP_SOC_ERR_IND;
	up_msg.head.module = MODULE_TYPE_CTRL;
	up_msg.head.len = sizeof(struct any_msg);
	cetclog_info("%s: data len: %d,data: %s", __func__, strlen(up_msg.data), up_msg.data);
	send_msg_to_ctrl_x801((struct msg_head *)&up_msg);
}

static void rsmc_heartbeat_process(void)
{
	cetclog_info("%s: heartbeat", __func__);
	g_nl_ctx.native_jiffies = (unsigned long)jiffies;
}

static void rsmc_heartbeat_callback(struct timer_list *t)
{
	if (!g_nl_ctx.heartbeat_running)
		return;
	if (jiffies - g_nl_ctx.native_jiffies > RSMC_HEART_BEAT_TIMEOUT) {
		struct enable_msg msg = {0};
		msg.head.type = CMD_INTER_INIT_REQ;
		msg.head.module = MODULE_TYPE_KNL;
		msg.head.len = sizeof(struct enable_msg);
		msg.status = 0;
		cetclog_info("%s: native heartbeat lost disable device", __func__);
		rsmc_clear_dn_inter_msg_list();
		rsmc_clear_dn_spi_msg_list();
		rsmc_nl_dn_msg_notify_inter_nonblock((struct msg_head *)&msg);
	} else if (jiffies - g_nl_ctx.soc_rx_jiffies > RSMC_SOC_HEART_BEAT_TIMEOUT) {
		cetclog_info("%s: x801 heartbeat lost report soc err", __func__);
		send_up_soc_err();
	} else {
		struct msg_head msg = {0};
		msg.type = CMD_INTER_HB_TIMER_REQ;
		msg.module = MODULE_TYPE_KNL;
		msg.len = sizeof(struct msg_head);
		rsmc_nl_dn_msg_notify_inter_nonblock((struct msg_head *)&msg);
	}
}

void rsmc_start_heartbeat_x801(void)
{
	if (g_nl_ctx.heartbeat_running) {
		del_timer_sync(&g_nl_ctx.heartbeat_timer);
	} else {
		g_nl_ctx.heartbeat_running = true;
		rsmc_heartbeat_process();
	}
	timer_setup(&g_nl_ctx.heartbeat_timer,
		rsmc_heartbeat_callback,
		TIMER_IRQSAFE);
	g_nl_ctx.heartbeat_timer.expires = jiffies + HZ;
	add_timer(&g_nl_ctx.heartbeat_timer);
	cetclog_info("%s", __func__);
	if (jiffies - g_nl_ctx.soc_tx_jiffies > RSMC_SOC_SEND_HEART_INTERVAL)
		send_heartbeat_to_chip();
}

void rsmc_stop_heartbeat_x801(void)
{
	if (g_nl_ctx.heartbeat_running) {
		del_timer_sync(&g_nl_ctx.heartbeat_timer);
		g_nl_ctx.heartbeat_running = false;
	}
	cetclog_info("%s", __func__);
}

void rsmc_update_soc_rx_jiffies(void)
{
	cetclog_info("%s", __func__);
	g_nl_ctx.soc_rx_jiffies = (unsigned long)jiffies;
}

void rsmc_update_soc_tx_jiffies(void)
{
	cetclog_info("%s", __func__);
	g_nl_ctx.soc_tx_jiffies = (unsigned long)jiffies;
}

int rsmc_netlink_recv(struct sk_buff *__skb, struct genl_info *info)
{
	struct nlmsghdr *nlh = NULL;
	struct sk_buff *skb = NULL;
	struct nlattr *nla = NULL;
	if (g_nl_ctx.chan_state != NETLINK_MSG_LOOP_INIT) {
		cetclog_err("%s: module not inited", __func__);
		return -EINVAL;
	}
	if (__skb == NULL) {
		cetclog_err("%s: __skb null", __func__);
		return -EINVAL;
	}
	skb = skb_get(__skb);
	if (skb == NULL) {
		cetclog_err("%s: skb null", __func__);
		return -EINVAL;
	}
	mutex_lock(&g_recv_mtx);
	if (skb->len < NLMSG_HDRLEN) {
		cetclog_err("%s: skb len error", __func__);
		goto skb_free;
	}
	nlh = info->nlhdr;
	if (nlh == NULL) {
		cetclog_err("%s: nlh = NULL", __func__);
		goto skb_free;
	}
	if ((nlh->nlmsg_len < sizeof(struct nlmsghdr)) ||
		(skb->len < nlh->nlmsg_len)) {
		cetclog_err("%s: nlmsg len error", __func__);
		goto skb_free;
	}
	nla = (struct nlattr*)GENLMSG_DATA(nlh);
	switch (nla->nla_type) {
	case NL_MSG_REG:
		cetclog_info("rsmc proc ML_MSG_REG\n");
		g_nl_ctx.native_pid = nlh->nlmsg_pid;
		break;
	case NL_MSG_HEARTBEAT:
		cetclog_info("rsmc proc NL_MSG_HEARTBEAT\n");
		rsmc_heartbeat_process();
		break;
	case NL_MSG_REQ:
		cetclog_info("rsmc proc NL_MSG_REQ\n");
		rsmc_nl_dn_msg_dipatch((struct msg_head *)NLA_DATA(nla));
		break;
	default:
		break;
	}
skb_free:
	kfree_skb(skb);
	mutex_unlock(&g_recv_mtx);
	return 0;
}

static struct msg_entity *rsmc_nl_dn_get_inter_msg(void)
{
	struct msg_entity *msg = NULL;
	int idx;

	spin_lock_bh(&g_nl_ctx.dn_inter_lock);
	if (g_nl_ctx.dn_inter_end == g_nl_ctx.dn_inter_start) {
		spin_unlock_bh(&g_nl_ctx.dn_inter_lock);
		return NULL;
	}
	idx = g_nl_ctx.dn_inter_start;
	if (idx >= MAX_NOR_MSG_LEN || idx < 0) {
		spin_unlock_bh(&g_nl_ctx.dn_inter_lock);
		cetclog_err("%s: idx error", __func__);
		return NULL;
	}
	if (g_nl_ctx.dn_inter_list[idx] == NULL) {
		cetclog_err("%s: list is null", __func__);
		msg = NULL;
	} else {
		msg = (struct msg_entity *)g_nl_ctx.dn_inter_list[idx];
		g_nl_ctx.dn_inter_list[idx] = NULL;
	}
	g_nl_ctx.dn_inter_start = (idx + 1) % MAX_NOR_MSG_LEN;
	spin_unlock_bh(&g_nl_ctx.dn_inter_lock);
	if (msg != NULL)
		cetclog_info("%s: msg id=%d", __func__, msg->msg.type);
	return msg;
}

struct msg_entity *rsmc_nl_dn_get_spi_msg(void)
{
	struct msg_entity *msg = NULL;
	int idx;

	spin_lock_bh(&g_nl_ctx.dn_spi_lock);
	if (g_nl_ctx.dn_spi_end == g_nl_ctx.dn_spi_start) {
		spin_unlock_bh(&g_nl_ctx.dn_spi_lock);
		return NULL;
	}
	idx = g_nl_ctx.dn_spi_start;
	if (idx >= MAX_NOR_MSG_LEN || idx < 0) {
		spin_unlock_bh(&g_nl_ctx.dn_spi_lock);
		cetclog_err("%s: idx error", __func__);
		return NULL;
	}
	if (g_nl_ctx.dn_spi_list[idx] == NULL) {
		cetclog_err("%s: list is null", __func__);
		msg = NULL;
	} else {
		msg = (struct msg_entity *)g_nl_ctx.dn_spi_list[idx];
		g_nl_ctx.dn_spi_list[idx] = NULL;
	}
	g_nl_ctx.dn_spi_start = (idx + 1) % MAX_NOR_MSG_LEN;
	spin_unlock_bh(&g_nl_ctx.dn_spi_lock);
	if (msg != NULL)
		cetclog_info("%s: msg id=%d", __func__, msg->msg.type);
	return msg;
}

static struct msg_entity *rsmc_nl_up_get_msg(void)
{
	struct msg_entity *msg = NULL;
	int idx;

	spin_lock_bh(&g_nl_ctx.up_lock);
	if (g_nl_ctx.up_end == g_nl_ctx.up_start) {
		spin_unlock_bh(&g_nl_ctx.up_lock);
		return NULL;
	}
	idx = g_nl_ctx.up_start;
	if (idx >= MAX_NOR_MSG_LEN || idx < 0) {
		spin_unlock_bh(&g_nl_ctx.up_lock);
		cetclog_err("%s: idx error", __func__);
		return NULL;
	}
	if (g_nl_ctx.up_list[idx] == NULL) {
		cetclog_err("%s: list is null", __func__);
		msg = NULL;
	} else {
		msg = (struct msg_entity *)g_nl_ctx.up_list[idx];
		g_nl_ctx.up_list[idx] = NULL;
	}
	g_nl_ctx.up_start = (idx + 1) % MAX_NOR_MSG_LEN;
	spin_unlock_bh(&g_nl_ctx.up_lock);
	if (msg != NULL)
		cetclog_info("%s: msg id=%d", __func__, msg->msg.type);
	return msg;
}

/* send a message to user space */
int rsmc_nl_up_notify_event_x801(struct msg_head *msg)
{
	int ret;
	int attr_len;
	struct sk_buff *skb = NULL;
	//struct nlmsghdr *nlh = NULL;
	struct nlattr *nla = NULL;
	//void *pdata = NULL;
	void *head = NULL;

	if (msg == NULL)
		return -1;

	mutex_lock(&g_send_mtx);
	if (!g_nl_ctx.native_pid) {
		cetclog_err("%s: err pid = %d\n",
			__func__, g_nl_ctx.native_pid);
		ret = -1;
		goto nty_end;
	}
	attr_len = nla_total_size(msg->len);
	skb = genlmsg_new(attr_len, GFP_ATOMIC);
	if (skb == NULL) {
		cetclog_err("%s: alloc skb fail\n", __func__);
		ret = -1;
		goto nty_end;
	}

	nla = genlmsg_put(skb, 0, 0, &rsmc_genl_family, 0, RSMC_GEML_MSG_CMD);
	if (nla == NULL) {
		kfree_skb(skb);
		skb = NULL;
		ret = -1;
		goto nty_end;
	}
/*
	pdata = NLA_DATA(nla);
	ret = memcpy_s(pdata, msg->len, msg, msg->len);
*/
	ret = nla_put(skb, 0, msg->len, msg);
	if (ret != EOK) {
		kfree_skb(skb);
		skb = NULL;
		ret = -1;
		goto nty_end;
	}
    head = genlmsg_data(nlmsg_data(nlmsg_hdr(skb)));
	genlmsg_end(skb, head);
	/* skb will be freed in netlink_unicast */
	ret = genlmsg_unicast(&init_net, skb, g_nl_ctx.native_pid);
	cetclog_info("%s,send msg to phy,ret:%d\n", __func__, ret);
nty_end:
	mutex_unlock(&g_send_mtx);
	return ret;
}

int rsmc_up_msg_thread(void *data)
{
	struct msg_entity *msg = NULL;

	cetclog_info("%s: enter", __func__);

	while (!kthread_should_stop()) {
		down(&g_nl_ctx.up_sema);
		if (g_nl_ctx.native_pid == 0)
			continue;
		msg = rsmc_nl_up_get_msg();
		while (msg != NULL) {
			rsmc_nl_up_notify_event_x801(&msg->msg);
			kfree(msg);
			msg = rsmc_nl_up_get_msg();
		}
	}
	return 0;
}

int rsmc_dn_msg_thread(void *data)
{
	struct msg_entity *msg = NULL;
	cetclog_info("%s: enter", __func__);
	while (!kthread_should_stop()) {
		down(&g_nl_ctx.dn_inter_sema);
		if (g_nl_ctx.native_pid == 0)
			continue;
		msg = rsmc_nl_dn_get_inter_msg();
		while (msg != NULL) {
			rsmc_process_cmd_x801(&msg->msg);
			kfree(msg);
			msg = rsmc_nl_dn_get_inter_msg();
		}
	}
	return 0;
}

/* netlink init function */
int rsmc_netlink_init(void)
{
	int ret;
	struct sched_param param;
/*
	struct netlink_kernel_cfg nb_nl_cfg = {
		.input = rsmc_netlink_recv,
	};

	g_nl_ctx.nlfd = netlink_kernel_create(&init_net,
		NETLINK_RSMC, &nb_nl_cfg);

	if (g_nl_ctx.nlfd == NULL) {
		cetclog_err("%s: netlink_handle_init failed", __func__);
		return -EINVAL;
	}
*/
	ret = genl_register_family(&rsmc_genl_family);
	if (ret) {
		cetclog_err("%s: generic register error\n", __func__);
		return -EINVAL;
	}
	sema_init(&g_nl_ctx.up_sema, 0);
	sema_init(&g_nl_ctx.dn_inter_sema, 0);
	spin_lock_init(&g_nl_ctx.up_lock);
	spin_lock_init(&g_nl_ctx.dn_inter_lock);
	spin_lock_init(&g_nl_ctx.dn_spi_lock);

	g_nl_ctx.up_task = kthread_run(
		rsmc_up_msg_thread,
		NULL,
		"rsmc_nl_up_thread");
	if (IS_ERR(g_nl_ctx.up_task)) {
		cetclog_err("%s: failed to create thread", __func__);
		g_nl_ctx.up_task = NULL;
		return -EINVAL;
	}
	param.sched_priority = MAX_RT_PRIO - 1;
	sched_setscheduler(g_nl_ctx.up_task, SCHED_FIFO, &param);

	g_nl_ctx.dn_task = kthread_run(
		rsmc_dn_msg_thread,
		NULL,
		"rsmc_nl_dn_thread");
	if (IS_ERR(g_nl_ctx.dn_task)) {
		cetclog_err("%s: failed to create thread", __func__);
		g_nl_ctx.dn_task = NULL;
		return -EINVAL;
	}
	param.sched_priority = MAX_RT_PRIO - 1;
	sched_setscheduler(g_nl_ctx.dn_task, SCHED_FIFO, &param);

	g_nl_ctx.chan_state = NETLINK_MSG_LOOP_INIT;

	return 0;
}

/* netlink deinit function */
void rsmc_netlink_exit(void)
{
/*
	if (g_nl_ctx.nlfd && g_nl_ctx.nlfd->sk_socket) {
		sock_release(g_nl_ctx.nlfd->sk_socket);
		g_nl_ctx.nlfd = NULL;
	}
*/
	genl_unregister_family(&rsmc_genl_family);
	cetclog_info("unregister genl family\n");
	if (g_nl_ctx.up_task != NULL) {
		kthread_stop(g_nl_ctx.up_task);
		g_nl_ctx.up_task = NULL;
	}
	if (g_nl_ctx.dn_task != NULL) {
		kthread_stop(g_nl_ctx.dn_task);
		g_nl_ctx.dn_task = NULL;
	}
}

int regist_model_x801(model_reg *fun, enum install_model model)
{
	msg_process *fn = NULL;

	if (fun == NULL)
		return -EINVAL;
	if (model > MODEL_NUM || model < 0) {
		cetclog_err("%s: invalid model, model is :%d", __func__, model);
		return -EINVAL;
	}
	fn = fun(rsmc_nl_up_msg_notify);
	if (fn == NULL) {
		cetclog_err("%s: fn null:%d", __func__, model);
		return -EINVAL;
	}

	g_nl_ctx.mod_cb[model] = fn;
	return 0;
}

int rsmc_main_thread_x801(void *data)
{
	model_reg *reg_fn = NULL;
	int rtn = 0;
	int model;

	cetclog_info("%s: enter", __func__);
	if (rsmc_netlink_init()) {
		cetclog_err("%s: init netlink_handle module failed", __func__);
		g_nl_ctx.chan_state = NETLINK_MSG_LOOP_EXIT;
		return -EINVAL;
	}

	for (model = 0; model < MODEL_NUM; model++) {
		if (module_init_map[model].model != model) {
			cetclog_err("%s: model init map error", __func__);
			return -EINVAL;
		}
		reg_fn = module_init_map[model].reg;
		if (reg_fn)
			rtn += regist_model_x801(reg_fn, model);
	}
	if (rtn < 0)
		return rtn;
	g_nl_ctx.up_start = 0;
	g_nl_ctx.up_end = 0;
	rtn = memset_s(g_nl_ctx.up_list, sizeof(struct list_head *) * (MAX_NOR_MSG_LEN + 4),
		0, sizeof(struct list_head *) * MAX_NOR_MSG_LEN);
	if (rtn != EOK)
		return -EINVAL;
	rsmc_clear_dn_inter_msg_list();
	rsmc_clear_dn_spi_msg_list();
	g_nl_ctx.heartbeat_running = false;
	cetclog_info("%s: netlink_handle module inited", __func__);
	return 0;
}

int rsmc_init_x801(void)
{
	struct task_struct *task = kthread_run(rsmc_main_thread_x801, NULL, "rsmc_main_thread_x801");
	if (IS_ERR(task)) {
		cetclog_err("%s: failed to create thread", __func__);
		task = NULL;
		return -EINVAL;
	}
	return 0;
}

void rsmc_exit_x801(void)
{
	model_unreg *unreg_fn = NULL;
	int model;
	for (model = 0; model < MODEL_NUM; model++) {
		if (module_init_map[model].model != model) {
			cetclog_err("%s: model init map error", __func__);
			continue;
		}
		unreg_fn = module_init_map[model].unreg;
		if (unreg_fn != NULL)
			unreg_fn(0);
	}
	g_nl_ctx.chan_state = NETLINK_MSG_LOOP_EXIT;
	rsmc_netlink_exit();
}

