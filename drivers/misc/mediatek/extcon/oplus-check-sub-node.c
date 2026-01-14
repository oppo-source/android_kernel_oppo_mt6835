#include <linux/extcon-provider.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb/role.h>
#include <linux/workqueue.h>
#ifdef OPLUS_FEATURE_CHG_BASIC
#include <linux/proc_fs.h>
#endif

#include "oplus-check-sub-node.h"

#define OPLUS_SUB_NODE_NAME_LEN 16
static int oplus_sub_node_name_parse_cmdline_match(char *match_str, char *result, int size)
{
	struct device_node *cmdline_node = NULL;
	const char *cmdline;
	char *match, *match_end;
	int len, match_str_len, ret;

	if (!result || !match_str)
		return -EINVAL;

	memset(result, '\0', size);
	match_str_len = strlen(match_str);

	cmdline_node = of_find_node_by_path("/chosen");
	if (!cmdline_node) {
		pr_err("%s:line%d: NULL pointer!!!\n", __func__, __LINE__);
		return -EINVAL;
	}

	ret = of_property_read_string(cmdline_node, "bootargs", &cmdline);
	if (ret) {
		pr_err("%s failed to read bootargs\n", __func__);
		return -EINVAL;
	}

	match = strstr(cmdline, match_str);
	if (!match) {
		pr_err("match: %s fail in cmdline\n", match_str);
		return -EINVAL;
	}

	match_end = strstr((match + match_str_len), ";");
	if (!match_end) {
		pr_err("match end of : %s fail in cmdline\n", match_str);
		return -EINVAL;
	}

	len = match_end - (match + match_str_len);
	if (len < 0 || len > size) {
		pr_err("match cmdline :%s fail, len = %d\n", match_str, len);
		return -EINVAL;
	}

	memcpy(result, (match + match_str_len), len);

	return 0;
}

static int oplus_get_sub_node_name_str(char *match_str, char *type)
{
	char result[32] = {};
	int ret;

	ret = oplus_sub_node_name_parse_cmdline_match(match_str, result, sizeof(result));
	if (ret < 0) {
		pr_err("match charge ic str fail\n");
		return ret;
	}
	snprintf(type, OPLUS_SUB_NODE_NAME_LEN, "%s", result);

	return ret;
}

struct device_node *oplus_get_sub_node_by_cmdline(char *match_str, struct device_node *father_node)
{
	char sub_node_name_str[OPLUS_SUB_NODE_NAME_LEN] = { 0 };
	struct device_node *sub_node = NULL;
	struct device_node *node = father_node;

	int rc = oplus_get_sub_node_name_str(match_str, sub_node_name_str);
	if (rc == 0) {
		sub_node = of_get_child_by_name(father_node, sub_node_name_str);
		if (sub_node) {
			node = sub_node;
			pr_err("current sub node name str = %s\n", sub_node_name_str);
		}
	}

	return node;
}
EXPORT_SYMBOL(oplus_get_sub_node_by_cmdline);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Oplus check Usb Sub Node");