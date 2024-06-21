// SPDX-License-Identifier: GPL-2.0+
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/of_gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>

struct wl28681c_platform_data {
	struct device *dev;
	struct regmap *regmap;
};

enum wl28681c_regulator_ids {
	WL28681C_LDO1,
	WL28681C_LDO2,
	WL28681C_LDO3,
	WL28681C_LDO4,
	WL28681C_LDO5,
	WL28681C_LDO6,
	WL28681C_LDO7,
};

enum wl28681c_registers {
	WL28681C_PRODUCT_ID = 0x00,
	WL28681C_SILICON_REV,
	WL28681C_IOUT,
	WL28681C_ENABLE = 0x03,
	WL28681C_LDO1VOUT,
	WL28681C_LDO2VOUT,
	WL28681C_LDO3VOUT,
	WL28681C_LDO4VOUT,
	WL28681C_LDO5VOUT,
	WL28681C_LDO6VOUT,
	WL28681C_LDO7VOUT,

	WL28681C_REG_MAX = 0x1E,
};

#define WL28681C_ID	 (0x0D)

static const struct regulator_ops wl28681c_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.map_voltage = regulator_map_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

// VOUT = 0.496 + (d - 61) *0.008V
#define WL28681C_NLDO(_num, _supply, _default)				\
	[WL28681C_LDO ## _num] = {					\
		.name =		   "ONLDO"#_num,			\
		.of_match =	   of_match_ptr("ONLDO"#_num),		\
		.regulators_node = of_match_ptr("regulators"),		\
		.type =		   REGULATOR_VOLTAGE,			\
		.owner =	   THIS_MODULE,				\
		.linear_ranges =   (struct linear_range[]) {		\
		      REGULATOR_LINEAR_RANGE(496000, 0x00, 0x3D, 0),	\
		      REGULATOR_LINEAR_RANGE(496000, 0x3D, 0xFF, 8000),	\
		},							\
		.n_linear_ranges = 2,					\
		.vsel_reg =	   WL28681C_LDO ## _num ## VOUT,	\
		.vsel_mask =	   0xff,				\
		.enable_reg =	   WL28681C_ENABLE,			\
		.enable_mask =	   BIT(_num - 1),			\
		.enable_time =	   150,					\
		.supply_name =	   _supply,				\
		.ops =		   &wl28681c_ops,			\
	}

// VOUT = 1.372V + d * 0.008V
#define WL28681C_PLDO(_num, _supply, _default)				\
	[WL28681C_LDO ## _num] = {					\
		.name =		   "ONLDO"#_num,				\
		.of_match =	   of_match_ptr("ONLDO"#_num),		\
		.regulators_node = of_match_ptr("regulators"),		\
		.type =		   REGULATOR_VOLTAGE,			\
		.owner =	   THIS_MODULE,				\
		.linear_ranges =   (struct linear_range[]) {		\
		      REGULATOR_LINEAR_RANGE(1372000, 0x0, 0xFF, 8000),	\
		},							\
		.n_linear_ranges = 1,					\
		.vsel_reg =	   WL28681C_LDO ## _num ## VOUT,	\
		.vsel_mask =	   0xff,				\
		.enable_reg =	   WL28681C_ENABLE,			\
		.enable_mask =	   BIT(_num - 1),			\
		.enable_time =	   150,					\
		.supply_name =	   _supply,				\
		.ops =		   &wl28681c_ops,			\
	}

static const struct regulator_desc wl28681c_regulators[] = {
	WL28681C_NLDO(1, "vin12",  496000),
	WL28681C_NLDO(2, "vin12",  496000),
	WL28681C_PLDO(3, "vin34", 1372000),
	WL28681C_PLDO(4, "vin34", 1372000),
	WL28681C_PLDO(5, "vin5",  1372000),
	WL28681C_PLDO(6, "vin6",  1372000),
	WL28681C_PLDO(7, "vin7",  1372000),
};

static const struct regmap_config wl28681c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = WL28681C_REG_MAX,
};

enum DBG_ARG_IDX {
	DBG_ARG_IDX_I2C_ADDR,
	DBG_ARG_IDX_I2C_DATA,
	DBG_ARG_IDX_MAX_NUM,
};

static u32 RegAddr = 0;
static u32 RegData = 0;
struct wl28681c_platform_data *g_pdata = NULL;

static ssize_t debug_i2c_ops_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	int len = 0;
//	int ret;

#define SHOW(buf, len, fmt, ...) { \
	len += snprintf(buf + len, PAGE_SIZE - len, fmt, ##__VA_ARGS__); \
}

//		ret = regmap_read(g_pdata->regmap, RegAddr, &RegData);

	SHOW(buf, len, "[test] i2c read 0x%08x = 0x%08x\n",
			RegAddr, RegData);

	pr_err("[test] DW9827C_read_FW");

	return len;
}

static ssize_t debug_i2c_ops_store(struct device *dev,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	char delim[] = " ";
	char *token = NULL;
	char *sbuf = kzalloc(sizeof(char) * (count + 1), GFP_KERNEL);
	char *s = sbuf;
	int ret;
	unsigned int num_para = 0;
	char *arg[DBG_ARG_IDX_MAX_NUM];
	u32 val;
	u32 reg;

	RegAddr = 0;
	RegData = 0;

	if (!sbuf)
		goto ERR_DEBUG_OPS_STORE;

	memcpy(sbuf, buf, count);

	token = strsep(&s, delim);
	while (token != NULL && num_para < DBG_ARG_IDX_MAX_NUM) {
		if (strlen(token)) {
			arg[num_para] = token;
			num_para++;
		}

		token = strsep(&s, delim);
	}

	if (num_para > DBG_ARG_IDX_MAX_NUM) {
		pr_err("Wrong command parameter number %u\n", num_para);
		goto ERR_DEBUG_OPS_STORE;
	}
	ret = kstrtouint(arg[DBG_ARG_IDX_I2C_ADDR], 0, &reg);
	if (ret)
		goto ERR_DEBUG_OPS_STORE;
	RegAddr = reg;

	if (num_para == DBG_ARG_IDX_MAX_NUM) {
		ret = kstrtouint(arg[DBG_ARG_IDX_I2C_DATA], 0, &val);
		if (ret)
			goto ERR_DEBUG_OPS_STORE;
		RegData = val;

		ret = regmap_write(g_pdata->regmap, RegAddr, RegData);

		pr_err("%s i2c write 0x%08x = 0x%08x ret = %d\n",
			__func__,
			RegAddr, RegData, ret);
	}

	ret = regmap_read(g_pdata->regmap, RegAddr, &RegData);

	pr_err("%s i2c read 0x%08x = 0x%08x\n",
			__func__,
			RegAddr, RegData);


ERR_DEBUG_OPS_STORE:

	kfree(sbuf);
	dev_dbg(dev, "exit %s\n", __func__);

	return count;
}

static DEVICE_ATTR_RW(debug_i2c_ops);



static int wl28681c_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct wl28681c_platform_data *pdata;
	struct regulator_config config = {};
	struct regulator_dev *rdev;
	//struct regmap *regmap;
	int i, ret;
	unsigned int data;
	struct pinctrl *pinctrl;
	struct pinctrl_state *reset_output_high;
	int access_time = 3;

	pr_info("wl28681c probe E");
	if (i2c->dev.of_node) {
		pdata = devm_kzalloc(&i2c->dev,
				sizeof(struct wl28681c_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&i2c->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
	} else {
		pdata = i2c->dev.platform_data;
	}

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "fail : i2c functionality check...\n");
		return -EOPNOTSUPP;
	}

	if (pdata == NULL) {
		dev_err(&i2c->dev, "fail : no platform data.\n");
		return -ENODATA;
	}

	pinctrl = devm_pinctrl_get(&i2c->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		dev_err(&i2c->dev, "Cannot find pinctrl!\n");
		//return ret;
	} else {
		reset_output_high = pinctrl_lookup_state(pinctrl, "reset_high");
		if (IS_ERR(reset_output_high)) {
			ret = PTR_ERR(reset_output_high);
			dev_err(&i2c->dev, "Cannot find pinctrl reset_output_high!\n");
		} else {
			pinctrl_select_state(pinctrl, reset_output_high);
		}
	}
	pdata->regmap = devm_regmap_init_i2c(i2c, &wl28681c_regmap);
	if (IS_ERR(pdata->regmap)) {
		ret = PTR_ERR(pdata->regmap);
		dev_err(&i2c->dev, "Failed to create regmap: %d\n", ret);
		return ret;
	}

	ret = regmap_read(pdata->regmap, WL28681C_PRODUCT_ID, &data);

	while(ret< 0 && --access_time) {
		mdelay(2);
		ret = regmap_read(pdata->regmap, WL28681C_PRODUCT_ID, &data);
	}
	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to read PRODUCT_ID: %d\n", ret);
		return ret;
	}
	if (data != WL28681C_ID) {
		dev_err(&i2c->dev, "Unsupported device id: 0x%x.\n", data);
		return -ENODEV;
	}
//		ret = regmap_read(pdata->regmap, 0x0C, &data);
//		dev_err(&i2c->dev, "read reg[0x0c] =0x%02x.\n", data);

	config.dev = &i2c->dev;
	config.regmap = pdata->regmap;
	config.init_data = NULL;

	for (i = 0; i < ARRAY_SIZE(wl28681c_regulators); i++) {
		rdev = devm_regulator_register(&i2c->dev,
					       &wl28681c_regulators[i],
					       &config);
		if (IS_ERR(rdev)) {
			ret = PTR_ERR(rdev);
			dev_err(&i2c->dev, "Failed to register %s: %d\n",
				wl28681c_regulators[i].name, ret);
			return ret;
		}

//		if (i < 2) {
//			config.ena_gpiod = gpiod_get_optional(&i2c->dev, "vin12_ena",
//				 GPIOD_OUT_LOW| GPIOD_FLAGS_BIT_NONEXCLUSIVE);
//			if (IS_ERR(config.ena_gpiod)) {
//				dev_err(&i2c->dev, "no vin12_ena config\n");
//			}
//		} else {
//			config.ena_gpiod = NULL;
//		}
		dev_err(&i2c->dev, "regulator register %s: %d\n",
			wl28681c_regulators[i].name, ret);
	}

	g_pdata = pdata;
	ret = device_create_file(&i2c->dev, &dev_attr_debug_i2c_ops);

	dev_err(&i2c->dev, "regulator probe end\n");
	pr_info("wl28681c probe X");
	return 0;
}

static const struct i2c_device_id wl28681c_i2c_id[] = {
	{ "wl28681c-pmic", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, wl28681c_i2c_id);

static struct i2c_driver wl28681c_regulator_driver = {
	.driver = {
		.name = "wl28681c-pmic",
		.owner = THIS_MODULE
	},
	.probe = wl28681c_i2c_probe,
	.id_table = wl28681c_i2c_id,
};

module_i2c_driver(wl28681c_regulator_driver);

MODULE_DESCRIPTION("WL28681C PMIC voltage regulator driver");
MODULE_AUTHOR("XXX");
MODULE_LICENSE("GPL");
