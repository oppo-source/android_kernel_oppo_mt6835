#include "rsmc_spi_x801.h"
/*
#include <securec.h>
#include <cetc_platform/log/cetc_log.h>
*/
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/hwspinlock.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <uapi/linux/gpio.h>
#include <linux/regulator/consumer.h>
#include "../module_type.h"
#include "rsmc_msg_loop_x801.h"
#include "rsmc_device_x801.h"

#ifdef CETCLOG_TAG
#undef CETCLOG_TAG
#endif
#define CETCLOG_TAG RSMC_SPI

#define RSMC_DEVICE_NAME "rsmc"
#define RSMC_MISC_DEVICE_NAME "rsmc"
#define DELAY_100_MS 100
#define SPI_READ_RETRY_TIME 5

static struct smc_core_data *g_smc_core = NULL;

struct smc_core_data *smc_get_core_data_x801(void)
{
	return g_smc_core;
}

int memcpy_s(void *dst, int len_d, void *src, int len_t) {
	if (len_t > len_d) {
		return RSMC_ERROR;
	}
	memcpy(dst, src, len_t);
	return EOK;
}

int memset_s(void *src, int len_d, int n, int len_s)
{
	if (len_s > len_d) {
		return RSMC_ERROR;
	}
	memset(src, n, len_s);
	return EOK;
}

static int smc_setup_tcxo_pwr(struct smc_core_data *cd)
{
	struct rsmc_power_supply *power = NULL;
	int ret;

	if (cd == NULL)
		return 0;
	power = &cd->rsmc_powers;
	if (power->type == RSMC_POWER_UNUSED)
		return 0;
	if (power->use_count) {
		power->use_count++;
		return 0;
	}
	switch (power->type) {
	case RSMC_POWER_LDO:
		power->regulator = regulator_get(&cd->sdev->dev, "rsmc-tcxo-vcc");
		if (IS_ERR_OR_NULL(power->regulator)) {
			cetclog_err("%s: fail to get %s\n", __func__, "rsmc-tcxo-vcc");
			return -EINVAL;
		}
		ret = regulator_set_voltage(power->regulator, power->ldo_value,
			power->ldo_value);
		if (ret) {
			regulator_put(power->regulator);
			cetclog_err("%s: fail to set %s\n", __func__, "rsmc-tcxo-vcc");
			return ret;
		}
		cetclog_info("%s: tcxo-vcc setup succ!", __func__);
		break;
	case RSMC_POWER_GPIO:
		ret = gpio_request(power->gpio, "rsmc-tcxo-vcc-gpio");
		if (ret) {
			cetclog_err("%s: request gpio %d for %s failed\n", __func__, power->gpio, "rsmc-tcxo-vcc-gpio");
			return ret;
		}
		ret = gpio_get_value(power->gpio);
		cetclog_info("%s: tcxo_pwr_gpio %d value %d", __func__, power->gpio, ret);

		ret = gpio_direction_output(power->gpio, GPIO_LOW);
		if (ret) {
			cetclog_err("%s: tcxo_pwr_gpio %d output high err %d", __func__, power->gpio, ret);
			return ret;
		}

		ret = gpio_get_value(power->gpio);
		cetclog_info("%s: tcxo_pwr_gpio %d value %d", __func__, power->gpio, ret);
		break;
	default:
		break;
	}
	power->use_count++;
	return 0;
}

static int smc_setup_soc_en_gpio(struct smc_core_data *cd)
{
	int rc;

	if (cd == NULL)
		return -EINVAL;
	if (gpio_is_valid(cd->gpios.soc_en_gpio)) {
		rc = gpio_request(cd->gpios.soc_en_gpio, "rsmc_soc_en");
		if (rc) {
			cetclog_err("%s: soc_en_gpio %d request failed %d\n",
				__func__, cd->gpios.soc_en_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.soc_en_gpio);
		cetclog_info("%s: soc_en_gpio %d value %d",
			__func__, cd->gpios.soc_en_gpio, rc);

		rc = gpio_direction_output(cd->gpios.soc_en_gpio, GPIO_LOW);
		if (rc) {
			cetclog_err("%s: soc_en_gpio gpio %d output high err %d",
				__func__, cd->gpios.soc_en_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.soc_en_gpio);
		cetclog_info("%s: soc_en_gpio %d value %d",
			__func__, cd->gpios.soc_en_gpio, rc);
	}
	return 0;
}

static int smc_setup_mcu_reset_gpio(struct smc_core_data *cd)
{
	int rc;

	if (cd == NULL)
		return -EINVAL;
	if (gpio_is_valid(cd->gpios.mcu_reset_gpio)) {
		rc = gpio_request(cd->gpios.mcu_reset_gpio, "rsmc_mcu_reset");
		if (rc) {
			cetclog_err("%s: mcu_reset_gpio %d failed %d\n",
				__func__, cd->gpios.mcu_reset_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.mcu_reset_gpio);
		cetclog_info("%s: mcu_reset_gpio %d value %d",
			__func__, cd->gpios.mcu_reset_gpio, rc);

		rc = gpio_direction_output(cd->gpios.mcu_reset_gpio, GPIO_LOW);
		if (rc) {
			cetclog_err("%s: mcu_reset_gpio %d output high err %d",
				__func__, cd->gpios.mcu_reset_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.mcu_reset_gpio);
		cetclog_info("%s: mcu_reset_gpio %d value %d",
			__func__, cd->gpios.mcu_reset_gpio, rc);
	}
	return 0;
}

static int smc_setup_tx_ant_gpio(struct smc_core_data *cd)
{
	int rc;

	if (cd == NULL)
		return -EINVAL;
	if (gpio_is_valid(cd->gpios.tx_ant_gpio)) {
		rc = gpio_request(cd->gpios.tx_ant_gpio, "rsmc_tx_ant");
		if (rc) {
			cetclog_err("%s: tx_ant_gpio %d request failed %d\n",
				__func__, cd->gpios.tx_ant_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.tx_ant_gpio);
		cetclog_info("%s: tx_ant_gpio %d value %d",
			__func__, cd->gpios.tx_ant_gpio, rc);

		rc = gpio_direction_output(cd->gpios.tx_ant_gpio, GPIO_LOW);
		if (rc) {
			cetclog_err("%s: tx_ant_gpio %d output low err %d",
				__func__, cd->gpios.tx_ant_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.tx_ant_gpio);
		cetclog_info("%s: tx_ant_gpio %d value %d",
			__func__, cd->gpios.tx_ant_gpio, rc);
	}
	return 0;
}

static int smc_setup_rx_ant_gpio(struct smc_core_data *cd)
{
	int rc;

	if (cd == NULL)
		return -EINVAL;
	if (gpio_is_valid(cd->gpios.rx_ant_gpio)) {
		rc = gpio_request(cd->gpios.rx_ant_gpio, "rsmc_rx_ant");
		if (rc) {
			cetclog_err("%s: rx_ant_gpio %d request failed %d\n",
				__func__, cd->gpios.rx_ant_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.rx_ant_gpio);
		cetclog_info("%s: rx_ant_gpio %d value %d",
			__func__, cd->gpios.rx_ant_gpio, rc);

		rc = gpio_direction_output(cd->gpios.rx_ant_gpio, GPIO_HIGH);
		if (rc) {
			cetclog_err("%s: rx_ant_gpio %d output high err %d",
				__func__, cd->gpios.rx_ant_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.rx_ant_gpio);
		cetclog_info("%s: rx_ant_gpio %d value %d",
			__func__, cd->gpios.rx_ant_gpio, rc);
	}
	return 0;
}

static int smc_setup_vc_ant_gpio(struct smc_core_data *cd)
{
	int rc;

	if (cd == NULL)
		return -EINVAL;
	if (gpio_is_valid(cd->gpios.vc1_ant_gpio)) {
		rc = gpio_request(cd->gpios.vc1_ant_gpio, "rsmc_vc1_ant");
		if (rc) {
			cetclog_err("%s: vc1 %d reqfail %d", __func__, cd->gpios.vc1_ant_gpio, rc);
			return rc;
		}

		rc = gpio_direction_output(cd->gpios.vc1_ant_gpio, cd->gpios.vc1_default_value);
		if (rc) {
			cetclog_err("%s: vc1 %d output %d", __func__, cd->gpios.vc1_ant_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.vc1_ant_gpio);
		cetclog_info("%s: vc1_ant_gpio %d value %d", __func__, cd->gpios.vc1_ant_gpio, rc);
	}
	if (gpio_is_valid(cd->gpios.vc2_ant_gpio)) {
		rc = gpio_request(cd->gpios.vc2_ant_gpio, "rsmc_vc2_ant");
		if (rc) {
			cetclog_err("%s: vc2 %d reqfail %d", __func__, cd->gpios.vc2_ant_gpio, rc);
			return rc;
		}

		rc = gpio_direction_output(cd->gpios.vc2_ant_gpio, cd->gpios.vc2_default_value);
		if (rc) {
			cetclog_err("%s: vc2 %d output %d", __func__, cd->gpios.vc2_ant_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.vc2_ant_gpio);
		cetclog_info("%s: vc2_ant_gpio %d value %d", __func__, cd->gpios.vc2_ant_gpio, rc);
	}
	if (gpio_is_valid(cd->gpios.vc3_ant_gpio)) {
		rc = gpio_request(cd->gpios.vc3_ant_gpio, "rsmc_vc3_ant");
		if (rc) {
			cetclog_err("%s: vc3 %d reqfail %d", __func__, cd->gpios.vc3_ant_gpio, rc);
			return rc;
		}

		rc = gpio_direction_output(cd->gpios.vc3_ant_gpio, cd->gpios.vc3_default_value);
		if (rc) {
			cetclog_err("%s: vc3 %d output %d", __func__, cd->gpios.vc3_ant_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.vc3_ant_gpio);
		cetclog_info("%s: vc3_ant_gpio %d value %d", __func__, cd->gpios.vc3_ant_gpio, rc);
	}

	if (gpio_is_valid(cd->gpios.vc4_ant_gpio)) {
		rc = gpio_request(cd->gpios.vc4_ant_gpio, "rsmc_vc4_ant");
		if (rc) {
			cetclog_err("%s: vc4 %d reqfail %d", __func__, cd->gpios.vc4_ant_gpio, rc);
			return rc;
		}

		rc = gpio_direction_output(cd->gpios.vc4_ant_gpio, cd->gpios.vc4_default_value);
		if (rc) {
			cetclog_err("%s: vc4 %d output %d", __func__, cd->gpios.vc4_ant_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.vc4_ant_gpio);
		cetclog_info("%s: vc4_ant_gpio %d value %d", __func__, cd->gpios.vc4_ant_gpio, rc);
	}

	if (gpio_is_valid(cd->gpios.Llna_gpio)) {
		rc = gpio_request(cd->gpios.Llna_gpio, "Llna_gpio");
		if (rc) {
			cetclog_err("%s: Llna %d reqfail %d", __func__, cd->gpios.Llna_gpio, rc);
			return rc;
		}

		rc = gpio_direction_output(cd->gpios.Llna_gpio, cd->gpios.Llna_default_value);
		if (rc) {
			cetclog_err("%s: Llna %d output %d", __func__, cd->gpios.Llna_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.Llna_gpio);
		cetclog_info("%s: Llna_gpio %d value %d", __func__, cd->gpios.Llna_gpio, rc);
	}

	if (gpio_is_valid(cd->gpios.S2lna_gpio)) {
		rc = gpio_request(cd->gpios.S2lna_gpio, "S2lna_gpio");
		if (rc) {
			cetclog_err("%s: S2lna %d reqfail %d", __func__, cd->gpios.S2lna_gpio, rc);
			return rc;
		}

		rc = gpio_direction_output(cd->gpios.S2lna_gpio, cd->gpios.S2lna_default_value);
		if (rc) {
			cetclog_err("%s: S2lna %d output %d", __func__, cd->gpios.S2lna_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.S2lna_gpio);
		cetclog_info("%s: S2lna_gpio %d value %d", __func__, cd->gpios.S2lna_gpio, rc);
	}

	if (gpio_is_valid(cd->gpios.Lprotect_gpio)) {
		rc = gpio_request(cd->gpios.Lprotect_gpio, "Lprotect_gpio");
		if (rc) {
			cetclog_err("%s: Lprotect %d reqfail %d", __func__, cd->gpios.Lprotect_gpio, rc);
			return rc;
		}

		rc = gpio_direction_output(cd->gpios.Lprotect_gpio, cd->gpios.Lprotect_default_value);
		if (rc) {
			cetclog_err("%s: Lprotect %d output %d", __func__, cd->gpios.Lprotect_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.Lprotect_gpio);
		cetclog_info("%s: Lprotect_gpio %d value %d", __func__, cd->gpios.Lprotect_gpio, rc);
	}

	if (gpio_is_valid(cd->gpios.ant12_gpio)) {
		rc = gpio_request(cd->gpios.ant12_gpio, "ant12_gpio");
		if (rc) {
			cetclog_err("%s: ant12 %d reqfail %d", __func__, cd->gpios.ant12_gpio, rc);
			return rc;
		}

		rc = gpio_direction_output(cd->gpios.ant12_gpio, cd->gpios.ant12_default_value);
		if (rc) {
			cetclog_err("%s: ant12 %d output %d", __func__, cd->gpios.ant12_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.ant12_gpio);
		cetclog_info("%s: ant12_gpio %d value %d", __func__, cd->gpios.ant12_gpio, rc);
	}
	return 0;
}

static int smc_setup_pa_soc_pwr_gpio(struct smc_core_data *cd)
{
	int rc;

	if (cd == NULL)
		return -EINVAL;
	if (gpio_is_valid(cd->gpios.pa_pwr_gpio)) {
		rc = gpio_request(cd->gpios.pa_pwr_gpio, "rsmc_pa_pwr");
		if (rc) {
			cetclog_err("%s: pa_pwr_gpio %d request failed %d\n",
				__func__, cd->gpios.pa_pwr_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.pa_pwr_gpio);
		cetclog_info("%s: pa_soc_pwr_gpio %d value %d",
			__func__, cd->gpios.pa_pwr_gpio, rc);

		rc = gpio_direction_output(cd->gpios.pa_pwr_gpio, GPIO_LOW);
		if (rc) {
			cetclog_err("%s: pa_soc_pwr_gpio %d output high err %d",
				__func__, cd->gpios.pa_pwr_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.pa_pwr_gpio);
		cetclog_info("%s: pa_soc_pwr_gpio %d value %d",
			__func__, cd->gpios.pa_pwr_gpio, rc);
	}
	return 0;
}

static int smc_setup_irq_gpio(struct smc_core_data *cd)
{
	int rc;
	if (cd == NULL) {
		cetclog_err("%s: cd null", __func__);
		return -EINVAL;
	}
	if (gpio_is_valid(cd->gpios.irq_gpio)) {
		rc = gpio_request(cd->gpios.irq_gpio, "rsmc_irq");
		if (rc) {
			cetclog_err("%s: irq_gpio %d request failed %d\n",
				__func__, cd->gpios.irq_gpio, rc);
			return rc;
		}

		rc = gpio_get_value(cd->gpios.irq_gpio);
		cetclog_info("%s: irq_gpio %d value %d",
			__func__, cd->gpios.irq_gpio, rc);

		gpio_set_value(cd->gpios.irq_gpio, GPIO_HIGH);

		rc = gpio_direction_input(cd->gpios.irq_gpio);
		if (rc) {
			cetclog_err("%s: irq_gpio %d input err %d",
				__func__, cd->gpios.irq_gpio, rc);
			return rc;
		}
		rc = gpio_get_value(cd->gpios.irq_gpio);
		cetclog_info("%s: irq_gpio %d value %d",
			__func__, cd->gpios.irq_gpio, rc);
	}
	return 0;
}

static int smc_setup_gpio(struct smc_core_data *cd)
{
	int rc;

	if (cd == NULL)
		return -EINVAL;
	// tcxo pwr
	rc = smc_setup_tcxo_pwr(cd);
	// init gpio
	rc = smc_setup_soc_en_gpio(cd);
	// reset gpio
	rc = smc_setup_mcu_reset_gpio(cd);
	// ant tx gpio
	rc = smc_setup_tx_ant_gpio(cd);
	// rx ant gpio
	rc = smc_setup_rx_ant_gpio(cd);
	// vc ant gpio
	rc = smc_setup_vc_ant_gpio(cd);
	// PA PWR
	rc = smc_setup_pa_soc_pwr_gpio(cd);
	// IRQ GPIO
	rc = smc_setup_irq_gpio(cd);
	return 0;
}

static void smc_free_gpio(struct smc_core_data *cd)
{
	if (cd == NULL)
		return;
	if (gpio_is_valid(cd->gpios.soc_en_gpio))
		gpio_free(cd->gpios.soc_en_gpio);
	if (gpio_is_valid(cd->gpios.mcu_reset_gpio))
		gpio_free(cd->gpios.mcu_reset_gpio);
	if (gpio_is_valid(cd->gpios.pa_pwr_gpio))
		gpio_free(cd->gpios.pa_pwr_gpio);
	if (gpio_is_valid(cd->gpios.tx_ant_gpio))
		gpio_free(cd->gpios.tx_ant_gpio);
	if (gpio_is_valid(cd->gpios.rx_ant_gpio))
		gpio_free(cd->gpios.rx_ant_gpio);
	if (gpio_is_valid(cd->gpios.vc1_ant_gpio))
		gpio_free(cd->gpios.vc1_ant_gpio);
	if (gpio_is_valid(cd->gpios.vc2_ant_gpio))
		gpio_free(cd->gpios.vc2_ant_gpio);
	if (gpio_is_valid(cd->gpios.vc3_ant_gpio))
		gpio_free(cd->gpios.vc3_ant_gpio);
	if (gpio_is_valid(cd->gpios.irq_gpio))
		gpio_free(cd->gpios.irq_gpio);
	if (gpio_is_valid(cd->gpios.vc4_ant_gpio))
		gpio_free(cd->gpios.vc4_ant_gpio);
	if (gpio_is_valid(cd->gpios.Llna_gpio))
		gpio_free(cd->gpios.Llna_gpio);
	if (gpio_is_valid(cd->gpios.S2lna_gpio))
		gpio_free(cd->gpios.S2lna_gpio);
	if (gpio_is_valid(cd->gpios.Lprotect_gpio))
		gpio_free(cd->gpios.Lprotect_gpio);
	if (gpio_is_valid(cd->gpios.ant12_gpio))
		gpio_free(cd->gpios.ant12_gpio);
}

static void rsmc_tcxo_power_free(struct smc_core_data *cd)
{
	struct rsmc_power_supply *power = NULL;

	if (cd == NULL)
		return;
	power = &cd->rsmc_powers;
	if (power->type == RSMC_POWER_UNUSED)
		return;
	if ((--power->use_count) > 0)
		return;

	switch (power->type) {
	case RSMC_POWER_LDO:
		if (IS_ERR_OR_NULL(power->regulator)) {
			cetclog_err("%s:fail to get %s\n", __func__, "tcxo-vcc");
			return;
		}
		regulator_put(power->regulator);
		break;
	case RSMC_POWER_GPIO:
		gpio_direction_output(power->gpio, 0);
		gpio_free(power->gpio);
		break;
	default:
		cetclog_err("%s: invalid power type %d\n",
			__func__, power->type);
		break;
	}
}

static int smc_setup_spi(struct smc_core_data *cd)
{
	int rc;

	cetclog_info("%s: enter", __func__);
	if (cd == NULL)
		return -EINVAL;
	cd->sdev->bits_per_word = X801_MIN_BITS_PER_WORD;
	cd->sdev->mode = SPI_MODE_0;
	cd->sdev->max_speed_hz = X801_MIN_SPEED;

	rc = spi_setup(cd->sdev);
	if (rc) {
		cetclog_err("%s: spi setup fail\n", __func__);
		return rc;
	}
	cetclog_info("%s: succ", __func__);
	return 0;
}

static irqreturn_t rsmc_irq_thread(int irq, void *dev_id)
{
	struct smc_core_data *cd = (struct smc_core_data *)dev_id;
	disable_irq_nosync(cd->irq);
	cetclog_info("%s: irq:%d, spi status:%d", __func__, cd->irq, get_rsmc_spi_status());
	rsmc_spi_recv_proc();
	enable_irq(cd->irq);
	return IRQ_HANDLED;
}

void rsmc_spi_irq_ctl_x801(bool ctl)
{
	struct smc_core_data *cd = smc_get_core_data_x801();
	if (ctl)
		enable_irq(cd->irq);
	else
		disable_irq(cd->irq);
	cetclog_info("%s: irq:%d,ctl:%d", __func__, cd->irq, (u32)ctl);
}

int smc_setup_irq(struct smc_core_data *cd)
{
	int rc, irq;
	unsigned long irq_flag_type;

	if (cd == NULL) {
		cetclog_err("%s: smc_core_data is null\n", __func__);
		return -EINVAL;
	}

	irq = gpio_to_irq(cd->gpios.irq_gpio);
	
	irq_flag_type = IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND;
	rc = request_threaded_irq(irq, NULL, rsmc_irq_thread, irq_flag_type, "rsmc", cd);
	if (rc) {
		cetclog_err("%s: irq:%d,rc:%d", __func__, irq, rc);
		return rc;
	}
	cetclog_info("%s: irq:%d,rc:%d", __func__, irq, rc);
	enable_irq_wake(irq);
	disable_irq(irq);
	cd->irq = irq;

	return 0;
}

static int smc_core_init(struct smc_core_data *cd)
{
	int rc;
	dev_set_drvdata(&cd->sdev->dev, cd);
	rc = smc_setup_irq(cd);
	if (rc) {
		cetclog_err("%s:failed to setup irq", __func__);
		return 0;
	}
	atomic_set(&cd->register_flag, 1);
	return 0;
}

int spi_read_data_valid(const unsigned char *data, unsigned int length, int *i)
{
	for (*i = 0; *i < length; (*i)++) {
		if (data[*i] != 0)
			return 1;
	}
	return 0;
}

int rsmc_spi_read_x801(unsigned char *data, unsigned int length)
{
	int retval;
	struct smc_core_data *cd = smc_get_core_data_x801();
	mutex_lock(&cd->spi_mutex);
	retval = memset_s(&cd->t, sizeof(struct spi_transfer),
		0x00, sizeof(struct spi_transfer));
	if (retval != EOK)
		cetclog_err("%s: memset_s t fail", __func__);
	retval = memset_s(cd->tx_buf, sizeof(cd->tx_buf), 0, sizeof(cd->tx_buf));
	if (retval != EOK)
		cetclog_err("%s: memset_s tx_buf fail", __func__);
	spi_message_init(&cd->msg);
	cd->t.len = length;
	cd->t.tx_buf = cd->tx_buf;
	cd->t.rx_buf = data;

	cd->t.bits_per_word = X801_MAX_BITS_PER_WORD;
	spi_message_add_tail(&cd->t, &cd->msg);
	retval = spi_sync(cd->sdev, &cd->msg);
	if (retval == 0)
		retval = length;
	else
		cetclog_err("%s: spi_sync fail %d", __func__, retval);
	mutex_unlock(&cd->spi_mutex);
	cetclog_err("SPI read success\n");
	return retval;
}

int rsmc_spi_write_x801(unsigned char *data, unsigned int length)
{
	int ret;
	struct smc_core_data *cd = smc_get_core_data_x801();
	mutex_lock(&cd->spi_mutex);
	ret = memset_s(&cd->t, sizeof(struct spi_transfer),
		0x00, sizeof(struct spi_transfer));
	if (ret != EOK)
		cetclog_err("%s: memset_s fail", __func__);
	ret = memset_s(cd->tx_buf, sizeof(cd->tx_buf), 0, sizeof(cd->tx_buf));
	if (ret != EOK)
		cetclog_err("%s: tx_buf memset_s fail", __func__);
	ret = memcpy_s(cd->tx_buf, sizeof(cd->tx_buf), data, length);
	if (ret != EOK)
		cetclog_err("%s: tx_buf memcpy_s fail", __func__);

	spi_message_init(&cd->msg);
	cd->t.len = length;
	cd->t.tx_buf = cd->tx_buf;

	cd->t.bits_per_word = X801_MAX_BITS_PER_WORD;
	spi_message_add_tail(&cd->t, &cd->msg);
	ret = spi_sync(cd->sdev, &cd->msg);
	if (ret == 0)
		ret = length;
	else
		cetclog_err("%s: spi_sync error = %d\n", __func__, ret);
	mutex_unlock(&cd->spi_mutex);
	cetclog_err("SPI write success\n");
	return ret;
}

int rsmc_spi_reg_read(u32 addr, u32 *value)
{
	int rc, ret;
	struct smc_core_data *cd = smc_get_core_data_x801();

	if (value == NULL)
		return -EINVAL;
	mutex_lock(&cd->spi_mutex);

	cd->tx_buff[0] = 0x80 | addr;

	/* read data */
	ret = memset_s(&cd->t, sizeof(struct spi_transfer), 0, sizeof(struct spi_transfer));
	if (ret != EOK)
		cetclog_err("%s: memset_s fail", __func__);

	spi_message_init(&cd->msg);

	cd->t.rx_buf = cd->rx_buff;
	cd->t.tx_buf = cd->tx_buff;
	cd->t.len = sizeof(u32);

	cd->t.bits_per_word = X801_MIN_BITS_PER_WORD;
	spi_message_add_tail(&cd->t, &cd->msg);
	rc = spi_sync(cd->sdev, &cd->msg);
	if (rc) {
		cetclog_err("%s: spi_sync %d\n", __func__, rc);
		mdelay(SPI_MSLEEP_SHORT_TIME);
		rc = spi_sync(cd->sdev, &cd->msg);
		cetclog_err("%s: spi_sync %d\n", __func__, rc);
	}
	*value = (cd->rx_buff[0]<<24) |
		(cd->rx_buff[1]<<16) |
		(cd->rx_buff[2]<<8) |
		(cd->rx_buff[3]);
	mutex_unlock(&cd->spi_mutex);
	cetclog_err("SPI reg read success\n");
	cetclog_info("%s: addr:%d,value:0x%x", __func__, addr, *value);

	return rc;
}

int rsmc_spi_reg_write(u32 addr, u32 value)
{
	int rc, ret;
	struct smc_core_data *cd = smc_get_core_data_x801();

	mutex_lock(&cd->spi_mutex);

	/* set header */
	cd->tx_buff[0] = addr & 0x7F;
	cd->tx_buff[1] = (0xff & (value >> 16));
	cd->tx_buff[2] = (0xff & (value >> 8));
	cd->tx_buff[3] = (0xff & value);

	/* write data */
	ret = memset_s(&cd->t, sizeof(struct spi_transfer), 0, sizeof(struct spi_transfer));
	if (ret != EOK)
		cetclog_err("%s: memset_s fail", __func__);

	spi_message_init(&cd->msg);

	cd->t.tx_buf = cd->tx_buff;
	cd->t.rx_buf = cd->rx_buff;
	cd->t.len = sizeof(u32);

	cd->t.bits_per_word = X801_MIN_BITS_PER_WORD;
	spi_message_add_tail(&cd->t, &cd->msg);
	rc = spi_sync(cd->sdev, &cd->msg);
	if (rc) {
		cetclog_err("%s: spi_sync %d\n", __func__, rc);
		mdelay(SPI_MSLEEP_SHORT_TIME);
		rc = spi_sync(cd->sdev, &cd->msg);
		cetclog_err("%s: spi_sync %d\n", __func__, rc);
	}
	cetclog_err("SPI reg write success\n");
	mutex_unlock(&cd->spi_mutex);

	return rc;
}

int rsmc_spi_set_min_speed(void)
{
	int rc;
	struct smc_core_data *cd = smc_get_core_data_x801();
	if (cd == NULL)
		return -EINVAL;
	cd->sdev->bits_per_word = X801_MIN_BITS_PER_WORD;
	cd->sdev->mode = SPI_MODE_0;
	cd->sdev->max_speed_hz = X801_MIN_SPEED;
	rc = spi_setup(cd->sdev);
	if (rc) {
		cetclog_err("%s: spi setup fail\n", __func__);
		return rc;
	}
	cetclog_info("%s: set speed %d succ", __func__, cd->sdev->max_speed_hz);
	return 0;
}

int rsmc_spi_set_max_speed(void)
{
	int rc;
	struct smc_core_data *cd = smc_get_core_data_x801();
	if (cd == NULL)
		return -EINVAL;
	cd->sdev->bits_per_word = X801_MAX_BITS_PER_WORD;
	cd->sdev->mode = SPI_MODE_0;
	cd->sdev->max_speed_hz = X801_MAX_SPEED;
	rc = spi_setup(cd->sdev);
	if (rc) {
		cetclog_err("%s: spi setup fail\n", __func__);
		return rc;
	}
	cetclog_info("%s: set speed %d succ", __func__, cd->sdev->max_speed_hz);
	return 0;
}

int rsmc_register_dev_x801(struct smc_device *dev)
{
	int rc = -EINVAL;
	struct smc_core_data *cd = smc_get_core_data_x801();

	cetclog_info("%s: enter", __func__);

	if (dev == NULL || cd == NULL) {
		cetclog_err("%s: input null", __func__);
		goto register_err;
	}
	/* check device configed ot not */
	if (atomic_read(&cd->register_flag)) {
		cetclog_err("%s: smc have registerd", __func__);
		goto register_err;
	}
	dev->smc_core = cd;
	dev->gpios = &cd->gpios;
	dev->sdev = cd->sdev;
	cd->smc_dev = dev;

	rc = smc_setup_gpio(cd);
	if (rc) {
		cetclog_err("%s: spi dev init fail", __func__);
		goto dev_init_err;
	}
	rc = smc_setup_spi(cd);
	if (rc) {
		cetclog_err("%s: spi dev init fail", __func__);
		goto err;
	}

	rc = smc_core_init(cd);
	if (rc) {
		cetclog_err("%s: core init", __func__);
		goto err;
	}
	return 0;
err:
	smc_free_gpio(cd);
	rsmc_tcxo_power_free(cd);
dev_init_err:
	cd->smc_dev = 0;
register_err:
	return rc;
}

int rsmc_unregister_dev_x801(struct smc_device *dev)
{
	struct smc_core_data *cd = smc_get_core_data_x801();

	smc_free_gpio(cd);
	rsmc_tcxo_power_free(cd);
	return 0;
}

static int smc_parse_tcxo_power_config(struct device_node *smc_node,
	struct smc_core_data *cd)
{
	struct rsmc_power_supply *power = NULL;
	int rc;
	u32 value;

	power = &cd->rsmc_powers;
	rc = of_property_read_u32(smc_node, "rsmc-tcxo-vcc-type", &power->type);
	if (rc || power->type == RSMC_POWER_UNUSED) {
		cetclog_info("%s: power type not config or 0, unused\n", __func__);
		return 0;
	}
	switch (power->type) {
	case RSMC_POWER_GPIO:
		power->gpio = of_get_named_gpio(smc_node, "tcxo_pwr_gpio", 0);
		cetclog_info("rsmc-vcc-gpio = %d\n", power->gpio);
		break;
	case RSMC_POWER_LDO:
		rc = of_property_read_u32(smc_node, "rsmc-tcxo-vcc-value",
				&power->ldo_value);
		cetclog_info("rsmc-tcxo-vcc-value = %d\n", power->ldo_value);
		break;
	default:
		cetclog_info("%s: invaild power type %d", __func__, power->type);
		break;
	}
	rc = of_property_read_u32(smc_node, "tcxo_pwr_after_delay_ms", &value);
	if (!rc) {
		power->tcxo_pwr_after_delay_ms = (u32)value;
		cetclog_info("%s: tcxo_pwr_after_delay_ms %d\n", __func__, value);
	}
	return 0;
}

int smc_parse_gpio_config_x801(struct device_node *smc_node,
	struct smc_core_data *cd)
{
	u32 value;
	if ((smc_node == NULL) || (cd == NULL)) {
		cetclog_info("%s: input null\n", __func__);
		return -ENODEV;
	}

	value = of_get_named_gpio(smc_node, "en_gpio", 0);
	cetclog_info("soc_en_gpio = %d", value);
	cd->gpios.soc_en_gpio = value;

	value = of_get_named_gpio(smc_node, "mcu_rst_gpio", 0);
	cetclog_info("rst_gpio = %d", value);
	cd->gpios.mcu_reset_gpio = value;

	value = of_get_named_gpio(smc_node, "pa_gpio", 0);
	cetclog_info("pa_gpio = %d", value);
	cd->gpios.pa_pwr_gpio = value;

	value = of_get_named_gpio(smc_node, "rx_ant_gpio", 0);
	cetclog_info("rx_ant_gpio = %d", value);
	cd->gpios.rx_ant_gpio = value;

	value = of_get_named_gpio(smc_node, "tx_ant_gpio", 0);
	cetclog_info("tx_ant_gpio = %d", value);
	cd->gpios.tx_ant_gpio = value;

	value = of_get_named_gpio(smc_node, "irq_gpio", 0);
	cetclog_info("irq_gpio = %d", value);
	cd->gpios.irq_gpio = value;

	value = of_get_named_gpio_flags(smc_node, "rsmc_vc1_ant_gpio", 0,
		&cd->gpios.vc1_default_value);
	cetclog_info("rsmc_vc1_ant_gpio = %d, default_value = %d",
		value, cd->gpios.vc1_default_value);
	cd->gpios.vc1_ant_gpio = value;

	value = of_get_named_gpio_flags(smc_node, "rsmc_vc2_ant_gpio", 0,
		&cd->gpios.vc2_default_value);
	cetclog_info("rsmc_vc2_ant_gpio = %d, default_value = %d",
		value, cd->gpios.vc2_default_value);
	cd->gpios.vc2_ant_gpio = value;

	value = of_get_named_gpio_flags(smc_node, "rsmc_vc3_ant_gpio", 0,
		&cd->gpios.vc3_default_value);
	cetclog_info("rsmc_vc3_ant_gpio = %d, default_value = %d",
		value, cd->gpios.vc3_default_value);
	cd->gpios.vc3_ant_gpio = value;

	value = of_get_named_gpio_flags(smc_node, "rsmc_vc4_ant_gpio", 0,
		&cd->gpios.vc4_default_value);
	cetclog_info("rsmc_vc4_ant_gpio = %d, default_value = %d",
		value, cd->gpios.vc4_default_value);
	cd->gpios.vc4_ant_gpio = value;

	value = of_get_named_gpio_flags(smc_node, "Llna_gpio", 0,
		&cd->gpios.Llna_default_value);
	cetclog_info("Llna_gpio = %d, default_value = %d",
		value, cd->gpios.Llna_default_value);
	cd->gpios.Llna_gpio = value;

	value = of_get_named_gpio_flags(smc_node, "S2lna_gpio", 0,
		&cd->gpios.S2lna_default_value);
	cetclog_info("S2lna_gpio = %d, default_value = %d",
		value, cd->gpios.S2lna_default_value);
	cd->gpios.S2lna_gpio = value;

	value = of_get_named_gpio_flags(smc_node, "Lprotect_gpio", 0,
		&cd->gpios.Lprotect_default_value);
	cetclog_info("Lprotect_gpio = %d, default_value = %d",
		value, cd->gpios.Lprotect_default_value);
	cd->gpios.Lprotect_gpio = value;

	value = of_get_named_gpio_flags(smc_node, "ant12_gpio", 0,
		&cd->gpios.ant12_default_value);
	cetclog_info("ant12_gpio = %d, default_value = %d",
		value, cd->gpios.ant12_default_value);
	cd->gpios.ant12_gpio = value;
	return 0;
}

int smc_parse_spi_config_x801(struct device_node *smc_node,
	struct smc_core_data *cd)
{
	int rc;
	u32 value;
	struct smc_spi_config *spi_config = NULL;

	if (smc_node == NULL || cd == NULL) {
		cetclog_info("%s: input null\n", __func__);
		return -ENODEV;
	}

	spi_config = &cd->spi_config;

	rc = of_property_read_u32(smc_node, "spi-max-frequency", &value);
	if (!rc) {
		spi_config->max_speed_hz = value;
		cetclog_info("%s: spi-max-frequency configured %d\n",
				__func__, value);
	}
	rc = of_property_read_u32(smc_node, "spi-bus-id", &value);
	if (!rc) {
		spi_config->bus_id = (u8)value;
		cetclog_info("%s: spi-bus-id configured %d\n", __func__, value);
	}
	rc = of_property_read_u32(smc_node, "spi-mode", &value);
	if (!rc) {
		spi_config->mode = value;
		cetclog_info("%s: spi-mode configured %d\n", __func__, value);
	}
	rc = of_property_read_u32(smc_node, "bits-per-word", &value);
	if (!rc) {
		spi_config->bits_per_word = value;
		cetclog_info("%s: bits-per-word configured %d\n", __func__, value);
	}

	if (!cd->spi_config.max_speed_hz)
		cd->spi_config.max_speed_hz = SMC_SPI_SPEED_DEFAULT;
	if (!cd->spi_config.mode)
		cd->spi_config.mode = SPI_MODE_0;
	if (!cd->spi_config.bits_per_word)
		cd->spi_config.bits_per_word = SMC_SPI_DEFAULT_BITS_PER_WORD;

	cd->sdev->mode = spi_config->mode;
	cd->sdev->max_speed_hz = spi_config->max_speed_hz;
	cd->sdev->bits_per_word = spi_config->bits_per_word;
	return 0;
}

int smc_parse_config_x801(struct device_node *smc_node,
	struct smc_core_data *cd)
{
	int ret;

	if ((smc_node == NULL) || (cd == NULL)) {
		cetclog_info("%s: input null\n", __func__);
		return -ENODEV;
	}

	ret = smc_parse_tcxo_power_config(smc_node, cd);
	if (ret != 0)
		return ret;
	ret = smc_parse_gpio_config_x801(smc_node, cd);
	if (ret != 0)
		return ret;
	ret = smc_parse_spi_config_x801(smc_node, cd);
	if (ret != 0)
		return ret;

	cd->smc_node = smc_node;

	return 0;
}

static int oplus_rsmc_pin_init(struct smc_core_data *cd) {
	int ret = 0;
	cd->pinctrl = devm_pinctrl_get(cd->sdev->controller->dev.parent);
	if(IS_ERR(cd->pinctrl))
	{
		cetclog_err("rsmc pinctrl get error\n");
		ret = PTR_ERR(cd->pinctrl);
		return ret;
	}

	cd->pin_states[SPI_DEFAULT] = pinctrl_lookup_state(cd->pinctrl, SPI_DEFAULT_STATE);
	if(IS_ERR(cd->pin_states[SPI_DEFAULT]))
	{
		cetclog_err("get the state of spi default is fail\n");
		ret = PTR_ERR(cd->pin_states[SPI_DEFAULT]);
		return ret;
	}

	cd->pin_states[SPI_ACTIVE] = pinctrl_lookup_state(cd->pinctrl, SPI_ACTIVE_STATE);
	if(IS_ERR(cd->pin_states[SPI_ACTIVE]))
	{
		cetclog_err("get the state of spi active is fail\n");
		ret = PTR_ERR(cd->pin_states[SPI_ACTIVE]);
		return ret;
	}
	pinctrl_select_state(cd->pinctrl, cd->pin_states[SPI_DEFAULT]);
	cetclog_info("Change spi state to default\n");
	return ret;
}

static int smc_probe(struct spi_device *sdev)
{
	struct smc_core_data *cd = NULL;
	int rc;

	cetclog_info("%s: in\n", __func__);

	cd = kzalloc(sizeof(struct smc_core_data), GFP_KERNEL);
	if (cd == NULL)
		return -ENOMEM;
	cd->sdev = sdev;
	rc = smc_parse_config_x801(sdev->dev.of_node, cd);
	if (rc) {
		cetclog_err("%s: parse dts fail\n", __func__);
		kfree(cd);
		return -ENODEV;
	}
	mutex_init(&cd->spi_mutex);
	atomic_set(&cd->register_flag, 0);
	spi_set_drvdata(sdev, cd);
	rc = memset_s(cd->tx_buf, sizeof(cd->tx_buf), 0x00, sizeof(cd->tx_buf));
	if (rc != EOK)
		return -ENODEV;
	if (oplus_rsmc_pin_init(cd)){
		cetclog_err("rsmc pinctrl set error\n");
	}
	g_smc_core = cd;
	cetclog_info("%s: out\n", __func__);
	return 0;
}

static int smc_remove(struct spi_device *sdev)
{
	struct smc_core_data *cd = spi_get_drvdata(sdev);

	cetclog_info("%s: in\n", __func__);
	mutex_destroy(&cd->spi_mutex);
	kfree(cd);
	cd = NULL;
	return 0;
}

const struct of_device_id g_rsmc_psoc_match_table_x801[] = {
	{.compatible = "cetc,rsmc"},
	{},
};
EXPORT_SYMBOL_GPL(g_rsmc_psoc_match_table_x801);

static const struct spi_device_id g_rsmc_device_id_x801[] = {
	{RSMC_DEVICE_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(spi, g_rsmc_device_id_x801);

static struct spi_driver g_rsmc_spi_driver = {
	.probe = smc_probe,
	.remove = smc_remove,
	.id_table = g_rsmc_device_id_x801,
	.driver = {
		.name = RSMC_DEVICE_NAME,
		.owner = THIS_MODULE,
		.bus = &spi_bus_type,
		.of_match_table = of_match_ptr(g_rsmc_psoc_match_table_x801),
	},
};

int rsmc_spi_init_x801(void)
{
	int ret = spi_register_driver(&g_rsmc_spi_driver);

	cetclog_info("%s: call spi_register_driver ret %d", __func__, ret);
	return ret;
}

void rsmc_spi_exit_x801(void)
{
	spi_unregister_driver(&g_rsmc_spi_driver);
}

