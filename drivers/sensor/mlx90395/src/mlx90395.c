/*
 * Copyright (c) 2022 Stefano Cottafavi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h> // For Zephyr types and stdint.h

#include <zephyr/kernel.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MLX90395, CONFIG_SENSOR_LOG_LEVEL);

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#ifdef CONFIG_I2C
#include <zephyr/drivers/i2c.h>
#endif
#ifdef CONFIG_SPI
#include <zephyr/drivers/spi.h>
#endif

#include <mlx.h>
#include <mlx90395.h>

#define DT_DRV_COMPAT melexis_mlx90395


static inline uint8_t sensor_to_zyxt(enum sensor_channel chan)
{
	uint8_t zyxt = MLX90395_NOTHING;

	switch (chan) {
	case SENSOR_CHAN_MAGN_X:
		zyxt = MLX90395_X;
		break;

	case SENSOR_CHAN_MAGN_Y:
		zyxt = MLX90395_Y;
		break;

	case SENSOR_CHAN_MAGN_Z:
		zyxt = MLX90395_Z;
		break;

	case SENSOR_CHAN_MAGN_XYZ:
		zyxt = MLX90395_X | MLX90395_Y | MLX90395_Z;
		break;

	case SENSOR_CHAN_DIE_TEMP:
		zyxt = MLX90395_T;
		break;

	case SENSOR_CHAN_ALL:
		zyxt = MLX90395_X | MLX90395_Y | MLX90395_Z | MLX90395_T;
		break;

	default:
	{
		zyxt = MLX90395_NOTHING;
		break;
	}
	}

	return zyxt;
}

// TODO: implement the tables from DS pag 32, depending on RES, GAIN_SEL, etc..

static void mlx90395_convert_xy(struct sensor_value *val, uint16_t sample)
{

	int32_t conv_val;

	conv_val = sample / 140 * 1000;
	val->val1 = conv_val / 1000;
	val->val2 = conv_val % 1000;
}
static void mlx90395_convert_z(struct sensor_value *val, uint16_t sample)
{

	int32_t conv_val;

	conv_val = sample / 140 * 1000;
	val->val1 = conv_val / 1000;
	val->val2 = conv_val % 1000;
}
static void mlx90395_convert_t(struct sensor_value *val, uint16_t sample)
{

	// MLX90395 datasheet pag 15
	int32_t conv_val = sample * (1000000 / 50);

	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

void mlx90395_decode(const struct device *dev, uint8_t zyxt)
{
	struct mlx_data *data = dev->data;

	// ignore 1st, status always 2nd byte
	uint8_t *p = (uint8_t *)&data->data_buffer[1];
	data->status = *p;
	p += 1;

	// zyxt==0 -> Status-CRC-X-Y-Z-T-V
	if(zyxt==0) {
		data->crc = *p;
		p += 1;
		if ((zyxt & MLX90395_X) != 0) {
			data->x = sys_get_be16(p);
			p += 2;
		}
		if ((zyxt & MLX90395_Y) != 0) {
			data->y = sys_get_be16(p);
			p += 2;
		}
		if ((zyxt & MLX90395_Z) != 0) {
			data->z = sys_get_be16(p);
			p += 2;
		}
		if ((zyxt & MLX90395_T) != 0) {
			data->t = sys_get_be16(p);
			p += 2;
		}
		data->v = sys_get_be16(p);

	// zyxt!=0 -> Status-T-X-Y-Z-CRC
	} else {
		if ((zyxt & MLX90395_T) != 0) {
			data->t = sys_get_be16(p);
			p += 2;
		}
		if ((zyxt & MLX90395_X) != 0) {
			data->x = sys_get_be16(p);
			p += 2;
		}
		if ((zyxt & MLX90395_Y) != 0) {
			data->y = sys_get_be16(p);
			p += 2;
		}
		if ((zyxt & MLX90395_Z) != 0) {
			data->z = sys_get_be16(p);
			p += 2;
		}
		data->crc = *p;
	}

}

static int mlx90395_sample_fetch(const struct device *dev, enum sensor_channel chan)
{

	struct mlx_data *data = dev->data;

	uint8_t zyxt = sensor_to_zyxt(chan);

	// start measurement if not in BURST mode
	// TODO: check the delay is ok
	/*if(data->mode != MLX_BURST_MODE) {
		mlx_cmd_SM(dev, zyxt);
		k_usleep(20);
	}*/

	mlx_cmd_RM(dev, zyxt);

	// extract values from rx buffer
	mlx90395_decode(dev, zyxt);

	return 0;
}

static int mlx90395_channel_get(const struct device *dev, enum sensor_channel chan,
								struct sensor_value *val)
{

	struct mlx_data *data = dev->data;

	// TODO: add support for single channels..
	// be super careful that the fetch
	// might happen on partial channels too...
	// and things get then out of alignment here

	if (chan == SENSOR_CHAN_ALL)
	{
		mlx90395_convert_xy(val++, data->x);
		mlx90395_convert_xy(val++, data->y);
		mlx90395_convert_z(val++, data->z);
		mlx90395_convert_t(val, data->t);
	}
	else if (chan == SENSOR_CHAN_MAGN_XYZ)
	{
		mlx90395_convert_xy(val++, data->x);
		mlx90395_convert_xy(val++, data->y);
		mlx90395_convert_z(val++, data->z);
	}
	else if (chan == SENSOR_CHAN_DIE_TEMP)
	{
		mlx90395_convert_t(val, data->t);
		LOG_DBG("TEMP: %d", data->t);
		LOG_DBG("STATUS: 0x%02X", data->status);
	}
	else
	{
		LOG_DBG("Only SENSOR_CHAN_ALL and XYZ spoorted at the moment.");
	}
	/*} else if (chan == SENSOR_CHAN_MAGN_X) {
		MLX90395_convert(val, MLX90395_x);
	} else if (chan == SENSOR_CHAN_MAGN_Y) {
		MLX90395_convert(val, MLX90395_y);
	} else if (chan == SENSOR_CHAN_MAGN_Z) {
		MLX90395_convert(val, MLX90395_z);
	}*/

	return 0;
}

int mlx_attr_get(const struct device *dev, enum sensor_channel chan,
				 enum sensor_attribute attr, struct sensor_value *val)
{
	// TODO: Main shit
	LOG_ERR("MLX90395_attr_get: Not implemented yet!: CHAN: %u, ATTR: %u", chan, attr);
	return 0;
}

int mlx_attr_set(const struct device *dev, enum sensor_channel chan,
				 enum sensor_attribute attr, const struct sensor_value *val)
{
	// TODO: Main shit
	LOG_ERR("MLX90395_attr_set: Not implemented yet!");
	return 0;
}

static const struct sensor_driver_api mlx90395_driver_api = {
	.sample_fetch = mlx90395_sample_fetch,
	.channel_get = mlx90395_channel_get,
	.attr_set = mlx_attr_set,
	.attr_get = mlx_attr_get
	//.trigger_set = MLX90395_trigger_set,
};

int mlx_dump(const struct device *dev) {

	uint16_t val;

	LOG_DBG("Dump MLX90395 registers:");

	for(int i=0; i<=0x01; i++) {
		mlx_cmd_RR(dev, i, &val);
		LOG_DBG("REG: 0x%02X --- VALUE: 0x%04X", i, val);
	}
	/*for(int i=0x26; i<=0x28; i++) {
		mlx_cmd_RR(dev, i, &val);
		//LOG_DBG("REG: 0x%02X --- VALUE: 0x%04X", i, val);
	}*/

	return 0;
}

int mlx90395_init(const struct device *dev)
{

	struct mlx_data *data = dev->data;
	const struct mlx_cfg *cfg = dev->config;

	if (!spi_is_ready_dt(&cfg->spi))
	{
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

	uint8_t zyxt = MLX90395_X | MLX90395_Y | MLX90395_Z | MLX90395_T;

	// exit any prev mode
	mlx_cmd_EX(dev);
	k_msleep(5);
	// warm reset
	mlx_cmd_RT(dev);
	k_msleep(50);
	// start burst mode
	mlx_cmd_SB(dev, zyxt);
	k_msleep(20);

	/*uint16_t val;
	mlx_cmd_RR(dev, 0x01, &val);
	LOG_DBG("VALUE: 0x%04X", val);

	val &= (~ MLX90395_REG_GAIN_SEL_BITS);
	mlx_cmd_WR(dev, 0x01, val);
	LOG_DBG("VALUE: 0x%04X", val);
	*/

	// TODO: add proper check on status byte
	data->mode = MLX_BURST_MODE;

	LOG_INF("Succesfully initialized MLX90395");

	mlx_dump(dev);

	return 0;
}

#define MLX90395_DEFINE(inst)                                                                                                           \
	static struct mlx_data mlx_data_##inst;                                                                                             \
                                                                                                                                        \
	static const struct mlx_cfg mlx_cfg_##inst = {                                                                                      \
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),                                                                                          \
					(.spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8), 0), ), ()) \
			COND_CODE_1(DT_INST_ON_BUS(inst, i2c),                                                                                      \
						(.i2c = I2C_DT_SPEC_INST_GET(inst), ), ())};                                                                    \
                                                                                                                                        \
	DEVICE_DT_INST_DEFINE(                                                                                                              \
		inst,                                                                                                                           \
		mlx90395_init,                                                                                                                  \
		NULL,                                                                                                                           \
		&mlx_data_##inst,                                                                                                               \
		&mlx_cfg_##inst,                                                                                                                \
		POST_KERNEL,                                                                                                                    \
		CONFIG_SENSOR_INIT_PRIORITY,                                                                                                    \
		&mlx90395_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MLX90395_DEFINE)