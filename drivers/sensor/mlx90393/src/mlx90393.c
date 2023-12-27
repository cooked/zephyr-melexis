/*
 * Copyright (c) 2022 Stefano Cottafavi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>		// For Zephyr types and stdint.h

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MLX90393, CONFIG_SENSOR_LOG_LEVEL);

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

//#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>

#include <mlx90393.h>

#define DT_DRV_COMPAT melexis_mlx90393

// TODO: replace with sys utils for byte handling
uint16_t assemble_16(uint8_t *p_data) {
	uint16_t result = p_data[0];
	result          = (result << 8) + p_data[1];
	return result;
}
uint32_t assemble_32(uint8_t *p_data) {
	int i;
	uint32_t result = p_data[0];
	for (i = 1; i < 4; i++) {
		result = (result << 8) + p_data[i];
	}
	return result;
}


static inline uint8_t get_zyxt_from_sensor_channel(enum sensor_channel chan) {
	uint8_t zyxt = MLX90393_NOTHING;

	switch (chan) {
		case SENSOR_CHAN_MAGN_X:
			zyxt = MLX90393_X;
			break;

		case SENSOR_CHAN_MAGN_Y:
			zyxt = MLX90393_Y;
			break;

		case SENSOR_CHAN_MAGN_Z:
			zyxt = MLX90393_Z;
			break;

		case SENSOR_CHAN_MAGN_XYZ:
			zyxt = MLX90393_X | MLX90393_Y | MLX90393_Z;
			break;

		case SENSOR_CHAN_DIE_TEMP:
			zyxt = MLX90393_T;
			break;

		case SENSOR_CHAN_ALL:
			zyxt = MLX90393_X | MLX90393_Y | MLX90393_Z | MLX90393_T;
			break;

		default: {
			zyxt = MLX90393_NOTHING;
			break;
		}
	}

	return zyxt;
}

// TODO: implement the tables from DS pag 32, depending on RES, GAIN_SEL, etc..

static void mlx90393_convert_xy(struct sensor_value *val, uint16_t sample) {

	int32_t conv_val;

	conv_val  = sample * 0.751f * 98/75 * 1000;
	val->val1 = conv_val / 1000;
	val->val2 = conv_val % 1000;

}

static void mlx90393_convert_z(struct sensor_value *val, uint16_t sample) {

	int32_t conv_val;

	conv_val  = sample * 1.21f * 98/75 *1000;
	val->val1 = conv_val / 1000;
	val->val2 = conv_val % 1000;

}

static void mlx90393_convert_t(struct sensor_value *val, uint16_t sample) {

	// see DS pag 12
	int32_t conv_val = (((sample - 46244 ) / 45.2) + 25) * 1000000;

	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}


void mlx90393_decode(const struct device *dev, uint8_t zyxt) {

	struct mlx90393_data *data = dev->data;

	uint8_t *p = (uint8_t *) data->mlx90393_data_buffer;

	data->mlx90393_status = *p;

	p += 1;

	if ((zyxt & MLX90393_T) != 0) {
		data->mlx90393_t = assemble_16(p);
		p += 2;
	}

	if ((zyxt & MLX90393_X) != 0) {
		data->mlx90393_x = assemble_16(p);
		p += 2;
	}

	if ((zyxt & MLX90393_Y) != 0) {
		data->mlx90393_y = assemble_16(p);
		p += 2;
	}

	if ((zyxt & MLX90393_Z) != 0) {
		data->mlx90393_z = assemble_16(p);
		p += 2;
	}
}


static int mlx90393_sample_fetch(const struct device *dev, enum sensor_channel chan) {

	// Double check to see that we have a valid channel to fetch and set zyxt
	uint8_t zyxt = get_zyxt_from_sensor_channel(chan);

	if (zyxt == MLX90393_NOTHING) {
		LOG_ERR("Unsupported sensor channel %d for MLX90393.", chan);
		return -ENOTSUP;
	}

	// TODO: if not bursting, kick off single measurement
	//if( data->current_mode != MLX90393_BURST_MODE) {
	//	mlx90393_command(MLX90393_CMD_SM, zyxt, 0, 0);
	//}

	int err = mlx90393_cmd_RM(dev, zyxt);

	return err;
}

static int mlx90393_channel_get(const struct device *dev, enum sensor_channel chan,
	struct sensor_value *val) {

	struct mlx90393_data *data = dev->data;

	// We can do an assert without an explicit message, as this function should
	// not even be called if the sample_fetch failed due to an invalid channel.
	__ASSERT_NO_MSG(
		chan == SENSOR_CHAN_MAGN_X ||
		chan == SENSOR_CHAN_MAGN_Y ||
		chan == SENSOR_CHAN_MAGN_Z ||
		chan == SENSOR_CHAN_MAGN_XYZ ||
		chan == SENSOR_CHAN_DIE_TEMP ||
		chan == SENSOR_CHAN_ALL);

	// Get zyxt from sensor channel configuration for RM command
	uint8_t zyxt = get_zyxt_from_sensor_channel(chan);

	// TODO: add support for single channels.. be super careful that the fetch
	// might happen on partial channels too... and things get then out of
	// alignment here

	// convert to sensor_value and store them as uint16
	mlx90393_decode(dev, zyxt);

	if( chan == SENSOR_CHAN_ALL ) {
		mlx90393_convert_xy(val++, data->mlx90393_x);
		mlx90393_convert_xy(val++, data->mlx90393_y);
		mlx90393_convert_z(val++, data->mlx90393_z);
		mlx90393_convert_t(val, data->mlx90393_t);
	} else if (chan == SENSOR_CHAN_MAGN_XYZ) {
		mlx90393_convert_xy(val++, data->mlx90393_x);
		mlx90393_convert_xy(val++, data->mlx90393_y);
		mlx90393_convert_z(val++, data->mlx90393_z);
	} else if (chan == SENSOR_CHAN_DIE_TEMP) {
		//mlx90393_convert_t(val, data->mlx90393_t);
		LOG_DBG("TEMP: %d", data->mlx90393_t);
	} else {
		LOG_DBG("Only SENSOR_CHAN_ALL and XYZ spoorted at the moment.");
	}
	/*} else if (chan == SENSOR_CHAN_MAGN_X) {
		mlx90393_convert(val, mlx90393_x);
	} else if (chan == SENSOR_CHAN_MAGN_Y) {
		mlx90393_convert(val, mlx90393_y);
	} else if (chan == SENSOR_CHAN_MAGN_Z) {
		mlx90393_convert(val, mlx90393_z);
	}*/

	return 0;
}


int mlx90393_attr_get(const struct device *dev, enum sensor_channel chan,
	enum sensor_attribute attr, struct sensor_value *val) {
	// TODO: Main shit
	LOG_ERR("mlx90393_attr_get: Not implemented yet!: CHAN: %u, ATTR: %u", chan, attr);
	return 0;
}

int mlx90393_attr_set(const struct device *dev, enum sensor_channel chan,
	enum sensor_attribute attr, const struct sensor_value *val) {
	// TODO: Main shit
	LOG_ERR("mlx90393_attr_set: Not implemented yet!");
	return 0;
}

static const struct sensor_driver_api mlx90393_driver_api = {
	.sample_fetch = mlx90393_sample_fetch,
	.channel_get  = mlx90393_channel_get,
	.attr_set = mlx90393_attr_set,
	.attr_get = mlx90393_attr_get
	//.trigger_set = mlx90393_trigger_set,
};

int mlx90393_init(const struct device *dev) {

	const struct mlx90393_cfg *cfg = dev->config;

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

	// TODO: TBD if this is the default mode we want
	// warm reset and set burst (min is 20ms, only multiples of 20ms allowed)
	mlx90393_reset(dev);
	mlx90393_set_mode(dev,
		MLX90393_T,
		MLX90393_BURST_MODE, 20);

	// Chip should now be initialized and sending out data
	LOG_INF("Succesfully initialized MLX90393");

	return 0;
}


#define MLX90393_DEFINE(inst)									\
	static struct mlx90393_data mlx90393_data_##inst;			\
																\
	static const struct mlx90393_cfg mlx90393_cfg_##inst = {	\
		COND_CODE_1(DT_INST_ON_BUS(inst, spi),					\
			    ( .spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER | \
							 SPI_MODE_CPOL | SPI_MODE_CPHA |	\
							 SPI_WORD_SET(8), 0),),())			\
		COND_CODE_1(DT_INST_ON_BUS(inst, i2c),					\
			    ( .i2c = I2C_DT_SPEC_INST_GET(inst),),())		\
	};															\
									\
	DEVICE_DT_INST_DEFINE( 			\
		inst, 						\
		mlx90393_init, 				\
		NULL,						\
		&mlx90393_data_##inst,		\
		&mlx90393_cfg_##inst,		\
		POST_KERNEL, 				\
		CONFIG_SENSOR_INIT_PRIORITY,	\
		&mlx90393_driver_api);			\

DT_INST_FOREACH_STATUS_OKAY(MLX90393_DEFINE)