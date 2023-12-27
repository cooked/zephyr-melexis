/*
 * Copyright (c) 2022 Stefano Cottafavi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#if CONFIG_MLX90393_SPI
#include <zephyr/drivers/spi.h>
#elif CONFIG_MLX90393_I2C
#include <zephyr/drivers/i2c.h>
#endif
#include <zephyr/logging/log.h>

#include "mlx90393.h"

LOG_MODULE_DECLARE(MLX90393);


uint8_t count_set_bits(int n) {
	uint8_t count = 0;
	while(n) {
        count += n & 1;
        n >>= 1;
    }
	return count;
}


/* Protocol */
int mlx90393_cmd_EX(const struct device *dev) {

	struct mlx90393_data *data = dev->data;

	data->mlx90393_data_buffer[0] = MLX90393_CMD_EX;

	return write_read_mlx90393( dev,
			data->mlx90393_data_buffer, 1,
			data->mlx90393_data_buffer, 1);
}
int mlx90393_cmd_SM(const struct device *dev, uint8_t zyxt) {

	struct mlx90393_data *data = dev->data;

	data->mlx90393_data_buffer[0] = MLX90393_CMD_SM | zyxt;

	return write_read_mlx90393( dev,
			data->mlx90393_data_buffer, 1,
			data->mlx90393_data_buffer, 1);
}
int mlx90393_cmd_RM(const struct device *dev, uint8_t zyxt) {

	struct mlx90393_data *data = dev->data;

	data->mlx90393_data_buffer[0] = MLX90393_CMD_RM | zyxt;

	for(int i=0; i<2*count_set_bits(zyxt); i++) {
        data->mlx90393_data_buffer[i+2] = 0x00;
    }

	int ret;

	ret = write_read_mlx90393( dev,
			&data->mlx90393_data_buffer[0], 1,
			&data->mlx90393_data_buffer[0], 1+2*count_set_bits(zyxt));

	return ret;
}
int mlx90393_cmd_SB(const struct device *dev, uint8_t zyxt) {

	struct mlx90393_data *data = dev->data;

	data->mlx90393_data_buffer[0] = MLX90393_CMD_SB | zyxt;

	return write_read_mlx90393( dev,
			data->mlx90393_data_buffer, 1,
			data->mlx90393_data_buffer, 1);
}
int mlx90393_cmd_WR(const struct device *dev, uint8_t zyxt, uint8_t reg, uint16_t val) {

	struct mlx90393_data *data = dev->data;

	data->mlx90393_data_buffer[0] = MLX90393_CMD_WR | zyxt;
	data->mlx90393_data_buffer[1] = (val & 0xff00) >> 8;
	data->mlx90393_data_buffer[2] = val & 0x00ff;
	data->mlx90393_data_buffer[3] = reg << 2;

	return write_read_mlx90393( dev,
			data->mlx90393_data_buffer, 4,
			data->mlx90393_data_buffer, 1);
}
int mlx90393_cmd_RR(const struct device *dev, uint8_t reg) {

	struct mlx90393_data *data = dev->data;

	data->mlx90393_data_buffer[0] = MLX90393_CMD_RR;
	data->mlx90393_data_buffer[1] = reg << 2;

	return write_read_mlx90393( dev,
			data->mlx90393_data_buffer, 2,
			data->mlx90393_data_buffer, 3);
}
int mlx90393_cmd_RT(const struct device *dev) {

	struct mlx90393_data *data = dev->data;

	data->mlx90393_data_buffer[0] = MLX90393_CMD_RT;

	return write_mlx90393( dev,
			data->mlx90393_data_buffer, 1);
}


/* Commands */
int mlx90393_reset(const struct device *dev) {
	mlx90393_cmd_EX(dev);
	k_msleep(50);
	mlx90393_cmd_RT(dev);
	k_msleep(50);
	return 0;
}

enum mlx90393_mode mlx90393_get_mode() {
	// TODO: implement, this should actually get the mode from the chip
	// sending some dummy cmd and reading its status

	// Check the mode bits to see which mode is currently active, if any
	/*uint8_t status = mlx90393_data_buffer[0];

	if (status & MLX90393_STATUS_BYTE_BURST_MODE) {
		return MLX90393_BURST_MODE;
	}

	if (status & MLX90393_STATUS_BYTE_SM_MODE) {
		return MLX90393_SINGLE_MEASUREMENT_MODE;
	}

	if (status & MLX90393_STATUS_BYTE_WOC_MODE) {
		return MLX90393_WAKE_ON_CHANGE_MODE;
	}*/

	return MLX90393_IDLE_MODE;
}

int mlx90393_set_mode(const struct device *dev, uint8_t zyxt, enum mlx90393_mode mode, uint32_t rate_ms) {

	struct mlx90393_data *data = dev->data;
	uint16_t rate;

	data->zyxt = zyxt;

	switch(mode) {

		case MLX90393_IDLE_MODE:
			mlx90393_cmd_EX(dev);
			break;

		case MLX90393_BURST_MODE:
			// set speed first Tinterval = BURST_DATARATE * 20ms, then mode
			rate = rate_ms/20;
			mlx90393_cmd_WR(dev, zyxt, MLX90393_REG_BURST_DATA_RATE, rate);
			LOG_DBG("STATUS after reset 0x%X", data->mlx90393_data_buffer[0]);	// show RESET?
			mlx90393_cmd_SB(dev, zyxt);
			k_msleep(20);
			break;

		case MLX90393_SINGLE_MEASUREMENT_MODE:
			mlx90393_cmd_SM(dev, zyxt);
			break;

		case MLX90393_WAKE_ON_CHANGE_MODE:
			// TODO: implement
			break;

	}

	// TODO: add error check
	return 0;
}

int mlx90393_get_res(const struct device *dev) {

	struct mlx90393_data *data = dev->data;

	mlx90393_cmd_RR(dev, MLX90393_REG_RES_X);

	//uint8_t status =	data->mlx90393_data_buffer[0];
	uint8_t res = 		data->mlx90393_data_buffer[1] & MLX90393_REG_RES_X_BITS;

	LOG_DBG(" --- RES = %d", res);

	// Check the mode bits to see which mode is currently active, if any
	/*uint8_t status = mlx90393_data_buffer[0];

	if (status & MLX90393_STATUS_BYTE_BURST_MODE) {
		return MLX90393_BURST_MODE;
	}

	if (status & MLX90393_STATUS_BYTE_SM_MODE) {
		return MLX90393_SINGLE_MEASUREMENT_MODE;
	}

	if (status & MLX90393_STATUS_BYTE_WOC_MODE) {
		return MLX90393_WAKE_ON_CHANGE_MODE;
	}*/

	return 0;
}

int mlx90393_dump_regs(const struct device *dev) {

	uint16_t reg;

	struct mlx90393_data *data = dev->data;

	//for(uint8_t i=0; i<9; i++) {
		/*mlx90393_cmd_RR(dev, 0x00);
		reg = mlx90393_data_buffer[1]<<8 | mlx90393_data_buffer[2];

		LOG_DBG(" - REG 0x00 = 0x%4X", reg);
		*/
	//}

	mlx90393_cmd_RR(dev, 0x01);
	reg = data->mlx90393_data_buffer[1]<<8 | data->mlx90393_data_buffer[2];
	printk(" - REG 0x01 = 0x%X \n", reg);

	// Check the mode bits to see which mode is currently active, if any
	/*uint8_t status = mlx90393_data_buffer[0];

	if (status & MLX90393_STATUS_BYTE_BURST_MODE) {
		return MLX90393_BURST_MODE;
	}

	if (status & MLX90393_STATUS_BYTE_SM_MODE) {
		return MLX90393_SINGLE_MEASUREMENT_MODE;
	}

	if (status & MLX90393_STATUS_BYTE_WOC_MODE) {
		return MLX90393_WAKE_ON_CHANGE_MODE;
	}*/

	return 0;
}
