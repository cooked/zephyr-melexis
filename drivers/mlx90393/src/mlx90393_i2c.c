/*
 * Copyright (c) 2022 Stefano Cottafavi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/i2c.h>
#include <logging/log.h>

#include "mlx90393.h"

LOG_MODULE_DECLARE(MLX90393);

int i2c_write_mlx90393(const struct device *dev,
		const void *write_buf, size_t num_write) {

	const struct mlx90393_data_t *data = dev->data;
	const struct mlx90393_config *cfg = dev->config;

	if(i2c_write(data->i2c, write_buf, num_write, cfg->addr)<0)
		return -EIO;
	return 0;
}

int i2c_write_read_mlx90393(const struct device *dev,
		const void *write_buf, size_t num_write, void *read_buf, size_t num_read) {

	const struct mlx90393_data_t *data = dev->data;
	const struct mlx90393_config *cfg = dev->config;

	if(i2c_write_read(data->i2c, cfg->addr, write_buf, num_write, read_buf, num_read)<0)
		return -EIO;
	return 0;
}
