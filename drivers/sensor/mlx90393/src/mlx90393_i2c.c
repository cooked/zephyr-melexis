/*
 * Copyright (c) 2022 Stefano Cottafavi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "mlx90393.h"

LOG_MODULE_DECLARE(MLX90393);

int write_mlx90393(const struct device *dev, const void *write_buf, size_t num_write) {

	const struct mlx90393_cfg *cfg = dev->config;
	i2c_write_dt(&cfg->i2c, write_buf, num_write);
	return 0;
}

int write_read_mlx90393(const struct device *dev,
	const void *write_buf, size_t num_write, void *read_buf, size_t num_read) {

	const struct mlx90393_config *cfg = dev->config;
	i2c_write_read_dt(&cfg->i2c, write_buf, num_write, read_buf, num_read);
	return 0;
}
