/*
 * Copyright (c) 2022 Anything Connected
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MLX90393_MLX90393_I2C_H_
#define ZEPHYR_DRIVERS_SENSOR_MLX90393_MLX90393_I2C_H_

#include <device.h>

int i2c_write_mlx90393(const struct device *dev,
		const void *write_buf, size_t num_write);
int i2c_write_read_mlx90393(const struct device *dev,
		const void *write_buf, size_t num_write,
		void *read_buf, size_t num_read);

#endif /* ZEPHYR_DRIVERS_SENSOR_MLX90393_MLX90393_I2C_H_ */
