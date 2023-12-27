/*
 * Copyright (c) 2022 Stefano Cottafavi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT melexis_mlx90395

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

#include "mlx.h"

LOG_MODULE_DECLARE(MLX90395, CONFIG_SENSOR_LOG_LEVEL);


int write_read_mlx(const struct device *dev,
	const void *write_buf, size_t num_write, void *read_buf, size_t num_read) {

	const struct mlx_cfg *cfg = dev->config;

	const struct spi_buf spi_buf_tx = {
		.buf = write_buf,
		.len = num_write,
	};
	struct spi_buf_set tx = {
		.buffers = &spi_buf_tx,
		.count = 1,
	};


	struct spi_buf spi_buf_rx = {
		.buf = read_buf,
		.len = num_read,
	};
	struct spi_buf_set rx = {
		.buffers = &spi_buf_rx,
		.count = 1,
	};


	int ret = spi_transceive_dt(&cfg->spi, &tx, &rx);


	return ret;
}

#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */