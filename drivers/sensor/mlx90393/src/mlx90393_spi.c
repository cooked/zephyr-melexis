/*
 * Copyright (c) 2022 Stefano Cottafavi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include "mlx90393.h"

LOG_MODULE_DECLARE(MLX90393);

int write_mlx90393(const struct device *dev, const void *write_buf, size_t num_write) {

	const struct mlx90393_cfg *cfg = dev->config;

	const struct spi_buf spi_buf_tx = {
		.buf = write_buf,
		.len = sizeof(write_buf),
	};
	struct spi_buf_set tx = {
		.buffers = &spi_buf_tx,
		.count = 1,
	};

	int ret = 0;

	// send command (and ignore rcv)
	ret = spi_transceive_dt(&cfg->spi, &tx, NULL);

	return ret;
}

int write_read_mlx90393(const struct device *dev,
	const void *write_buf, size_t num_write, void *read_buf, size_t num_read) {

	const struct mlx90393_cfg *cfg = dev->config;

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


	int ret = 0;

	// send command (and ignore rcv)
	ret = spi_transceive_dt(&cfg->spi, &tx, &rx);
	//	send dummy and keep rcv (result from previous transaction)
	ret = spi_transceive_dt(&cfg->spi, &tx, &rx);

	return ret;


}
