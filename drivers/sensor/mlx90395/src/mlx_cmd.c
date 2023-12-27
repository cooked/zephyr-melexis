/*
 * Copyright (c) 2022 Stefano Cottafavi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>

#include "mlx.h"

uint8_t count_set_bits(int n) {
	uint8_t count = 0;
	while(n) {
        count += n & 1;
        n >>= 1;
    }
	return count;
}


/* Protocol */
int mlx_cmd_EX(const struct device *dev) {

	struct mlx_data *data = dev->data;

	uint8_t cmd[2] = { MLX_CMD_EX, 0x00 };

	write_read_mlx( dev, cmd, 2, data->data_buffer, 2);

	return 0;
}
int mlx_cmd_SM(const struct device *dev, uint8_t zyxt) {

	struct mlx_data *data = dev->data;

	uint8_t cmd[2] = { MLX_CMD_SM | zyxt, 0x00 };

	write_read_mlx( dev, cmd, 2, data->data_buffer, 2);

	return 0;
}
int mlx_cmd_RM(const struct device *dev, uint8_t zyxt) {

	struct mlx_data *data = dev->data;

	// max possible lenght of RM command
	uint8_t cmd[13] = { 0 };
	cmd[0] = MLX_CMD_RM | zyxt;

	// see DS sec. 17.6.1.3
	uint8_t nsize = 3 + 2*(count_set_bits(zyxt) + ((zyxt==0)?1:0));

	write_read_mlx(dev, cmd, nsize, &data->data_buffer, nsize);

	return 0;
}
int mlx_cmd_SB(const struct device *dev, uint8_t zyxt) {

	struct mlx_data *data = dev->data;

	uint8_t cmd[2] = { MLX_CMD_SB | zyxt, 0x00 };

	write_read_mlx( dev, cmd, 2, data->data_buffer, 2);

	return 0;
}
int mlx_cmd_WR(const struct device *dev, uint8_t reg, uint16_t val) {

	struct mlx_data *data = dev->data;

	uint8_t cmd[5] = { MLX_CMD_WR, val>>8, val&0xff, reg<<2, 0 };

	write_read_mlx( dev, cmd, 5, data->data_buffer, 5);

	return 0;
}
int mlx_cmd_RR(const struct device *dev, uint8_t reg, uint16_t *val) {

	struct mlx_data *data = dev->data;

	uint8_t cmd[5] = { MLX_CMD_RR, reg<<2, 0, 0, 0 };

	write_read_mlx( dev, cmd, 5, data->data_buffer, 5);

	*val = (((uint16_t)data->data_buffer[3]) << 8) | data->data_buffer[4]; //sys_get_be16(&data->data_buffer[3]);

	return 0;
}
int mlx_cmd_RT(const struct device *dev) {

	struct mlx_data *data = dev->data;

	uint8_t cmd[1] = { MLX_CMD_RT };

	return write_read_mlx( dev, cmd, 1, data->data_buffer, 1);
}
