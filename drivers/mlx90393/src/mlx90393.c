

#include <zephyr/types.h>		// For Zephyr types and stdint.h
#include <device.h>
#include <devicetree.h>

#include <sys/__assert.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <init.h>
#include <kernel.h>
#include <string.h>
#include <logging/log.h>

#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>

#include <sensor_common.h>
#include <mlx90393.h>
#include <mlx90393_i2c.h>

LOG_MODULE_REGISTER(MLX90393, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT melexis_mlx90393

// Utility functions
const char *bit_rep[16] = {
	[ 0] = "0000", [ 1] = "0001", [ 2] = "0010", [ 3] = "0011",
	[ 4] = "0100", [ 5] = "0101", [ 6] = "0110", [ 7] = "0111",
	[ 8] = "1000", [ 9] = "1001", [10] = "1010", [11] = "1011",
	[12] = "1100", [13] = "1101", [14] = "1110", [15] = "1111",
};

void print_byte(uint8_t byte) {
	LOG_DBG("%s%s", bit_rep[byte >> 4], bit_rep[byte & 0x0F]);
}

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
uint8_t count_set_bits(int n) {
	uint8_t count = 0;
	while(n) {
        count += n & 1;
        n >>= 1;
    }
	return count;
}

////////////////////////////////////////////////////////////////////////////////
// Main driver implementation
////////////////////////////////////////////////////////////////////////////////

// Up to 9 bytes may be returned.
uint8_t mlx90393_data_buffer[10] = {0U};
uint8_t mlx90393_status = 0;
uint16_t mlx90393_t     = 0;
uint16_t mlx90393_x     = 0;
uint16_t mlx90393_y     = 0;
uint16_t mlx90393_z     = 0;

// Commands
// TODO: these ideally should be removed from the IF
// and abstracted into higher level functions
int mlx90393_cmd_EX(const struct device *dev) {

	mlx90393_data_buffer[0] = MLX90393_CMD_EX;

	return i2c_write_read_mlx90393( dev,
			mlx90393_data_buffer, 1,
			mlx90393_data_buffer, 1);
}
int mlx90393_cmd_SM(const struct device *dev, uint8_t zyxt) {

	mlx90393_data_buffer[0] = MLX90393_CMD_SM | zyxt;

	return i2c_write_read_mlx90393( dev,
			mlx90393_data_buffer, 1,
			mlx90393_data_buffer, 1);
}
int mlx90393_cmd_RM(const struct device *dev, uint8_t zyxt) {

	mlx90393_data_buffer[0] = MLX90393_CMD_RM | zyxt;

	for(int i=0; i<2*count_set_bits(zyxt); i++) {
        mlx90393_data_buffer[i+2] = 0x00;
    }

	return i2c_write_read_mlx90393( dev,
			&mlx90393_data_buffer[0], 1,
			&mlx90393_data_buffer[0], 1+2*count_set_bits(zyxt));
}
int mlx90393_cmd_SB(const struct device *dev, uint8_t zyxt) {

	mlx90393_data_buffer[0] = MLX90393_CMD_SB | zyxt;

	return i2c_write_read_mlx90393( dev,
			mlx90393_data_buffer, 1,
			mlx90393_data_buffer, 1);
}
int mlx90393_cmd_WR(const struct device *dev, uint8_t zyxt, uint8_t reg, uint16_t data) {

	mlx90393_data_buffer[0] = MLX90393_CMD_WR | zyxt;
	mlx90393_data_buffer[1] = (data & 0xff00) >> 8;
	mlx90393_data_buffer[2] = data & 0x00ff;
	mlx90393_data_buffer[3] = reg << 2;

	return i2c_write_read_mlx90393( dev,
			mlx90393_data_buffer, 4,
			mlx90393_data_buffer, 1);
}
int mlx90393_cmd_RR(const struct device *dev, uint8_t reg) {

	mlx90393_data_buffer[0] = MLX90393_CMD_RR;
	mlx90393_data_buffer[1] = reg << 2;

	return i2c_write_read_mlx90393( dev,
			mlx90393_data_buffer, 2,
			mlx90393_data_buffer, 3);
}
int mlx90393_cmd_RT(const struct device *dev) {

	mlx90393_data_buffer[0] = MLX90393_CMD_RT;

	return i2c_write_mlx90393( dev,
			mlx90393_data_buffer, 1);
}

// Interface
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

int mlx90393_reset(const struct device *dev) {
	mlx90393_cmd_EX(dev);
	k_msleep(50);
	mlx90393_cmd_RT(dev);
	k_msleep(50);
	return 0;
}

int mlx90393_set_mode(const struct device *dev, uint8_t zyxt, enum mlx90393_mode mode, uint32_t rate_ms) {

	struct mlx90393_data_t *data = dev->data;
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
			LOG_DBG("STATUS after reset 0x%X", mlx90393_data_buffer[0]);	// show RESET?
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

void mlx90393_decode(uint8_t zyxt) {
	uint8_t *p = (uint8_t *) mlx90393_data_buffer;

	mlx90393_status = *p;
	p += 1;

	if ((zyxt & MLX90393_T) != 0) {
		mlx90393_t = assemble_16(p);
		p += 2;
	}

	if ((zyxt & MLX90393_X) != 0) {
		mlx90393_x = assemble_16(p);
		p += 2;
	}

	if ((zyxt & MLX90393_Y) != 0) {
		mlx90393_y = assemble_16(p);
		p += 2;
	}

	if ((zyxt & MLX90393_Z) != 0) {
		mlx90393_z = assemble_16(p);
		p += 2;
	}
}

////////////////////////////////////////////////////////////////////////////////
// Zephyr Sensor API implementation
////////////////////////////////////////////////////////////////////////////////

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

// static void mlx90393_convert(struct sensor_value *val, int16_t sample, uint8_t adjustment) {
static void mlx90393_convert(struct sensor_value *val, uint16_t sample) {
	int32_t conv_val = sample;

	// conv_val  = sample * MLX90393_MICRO_GAUSS_PER_BIT * ((uint16_t) adjustment + 128) / 256;
	// val->val1 = conv_val / 1000000;
	// val->val2 = conv_val % 1000000;

	// TODO: Calculate proper MLX90393 magnetometer values. For now, use ints.
	val->val1 = conv_val;
	val->val2 = 0;
}

// TODO: TBC new implmentation wrecklab
static int mlx90393_sample_fetch(const struct device *dev, enum sensor_channel chan) {

	//struct mlx90393_data_t *data = dev->data;

	// Double check to see that we have a valid channel to fetch and set zyxt
	uint8_t zyxt = get_zyxt_from_sensor_channel(chan);

	if (zyxt == MLX90393_NOTHING) {
		LOG_ERR("Unsupported sensor channel %d for MLX90393.", chan);
		//return -ENOTSUP;
	}

	// TODO: if not bursting, kick off single measurement
	//if( data->current_mode != MLX90393_BURST_MODE) {
	//	mlx90393_command(MLX90393_CMD_SM, zyxt, 0, 0);
	//}

	// read measurement
	int err = mlx90393_cmd_RM(dev, zyxt);

	return err;
}


static int mlx90393_channel_get(const struct device *dev, enum sensor_channel chan,
	struct sensor_value *val) {

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

	// convert to sensor_value
	mlx90393_decode(zyxt);

	bool add_temp = (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_DIE_TEMP);

	if (chan == SENSOR_CHAN_MAGN_XYZ || chan == SENSOR_CHAN_ALL) {
		mlx90393_convert(val++, mlx90393_x);
		mlx90393_convert(val++, mlx90393_y);
		mlx90393_convert(val++, mlx90393_z);
	} else if (chan == SENSOR_CHAN_MAGN_X) {
		mlx90393_convert(val, mlx90393_x);
	} else if (chan == SENSOR_CHAN_MAGN_Y) {
		mlx90393_convert(val, mlx90393_y);
	} else if (chan == SENSOR_CHAN_MAGN_Z) {
		mlx90393_convert(val, mlx90393_z);
	}

	if (add_temp) {
		mlx90393_convert(val, mlx90393_t);
	}

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
	//.trigger_set = mlx90393_trigger_set,
	.sample_fetch = mlx90393_sample_fetch,
	.channel_get  = mlx90393_channel_get,
	.attr_set = mlx90393_attr_set,
	.attr_get = mlx90393_attr_get
};

int mlx90393_init(const struct device *dev) {

	LOG_INF("Starting MLX90393 initialization...");

	struct mlx90393_data_t *data = dev->data;
	const struct mlx90393_config *cfg = dev->config;

	// Setup i2c
	data->i2c = device_get_binding(cfg->i2c_label);

	if (data->i2c == NULL) {
		LOG_ERR("Failed to get pointer to %s device!", cfg->i2c_label);
		return -ENODEV;
	}

	// Chip should now be in the idle state
	LOG_INF("Succesfully initialized MLX90393");

	return 0;
}

static struct mlx90393_config mlx90393_cfg_0 = {
	.i2c_label = DT_INST_BUS_LABEL(0),
	.addr = DT_INST_REG_ADDR(0)
};

static struct mlx90393_data_t mlx90393_data;

DEVICE_DT_INST_DEFINE( \
	0, \
	mlx90393_init, \
	NULL, \
	&mlx90393_data, \
	&mlx90393_cfg_0, \
	POST_KERNEL, \
	CONFIG_SENSOR_INIT_PRIORITY, \
	&mlx90393_driver_api);
