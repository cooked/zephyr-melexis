
#ifndef __ZEPHYR_DRIVERS_SENSOR_MLX_H__
#define __ZEPHYR_DRIVERS_SENSOR_MLX_H__

#include <zephyr/kernel.h>				// For Zephyr's Device API
#include <zephyr/drivers/gpio.h>		// For Zephyr's GPIO API
#include <zephyr/drivers/sensor.h>		// For Zephyr's Sensor API

#include <zephyr/drivers/spi.h>

#include <zephyr/types.h>				// For Zephyr types and stdint.h
#include <zephyr/sys/util.h>

/*******************************************************************************
Key commands.
*******************************************************************************/

#define MLX_CMD_NOP (0x00)  /* No OPeration */
#define MLX_CMD_SB (0x10)   /* Start Burst mode */
#define MLX_CMD_SW (0x20)   /* Start Wake-up On Change */
#define MLX_CMD_SM (0x30)   /* Start Single Measurement (polling mode) */
#define MLX_CMD_RM (0x40)

#define MLX_CMD_RR (0x50)   /* Read from a Register */
#define MLX_CMD_WR (0x60)   /* Write to a Register */
#define MLX_CMD_EX (0x80)   /* EXit */
#define MLX_CMD_HR (0xD0)   /* Memory Recall */
#define MLX_CMD_HS (0xE0)   /* Memory Store */
#define MLX_CMD_RT (0xF0)   /* Reset */

// Bus interface (for SPI and I2C)
int write_read_mlx(const struct device *dev, const void *write_buf, size_t num_write, void *read_buf, size_t num_read);

enum mlx_mode {
	MLX_BURST_MODE,
	MLX_SINGLE_MEASUREMENT_MODE,
	MLX_WAKE_ON_CHANGE_MODE,
	MLX_IDLE_MODE,
};

/*******************************************************************************
Zephyr Sensor API implementation
*******************************************************************************/

int mlx_attr_set(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, const struct sensor_value *val);
int mlx_attr_get(const struct device *dev, enum sensor_channel chan, enum sensor_attribute attr, struct sensor_value *val);

struct mlx_data {

	// GPIO stuff required for the interrupt pin handling
	const struct device *gpio;
	struct gpio_callback gpio_callback;

	// K_SEM_DEFINE(data_ready_sem, 0, 1);
	//struct k_sem data_ready_sem;

	// zephyr interrupts/triggers
	sensor_trigger_handler_t trigger_handler;
	struct sensor_trigger trigger;

	// Current mode of the MLX90395
	enum mlx_mode mode;

	// active axes
	uint8_t zyxt;

	// Up to 9 bytes may be returned.
 	uint8_t data_buffer[16];
 	uint8_t status;
	uint8_t crc;
 	uint16_t x;
 	uint16_t y;
 	uint16_t z;
	uint16_t t;
	uint16_t v;
 	uint8_t memory[64 * 2]; // 64 16-bit words.

};

struct mlx_cfg {
	//struct i2c_dt_spec i2c;
	struct spi_dt_spec spi;
	uint8_t int_pin;
	uint8_t int_flags;
	const char *int_label;
	const char *gpio_label;
	gpio_pin_t gpio_pin;
	gpio_dt_flags_t gpio_dt_flags;

};

#endif /* __ZEPHYR_DRIVERS_SENSOR_MLX_H__ */