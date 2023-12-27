//

#ifndef __ZEPHYR_DRIVERS_SENSOR_MLX90395_H__
#define __ZEPHYR_DRIVERS_SENSOR_MLX90395_H__

#include <zephyr/device.h>				// For Zephyr's Device API
#include <zephyr/drivers/gpio.h>		// For Zephyr's GPIO API
#include <zephyr/drivers/sensor.h>		// For Zephyr's Sensor API

#include <zephyr/types.h>				// For Zephyr types and stdint.h
#include <zephyr/sys/util.h>			// For stuff like BIT(n) etc

//#if DT_INST_NODE_HAS_PROP(0, interrupt_gpios)
//#error "This MLX90395 driver depends on an interrupt GPIO pin being available!"
//#endif

/*******************************************************************************
Driver internals
*******************************************************************************/

// Assemble an uint16_t from <MSB,LSB>.
uint16_t assemble_16(uint8_t *p_data);
// Assemble an uint32_t from <MSB,LSB>.
uint32_t assemble_32(uint8_t *p_data);


/*******************************************************************************
Status byte and all the individual bits that comprise it
*******************************************************************************/

#define MLX90395_STATUS_BYTE_BURST_MODE		BIT(7)
#define MLX90395_STATUS_BYTE_WOC_MODE		BIT(6)
#define MLX90395_STATUS_BYTE_SM_MODE		BIT(5)
#define MLX90395_STATUS_BYTE_ERROR			BIT(4)
#define MLX90395_STATUS_BYTE_SED			BIT(3)
#define MLX90395_STATUS_BYTE_RS				BIT(2)
#define MLX90395_STATUS_BYTE_D1				BIT(1)
#define MLX90395_STATUS_BYTE_D0				BIT(0)

// Flags to use with "zyxt" variables.
#define MLX90395_NOTHING (0x0) /* Nothing */
#define MLX90395_T (0x01) /* Temperature */
#define MLX90395_X (0x02) /* X-axis */
#define MLX90395_Y (0x04) /* Y-axis */
#define MLX90395_Z (0x08) /* Z-axis */


/*******************************************************************************
Memory and registers
*******************************************************************************/

// Memory areas.
#define MLX90395_CUSTOMER_AREA_BEGIN		(0x00)
#define MLX90395_CUSTOMER_AREA_END			(0x1F)
#define MLX90395_CUSTOMER_AREA_FREE_BEGIN	(0x0A)
#define MLX90395_CUSTOMER_AREA_FREE_END		(0x1F)
#define MLX90395_MELEXIS_AREA_BEGIN			(0x20)
#define MLX90395_MELEXIS_AREA_END			(0x3F)

// TODO: Fill in registers and information (page 29-35)...
#define MLX90395_REG_Z_SERIES				(0x0)
#define MLX90395_REG_Z_SERIES_BITS			BIT(7)
#define MLX90395_REG_TRIG_INT				(0x01)
#define MLX90395_REG_TRIG_INT_BITS			BIT(15)


#define MLX90395_REG_GAIN_SEL_BITS			(BIT(4) | BIT(5) | BIT(6) | BIT(7))

// Sample rate is defined as Burst data rate * 20 ms
#define MLX90395_REG_BURST_DATA_RATE		(0x01)
#define MLX90395_REG_BURST_DATA_RATE_BITS	(BIT_MASK(5))

#define MLX90395_REG_BURST_SEL				(0x01)
#define MLX90395_REG_BURST_SEL_BITS			(BIT(6) | BIT(7) | BIT(8) | BIT(9))

#define MLX90395_REG_RES_X					(0x01)
#define MLX90395_REG_RES_X_BITS				(BIT(5) | BIT(6))

// etc...
// TODO: Add timing specifications here, for the delays later?
// TODO: Add XY-axis and Z-axis Noise over Conversion Time information?

enum async_init_state {
	ASYNC_INIT_STATE_IDLE,
	ASYNC_INIT_STATE_MEASURING,
	ASYNC_INIT_STATE_COUNT,
};


/*******************************************************************************
Driver internals
*******************************************************************************/

uint8_t count_set_bits(int n);

/* Bus */
int write_mlx(const struct device *dev, const void *write_buf, size_t num_write);
int write_read_mlx(const struct device *dev, const void *write_buf, size_t num_write, void *read_buf, size_t num_read);

// Commands
int mlx_cmd_WR(const struct device *dev, uint8_t reg, uint16_t val);
int mlx_cmd_SM(const struct device *dev, uint8_t zyxt);
int mlx_cmd_SB(const struct device *dev, uint8_t zyxt);
int mlx_cmd_RM(const struct device *dev, uint8_t zyxt);
int mlx_cmd_RR(const struct device *dev, uint8_t reg, uint16_t *val);
int mlx_cmd_EX(const struct device *dev);
int mlx_cmd_RT(const struct device *dev);

#endif /* __ZEPHYR_DRIVERS_SENSOR_MLX90395_H__ */
