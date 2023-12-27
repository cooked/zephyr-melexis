/*
 * Copyright (c) 2022 Stefano Cottafavi
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include "mlx90393.h"


#define SLEEP_TIME_MS   1000
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
const struct device *const mlx = DEVICE_DT_GET_ONE(melexis_mlx90393);

int main(void)
{
	int err;

	if (!device_is_ready(led.port)) {
		printk("LED is not ready\n");
		return 0;
	}

	if (!device_is_ready(mlx)) {
		printk("Magnetometer is not ready\n");
		return 0;
	}


	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (err < 0) {
		return 0;
	}

	printk("Raspberry Pi Pico test\n");


	// warm reset and set burst (min is 20ms, only multiples of 20ms allowed)
	mlx90393_reset(mlx);
	mlx90393_set_mode(mlx,
		MLX90393_T | MLX90393_X | MLX90393_Y | MLX90393_Z,
		MLX90393_BURST_MODE, 100);


	struct sensor_value mag[4];

	while (1) {

		err = sensor_sample_fetch(mlx);
		if (err) {
			printk("Failed to fetch MAG (%d)\n", err);
			continue;
		}
		err = sensor_channel_get(mlx, SENSOR_CHAN_ALL, mag);
		if (err) {
			printk("Failed to get data MAG (%d)\n", err);
			continue;
		}

		printk("Values mag --- %.2f, %.2f, %.2f \n",
			sensor_value_to_double(&mag[0]),
			sensor_value_to_double(&mag[1]),
			sensor_value_to_double(&mag[2])
		);


		err = gpio_pin_toggle_dt(&led);
		if (err < 0) {
			return 0;
		}


		k_msleep(SLEEP_TIME_MS);
	}
}
