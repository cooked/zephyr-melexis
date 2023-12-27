# SPDX-License-Identifier: Apache-2.0

# add *.h to global include paths
zephyr_include_directories(src)

zephyr_library()
zephyr_library_sources(src/mlx90393.c)
zephyr_library_sources(src/mlx90393_cmd.c)
zephyr_library_sources_ifdef(CONFIG_MLX90393_I2C src/mlx90393_i2c.c)
zephyr_library_sources_ifdef(CONFIG_MLX90393_SPI src/mlx90393_spi.c)