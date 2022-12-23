 # Copyright (c) 2022 Stefano Cottafavi
 #
 # SPDX-License-Identifier: Apache-2.0

if(CONFIG_MLX90393)
	# Add *.h to global include paths
	zephyr_include_directories(src)

	zephyr_library()
	zephyr_library_sources(src/mlx90393.c)
	zephyr_library_sources(src/mlx90393_i2c.c)
endif()
