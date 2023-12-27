# SPDX-License-Identifier: Apache-2.0

# add *.h to global include paths
zephyr_include_directories(src)

zephyr_library()
zephyr_library_sources(src/mlx90395.c)
zephyr_library_sources(src/mlx_spi.c)
zephyr_library_sources(src/mlx_cmd.c)