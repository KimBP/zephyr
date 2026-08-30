/*
 * Copyright (c) 2026 Kim Bondergaard
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DISPLAY_RGB_SCANOUT_H_
#define ZEPHYR_INCLUDE_DRIVERS_DISPLAY_RGB_SCANOUT_H_

#include <errno.h>
#include <stdint.h>

struct device;
struct display_buffer_descriptor;

struct display_rgb_scanout_api {
	int (*init)(const struct device *scanout, const struct device *panel);
	int (*write)(const struct device *scanout, const struct device *panel,
		     uint16_t x, uint16_t y,
		     const struct display_buffer_descriptor *desc, const void *buf);
};

int display_rgb_scanout_init(const struct device *scanout, const struct device *panel);
int display_rgb_scanout_write(const struct device *scanout, const struct device *panel,
			     uint16_t x, uint16_t y,
			     const struct display_buffer_descriptor *desc, const void *buf);

#endif /* ZEPHYR_INCLUDE_DRIVERS_DISPLAY_RGB_SCANOUT_H_ */
