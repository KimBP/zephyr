/*
 * Copyright (c) 2026 Kim Bøndergaard <kim@fam-boendergaard.dk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/drivers/display/rgb_scanout.h>

int display_rgb_scanout_init(const struct device *scanout, const struct device *panel)
{
	const struct display_rgb_scanout_api *api;

	if ((scanout == NULL) || (panel == NULL)) {
		return -EINVAL;
	}

	if (!device_is_ready(scanout)) {
		return -ENODEV;
	}

	api = scanout->api;
	if ((api == NULL) || (api->init == NULL)) {
		return -ENOSYS;
	}

	return api->init(scanout, panel);
}

int display_rgb_scanout_write(const struct device *scanout, const struct device *panel,
			     uint16_t x, uint16_t y,
			     const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct display_rgb_scanout_api *api;

	if ((scanout == NULL) || (panel == NULL)) {
		return -EINVAL;
	}

	if (!device_is_ready(scanout)) {
		return -ENODEV;
	}

	api = scanout->api;
	if ((api == NULL) || (api->write == NULL)) {
		return -ENOSYS;
	}

	return api->write(scanout, panel, x, y, desc, buf);
}
