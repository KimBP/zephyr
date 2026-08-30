/*
 * Copyright (c) 2026 Kim Bondergaard
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/display/rgb_scanout.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <hal/lcd_hal.h>
#include <hal/lcd_ll.h>

#define DT_DRV_COMPAT espressif_esp32_lcd_cam_rgb

LOG_MODULE_REGISTER(display_esp32_lcd_cam_rgb, CONFIG_DISPLAY_LOG_LEVEL);

#define ESP32_LCD_CAM_RGB_BUS_ID 0

struct esp32_lcd_cam_rgb_timing {
	uint16_t width;
	uint16_t height;
	uint8_t data_bus_width;
	uint32_t clock_frequency;
	uint16_t hsync_len;
	uint16_t hback_porch;
	uint16_t hfront_porch;
	uint16_t vsync_len;
	uint16_t vback_porch;
	uint16_t vfront_porch;
	bool hsync_active;
	bool vsync_active;
	bool de_active;
	bool pixelclk_active;
};

struct esp32_lcd_cam_rgb_ctx {
	lcd_hal_context_t hal;
	const struct device *panel;
	struct esp32_lcd_cam_rgb_timing timing;
	uint8_t *fb;
	size_t fb_capacity;
	size_t fb_size;
	size_t bytes_per_pixel;
	struct k_mutex lock;
	bool initialized;
	bool hw_ready;
};

struct esp32_lcd_cam_rgb_config {
	struct esp32_lcd_cam_rgb_timing timing;
};

static int esp32_lcd_cam_rgb_bytes_per_pixel(const struct esp32_lcd_cam_rgb_timing *timing)
{
	if (timing->data_bus_width <= 16U) {
		return 2;
	}

	if (timing->data_bus_width <= 24U) {
		return 3;
	}

	return -EINVAL;
}

static void esp32_lcd_cam_rgb_hw_prepare(struct esp32_lcd_cam_rgb_ctx *ctx)
{
	if (ctx->hw_ready) {
		return;
	}

	lcd_ll_enable_bus_clock(ESP32_LCD_CAM_RGB_BUS_ID, true);
	lcd_ll_reset_register(ESP32_LCD_CAM_RGB_BUS_ID);
	lcd_hal_init(&ctx->hal, ESP32_LCD_CAM_RGB_BUS_ID);
	lcd_ll_enable_clock(ctx->hal.dev, true);
	lcd_ll_reset(ctx->hal.dev);
	lcd_ll_fifo_reset(ctx->hal.dev);
	lcd_ll_enable_rgb_mode(ctx->hal.dev, true);
	lcd_ll_enable_color_convert(ctx->hal.dev, false);
	lcd_ll_enable_output_always_on(ctx->hal.dev, true);

	ctx->hw_ready = true;
}

static int esp32_lcd_cam_rgb_init_backend(const struct device *scanout,
					 const struct device *panel)
{
	const struct esp32_lcd_cam_rgb_config *cfg = scanout->config;
	struct esp32_lcd_cam_rgb_ctx *ctx = scanout->data;
	uint64_t fb_size64;
	int bpp;

	if ((scanout == NULL) || (panel == NULL)) {
		return -EINVAL;
	}

	if ((cfg->timing.width == 0U) || (cfg->timing.height == 0U)) {
		return -EINVAL;
	}

	bpp = esp32_lcd_cam_rgb_bytes_per_pixel(&cfg->timing);
	if (bpp < 0) {
		return bpp;
	}

	fb_size64 = (uint64_t)cfg->timing.width * (uint64_t)cfg->timing.height *
		   (uint64_t)bpp;
	if (fb_size64 > SIZE_MAX) {
		return -EOVERFLOW;
	}

	if (ctx->fb == NULL) {
		return -ENOMEM;
	}

	if ((size_t)fb_size64 > ctx->fb_capacity) {
		LOG_ERR("Framebuffer capacity too small: need %u have %u",
			(uint32_t)fb_size64, (uint32_t)ctx->fb_capacity);
		return -ENOMEM;
	}

	if (ctx->initialized) {
		if (ctx->panel == panel) {
			return 0;
		}
		return -EALREADY;
	}

	esp32_lcd_cam_rgb_hw_prepare(ctx);

	memset(ctx->fb, 0, (size_t)fb_size64);
	ctx->panel = panel;
	ctx->timing = cfg->timing;
	ctx->fb_size = (size_t)fb_size64;
	ctx->bytes_per_pixel = (size_t)bpp;
	ctx->initialized = true;

	LOG_INF("ESP32 LCD_CAM RGB backend ready: %ux%u @ %u Hz (%u-bit)",
		cfg->timing.width, cfg->timing.height,
		cfg->timing.clock_frequency, cfg->timing.data_bus_width);
	return 0;
}

static int esp32_lcd_cam_rgb_write_backend(const struct device *scanout,
					   const struct device *panel, uint16_t x, uint16_t y,
					   const struct display_buffer_descriptor *desc,
					   const void *buf)
{
	struct esp32_lcd_cam_rgb_ctx *ctx = scanout->data;
	size_t dst_stride;
	size_t src_stride;
	size_t row_copy;
	uint8_t *dst;
	const uint8_t *src;
	int ret;

	if ((scanout == NULL) || !ctx->initialized || (ctx->panel != panel)) {
		return -ENODEV;
	}

	if ((desc == NULL) || (buf == NULL)) {
		return -EINVAL;
	}

	if ((desc->width == 0U) || (desc->height == 0U) || (desc->pitch < desc->width)) {
		return -EINVAL;
	}

	if (((uint32_t)x + desc->width > ctx->timing.width) ||
	    ((uint32_t)y + desc->height > ctx->timing.height)) {
		return -EINVAL;
	}

	src_stride = (size_t)desc->pitch * ctx->bytes_per_pixel;
	row_copy = (size_t)desc->width * ctx->bytes_per_pixel;
	if (desc->buf_size < (src_stride * desc->height)) {
		return -EMSGSIZE;
	}

	dst_stride = (size_t)ctx->timing.width * ctx->bytes_per_pixel;
	dst = ctx->fb + ((size_t)y * dst_stride) + ((size_t)x * ctx->bytes_per_pixel);
	src = buf;

	ret = k_mutex_lock(&ctx->lock, K_FOREVER);
	if (ret != 0) {
		return ret;
	}

	for (uint16_t row = 0U; row < desc->height; row++) {
		memcpy(dst, src, row_copy);
		dst += dst_stride;
		src += src_stride;
	}

	k_mutex_unlock(&ctx->lock);
	return 0;
}

static int display_esp32_lcd_cam_rgb_init(const struct device *dev)
{
	struct esp32_lcd_cam_rgb_ctx *ctx = dev->data;

	k_mutex_init(&ctx->lock);
	return 0;
}

static const struct display_rgb_scanout_api esp32_lcd_cam_rgb_api = {
	.init = esp32_lcd_cam_rgb_init_backend,
	.write = esp32_lcd_cam_rgb_write_backend,
};

#define ESP32_LCD_CAM_RGB_BPP(inst) \
	((DT_INST_PROP(inst, data_bus_width) <= 16) ? 2U : 3U)

#define ESP32_LCD_CAM_RGB_FB_SIZE(inst) \
	((size_t)DT_INST_PROP(inst, width) * (size_t)DT_INST_PROP(inst, height) * \
	 (size_t)ESP32_LCD_CAM_RGB_BPP(inst))

#define ESP32_LCD_CAM_RGB_INST_DEFINE(inst) \
	BUILD_ASSERT(DT_INST_PROP(inst, data_bus_width) <= 24, \
		     "lcd_cam_rgb: data-bus-width > 24 is not supported"); \
	static __aligned(16) uint8_t __attribute__((section(".ext_ram.bss"))) \
		esp32_lcd_cam_rgb_fb_##inst[ESP32_LCD_CAM_RGB_FB_SIZE(inst)]; \
	static const struct esp32_lcd_cam_rgb_config esp32_lcd_cam_rgb_cfg_##inst = { \
		.timing = { \
			.width = DT_INST_PROP(inst, width), \
			.height = DT_INST_PROP(inst, height), \
			.data_bus_width = DT_INST_PROP(inst, data_bus_width), \
			.clock_frequency = DT_INST_PROP(inst, clock_frequency), \
			.hsync_len = DT_INST_PROP(inst, hsync_len), \
			.hback_porch = DT_INST_PROP(inst, hback_porch), \
			.hfront_porch = DT_INST_PROP(inst, hfront_porch), \
			.vsync_len = DT_INST_PROP(inst, vsync_len), \
			.vback_porch = DT_INST_PROP(inst, vback_porch), \
			.vfront_porch = DT_INST_PROP(inst, vfront_porch), \
			.hsync_active = DT_INST_PROP(inst, hsync_active), \
			.vsync_active = DT_INST_PROP(inst, vsync_active), \
			.de_active = DT_INST_PROP(inst, de_active), \
			.pixelclk_active = DT_INST_PROP(inst, pixelclk_active), \
		}, \
	}; \
	static struct esp32_lcd_cam_rgb_ctx esp32_lcd_cam_rgb_data_##inst = { \
		.fb = esp32_lcd_cam_rgb_fb_##inst, \
		.fb_capacity = ESP32_LCD_CAM_RGB_FB_SIZE(inst), \
	}; \
	DEVICE_DT_INST_DEFINE(inst, display_esp32_lcd_cam_rgb_init, NULL, \
			      &esp32_lcd_cam_rgb_data_##inst, &esp32_lcd_cam_rgb_cfg_##inst, \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, \
			      &esp32_lcd_cam_rgb_api);

DT_INST_FOREACH_STATUS_OKAY(ESP32_LCD_CAM_RGB_INST_DEFINE)
