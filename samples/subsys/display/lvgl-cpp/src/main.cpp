/*
 * Copyright (c) 2018 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>

#include <lvglpp/core/display.h>
#include <lvglpp/core/event.h>
#include <lvglpp/core/group.h>
#include <lvglpp/widgets/label/label.h>
#include <lvglpp/widgets/button/button.h>
#include <lvglpp/widgets/btnmatrix/btnmatrix.h>
#include <lvglpp/widgets/arc/arc.h>
#include <lvgl_zinput.h>

#include <memory>
#include <vector>
#include <string>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

static uint32_t count;

#ifdef CONFIG_GPIO
static struct gpio_dt_spec button_gpio = GPIO_DT_SPEC_GET_OR(
		DT_ALIAS(sw0), gpios, {0});
static struct gpio_callback button_callback;

static void button_isr_callback(const struct device *port,
				struct gpio_callback *cb,
				uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	count = 0;
}
#endif /* CONFIG_GPIO */

static void lv_btn_click_callback(lv_event_t *e)
{
	ARG_UNUSED(e);

	count = 0;
}

int main(void)
{
	char count_str[11] = {0};
	const struct device *display_dev;
	std::unique_ptr<lvgl::widgets::Label> hello_world_label;

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Device not ready, aborting test");
		return 0;
	}

#ifdef CONFIG_GPIO
	if (gpio_is_ready_dt(&button_gpio)) {
		int err;

		err = gpio_pin_configure_dt(&button_gpio, GPIO_INPUT);
		if (err) {
			LOG_ERR("failed to configure button gpio: %d", err);
			return 0;
		}

		gpio_init_callback(&button_callback, button_isr_callback,
				   BIT(button_gpio.pin));

		err = gpio_add_callback(button_gpio.port, &button_callback);
		if (err) {
			LOG_ERR("failed to add button callback: %d", err);
			return 0;
		}

		err = gpio_pin_interrupt_configure_dt(&button_gpio,
						      GPIO_INT_EDGE_TO_ACTIVE);
		if (err) {
			LOG_ERR("failed to enable button callback: %d", err);
			return 0;
		}
	}
#endif /* CONFIG_GPIO */

#ifdef CONFIG_LV_Z_ENCODER_INPUT
	auto arc = lvgl::widgets::Arc(lvgl::core::scr_act());
	arc.align(LV_ALIGN_CENTER, 0, -15);
	arc.set_size(150, 150);

	auto arc_group = lvgl::core::Group(lv_group_create());
	arc_group.add_obj(arc);

	lvgl::core::Z_EncoderInputDevice encoder;
	encoder.set_group(arc_group);

#endif /* CONFIG_LV_Z_ENCODER_INPUT */

#ifdef CONFIG_LV_Z_KEYPAD_INPUT
	static const std::vector<std::string> btnm_map = {"1", "2", "3", "4", ""};

	auto btn_matrix = lvgl::widgets::ButtonMatrix(lvgl::core::scr_act());
	btn_matrix.align(LV_ALIGN_CENTER, 0, 70);
	btn_matrix.set_map(btnm_map);
	btn_matrix.set_size(100,50);

	auto btn_matrix_group = lvgl::core::Group(lv_group_create());
	btn_matrix_group.add_obj(btn_matrix);

	lvgl::core::Z_KeypadInputDevice keypad;
	keypad.set_group(btn_matrix_group);

#endif /* CONFIG_LV_Z_KEYPAD_INPUT */

	if (IS_ENABLED(CONFIG_LV_Z_POINTER_INPUT)) {
		auto hello_world_button = lvgl::widgets::Button(lvgl::core::scr_act());
		hello_world_button.align(LV_ALIGN_CENTER, 0, -15);
		hello_world_button.add_event_cb(lv_btn_click_callback, LV_EVENT_CLICKED);
		hello_world_label = std::make_unique<lvgl::widgets::Label>(hello_world_button);
		hello_world_button.release_ptr();
	} else {
		hello_world_label = std::make_unique<lvgl::widgets::Label>(lvgl::core::scr_act());
	}

	hello_world_label->set_text("Hello world!");
	hello_world_label->align(LV_ALIGN_CENTER, 0, 0);

	auto count_label = lvgl::widgets::Label(lvgl::core::scr_act());
	count_label.align(LV_ALIGN_BOTTOM_MID, 0, 0);

	lv_task_handler();
	display_blanking_off(display_dev);

	while (1) {
		if ((count % 100) == 0U) {
			sprintf(count_str, "%d", count/100U);
			count_label.set_text(count_str);
		}
		lv_task_handler();
		++count;
		k_sleep(K_MSEC(10));
	}
}
