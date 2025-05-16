#include "lvgl_zinput.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <lvgl_common_input.h>

namespace lvgl::core {

#ifdef CONFIG_LV_Z_KEYPAD_INPUT

Z_KeypadInputDevice::Z_KeypadInputDevice()
{
    const struct device* dev =
        DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_lvgl_keypad_input));

    struct lvgl_common_input_data *common_data = 
        reinterpret_cast<struct lvgl_common_input_data*>(dev->data);

    // TODO: What if common_data == NULL
    indev_drv = common_data->indev_drv;
    update_driver();
}

#endif /* CONFIG_LV_Z_KEYPAD_INPUT */

#ifdef CONFIG_LV_Z_ENCODER_INPUT

Z_EncoderInputDevice::Z_EncoderInputDevice()
{
    const struct device* dev =
        DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_lvgl_encoder_input));

    struct lvgl_common_input_data *common_data =
        reinterpret_cast<struct lvgl_common_input_data*>(dev->data);

    // TODO: What if common_data == NULL
    indev_drv = common_data->indev_drv;
    update_driver();
}

#endif /* CONFIG_LV_Z_ENCODER_INPUT */

} /* namespace lvgl::core */
