#pragma once

#include <lvglpp/core/indev.h>


namespace lvgl::core {

#ifdef CONFIG_LV_Z_KEYPAD_INPUT

class Z_KeypadInputDevice: public KeypadInputDevice {
public:
    Z_KeypadInputDevice();

};

#endif /* CONFIG_LV_Z_KEYPAD_INPUT */

#ifdef CONFIG_LV_Z_ENCODER_INPUT

class Z_EncoderInputDevice: public EncoderInputDevice {
public:
    Z_EncoderInputDevice();

};

#endif /* CONFIG_LV_Z_ENCODER_INPUT */

} /* namespace lvgl::core */
