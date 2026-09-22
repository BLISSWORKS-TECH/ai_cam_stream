#pragma once

#include "driver/gpio.h"

namespace aicam {

// AI Cam PCB v2 camera wiring. Keep configurable status pins away from these
// signals; driving one of them from the LED task would corrupt camera traffic.
constexpr bool board_camera_uses_gpio(gpio_num_t pin)
{
    switch (pin) {
    case GPIO_NUM_4:  // SIOD
    case GPIO_NUM_5:  // SIOC
    case GPIO_NUM_6:  // VSYNC
    case GPIO_NUM_7:  // HREF
    case GPIO_NUM_8:  // PCLK
    case GPIO_NUM_9:  // D2
    case GPIO_NUM_10: // D1
    case GPIO_NUM_11: // D3
    case GPIO_NUM_12: // D0
    case GPIO_NUM_13: // D4
    case GPIO_NUM_15: // D7
    case GPIO_NUM_16: // XCLK
    case GPIO_NUM_17: // D6
    case GPIO_NUM_18: // D5
    case GPIO_NUM_38: // PWDN
        return true;
    default:
        return false;
    }
}

constexpr gpio_num_t BOARD_SPEAKER_BCLK_GPIO = GPIO_NUM_47;
constexpr gpio_num_t BOARD_SPEAKER_DATA_GPIO = GPIO_NUM_48;

} // namespace aicam
