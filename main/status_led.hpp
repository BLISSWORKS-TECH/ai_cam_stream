#pragma once

#include "esp_err.h"

namespace aicam {

esp_err_t status_led_init();
void status_led_set_pairing(bool active);
void status_led_set_receiver_linked(bool linked);
void status_led_set_stream_connected(bool connected);
void status_led_notify_motor_sent();
void status_led_notify_detection();
void status_led_set_ota_active(bool active);
void status_led_show_success();
void status_led_show_error();
void status_led_show_shutdown();
esp_err_t status_led_prepare_deep_sleep();

} // namespace aicam
