#include "status_led.hpp"

#include "board_pins.hpp"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

namespace aicam {
namespace {

constexpr char TAG[] = "status_led";
constexpr int64_t RESULT_INDICATOR_US = 800000;
constexpr int64_t ERROR_INDICATOR_US = 1500000;
constexpr int64_t SHUTDOWN_INDICATOR_US = 600000;
constexpr int64_t DETECTION_INDICATOR_US = 600000;
constexpr int64_t MOTOR_INDICATOR_US = 500000;
constexpr int64_t FAST_HALF_PERIOD_US = 100000;
constexpr int64_t SLOW_HALF_PERIOD_US = 1000000;

constexpr gpio_num_t RED_LED_GPIO =
    static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_RED_LED_GPIO);
constexpr gpio_num_t GREEN_LED_GPIO =
    static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_GREEN_LED_GPIO);
constexpr gpio_num_t BOARD_LED_GPIO =
    static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_BOARD_LED_GPIO);
struct LedState {
    bool pairing;
    bool receiver_linked;
    bool stream_connected;
    bool ota_active;
    int64_t ota_started_us;
    int64_t slow_blink_started_us;
    int64_t pairing_success_started_us;
    int64_t pairing_success_until_us;
    int64_t error_started_us;
    int64_t error_until_us;
    int64_t detection_started_us;
    int64_t detection_until_us;
    int64_t motor_active_until_us;
    int64_t shutdown_started_us;
    int64_t shutdown_until_us;
    bool shutting_down;
    bool deep_sleep;
};

portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
LedState s_state{};

void set_led_levels(bool red_on, bool green_on, bool board_on)
{
    // Legacy firmware drives both code channels active-high.
    gpio_set_level(RED_LED_GPIO, red_on ? 1 : 0);
    gpio_set_level(GREEN_LED_GPIO, green_on ? 1 : 0);
    // The XIAO user LED next to the charge section is active-low.
    gpio_set_level(BOARD_LED_GPIO, board_on ? 0 : 1);
}

bool fast_blink_on(int64_t now_us, int64_t started_us)
{
    return now_us >= started_us &&
        ((((now_us - started_us) / FAST_HALF_PERIOD_US) & 1LL) == 0);
}

void led_task(void *)
{
    for (;;) {
        const int64_t now_us = esp_timer_get_time();
        LedState state{};
        portENTER_CRITICAL(&s_state_lock);
        state = s_state;
        portEXIT_CRITICAL(&s_state_lock);

        bool red_on = false;
        bool green_on = false;
        if (state.deep_sleep) {
            red_on = false;
            green_on = false;
        } else if (state.shutting_down) {
            if (state.shutdown_until_us > now_us) {
                const bool blink_on =
                    fast_blink_on(now_us, state.shutdown_started_us);
                red_on = blink_on;
                green_on = blink_on;
            }
        } else {
            // RED and GREEN are independent code channels, matching ledAni.cpp
            // in the existing firmware rather than combining them on one GPIO.
            const bool slow_blink_on =
                (((now_us - state.slow_blink_started_us) / SLOW_HALF_PERIOD_US) & 1LL) != 0;
            red_on = state.pairing || state.receiver_linked || slow_blink_on;
            green_on = state.motor_active_until_us > now_us;

            // New OTA transport reuses the legacy RED status channel. Pairing
            // success and errors also affect RED only, leaving GREEN independent.
            if (state.ota_active) {
                red_on = fast_blink_on(now_us, state.ota_started_us);
            } else if (state.pairing_success_until_us > now_us) {
                red_on = fast_blink_on(
                    now_us, state.pairing_success_started_us);
            } else if (state.error_until_us > now_us) {
                red_on = fast_blink_on(now_us, state.error_started_us);
            }

            if (state.detection_until_us > now_us) {
                green_on = fast_blink_on(now_us, state.detection_started_us);
            }
        }

        // GPIO21 is not one of the existing firmware's RED/GREEN code channels.
        set_led_levels(red_on, green_on, false);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

} // namespace

esp_err_t status_led_init()
{
    const gpio_num_t pair_button_gpio =
        static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_PAIR_BUTTON_GPIO);
    const gpio_num_t power_button_gpio =
        static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_POWER_BUTTON_GPIO);
    if (!GPIO_IS_VALID_OUTPUT_GPIO(RED_LED_GPIO) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(GREEN_LED_GPIO) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(BOARD_LED_GPIO) ||
        RED_LED_GPIO == GREEN_LED_GPIO ||
        RED_LED_GPIO == BOARD_LED_GPIO ||
        GREEN_LED_GPIO == BOARD_LED_GPIO ||
        RED_LED_GPIO == pair_button_gpio ||
        GREEN_LED_GPIO == pair_button_gpio ||
        BOARD_LED_GPIO == pair_button_gpio ||
        RED_LED_GPIO == power_button_gpio ||
        GREEN_LED_GPIO == power_button_gpio ||
        BOARD_LED_GPIO == power_button_gpio ||
        board_camera_uses_gpio(RED_LED_GPIO) ||
        board_camera_uses_gpio(GREEN_LED_GPIO) ||
        board_camera_uses_gpio(BOARD_LED_GPIO) ||
        RED_LED_GPIO == BOARD_SPEAKER_BCLK_GPIO ||
        RED_LED_GPIO == BOARD_SPEAKER_DATA_GPIO ||
        GREEN_LED_GPIO == BOARD_SPEAKER_BCLK_GPIO ||
        GREEN_LED_GPIO == BOARD_SPEAKER_DATA_GPIO ||
        BOARD_LED_GPIO == BOARD_SPEAKER_BCLK_GPIO ||
        BOARD_LED_GPIO == BOARD_SPEAKER_DATA_GPIO) {
        ESP_LOGE(TAG, "Unsafe or conflicting LED GPIO configuration");
        return ESP_ERR_INVALID_ARG;
    }

    // A previous legacy image can leave deep-sleep GPIO holds enabled.
    gpio_hold_dis(RED_LED_GPIO);
    gpio_hold_dis(GREEN_LED_GPIO);
    gpio_hold_dis(BOARD_LED_GPIO);
    gpio_hold_dis(BOARD_SPEAKER_BCLK_GPIO);
    gpio_hold_dis(BOARD_SPEAKER_DATA_GPIO);

    // Preload safe output latches before enabling the pins as outputs.
    gpio_set_level(RED_LED_GPIO, 0);
    gpio_set_level(GREEN_LED_GPIO, 0);
    gpio_set_level(BOARD_LED_GPIO, 1);
    gpio_set_level(BOARD_SPEAKER_BCLK_GPIO, 0);
    gpio_set_level(BOARD_SPEAKER_DATA_GPIO, 0);

    gpio_config_t config{};
    config.pin_bit_mask =
        (1ULL << RED_LED_GPIO) |
        (1ULL << GREEN_LED_GPIO) |
        (1ULL << BOARD_LED_GPIO) |
        (1ULL << BOARD_SPEAKER_BCLK_GPIO) |
        (1ULL << BOARD_SPEAKER_DATA_GPIO);
    config.mode = GPIO_MODE_OUTPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    const esp_err_t result = gpio_config(&config);
    if (result != ESP_OK) {
        return result;
    }
    set_led_levels(false, false, false);

    const int64_t slow_blink_started_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_state_lock);
    s_state.slow_blink_started_us = slow_blink_started_us;
    portEXIT_CRITICAL(&s_state_lock);

    if (xTaskCreate(led_task, "status_led", 2048, nullptr, 1, nullptr) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG,
             "Existing-FW LED mapping: RED GPIO%d active-high, GREEN GPIO%d active-high; board GPIO%d off",
             RED_LED_GPIO, GREEN_LED_GPIO, BOARD_LED_GPIO);
    return ESP_OK;
}

void status_led_set_pairing(bool active)
{
    portENTER_CRITICAL(&s_state_lock);
    s_state.pairing = active;
    portEXIT_CRITICAL(&s_state_lock);
}

void status_led_set_stream_connected(bool connected)
{
    // Stored for parity with the legacy UDP-client state. It intentionally
    // does not participate in LED output selection.
    portENTER_CRITICAL(&s_state_lock);
    s_state.stream_connected = connected;
    portEXIT_CRITICAL(&s_state_lock);
}

void status_led_set_receiver_linked(bool linked)
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_state_lock);
    if (s_state.receiver_linked && !linked) {
        s_state.slow_blink_started_us = now_us;
    }
    s_state.receiver_linked = linked;
    portEXIT_CRITICAL(&s_state_lock);
}

void status_led_notify_motor_sent()
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_state_lock);
    s_state.motor_active_until_us = now_us + MOTOR_INDICATOR_US;
    portEXIT_CRITICAL(&s_state_lock);
}

void status_led_notify_detection()
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_state_lock);
    // The existing firmware gives detection priority on GREEN only. RED keeps
    // showing link/pairing state at the same time.
    if (s_state.detection_until_us <= now_us) {
        s_state.detection_started_us = now_us;
        s_state.detection_until_us = now_us + DETECTION_INDICATOR_US;
    }
    portEXIT_CRITICAL(&s_state_lock);
}

void status_led_set_ota_active(bool active)
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_state_lock);
    s_state.ota_active = active;
    if (active) {
        s_state.ota_started_us = now_us;
        s_state.pairing_success_until_us = 0;
        s_state.error_until_us = 0;
    }
    portEXIT_CRITICAL(&s_state_lock);
}

void status_led_show_success()
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_state_lock);
    s_state.pairing = false;
    s_state.shutting_down = false;
    s_state.error_until_us = 0;
    s_state.pairing_success_started_us = now_us;
    s_state.pairing_success_until_us = now_us + RESULT_INDICATOR_US;
    portEXIT_CRITICAL(&s_state_lock);
}

void status_led_show_error()
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_state_lock);
    s_state.ota_active = false;
    s_state.shutting_down = false;
    s_state.pairing_success_until_us = 0;
    s_state.error_started_us = now_us;
    s_state.error_until_us = now_us + ERROR_INDICATOR_US;
    portEXIT_CRITICAL(&s_state_lock);
}

void status_led_show_shutdown()
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_state_lock);
    s_state.ota_active = false;
    s_state.pairing = false;
    s_state.pairing_success_until_us = 0;
    s_state.error_until_us = 0;
    s_state.detection_until_us = 0;
    s_state.shutdown_started_us = now_us;
    s_state.shutdown_until_us = now_us + SHUTDOWN_INDICATOR_US;
    s_state.shutting_down = true;
    portEXIT_CRITICAL(&s_state_lock);
}

esp_err_t status_led_prepare_deep_sleep()
{
    portENTER_CRITICAL(&s_state_lock);
    s_state.deep_sleep = true;
    portEXIT_CRITICAL(&s_state_lock);

    // Allow the LED task to observe deep_sleep before latching the safe levels.
    vTaskDelay(pdMS_TO_TICKS(75));
    set_led_levels(false, false, false);
    gpio_set_level(BOARD_SPEAKER_BCLK_GPIO, 0);
    gpio_set_level(BOARD_SPEAKER_DATA_GPIO, 0);
    ESP_RETURN_ON_ERROR(gpio_hold_en(RED_LED_GPIO), TAG, "RED hold failed");
    ESP_RETURN_ON_ERROR(gpio_hold_en(GREEN_LED_GPIO), TAG, "GREEN hold failed");
    ESP_RETURN_ON_ERROR(gpio_hold_en(BOARD_LED_GPIO), TAG, "board LED hold failed");
    ESP_RETURN_ON_ERROR(
        gpio_hold_en(BOARD_SPEAKER_BCLK_GPIO), TAG, "speaker BCLK hold failed");
    ESP_RETURN_ON_ERROR(
        gpio_hold_en(BOARD_SPEAKER_DATA_GPIO), TAG, "speaker DATA hold failed");
    return ESP_OK;
}

} // namespace aicam
