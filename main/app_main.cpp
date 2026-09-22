#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "board_pins.hpp"
#include "camera_stream.hpp"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "motor_bridge.hpp"
#include "nvs_flash.h"
#include "ota_server.hpp"
#include "sdkconfig.h"
#include "status_led.hpp"
#include "stream_protocol.hpp"

namespace {

constexpr char TAG[] = "ai_cam_stream";
constexpr uint8_t STATUS_OK = 0;
constexpr uint8_t STATUS_BAD_REQUEST = 1;
constexpr uint8_t STATUS_APPLY_FAILED = 2;
constexpr uint8_t STATUS_BUSY = 3;
constexpr size_t CONTROL_DATAGRAM_MAX = 128;
constexpr int64_t DETECTION_REPLAY_WINDOW_US = 5000000;
constexpr size_t DETECTION_REPLAY_CACHE_SIZE = 8;

struct DetectionReplayEntry {
    bool valid;
    uint32_t source_ip_be;
    uint16_t source_port_be;
    uint32_t sequence;
    int64_t seen_us;
};

volatile uint32_t s_controller_ip_be = 0;
volatile bool s_motor_stop_requested = false;
bool s_stream_was_enabled_before_ota = false;
bool s_pairing_boot = false;
bool s_pairing_transition = false;
bool s_sleep_transition = false;
SemaphoreHandle_t s_control_gate = nullptr;
SemaphoreHandle_t s_control_ready_signal = nullptr;
DetectionReplayEntry s_detection_replay_cache[DETECTION_REPLAY_CACHE_SIZE]{};
size_t s_detection_replay_cursor = 0;

esp_err_t stop_motors()
{
    const uint8_t stop[6] = {3, 0, 3, 0, 3, 0};
    esp_err_t result = ESP_FAIL;
    // Send repeatedly because ESP-NOW is intentionally connectionless.
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (aicam::motor_bridge_send(stop) == ESP_OK) {
            result = ESP_OK;
        }
        if (attempt != 2) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    return result;
}

esp_err_t prepare_for_ota(void *)
{
    if (!s_control_gate ||
        xSemaphoreTake(s_control_gate, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if (s_pairing_transition || s_sleep_transition) {
        xSemaphoreGive(s_control_gate);
        ESP_LOGW(TAG, "OTA rejected because another power-state transition is pending");
        return ESP_ERR_INVALID_STATE;
    }
    s_stream_was_enabled_before_ota = aicam::camera_stream_is_enabled();
    aicam::camera_stream_enable(false);
    const esp_err_t stop_result = stop_motors();
    // Let the final connectionless ESP-NOW STOP leave the Wi-Fi queue before
    // flash erase/write activity begins. The receiver must keep its own timeout.
    vTaskDelay(pdMS_TO_TICKS(30));
    xSemaphoreGive(s_control_gate);
    if (stop_result != ESP_OK) {
        ESP_LOGE(TAG, "Could not queue an ESP-NOW STOP before OTA");
        return stop_result;
    }
    aicam::status_led_set_ota_active(true);
    ESP_LOGI(TAG, "Streaming stopped and motors stopped for OTA");
    return ESP_OK;
}

void ota_completed(bool success, esp_err_t result, void *)
{
    if (s_control_gate &&
        xSemaphoreTake(s_control_gate, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (!success && s_stream_was_enabled_before_ota) {
            // A failed upload never restarts motors, but video can safely resume
            // for the previously registered client.
            aicam::camera_stream_enable(true);
        }
        s_stream_was_enabled_before_ota = false;
        xSemaphoreGive(s_control_gate);
    }
    if (success) {
        aicam::status_led_set_ota_active(false);
        aicam::status_led_show_success();
        ESP_LOGI(TAG, "OTA verified; reboot scheduled");
    } else {
        aicam::status_led_show_error();
        ESP_LOGE(TAG, "OTA failed: %s", esp_err_to_name(result));
    }
}

bool running_image_is_pending_verify()
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state{};
    return running && esp_ota_get_state_partition(running, &state) == ESP_OK &&
           state == ESP_OTA_IMG_PENDING_VERIFY;
}

[[noreturn]] void fail_startup_and_rollback(const char *operation, esp_err_t result)
{
    ESP_LOGE(TAG, "%s failed: %s", operation, esp_err_to_name(result));
    if (running_image_is_pending_verify()) {
        ESP_LOGE(TAG, "New OTA image failed self-test; rolling back");
        esp_ota_mark_app_invalid_rollback_and_reboot();
    }
    abort();
}

void require_startup_ok(esp_err_t result, const char *operation)
{
    if (result != ESP_OK) {
        fail_startup_and_rollback(operation, result);
    }
}

void confirm_running_image()
{
    if (!running_image_is_pending_verify()) {
        return;
    }
    const esp_err_t result = esp_ota_mark_app_valid_cancel_rollback();
    if (result != ESP_OK) {
        fail_startup_and_rollback("OTA image confirmation", result);
    }
    ESP_LOGI(TAG, "OTA self-test passed; running image marked valid");
}

// Caller must hold s_control_gate. On success the caller intentionally keeps
// the gate until esp_restart(), so no motor/OTA operation can enter the gap.
esp_err_t prepare_pairing_restart_locked()
{
    if (s_sleep_transition || aicam::ota_server_update_in_progress() ||
        running_image_is_pending_verify()) {
        return ESP_ERR_INVALID_STATE;
    }

    const bool stream_was_enabled = aicam::camera_stream_is_enabled();
    const bool motors_were_running = aicam::motor_bridge_motors_running();
    s_pairing_transition = true;
    aicam::status_led_set_pairing(true);
    aicam::camera_stream_enable(false);

    const esp_err_t stop_result = stop_motors();
    if (stop_result != ESP_OK && motors_were_running) {
        s_pairing_transition = false;
        aicam::status_led_set_pairing(false);
        if (stream_was_enabled) {
            aicam::camera_stream_enable(true);
        }
        return stop_result;
    }
    if (stop_result != ESP_OK) {
        ESP_LOGW(TAG, "Previous peer did not confirm STOP; no motor was tracked as running");
    }

    const esp_err_t request_result = aicam::motor_bridge_request_pairing();
    if (request_result != ESP_OK) {
        s_pairing_transition = false;
        aicam::status_led_set_pairing(false);
        if (stream_was_enabled) {
            aicam::camera_stream_enable(true);
        }
    }
    return request_result;
}

void wifi_event_handler(void *, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base != WIFI_EVENT) {
        return;
    }

    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        const auto *event = static_cast<wifi_event_ap_staconnected_t *>(event_data);
        ESP_LOGI(TAG, "Client connected: %02X:%02X:%02X:%02X:%02X:%02X",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5]);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        const auto *event = static_cast<wifi_event_ap_stadisconnected_t *>(event_data);
        ESP_LOGW(TAG, "Client disconnected: %02X:%02X:%02X:%02X:%02X:%02X",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5]);
        if (s_pairing_boot) {
            return;
        }
        aicam::camera_stream_enable(false);
        aicam::camera_stream_set_client(0, 0);
        aicam::status_led_set_stream_connected(false);
        s_controller_ip_be = 0;
        s_motor_stop_requested = true;
    }
}

esp_err_t init_nvs()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase failed");
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t init_softap(uint8_t channel, bool pairing_mode)
{
    if (channel < 1 || channel > 13) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop init failed");
    if (!esp_netif_create_default_wifi_ap()) {
        return ESP_FAIL;
    }

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_config), TAG, "Wi-Fi init failed");
    ESP_RETURN_ON_ERROR(
        esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, nullptr, nullptr),
        TAG,
        "Wi-Fi event registration failed");

    wifi_country_t country{};
    memcpy(country.cc, "KR", 2);
    country.schan = 1;
    country.nchan = 13;
    country.policy = WIFI_COUNTRY_POLICY_MANUAL;
    ESP_RETURN_ON_ERROR(esp_wifi_set_country(&country), TAG, "country setup failed");

    uint8_t mac[6]{};
    ESP_RETURN_ON_ERROR(esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP), TAG, "MAC read failed");

    const char *ssid_prefix = CONFIG_AICAM_STREAM_WIFI_SSID_PREFIX;
    if (strlen(ssid_prefix) > 25) {
        ESP_LOGE(TAG, "SoftAP SSID prefix must be at most 25 characters");
        return ESP_ERR_INVALID_ARG;
    }

    char ssid[33]{};
    std::snprintf(
        ssid,
        sizeof(ssid),
        "%s_%02X%02X%02X",
        ssid_prefix,
        mac[3],
        mac[4],
        mac[5]);

    const char *password = CONFIG_AICAM_STREAM_WIFI_PASSWORD;
    const size_t password_length = strlen(password);
    if ((password_length != 0 && password_length < 8) || password_length > 63) {
        ESP_LOGE(TAG, "SoftAP password must be empty or contain 8..63 characters");
        return ESP_ERR_INVALID_ARG;
    }

    wifi_config_t config{};
    const size_t ssid_length = std::min(strlen(ssid), sizeof(config.ap.ssid));
    memcpy(config.ap.ssid, ssid, ssid_length);
    config.ap.ssid_len = ssid_length;
    memcpy(config.ap.password, password,
           std::min(password_length, sizeof(config.ap.password) - 1));
    config.ap.channel = channel;
    config.ap.max_connection = 1;
    config.ap.authmode = password_length == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    config.ap.pmf_cfg.capable = false;
    config.ap.pmf_cfg.required = false;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "AP mode setup failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &config), TAG, "AP config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "AP start failed");

    // The camera is a dedicated access point: disable power saving and allow
    // 802.11n. HT40 raises peak throughput but remains a build-time option for
    // phones that only associate reliably at 20 MHz.
    esp_wifi_set_ps(WIFI_PS_NONE);
    ESP_RETURN_ON_ERROR(
        esp_wifi_set_protocol(
            WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N),
        TAG,
        "Wi-Fi protocol setup failed");
    wifi_bandwidth_t bandwidth = WIFI_BW_HT20;
#if CONFIG_AICAM_STREAM_WIFI_HT40
    if (!pairing_mode) {
        const esp_err_t bandwidth_result =
            esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40);
        if (bandwidth_result == ESP_OK) {
            bandwidth = WIFI_BW_HT40;
        } else {
            ESP_LOGW(TAG, "HT40 unavailable on channel %u; using HT20: %s",
                     channel, esp_err_to_name(bandwidth_result));
            ESP_RETURN_ON_ERROR(
                esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20),
                TAG,
                "HT20 fallback failed");
        }
    } else {
        ESP_RETURN_ON_ERROR(
            esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20),
            TAG,
            "pairing HT20 setup failed");
    }
#else
    ESP_RETURN_ON_ERROR(
        esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20), TAG, "bandwidth setup failed");
#endif
    ESP_RETURN_ON_ERROR(esp_wifi_set_max_tx_power(78), TAG, "TX power setup failed");

    ESP_LOGI(TAG, "SoftAP ready: SSID=%s, IP=192.168.4.1, channel=%d, %s",
             ssid,
             channel,
             bandwidth == WIFI_BW_HT40 ? "HT40" : "HT20");
    return ESP_OK;
}

int create_control_socket()
{
    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "control socket() failed: errno=%d", errno);
        return -1;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(aicam::CONTROL_PORT);
    if (bind(sock, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
        ESP_LOGE(TAG, "control bind() failed: errno=%d", errno);
        close(sock);
        return -1;
    }

    timeval timeout{};
    timeout.tv_usec = 50000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    return sock;
}

void send_response(
    int sock,
    const sockaddr_in &destination,
    uint8_t request_command,
    uint32_t sequence_host,
    const uint8_t *payload,
    size_t payload_length)
{
    uint8_t datagram[sizeof(aicam::ControlHeader) + aicam::CONTROL_PAYLOAD_MAX]{};
    auto *header = reinterpret_cast<aicam::ControlHeader *>(datagram);
    header->magic = htonl(aicam::CONTROL_MAGIC);
    header->version = aicam::PROTOCOL_VERSION;
    header->command = request_command | aicam::CMD_RESPONSE_BIT;
    header->payload_len = htons(static_cast<uint16_t>(payload_length));
    header->sequence = htonl(sequence_host);
    if (payload_length != 0) {
        memcpy(datagram + sizeof(*header), payload, payload_length);
    }
    sendto(sock,
           datagram,
           sizeof(*header) + payload_length,
           0,
           reinterpret_cast<const sockaddr *>(&destination),
           sizeof(destination));
}

bool valid_motor_payload(const uint8_t *payload)
{
    for (size_t motor = 0; motor < 3; ++motor) {
        const uint8_t direction = payload[motor * 2];
        const uint8_t speed = payload[motor * 2 + 1];
        if (direction < 1 || direction > 3 || speed > 100) {
            return false;
        }
    }
    return true;
}

bool is_duplicate_detection_event(
    const sockaddr_in &source, uint32_t sequence, int64_t now_us)
{
    for (DetectionReplayEntry &entry : s_detection_replay_cache) {
        if (entry.valid &&
            now_us >= entry.seen_us &&
            now_us - entry.seen_us <= DETECTION_REPLAY_WINDOW_US &&
            entry.source_ip_be == source.sin_addr.s_addr &&
            entry.source_port_be == source.sin_port &&
            entry.sequence == sequence) {
            return true;
        }
    }

    DetectionReplayEntry &entry =
        s_detection_replay_cache[s_detection_replay_cursor];
    entry.valid = true;
    entry.source_ip_be = source.sin_addr.s_addr;
    entry.source_port_be = source.sin_port;
    entry.sequence = sequence;
    entry.seen_us = now_us;
    s_detection_replay_cursor =
        (s_detection_replay_cursor + 1) % DETECTION_REPLAY_CACHE_SIZE;
    return false;
}

void handle_control_datagram(
    int sock,
    const uint8_t *datagram,
    size_t datagram_length,
    const sockaddr_in &source)
{
    if (datagram_length < sizeof(aicam::ControlHeader)) {
        return;
    }

    aicam::ControlHeader wire_header{};
    memcpy(&wire_header, datagram, sizeof(wire_header));
    const uint32_t magic = ntohl(wire_header.magic);
    const uint16_t payload_length = ntohs(wire_header.payload_len);
    const uint32_t sequence = ntohl(wire_header.sequence);
    if (magic != aicam::CONTROL_MAGIC ||
        wire_header.version != aicam::PROTOCOL_VERSION ||
        (wire_header.command & aicam::CMD_RESPONSE_BIT) != 0 ||
        payload_length > aicam::CONTROL_PAYLOAD_MAX ||
        datagram_length != sizeof(wire_header) + payload_length) {
        return;
    }

    const uint8_t *payload = datagram + sizeof(wire_header);
    uint8_t response[20]{};
    size_t response_length = 1;
    response[0] = STATUS_OK;

    bool gate_taken = false;
    bool ota_busy = aicam::ota_server_update_in_progress();
    if (wire_header.command != aicam::CMD_PING) {
        if (!s_control_gate || xSemaphoreTake(s_control_gate, portMAX_DELAY) != pdTRUE) {
            response[0] = STATUS_BUSY;
            send_response(
                sock, source, wire_header.command, sequence, response, response_length);
            return;
        }
        gate_taken = true;
        // Recheck only after taking the same gate used by OTA preparation. If
        // OTA claimed first, no state-changing command can pass this point.
        ota_busy = aicam::ota_server_update_in_progress();
        if (ota_busy) {
            xSemaphoreGive(s_control_gate);
            response[0] = STATUS_BUSY;
            send_response(
                sock, source, wire_header.command, sequence, response, response_length);
            return;
        }
    }

    switch (wire_header.command) {
    case aicam::CMD_HELLO:
        if (payload_length != 1 || payload[0] > 2) {
            response[0] = STATUS_BAD_REQUEST;
            break;
        }
        s_controller_ip_be = source.sin_addr.s_addr;
        aicam::camera_stream_set_client(source.sin_addr.s_addr, htons(aicam::VIDEO_PORT));
        if (aicam::camera_stream_request_profile(
                static_cast<aicam::StreamProfile>(payload[0])) != ESP_OK) {
            response[0] = STATUS_APPLY_FAILED;
            break;
        }
        aicam::camera_stream_enable(true);
        aicam::status_led_set_stream_connected(true);
        response[1] = payload[0];
        response_length = 2;
        ESP_LOGI(TAG, "Streaming requested by %s, profile=%u",
                 inet_ntoa(source.sin_addr), payload[0]);
        break;

    case aicam::CMD_STOP_STREAM:
        if (payload_length != 0) {
            response[0] = STATUS_BAD_REQUEST;
            break;
        }
        aicam::camera_stream_enable(false);
        aicam::status_led_set_stream_connected(false);
        break;

    case aicam::CMD_SET_PROFILE:
        if (payload_length != 1 || payload[0] > 2) {
            response[0] = STATUS_BAD_REQUEST;
        } else if (aicam::camera_stream_request_profile(
                       static_cast<aicam::StreamProfile>(payload[0])) != ESP_OK) {
            response[0] = STATUS_APPLY_FAILED;
        } else {
            response[1] = payload[0];
            response_length = 2;
        }
        break;

    case aicam::CMD_MOTOR:
        if (payload_length != 6 || !valid_motor_payload(payload)) {
            response[0] = STATUS_BAD_REQUEST;
        } else if (aicam::motor_bridge_send(payload) != ESP_OK) {
            response[0] = STATUS_APPLY_FAILED;
        }
        break;

    case aicam::CMD_SET_PEER:
        if (payload_length != 7) {
            response[0] = STATUS_BAD_REQUEST;
        } else if (aicam::motor_bridge_set_peer(payload, payload[6], true) != ESP_OK) {
            response[0] = STATUS_APPLY_FAILED;
        }
        break;

    case aicam::CMD_DETECTION_EVENT:
        if (payload_length != 0) {
            response[0] = STATUS_BAD_REQUEST;
        } else if (!is_duplicate_detection_event(
                       source, sequence, esp_timer_get_time())) {
            // Standalone vision is intentionally absent. PC/mobile vision
            // reports an AprilTag or color detection through this event. UDP
            // retries reuse the sequence and must not restart the three flashes.
            aicam::status_led_notify_detection();
        }
        break;

    case aicam::CMD_PING: {
        if (payload_length != 0) {
            response[0] = STATUS_BAD_REQUEST;
            break;
        }
        aicam::StreamStats stats{};
        aicam::camera_stream_get_stats(&stats);
        response[1] = static_cast<uint8_t>(aicam::camera_stream_active_profile());
        response[2] = aicam::camera_stream_is_enabled() ? 1 : 0;
        response[3] = ota_busy ? 1 : 0;
        const uint32_t frames_sent_be = htonl(stats.frames_sent);
        const uint32_t frames_dropped_be = htonl(stats.frames_dropped);
        const uint32_t send_errors_be = htonl(stats.send_errors);
        memcpy(response + 4, &frames_sent_be, sizeof(frames_sent_be));
        memcpy(response + 8, &frames_dropped_be, sizeof(frames_dropped_be));
        memcpy(response + 12, &send_errors_be, sizeof(send_errors_be));
        aicam::MotorBridgeStatus motor_status{};
        aicam::motor_bridge_get_status(&motor_status);
        response[16] = motor_status.linked ? 1 : 0;
        response[17] = motor_status.paired ? 1 : 0;
        response[18] = motor_status.channel;
        response[19] = motor_status.receiver_battery;
        response_length = 20;
        break;
    }

    default:
        response[0] = STATUS_BAD_REQUEST;
        break;
    }

    if (gate_taken) {
        xSemaphoreGive(s_control_gate);
    }

    send_response(
        sock, source, wire_header.command, sequence, response, response_length);
}

void control_task(void *)
{
    int sock = -1;
    uint8_t datagram[CONTROL_DATAGRAM_MAX];
    bool startup_announced = false;

    for (;;) {
        if (sock < 0) {
            sock = create_control_socket();
            if (sock < 0) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            ESP_LOGI(TAG, "Control server listening on UDP %u", aicam::CONTROL_PORT);
            if (!startup_announced && s_control_ready_signal) {
                startup_announced = true;
                xSemaphoreGive(s_control_ready_signal);
            }
        }

        sockaddr_in source{};
        socklen_t source_length = sizeof(source);
        const ssize_t received = recvfrom(
            sock,
            datagram,
            sizeof(datagram),
            0,
            reinterpret_cast<sockaddr *>(&source),
            &source_length);
        if (received > 0) {
            // Only one station can join the AP. Once HELLO identifies the
            // controller, ignore control traffic spoofed from another IP.
            if (s_controller_ip_be == 0 ||
                s_controller_ip_be == source.sin_addr.s_addr ||
                (received >= static_cast<ssize_t>(sizeof(aicam::ControlHeader)) &&
                 datagram[5] == aicam::CMD_HELLO)) {
                handle_control_datagram(
                    sock, datagram, static_cast<size_t>(received), source);
            }
        } else if (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            ESP_LOGW(TAG, "control recvfrom() failed: errno=%d; reopening socket", errno);
            close(sock);
            sock = -1;
        }

        aicam::motor_bridge_poll_failsafe();
        if (s_motor_stop_requested) {
            s_motor_stop_requested = false;
            stop_motors();
        }
    }
}

esp_err_t init_pair_button()
{
    const gpio_num_t pin =
        static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_PAIR_BUTTON_GPIO);
    const gpio_num_t red_led =
        static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_RED_LED_GPIO);
    const gpio_num_t green_led =
        static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_GREEN_LED_GPIO);
    const gpio_num_t board_led =
        static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_BOARD_LED_GPIO);
    if (!GPIO_IS_VALID_GPIO(pin) || aicam::board_camera_uses_gpio(pin) ||
        pin == red_led || pin == green_led || pin == board_led ||
        pin == aicam::BOARD_SPEAKER_BCLK_GPIO ||
        pin == aicam::BOARD_SPEAKER_DATA_GPIO) {
        ESP_LOGE(TAG, "Unsafe or conflicting pairing button GPIO configuration");
        return ESP_ERR_INVALID_ARG;
    }
    gpio_hold_dis(pin);
    gpio_config_t config{};
    config.pin_bit_mask = 1ULL << pin;
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_ENABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    return gpio_config(&config);
}

esp_err_t init_power_button()
{
    const gpio_num_t pin =
        static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_POWER_BUTTON_GPIO);
    const gpio_num_t pair_button =
        static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_PAIR_BUTTON_GPIO);
    const gpio_num_t red_led =
        static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_RED_LED_GPIO);
    const gpio_num_t green_led =
        static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_GREEN_LED_GPIO);
    const gpio_num_t board_led =
        static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_BOARD_LED_GPIO);
    if (!GPIO_IS_VALID_GPIO(pin) || !esp_sleep_is_valid_wakeup_gpio(pin) ||
        aicam::board_camera_uses_gpio(pin) || pin == pair_button ||
        pin == red_led || pin == green_led || pin == board_led ||
        pin == aicam::BOARD_SPEAKER_BCLK_GPIO ||
        pin == aicam::BOARD_SPEAKER_DATA_GPIO) {
        ESP_LOGE(TAG, "Unsafe, conflicting, or non-wakeup power button GPIO");
        return ESP_ERR_INVALID_ARG;
    }

    gpio_hold_dis(pin);
    gpio_config_t config{};
    config.pin_bit_mask = 1ULL << pin;
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_ENABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    return gpio_config(&config);
}

enum class SleepReason {
    PowerButton,
    VideoIdle,
};

esp_err_t enter_deep_sleep(SleepReason reason)
{
    if (!s_control_gate ||
        xSemaphoreTake(s_control_gate, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    if ((reason == SleepReason::VideoIdle && s_pairing_boot) ||
        s_pairing_transition || s_sleep_transition ||
        aicam::ota_server_update_in_progress() || running_image_is_pending_verify()) {
        xSemaphoreGive(s_control_gate);
        ESP_LOGW(TAG, "Deep sleep deferred during pairing, OTA, or image validation");
        return ESP_ERR_INVALID_STATE;
    }

    s_sleep_transition = true;
    const bool stream_was_enabled = aicam::camera_stream_is_enabled();
    const bool motors_were_running = aicam::motor_bridge_motors_running();
    const int64_t shutdown_started_us = esp_timer_get_time();
    ESP_LOGI(TAG, "Entering deep sleep: %s",
             reason == SleepReason::PowerButton ? "power button" : "video inactive");
    aicam::status_led_show_shutdown();
    aicam::camera_stream_enable(false);

    const esp_err_t stop_result = stop_motors();
    if (stop_result != ESP_OK && motors_were_running) {
        // The legacy receiver does not provide a sufficiently fast independent
        // motor timeout. Never sleep while a running motor may have missed STOP.
        ESP_LOGE(TAG, "Deep sleep cancelled: motor STOP could not be queued");
        if (stream_was_enabled) {
            aicam::camera_stream_enable(true);
        }
        s_sleep_transition = false;
        xSemaphoreGive(s_control_gate);
        aicam::status_led_show_error();
        return stop_result;
    }
    if (stop_result != ESP_OK) {
        ESP_LOGW(TAG, "Previous peer did not accept STOP; no motor was tracked as running");
    }
    vTaskDelay(pdMS_TO_TICKS(30));

    const esp_err_t camera_result = aicam::camera_stream_shutdown(5000);
    if (camera_result != ESP_OK) {
        ESP_LOGE(TAG, "Safe camera shutdown failed: %s; rebooting",
                 esp_err_to_name(camera_result));
        aicam::status_led_show_error();
        vTaskDelay(pdMS_TO_TICKS(1600));
        esp_restart();
        return camera_result;
    }

    constexpr int64_t SHUTDOWN_ANIMATION_US = 650000;
    const int64_t animation_remaining_us =
        SHUTDOWN_ANIMATION_US - (esp_timer_get_time() - shutdown_started_us);
    if (animation_remaining_us > 0) {
        vTaskDelay(pdMS_TO_TICKS(
            static_cast<uint32_t>((animation_remaining_us + 999) / 1000)));
    }

    const gpio_num_t power_button =
        static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_POWER_BUTTON_GPIO);
    esp_err_t result = esp_sleep_enable_ext1_wakeup_io(
        1ULL << power_button, ESP_EXT1_WAKEUP_ANY_LOW);
    if (result == ESP_OK) {
        result = rtc_gpio_pulldown_dis(power_button);
    }
    if (result == ESP_OK) {
        result = rtc_gpio_pullup_en(power_button);
    }
    if (result == ESP_OK) {
        result = aicam::status_led_prepare_deep_sleep();
    }
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Deep-sleep preparation failed: %s; rebooting",
                 esp_err_to_name(result));
        vTaskDelay(pdMS_TO_TICKS(250));
        esp_restart();
        return result;
    }

    gpio_deep_sleep_hold_en();
    ESP_LOGI(TAG, "Power is off; press GPIO%d power button to wake",
             CONFIG_AICAM_STREAM_POWER_BUTTON_GPIO);
    esp_deep_sleep_start();
    return ESP_FAIL;
}

void power_manager_task(void *)
{
    const gpio_num_t pin =
        static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_POWER_BUTTON_GPIO);
    bool pressed = gpio_get_level(pin) == 0;
    bool long_press_reached = false;
    int64_t pressed_at_us = pressed ? esp_timer_get_time() : 0;
    int64_t last_video_activity_us = esp_timer_get_time();
    int64_t observed_frame_sent_us = 0;

    ESP_LOGI(TAG, "Power button GPIO%d: hold %u ms then release to turn off",
             pin, CONFIG_AICAM_STREAM_POWER_BUTTON_HOLD_MS);
#if CONFIG_AICAM_STREAM_AUTO_SLEEP_ENABLED
    ESP_LOGI(TAG, "Automatic sleep after %u ms without transmitted video",
             CONFIG_AICAM_STREAM_AUTO_SLEEP_TIMEOUT_MS);
#endif

    for (;;) {
        const int64_t now_us = esp_timer_get_time();
        const bool active = gpio_get_level(pin) == 0;
        if (active && !pressed) {
            pressed = true;
            long_press_reached = false;
            pressed_at_us = now_us;
        } else if (active && !long_press_reached &&
                   now_us - pressed_at_us >=
                       static_cast<int64_t>(CONFIG_AICAM_STREAM_POWER_BUTTON_HOLD_MS) *
                           1000LL) {
            long_press_reached = true;
            ESP_LOGI(TAG, "Power-off hold accepted; release the button to turn off");
        } else if (!active && pressed) {
            pressed = false;
            if (long_press_reached) {
                const esp_err_t result = enter_deep_sleep(SleepReason::PowerButton);
                if (result != ESP_OK) {
                    ESP_LOGW(TAG, "Power-off request was deferred: %s",
                             esp_err_to_name(result));
                    aicam::status_led_show_error();
                    last_video_activity_us = now_us;
                }
            }
            long_press_reached = false;
        }

        aicam::StreamStats stats{};
        aicam::camera_stream_get_stats(&stats);
        if (stats.last_frame_sent_us > observed_frame_sent_us) {
            observed_frame_sent_us = stats.last_frame_sent_us;
            last_video_activity_us = stats.last_frame_sent_us;
        }

#if CONFIG_AICAM_STREAM_AUTO_SLEEP_ENABLED
        const bool transition_active =
            s_pairing_boot || s_pairing_transition || s_sleep_transition ||
            aicam::ota_server_update_in_progress();
        if (transition_active) {
            last_video_activity_us = now_us;
        } else if (!active &&
                   now_us - last_video_activity_us >=
                       static_cast<int64_t>(CONFIG_AICAM_STREAM_AUTO_SLEEP_TIMEOUT_MS) *
                           1000LL) {
            const esp_err_t result = enter_deep_sleep(SleepReason::VideoIdle);
            if (result != ESP_OK) {
                ESP_LOGW(TAG, "Automatic sleep was deferred: %s",
                         esp_err_to_name(result));
                last_video_activity_us = now_us;
            }
        }
#endif
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

struct PairingPowerButtonMonitor {
    bool pressed;
    bool long_press_reached;
    bool shutdown_requested;
    int64_t pressed_at_us;
};

bool pairing_power_button_cancel(void *context)
{
    auto *monitor = static_cast<PairingPowerButtonMonitor *>(context);
    const gpio_num_t pin =
        static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_POWER_BUTTON_GPIO);
    const int64_t now_us = esp_timer_get_time();
    const bool active = gpio_get_level(pin) == 0;
    if (active && !monitor->pressed) {
        monitor->pressed = true;
        monitor->long_press_reached = false;
        monitor->pressed_at_us = now_us;
    } else if (active && !monitor->long_press_reached &&
               now_us - monitor->pressed_at_us >=
                   static_cast<int64_t>(CONFIG_AICAM_STREAM_POWER_BUTTON_HOLD_MS) *
                       1000LL) {
        monitor->long_press_reached = true;
        ESP_LOGI(TAG, "Pairing power-off hold accepted; release to turn off");
    } else if (!active && monitor->pressed) {
        monitor->pressed = false;
        if (monitor->long_press_reached) {
            monitor->shutdown_requested = true;
            return true;
        }
        monitor->long_press_reached = false;
    }
    return false;
}

void pairing_button_task(void *)
{
    const gpio_num_t pin =
        static_cast<gpio_num_t>(CONFIG_AICAM_STREAM_PAIR_BUTTON_GPIO);
    bool pressed = false;
    bool handled_until_release = false;
    int64_t pressed_at_us = 0;

    for (;;) {
        const bool active = gpio_get_level(pin) == 0;
        if (!active) {
            pressed = false;
            handled_until_release = false;
        } else if (!pressed) {
            pressed = true;
            pressed_at_us = esp_timer_get_time();
        } else if (!handled_until_release &&
                   esp_timer_get_time() - pressed_at_us >=
                       static_cast<int64_t>(CONFIG_AICAM_STREAM_PAIR_BUTTON_HOLD_MS) * 1000LL) {
            handled_until_release = true;
            if (xSemaphoreTake(s_control_gate, pdMS_TO_TICKS(1000)) != pdTRUE) {
                ESP_LOGE(TAG, "Could not enter pairing mode: control gate busy");
                aicam::status_led_show_error();
            } else {
                const esp_err_t request_result = prepare_pairing_restart_locked();
                if (request_result == ESP_OK) {
                    ESP_LOGI(TAG, "Pairing requested; rebooting on channel %u",
                             CONFIG_AICAM_STREAM_PAIRING_CHANNEL);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    esp_restart();
                } else {
                    xSemaphoreGive(s_control_gate);
                    if (request_result == ESP_ERR_INVALID_STATE) {
                        ESP_LOGW(TAG, "Pairing request ignored during OTA activity or validation");
                        continue;
                    }
                    ESP_LOGE(TAG, "Could not request pairing: %s",
                             esp_err_to_name(request_result));
                    aicam::status_led_show_error();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

} // namespace

extern "C" void app_main()
{
    // Legacy firmware can leave output pads latched across deep sleep. Release
    // the global latch first; each subsystem then releases and configures its pins.
    gpio_deep_sleep_hold_dis();
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
        ESP_LOGI(TAG, "Woke from deep sleep by the power button");
    }

    s_control_gate = xSemaphoreCreateMutex();
    require_startup_ok(
        s_control_gate ? ESP_OK : ESP_ERR_NO_MEM, "control/OTA gate creation");
    s_control_ready_signal = xSemaphoreCreateBinary();
    require_startup_ok(
        s_control_ready_signal ? ESP_OK : ESP_ERR_NO_MEM, "control readiness signal creation");
    require_startup_ok(init_nvs(), "NVS initialization");
    require_startup_ok(aicam::status_led_init(), "status LED initialization");
    require_startup_ok(init_pair_button(), "pairing button initialization");
    require_startup_ok(init_power_button(), "power button initialization");

    bool pairing_requested = false;
    require_startup_ok(
        aicam::motor_bridge_consume_pairing_request(&pairing_requested),
        "pairing request load");
    const bool pairing_deferred_for_ota_validation =
        pairing_requested && running_image_is_pending_verify();
    if (pairing_deferred_for_ota_validation) {
        pairing_requested = false;
        ESP_LOGW(TAG, "Deferring pairing until the new OTA image passes its normal boot self-test");
    }
    s_pairing_boot = pairing_requested;
    const uint8_t radio_channel = pairing_requested
        ? CONFIG_AICAM_STREAM_PAIRING_CHANNEL
        : aicam::motor_bridge_startup_channel();

    require_startup_ok(
        init_softap(radio_channel, pairing_requested), "SoftAP initialization");
    require_startup_ok(
        aicam::motor_bridge_init(radio_channel, pairing_requested),
        "ESP-NOW motor bridge initialization");

    if (pairing_requested) {
        aicam::status_led_set_pairing(true);
        ESP_LOGI(TAG, "Pairing mode: press the receiver pairing button within %u ms",
                 CONFIG_AICAM_STREAM_PAIRING_TIMEOUT_MS);
        PairingPowerButtonMonitor power_monitor{};
        power_monitor.pressed =
            gpio_get_level(static_cast<gpio_num_t>(
                CONFIG_AICAM_STREAM_POWER_BUTTON_GPIO)) == 0;
        power_monitor.pressed_at_us =
            power_monitor.pressed ? esp_timer_get_time() : 0;
        const esp_err_t pairing_result =
            aicam::motor_bridge_run_pairing(
                CONFIG_AICAM_STREAM_PAIRING_TIMEOUT_MS,
                pairing_power_button_cancel,
                &power_monitor);
        if (power_monitor.shutdown_requested) {
            const esp_err_t sleep_result = enter_deep_sleep(SleepReason::PowerButton);
            ESP_LOGE(TAG, "Could not power off from pairing mode: %s",
                     esp_err_to_name(sleep_result));
            aicam::status_led_show_error();
            vTaskDelay(pdMS_TO_TICKS(1600));
            esp_restart();
            return;
        }
        if (pairing_result == ESP_OK) {
            aicam::status_led_show_success();
            ESP_LOGI(TAG, "Pairing complete; restarting on the receiver channel");
            vTaskDelay(pdMS_TO_TICKS(900));
        } else {
            aicam::status_led_set_pairing(false);
            aicam::status_led_show_error();
            ESP_LOGW(TAG, "Pairing ended without changing the previous peer: %s",
                     esp_err_to_name(pairing_result));
            vTaskDelay(pdMS_TO_TICKS(1600));
        }
        esp_restart();
        return;
    }

    require_startup_ok(aicam::camera_stream_init(), "camera initialization");
    require_startup_ok(aicam::camera_stream_start(), "camera stream task start");
    require_startup_ok(
        aicam::camera_stream_wait_ready(5000), "video UDP service readiness");

    const BaseType_t created = xTaskCreatePinnedToCore(
        control_task,
        "stream_control",
        6 * 1024,
        nullptr,
        8,
        nullptr,
        0);
    if (created != pdPASS) {
        fail_startup_and_rollback("control task start", ESP_ERR_NO_MEM);
    }
    if (xSemaphoreTake(s_control_ready_signal, pdMS_TO_TICKS(5000)) != pdTRUE) {
        fail_startup_and_rollback("control UDP service readiness", ESP_ERR_TIMEOUT);
    }

#if CONFIG_AICAM_OTA_ENABLED
    if (strlen(CONFIG_AICAM_STREAM_WIFI_PASSWORD) == 0) {
        ESP_LOGW(TAG, "OTA disabled because the SoftAP is open; configure WPA2 first");
    } else {
        aicam::OtaServerConfig ota_config{};
        ota_config.port = CONFIG_AICAM_OTA_PORT;
        ota_config.ota_key = CONFIG_AICAM_OTA_KEY;
        ota_config.prepare_callback = prepare_for_ota;
        ota_config.completion_callback = ota_completed;
        require_startup_ok(aicam::ota_server_start(ota_config), "OTA server start");
    }
#endif

    const BaseType_t pair_button_created = xTaskCreatePinnedToCore(
        pairing_button_task,
        "pair_button",
        3072,
        nullptr,
        3,
        nullptr,
        0);
    require_startup_ok(
        pair_button_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM,
        "pairing button task start");

    // An OTA candidate is confirmed only after all enabled runtime services
    // above have initialized successfully. Earlier crashes trigger rollback.
    confirm_running_image();

    if (pairing_deferred_for_ota_validation) {
        const bool gate_taken =
            xSemaphoreTake(s_control_gate, pdMS_TO_TICKS(1000)) == pdTRUE;
        const esp_err_t request_result = gate_taken
            ? prepare_pairing_restart_locked()
            : ESP_ERR_TIMEOUT;
        if (gate_taken && request_result == ESP_OK) {
            ESP_LOGI(TAG, "OTA image validated; restarting into deferred pairing mode");
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
        } else {
            if (gate_taken) {
                xSemaphoreGive(s_control_gate);
            }
            ESP_LOGE(TAG, "Could not restore deferred pairing request: %s",
                     esp_err_to_name(request_result));
            aicam::status_led_show_error();
        }
    }

    const BaseType_t power_manager_created = xTaskCreatePinnedToCore(
        power_manager_task,
        "power_manager",
        4096,
        nullptr,
        3,
        nullptr,
        0);
    require_startup_ok(
        power_manager_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM,
        "power manager task start");

    ESP_LOGI(TAG, "Ready. Connect to the AP and send HELLO to 192.168.4.1:%u",
             aicam::CONTROL_PORT);
}
