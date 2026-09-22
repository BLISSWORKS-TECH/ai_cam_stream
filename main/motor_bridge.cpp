#include "motor_bridge.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "espnow_protocol.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "sdkconfig.h"
#include "status_led.hpp"

namespace aicam {
namespace {

constexpr char TAG[] = "motor_bridge";
constexpr char NVS_NAMESPACE[] = "stream";
constexpr char NVS_PEER_KEY[] = "motor_peer";
constexpr char NVS_PAIR_PENDING_KEY[] = "pair_pending";
constexpr size_t MOTOR_PACKET_BYTES = 17;
constexpr uint32_t SEND_RESULT_TIMEOUT_MS = 250;
constexpr uint8_t BROADCAST_MAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

struct PersistedPeer {
    uint8_t mac[6];
    uint8_t channel;
};

struct SendResult {
    uint8_t destination[6];
    esp_now_send_status_t status;
};

uint8_t s_peer_mac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t s_radio_channel = CONFIG_AICAM_STREAM_WIFI_CHANNEL;
portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
int64_t s_last_motor_command_us = 0;
int64_t s_last_heartbeat_us = 0;
bool s_motor_running = false;
bool s_failsafe_sent = true;
uint32_t s_motor_state_generation = 0;
bool s_pairing_mode = false;
bool s_peer_is_paired = false;
bool s_last_reported_link = false;
ReceiverHeartbeat s_heartbeat{};
QueueHandle_t s_pair_advertisement_queue = nullptr;
QueueHandle_t s_send_result_queue = nullptr;
SemaphoreHandle_t s_send_mutex = nullptr;
bool s_send_callback_pending = false; // guarded by s_send_mutex

bool mac_equal(const uint8_t left[6], const uint8_t right[6])
{
    return memcmp(left, right, 6) == 0;
}

bool is_broadcast_mac(const uint8_t mac[6])
{
    return mac_equal(mac, BROADCAST_MAC);
}

bool is_unicast_mac(const uint8_t mac[6])
{
    if ((mac[0] & 0x01U) != 0) {
        return false;
    }
    uint8_t combined = 0;
    for (size_t index = 0; index < 6; ++index) {
        combined |= mac[index];
    }
    return combined != 0;
}

bool valid_channel(uint8_t channel)
{
    return channel >= 1 && channel <= 13;
}

bool valid_persisted_peer(const PersistedPeer &peer)
{
    return valid_channel(peer.channel) && is_unicast_mac(peer.mac);
}

bool parse_mac(const char *text, uint8_t mac[6])
{
    unsigned values[6]{};
    if (!text || std::sscanf(text, "%x:%x:%x:%x:%x:%x",
                             &values[0], &values[1], &values[2],
                             &values[3], &values[4], &values[5]) != 6) {
        return false;
    }
    for (size_t i = 0; i < 6; ++i) {
        if (values[i] > 0xffU) {
            return false;
        }
        mac[i] = static_cast<uint8_t>(values[i]);
    }
    return is_unicast_mac(mac) || is_broadcast_mac(mac);
}

esp_err_t save_peer(const uint8_t mac[6], uint8_t channel)
{
    if (!is_unicast_mac(mac) || !valid_channel(channel)) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    PersistedPeer peer{};
    memcpy(peer.mac, mac, sizeof(peer.mac));
    peer.channel = channel;
    err = nvs_set_blob(handle, NVS_PEER_KEY, &peer, sizeof(peer));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t erase_saved_peer()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_erase_key(handle, NVS_PEER_KEY);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    } else if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

bool load_peer(PersistedPeer *peer)
{
    if (!peer) {
        return false;
    }
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    size_t size = sizeof(*peer);
    const esp_err_t err = nvs_get_blob(handle, NVS_PEER_KEY, peer, &size);
    nvs_close(handle);
    return err == ESP_OK && size == sizeof(*peer) && valid_persisted_peer(*peer);
}

PersistedPeer configured_peer()
{
    PersistedPeer peer{};
    if (load_peer(&peer)) {
        return peer;
    }
    if (!parse_mac(CONFIG_AICAM_STREAM_RECEIVER_MAC, peer.mac)) {
        memcpy(peer.mac, BROADCAST_MAC, sizeof(peer.mac));
    }
    peer.channel = CONFIG_AICAM_STREAM_WIFI_CHANNEL;
    return peer;
}

esp_err_t set_pairing_pending(uint8_t value)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u8(handle, NVS_PAIR_PENDING_KEY, value);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t add_or_modify_peer(const uint8_t mac[6], uint8_t channel)
{
    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, mac, sizeof(peer.peer_addr));
    peer.channel = channel;
    peer.ifidx = WIFI_IF_AP;
    peer.encrypt = false;
    if (esp_now_is_peer_exist(mac)) {
        return esp_now_mod_peer(&peer);
    }
    return esp_now_add_peer(&peer);
}

void send_callback(const uint8_t *destination, esp_now_send_status_t status)
{
    if (!destination || !s_send_result_queue) {
        return;
    }
    SendResult result{};
    memcpy(result.destination, destination, sizeof(result.destination));
    result.status = status;
    // This callback runs in the Wi-Fi task. Never block it; normal operation
    // has at most one in-flight packet because the send mutex serializes sends.
    xQueueSend(s_send_result_queue, &result, 0);
}

void receive_callback(
    const esp_now_recv_info_t *receive_info, const uint8_t *data, int length)
{
    if (!receive_info || !data || length <= 0) {
        return;
    }

    if (s_pairing_mode) {
        PairAdvertisement advertisement{};
        if (s_pair_advertisement_queue &&
            parse_pair_advertisement(data, static_cast<size_t>(length), &advertisement) &&
            mac_equal(receive_info->src_addr, advertisement.receiver_mac)) {
            // The callback runs in the Wi-Fi task, not an ISR. Keep only the
            // latest valid advertisement and do all NVS/delay work elsewhere.
            xQueueOverwrite(s_pair_advertisement_queue, &advertisement);
        }
        return;
    }

    uint8_t expected_mac[6]{};
    bool paired = false;
    portENTER_CRITICAL(&s_state_lock);
    memcpy(expected_mac, s_peer_mac, sizeof(expected_mac));
    paired = s_peer_is_paired;
    portEXIT_CRITICAL(&s_state_lock);
    if (!paired || !mac_equal(receive_info->src_addr, expected_mac)) {
        return;
    }

    ReceiverHeartbeat heartbeat{};
    if (!parse_receiver_heartbeat(data, static_cast<size_t>(length), &heartbeat)) {
        return;
    }
    portENTER_CRITICAL(&s_state_lock);
    s_heartbeat = heartbeat;
    s_last_heartbeat_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_state_lock);
}

void build_motor_packet(const uint8_t payload[6], uint8_t packet[MOTOR_PACKET_BYTES])
{
    memset(packet, 0, MOTOR_PACKET_BYTES);
    packet[0] = 0x54;
    packet[1] = 0x55;
    packet[2] = 0x00;
    packet[3] = 9;
    packet[4] = 0x01; // MOTOR_APP_REMOCON
    packet[5] = 0;
    packet[6] = 0;

    // Input order: blue dir/speed, green dir/speed, red dir/speed.
    packet[7] = payload[0];
    packet[8] = payload[2];
    packet[9] = payload[4];
    packet[10] = payload[1];
    packet[11] = payload[3];
    packet[12] = payload[5];
    packet[15] = 0x24;
    packet[16] = 0x24;
}

esp_err_t send_confirmed_locked(
    const uint8_t destination[6], const uint8_t *packet, size_t length)
{
    SendResult result{};
    if (s_send_callback_pending) {
        // Do not start a new packet until the callback belonging to the timed-out
        // packet is consumed. ESP-NOW callbacks have no sequence ID, so allowing
        // two logical generations would let a late MOVE result acknowledge STOP.
        if (xQueueReceive(
                s_send_result_queue,
                &result,
                pdMS_TO_TICKS(SEND_RESULT_TIMEOUT_MS)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        s_send_callback_pending = false;
    }

    // The queue should now be empty; discard only impossible duplicate/stale
    // entries before establishing the next single outstanding send.
    while (xQueueReceive(s_send_result_queue, &result, 0) == pdTRUE) {
    }

    esp_err_t outcome = esp_now_send(destination, packet, length);
    if (outcome == ESP_OK) {
        s_send_callback_pending = true;
        const TickType_t started = xTaskGetTickCount();
        const TickType_t timeout = pdMS_TO_TICKS(SEND_RESULT_TIMEOUT_MS);
        outcome = ESP_ERR_TIMEOUT;
        for (;;) {
            const TickType_t elapsed = xTaskGetTickCount() - started;
            if (elapsed >= timeout ||
                xQueueReceive(s_send_result_queue, &result, timeout - elapsed) != pdTRUE) {
                break;
            }
            if (mac_equal(result.destination, destination)) {
                s_send_callback_pending = false;
                outcome = result.status == ESP_NOW_SEND_SUCCESS ? ESP_OK : ESP_FAIL;
                break;
            }
            // Only one send is outstanding; an unexpected destination is still
            // its callback and must be consumed without falsely reporting success.
            s_send_callback_pending = false;
            outcome = ESP_FAIL;
            break;
        }
    }

    return outcome;
}

esp_err_t send_repeated_confirmed(
    const uint8_t destination[6],
    const uint8_t *packet,
    size_t length,
    int repeats,
    uint32_t interval_ms,
    int *confirmed_count = nullptr)
{
    if (confirmed_count) {
        *confirmed_count = 0;
    }
    if (!s_send_mutex || !s_send_result_queue || repeats <= 0 ||
        xSemaphoreTake(s_send_mutex, pdMS_TO_TICKS(SEND_RESULT_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    int confirmed = 0;
    esp_err_t last_error = ESP_FAIL;
    for (int attempt = 0; attempt < repeats; ++attempt) {
        last_error = send_confirmed_locked(destination, packet, length);
        if (last_error == ESP_OK) {
            ++confirmed;
        }
        if (attempt + 1 < repeats && interval_ms != 0) {
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
        }
    }
    xSemaphoreGive(s_send_mutex);
    if (confirmed_count) {
        *confirmed_count = confirmed;
    }
    return confirmed > 0 ? ESP_OK : last_error;
}

esp_err_t send_packet(const uint8_t packet[MOTOR_PACKET_BYTES], int repeats)
{
    uint8_t destination[6]{};
    portENTER_CRITICAL(&s_state_lock);
    memcpy(destination, s_peer_mac, sizeof(destination));
    portEXIT_CRITICAL(&s_state_lock);

    const esp_err_t result = send_repeated_confirmed(
        destination, packet, MOTOR_PACKET_BYTES, repeats, 10);
    if (result == ESP_OK) {
        // This is the common callback-confirmed path for MOVE and every STOP
        // packet source (normal command, peer change, failsafe and shutdown).
        status_led_notify_motor_sent();
    }
    return result;
}

esp_err_t send_stop(int repeats)
{
    const uint8_t stop_payload[6] = {3, 0, 3, 0, 3, 0};
    uint8_t packet[MOTOR_PACKET_BYTES];
    build_motor_packet(stop_payload, packet);
    return send_packet(packet, repeats);
}

} // namespace

esp_err_t motor_bridge_consume_pairing_request(bool *requested)
{
    if (!requested) {
        return ESP_ERR_INVALID_ARG;
    }
    *requested = false;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t value = 0;
    err = nvs_get_u8(handle, NVS_PAIR_PENDING_KEY, &value);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    } else if (err == ESP_OK) {
        *requested = value != 0;
        const esp_err_t erase_result = nvs_erase_key(handle, NVS_PAIR_PENDING_KEY);
        if (erase_result != ESP_OK && erase_result != ESP_ERR_NVS_NOT_FOUND) {
            err = erase_result;
        } else {
            err = nvs_commit(handle);
        }
    }
    nvs_close(handle);
    return err;
}

esp_err_t motor_bridge_request_pairing()
{
    return set_pairing_pending(1);
}

uint8_t motor_bridge_startup_channel()
{
    return configured_peer().channel;
}

esp_err_t motor_bridge_init(uint8_t radio_channel, bool pairing_mode)
{
    if (!valid_channel(radio_channel)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_radio_channel = radio_channel;
    s_pairing_mode = pairing_mode;
    s_send_result_queue = xQueueCreate(4, sizeof(SendResult));
    s_send_mutex = xSemaphoreCreateMutex();
    if (!s_send_result_queue || !s_send_mutex) {
        return ESP_ERR_NO_MEM;
    }
    if (pairing_mode) {
        s_pair_advertisement_queue = xQueueCreate(1, sizeof(PairAdvertisement));
        if (!s_pair_advertisement_queue) {
            return ESP_ERR_NO_MEM;
        }
    }

    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_RETURN_ON_ERROR(esp_now_register_send_cb(send_callback), TAG, "send callback failed");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(receive_callback), TAG, "receive callback failed");

    constexpr const char *pmk = CONFIG_AICAM_STREAM_ESPNOW_PMK;
    if (strlen(pmk) == ESP_NOW_KEY_LEN) {
        err = esp_now_set_pmk(reinterpret_cast<const uint8_t *>(pmk));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_now_set_pmk failed: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGW(TAG, "ESP-NOW PMK must contain exactly %d bytes", ESP_NOW_KEY_LEN);
    }

    PersistedPeer peer{};
    if (pairing_mode) {
        memcpy(peer.mac, BROADCAST_MAC, sizeof(peer.mac));
        peer.channel = radio_channel;
    } else {
        peer = configured_peer();
        if (peer.channel != radio_channel) {
            ESP_LOGE(TAG, "Peer channel %u differs from radio channel %u",
                     peer.channel, radio_channel);
            return ESP_ERR_INVALID_STATE;
        }
    }

    ESP_RETURN_ON_ERROR(add_or_modify_peer(peer.mac, radio_channel), TAG, "peer add failed");
    portENTER_CRITICAL(&s_state_lock);
    memcpy(s_peer_mac, peer.mac, sizeof(s_peer_mac));
    s_peer_is_paired = !is_broadcast_mac(peer.mac);
    s_last_heartbeat_us = 0;
    s_last_reported_link = false;
    portEXIT_CRITICAL(&s_state_lock);
    status_led_set_receiver_linked(false);

    ESP_LOGI(TAG, "%s peer %02X:%02X:%02X:%02X:%02X:%02X channel=%u",
             pairing_mode ? "Pairing" : "Motor",
             peer.mac[0], peer.mac[1], peer.mac[2],
             peer.mac[3], peer.mac[4], peer.mac[5], peer.channel);
    return ESP_OK;
}

esp_err_t motor_bridge_run_pairing(
    uint32_t timeout_ms,
    PairingCancelCallback cancel_callback,
    void *cancel_context)
{
    if (!s_pairing_mode || !s_pair_advertisement_queue || timeout_ms == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    PairAdvertisement advertisement{};
    const int64_t deadline_us =
        esp_timer_get_time() + static_cast<int64_t>(timeout_ms) * 1000LL;
    for (;;) {
        if (cancel_callback && cancel_callback(cancel_context)) {
            return ESP_ERR_INVALID_STATE;
        }
        const int64_t remaining_us = deadline_us - esp_timer_get_time();
        if (remaining_us <= 0) {
            return ESP_ERR_TIMEOUT;
        }
        const uint32_t wait_ms = static_cast<uint32_t>(
            std::min<int64_t>(20, (remaining_us + 999) / 1000));
        if (xQueueReceive(
                s_pair_advertisement_queue,
                &advertisement,
                pdMS_TO_TICKS(wait_ms)) == pdTRUE) {
            break;
        }
    }

    uint8_t camera_ap_mac[6]{};
    ESP_RETURN_ON_ERROR(
        esp_wifi_get_mac(WIFI_IF_AP, camera_ap_mac), TAG, "camera AP MAC read failed");

    PersistedPeer previous_peer{};
    const bool had_previous_peer = load_peer(&previous_peer);

    // Save before acknowledging so a receiver never commits a camera that
    // cannot persist the matching receiver. The legacy protocol has no final
    // confirmation packet, so transmit several callback-confirmed broadcasts.
    ESP_RETURN_ON_ERROR(
        save_peer(advertisement.receiver_mac, advertisement.operational_channel),
        TAG,
        "paired receiver save failed");

    uint8_t ack[ESPNOW_PAIR_PACKET_BYTES]{};
    build_pair_ack(
        advertisement.operational_channel,
        advertisement.token,
        camera_ap_mac,
        ack);
    int transmitted = 0;
    const esp_err_t last_error = send_repeated_confirmed(
        BROADCAST_MAC, ack, sizeof(ack), 5, 50, &transmitted);
    if (transmitted == 0) {
        const esp_err_t rollback_result = had_previous_peer
            ? save_peer(previous_peer.mac, previous_peer.channel)
            : erase_saved_peer();
        if (rollback_result != ESP_OK) {
            ESP_LOGE(TAG, "pairing ACK failed and peer rollback failed: %s",
                     esp_err_to_name(rollback_result));
            return rollback_result;
        }
        if (cancel_callback && cancel_callback(cancel_context)) {
            return ESP_ERR_INVALID_STATE;
        }
        return last_error;
    }

    if (cancel_callback && cancel_callback(cancel_context)) {
        ESP_LOGI(TAG, "Power-off requested after pairing data was safely committed");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Paired receiver %02X:%02X:%02X:%02X:%02X:%02X; operational channel=%u",
             advertisement.receiver_mac[0], advertisement.receiver_mac[1],
             advertisement.receiver_mac[2], advertisement.receiver_mac[3],
             advertisement.receiver_mac[4], advertisement.receiver_mac[5],
             advertisement.operational_channel);
    return ESP_OK;
}

esp_err_t motor_bridge_set_peer(const uint8_t mac[6], uint8_t channel, bool persist)
{
    if (!mac || channel != s_radio_channel ||
        (!is_unicast_mac(mac) && !is_broadcast_mac(mac))) {
        // SoftAP and ESP-NOW share one radio. Changing channels at runtime
        // would disconnect the video client, so a different channel requires
        // the physical pairing/reboot flow.
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t previous_mac[6]{};
    bool motor_running = false;
    portENTER_CRITICAL(&s_state_lock);
    memcpy(previous_mac, s_peer_mac, sizeof(previous_mac));
    motor_running = s_motor_running;
    portEXIT_CRITICAL(&s_state_lock);

    if (motor_running) {
        const esp_err_t stop_result = send_stop(3);
        if (stop_result != ESP_OK) {
            ESP_LOGE(TAG, "refusing peer change because old-peer STOP failed: %s",
                     esp_err_to_name(stop_result));
            return stop_result;
        }
        portENTER_CRITICAL(&s_state_lock);
        ++s_motor_state_generation;
        s_motor_running = false;
        s_failsafe_sent = true;
        portEXIT_CRITICAL(&s_state_lock);
    }

    esp_err_t err = add_or_modify_peer(mac, channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_now peer update failed: %s", esp_err_to_name(err));
        return err;
    }
    if (persist) {
        err = save_peer(mac, channel);
        if (err != ESP_OK) {
            if (!mac_equal(mac, previous_mac)) {
                esp_now_del_peer(mac);
            }
            ESP_LOGE(TAG, "peer persistence failed: %s", esp_err_to_name(err));
            return err;
        }
    }

    portENTER_CRITICAL(&s_state_lock);
    memcpy(s_peer_mac, mac, sizeof(s_peer_mac));
    s_peer_is_paired = !is_broadcast_mac(mac);
    s_last_heartbeat_us = 0;
    s_last_reported_link = false;
    portEXIT_CRITICAL(&s_state_lock);
    status_led_set_receiver_linked(false);

    if (!mac_equal(previous_mac, mac) && esp_now_is_peer_exist(previous_mac)) {
        esp_now_del_peer(previous_mac);
    }
    return ESP_OK;
}

esp_err_t motor_bridge_send(const uint8_t motor_payload[6])
{
    if (!motor_payload || s_pairing_mode) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t sanitized[6];
    memcpy(sanitized, motor_payload, sizeof(sanitized));
    bool running = false;
    for (size_t motor = 0; motor < 3; ++motor) {
        const size_t dir_index = motor * 2;
        const size_t speed_index = dir_index + 1;
        if (sanitized[dir_index] < 1 || sanitized[dir_index] > 3 ||
            sanitized[speed_index] > 100) {
            return ESP_ERR_INVALID_ARG;
        }
        if (sanitized[dir_index] == 3 || sanitized[speed_index] == 0) {
            sanitized[dir_index] = 3;
            sanitized[speed_index] = 0;
        } else {
            running = true;
        }
    }

    uint8_t packet[MOTOR_PACKET_BYTES];
    build_motor_packet(sanitized, packet);
    const esp_err_t err = send_packet(packet, 1);
    if (err == ESP_OK) {
        portENTER_CRITICAL(&s_state_lock);
        ++s_motor_state_generation;
        s_last_motor_command_us = esp_timer_get_time();
        s_motor_running = running;
        s_failsafe_sent = !running;
        portEXIT_CRITICAL(&s_state_lock);
    }
    return err;
}

bool motor_bridge_motors_running()
{
    portENTER_CRITICAL(&s_state_lock);
    const bool running = s_motor_running;
    portEXIT_CRITICAL(&s_state_lock);
    return running;
}

void motor_bridge_get_status(MotorBridgeStatus *status)
{
    if (!status) {
        return;
    }
    const int64_t now_us = esp_timer_get_time();
    int64_t last_heartbeat_us = 0;
    portENTER_CRITICAL(&s_state_lock);
    status->paired = s_peer_is_paired;
    status->channel = s_radio_channel;
    memcpy(status->peer_mac, s_peer_mac, sizeof(status->peer_mac));
    status->receiver_action = s_heartbeat.action;
    status->remote_battery = s_heartbeat.remote_battery;
    status->receiver_battery = s_heartbeat.receiver_battery;
    status->receiver_firmware_x100 = s_heartbeat.firmware_x100;
    last_heartbeat_us = s_last_heartbeat_us;
    portEXIT_CRITICAL(&s_state_lock);

    const int64_t age_us = last_heartbeat_us > 0 ? now_us - last_heartbeat_us : INT64_MAX;
    status->linked = status->paired && age_us >= 0 &&
        age_us < static_cast<int64_t>(CONFIG_AICAM_STREAM_ESPNOW_LINK_TIMEOUT_MS) * 1000LL;
    status->heartbeat_age_ms = last_heartbeat_us > 0
        ? static_cast<uint32_t>(std::min<int64_t>(age_us / 1000LL, UINT32_MAX))
        : UINT32_MAX;
}

void motor_bridge_poll_failsafe()
{
    const int64_t now_us = esp_timer_get_time();
    bool should_stop = false;
    uint32_t stop_generation = 0;
    portENTER_CRITICAL(&s_state_lock);
    if (s_motor_running && !s_failsafe_sent && s_last_motor_command_us != 0 &&
        now_us - s_last_motor_command_us >=
            static_cast<int64_t>(CONFIG_AICAM_STREAM_MOTOR_FAILSAFE_MS) * 1000LL) {
        stop_generation = s_motor_state_generation;
        // Reserve this timeout while the callback-confirmed STOP is in flight.
        s_failsafe_sent = true;
        should_stop = true;
    }
    portEXIT_CRITICAL(&s_state_lock);

    if (should_stop) {
        ESP_LOGW(TAG, "Motor failsafe timeout; sending STOP");
        const esp_err_t stop_result = send_stop(3);
        portENTER_CRITICAL(&s_state_lock);
        if (s_motor_state_generation == stop_generation) {
            if (stop_result == ESP_OK) {
                s_motor_running = false;
                s_failsafe_sent = true;
            } else {
                // Keep the timed-out running state so the next 50 ms poll retries.
                s_failsafe_sent = false;
            }
        }
        portEXIT_CRITICAL(&s_state_lock);
        if (stop_result != ESP_OK) {
            ESP_LOGE(TAG, "Motor failsafe STOP failed; retrying: %s",
                     esp_err_to_name(stop_result));
        }
    }

    MotorBridgeStatus status{};
    motor_bridge_get_status(&status);
    if (status.linked != s_last_reported_link) {
        s_last_reported_link = status.linked;
        ESP_LOGI(TAG, "Receiver link %s", status.linked ? "connected" : "timed out");
    }
    status_led_set_receiver_linked(status.linked);
}

} // namespace aicam
