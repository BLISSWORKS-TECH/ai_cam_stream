#pragma once

#include <cstdint>
#include "esp_err.h"

namespace aicam {

using PairingCancelCallback = bool (*)(void *context);

struct MotorBridgeStatus {
    bool paired;
    bool linked;
    uint8_t channel;
    uint8_t peer_mac[6];
    uint8_t receiver_action;
    uint8_t remote_battery;
    uint8_t receiver_battery;
    uint16_t receiver_firmware_x100;
    uint32_t heartbeat_age_ms;
};

esp_err_t motor_bridge_consume_pairing_request(bool *requested);
esp_err_t motor_bridge_request_pairing();
uint8_t motor_bridge_startup_channel();
esp_err_t motor_bridge_init(uint8_t radio_channel, bool pairing_mode);
esp_err_t motor_bridge_run_pairing(
    uint32_t timeout_ms,
    PairingCancelCallback cancel_callback = nullptr,
    void *cancel_context = nullptr);
esp_err_t motor_bridge_set_peer(const uint8_t mac[6], uint8_t channel, bool persist);
esp_err_t motor_bridge_send(const uint8_t motor_payload[6]);
bool motor_bridge_motors_running();
void motor_bridge_poll_failsafe();
void motor_bridge_get_status(MotorBridgeStatus *status);

} // namespace aicam
