#pragma once

#include <cstddef>
#include <cstdint>

namespace aicam {

constexpr size_t ESPNOW_PAIR_PACKET_BYTES = 18;
constexpr size_t ESPNOW_HEARTBEAT_PACKET_BYTES = 12;

struct PairAdvertisement {
    uint8_t receiver_mac[6];
    uint8_t operational_channel;
    uint16_t token;
};

struct ReceiverHeartbeat {
    uint8_t status;
    uint8_t action;
    uint8_t remote_battery;
    uint8_t receiver_battery;
    uint16_t firmware_x100;
};

bool parse_pair_advertisement(
    const uint8_t *data, size_t length, PairAdvertisement *advertisement);
bool parse_receiver_heartbeat(
    const uint8_t *data, size_t length, ReceiverHeartbeat *heartbeat);
void build_pair_ack(
    uint8_t operational_channel,
    uint16_t token,
    const uint8_t camera_ap_mac[6],
    uint8_t output[ESPNOW_PAIR_PACKET_BYTES]);

} // namespace aicam
