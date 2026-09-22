#include "espnow_protocol.hpp"

#include <cstring>

namespace aicam {
namespace {

constexpr uint8_t HEADER_1 = 0x54;
constexpr uint8_t HEADER_2 = 0x55;
constexpr uint8_t INFO_SENSOR = 0x01;
constexpr uint8_t INFO_ADVERTISE = 0xE2;
constexpr uint8_t INFO_ACK = 0xE3;
constexpr uint8_t LEGACY_DATA_LENGTH = 4;
constexpr uint8_t DEV_RECEIVER = 0;
constexpr uint8_t DEV_REMOCON = 1;
constexpr uint8_t DEV_CAM = 2;
constexpr uint8_t STAT_READY = 0;
constexpr uint8_t DELIMITER = 0x24;

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

} // namespace

bool parse_pair_advertisement(
    const uint8_t *data, size_t length, PairAdvertisement *advertisement)
{
    if (!data || !advertisement || length != ESPNOW_PAIR_PACKET_BYTES ||
        data[0] != HEADER_1 || data[1] != HEADER_2 ||
        data[2] != INFO_ADVERTISE || data[3] != LEGACY_DATA_LENGTH ||
        data[4] != DEV_RECEIVER || data[5] != DEV_REMOCON ||
        data[7] < 1 || data[7] > 13 ||
        data[16] != DELIMITER || data[17] != DELIMITER ||
        !is_unicast_mac(data + 10)) {
        return false;
    }

    memcpy(advertisement->receiver_mac, data + 10, sizeof(advertisement->receiver_mac));
    advertisement->operational_channel = data[7];
    advertisement->token =
        static_cast<uint16_t>(data[8]) |
        (static_cast<uint16_t>(data[9]) << 8U);
    return true;
}

bool parse_receiver_heartbeat(
    const uint8_t *data, size_t length, ReceiverHeartbeat *heartbeat)
{
    // The legacy receiver keeps TO=DEV_REMOCON even for the copy sent to a
    // camera, so that value is intentionally accepted here.
    if (!data || !heartbeat || length != ESPNOW_HEARTBEAT_PACKET_BYTES ||
        data[0] != HEADER_1 || data[1] != HEADER_2 ||
        data[2] != INFO_SENSOR || data[3] != LEGACY_DATA_LENGTH ||
        data[4] != DEV_RECEIVER || data[5] != DEV_REMOCON) {
        return false;
    }

    heartbeat->status = data[6];
    heartbeat->action = data[7];
    heartbeat->remote_battery = data[8];
    heartbeat->receiver_battery = data[9];
    heartbeat->firmware_x100 =
        static_cast<uint16_t>(data[10]) |
        (static_cast<uint16_t>(data[11]) << 8U);
    return true;
}

void build_pair_ack(
    uint8_t operational_channel,
    uint16_t token,
    const uint8_t camera_ap_mac[6],
    uint8_t output[ESPNOW_PAIR_PACKET_BYTES])
{
    memset(output, 0, ESPNOW_PAIR_PACKET_BYTES);
    output[0] = HEADER_1;
    output[1] = HEADER_2;
    output[2] = INFO_ACK;
    output[3] = LEGACY_DATA_LENGTH;
    output[4] = DEV_CAM;
    output[5] = DEV_RECEIVER;
    output[6] = STAT_READY;
    output[7] = operational_channel;
    output[8] = static_cast<uint8_t>(token & 0xFFU);
    output[9] = static_cast<uint8_t>((token >> 8U) & 0xFFU);
    memcpy(output + 10, camera_ap_mac, 6);
    output[16] = DELIMITER;
    output[17] = DELIMITER;
}

} // namespace aicam
