#pragma once

#include <cstdint>
#include "esp_err.h"
#include "stream_protocol.hpp"

namespace aicam {

struct StreamStats {
    uint32_t frames_sent;
    uint32_t frames_dropped;
    uint32_t packets_sent;
    uint32_t send_errors;
    uint32_t active_profile;
    uint32_t last_frame_bytes;
    int64_t last_frame_sent_us;
};

esp_err_t camera_stream_init();
esp_err_t camera_stream_start();
esp_err_t camera_stream_wait_ready(uint32_t timeout_ms);
esp_err_t camera_stream_shutdown(uint32_t timeout_ms);

void camera_stream_set_client(uint32_t ipv4_network_order, uint16_t port_network_order);
void camera_stream_enable(bool enabled);
bool camera_stream_is_enabled();

esp_err_t camera_stream_request_profile(StreamProfile profile);
StreamProfile camera_stream_active_profile();
void camera_stream_get_stats(StreamStats *out);

} // namespace aicam
