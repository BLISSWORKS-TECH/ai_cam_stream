#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

namespace aicam {

inline constexpr uint16_t OTA_DEFAULT_PORT = 8080;
inline constexpr char OTA_UPLOAD_PATH[] = "/ota";
inline constexpr char OTA_KEY_HEADER[] = "X-AICam-OTA-Key";
inline constexpr char OTA_SHA256_HEADER[] = "X-AICam-SHA256";
inline constexpr size_t OTA_KEY_MIN_LENGTH = 8;
inline constexpr size_t OTA_KEY_MAX_LENGTH = 63;

// Called from the HTTP server task after the request has been authenticated and
// checked, but before flash erase/write begins. The application should stop the
// video stream and command all motors to stop here. Returning an error rejects
// the update without touching the OTA partition.
using OtaPrepareCallback = esp_err_t (*)(void *context);

// Called once after a prepare callback was attempted. On failure, applications
// can use this hook to restore services stopped by the prepare callback. On
// success, the selected OTA partition is already the next boot partition.
using OtaCompletionCallback = void (*)(bool success, esp_err_t result, void *context);

struct OtaServerConfig {
    uint16_t port = OTA_DEFAULT_PORT;

    // Required 8..63-character visible-ASCII shared secret. Requests must
    // supply the exact value in X-AICam-OTA-Key. The value is copied.
    const char *ota_key = nullptr;

    OtaPrepareCallback prepare_callback = nullptr;
    OtaCompletionCallback completion_callback = nullptr;
    void *callback_context = nullptr;

    // Absolute request-body deadline. This prevents a stalled uploader from
    // leaving the device indefinitely in an update/stopped state.
    uint32_t upload_timeout_ms = 5 * 60 * 1000;

    // When enabled, a successful response is followed by a delayed reboot.
    bool reboot_on_success = true;
};

// Starts an HTTP server exposing POST /ota. The request body must be the raw
// application .bin, accompanied by X-AICam-OTA-Key and X-AICam-SHA256 headers.
esp_err_t ota_server_start(const OtaServerConfig &config);

// Stops the server. Returns ESP_ERR_INVALID_STATE while an upload is active.
esp_err_t ota_server_stop();

bool ota_server_update_in_progress();

} // namespace aicam
