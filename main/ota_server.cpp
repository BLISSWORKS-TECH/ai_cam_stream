#include "ota_server.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>

#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/sha256.h"
#include "sdkconfig.h"

namespace aicam {
namespace {

constexpr char TAG[] = "ota_server";
constexpr size_t RECEIVE_BUFFER_SIZE = 4096;
constexpr size_t SHA256_SIZE = 32;
constexpr size_t SHA256_HEX_SIZE = SHA256_SIZE * 2;
constexpr size_t IMAGE_METADATA_SIZE =
    sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) +
    sizeof(esp_app_desc_t);
constexpr char EXPECTED_PROJECT_NAME[] = "ai_cam_stream";
constexpr uint32_t REBOOT_DELAY_MS = 1000;

portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
httpd_handle_t s_server = nullptr;
bool s_server_starting = false;
bool s_update_in_progress = false;
OtaServerConfig s_config{};
char s_ota_key[OTA_KEY_MAX_LENGTH + 1]{};

esp_err_t send_json(httpd_req_t *request, const char *status, const char *body)
{
    httpd_resp_set_status(request, status);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    if (status[0] != '2') {
        // Do not keep a connection alive when rejecting a request before its
        // potentially large firmware body has been consumed.
        httpd_resp_set_hdr(request, "Connection", "close");
    }
    return httpd_resp_sendstr(request, body);
}

bool constant_time_equal(const uint8_t *left, const uint8_t *right, size_t length)
{
    uint8_t difference = 0;
    for (size_t index = 0; index < length; ++index) {
        difference |= left[index] ^ right[index];
    }
    return difference == 0;
}

bool request_is_authorized(httpd_req_t *request)
{
    const size_t expected_length = strlen(s_ota_key);
    const size_t supplied_length =
        httpd_req_get_hdr_value_len(request, OTA_KEY_HEADER);
    if (supplied_length != expected_length || supplied_length == 0 ||
        supplied_length > OTA_KEY_MAX_LENGTH) {
        return false;
    }

    char supplied_key[OTA_KEY_MAX_LENGTH + 1]{};
    if (httpd_req_get_hdr_value_str(
            request, OTA_KEY_HEADER, supplied_key, sizeof(supplied_key)) != ESP_OK) {
        return false;
    }

    return constant_time_equal(
        reinterpret_cast<const uint8_t *>(supplied_key),
        reinterpret_cast<const uint8_t *>(s_ota_key),
        expected_length);
}

int hex_nibble(char character)
{
    if (character >= '0' && character <= '9') {
        return character - '0';
    }
    const unsigned char lowered = static_cast<unsigned char>(
        std::tolower(static_cast<unsigned char>(character)));
    if (lowered >= 'a' && lowered <= 'f') {
        return lowered - 'a' + 10;
    }
    return -1;
}

bool parse_expected_sha256(httpd_req_t *request, uint8_t output[SHA256_SIZE])
{
    if (httpd_req_get_hdr_value_len(request, OTA_SHA256_HEADER) != SHA256_HEX_SIZE) {
        return false;
    }

    char hexadecimal[SHA256_HEX_SIZE + 1]{};
    if (httpd_req_get_hdr_value_str(
            request, OTA_SHA256_HEADER, hexadecimal, sizeof(hexadecimal)) != ESP_OK) {
        return false;
    }

    for (size_t index = 0; index < SHA256_SIZE; ++index) {
        const int high = hex_nibble(hexadecimal[index * 2]);
        const int low = hex_nibble(hexadecimal[index * 2 + 1]);
        if (high < 0 || low < 0) {
            return false;
        }
        output[index] = static_cast<uint8_t>((high << 4) | low);
    }
    return true;
}

esp_err_t receive_exact_before_deadline(
    httpd_req_t *request,
    uint8_t *output,
    size_t output_size,
    int64_t deadline_us)
{
    size_t offset = 0;
    while (offset != output_size) {
        if (esp_timer_get_time() >= deadline_us) {
            return ESP_ERR_TIMEOUT;
        }
        const int received = httpd_req_recv(
            request,
            reinterpret_cast<char *>(output + offset),
            output_size - offset);
        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (received <= 0) {
            return ESP_FAIL;
        }
        offset += static_cast<size_t>(received);
        if (esp_timer_get_time() >= deadline_us) {
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}

bool validate_image_metadata(const uint8_t metadata[IMAGE_METADATA_SIZE])
{
    esp_image_header_t image_header{};
    memcpy(&image_header, metadata, sizeof(image_header));
    if (image_header.magic != ESP_IMAGE_HEADER_MAGIC) {
        ESP_LOGE(TAG, "Rejected image with invalid ESP image magic");
        return false;
    }
    if (image_header.segment_count == 0 ||
        image_header.segment_count > ESP_IMAGE_MAX_SEGMENTS) {
        ESP_LOGE(TAG, "Rejected image with invalid segment count: %u",
                 image_header.segment_count);
        return false;
    }
    if (image_header.chip_id != CONFIG_IDF_FIRMWARE_CHIP_ID) {
        ESP_LOGE(TAG, "Rejected image for chip ID 0x%04x (expected 0x%04x)",
                 image_header.chip_id, CONFIG_IDF_FIRMWARE_CHIP_ID);
        return false;
    }

    esp_app_desc_t app_description{};
    const size_t description_offset =
        sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t);
    memcpy(&app_description, metadata + description_offset, sizeof(app_description));
    if (app_description.magic_word != ESP_APP_DESC_MAGIC_WORD) {
        ESP_LOGE(TAG, "Rejected image with invalid application descriptor");
        return false;
    }
    if (strncmp(
            app_description.project_name,
            EXPECTED_PROJECT_NAME,
            sizeof(app_description.project_name)) != 0) {
        ESP_LOGE(TAG, "Rejected image for a different project");
        return false;
    }

    ESP_LOGI(TAG, "Incoming firmware version=%.*s, project=%.*s",
             static_cast<int>(sizeof(app_description.version)),
             app_description.version,
             static_cast<int>(sizeof(app_description.project_name)),
             app_description.project_name);
    return true;
}

bool claim_update()
{
    bool claimed = false;
    portENTER_CRITICAL(&s_state_lock);
    if (!s_update_in_progress) {
        s_update_in_progress = true;
        claimed = true;
    }
    portEXIT_CRITICAL(&s_state_lock);
    return claimed;
}

void release_update()
{
    portENTER_CRITICAL(&s_state_lock);
    s_update_in_progress = false;
    portEXIT_CRITICAL(&s_state_lock);
}

void notify_completion(bool success, esp_err_t result)
{
    if (s_config.completion_callback) {
        s_config.completion_callback(success, result, s_config.callback_context);
    }
}

void finish_failed_update(esp_err_t result)
{
    notify_completion(false, result);
    release_update();
}

void reboot_task(void *)
{
    vTaskDelay(pdMS_TO_TICKS(REBOOT_DELAY_MS));
    esp_restart();
}

void schedule_reboot()
{
    if (xTaskCreate(
            reboot_task, "ota_reboot", 2048, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGW(TAG, "Could not create reboot task; rebooting immediately");
        esp_restart();
    }
}

esp_err_t ota_post_handler(httpd_req_t *request)
{
    if (!request_is_authorized(request)) {
        ESP_LOGW(TAG, "Rejected OTA request with invalid key");
        return send_json(
            request, "401 Unauthorized", "{\"ok\":false,\"error\":\"unauthorized\"}");
    }

    uint8_t expected_sha256[SHA256_SIZE]{};
    if (!parse_expected_sha256(request, expected_sha256)) {
        return send_json(
            request,
            "400 Bad Request",
            "{\"ok\":false,\"error\":\"invalid X-AICam-SHA256 header\"}");
    }

    const esp_partition_t *running_partition = esp_ota_get_running_partition();
    esp_ota_img_states_t running_state = ESP_OTA_IMG_UNDEFINED;
    if (running_partition &&
        esp_ota_get_state_partition(running_partition, &running_state) == ESP_OK &&
        running_state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGW(TAG, "Rejected OTA while the running image awaits validation");
        return send_json(
            request,
            "503 Service Unavailable",
            "{\"ok\":false,\"error\":\"running firmware is not yet validated\"}");
    }

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(nullptr);
    if (!update_partition) {
        ESP_LOGE(TAG, "No OTA application partition is available");
        return send_json(
            request,
            "503 Service Unavailable",
            "{\"ok\":false,\"error\":\"no OTA partition\"}");
    }

    if (request->content_len <= 0) {
        return send_json(
            request,
            "411 Length Required",
            "{\"ok\":false,\"error\":\"raw firmware body required\"}");
    }
    const size_t image_size = static_cast<size_t>(request->content_len);
    if (image_size > update_partition->size) {
        return send_json(
            request,
            "413 Content Too Large",
            "{\"ok\":false,\"error\":\"image exceeds OTA partition\"}");
    }
    if (image_size < IMAGE_METADATA_SIZE) {
        return send_json(
            request,
            "400 Bad Request",
            "{\"ok\":false,\"error\":\"firmware image is too small\"}");
    }

    // Inspect enough of the request to reject unrelated or wrong-chip images
    // before stopping services, erasing flash, or opening an OTA handle.
    const int64_t deadline_us =
        esp_timer_get_time() + static_cast<int64_t>(s_config.upload_timeout_ms) * 1000;
    uint8_t image_metadata[IMAGE_METADATA_SIZE]{};
    esp_err_t result = receive_exact_before_deadline(
        request, image_metadata, sizeof(image_metadata), deadline_us);
    if (result != ESP_OK) {
        if (result == ESP_ERR_TIMEOUT) {
            return send_json(
                request,
                "408 Request Timeout",
                "{\"ok\":false,\"error\":\"upload timed out\"}");
        }
        return send_json(
            request,
            "400 Bad Request",
            "{\"ok\":false,\"error\":\"incomplete firmware body\"}");
    }
    if (!validate_image_metadata(image_metadata)) {
        return send_json(
            request,
            "422 Unprocessable Content",
            "{\"ok\":false,\"error\":\"firmware metadata rejected\"}");
    }

    uint8_t *receive_buffer = static_cast<uint8_t *>(malloc(RECEIVE_BUFFER_SIZE));
    if (!receive_buffer) {
        return send_json(
            request,
            "500 Internal Server Error",
            "{\"ok\":false,\"error\":\"out of memory\"}");
    }

    if (!claim_update()) {
        free(receive_buffer);
        return send_json(
            request,
            "409 Conflict",
            "{\"ok\":false,\"error\":\"update already in progress\"}");
    }

    result = ESP_OK;
    if (s_config.prepare_callback) {
        result = s_config.prepare_callback(s_config.callback_context);
    }
    if (result != ESP_OK) {
        free(receive_buffer);
        ESP_LOGE(TAG, "OTA prepare callback failed: %s", esp_err_to_name(result));
        finish_failed_update(result);
        return send_json(
            request,
            "503 Service Unavailable",
            "{\"ok\":false,\"error\":\"device could not prepare for OTA\"}");
    }

    esp_ota_handle_t ota_handle = 0;
    bool ota_handle_open = false;
    result = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (result != ESP_OK) {
        free(receive_buffer);
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(result));
        finish_failed_update(result);
        return send_json(
            request,
            "500 Internal Server Error",
            "{\"ok\":false,\"error\":\"could not begin OTA\"}");
    }
    ota_handle_open = true;

    mbedtls_sha256_context sha256_context;
    mbedtls_sha256_init(&sha256_context);
    if (mbedtls_sha256_starts(&sha256_context, 0) != 0) {
        result = ESP_FAIL;
    }

    if (result == ESP_OK) {
        result = esp_ota_write(ota_handle, image_metadata, sizeof(image_metadata));
    }
    if (result == ESP_OK &&
        mbedtls_sha256_update(
            &sha256_context, image_metadata, sizeof(image_metadata)) != 0) {
        result = ESP_FAIL;
    }

    size_t remaining = image_size - sizeof(image_metadata);
    size_t written = result == ESP_OK ? sizeof(image_metadata) : 0;

    while (result == ESP_OK && remaining != 0) {
        // Check the absolute deadline even while data continues to arrive. A
        // slow byte-by-byte sender therefore cannot keep an OTA slot forever.
        if (esp_timer_get_time() >= deadline_us) {
            result = ESP_ERR_TIMEOUT;
            break;
        }
        const size_t requested = std::min(remaining, RECEIVE_BUFFER_SIZE);
        const int received = httpd_req_recv(
            request, reinterpret_cast<char *>(receive_buffer), requested);
        if (received == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (received <= 0) {
            result = ESP_FAIL;
            break;
        }
        if (esp_timer_get_time() >= deadline_us) {
            result = ESP_ERR_TIMEOUT;
            break;
        }

        result = esp_ota_write(ota_handle, receive_buffer, static_cast<size_t>(received));
        if (result != ESP_OK) {
            break;
        }
        if (mbedtls_sha256_update(
                &sha256_context, receive_buffer, static_cast<size_t>(received)) != 0) {
            result = ESP_FAIL;
            break;
        }
        remaining -= static_cast<size_t>(received);
        written += static_cast<size_t>(received);
    }

    free(receive_buffer);
    receive_buffer = nullptr;

    uint8_t actual_sha256[SHA256_SIZE]{};
    if (result == ESP_OK &&
        mbedtls_sha256_finish(&sha256_context, actual_sha256) != 0) {
        result = ESP_FAIL;
    }
    mbedtls_sha256_free(&sha256_context);

    if (result == ESP_OK &&
        !constant_time_equal(expected_sha256, actual_sha256, SHA256_SIZE)) {
        ESP_LOGE(TAG, "OTA SHA-256 mismatch after %u bytes", static_cast<unsigned>(written));
        result = ESP_ERR_INVALID_CRC;
    }

    if (result != ESP_OK) {
        if (ota_handle_open) {
            esp_ota_abort(ota_handle);
        }
        ESP_LOGE(TAG, "OTA upload failed after %u/%u bytes: %s",
                 static_cast<unsigned>(written),
                 static_cast<unsigned>(image_size),
                 esp_err_to_name(result));
        finish_failed_update(result);
        if (result == ESP_ERR_TIMEOUT) {
            return send_json(
                request,
                "408 Request Timeout",
                "{\"ok\":false,\"error\":\"upload timed out\"}");
        }
        if (result == ESP_ERR_INVALID_CRC) {
            return send_json(
                request,
                "422 Unprocessable Content",
                "{\"ok\":false,\"error\":\"SHA-256 mismatch\"}");
        }
        return send_json(
            request,
            "500 Internal Server Error",
            "{\"ok\":false,\"error\":\"upload or flash write failed\"}");
    }

    // esp_ota_end performs the ESP image structure, checksum and signature
    // validation. The handle is invalid after this call, regardless of result.
    result = esp_ota_end(ota_handle);
    ota_handle_open = false;
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "OTA image validation failed: %s", esp_err_to_name(result));
        finish_failed_update(result);
        return send_json(
            request,
            "422 Unprocessable Content",
            "{\"ok\":false,\"error\":\"invalid firmware image\"}");
    }

    result = esp_ota_set_boot_partition(update_partition);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Could not select OTA boot partition: %s", esp_err_to_name(result));
        finish_failed_update(result);
        return send_json(
            request,
            "500 Internal Server Error",
            "{\"ok\":false,\"error\":\"could not select boot partition\"}");
    }

    ESP_LOGI(TAG, "OTA image accepted: %u bytes, next partition=%s",
             static_cast<unsigned>(written), update_partition->label);
    const bool reboot = s_config.reboot_on_success;
    // Keep the busy flag asserted after success. The application can use it to
    // reject stream/motor commands during the response-to-reboot window (and
    // until a deliberate reboot when automatic reboot is disabled).
    notify_completion(true, ESP_OK);
    const esp_err_t response_result = send_json(
        request,
        "200 OK",
        reboot
            ? "{\"ok\":true,\"message\":\"update accepted; rebooting\"}"
            : "{\"ok\":true,\"message\":\"update accepted\"}");
    if (reboot) {
        schedule_reboot();
    }
    return response_result;
}

} // namespace

esp_err_t ota_server_start(const OtaServerConfig &config)
{
    if (config.port == 0 || config.upload_timeout_ms == 0 || !config.ota_key) {
        return ESP_ERR_INVALID_ARG;
    }
    const size_t key_length = strnlen(config.ota_key, OTA_KEY_MAX_LENGTH + 1);
    if (key_length < OTA_KEY_MIN_LENGTH || key_length > OTA_KEY_MAX_LENGTH) {
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t index = 0; index < key_length; ++index) {
        const unsigned char character =
            static_cast<unsigned char>(config.ota_key[index]);
        if (character < 0x21 || character > 0x7e) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    portENTER_CRITICAL(&s_state_lock);
    if (s_server || s_server_starting) {
        portEXIT_CRITICAL(&s_state_lock);
        return ESP_ERR_INVALID_STATE;
    }
    s_server_starting = true;
    s_config = config;
    memcpy(s_ota_key, config.ota_key, key_length);
    s_ota_key[key_length] = '\0';
    s_config.ota_key = s_ota_key;
    portEXIT_CRITICAL(&s_state_lock);

    httpd_config_t http_config = HTTPD_DEFAULT_CONFIG();
    http_config.server_port = config.port;
    http_config.stack_size = 8 * 1024;
    http_config.max_uri_handlers = 1;
    http_config.max_open_sockets = 2;
    http_config.lru_purge_enable = true;
    http_config.recv_wait_timeout = 10;
    http_config.send_wait_timeout = 10;

    httpd_handle_t server = nullptr;
    esp_err_t result = httpd_start(&server, &http_config);
    if (result == ESP_OK) {
        const httpd_uri_t upload_uri = {
            .uri = OTA_UPLOAD_PATH,
            .method = HTTP_POST,
            .handler = ota_post_handler,
            .user_ctx = nullptr,
        };
        result = httpd_register_uri_handler(server, &upload_uri);
    }
    if (result != ESP_OK && server) {
        httpd_stop(server);
        server = nullptr;
    }

    portENTER_CRITICAL(&s_state_lock);
    s_server = server;
    s_server_starting = false;
    if (result != ESP_OK) {
        memset(s_ota_key, 0, sizeof(s_ota_key));
        s_config = OtaServerConfig{};
    }
    portEXIT_CRITICAL(&s_state_lock);

    if (result == ESP_OK) {
        ESP_LOGI(TAG, "OTA server listening on port %u at POST %s",
                 config.port, OTA_UPLOAD_PATH);
    } else {
        ESP_LOGE(TAG, "Could not start OTA server: %s", esp_err_to_name(result));
    }
    return result;
}

esp_err_t ota_server_stop()
{
    httpd_handle_t server = nullptr;
    portENTER_CRITICAL(&s_state_lock);
    if (!s_server || s_server_starting || s_update_in_progress) {
        portEXIT_CRITICAL(&s_state_lock);
        return ESP_ERR_INVALID_STATE;
    }
    server = s_server;
    s_server = nullptr;
    portEXIT_CRITICAL(&s_state_lock);

    const esp_err_t result = httpd_stop(server);
    portENTER_CRITICAL(&s_state_lock);
    memset(s_ota_key, 0, sizeof(s_ota_key));
    s_config = OtaServerConfig{};
    portEXIT_CRITICAL(&s_state_lock);
    return result;
}

bool ota_server_update_in_progress()
{
    portENTER_CRITICAL(&s_state_lock);
    const bool in_progress = s_update_in_progress;
    portEXIT_CRITICAL(&s_state_lock);
    return in_progress;
}

} // namespace aicam
