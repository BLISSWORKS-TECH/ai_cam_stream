#include "camera_stream.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>

#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

namespace aicam {
namespace {

constexpr gpio_num_t PIN_PWDN = GPIO_NUM_38;
constexpr int PIN_RESET = -1;
constexpr int PIN_XCLK = 16;
constexpr int PIN_SIOD = 4;
constexpr int PIN_SIOC = 5;
constexpr int PIN_D7 = 15;
constexpr int PIN_D6 = 17;
constexpr int PIN_D5 = 18;
constexpr int PIN_D4 = 13;
constexpr int PIN_D3 = 11;
constexpr int PIN_D2 = 9;
constexpr int PIN_D1 = 10;
constexpr int PIN_D0 = 12;
constexpr int PIN_VSYNC = 6;
constexpr int PIN_HREF = 7;
constexpr int PIN_PCLK = 8;

constexpr char TAG[] = "camera_stream";
constexpr uint32_t PROFILE_WARMUP_FRAMES = 2;
constexpr uint32_t TX_PACING_INTERVAL_PACKETS = 16;
constexpr uint32_t TX_PACING_US = 250;

struct ProfileConfig {
    framesize_t frame_size;
    uint8_t jpeg_quality;
    const char *name;
};

// Lower JPEG quality numbers produce larger, higher-quality JPEG images.
constexpr ProfileConfig PROFILES[] = {
    {FRAMESIZE_VGA, 22, "MAX_FPS"},             // 640x480
    {FRAMESIZE_SVGA, 14, "BALANCED"},           // 800x600
    {FRAMESIZE_UXGA, 12, "MAX_RESOLUTION"},     // 1600x1200
};

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_stream_enabled = false;
static uint32_t s_client_ip_be = 0;
static uint16_t s_client_port_be = 0;
static uint8_t s_requested_profile = CONFIG_AICAM_STREAM_DEFAULT_PROFILE;
static uint8_t s_active_profile = CONFIG_AICAM_STREAM_DEFAULT_PROFILE;
static uint32_t s_control_generation = 0;
static TaskHandle_t s_stream_task_handle = nullptr;
static SemaphoreHandle_t s_startup_signal = nullptr;
static SemaphoreHandle_t s_shutdown_signal = nullptr;
static esp_err_t s_startup_result = ESP_ERR_INVALID_STATE;
static bool s_shutdown_requested = false;
static bool s_camera_initialized = false;
static StreamStats s_stats{};

void signal_stream_startup(esp_err_t result)
{
    portENTER_CRITICAL(&s_lock);
    s_startup_result = result;
    portEXIT_CRITICAL(&s_lock);
    if (s_startup_signal) {
        xSemaphoreGive(s_startup_signal);
    }
}

uint64_t host_to_network_u64(uint64_t value)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return __builtin_bswap64(value);
#else
    return value;
#endif
}

bool valid_profile(uint8_t profile)
{
    return profile <= static_cast<uint8_t>(StreamProfile::MaxResolution);
}

esp_err_t apply_profile(StreamProfile profile)
{
    const uint8_t index = static_cast<uint8_t>(profile);
    if (!valid_profile(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t previous_index;
    portENTER_CRITICAL(&s_lock);
    previous_index = s_active_profile;
    portEXIT_CRITICAL(&s_lock);
    if (!valid_profile(previous_index)) {
        previous_index = static_cast<uint8_t>(StreamProfile::Balanced);
    }

    const ProfileConfig &cfg = PROFILES[index];
    const ProfileConfig &previous = PROFILES[previous_index];
    if (sensor->set_framesize(sensor, cfg.frame_size) != 0 ||
        sensor->set_quality(sensor, cfg.jpeg_quality) != 0 ||
        sensor->status.framesize != cfg.frame_size ||
        sensor->status.quality != cfg.jpeg_quality) {
        ESP_LOGE(TAG, "Failed to apply profile %s", cfg.name);
        sensor->set_framesize(sensor, previous.frame_size);
        sensor->set_quality(sensor, previous.jpeg_quality);
        return ESP_FAIL;
    }

    // Discard frames captured while the OV2640 is changing timing and exposure.
    for (uint32_t i = 0; i < PROFILE_WARMUP_FRAMES; ++i) {
        bool shutdown_requested;
        portENTER_CRITICAL(&s_lock);
        shutdown_requested = s_shutdown_requested;
        portEXIT_CRITICAL(&s_lock);
        if (shutdown_requested) {
            sensor->set_framesize(sensor, previous.frame_size);
            sensor->set_quality(sensor, previous.jpeg_quality);
            return ESP_ERR_INVALID_STATE;
        }
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera did not produce a warm-up frame for %s", cfg.name);
            sensor->set_framesize(sensor, previous.frame_size);
            sensor->set_quality(sensor, previous.jpeg_quality);
            return ESP_FAIL;
        }
        esp_camera_fb_return(fb);
    }

    portENTER_CRITICAL(&s_lock);
    s_active_profile = index;
    s_stats.active_profile = index;
    portEXIT_CRITICAL(&s_lock);

    ESP_LOGI(TAG, "Profile applied: %s, JPEG quality=%u", cfg.name, cfg.jpeg_quality);
    return ESP_OK;
}

int create_video_socket()
{
    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "video socket() failed: errno=%d", errno);
        return -1;
    }

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(VIDEO_PORT);
    if (bind(sock, reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0) {
        ESP_LOGE(TAG, "video bind() failed: errno=%d", errno);
        close(sock);
        return -1;
    }

    int tos = IPTOS_LOWDELAY | IPTOS_THROUGHPUT;
    setsockopt(sock, IPPROTO_IP, IP_TOS, &tos, sizeof(tos));

    const int flags = fcntl(sock, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    }
    return sock;
}

void update_success_stats(size_t frame_bytes, uint32_t packets)
{
    portENTER_CRITICAL(&s_lock);
    ++s_stats.frames_sent;
    s_stats.packets_sent += packets;
    s_stats.last_frame_bytes = static_cast<uint32_t>(frame_bytes);
    s_stats.last_frame_sent_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_lock);
}

void update_drop_stats(bool send_error)
{
    portENTER_CRITICAL(&s_lock);
    ++s_stats.frames_dropped;
    if (send_error) {
        ++s_stats.send_errors;
    }
    portEXIT_CRITICAL(&s_lock);
}

void stream_task(void *)
{
    const int sock = create_video_socket();
    if (sock < 0) {
        signal_stream_startup(ESP_FAIL);
        s_stream_task_handle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    auto *packet = static_cast<uint8_t *>(heap_caps_malloc(
        sizeof(VideoChunkHeader) + VIDEO_CHUNK_PAYLOAD_MAX,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (!packet) {
        ESP_LOGE(TAG, "Unable to allocate UDP packet buffer");
        close(sock);
        signal_stream_startup(ESP_ERR_NO_MEM);
        s_stream_task_handle = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    signal_stream_startup(ESP_OK);

    uint32_t stream_id = esp_random();
    if (stream_id == 0) {
        stream_id = 1;
    }
    uint32_t frame_id = 0;
    int64_t stats_started_us = esp_timer_get_time();
    uint32_t stats_frames = 0;
    uint64_t stats_bytes = 0;

    for (;;) {
        uint8_t requested;
        uint8_t active;
        bool shutdown_requested;
        portENTER_CRITICAL(&s_lock);
        requested = s_requested_profile;
        active = s_active_profile;
        shutdown_requested = s_shutdown_requested;
        portEXIT_CRITICAL(&s_lock);
        if (shutdown_requested) {
            break;
        }
        if (requested != active && valid_profile(requested)) {
            if (apply_profile(static_cast<StreamProfile>(requested)) == ESP_OK) {
                if (++stream_id == 0) {
                    ++stream_id;
                }
                frame_id = 0;
            } else {
                portENTER_CRITICAL(&s_lock);
                s_requested_profile = s_active_profile;
                portEXIT_CRITICAL(&s_lock);
                vTaskDelay(pdMS_TO_TICKS(250));
            }
        }

        uint32_t client_ip_be;
        uint16_t client_port_be;
        uint32_t control_generation;
        uint8_t profile;
        bool enabled;
        portENTER_CRITICAL(&s_lock);
        client_ip_be = s_client_ip_be;
        client_port_be = s_client_port_be;
        control_generation = s_control_generation;
        profile = s_active_profile;
        enabled = s_stream_enabled;
        portEXIT_CRITICAL(&s_lock);

        if (!enabled || client_ip_be == 0 || client_port_be == 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Camera capture failed");
            update_drop_stats(false);
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        if (fb->format != PIXFORMAT_JPEG || !fb->buf || fb->len == 0 ||
            fb->len > VIDEO_FRAME_SIZE_MAX) {
            ESP_LOGW(TAG, "Invalid camera frame");
            update_drop_stats(false);
            esp_camera_fb_return(fb);
            continue;
        }

        const size_t chunk_count_size =
            (fb->len + VIDEO_CHUNK_PAYLOAD_MAX - 1) / VIDEO_CHUNK_PAYLOAD_MAX;
        if (chunk_count_size == 0 || chunk_count_size > UINT16_MAX) {
            ESP_LOGW(TAG, "Frame too large to packetize: %u bytes", static_cast<unsigned>(fb->len));
            update_drop_stats(false);
            esp_camera_fb_return(fb);
            continue;
        }

        sockaddr_in destination{};
        destination.sin_family = AF_INET;
        destination.sin_addr.s_addr = client_ip_be;
        destination.sin_port = client_port_be;

        const uint16_t chunk_count = static_cast<uint16_t>(chunk_count_size);
        const uint8_t jpeg_quality = PROFILES[profile].jpeg_quality;
        uint64_t timestamp_us =
            static_cast<uint64_t>(fb->timestamp.tv_sec) * 1000000ULL +
            static_cast<uint64_t>(fb->timestamp.tv_usec);
        if (timestamp_us == 0) {
            timestamp_us = static_cast<uint64_t>(esp_timer_get_time());
        }
        bool complete = true;
        bool drop_counted = false;
        uint32_t sent_packets = 0;

        for (uint16_t chunk = 0; chunk < chunk_count; ++chunk) {
            if ((chunk & 0x07U) == 0) {
                bool state_matches;
                portENTER_CRITICAL(&s_lock);
                state_matches = s_stream_enabled &&
                                !s_shutdown_requested &&
                                s_client_ip_be == client_ip_be &&
                                s_client_port_be == client_port_be &&
                                s_control_generation == control_generation;
                portEXIT_CRITICAL(&s_lock);
                if (!state_matches) {
                    complete = false;
                    update_drop_stats(false);
                    drop_counted = true;
                    break;
                }
            }

            const size_t offset = static_cast<size_t>(chunk) * VIDEO_CHUNK_PAYLOAD_MAX;
            const size_t payload_size = std::min(VIDEO_CHUNK_PAYLOAD_MAX, fb->len - offset);

            auto *header = reinterpret_cast<VideoChunkHeader *>(packet);
            header->magic = htonl(VIDEO_MAGIC);
            header->version = PROTOCOL_VERSION;
            header->flags = 0;
            if (chunk == 0) {
                header->flags |= VIDEO_FLAG_SOF;
            }
            if (chunk + 1 == chunk_count) {
                header->flags |= VIDEO_FLAG_EOF;
            }
            header->header_size = htons(sizeof(VideoChunkHeader));
            header->stream_id = htonl(stream_id);
            header->frame_id = htonl(frame_id);
            header->timestamp_us = host_to_network_u64(timestamp_us);
            header->frame_size = htonl(static_cast<uint32_t>(fb->len));
            header->width = htons(static_cast<uint16_t>(fb->width));
            header->height = htons(static_cast<uint16_t>(fb->height));
            header->chunk_index = htons(chunk);
            header->chunk_count = htons(chunk_count);
            header->payload_size = htons(static_cast<uint16_t>(payload_size));
            header->jpeg_quality = jpeg_quality;
            header->profile = profile;

            memcpy(packet + sizeof(VideoChunkHeader), fb->buf + offset, payload_size);
            const size_t datagram_size = sizeof(VideoChunkHeader) + payload_size;
            ssize_t sent = sendto(
                sock,
                packet,
                datagram_size,
                0,
                reinterpret_cast<sockaddr *>(&destination),
                sizeof(destination));

            if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK ||
                             errno == ENOBUFS || errno == ENOMEM)) {
                esp_rom_delay_us(TX_PACING_US);
                sent = sendto(
                    sock,
                    packet,
                    datagram_size,
                    0,
                    reinterpret_cast<sockaddr *>(&destination),
                    sizeof(destination));
            }

            if (sent != static_cast<ssize_t>(datagram_size)) {
                complete = false;
                update_drop_stats(true);
                drop_counted = true;
                break; // Never queue stale fragments behind the next frame.
            }
            ++sent_packets;

            // Let higher-priority Wi-Fi/control work run during large UXGA frames.
            if ((chunk + 1U) % TX_PACING_INTERVAL_PACKETS == 0) {
                esp_rom_delay_us(TX_PACING_US);
                taskYIELD();
            }
        }

        if (complete) {
            update_success_stats(fb->len, sent_packets);
            ++stats_frames;
            stats_bytes += fb->len;
        } else if (!drop_counted) {
            update_drop_stats(false);
        }

        ++frame_id;
        esp_camera_fb_return(fb);

        const int64_t now_us = esp_timer_get_time();
        if (now_us - stats_started_us >= 5000000LL) {
            const double seconds = static_cast<double>(now_us - stats_started_us) / 1000000.0;
            const double fps = stats_frames / seconds;
            const double mbps = (static_cast<double>(stats_bytes) * 8.0) / seconds / 1000000.0;
            uint8_t log_profile;
            portENTER_CRITICAL(&s_lock);
            log_profile = s_active_profile;
            portEXIT_CRITICAL(&s_lock);
            ESP_LOGI(TAG, "stream %.1f fps, %.2f Mbit/s, profile=%s, free_psram=%u",
                     fps,
                     mbps,
                     PROFILES[log_profile].name,
                     static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
            stats_frames = 0;
            stats_bytes = 0;
            stats_started_us = now_us;
        }
    }

    heap_caps_free(packet);
    close(sock);
    portENTER_CRITICAL(&s_lock);
    s_stream_task_handle = nullptr;
    portEXIT_CRITICAL(&s_lock);
    if (s_shutdown_signal) {
        xSemaphoreGive(s_shutdown_signal);
    }
    ESP_LOGI(TAG, "Video task stopped cleanly");
    vTaskDelete(nullptr);
}

} // namespace

esp_err_t camera_stream_init()
{
    gpio_hold_dis(PIN_PWDN);

    camera_config_t config{};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = PIN_D0;
    config.pin_d1 = PIN_D1;
    config.pin_d2 = PIN_D2;
    config.pin_d3 = PIN_D3;
    config.pin_d4 = PIN_D4;
    config.pin_d5 = PIN_D5;
    config.pin_d6 = PIN_D6;
    config.pin_d7 = PIN_D7;
    config.pin_xclk = PIN_XCLK;
    config.pin_pclk = PIN_PCLK;
    config.pin_vsync = PIN_VSYNC;
    config.pin_href = PIN_HREF;
    config.pin_sccb_sda = PIN_SIOD;
    config.pin_sccb_scl = PIN_SIOC;
    config.pin_pwdn = PIN_PWDN;
    config.pin_reset = PIN_RESET;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    // Allocate for the largest profile once. Runtime changes only shrink or restore
    // the sensor output, avoiding expensive camera driver reinitialization.
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = PROFILES[static_cast<uint8_t>(StreamProfile::MaxResolution)].jpeg_quality;
    // Two buffers enable continuous JPEG capture while keeping only one queued
    // frame in LATEST mode. A third buffer adds latency without helping control.
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    const esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: %s", esp_err_to_name(err));
        return err;
    }

    portENTER_CRITICAL(&s_lock);
    s_camera_initialized = true;
    portEXIT_CRITICAL(&s_lock);

    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor) {
        esp_camera_deinit();
        portENTER_CRITICAL(&s_lock);
        s_camera_initialized = false;
        portEXIT_CRITICAL(&s_lock);
        return ESP_FAIL;
    }

#if CONFIG_AICAM_STREAM_ROTATE_180
    if (sensor->set_hmirror) {
        sensor->set_hmirror(sensor, 1);
    }
    if (sensor->set_vflip) {
        sensor->set_vflip(sensor, 1);
    }
#endif

    const uint8_t default_profile = valid_profile(CONFIG_AICAM_STREAM_DEFAULT_PROFILE)
                                        ? CONFIG_AICAM_STREAM_DEFAULT_PROFILE
                                        : static_cast<uint8_t>(StreamProfile::Balanced);
    portENTER_CRITICAL(&s_lock);
    s_requested_profile = default_profile;
    s_active_profile = default_profile;
    ++s_control_generation;
    portEXIT_CRITICAL(&s_lock);
    const esp_err_t profile_result =
        apply_profile(static_cast<StreamProfile>(default_profile));
    if (profile_result != ESP_OK) {
        esp_camera_deinit();
        portENTER_CRITICAL(&s_lock);
        s_camera_initialized = false;
        portEXIT_CRITICAL(&s_lock);
    }
    return profile_result;
}

esp_err_t camera_stream_start()
{
    if (s_stream_task_handle) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_startup_signal) {
        s_startup_signal = xSemaphoreCreateBinary();
        if (!s_startup_signal) {
            return ESP_ERR_NO_MEM;
        }
    } else {
        xSemaphoreTake(s_startup_signal, 0);
    }
    if (!s_shutdown_signal) {
        s_shutdown_signal = xSemaphoreCreateBinary();
        if (!s_shutdown_signal) {
            return ESP_ERR_NO_MEM;
        }
    } else {
        xSemaphoreTake(s_shutdown_signal, 0);
    }
    portENTER_CRITICAL(&s_lock);
    s_startup_result = ESP_ERR_INVALID_STATE;
    s_shutdown_requested = false;
    portEXIT_CRITICAL(&s_lock);

    const BaseType_t ok = xTaskCreatePinnedToCore(
        stream_task,
        "jpeg_udp_stream",
        8 * 1024,
        nullptr,
        6,
        &s_stream_task_handle,
        1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t camera_stream_shutdown(uint32_t timeout_ms)
{
    TaskHandle_t task_handle;
    portENTER_CRITICAL(&s_lock);
    s_shutdown_requested = true;
    s_stream_enabled = false;
    ++s_control_generation;
    task_handle = s_stream_task_handle;
    portEXIT_CRITICAL(&s_lock);

    if (task_handle) {
        if (!s_shutdown_signal ||
            xSemaphoreTake(s_shutdown_signal, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
            ESP_LOGE(TAG, "Timed out waiting for the video task to stop");
            return ESP_ERR_TIMEOUT;
        }
    }

    bool camera_initialized;
    portENTER_CRITICAL(&s_lock);
    camera_initialized = s_camera_initialized;
    portEXIT_CRITICAL(&s_lock);
    esp_err_t result = ESP_OK;
    if (camera_initialized) {
        result = esp_camera_deinit();
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "esp_camera_deinit failed: %s", esp_err_to_name(result));
            return result;
        }
        portENTER_CRITICAL(&s_lock);
        s_camera_initialized = false;
        portEXIT_CRITICAL(&s_lock);
    }

    gpio_hold_dis(PIN_PWDN);
    gpio_config_t power_config{};
    power_config.pin_bit_mask = 1ULL << PIN_PWDN;
    power_config.mode = GPIO_MODE_OUTPUT;
    power_config.pull_up_en = GPIO_PULLUP_DISABLE;
    power_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    power_config.intr_type = GPIO_INTR_DISABLE;
    result = gpio_config(&power_config);
    if (result != ESP_OK) {
        return result;
    }
    gpio_set_level(PIN_PWDN, 1);
    ESP_RETURN_ON_ERROR(gpio_hold_en(PIN_PWDN), TAG, "camera PWDN hold failed");
    ESP_LOGI(TAG, "Camera deinitialized and powered down");
    return ESP_OK;
}

esp_err_t camera_stream_wait_ready(uint32_t timeout_ms)
{
    if (!s_startup_signal) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_startup_signal, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    portENTER_CRITICAL(&s_lock);
    const esp_err_t result = s_startup_result;
    portEXIT_CRITICAL(&s_lock);
    return result;
}

void camera_stream_set_client(uint32_t ipv4_network_order, uint16_t port_network_order)
{
    portENTER_CRITICAL(&s_lock);
    s_client_ip_be = ipv4_network_order;
    s_client_port_be = port_network_order;
    ++s_control_generation;
    portEXIT_CRITICAL(&s_lock);
}

void camera_stream_enable(bool enabled)
{
    portENTER_CRITICAL(&s_lock);
    s_stream_enabled = enabled;
    ++s_control_generation;
    portEXIT_CRITICAL(&s_lock);
}

bool camera_stream_is_enabled()
{
    portENTER_CRITICAL(&s_lock);
    const bool enabled = s_stream_enabled;
    portEXIT_CRITICAL(&s_lock);
    return enabled;
}

esp_err_t camera_stream_request_profile(StreamProfile profile)
{
    const uint8_t value = static_cast<uint8_t>(profile);
    if (!valid_profile(value)) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_lock);
    if (s_requested_profile != value) {
        s_requested_profile = value;
        ++s_control_generation;
    }
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

StreamProfile camera_stream_active_profile()
{
    portENTER_CRITICAL(&s_lock);
    const uint8_t active = s_active_profile;
    portEXIT_CRITICAL(&s_lock);
    return static_cast<StreamProfile>(active);
}

void camera_stream_get_stats(StreamStats *out)
{
    if (!out) {
        return;
    }
    portENTER_CRITICAL(&s_lock);
    *out = s_stats;
    portEXIT_CRITICAL(&s_lock);
}

} // namespace aicam
