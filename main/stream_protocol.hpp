#pragma once

#include <cstddef>
#include <cstdint>

namespace aicam {

constexpr uint32_t VIDEO_MAGIC = 0x4149434D;   // "AICM"
constexpr uint32_t CONTROL_MAGIC = 0x41494343; // "AICC"
constexpr uint8_t PROTOCOL_VERSION = 1;

constexpr uint16_t VIDEO_PORT = 5005;
constexpr uint16_t CONTROL_PORT = 5006;
constexpr size_t VIDEO_CHUNK_PAYLOAD_MAX = 1400;
constexpr size_t VIDEO_FRAME_SIZE_MAX = 8 * 1024 * 1024;
constexpr size_t CONTROL_PAYLOAD_MAX = 20;

enum VideoFlags : uint8_t {
    VIDEO_FLAG_SOF = 1U << 0,
    VIDEO_FLAG_EOF = 1U << 1,
};

enum class StreamProfile : uint8_t {
    MaxFps = 0,
    Balanced = 1,
    MaxResolution = 2,
};

enum ControlCommand : uint8_t {
    CMD_HELLO = 0x01,
    CMD_STOP_STREAM = 0x02,
    CMD_SET_PROFILE = 0x03,
    CMD_MOTOR = 0x10,
    CMD_SET_PEER = 0x11,
    CMD_PING = 0x12,
    CMD_DETECTION_EVENT = 0x13,
    CMD_RESPONSE_BIT = 0x80,
};

#pragma pack(push, 1)

// All multi-byte fields are transmitted in network byte order.
// Python format: !IBBHIIQIHHHHHBB (40 bytes)
struct VideoChunkHeader {
    uint32_t magic;
    uint8_t version;
    uint8_t flags;
    uint16_t header_size;
    uint32_t stream_id;
    uint32_t frame_id;
    uint64_t timestamp_us;
    uint32_t frame_size;
    uint16_t width;
    uint16_t height;
    uint16_t chunk_index;
    uint16_t chunk_count;
    uint16_t payload_size;
    uint8_t jpeg_quality;
    uint8_t profile;
};

// Header is followed by payload_len bytes. Payload is limited to 20 bytes.
// Python header format: !IBBHI (12 bytes)
struct ControlHeader {
    uint32_t magic;
    uint8_t version;
    uint8_t command;
    uint16_t payload_len;
    uint32_t sequence;
};

#pragma pack(pop)

static_assert(sizeof(VideoChunkHeader) == 40, "Unexpected video header size");
static_assert(sizeof(ControlHeader) == 12, "Unexpected control header size");

} // namespace aicam
