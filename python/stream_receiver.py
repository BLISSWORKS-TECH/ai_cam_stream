"""AI Cam Stream 수신 뷰어.

ESP32 SoftAP(기본 192.168.4.1)에 접속한 상태에서 실행하면, 컨트롤 포트(5006)로
HELLO 명령을 보내 스트리밍을 시작시키고, 비디오 포트(5005)로 들어오는 JPEG 청크를
재조립하여 OpenCV 창으로 출력한다.

프로토콜은 main/stream_protocol.hpp 를 그대로 따른다:
  - ControlHeader: !IBBHI  (magic, version, command, payload_len, sequence) = 12 bytes
  - VideoChunkHeader: !IBBHIIQIHHHHHBB
      (magic, version, flags, header_size, stream_id, frame_id, timestamp_us,
       frame_size, width, height, chunk_index, chunk_count, payload_size,
       jpeg_quality, profile) = 40 bytes

SoftAP에 연결한 뒤 그냥 실행하면 된다(별도 인자 불필요):
    python stream_receiver.py
"""

from __future__ import annotations

import socket
import struct
import sys
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

# ESP32 SoftAP 기본값. 필요하면 여기만 바꾸면 된다.
DEVICE_HOST = "192.168.4.1"
STREAM_PROFILE = 0  # 0=MaxFps, 1=Balanced, 2=MaxResolution

VIDEO_MAGIC = 0x4149434D  # "AICM"
CONTROL_MAGIC = 0x41494343  # "AICC"
PROTOCOL_VERSION = 1

VIDEO_PORT = 5005
CONTROL_PORT = 5006

CMD_HELLO = 0x01
CMD_STOP_STREAM = 0x02
CMD_SET_PROFILE = 0x03
CMD_PING = 0x12
CMD_RESPONSE_BIT = 0x80

VIDEO_FLAG_SOF = 1 << 0
VIDEO_FLAG_EOF = 1 << 1

CONTROL_HEADER_FMT = "!IBBHI"
CONTROL_HEADER_SIZE = struct.calcsize(CONTROL_HEADER_FMT)

VIDEO_HEADER_FMT = "!IBBHIIQIHHHHHBB"
VIDEO_HEADER_SIZE = struct.calcsize(VIDEO_HEADER_FMT)

RECV_BUFFER_SIZE = 2048


@dataclass
class PendingFrame:
    frame_id: int
    frame_size: int
    chunk_count: int
    width: int
    height: int
    chunks: dict = field(default_factory=dict)

    def is_complete(self) -> bool:
        return len(self.chunks) == self.chunk_count

    def assemble(self) -> bytes:
        return b"".join(self.chunks[i] for i in range(self.chunk_count))


def build_control_datagram(command: int, sequence: int, payload: bytes = b"") -> bytes:
    header = struct.pack(
        CONTROL_HEADER_FMT,
        CONTROL_MAGIC,
        PROTOCOL_VERSION,
        command,
        len(payload),
        sequence,
    )
    return header + payload


def parse_control_response(datagram: bytes):
    if len(datagram) < CONTROL_HEADER_SIZE:
        return None
    magic, version, command, payload_len, sequence = struct.unpack(
        CONTROL_HEADER_FMT, datagram[:CONTROL_HEADER_SIZE]
    )
    if magic != CONTROL_MAGIC or version != PROTOCOL_VERSION:
        return None
    payload = datagram[CONTROL_HEADER_SIZE : CONTROL_HEADER_SIZE + payload_len]
    return command, sequence, payload


def send_control_command(
    sock: socket.socket,
    host: str,
    command: int,
    sequence: int,
    payload: bytes = b"",
    retries: int = 5,
    timeout_s: float = 1.0,
):
    """컨트롤 명령을 보내고 응답을 기다린다. 성공 시 (command, sequence, payload) 반환."""
    datagram = build_control_datagram(command, sequence, payload)
    sock.settimeout(timeout_s)
    for attempt in range(retries):
        sock.sendto(datagram, (host, CONTROL_PORT))
        try:
            response, _ = sock.recvfrom(RECV_BUFFER_SIZE)
        except socket.timeout:
            print(f"[control] no response, retry {attempt + 1}/{retries}")
            continue
        parsed = parse_control_response(response)
        if parsed is None:
            continue
        resp_command, resp_sequence, resp_payload = parsed
        if resp_sequence != sequence or (resp_command & ~CMD_RESPONSE_BIT) != command:
            continue
        return resp_command, resp_sequence, resp_payload
    return None


def start_stream(control_sock: socket.socket, host: str, profile: int) -> bool:
    result = send_control_command(control_sock, host, CMD_HELLO, sequence=1, payload=bytes([profile]))
    if result is None:
        print("[control] HELLO failed: no response from device")
        return False
    _, _, payload = result
    status = payload[0] if payload else 1
    if status != 0:
        print(f"[control] HELLO rejected, status={status}")
        return False
    print(f"[control] streaming started, profile={profile}")
    return True


def stop_stream(control_sock: socket.socket, host: str) -> None:
    send_control_command(control_sock, host, CMD_STOP_STREAM, sequence=2, retries=2, timeout_s=0.5)


def receive_and_display(host: str, profile: int, listen_port: int) -> None:
    control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    video_sock.bind(("0.0.0.0", listen_port))
    video_sock.settimeout(1.0)

    if not start_stream(control_sock, host, profile):
        control_sock.close()
        video_sock.close()
        sys.exit(1)

    window_name = "AI Cam Stream"
    pending: PendingFrame | None = None
    last_frame_shown = 0.0
    frames_shown = 0
    started_at = time.time()

    try:
        while True:
            try:
                datagram, _ = video_sock.recvfrom(RECV_BUFFER_SIZE)
            except socket.timeout:
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            if len(datagram) < VIDEO_HEADER_SIZE:
                continue

            (
                magic,
                version,
                flags,
                header_size,
                stream_id,
                frame_id,
                timestamp_us,
                frame_size,
                width,
                height,
                chunk_index,
                chunk_count,
                payload_size,
                jpeg_quality,
                stream_profile,
            ) = struct.unpack(VIDEO_HEADER_FMT, datagram[:VIDEO_HEADER_SIZE])

            if magic != VIDEO_MAGIC or version != PROTOCOL_VERSION:
                continue

            payload = datagram[header_size : header_size + payload_size]
            if len(payload) != payload_size:
                continue

            if flags & VIDEO_FLAG_SOF:
                pending = PendingFrame(frame_id, frame_size, chunk_count, width, height)

            if pending is None or pending.frame_id != frame_id:
                continue

            pending.chunks[chunk_index] = payload

            if not pending.is_complete():
                continue

            jpeg_bytes = pending.assemble()
            pending = None

            image_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            frames_shown += 1
            elapsed = time.time() - started_at
            fps = frames_shown / elapsed if elapsed > 0 else 0.0
            cv2.putText(
                frame,
                f"{width}x{height} q={jpeg_quality} fps={fps:.1f}",
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
            cv2.imshow(window_name, frame)
            last_frame_shown = time.time()

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        print("[control] stopping stream")
        stop_stream(control_sock, host)
        control_sock.close()
        video_sock.close()
        cv2.destroyAllWindows()


def main() -> None:
    receive_and_display(DEVICE_HOST, STREAM_PROFILE, VIDEO_PORT)


if __name__ == "__main__":
    main()
