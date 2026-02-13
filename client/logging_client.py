#!/usr/bin/env python3
import socket
import struct
import json
import time

try:
    import zstandard as zstd
    HAS_ZSTD = True
except ImportError:
    HAS_ZSTD = False

# === Configuration ===
LOCAL_IP = "0.0.0.0"       # Listen on all interfaces
RECEIVE_PORT = 8502        # Must match NetworkGateway send_port
BUFFER_SIZE = 65535        # Max UDP packet size


def parse_header(packet: bytes):
    """
    Header layout (from C++ createHeader):
      [8 bytes timestamp (double)][1 byte compressed flag][topic\\0][type\\0][payload...]
    """
    if len(packet) < 11:
        return None, None, None, False, None

    timestamp = struct.unpack("d", packet[:8])[0]
    compressed = packet[8] != 0
    remainder = packet[9:]

    topic_end = remainder.find(b"\0")
    if topic_end == -1:
        return None, None, None, False, None
    topic = remainder[:topic_end].decode(errors="replace")

    type_start = topic_end + 1
    type_end = remainder.find(b"\0", type_start)
    if type_end == -1:
        return None, None, None, False, None
    msg_type = remainder[type_start:type_end].decode(errors="replace")

    payload = remainder[type_end + 1:]
    return timestamp, topic, msg_type, compressed, payload


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LOCAL_IP, RECEIVE_PORT))
    sock.settimeout(1.0)

    decompressor = zstd.ZstdDecompressor() if HAS_ZSTD else None

    print(f"[UDP Client] Listening on {LOCAL_IP}:{RECEIVE_PORT}")
    if not HAS_ZSTD:
        print("[UDP Client] Warning: zstandard not installed, compressed messages will be shown as raw bytes")

    while True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            timestamp, topic, msg_type, compressed, payload = parse_header(data)
            if not topic:
                continue

            if compressed and decompressor:
                payload = decompressor.decompress(payload)

            try:
                msg = json.loads(payload.decode("utf-8", errors="replace"))
                msg_str = json.dumps(msg, ensure_ascii=False)
            except Exception:
                msg_str = payload.decode("utf-8", errors="replace")

            delay = time.time() - timestamp
            comp_tag = " [compressed]" if compressed else ""
            print(f"\n[{topic}] ({msg_type}{comp_tag}) @ {timestamp:.3f} (+{delay*1000:.1f} ms):\n{msg_str}")

        except socket.timeout:
            continue
        except KeyboardInterrupt:
            print("\nExiting.")
            break


if __name__ == "__main__":
    main()
