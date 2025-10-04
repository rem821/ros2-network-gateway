#!/usr/bin/env python3
import socket
import struct
import json
import time

# === Configuration ===
LOCAL_IP = "0.0.0.0"       # Listen on all interfaces
RECEIVE_PORT = 8502        # Must match NetworkGateway send_port
BUFFER_SIZE = 65535        # Max UDP packet size

# === Helper: parse your C++ header format ===
def parse_header(packet: bytes):
    """
    Header layout (from your C++ createHeader):
    [8 bytes timestamp(double)][topic][\0][type][\0][payload...]
    """
    if len(packet) < 10:
        return None, None, None

    # Extract timestamp
    timestamp = struct.unpack("d", packet[:8])[0]

    # Split after timestamp
    remainder = packet[8:]

    # Find null terminators
    topic_end = remainder.find(b"\0")
    if topic_end == -1:
        return None, None, None
    topic = remainder[:topic_end].decode(errors="replace")

    type_start = topic_end + 1
    type_end = remainder.find(b"\0", type_start)
    if type_end == -1:
        return None, None, None
    msg_type = remainder[type_start:type_end].decode(errors="replace")

    payload = remainder[type_end + 1:]
    return timestamp, topic, msg_type, payload


# === Main ===
def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LOCAL_IP, RECEIVE_PORT))
    sock.settimeout(1.0)

    print(f"[UDP Client] Listening on {LOCAL_IP}:{RECEIVE_PORT}")

    while True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            timestamp, topic, msg_type, payload = parse_header(data)
            if not topic:
                continue

            try:
                # Payload is expected to be UTF-8 JSON
                msg = json.loads(payload.decode("utf-8", errors="replace"))
                msg_str = json.dumps(msg, ensure_ascii=False)
            except Exception as e:
                msg_str = payload.decode("utf-8", errors="replace")

            delay = time.time() - timestamp
            print(f"\n[{topic}] ({msg_type}) @ {timestamp:.3f} (+{delay*1000:.1f} ms):\n{msg_str}")

        except socket.timeout:
            continue
        except KeyboardInterrupt:
            print("\nExiting.")
            break


if __name__ == "__main__":
    main()
