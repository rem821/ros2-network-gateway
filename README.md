# ROS2 Network Gateway

A lightweight ROS2 package that bridges ROS2 topics to non-ROS2 devices over UDP. It dynamically discovers topics at runtime, serializes any message type to JSON using introspection, and sends the data as UDP packets with optional Zstd compression.

Built as part of the [BUT Telepresence](https://github.com/Robotics-BUT/BUT_Telepresence) project to stream robot telemetry (battery state, clock, sensors, ...) to a Meta Quest VR headset that cannot run ROS2 natively.

## How it works

```
ROS2 topic graph                        Non-ROS2 device
 ┌──────────┐                           ┌──────────────┐
 │ /battery  │──┐                       │  VR headset  │
 │ /clock    │──┤   ┌───────────────┐   │  or any UDP  │
 │ /imu      │──┼──>│ Network       │──>│  client      │
 │ /odom     │──┤   │ Gateway node  │   │              │
 │ ...       │──┘   └───────────────┘   └──────────────┘
 └──────────┘        subscribe ──> JSON ──> UDP
```

The gateway node periodically queries the ROS2 graph for available topics (`get_topic_names_and_types`). For each topic matching the configured whitelist it creates a **generic subscription** — no compile-time message type dependency is needed. Incoming messages are deserialized through `rosidl_typesupport_introspection_cpp`, recursively converted to JSON, optionally compressed with Zstd, and sent over UDP.

Two kinds of packets are sent for every subscribed topic:

| Packet type | Interval | Content |
|---|---|---|
| **Proto** (schema) | every 5 s | JSON description of the message structure (field names, types, nesting) |
| **Data** | configurable (default 500 ms) | JSON-serialized latest message value (optionally Zstd-compressed) |

## Requirements

- ROS2 Jazzy (tested), should work on Humble+
- C++20 compiler
- [Boost.Asio](https://www.boost.org/doc/libs/release/doc/html/boost_asio.html) (for async UDP)
- [nlohmann/json](https://github.com/nlohmann/json) (header-only, vendored or system)
- [Zstandard](https://github.com/facebook/zstd) (`libzstd-dev`)

## Building

```bash
# From your colcon workspace root
sudo apt install libzstd-dev              # if not already installed
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ros2_network_gateway
source install/setup.bash
```

## Usage

### Running the gateway node

```bash
ros2 run ros2_network_gateway network_gateway
```

Or with parameter overrides:

```bash
ros2 run ros2_network_gateway network_gateway --ros-args \
  -p topics:="[/battery_voltage, /imu/data]" \
  -p remote_address:="192.168.1.100" \
  -p send_port:=9000 \
  -p compression_level:=3
```

Pass an empty topic list to forward **all** discovered topics:

```bash
ros2 run ros2_network_gateway network_gateway --ros-args \
  -p topics:="[]"
```

### Running the test client

A Python logging client is included for quick debugging. It listens for UDP packets from the gateway and pretty-prints them:

```bash
python3 client/logging_client.py
```

If Zstd compression is enabled on the gateway side, install the Python binding to decompress automatically:

```bash
pip install zstandard
```

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `network_interface` | string | `"UDP"` | Transport backend (only UDP is implemented) |
| `topics` | string[] | `["/loki_1/chassis/battery_voltage", "/loki_1/chassis/clock"]` | Topic whitelist. Empty list = forward all topics |
| `topic_refresh_rate` | int | `5000` | How often to re-scan the ROS2 graph for new/removed topics (ms) |
| `topic_publish_rate` | int | `500` | How often to send the latest message for each topic over UDP (ms) |
| `local_address` | string | `"10.0.0.230"` | Local IP to bind the receive socket to |
| `receive_port` | int | `8500` | Local UDP port for incoming packets |
| `remote_address` | string | `"10.0.20.219"` | Destination IP to send data to |
| `send_port` | int | `8502` | Destination UDP port |
| `compression_level` | int | `0` | Zstd compression level (0 = disabled, 1-22 = enabled, higher = smaller but slower) |

## Wire protocol

Every UDP packet has a binary header followed by the payload:

```
 0                   8     9                      N   N+1                    M   M+1
 ├───── timestamp ───┤ CF  ├──── topic name ──────┤ \0 ├──── message type ──┤ \0 ├── payload ──...
       (double)       (u8)       (UTF-8 string)             (UTF-8 string)
```

| Field | Size | Description |
|---|---|---|
| `timestamp` | 8 bytes | ROS clock time in seconds (`double`, little-endian) |
| `compressed` | 1 byte | `0x00` = payload is raw JSON, `0x01` = payload is Zstd-compressed JSON |
| `topic` | variable | Null-terminated UTF-8 topic name (e.g. `/imu/data`) |
| `type` | variable | Null-terminated UTF-8 message type (e.g. `sensor_msgs/msg/Imu`) |
| `payload` | variable | JSON string or Zstd-compressed JSON |

### Proto packet payload example

Sent every 5 seconds per topic. Describes the message structure so the client can interpret data packets without ROS2 type definitions:

```json
{
  "name": "Imu",
  "namespace": "sensor_msgs::msg",
  "fields": [
    {"name": "header",    "is_array": false, "type": "message", "subtype": {"name": "Header", "namespace": "std_msgs::msg", "fields": [...]}},
    {"name": "orientation", "is_array": false, "type": "message", "subtype": {...}},
    {"name": "angular_velocity_covariance", "is_array": true, "type": "float64", "subtype": "primitive"}
  ]
}
```

### Data packet payload example

```json
{
  "header": [{"stamp": [{"sec": [1234567890], "nanosec": [123456789]}], "frame_id": ["base_link"]}],
  "angular_velocity": [{"x": [0.01], "y": [0.02], "z": [0.0]}]
}
```

## Project structure

```
ros2_network_gateway/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/ros2_network_gateway/
│   ├── network_gateway.hpp              # Main gateway node
│   ├── subscription_manager.hpp         # Per-topic subscription & serialization
│   └── network_interfaces/
│       ├── network_interface_base.hpp   # Abstract transport interface
│       └── udp_interface.hpp            # UDP implementation (Boost.Asio)
├── src/
│   ├── network_gateway.cpp
│   ├── subscription_manager.cpp
│   └── network_interfaces/
│       └── udp_interface.cpp
└── client/
    └── logging_client.py                # Python test client
```

### Key classes

- **`NetworkGateway`** — Main ROS2 node. Handles parameter loading, topic discovery, timer management, and packet assembly.
- **`SubscriptionManager`** — Created per topic. Uses generic subscriptions and runtime introspection to deserialize any ROS2 message type into JSON without compile-time dependencies on message packages.
- **`NetworkInterface`** — Abstract base class for pluggable network transports.
- **`UdpInterface`** — Concrete UDP transport using Boost.Asio with async receive and synchronous send.

## Current limitations

- **Unidirectional** — Only ROS2-to-network is implemented. The receive path (`receiveData`) is stubbed out for future work.
- **UDP only** — The `NetworkInterface` base class is designed for extensibility (TCP, WebSocket, etc.), but only UDP is implemented.
- **Whitelist only** — Topic filtering supports whitelisting by exact name. Regex and blacklist patterns are not yet implemented.
- **No batching** — Each topic is sent as a separate UDP packet. High topic counts may benefit from batching multiple messages into one packet.

## License

MIT
