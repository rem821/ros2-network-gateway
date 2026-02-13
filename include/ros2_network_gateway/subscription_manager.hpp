/**
 * @file subscription_manager.hpp
 * @brief Per-topic handler: generic subscription, introspection-based deserialization, and JSON conversion.
 *
 * SubscriptionManager subscribes to a single ROS2 topic using a generic (type-erased)
 * subscription. When a message arrives it is deserialized at runtime via the
 * rosidl_typesupport_introspection_cpp library and recursively converted to a
 * nlohmann::json object. The resulting JSON string (optionally Zstd-compressed) is
 * stored for later retrieval by NetworkGateway::sendData().
 *
 * On the first received message the type's structure is captured as a "proto" —
 * a JSON schema describing field names, types, and nesting — so that the remote
 * client can interpret data packets without having access to ROS2 message definitions.
 */
#pragma once
#include <mutex>
#include <span>
#include <string>
#include <nlohmann/json.hpp>
#include <rclcpp/node.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rclcpp/typesupport_helpers.hpp>

class SubscriptionManager {
public:
    /**
     * @param node             Parent ROS2 node (used for subscription creation and logging).
     * @param topicName        Fully-qualified topic name (e.g. "/imu/data").
     * @param topicType        ROS2 type string (e.g. "sensor_msgs/msg/Imu").
     * @param compressionLevel Zstd compression level for the JSON payload (0 = disabled).
     */
    SubscriptionManager(rclcpp::Node::SharedPtr node, std::string topicName,
                        std::string topicType, uint8_t compressionLevel);

    /** @brief Re-create the subscription if it was dropped. Called before every send. */
    void checkSubscription();

    /** @brief Thread-safe check whether at least one message has been received. */
    [[nodiscard]] bool hasData() const {
        std::lock_guard lock(dataMutex_);
        return hasData_;
    }

    /**
     * @brief Return the latest serialized payload (JSON or compressed bytes).
     * @param wipe If true, clear the internal buffer and reset hasData after copying.
     */
    std::string getData(bool wipe) {
        std::lock_guard lock(dataMutex_);
        std::string data = data_;
        if (wipe) {
            data_.clear();
            hasData_ = false;
        }
        return data;
    }

    /** @brief Return the JSON schema of the message type (populated after first message). */
    [[nodiscard]] std::string getMessageProto() {
        std::lock_guard lock(dataMutex_);
        return messageProto_;
    }

    [[nodiscard]] std::string getTopicName() const { return topicName_; }
    [[nodiscard]] std::string getTopicType() const { return topicType_; }

    /** @brief Whether the current payload in data_ is Zstd-compressed. */
    [[nodiscard]] bool isCompressed() const { return compressed_; }

private:
    /** @brief Create a generic (type-erased) subscription for topicName_/topicType_. */
    void subscribe();

    /**
     * @brief Subscription callback: deserialize a raw ROS2 message and convert it to JSON.
     *
     * Steps:
     *  1. Load introspection typesupport for the message type at runtime.
     *  2. On first call, build and cache the message proto (JSON schema).
     *  3. Allocate untyped memory, deserialize the CDR-encoded message into it.
     *  4. Recursively walk the introspection metadata to build a JSON object.
     *  5. Optionally compress the JSON string with Zstd.
     *  6. Store the result in data_ (thread-safe).
     */
    void handleMessage(const rclcpp::Node::SharedPtr &node, const std::string &topic, const std::string &type,
                       const std::shared_ptr<const rclcpp::SerializedMessage> &serializedMsg);

    /**
     * @brief Recursively build a JSON schema describing the message structure.
     *
     * Output example:
     * @code
     * {"name": "Imu", "namespace": "sensor_msgs::msg", "fields": [
     *   {"name": "header", "is_array": false, "type": "message", "subtype": {...}},
     *   {"name": "angular_velocity_covariance", "is_array": true, "type": "float64", "subtype": "primitive"}
     * ]}
     * @endcode
     */
    nlohmann::json describeMessageType(const rosidl_typesupport_introspection_cpp::MessageMembers *members);

    /** @brief Map a rosidl type ID constant to a human-readable string ("float64", "string", ...). */
    static std::string rosTypeToString(uint8_t typeId);

    /**
     * @brief Convert a single message field (scalar or array) to a JSON array.
     *
     * For arrays, iterates via the introspection size/get_const function pointers.
     * For nested messages, recurses into messageToJson().
     */
    nlohmann::json memberToJson(const void *field, const rosidl_typesupport_introspection_cpp::MessageMember &member);

    /**
     * @brief Convert a complete deserialized message to a JSON object.
     *
     * Walks every member at its byte offset within the untyped message buffer
     * and delegates to memberToJson() for each.
     */
    nlohmann::json messageToJson(const void *msg, const rosidl_typesupport_introspection_cpp::MessageMembers *members);

    /**
     * @brief Cast a single primitive value from an untyped pointer to a JSON value.
     *
     * Handles all 16 ROS2 primitive types (bool through wstring).
     */
    static nlohmann::json rosTypeToJson(const rosidl_typesupport_introspection_cpp::MessageMember *member,
                                        const void *elem);

    /**
     * @brief Compress raw bytes using Zstd at the configured compression level.
     * @throws std::runtime_error on Zstd error.
     */
    [[nodiscard]] std::vector<uint8_t> compress(std::vector<uint8_t> const &data) const;

    /**
     * @brief Decompress a Zstd-compressed buffer.
     * @throws std::runtime_error if the data was not compressed by Zstd or on decompression error.
     */
    static std::vector<uint8_t> decompress(std::span<const uint8_t> compressedData);

    rclcpp::Node::SharedPtr node_;
    std::string topicName_;
    std::string topicType_;
    uint8_t compressionLevel_;                          ///< 0 = no compression, 1-22 = Zstd level.

    rclcpp::GenericSubscription::SharedPtr subscriber_;

    mutable std::mutex dataMutex_;                      ///< Guards messageProto_, hasData_, compressed_, data_.
    std::string messageProto_;                          ///< Cached JSON schema (populated on first message).
    bool hasData_ = false;                              ///< True after the first message has been received.
    bool compressed_ = false;                           ///< Whether data_ currently holds compressed bytes.
    std::string data_;                                  ///< Latest payload: raw JSON string or Zstd-compressed bytes.
};
