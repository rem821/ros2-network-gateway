/**
 * @file subscription_manager.cpp
 * @brief Implementation of SubscriptionManager — runtime introspection and JSON serialization.
 *
 * The core idea: subscribe to any ROS2 topic without knowing its type at compile time.
 * When a message arrives we:
 *  1. Load its typesupport library dynamically.
 *  2. Allocate a raw byte buffer of the right size and deserialize the CDR data into it.
 *  3. Walk the introspection metadata (field names, offsets, type IDs) to build a JSON
 *     representation recursively.
 *
 * This avoids linking against every possible message package.
 */
#include <utility>
#include <zstd.h>
#include "ros2_network_gateway/subscription_manager.hpp"

using json = nlohmann::json;


SubscriptionManager::SubscriptionManager(rclcpp::Node::SharedPtr node, std::string topicName,
                                         std::string topicType,
                                         const uint8_t compressionLevel) : node_(std::move(node)),
                                                                           topicName_(std::move(topicName)),
                                                                           topicType_(std::move(topicType)),
                                                                           compressionLevel_(compressionLevel) {
    subscribe();
}

void SubscriptionManager::subscribe() {
    RCLCPP_INFO(node_->get_logger(), "Subscribing to topic %s", topicName_.c_str());
    // Generic subscription: works with any message type — receives raw serialized bytes.
    subscriber_ = node_->create_generic_subscription(
        topicName_, topicType_, rclcpp::QoS(1),
        [this, node = node_, topicName = topicName_, topicType = topicType_](
    const std::shared_ptr<const rclcpp::SerializedMessage> &serializedMsg) {
            this->handleMessage(node, topicName, topicType, serializedMsg);
        });
}


void SubscriptionManager::handleMessage(
    const rclcpp::Node::SharedPtr &node,
    const std::string &topic,
    const std::string &type,
    const std::shared_ptr<const rclcpp::SerializedMessage> &serializedMsg) {
    // 1. Load the introspection typesupport for this message type at runtime.
    //    This gives us field names, types, offsets — everything needed to interpret raw bytes.
    const auto lib = rclcpp::get_typesupport_library(type, "rosidl_typesupport_introspection_cpp");
    const rosidl_message_type_support_t *ts = rclcpp::get_message_typesupport_handle(
        type, "rosidl_typesupport_introspection_cpp", *lib);

    if (!ts) {
        RCLCPP_ERROR(node->get_logger(), "No typesupport for type %s", type.c_str());
        return;
    }

    const auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts->data);

    // 2. On first message, build and cache the JSON schema ("proto") so the remote
    //    client can interpret subsequent data packets without ROS2 type definitions.
    {
        std::lock_guard lock(dataMutex_);
        if (messageProto_.empty()) {
            messageProto_ = describeMessageType(members).dump();
            RCLCPP_DEBUG(node_->get_logger(), "Message proto: %s", messageProto_.c_str());
        }
    }

    // 3. Allocate a byte buffer matching the message's in-memory size and initialize it
    //    with default values (zeros / empty strings / etc.).
    std::vector<uint8_t> msgBuffer(members->size_of_);
    void *untyped_msg = msgBuffer.data();
    members->init_function(untyped_msg, rosidl_runtime_cpp::MessageInitialization::ALL);

    // 4. Deserialize the CDR-encoded bytes from the middleware into our untyped buffer.
    const rclcpp::SerializedMessage &tmp(*serializedMsg);
    const auto ret = rmw_deserialize(&tmp.get_rcl_serialized_message(), ts, untyped_msg);
    if (ret != RMW_RET_OK) {
        RCLCPP_ERROR(node_->get_logger(), "Deserialization failed for topic %s", topic.c_str());
        members->fini_function(untyped_msg);
        return;
    }

    // 5. Recursively walk the introspection metadata to convert the buffer to JSON.
    const nlohmann::json j = messageToJson(untyped_msg, members);
    try {
        std::string jsonStr = j.dump();
        RCLCPP_DEBUG(node->get_logger(), "Topic %s (%s): %s", topic.c_str(), type.c_str(), jsonStr.c_str());

        // 6. Optionally compress before storing.
        if (compressionLevel_ > 0) {
            std::vector<uint8_t> raw(jsonStr.begin(), jsonStr.end());
            auto compressed = compress(raw);
            std::lock_guard lock(dataMutex_);
            data_ = std::string(compressed.begin(), compressed.end());
            compressed_ = true;
            hasData_ = true;
        } else {
            std::lock_guard lock(dataMutex_);
            data_ = std::move(jsonStr);
            compressed_ = false;
            hasData_ = true;
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to serialize/compress message: %s", e.what());
    }

    // 7. Finalize (release dynamic strings, sequences, etc. allocated inside the buffer).
    members->fini_function(untyped_msg);
}

nlohmann::json SubscriptionManager::describeMessageType(
    const rosidl_typesupport_introspection_cpp::MessageMembers *members) {
    nlohmann::json j;
    j["name"] = members->message_name_;
    j["namespace"] = members->message_namespace_;
    j["fields"] = nlohmann::json::array();

    for (uint32_t i = 0; i < members->member_count_; ++i) {
        const auto &m = members->members_[i];
        nlohmann::json f;
        f["name"] = m.name_;
        f["is_array"] = m.is_array_;
        f["type"] = rosTypeToString(m.type_id_);

        if (m.type_id_ != rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
            f["subtype"] = "primitive";
        } else {
            // Nested message type — recurse to describe its structure.
            const auto subMembers =
                    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(m.members_->data);
            f["subtype"] = describeMessageType(subMembers);
        }

        j["fields"].push_back(f);
    }

    return j;
}

std::string SubscriptionManager::rosTypeToString(const uint8_t typeId) {
    using namespace rosidl_typesupport_introspection_cpp;
    switch (typeId) {
        case ROS_TYPE_BOOL: return "bool";
        case ROS_TYPE_BYTE: return "byte";
        case ROS_TYPE_CHAR: return "char";
        case ROS_TYPE_FLOAT32: return "float32";
        case ROS_TYPE_FLOAT64: return "float64";
        case ROS_TYPE_INT8: return "int8";
        case ROS_TYPE_UINT8: return "uint8";
        case ROS_TYPE_INT16: return "int16";
        case ROS_TYPE_UINT16: return "uint16";
        case ROS_TYPE_INT32: return "int32";
        case ROS_TYPE_UINT32: return "uint32";
        case ROS_TYPE_INT64: return "int64";
        case ROS_TYPE_UINT64: return "uint64";
        case ROS_TYPE_STRING: return "string";
        case ROS_TYPE_WSTRING: return "wstring";
        case ROS_TYPE_MESSAGE: return "message";
        default: return "unknown";
    }
}

nlohmann::json SubscriptionManager::memberToJson(const void *field,
                                                 const rosidl_typesupport_introspection_cpp::MessageMember &member) {
    nlohmann::json j = json::array();
    if (member.is_array_) {
        // Dynamic or fixed-size array — use the introspection function pointers.
        size_t size = member.size_function(field);
        for (size_t i = 0; i < size; i++) {
            const void *elem = member.get_const_function(field, i);
            if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
                auto subMembers = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member.
                    members_->data);
                j.push_back(messageToJson(elem, subMembers));
            } else {
                j.push_back(rosTypeToJson(&member, elem));
            }
        }
    } else {
        // Scalar field — wrap in a single-element array for uniform output format.
        if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
            auto subMembers = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member.members_
                ->
                data);
            j.push_back(messageToJson(field, subMembers));
        } else {
            j.push_back(rosTypeToJson(&member, field));
        }
    }
    return j;
}

nlohmann::json SubscriptionManager::messageToJson(const void *msg,
                                                  const rosidl_typesupport_introspection_cpp::MessageMembers *members) {
    nlohmann::json j = json::object();
    for (uint32_t i = 0; i < members->member_count_; i++) {
        const auto &member = members->members_[i];
        // Each member's data lives at a known byte offset within the message buffer.
        const void *field = static_cast<const uint8_t *>(msg) + member.offset_;
        const nlohmann::json fieldJson = memberToJson(field, member);
        j[member.name_] = fieldJson;
    }

    return j;
}

nlohmann::json SubscriptionManager::rosTypeToJson(const rosidl_typesupport_introspection_cpp::MessageMember *member,
                                                  const void *elem) {
    nlohmann::json j = json::object();
    // Cast the raw pointer to the concrete C++ type based on the rosidl type ID.
    switch (member->type_id_) {
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
            j = *static_cast<const float *>(elem);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
            j = *static_cast<const double *>(elem);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
            j = *static_cast<const long double *>(elem);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
            j = *static_cast<const unsigned char *>(elem);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
            j = *static_cast<const char16_t *>(elem);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
            j = *static_cast<const bool *>(elem);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
            j = *static_cast<const std::byte *>(elem);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
            j = *static_cast<const uint8_t *>(elem);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
            j = *static_cast<const int8_t *>(elem);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
            j = *static_cast<const uint16_t *>(elem);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
            j = *static_cast<const int16_t *>(elem);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
            j = *static_cast<const uint32_t *>(elem);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
            j = *static_cast<const int32_t *>(elem);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
            j = *static_cast<const uint64_t *>(elem);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
            j = *static_cast<const int64_t *>(elem);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
            j = *static_cast<const std::string *>(elem);
            break;
        case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
            j = *static_cast<const std::u16string *>(elem);
            break;
        default:
            j.push_back("<unhandled>");
    }

    return j;
}

std::vector<uint8_t> SubscriptionManager::compress(std::vector<uint8_t> const &data) const {
    std::vector<uint8_t> compressedData;
    const size_t compressedCapacity = ZSTD_compressBound(data.size());

    compressedData.resize(compressedCapacity);

    const size_t compressedSize = ZSTD_compress(
        compressedData.data(), compressedCapacity, data.data(), data.size(),
        compressionLevel_);

    if (ZSTD_isError(compressedSize)) {
        throw std::runtime_error(ZSTD_getErrorName(compressedSize));
    }

    compressedData.resize(compressedSize);
    return compressedData;
}

std::vector<uint8_t> SubscriptionManager::decompress(std::span<const uint8_t> compressedData) {
    std::vector<uint8_t> data;
    const size_t decompressedSize = ZSTD_getFrameContentSize(
        compressedData.data(), compressedData.size());

    if (decompressedSize == ZSTD_CONTENTSIZE_ERROR) {
        throw std::runtime_error("Not compressed by Zstd");
    } else if (decompressedSize == ZSTD_CONTENTSIZE_UNKNOWN) {
        throw std::runtime_error("Original size unknown");
    }

    data.resize(decompressedSize);

    const size_t decompressedResult = ZSTD_decompress(
        data.data(), decompressedSize, compressedData.data(),
        compressedData.size());

    if (ZSTD_isError(decompressedResult)) {
        throw std::runtime_error(ZSTD_getErrorName(decompressedResult));
    }

    return data;
}

void SubscriptionManager::checkSubscription() {
    if (!subscriber_) {
        subscribe();
    }
}
