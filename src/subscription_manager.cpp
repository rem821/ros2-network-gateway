//
// Created by standa on 10/2/25.
//

#include <utility>
#include <zstd.h>
#include "ros2_network_gateway/subscription_manager.hpp"

using json = nlohmann::json;


SubscriptionManager::SubscriptionManager(rclcpp::Node::SharedPtr node, std::string  topicName,
                                         std::string topicType, const uint8_t compressionLevel) : node_(std::move(node)),
    topicName_(std::move(topicName)),
    topicType_(std::move(topicType)), compressionLevel_(compressionLevel)
{
    subscribe();
}

void SubscriptionManager::subscribe()
{
    if (topicName_ == "/loki_1/debug/teb/feedback") return;
    if (topicName_ == "/map_diff_vect") return;
    if (topicName_ == "/loki_1/teb_obstacles") return;
    if (topicName_ == "/shared_plans") return;

    RCLCPP_INFO(node_->get_logger(), "Subscribing to topic %s", topicName_.c_str());
    // try
    // {
        subscriber_ = node_->create_generic_subscription(
            topicName_, topicType_, rclcpp::QoS(1),
            [this, node = node_, topicName = topicName_, topicType = topicType_](
            const std::shared_ptr<const rclcpp::SerializedMessage>& serializedMsg)
            {
                this->handleMessage(node, topicName, topicType, serializedMsg);
            });
    // }
    // catch (...)
    // {
    //     RCLCPP_ERROR(node_->get_logger(), "Failed to subscribe to topic %s of type %s", topicName_.c_str(),
    //                  topicType_.c_str());
    // }
}


void SubscriptionManager::handleMessage(
    const rclcpp::Node::SharedPtr& node,
    const std::string& topic,
    const std::string& type,
    const std::shared_ptr<const rclcpp::SerializedMessage>& serializedMsg)
{
    // 1. Lookup type support for this message type
    const auto lib = rclcpp::get_typesupport_library(type, "rosidl_typesupport_introspection_cpp");
    const rosidl_message_type_support_t* ts = rclcpp::get_typesupport_handle(type, "rosidl_typesupport_introspection_cpp", *lib);

    if (!ts)
    {
        RCLCPP_ERROR(node->get_logger(), "No typesupport for type %s", type.c_str());
        return;
    }

    const auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(ts->data);

    // 2. Allocate untyped message memory
    void* untyped_msg = malloc(members->size_of_);
    members->init_function(untyped_msg, rosidl_runtime_cpp::MessageInitialization::ALL);

    // 3. Deserialize into it
    const rclcpp::SerializedMessage& tmp(*serializedMsg);
    rmw_deserialize(&tmp.get_rcl_serialized_message(), ts, untyped_msg);


    // 4. Compress the stream
    // try {
    //     data_ = compress();
    // } catch (const std::exception & e) {
    //     RCLCPP_ERROR(
    //       this->get_logger(),
    //       "Compression Failed: %s", e.what());
    //     return;
    // }

    // 4. Convert to JSON recursively
    const nlohmann::json j = messageToJson(untyped_msg, members);
    try {
        data_ = j.dump();
        hasData_ = true;
    } catch (...) {
        RCLCPP_ERROR(node->get_logger(), "Couldnt convert the stream to JSON");
    }

    // 5. Print JSON
    //RCLCPP_INFO(node->get_logger(), "Topic %s (%s): %s", topic.c_str(), type.c_str(), data_.c_str());
    // 6. Cleanup
    members->fini_function(untyped_msg);
}

nlohmann::json SubscriptionManager::memberToJson(const void* field,
                                                 const rosidl_typesupport_introspection_cpp::MessageMember& member)
{
    nlohmann::json j = json::array();
    if (member.is_array_)
    {
        size_t size = member.size_function(field);
        for (size_t i = 0; i < size; i++)
        {
            const void* elem = member.get_const_function(field, i);
            if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE)
            {
                auto subMembers = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(member.
                    members_->data);
                j.push_back(messageToJson(elem, subMembers));
            }
            else
            {
                // Handle primitive array element
                j.push_back(rosTypeToJson(&member, elem));
            }
        }
    }
    else
    {
        if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE)
        {
            auto subMembers = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(member.members_->
                data);
            j.push_back(messageToJson(field, subMembers));
        }
        else
        {
            j.push_back(rosTypeToJson(&member, field));
        }
    }
    return j;
}

nlohmann::json SubscriptionManager::messageToJson(const void* msg,
                                                  const rosidl_typesupport_introspection_cpp::MessageMembers* members)
{
    nlohmann::json j = json::object();
    for (uint32_t i = 0; i < members->member_count_; i++)
    {
        const auto& member = members->members_[i];
        const void* field = static_cast<const uint8_t*>(msg) + member.offset_;
        const nlohmann::json fieldJson = memberToJson(field, member);
        j[member.name_] = fieldJson;
    }

    return j;
}

nlohmann::json SubscriptionManager::rosTypeToJson(const rosidl_typesupport_introspection_cpp::MessageMember* member,
                                                  const void* elem)
{
    nlohmann::json j = json::object();
    switch (member->type_id_)
    {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
        j = *static_cast<const float*>(elem);
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
        j = *static_cast<const double*>(elem);
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
        j = *static_cast<const long double*>(elem);
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
        j = *static_cast<const unsigned char*>(elem);
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WCHAR:
        j = *static_cast<const char16_t*>(elem);
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
        j = *static_cast<const bool*>(elem);
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
        j = *static_cast<const std::byte*>(elem);
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
        j = *static_cast<const uint8_t*>(elem);
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
        j = *static_cast<const int8_t*>(elem);
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
        j = *static_cast<const uint16_t*>(elem);
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
        j = *static_cast<const int16_t*>(elem);
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
        j = *static_cast<const uint32_t*>(elem);
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
        j = *static_cast<const int32_t*>(elem);
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
        j = *static_cast<const uint64_t*>(elem);
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
        j = *static_cast<const int64_t*>(elem);
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
        j = *static_cast<const std::string*>(elem);
        break;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
        j = *static_cast<const std::u16string*>(elem);
        break;
    default:
        j.push_back("<unhandled>");
    }

    return j;
}

std::vector<uint8_t> SubscriptionManager::compress(std::vector<uint8_t> const & data) const
{
    std::vector<uint8_t> compressedData;
    const size_t compressedCapacity = ZSTD_compressBound(data.size());

    // Resize the output buffer to the capacity needed
    compressedData.resize(compressedCapacity);

    // Compress the data
    const size_t compressedSize = ZSTD_compress(
      compressedData.data(), compressedCapacity, data.data(), data.size(),
      compressionLevel_);

    // Check for errors
    if (ZSTD_isError(compressedSize)) {
        throw std::runtime_error(ZSTD_getErrorName(compressedSize));
    }

    // Resize compressed_data to actual compressed size
    compressedData.resize(compressedSize);
    return compressedData;
}

std::vector<uint8_t> SubscriptionManager::decompress(std::span<const uint8_t> compressedData)
{
    std::vector<uint8_t> data;
    // Find the size of the original uncompressed data
    const size_t decompressedSize = ZSTD_getFrameContentSize(
      compressedData.data(), compressedData.size());

    // Check if the size is known and valid
    if (decompressedSize == ZSTD_CONTENTSIZE_ERROR) {
        throw std::runtime_error("Not compressed by Zstd");
    } else if (decompressedSize == ZSTD_CONTENTSIZE_UNKNOWN) {
        throw std::runtime_error("Original size unknown");
    }

    // Resize the output buffer to the size of the uncompressed data
    data.resize(decompressedSize);

    // Decompress the data
    const size_t decompressedResult = ZSTD_decompress(
      data.data(), decompressedSize, compressedData.data(),
      compressedData.size());

    // Check for errors during decompression
    if (ZSTD_isError(decompressedResult)) {
        throw std::runtime_error(ZSTD_getErrorName(decompressedResult));
    }

    return data;
}

void SubscriptionManager::checkSubscription()
{
    if (!subscriber_)
    {
        subscribe();
    }
}
